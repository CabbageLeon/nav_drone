/**
 * controller_node — position setpoint manager for PX4 offboard control
 *
 * Single mode: position setpoint forwarding.
 * PX4 handles trajectory execution internally — acceleration, cruise, deceleration, hover.
 *
 * Topics:
 *   Sub: /uav/goal              (ControlCommand)  target position from user/planner
 *   Sub: /uav/odometry           (Odometry)        current position
 *   Pub: /uav/control_command    (ControlCommand)  setpoint → px4_bridge
 *   Pub: /uav/controller/state   (ControllerState) monitoring
 *
 * Services:
 *   /uav/set_control_mode (SetControlMode)
 */

#include "rclcpp/rclcpp.hpp"
#include "uav_msgs/msg/control_command.hpp"
#include "uav_msgs/msg/controller_state.hpp"
#include "uav_msgs/srv/set_control_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "uav_controller/setpoint_manager.hpp"

#include <memory>

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    // —— Parameters (all from YAML, sentinels detect missing config) ——
    declare_parameter("control_rate", -1.0);
    declare_parameter("goal_tolerance_xy", -1.0);
    declare_parameter("goal_tolerance_z", -1.0);
    declare_parameter("auto_hold", false);

    if (!load_and_validate())
    {
      RCLCPP_ERROR(get_logger(),
                   "controller_node: required parameters missing. "
                   "Pass controller_params.yaml in launch file.");
      rclcpp::shutdown();
      return;
    }

    manager_ = std::make_unique<uav_controller::SetpointManager>(manager_params_);

    // —— Publishers ——
    cmd_pub_ = create_publisher<uav_msgs::msg::ControlCommand>(
        "/uav/control_command", 10);
    state_pub_ = create_publisher<uav_msgs::msg::ControllerState>(
        "/uav/controller/state", 10);

    // —— Subscriptions ——
    goal_sub_ = create_subscription<uav_msgs::msg::ControlCommand>(
        "/uav/goal", 10,
        std::bind(&ControllerNode::goal_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/uav/odometry", 10,
        std::bind(&ControllerNode::odom_callback, this, std::placeholders::_1));

    // —— Service ——
    mode_srv_ = create_service<uav_msgs::srv::SetControlMode>(
        "/uav/set_control_mode",
        std::bind(&ControllerNode::mode_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // —— Control loop ——
    double rate = get_parameter("control_rate").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    loop_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ControllerNode::loop, this));

    RCLCPP_INFO(get_logger(),
                "controller_node started (rate=%.1f Hz, auto_hold=%s)",
                rate, auto_hold_ ? "true" : "false");
  }

private:
  uav_controller::SetpointManager::Params manager_params_;
  bool auto_hold_{true};
  std::unique_ptr<uav_controller::SetpointManager> manager_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  rclcpp::Publisher<uav_msgs::msg::ControlCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<uav_msgs::msg::ControllerState>::SharedPtr state_pub_;
  rclcpp::Subscription<uav_msgs::msg::ControlCommand>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<uav_msgs::srv::SetControlMode>::SharedPtr mode_srv_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  double last_goal_x_{NAN}, last_goal_y_{NAN}, last_goal_z_{NAN};

  bool load_and_validate()
  {
    auto fail = [this](const char *name) {
      RCLCPP_ERROR(get_logger(), "Parameter '%s' not set in YAML config", name);
      return false;
    };

    double rate = get_parameter("control_rate").as_double();
    if (rate <= 0.0) return fail("control_rate");

    double tol_xy = get_parameter("goal_tolerance_xy").as_double();
    if (tol_xy < 0.0) return fail("goal_tolerance_xy");
    manager_params_.goal_tolerance_xy = tol_xy;

    double tol_z = get_parameter("goal_tolerance_z").as_double();
    if (tol_z < 0.0) return fail("goal_tolerance_z");
    manager_params_.goal_tolerance_z = tol_z;

    auto_hold_ = get_parameter("auto_hold").as_bool();
    return true;
  }

  void goal_callback(uav_msgs::msg::ControlCommand::SharedPtr msg)
  {
    double gx = msg->pose.pose.position.x;
    double gy = msg->pose.pose.position.y;
    double gz = msg->pose.pose.position.z;
    if (gx != last_goal_x_ || gy != last_goal_y_ || gz != last_goal_z_)
    {
      RCLCPP_INFO(get_logger(), "Goal: odom(%.2f, %.2f, %.2f)", gx, gy, gz);
      last_goal_x_ = gx; last_goal_y_ = gy; last_goal_z_ = gz;
    }
    manager_->setGoal(*msg);
  }

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; }

  void mode_callback(
      const std::shared_ptr<uav_msgs::srv::SetControlMode::Request> req,
      std::shared_ptr<uav_msgs::srv::SetControlMode::Response> res)
  {
    res->success = (req->control_mode == uav_msgs::msg::ControlCommand::MODE_POSITION);
    res->message = res->success ? "Mode: POSITION" : "Unknown mode";
  }

  void loop()
  {
    if (!latest_odom_)
    {
      uav_msgs::msg::ControlCommand stop;
      stop.control_mode = uav_msgs::msg::ControlCommand::MODE_POSITION;
      cmd_pub_->publish(stop);
      return;
    }

    nav_msgs::msg::Odometry odom_ref = *latest_odom_;
    uav_msgs::msg::ControlCommand cmd = manager_->update(odom_ref);

    auto now = get_clock()->now();
    cmd.pose.header.stamp = now;
    cmd.pose.header.frame_id = "odom";
    cmd_pub_->publish(cmd);

    // Monitoring
    auto state = uav_msgs::msg::ControllerState();
    state.header.stamp = now;
    state.active_mode = uav_msgs::msg::ControlCommand::MODE_POSITION;
    state.current_pose.pose = odom_ref.pose.pose;
    state.position_error_x = cmd.pose.pose.position.x - odom_ref.pose.pose.position.x;
    state.position_error_y = cmd.pose.pose.position.y - odom_ref.pose.pose.position.y;
    state.position_error_z = cmd.pose.pose.position.z - odom_ref.pose.pose.position.z;
    state_pub_->publish(state);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
