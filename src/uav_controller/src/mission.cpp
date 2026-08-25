/**
 * mission_node — take off, then forward SPF/VLM goals to /uav/goal.
 *
 * The visual-language inference runs in spf_node and publishes /uav/spf_goal.
 * This node keeps the existing controller/bridge interface unchanged for PX4
 * offboard setpoint streaming.
 */

#include "rclcpp/rclcpp.hpp"
#include "uav_msgs/msg/control_command.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <limits>
#include <string>

class MissionNode : public rclcpp::Node
{
public:
  MissionNode() : Node("mission_node")
  {
    declare_parameter("publish_rate_hz", -1.0);
    declare_parameter("takeoff_height_m", -1.0);
    declare_parameter("takeoff_tolerance_m", -1.0);
    declare_parameter("target_timeout_s", -1.0);
    declare_parameter("odom_topic", "");
    declare_parameter("spf_goal_topic", "");
    declare_parameter("goal_topic", "");

    if (!load_and_validate())
    {
      RCLCPP_ERROR(get_logger(),
                   "mission_node: required parameters missing. "
                   "Pass mission_params.yaml in launch file.");
      rclcpp::shutdown();
      return;
    }

    goal_pub_ = create_publisher<uav_msgs::msg::ControlCommand>(goal_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&MissionNode::odom_cb, this, std::placeholders::_1));
    spf_goal_sub_ = create_subscription<uav_msgs::msg::ControlCommand>(
        spf_goal_topic_, 10,
        std::bind(&MissionNode::spf_goal_cb, this, std::placeholders::_1));

    const double rate = get_parameter("publish_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MissionNode::loop, this));

    RCLCPP_INFO(get_logger(),
                "mission_node: takeoff %.1fm, then follow SPF goals from %s",
                takeoff_height_m_, spf_goal_topic_.c_str());
  }

private:
  enum class Phase { WAIT_ODOM, TAKEOFF, TRACK_SPF };

  Phase phase_{Phase::WAIT_ODOM};
  double takeoff_height_m_{0.0};
  double takeoff_tolerance_m_{0.0};
  double target_timeout_s_{0.0};
  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};
  double takeoff_z_{0.0};
  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_z_{0.0};
  bool hold_set_{false};
  bool home_set_{false};
  bool takeoff_announced_{false};
  bool tracking_announced_{false};
  bool stale_warned_{false};

  std::string odom_topic_;
  std::string spf_goal_topic_;
  std::string goal_topic_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  uav_msgs::msg::ControlCommand::SharedPtr latest_spf_goal_;
  rclcpp::Time latest_spf_goal_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<uav_msgs::msg::ControlCommand>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<uav_msgs::msg::ControlCommand>::SharedPtr spf_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool load_and_validate()
  {
    auto fail = [this](const char *name) {
      RCLCPP_ERROR(get_logger(), "Parameter '%s' not set in YAML config", name);
      return false;
    };

    const double rate = get_parameter("publish_rate_hz").as_double();
    if (rate < 10.0) return fail("publish_rate_hz");

    takeoff_height_m_ = get_parameter("takeoff_height_m").as_double();
    if (takeoff_height_m_ <= 0.0) return fail("takeoff_height_m");

    takeoff_tolerance_m_ = get_parameter("takeoff_tolerance_m").as_double();
    if (takeoff_tolerance_m_ <= 0.0) return fail("takeoff_tolerance_m");

    target_timeout_s_ = get_parameter("target_timeout_s").as_double();
    if (target_timeout_s_ <= 0.0) return fail("target_timeout_s");

    odom_topic_ = get_parameter("odom_topic").as_string();
    if (odom_topic_.empty()) return fail("odom_topic");

    spf_goal_topic_ = get_parameter("spf_goal_topic").as_string();
    if (spf_goal_topic_.empty()) return fail("spf_goal_topic");

    goal_topic_ = get_parameter("goal_topic").as_string();
    if (goal_topic_.empty()) return fail("goal_topic");

    return true;
  }

  void odom_cb(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
  }

  void spf_goal_cb(uav_msgs::msg::ControlCommand::SharedPtr msg)
  {
    const auto &p = msg->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite SPF goal");
      return;
    }

    latest_spf_goal_ = msg;
    latest_spf_goal_time_ = get_clock()->now();
    stale_warned_ = false;
    RCLCPP_INFO(get_logger(), "SPF goal: odom(%.2f, %.2f, %.2f)",
                p.x, p.y, p.z);
  }

  void publish_goal(double x, double y, double z)
  {
    uav_msgs::msg::ControlCommand cmd;
    cmd.control_mode = uav_msgs::msg::ControlCommand::MODE_POSITION;
    cmd.pose.header.stamp = get_clock()->now();
    cmd.pose.header.frame_id = "odom";
    cmd.pose.pose.position.x = x;
    cmd.pose.pose.position.y = y;
    cmd.pose.pose.position.z = z;
    cmd.pose.pose.orientation.w = 1.0;
    cmd.yaw = std::numeric_limits<float>::quiet_NaN();
    goal_pub_->publish(cmd);
  }

  void publish_goal(const uav_msgs::msg::ControlCommand &source)
  {
    auto cmd = source;
    cmd.control_mode = uav_msgs::msg::ControlCommand::MODE_POSITION;
    cmd.pose.header.stamp = get_clock()->now();
    cmd.pose.header.frame_id = "odom";
    cmd.yaw = std::numeric_limits<float>::quiet_NaN();
    goal_pub_->publish(cmd);
  }

  bool spf_goal_fresh()
  {
    if (!latest_spf_goal_) return false;
    return (get_clock()->now() - latest_spf_goal_time_).seconds() <= target_timeout_s_;
  }

  void loop()
  {
    if (!latest_odom_) return;

    const auto &pos = latest_odom_->pose.pose.position;

    if (!home_set_)
    {
      home_x_ = pos.x;
      home_y_ = pos.y;
      home_z_ = pos.z;
      takeoff_z_ = home_z_ + takeoff_height_m_;
      home_set_ = true;
      phase_ = Phase::TAKEOFF;
    }

    switch (phase_)
    {
    case Phase::WAIT_ODOM:
      break;

    case Phase::TAKEOFF:
      publish_goal(home_x_, home_y_, takeoff_z_);
      if (!takeoff_announced_)
      {
        RCLCPP_INFO(get_logger(), "TAKEOFF to odom z=%.2f", takeoff_z_);
        takeoff_announced_ = true;
      }
      if (std::abs(pos.z - takeoff_z_) <= takeoff_tolerance_m_)
      {
        hold_x_ = home_x_;
        hold_y_ = home_y_;
        hold_z_ = takeoff_z_;
        hold_set_ = true;
        phase_ = Phase::TRACK_SPF;
        RCLCPP_INFO(get_logger(), "TRACK_SPF; holding odom(%.2f, %.2f, %.2f)",
                    hold_x_, hold_y_, hold_z_);
      }
      break;

    case Phase::TRACK_SPF:
      if (!tracking_announced_)
      {
        RCLCPP_INFO(get_logger(), "Waiting for VLM/SPF goals on %s",
                    spf_goal_topic_.c_str());
        tracking_announced_ = true;
      }

      if (spf_goal_fresh())
      {
        const auto &p = latest_spf_goal_->pose.pose.position;
        hold_x_ = p.x;
        hold_y_ = p.y;
        hold_z_ = p.z;
        hold_set_ = true;
        publish_goal(*latest_spf_goal_);
        return;
      }

      if (latest_spf_goal_ && !stale_warned_)
      {
        RCLCPP_WARN(get_logger(), "SPF goal stale; holding last fixed goal");
        stale_warned_ = true;
      }
      if (!hold_set_)
      {
        hold_x_ = pos.x;
        hold_y_ = pos.y;
        hold_z_ = pos.z;
        hold_set_ = true;
      }
      publish_goal(hold_x_, hold_y_, hold_z_);
      break;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionNode>());
  rclcpp::shutdown();
  return 0;
}
