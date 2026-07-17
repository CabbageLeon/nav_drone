/**
 * px4_bridge_node — PX4 ↔ ROS2 boundary adapter (MAVROS version)
 *
 * Replaces uXRCE-DDS with MAVROS for PX4 v1.13 compatibility:
 *   - fmu/* topics → /mavros/* topics + services
 *   - Same upper interface: /uav/control_command in, /uav/bridge/* out
 *
 * Frame: MAVROS publishes NED, bridge converts to odom frame (ned or enu).
 * Setpoint: publishes to /mavros/setpoint_position/local (NED).
 */

#include "rclcpp/rclcpp.hpp"
#include "uav_msgs/msg/control_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "uav_px4_bridge/frame_transforms.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

class Px4BridgeNode : public rclcpp::Node
{
  enum class Phase { SETUP, OFFBOARD, ARM, CONTROL };

public:
  Px4BridgeNode() : Node("px4_bridge_node"), phase_(Phase::SETUP)
  {
    // —— Parameters (all from YAML) ——
    declare_parameter("enable_auto_sequence", false);
    declare_parameter("odom_frame_convention", "");
    declare_parameter("enable_odometry_forwarding", false);
    declare_parameter("odometry_forwarding_topic", "");

    if (get_parameter("odom_frame_convention").as_string().empty())
    {
      RCLCPP_ERROR(get_logger(), "'odom_frame_convention' not set in YAML.");
      rclcpp::shutdown(); return;
    }
    std::string odom_conv = get_parameter("odom_frame_convention").as_string();
    if (odom_conv != "ned" && odom_conv != "enu")
    {
      RCLCPP_ERROR(get_logger(), "'odom_frame_convention' must be 'ned' or 'enu'.");
      rclcpp::shutdown(); return;
    }

    // —— MAVROS publishers (use BEST_EFFORT — MAVROS plugins use this) ——
    setpoint_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", rclcpp::SensorDataQoS());

    // —— MAVROS subscriptions (use SensorDataQoS — MAVROS uses BEST_EFFORT) ——
    auto mavros_qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", mavros_qos,
        std::bind(&Px4BridgeNode::state_cb, this, std::placeholders::_1));
    local_pos_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", mavros_qos,
        std::bind(&Px4BridgeNode::local_pos_cb, this, std::placeholders::_1));

    // —— MAVROS service clients ——
    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // —— ROS2 publishers ——
    bridge_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "/uav/bridge/px4_odometry", 10);
    bridge_status_pub_ = create_publisher<std_msgs::msg::String>(
        "/uav/bridge/flight_status", 10);

    // —— ROS2 subscription ——
    cmd_sub_ = create_subscription<uav_msgs::msg::ControlCommand>(
        "/uav/control_command", 10,
        std::bind(&Px4BridgeNode::cmd_cb, this, std::placeholders::_1));

    // —— Main loop: 20 Hz ——
    loop_timer_ = create_wall_timer(50ms, std::bind(&Px4BridgeNode::loop, this));

    RCLCPP_INFO(get_logger(), "px4_bridge_node (MAVROS) started, odom=%s", odom_conv.c_str());
  }

private:
  Phase phase_;
  rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};
  int retry_count_{0};

  // MAVROS state
  mavros_msgs::msg::State::SharedPtr state_;
  geometry_msgs::msg::PoseStamped::SharedPtr local_pos_;
  uav_msgs::msg::ControlCommand::SharedPtr cmd_;
  bool cmd_received_{false};

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr bridge_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bridge_status_pub_;

  // Subscriptions
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<uav_msgs::msg::ControlCommand>::SharedPtr cmd_sub_;

  // Service clients
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  // Frame helpers
  bool use_enu() const { return get_parameter("odom_frame_convention").as_string() == "enu"; }

  Eigen::Vector3d ned_to_odom(double x, double y, double z)
  { Eigen::Vector3d v(x, y, z); return use_enu() ? uav_px4_bridge::ned_to_enu(v) : v; }
  Eigen::Vector3d odom_to_ned(double x, double y, double z)
  { Eigen::Vector3d v(x, y, z); return use_enu() ? uav_px4_bridge::enu_to_ned(v) : v; }

  double elapsed() { return (get_clock()->now() - phase_start_).seconds(); }
  void set_phase(Phase p) { phase_ = p; phase_start_ = get_clock()->now(); retry_count_ = 0; }

  // Callbacks
  void state_cb(mavros_msgs::msg::State::SharedPtr m) { state_ = m; }
  void local_pos_cb(geometry_msgs::msg::PoseStamped::SharedPtr m) { local_pos_ = m; }
  void cmd_cb(uav_msgs::msg::ControlCommand::SharedPtr m) { cmd_ = m; cmd_received_ = true; }

  // Publish odometry (MAVROS already publishes ENU, no NED→ENU conversion needed)
  void publish_odometry()
  {
    if (!local_pos_) return;
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = local_pos_->pose.position.x;
    odom.pose.pose.position.y = local_pos_->pose.position.y;
    odom.pose.pose.position.z = local_pos_->pose.position.z;
    odom.pose.pose.orientation = local_pos_->pose.orientation;

    bridge_odom_pub_->publish(odom);
  }

  void publish_status()
  {
    if (!state_) return;
    auto msg = std_msgs::msg::String();
    msg.data = std::string("mode=") + state_->mode +
               " armed=" + (state_->armed ? "true" : "false") +
               " connected=" + (state_->connected ? "true" : "false");
    bridge_status_pub_->publish(msg);
  }

  void publish_setpoint_ned(double x, double y, double z)
  {
    auto sp = geometry_msgs::msg::PoseStamped();
    sp.header.stamp = get_clock()->now();
    sp.pose.position.x = x;
    sp.pose.position.y = y;
    sp.pose.position.z = z;
    sp.pose.orientation.w = 1.0;
    setpoint_pub_->publish(sp);
  }

  void request_set_mode(const std::string &mode)
  {
    if (!set_mode_client_->wait_for_service(500ms))
    { RCLCPP_WARN(get_logger(), "set_mode service not available"); return; }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    set_mode_client_->async_send_request(req);
  }

  void request_arm(bool arm)
  {
    if (!arming_client_->wait_for_service(500ms))
    { RCLCPP_WARN(get_logger(), "arming service not available"); return; }
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm;
    arming_client_->async_send_request(req);
  }

  // Main loop
  void loop()
  {
    publish_odometry();
    publish_status();

    // Default setpoint (hold current position). MAVROS data is already ENU.
    double sp_x = 0, sp_y = 0, sp_z = 0;
    if (local_pos_)
    {
      sp_x = local_pos_->pose.position.x;
      sp_y = local_pos_->pose.position.y;
      sp_z = local_pos_->pose.position.z;
    }

    if (cmd_received_ && cmd_)
    {
      // MAVROS expects ENU input on setpoint_position/local (converts to NED internally)
      sp_x = cmd_->pose.pose.position.x;
      sp_y = cmd_->pose.pose.position.y;
      sp_z = cmd_->pose.pose.position.z;
    }

    publish_setpoint_ned(sp_x, sp_y, sp_z);

    // Auto sequence (needs MAVROS connection)
    if (!get_parameter("enable_auto_sequence").as_bool()) return;
    if (!state_ || !state_->connected) return;

    double t = elapsed();

    switch (phase_)
    {
    case Phase::SETUP:
      if (t > 5.0)  // PX4 requires ~100 setpoints before offboard mode switch
      {
        RCLCPP_INFO(get_logger(), "→ OFFBOARD mode");
        request_set_mode("OFFBOARD");
        set_phase(Phase::OFFBOARD);
      }
      break;

    case Phase::OFFBOARD:
      if (state_->mode == "OFFBOARD")
      { RCLCPP_INFO(get_logger(), "Offboard OK → ARM"); set_phase(Phase::ARM); }
      else if (retry_count_ < 3 && t > 3.0)
      { retry_count_++; request_set_mode("OFFBOARD"); phase_start_ = get_clock()->now(); }
      break;

    case Phase::ARM:
      if (state_->armed)
      { RCLCPP_INFO(get_logger(), "Armed → CONTROL"); set_phase(Phase::CONTROL); }
      else if (retry_count_ < 3 && t > 2.0)
      { retry_count_++; request_arm(true); phase_start_ = get_clock()->now(); }
      break;

    case Phase::CONTROL:
      if (!state_->armed)
      { RCLCPP_WARN(get_logger(), "Disarmed unexpectedly"); }
      break;
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
