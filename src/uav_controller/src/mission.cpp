/**
 * mission — 起飞 0.5m → 前飞 1m → 降落
 * 通过 /uav/goal 发布位置目标，依赖 bridge 的 auto_sequence 完成 offboard + arm。
 */

#include "rclcpp/rclcpp.hpp"
#include "uav_msgs/msg/control_command.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class MissionNode : public rclcpp::Node
{
public:
  MissionNode() : Node("mission_node")
  {
    goal_pub_ = create_publisher<uav_msgs::msg::ControlCommand>("/uav/goal", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/uav/odometry", 10,
        std::bind(&MissionNode::odom_cb, this, std::placeholders::_1));
    timer_ = create_wall_timer(100ms, std::bind(&MissionNode::loop, this));
    phase_start_ = get_clock()->now();
    RCLCPP_INFO(get_logger(), "mission: takeoff %.1fm → fwd %.1fm → land", TAKEOFF_Z, FORWARD_X);
  }

private:
  static constexpr double TAKEOFF_Z = 0.5;   // 0.5m up
  static constexpr double FORWARD_X = 1.0;   // 1m forward

  enum Phase { WAIT_ARM, TAKEOFF, FORWARD, LAND, DONE };
  Phase phase_{WAIT_ARM};
  rclcpp::Time phase_start_;
  rclcpp::Time target_reached_;
  bool reached_stable_{false};
  double cur_x_{0}, cur_z_{0};

  rclcpp::Publisher<uav_msgs::msg::ControlCommand>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void odom_cb(nav_msgs::msg::Odometry::SharedPtr m)
  { cur_x_ = m->pose.pose.position.x; cur_z_ = m->pose.pose.position.z; }

  void send_goal(double x, double y, double z)
  {
    uav_msgs::msg::ControlCommand cmd;
    cmd.control_mode = uav_msgs::msg::ControlCommand::MODE_POSITION;
    cmd.pose.header.frame_id = "odom";
    cmd.pose.pose.position.x = x;
    cmd.pose.pose.position.y = y;
    cmd.pose.pose.position.z = z;
    cmd.yaw = NAN;
    goal_pub_->publish(cmd);
  }

  void send_land()
  {
    uav_msgs::msg::ControlCommand cmd;
    cmd.control_mode = uav_msgs::msg::ControlCommand::MODE_LAND;
    cmd.pose.header.frame_id = "odom";
    cmd.pose.pose.position.x = cur_x_;
    cmd.pose.pose.position.y = 0;
    cmd.pose.pose.position.z = cur_z_;
    cmd.yaw = NAN;
    goal_pub_->publish(cmd);
  }

  bool at_target(double target, double current, double tol)
  {
    if (std::abs(current - target) < tol)
    {
      if (!reached_stable_) { reached_stable_ = true; target_reached_ = get_clock()->now(); }
      return (get_clock()->now() - target_reached_).seconds() > 2.0;
    }
    reached_stable_ = false;
    return false;
  }

  void loop()
  {
    switch (phase_)
    {
    case WAIT_ARM:
      send_goal(0, 0, 0);
      if ((get_clock()->now() - phase_start_).seconds() > 6.0)
      { phase_ = TAKEOFF; phase_start_ = get_clock()->now(); RCLCPP_INFO(get_logger(), "TAKEOFF"); }
      break;

    case TAKEOFF:
      send_goal(0, 0, TAKEOFF_Z);
      if (at_target(TAKEOFF_Z, cur_z_, 0.2))
      { phase_ = FORWARD; phase_start_ = get_clock()->now(); reached_stable_ = false;
        RCLCPP_INFO(get_logger(), "FORWARD"); }
      break;

    case FORWARD:
      send_goal(FORWARD_X, 0, TAKEOFF_Z);
      if (at_target(FORWARD_X, cur_x_, 0.2))
      { phase_ = LAND; phase_start_ = get_clock()->now(); reached_stable_ = false;
        RCLCPP_INFO(get_logger(), "LAND"); }
      break;

    case LAND:
      send_land();
      if ((get_clock()->now() - phase_start_).seconds() > 10.0)
      { phase_ = DONE; RCLCPP_INFO(get_logger(), "Mission complete — landing triggered"); }
      break;

    case DONE:
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
