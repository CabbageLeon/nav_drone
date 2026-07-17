// SetpointManager — position setpoint dispatcher
//
// Single mode: forwards position setpoints to PX4.
// PX4 handles all trajectory execution (acceleration, cruise, deceleration, hover).
// When no goal is active, holds current position.
#pragma once

#include "uav_controller/controller_base.hpp"

namespace uav_controller
{

class SetpointManager : public ControllerBase
{
public:
  struct Params
  {
    double goal_tolerance_xy;
    double goal_tolerance_z;
    Params() : goal_tolerance_xy(0.3), goal_tolerance_z(0.3) {}
  };

  explicit SetpointManager(const Params &p = {}) : params_(p) {}

  void setGoal(const uav_msgs::msg::ControlCommand &cmd) override
  {
    goal_ = cmd;
    goal_set_ = true;
  }

  uav_msgs::msg::ControlCommand update(
      const nav_msgs::msg::Odometry &odom) override
  {
    if (!goal_set_)
      return make_hold(odom);
    return goal_;
  }

  void reset() override { goal_set_ = false; }

  bool hasGoal() const override { return goal_set_; }

  bool isGoalReached(const nav_msgs::msg::Odometry &odom) const override
  {
    if (!goal_set_) return false;
    double dx = goal_.pose.pose.position.x - odom.pose.pose.position.x;
    double dy = goal_.pose.pose.position.y - odom.pose.pose.position.y;
    double dz = goal_.pose.pose.position.z - odom.pose.pose.position.z;
    return (dx * dx + dy * dy) <
               (params_.goal_tolerance_xy * params_.goal_tolerance_xy) &&
           std::abs(dz) < params_.goal_tolerance_z;
  }

private:
  Params params_;
  uav_msgs::msg::ControlCommand goal_;
  bool goal_set_{false};

  uav_msgs::msg::ControlCommand make_hold(
      const nav_msgs::msg::Odometry &odom)
  {
    uav_msgs::msg::ControlCommand cmd;
    cmd.control_mode = uav_msgs::msg::ControlCommand::MODE_POSITION;
    cmd.pose.pose = odom.pose.pose;
    cmd.yaw = NAN;
    return cmd;
  }
};

} // namespace uav_controller
