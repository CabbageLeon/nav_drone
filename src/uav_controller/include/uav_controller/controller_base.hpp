// ControllerBase — abstract setpoint manager interface
//
// Does NOT implement PID (PX4 handles that internally).
// Responsibilities:
//   1. Accept goals from planner / user
//   2. Forward setpoints to PX4 via bridge
//   3. Manage takeoff / landing / hold sequences
//   4. Track goal progress (arrival detection)
//
// Frame conventions:
//   - pose:  odom frame (world, origin at PX4 local position origin)
//   - twist: base_link frame (body, X = forward)
#pragma once

#include <uav_msgs/msg/control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>

namespace uav_controller
{

class ControllerBase
{
public:
  using SharedPtr = std::shared_ptr<ControllerBase>;

  virtual ~ControllerBase() = default;

  // Accept a new goal. May be position (odom frame) or velocity (base_link).
  virtual void setGoal(const uav_msgs::msg::ControlCommand &cmd) = 0;

  // Compute the setpoint that should be sent to PX4 right now.
  // For position mode: returns the target position (in odom frame).
  // For velocity mode: returns the target velocity (in base_link frame).
  // When holding: returns current position to maintain hover.
  virtual uav_msgs::msg::ControlCommand update(
      const nav_msgs::msg::Odometry &odom) = 0;

  // Reset internal state (clear goal, integrators if any).
  virtual void reset() = 0;

  // Whether the current goal has been reached.
  virtual bool isGoalReached(const nav_msgs::msg::Odometry &odom) const = 0;

  // Whether a goal is currently active.
  virtual bool hasGoal() const = 0;
};

} // namespace uav_controller
