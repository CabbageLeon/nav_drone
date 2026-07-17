// Frame transform utilities — boundary adapter between PX4 and ROS2
//
// Two independent transforms:
//
// 1. BODY frame:  PX4 FRD (Forward-Right-Down) ↔ ROS FLU (Forward-Left-Up)
//    Always applied. This is NOT ENU/NED — it's the body frame convention.
//    Rotation: +180° around X (Forward) axis.
//
// 2. WORLD frame: PX4 NED (North-East-Down) ↔ ROS ENU (East-North-Up)
//    Configurable via param. Needed for RViz/Nav2 interop, not for flight.
//    Transformation: swap X↔Y, flip Z sign.
//
// The primary frames in this system are:
//   - odom:    world frame (default: same convention as PX4 local origin)
//   - base_link: body frame, X=forward, Y=left, Z=up (FLU)
//
// Coordinate transforms for sensors attach to base_link via TF2 static transforms
// (managed by uav_estimator), NOT via NED/ENU math.
#pragma once

#include "uav_px4_bridge/frame_transforms.h"
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace uav_px4_bridge
{

// ── Body frame: FRD ↔ FLU (always needed) ──
// PX4 body = FRD (Forward, Right, Down)
// ROS  body = FLU (Forward, Left, Up)
// Transform: +180° around X

inline Eigen::Vector3d px4_body_to_ros_body(const Eigen::Vector3d &frd)
{
  // Y → -Y (Right → Left), Z → -Z (Down → Up)
  return Eigen::Vector3d(frd.x(), -frd.y(), -frd.z());
}

inline Eigen::Vector3d ros_body_to_px4_body(const Eigen::Vector3d &flu)
{
  return Eigen::Vector3d(flu.x(), -flu.y(), -flu.z());
}

// px4_to_ros_orientation / ros_to_px4_orientation are v1.15 additions,
// not present in v1.13; expanded inline as the two-step chain.
inline Eigen::Quaterniond px4_attitude_to_ros(const Eigen::Quaterniond &px4_q)
{
  // PX4: aircraft→NED  →  ROS: base_link→ENU (two steps: NED→ENU world, aircraft→base_link body)
  return px4_ros_com::frame_transforms::baselink_to_aircraft_orientation(
             px4_ros_com::frame_transforms::ned_to_enu_orientation(px4_q));
}

inline Eigen::Quaterniond ros_attitude_to_px4(const Eigen::Quaterniond &ros_q)
{
  return px4_ros_com::frame_transforms::aircraft_to_baselink_orientation(
             px4_ros_com::frame_transforms::enu_to_ned_orientation(ros_q));
}

// ── World frame: NED ↔ ENU (configurable, for simulation interop) ──

inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d &ned)
{
  return px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned);
}

inline Eigen::Vector3d enu_to_ned(const Eigen::Vector3d &enu)
{
  return px4_ros_com::frame_transforms::enu_to_ned_local_frame(enu);
}

// ── Combined: PX4 body-velocity (FRD, in NED world) → ROS body-velocity (FLU, in ENU world) ──

inline Eigen::Vector3d px4_velocity_to_ros_enu(const Eigen::Vector3d &px4_body_vel)
{
  auto enu_world_vel = ned_to_enu(px4_body_vel);
  return px4_body_to_ros_body(enu_world_vel);
}

inline Eigen::Vector3d ros_velocity_to_px4_ned(const Eigen::Vector3d &ros_body_vel)
{
  auto ned_world_vel = enu_to_ned(ros_body_vel);
  return ros_body_to_px4_body(ned_world_vel);
}

// ── Yaw transforms (simple angle conversion) ──

inline double ned_yaw_to_enu(double ned_yaw)
{
  double enu_yaw = M_PI_2 - ned_yaw;
  while (enu_yaw > M_PI)  enu_yaw -= 2.0 * M_PI;
  while (enu_yaw < -M_PI) enu_yaw += 2.0 * M_PI;
  return enu_yaw;
}

inline double enu_yaw_to_ned(double enu_yaw)
{
  double ned_yaw = M_PI_2 - enu_yaw;
  while (ned_yaw > M_PI)  ned_yaw -= 2.0 * M_PI;
  while (ned_yaw < -M_PI) ned_yaw += 2.0 * M_PI;
  return ned_yaw;
}

} // namespace uav_px4_bridge
