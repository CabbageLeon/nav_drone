# Control command — unified interface between controller and px4_bridge
#
# Single mode: position setpoint in odom frame.
# PX4 handles the full trajectory (acceleration, cruise, deceleration, hover)
# using its internal position controller.
#
# Frame conventions:
#   - pose: odom frame (world, origin at PX4 local position reference)
#   - yaw:  odom frame yaw angle [rad]
#
# PX4 frame conversion (NED↔odom) is handled internally by uav_px4_bridge.

uint8 MODE_POSITION=0  # Position control in odom frame
uint8 MODE_LAND=1      # Trigger PX4 AUTO.LAND (bridge handles mode switch)

uint8 control_mode      # MODE_POSITION or MODE_LAND

geometry_msgs/PoseStamped pose  # odom frame — target position + orientation

float32 yaw             # Desired yaw in odom frame [rad], NaN = hold current
