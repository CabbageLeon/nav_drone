# Planner goal — placeholder for future uav_planner
std_msgs/Header header

# Target waypoint in ENU
geometry_msgs/PoseStamped target_pose

# Optional: desired arrival velocity / tolerance
float32 position_tolerance   # meters, default 0.2
float32 yaw_tolerance        # radians, default 0.1

# Optional: time constraint
builtin_interfaces/Duration timeout  # zero = no timeout
