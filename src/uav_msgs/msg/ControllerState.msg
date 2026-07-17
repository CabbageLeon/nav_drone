# Controller feedback: current state and tracking error
std_msgs/Header header

uint8 MODE_POSITION=0
uint8 active_mode

# Current state
geometry_msgs/PoseStamped current_pose

# Tracking error (target - current) in odom frame
float32 position_error_x
float32 position_error_y
float32 position_error_z
