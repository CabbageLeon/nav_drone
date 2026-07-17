cmake_minimum_required(VERSION 3.8)
project(uav_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

# Message definitions
set(msg_files
  msg/ControlCommand.msg
  msg/ControllerState.msg
  msg/PlannerGoal.msg
  msg/Obstacle.msg
  msg/ObstacleArray.msg
)

# Service definitions
set(srv_files
  srv/SetControlMode.srv
)

# Action definitions
set(action_files
  action/NavigateTo.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
