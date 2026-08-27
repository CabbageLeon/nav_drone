cmake_minimum_required(VERSION 3.8)
project(uav_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp uav_msgs nav_msgs geometry_msgs)

add_executable(mission_node src/mission.cpp)
ament_target_dependencies(mission_node rclcpp uav_msgs nav_msgs geometry_msgs)

install(TARGETS controller_node mission_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/spf.py
  src/spf_image_goal_test.py
  src/ugv_communicate.py
  src/tcp_protocol.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
