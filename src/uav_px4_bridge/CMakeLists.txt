cmake_minimum_required(VERSION 3.8)
project(uav_px4_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Vendored frame_transforms
add_library(frame_transforms STATIC
  src/lib/frame_transforms.cpp
)
ament_target_dependencies(frame_transforms
  geometry_msgs sensor_msgs Eigen3
)

# px4_bridge_node (MAVROS version)
add_executable(px4_bridge_node
  src/px4_bridge_node.cpp
)
ament_target_dependencies(px4_bridge_node
  rclcpp mavros_msgs uav_msgs nav_msgs geometry_msgs std_msgs Eigen3
)
target_link_libraries(px4_bridge_node frame_transforms)

install(TARGETS px4_bridge_node frame_transforms
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
