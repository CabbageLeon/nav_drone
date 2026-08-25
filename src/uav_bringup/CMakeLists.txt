cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

find_package(ament_cmake REQUIRED)

install(FILES launch/uav_system.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(CODE "
  file(REMOVE
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/sitl_bringup.launch.py\"
    \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch/control_only.launch.py\"
  )
")

ament_package()
