cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

# ---------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------------------------------------------------------------
# Compiler warnings
# ---------------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ---------------------------------------------------------------
# wheel_odometry_node
# ---------------------------------------------------------------
add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)

ament_target_dependencies(wheel_odometry_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ---------------------------------------------------------------
# scan_restamper
# ---------------------------------------------------------------
add_executable(scan_restamper src/scan_restamper.cpp)

ament_target_dependencies(scan_restamper
  rclcpp
  sensor_msgs
)

# ---------------------------------------------------------------
# Install binaries
# ---------------------------------------------------------------
install(TARGETS
  wheel_odometry_node
  scan_restamper
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------
# Install config and launch files
# ---------------------------------------------------------------
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

