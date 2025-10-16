cmake_minimum_required(VERSION 3.8)
project(xbot_utils)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Declare the scan_to_pointcloud library
add_library(scan_to_pointcloud
  src/scan_to_pointcloud.cpp
)
ament_target_dependencies(scan_to_pointcloud
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
    nav_msgs
)

# Declare the scan_to_pointcloud_node executable
add_executable(scan_to_pointcloud_node src/scan_to_pointcloud_node.cpp)
target_link_libraries(scan_to_pointcloud_node
  scan_to_pointcloud
)
ament_target_dependencies(scan_to_pointcloud_node
  rclcpp
)

# Install targets
install(TARGETS
  scan_to_pointcloud
  scan_to_pointcloud_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(scan_to_pointcloud)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

ament_package()
