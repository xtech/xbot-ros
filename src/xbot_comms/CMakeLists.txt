cmake_minimum_required(VERSION 3.8)
project(xbot_comms)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(XBOT_BUILD_LIB_SERVICE OFF)
add_subdirectory(ext/xbot_framework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(xbot_comms src/xbot-comms.cpp
        src/xbot_comms_node.cpp
        src/xbot_comms_node.h
        src/diff_drive_service_interface.cpp
        src/imu_service_interface.cpp
)
target_link_libraries(xbot_comms PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${sensor_msgs_TARGETS})
target_link_libraries(xbot_comms PRIVATE xbot-service-interface)
target_include_directories(xbot_comms PRIVATE ext/service-definitions)

target_add_service_interface(xbot_comms ImuServiceInterface ${CMAKE_CURRENT_SOURCE_DIR}/ext/service-definitions/imu_service.json)
target_add_service_interface(xbot_comms DiffDriveServiceInterface ${CMAKE_CURRENT_SOURCE_DIR}/ext/service-definitions/diff_drive_service.json)
target_add_service_interface(xbot_comms PowerServiceInterface ${CMAKE_CURRENT_SOURCE_DIR}/ext/service-definitions/power_service.json)

install(TARGETS
        xbot_comms
        DESTINATION lib/${PROJECT_NAME})

ament_package()