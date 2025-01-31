cmake_minimum_required(VERSION 3.8)

project(ars548_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD_REQUIRED 23)
set (CMAKE_BUILD_TYPE Release)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ars548_messages REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
include_directories(include)

add_executable(ars548_driver src/ars548_driver_node.cpp)
ament_target_dependencies(ars548_driver rclcpp ars548_messages sensor_msgs tf2_geometry_msgs)

add_executable(ars548_filter_node src/ars548_filter_node.cpp)
ament_target_dependencies(ars548_filter_node rclcpp sensor_msgs ars548_messages tf2_geometry_msgs) 

add_executable(radar_setup src/radar_setup.cpp)
ament_target_dependencies(radar_setup tf2_geometry_msgs sensor_msgs ars548_messages)

install(TARGETS
  ars548_driver
  radar_setup
  ars548_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
  )
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
  )
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

ament_package()
