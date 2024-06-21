cmake_minimum_required(VERSION 3.8)

project(ars548_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set (CMAKE_BUILD_TYPE Debug)
# find dependencies
# find_package(roscpp REQUIRED)
# find_package(ars548_messages REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             sensor_msgs
             geometry_msgs
             ars548_messages
             roscpp
             tf2_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}

)

catkin_package (
    INCLUDE_DIRS include
    CATKIN_DEPENDS ars548_messages
)

add_executable(ars548_driver src/ars548_driver_node.cpp)
add_dependencies(ars548_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(ars548_driver ${catkin_LIBRARIES} )


# TODO: driver filter
#add_executable(ars548_driver_filter src/ars548_driver_filter.cpp)
#add_dependencies(ars548_driver_filter roscpp sensor_msgs tf2 geometry_msgs) 

install(TARGETS
  ars548_driver
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

