cmake_minimum_required(VERSION 3.8)
project(ars548_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs)

add_message_files(FILES 
  Detection.msg  DetectionList.msg Object.msg ObjectList.msg Status.msg AccelerationLateralCoG.msg
  AccelerationLongitudinalCoG.msg CharacteristicSpeed.msg DrivingDirection.msg SensorConfiguration.msg
  SteeringAngleFrontAxle.msg VelocityVehicle.msg YawRate.msg   )

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

install(TARGETS
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
