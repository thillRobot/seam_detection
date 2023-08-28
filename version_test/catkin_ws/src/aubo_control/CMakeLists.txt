cmake_minimum_required(VERSION 2.8.3)
project(aubo_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation roscpp rospy std_msgs geometry_msgs sensor_msgs tf trajectory_msgs control_msgs)


## Generate messages in the 'msg' folder
add_message_files(FILES
  gcodeStatus.msg
  gcodeAction.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  gcodeCmdService.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  control_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
 
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)