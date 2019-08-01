cmake_minimum_required(VERSION 2.8.3)
project(wheelchair)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  std_msgs
)

add_message_files(
  FILES
  joy.msg
  canMSG.msg
  FullcanMSG.msg
)
add_service_files(
  FILES
  batLevel.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pkg
#  CATKIN_DEPENDS message_generation message_runtime rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
