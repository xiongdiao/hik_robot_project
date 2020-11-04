##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rosjava_hikrobot_msgs)

##############################################################################
# Dependencies
##############################################################################


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  move_base_msgs
  actionlib_msgs
)

##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  HikRobotSetTaskMsg.msg
  HikRobotVoiceOut.msg
)


add_service_files(
  DIRECTORY srv
  FILES
  HikRobotSetTaskSrv.srv
) 


generate_messages(DEPENDENCIES std_msgs move_base_msgs)

##############################################################################
# Package
##############################################################################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
