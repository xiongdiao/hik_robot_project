##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(hik_robot_task)

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
  geometry_msgs
)

##############################################################################
# Messages
##############################################################################

add_message_files(
  DIRECTORY msg
  FILES
  HikRobotSetTaskMsg.msg
  HikRobotVoiceOut.msg
  HikRobotSetModulesMsg.msg
  HikRobotFile.msg

)


add_service_files(
  DIRECTORY srv
  FILES
  HikRobotSetTaskSrv.srv
  HikRobotVoiceOutSrv.srv
  HikRobotFileSrv.srv
  HikRobotNavSrv.srv
  HikRobotPatrolSrv.srv

) 


generate_messages(DEPENDENCIES std_msgs geometry_msgs  move_base_msgs )

##############################################################################
# Package
##############################################################################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs  move_base_msgs 
)
