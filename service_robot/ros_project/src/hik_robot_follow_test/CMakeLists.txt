cmake_minimum_required(VERSION 3.0.2)
project(hik_robot_follow_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  move_base_msgs
  roscpp
  rospy
  std_msgs
)

add_message_files(
    DIRECTORY msg
)

add_service_files( 
    DIRECTORY srv
)
    

generate_messages( DEPENDENCIES 
    std_msgs 
    move_base_msgs 
    actionlib_msgs
)

catkin_package(CATKIN_DEPENDS 
    message_runtime 
    std_msgs
)


catkin_install_python(PROGRAMS 
    scripts/xd_follow_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



