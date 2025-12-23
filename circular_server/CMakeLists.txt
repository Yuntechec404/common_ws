cmake_minimum_required(VERSION 3.0.2)
project(circular_server)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  actionlib
  actionlib_msgs
  geometry_msgs
  tf
  message_generation
  circular_saw_controller
  forklift_msg
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES TopologyMap.action
  FILES PBVSFoundationpose.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
