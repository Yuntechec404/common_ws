cmake_minimum_required(VERSION 2.8.3)
project(forklift_server)

find_package(
catkin REQUIRED COMPONENTS 
roscpp 
actionlib 
actionlib_msgs 
geometry_msgs 
tf 
forklift_msg  
message_generation
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES TopologyMap.action
  FILES PBVS.action
  FILES PBVSMegapose.action
)

add_message_files(
  FILES
  Detection.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


