cmake_minimum_required(VERSION 3.0.2)
project(circular_saw_controller)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial tf nav_msgs message_generation)

add_message_files(
  FILES
  CircularSawCmd.msg
  CircularSawState.msg
)

generate_messages(DEPENDENCIES std_msgs)
## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp serial std_msgs message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(circular_saw src/circular_saw.cpp)
target_link_libraries(circular_saw ${catkin_LIBRARIES})
# add_dependencies(circular_saw circular_saw_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
