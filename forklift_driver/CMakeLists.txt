cmake_minimum_required(VERSION 3.0.2)
project(forklift_driver)

find_package(catkin REQUIRED COMPONENTS 
roscpp 
rospy 
std_msgs 
genmsg 
serial 
tf 
nav_msgs 
forklift_msg
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package()
add_compile_options(-std=c++11)

include_directories(
include 
${catkin_INCLUDE_DIRS} 
)
add_executable(forklift_driver node/forklift.cpp node/stm32.cpp)
target_link_libraries(forklift_driver ${catkin_LIBRARIES})

add_executable(car_driver src/car_driver.cpp)
target_link_libraries(car_driver ${catkin_LIBRARIES})
