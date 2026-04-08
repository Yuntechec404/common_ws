cmake_minimum_required(VERSION 3.0.2)
project(laser_scan_obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  # apriltag_ros
)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  roscpp
  # apriltag_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_scan_obstacle_detection src/laser_scan_obstacle_detection.cpp)
target_link_libraries(laser_scan_obstacle_detection ${catkin_LIBRARIES})
add_executable(laser_scan_obstacle_detection_2xcamera src/laser_scan_obstacle_detection_2xcamera.cpp)
target_link_libraries(laser_scan_obstacle_detection_2xcamera ${catkin_LIBRARIES})
# add_executable(laser_scan_obstacle_detection_with_servoing src/laser_scan_obstacle_detection_with_servoing.cpp)
# target_link_libraries(laser_scan_obstacle_detection_with_servoing ${catkin_LIBRARIES})
