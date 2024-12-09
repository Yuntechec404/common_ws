cmake_minimum_required(VERSION 3.8)
project(slam_nav2)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(visual_servoing REQUIRED)

# Add the executable
# add_executable(lidar_tf_node src/lidar_tf_node.cpp)
# ament_target_dependencies(lidar_tf_node
#   rclcpp
#   geometry_msgs
#   tf2_ros
#   tf2
# )

# Install targets
install(PROGRAMS
  node/topologymap_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and param files
install(DIRECTORY
  launch
  param
  maps
  DESTINATION share/${PROJECT_NAME}
)
ament_package()