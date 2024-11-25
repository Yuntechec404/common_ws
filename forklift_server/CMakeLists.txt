cmake_minimum_required(VERSION 3.5)
project(forklift_server)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()
# c++ & python
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# message
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
# find_package(visual_servoing REQUIRED)
# tf
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# 3rd party
find_package(slam_nav2 REQUIRED)

# message
# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/VisualServoing.action"
#   DEPENDENCIES std_msgs
# )

install(PROGRAMS
  node/ctrl_server.py
  node/test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()