cmake_minimum_required(VERSION 3.5)
project(visual_servoing)

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
# tf
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# 3rd party
find_package(serial REQUIRED)

# message
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/VisualServoing.action"
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  node/visual_servoing_node.py
  node/pose_visualization.py
  script/action_sequence.py
  script/action.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
