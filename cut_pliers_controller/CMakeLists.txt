cmake_minimum_required(VERSION 3.0.2)
project(cut_pliers_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  tf
)

## 宣告要產生的訊息檔案
add_message_files(
  FILES
  CmdCutPliers.msg
)

# 產生訊息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# catkin 包含的訊息與服務
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime serial tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 加入可執行檔案與編譯設置
add_executable(cut_pliers_controller src/cut_pliers_controller.cpp)
target_link_libraries(cut_pliers_controller ${catkin_LIBRARIES})
add_dependencies(cut_pliers_controller ${catkin_EXPORTED_TARGETS} cut_pliers_controller_generate_messages_cpp)

add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})
add_dependencies(keyboard ${catkin_EXPORTED_TARGETS} cut_pliers_controller_generate_messages_cpp)

add_executable(cut_pliers_arm1 src/cut_pliers_arm1.cpp)
target_link_libraries(cut_pliers_arm1 ${catkin_LIBRARIES})
add_dependencies(cut_pliers_arm1 ${catkin_EXPORTED_TARGETS} cut_pliers_controller_generate_messages_cpp)

add_executable(cut_pliers_arm2 src/cut_pliers_arm2.cpp)
target_link_libraries(cut_pliers_arm2 ${catkin_LIBRARIES})
add_dependencies(cut_pliers_arm2 ${catkin_EXPORTED_TARGETS} cut_pliers_controller_generate_messages_cpp)

add_executable(keyboard_publisher src/keyboard.cpp)
target_link_libraries(keyboard_publisher ${catkin_LIBRARIES})
add_dependencies(keyboard_publisher ${catkin_EXPORTED_TARGETS} cut_pliers_controller_generate_messages_cpp)

install(TARGETS cut_pliers_controller keyboard cut_pliers_arm1 cut_pliers_arm2 keyboard_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
