# ROS2 humble common package

# visual_servoing

## Dependency
```sh
$ sudo apt install ros-humble-tf-transformations
$ sudo pip3 install transforms3d
```
# AprilTag

## Dependency
```sh
$ sudo apt install ros-humble-apriltag-msgs
$ sudo apt install ros-humble-apriltag
```
# Run
```sh
 $ ros2 run apriltag_ros apriltag_node --ros-args     -p image_rect:=/camera/camera/color/image_raw     -p camera_info:=/camera/camera/color/camera_info     --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml
```