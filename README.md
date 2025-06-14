# Common Project Installation
此專案包含以下專案，請先依照教學安裝或移除專案  
* ZED 驅動，[安裝 cuda 與 ZED SDK](https://hackmd.io/0GbZVLJGRn6CMj-0_2wntw)。  
* RealSense 驅動，請先安裝 RealSense SDK 或通過以下方式安裝依賴：  
```
$ sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

$ sudo apt-get install ros-$ROS_DISTRO-realsense2-description
```
* apriltag_ros，[要先裝 apriltag](https://github.com/AprilRobotics/apriltag_ros)  

建立工作區
```
$ mkdir -p ~/common_ws/src 

$ cd ~/common_ws/src

$ git clone https://github.com/Yuntechec404/common_ws.git
```

安裝依賴：  
```
$ cd ~/common_ws

$ rosdep install --from-paths src --ignore-src -r -y
```

編譯並設定環境變數：  
```
$ catkin_make

$ echo "source ~/common_ws/devel/setup.bash" >> ~/.bashrc

$ source ~/.bashrc
```

# Q&A
1. 如果導航時出現問題，有可能是以下依賴尚未安裝  
* move_base
* global_planner
* teb_local_planner
* map_server
* hector_trajectory_server
```
$ sudo apt-get install ros-noetic-move-base ros-noetic-global-planner ros-noetic-teb-local-planner ros-noetic-map-server ros-noetic-hector-trajectory-server
```

2. four-wheel-differential-drive 專案報錯，尚未安裝 Serial
```
$ sudo apt-get install ros-noetic-serial
```

# 20250218 以下錯誤未處理
apriltag_ros 專案出現 OpenCV 版本不符，預設為 OpenCV 4.2.0，需要 OpenCV 4.7.0  
請參考 [Ubuntu 20.04 原始碼編譯安裝 OpenCV 4.7.0](https://blog.csdn.net/weixin_43863869/article/details/128552342)  
<br/>
下載原始碼：  
```
$ wget https://github.com/opencv/opencv/archive/refs/tags/4.7.0.zip

$ unzip 4.7.0.zip
```

安裝依賴：  
※ 此步驟安裝 ROS 時就已安裝，除非編譯時出現狀況，否則無需操作  
```
$ cd opencv-4.7.0

$ sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg.dev libtiff5.dev libswscale-dev
```

編譯：  
```
$ cd opencv-4.7.0

$ mkdir build

$ cd build

$ cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TIFF=ON ..
```
安裝：
```
$ sudo make install
```