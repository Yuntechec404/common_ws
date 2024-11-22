import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定義參數檔案的路徑
    package_path = get_package_share_directory('slam_nav2')
    slam_toolbox_params = os.path.join(package_path, 'param', 'slam2d_online_async_params.yaml')
    rviz_config = os.path.join(package_path, 'param', 'slam_default_view.rviz')

    # lidar_tf_node 節點
    lidar_tf_node = Node(
        package='slam_nav2',
        executable='lidar_tf_node',
        name='lidar_tf_node',
        output='screen',
        parameters=[{
            'lidar_tf_dist': [0.32, 0.0, 0.69, 0.0, 0.0, 0.0],  # 光達與 base_link 的相對距離
            'lidar_tf_name': 'velodyne'  # 光達 TF Name
        }]
    )

    # slam_toolbox 節點
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_toolbox_params]
    )

    # RViz2 節點
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 定義執行順序
    return LaunchDescription([
        # lidar_tf_node,  # 先啟動 TF 節點
        slam_toolbox_node,  # 再啟動 SLAM 節點
        rviz_node  # 最後啟動 RViz
    ])
