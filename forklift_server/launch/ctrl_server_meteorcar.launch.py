import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the list of commands
    command_list = [
        # ['odom', 'front', 0.5],  # 使用odom前進0.5m
        ['PBVS', 'parking_bodycamera', '2'],  # 使用牙叉相機對位棧板
        ['PBVS', 'raise_pallet', '2'],  # 叉起棧板
        # ['odom', 'front', -0.5],  # 使用odom後退0.5m
        # ['odom', 'turn', 90],  # 使用odom右轉90度
        # ['odom', 'front', 5],  # 使用odom前進5m
        # ['odom', 'turn', -90],  # 使用odom左轉90度
        # ['PBVS', 'parking_bodycamera', 2],  # 使用牙叉相機對位貨價
        # ['PBVS', 'drop_pallet', 2],  # 放下棧板
        # ['odom', 'front', -0.5],  # 使用odom後退0.5m
    ]

    # Create the launch description and node
    return LaunchDescription([
        Node(
            package='forklift_server',
            executable='ctrl_server.py',
            name='ctrl_server',
            output='screen',
            parameters=[{'command': command_list}],
        )
    ])
