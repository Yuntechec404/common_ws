import os

import launch
import launch.actions
import launch.events

from launch import LaunchDescription
from launch.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the launch description and node

    param_dir = launch.substitutions.LaunchConfiguration(
            'param_dir',
            default=os.path.join(
                get_package_share_directory('forklift_server'),
                'param',
                'parameter.yaml'))

    return LaunchDescription([
        Node(
            package='forklift_server',
            executable='ctrl_server.py',
            name='ctrl_server',
            parameters=[param_dir],
            output='screen'
        )
    ])
