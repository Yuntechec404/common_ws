import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('forklift_server'),
        'param',
        'parameter.yaml'
    )
        
    node = Node(
        package='forklift_server',
        executable='ctrl_server_.py',
        name='ctrl_server_',
        parameters=[param]
    )

    return LaunchDescription([node])
