import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param = os.path.join(
        get_package_share_directory('forklift_server'),
        'param',
        'test.yaml'
    )
        
    node = Node(
        package='forklift_server',
        executable='test_node.py',
        name='py_test',  # 這裡的名字應該與您的 `Node` 類名一致
        parameters=[param]
    )

    return LaunchDescription([node])
