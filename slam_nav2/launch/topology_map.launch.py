from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_nav2',
            executable='topologymap_server_node.py',
            name='topologymap_server_node',
            output='screen',
            parameters=[
                {'start_node': 'P1'},
                {'graph': '{"P1": {"P2": 1, "P3": 6}, "P2": {"P1": 1}, "P3": {"P1": 6}}'},
                {'waypoints': '{"P1": [0.507, -0.382, 0.084, 0.996], "P2": [2.405, -0.539, 0.088, 0.996], "P3": [2.405, -0.539, 0.088, 0.996]}'}
            ]
        )
    ])
