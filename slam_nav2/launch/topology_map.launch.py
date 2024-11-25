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
                {'waypoints': '{"P1": [1.786, -0.379, -0.999, 0.011], "P2": [6.653, -0.414, 0.719, 0.694], "P3": [9.287, 0.002, -0.663, 0.748]}'}
            ]
        )
    ])
