#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visual_servoing.action import VisualServoing
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json  # 用於解析導航點字典

class CtrlServer(Node):

    def __init__(self):
        super().__init__("ctrl_server")

        # 宣告參數
        self.declare_parameter('command_list_yaml')
        self.declare_parameter('waypoints')
        command_list_yaml = self.get_parameter('command_list_yaml').get_parameter_value().string_array_value
        waypoints_json = self.get_parameter('waypoints').get_parameter_value().string_value

        # 解析參數
        self.command_2d = [cmd.split(',') for cmd in command_list_yaml]
        self.waypoints = json.loads(waypoints_json)
        self.get_logger().info(f"command_2d: {self.command_2d}")
        self.get_logger().info(f"waypoints: {self.waypoints}")

        # 初始化動作客戶端
        self.pbvs_client = ActionClient(self, VisualServoing, 'VisualServoing')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pbvs_client.wait_for_server()
        self.nav_client.wait_for_server()

        # 開始執行命令
        self.current_command_index = 0
        self.execute_next_command()

    def execute_next_command(self):
        if self.current_command_index < len(self.command_2d):
            cmd = self.command_2d[self.current_command_index]
            action_type = cmd[0]

            if action_type == 'PBVS' or action_type == 'odom' :
                self.execute_pbvs(cmd[1], float(cmd[2]))
            elif action_type == 'TopologyMap':
                self.execute_navigation(cmd[1])
            else:
                self.get_logger().error(f"Unknown command: {cmd}")
                self.current_command_index += 1
                self.execute_next_command()

    def execute_pbvs(self, command, layer_dist):
        self.get_logger().info(f"Executing PBVS action: {command}, layer: {layer_dist}")
        goal_msg = VisualServoing.Goal()
        goal_msg.command = command
        goal_msg.layer_dist = layer_dist

        send_goal_future = self.pbvs_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.pbvs_response_callback)

    def pbvs_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('PBVS goal rejected')
        else:
            self.get_logger().info('PBVS goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.pbvs_result_callback)

    def pbvs_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'PBVS result: {result.result}')
        self.current_command_index += 1
        self.execute_next_command()

    def execute_navigation(self, target):
        if target not in self.waypoints:
            self.get_logger().error(f"Unknown waypoint: {target}")
            self.current_command_index += 1
            self.execute_next_command()
            return

        coords = self.waypoints[target]
        self.get_logger().info(f"Navigating to waypoint: {target}, coordinates: {coords}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = coords[0]
        goal_msg.pose.pose.position.y = coords[1]
        goal_msg.pose.pose.orientation.z = coords[2]
        goal_msg.pose.pose.orientation.w = coords[3]
        goal_msg.behavior_tree = ''

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
        else:
            self.get_logger().info('Navigation goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        self.get_logger().info('Navigation result received.')
        self.current_command_index += 1
        self.execute_next_command()

def main(args=None):
    rclpy.init(args=args)
    ctrl_server = CtrlServer()
    rclpy.spin(ctrl_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
