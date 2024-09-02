#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visual_servoing.action import VisualServoing
from geometry_msgs.msg import Twist
import math # using math.pi
import time

class CtrlServer(Node):

    def __init__(self):
        super().__init__("ctrl_server")

        # 宣告參數
        self.declare_parameter('command_list_yaml')
        # 獲取參數
        command_list_yaml = self.get_parameter('command_list_yaml').get_parameter_value().string_array_value
        # 轉換為二維清單
        self.command_2d = [cmd.split(',') for cmd in command_list_yaml]
        self.get_logger().info(f"command_2d: {self.command_2d}{type(self.command_2d)}")

        # 初始化動作客戶端
        self.pbvs_client = ActionClient(self, VisualServoing, 'VisualServoing')
        self.pbvs_client.wait_for_server()
        
        # 訂閱命令列表
        self.execute_commands()

    def execute_commands(self):
        self.current_command_index = 0
        self.execute_next_command()

    def execute_next_command(self):
        if self.current_command_index < len(self.command_2d):
            cmd = self.command_2d[self.current_command_index]
            action_type = cmd[0]
            action_name = cmd[1]

            if action_type == 'PBVS':
                action_param = int(cmd[2])
                self.get_logger().info(f"Executing {action_type} Action: {action_name} with layer {action_param}")
                self.execute_action(action_name, action_param)
            elif action_type == 'odom':
                action_param = float(cmd[2])
                self.get_logger().info(f"Executing {action_type} Action: {action_name} with layer {action_param}")
                self.execute_action(action_name, action_param)
            
            else:
                self.get_logger().error(f"Unknown command: {cmd}")

    def execute_action(self, command, layer):
        goal_msg = VisualServoing.Goal()
        goal_msg.command = command
        goal_msg.layer = layer
        send_goal_future = self.pbvs_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        while(result.result != "success"):  # 等待動作完成
            time.sleep(0.1)                 # 避免佔用過多CPU
            result = future.result().result

        self.get_logger().info(f'PBVS Action result: {result.result}')
        self.current_command_index += 1 # 移到下一個動作
        self.execute_next_command()

def main(args=None):
    rclpy.init(args=args)
    ctrl_server = CtrlServer()
    rclpy.spin(ctrl_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
