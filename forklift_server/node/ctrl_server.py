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
        super().__init__('ctrl_server')
        self.get_logger().warn(self.get_name() + " start")
        # 讀取命令列表參數
        self.declare_parameter('command', [])
        self.command = self.get_parameter('command').get_parameter_value().string_array_value
        # 初始化動作客戶端
        self.pbvs_client = ActionClient(self, VisualServoing, 'VisualServoing')
        self.pbvs_client.wait_for_server()
        # 訂閱命令列表
        self.execute_commands()

    def execute_commands(self):
        for cmd in self.command:
            action_type = cmd[0]
            action_name = cmd[1]
            action_param = cmd[2]

            if action_type == 'PBVS':
                self.get_logger().info(f"Executing PBVS Action: {action_name} with layer {action_param}")
                self.execute_pbvs_action(action_name, action_param)
            
            elif action_type == 'odom':
                self.get_logger().info(f"Executing Odom Action: {action_name} with param {action_param}")
                self.execute_odom_action(action_name, action_param)
            
            else:
                self.get_logger().error(f"Unknown command: {cmd}")

    def execute_pbvs_action(self, command, layer):
        goal_msg = VisualServoing.Goal()
        goal_msg.command = command
        goal_msg.layer = layer
        self.pbvs_client.send_goal(goal_msg)
        self.pbvs_client.wait_for_result()
        result = self.pbvs_client.get_result()
        self.get_logger().info(f"PBVS Action result: {result}")

    def execute_odom_action(self, action, param):
        twist = Twist()
        # if action == 'front':
        #      = param
        # elif action == 'turn':
        #      = param
        # else:
        #     self.get_logger().error(f"Invalid odom action: {action}")
        #     return
        
        # # 發布 Twist 消息
        # cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # cmd_pub.publish(twist)
        # self.get_logger().info(f"Odom Action executed: {action} with param {param}")

def main(args=None):
    rclpy.init(args=args)
    ctrl_server = CtrlServer()
    rclpy.spin(ctrl_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
