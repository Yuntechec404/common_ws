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
            action_param = int(cmd[2])

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

    def cmd_pub(self, twist): #限制速度的範圍
        if twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.2
        if twist.linear.x > 0 and twist.linear.x < 0.02:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.02:
            twist.linear.x =-0.02

        if twist.angular.z > 0.2:
            twist.angular.z =0.2
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.05:
            twist.angular.z =0.05
        elif twist.angular.z < 0 and twist.angular.z > -0.05:
            twist.angular.z =-0.05
        
        self.pub_cmd_vel.publish(twist)

    def execute_odom_action(self, action, param): 
        if action == 'front':
            self.fnGoStraight(param, 0.1, 0.15)
        elif action == 'back':
            self.fnGoStraight(-param, 0.1, 0.15)
        elif action == 'turn_right':
            self.fnTurn(param, 0.1, 0.15)
        elif action == 'turn_left':
            self.fnTurn(-param, 0.1, 0.15)    
        else:
            self.get_logger().error(f"Invalid odom action: {action}")
            return

    def fnTurn(self, target_angle, Kp=0.2, theta=0.):
        twist = Twist()
        twist.angular.z = Kp * theta
        target_angle_rad = math.radians(target_angle)   # 計算目標角度（弧度）
        time_needed = target_angle_rad / (Kp * theta)   # 計算所需的行駛時間
        start_time = self.get_clock().now().to_msg().sec    # 記錄開始的時間
        while (self.get_clock().now().to_msg().sec - start_time) < time_needed:
            self.cmd_pub(twist)
            time.sleep(0.1)  # 每 0.1 秒發送一次指令
        twist.angular.z = 0.0   # 停止機器人
        self.cmd_pub(twist)

    def fnGoStraight(self, distance, Kp=0.2, v=0.):
        twist = Twist()
        twist.linear.x = Kp * v
        time_needed = distance / (Kp * v)   # 計算所需的行駛時間
        start_time = self.get_clock().now().to_msg().sec    # 獲取當前時間
        # 開始移動
        while (self.get_clock().now().to_msg().sec - start_time) < time_needed:
            self.cmd_pub(twist)
            time.sleep(0.1)  # 每 0.1 秒發送一次指令
        twist.linear.x = 0.0    # 停止機器人
        self.cmd_pub(twist)

def main(args=None):
    rclpy.init(args=args)
    ctrl_server = CtrlServer()
    rclpy.spin(ctrl_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()