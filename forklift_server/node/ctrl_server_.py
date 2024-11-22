#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visual_servoing.action import VisualServoing, TopologyMap

class CtrlServer(Node):

    def __init__(self):
        super().__init__("ctrl_server_")

        # 宣告參數
        self.declare_parameter('command_list_yaml')
        # 獲取參數
        command_list_yaml = self.get_parameter('command_list_yaml').get_parameter_value().string_array_value
        # 轉換為二維清單
        self.command_2d = [cmd.split(',') for cmd in command_list_yaml]
        self.get_logger().info(f"command_2d: {self.command_2d}{type(self.command_2d)}")

    async def send_pbvs(self, command):
        self.get_logger().info(f"Sending PBVS command: {command}")
        client = ActionClient(self, VisualServoing, 'VisualServoing')
        await client.wait_for_server()
        goal_msg = VisualServoing.Goal()
        goal_msg.command = command

        future = client.send_goal_async(goal_msg)
        goal_handle = await future
        if not goal_handle.accepted:
            self.get_logger().warn('PBVS goal rejected')
            return
        result_future = goal_handle.get_result_async()
        result = await result_future
        self.get_logger().info(f"PBVS result: {result.result}")

    async def send_topology_map(self, goal):
        self.get_logger().info(f"Sending TopologyMap goal: {goal}")
        client = ActionClient(self, TopologyMap, 'TopologyMap')
        await client.wait_for_server()
        goal_msg = TopologyMap.Goal()
        goal_msg.goal = goal

        future = client.send_goal_async(goal_msg)
        goal_handle = await future
        if not goal_handle.accepted:
            self.get_logger().warn('TopologyMap goal rejected')
            return
        result_future = goal_handle.get_result_async()
        result = await result_future
        self.get_logger().info(f"TopologyMap result: {result.result}")

    async def process_commands(self):
        for msg in self.command_2d:
            if msg[0] == 'PBVS' or msg[0] == 'odom':
                self.get_logger().info(f"Processing PBVS: {msg[1]}")
                await self.send_pbvs(msg[1])

            elif msg[0] == 'TopologyMap':
                self.get_logger().info(f"Processing TopologyMap: {msg[1]}")
                await self.send_topology_map(msg[1])
            else:
                self.get_logger().warn(f"Unknown command: {msg}")

        self.get_logger().info("Finished processing commands.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    ctrl_server_ = CtrlServer()
    rclpy.spin_until_future_complete(ctrl_server_, ctrl_server_.process_commands())

if __name__ == '__main__':
    main()
