#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from visual_servoing.action import TopologyMap
import heapq
import math
import json  # 用於解析 JSON 格式的參數

class TopologyMapServerNode(Node):

    def __init__(self):
        super().__init__('topologymap_server_node')

        # 初始化 Action Server
        self._action_server = ActionServer(self, TopologyMap, 'TopologyMap', self.execute_callback)

        # 載入參數
        self.declare_parameter("graph", "{}")  # 預設為 JSON 格式的空字串
        self.declare_parameter("waypoints", "{}")
        self.declare_parameter("start_node", "P1")

        # 將參數轉換為 Python 字典
        try:
            self.graph = json.loads(self.get_parameter("graph").get_parameter_value().string_value)
            self.waypoints = json.loads(self.get_parameter("waypoints").get_parameter_value().string_value)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON parameters: {e}")
            self.graph = {}
            self.waypoints = {}

        self.start_node = self.get_parameter("start_node").get_parameter_value().string_value

        self.get_logger().info(f"Graph: {self.graph}")
        self.get_logger().info(f"Waypoints: {self.waypoints}")
        self.get_logger().info(f"Start Node: {self.start_node}")

    def execute_callback(self, goal_handle):
        """Action 執行回呼函數"""
        self.get_logger().info(f"Received goal: {goal_handle.request.goal}")

        # 確定起始節點和目標節點
        goal_node = goal_handle.request.goal

        # 計算最短路徑
        path, distance = self.calculate_shortest_path(self.start_node, goal_node)

        # 構建結果
        result = TopologyMap.Result()
        if path:
            result.path = path
            result.distance = distance
            self.get_logger().info(f"Path: {path}, Distance: {distance}")
            goal_handle.succeed()
        else:
            result.path = []
            result.distance = math.inf
            self.get_logger().warn(f"Path to {goal_node} not found!")
            goal_handle.abort()

        return result

    def calculate_shortest_path(self, start, goal):
        """使用 Dijkstra 演算法計算最短路徑"""
        if start not in self.graph or goal not in self.graph:
            return None, math.inf

        # 優先佇列
        priority_queue = []
        heapq.heappush(priority_queue, (0, start))
        visited = set()
        parent = {start: None}
        distance = {vertex: math.inf for vertex in self.graph}
        distance[start] = 0

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)
            if current_node in visited:
                continue

            visited.add(current_node)

            for neighbor, weight in self.graph[current_node].items():
                if neighbor not in visited:
                    new_distance = current_distance + weight
                    if new_distance < distance[neighbor]:
                        distance[neighbor] = new_distance
                        parent[neighbor] = current_node
                        heapq.heappush(priority_queue, (new_distance, neighbor))

        # 獲取從起始點到目標點的完整路徑
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parent.get(current)

        path.reverse()
        if distance[goal] == math.inf:
            return None, math.inf
        return path, distance[goal]


def main(args=None):
    rclpy.init(args=args)
    node = TopologyMapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TopologyMapServerNode...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
