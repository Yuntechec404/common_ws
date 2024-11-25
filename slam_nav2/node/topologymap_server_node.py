#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from visual_servoing.action import TopologyMap
import heapq
import math
import json

class TopologyMapServerNode(Node):

    def __init__(self):
        super().__init__('topologymap_server_node')

        # 載入參數
        self.declare_parameter("graph", "{}")
        self.declare_parameter("waypoints", "{}")
        self.declare_parameter("start_node", "P1")

        try:
            graph_param = self.get_parameter("graph").value
            waypoints_param = self.get_parameter("waypoints").value
            self.graph = json.loads(graph_param if isinstance(graph_param, str) else '{}')
            self.waypoints = json.loads(waypoints_param if isinstance(waypoints_param, str) else '{}')
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode parameters: {e}")
            self.graph = {}
            self.waypoints = {}
        self.start_node = self.get_parameter("start_node").value

        self.get_logger().info(f"Graph: {self.graph}")
        self.get_logger().info(f"Waypoints: {self.waypoints}")
        self.get_logger().info(f"Start Node: {self.start_node}")

        # 初始化 Action Server
        self._action_server = ActionServer(self, TopologyMap, 'TopologyMap', self.execute_callback)

    def execute_callback(self, goal_handle):
        """Action 執行回呼函數"""
        self.get_logger().info(f"Received goal: {goal_handle.request.goal}")
        goal_node = goal_handle.request.goal    # 確定目標節點
        path, distance = self.calculate_shortest_path(self.start_node, goal_node)   # 計算最短路徑

        if goal_node not in self.graph:
            self.get_logger().error(f"Goal node {goal_node} is not defined in the graph!")
            result = TopologyMap.Result()
            result.path = []
            result.distance = float('inf')
            goal_handle.abort()
            return result
        
        result = TopologyMap.Result()   # 構建結果
        if path:
            result.path = path
            result.distance = float(distance)  # 確保 distance 是 float
            self.get_logger().info(f"Path: {path}, Distance: {distance}")
            goal_handle.succeed()
        else:
            result.path = []
            result.distance = float('inf')  # 顯式轉換為 float 類型
            self.get_logger().warn(f"Path to {goal_node} not found!")
            goal_handle.abort()

        return result

    def calculate_shortest_path(self, start, goal):
        """使用 Dijkstra 演算法計算最短路徑"""
        if start not in self.graph or goal not in self.graph:
            return None, float('inf')

        # 優先佇列
        priority_queue = []
        heapq.heappush(priority_queue, (0, start))
        visited = set()
        parent = {start: None}
        distance = {vertex: float('inf') for vertex in self.graph}
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
        if distance[goal] == float('inf'):
            return None, float('inf')
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
