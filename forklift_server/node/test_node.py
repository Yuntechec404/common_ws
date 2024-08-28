#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TestYAMLParams(Node):

    def __init__(self):
        super().__init__('py_test')
        
        # 宣告參數
        self.declare_parameter('bool_value')
        self.declare_parameter('int_number')
        self.declare_parameter('float_number')
        self.declare_parameter('str_text')
        self.declare_parameter('bool_array')
        self.declare_parameter('int_array')
        self.declare_parameter('float_array')
        self.declare_parameter('str_array')
        self.declare_parameter('nested_param.another_int')
        self.declare_parameter('command_list')
        
        # 獲取參數並打印
        bool_value = self.get_parameter('bool_value').get_parameter_value().bool_value
        int_number = self.get_parameter('int_number').get_parameter_value().integer_value
        float_number = self.get_parameter('float_number').get_parameter_value().double_value
        str_text = self.get_parameter('str_text').get_parameter_value().string_value
        command_list = self.get_parameter('command_list').get_parameter_value().string_value

        self.get_logger().info(f"bool_value: {bool_value}")
        self.get_logger().info(f"int_number: {int_number}")
        self.get_logger().info(f"float_number: {float_number}")
        self.get_logger().info(f"str_text: {str_text}")
        self.get_logger().info(f"str_text: {command_list}")

        # 如果您需要獲取陣列參數
        bool_array = self.get_parameter('bool_array').get_parameter_value().bool_array_value
        int_array = self.get_parameter('int_array').get_parameter_value().integer_array_value
        float_array = self.get_parameter('float_array').get_parameter_value().double_array_value
        str_array = self.get_parameter('str_array').get_parameter_value().string_array_value

        self.get_logger().info(f"bool_array: {bool_array}")
        self.get_logger().info(f"int_array: {int_array}")
        self.get_logger().info(f"float_array: {float_array}")
        self.get_logger().info(f"str_array: {str_array}")

        # 獲取嵌套的參數
        nested_param = self.get_parameter('nested_param.another_int').get_parameter_value().integer_value
        self.get_logger().info(f"nested_param.another_int: {nested_param}")

        command_arr = self.string_split(command_list)
    
    def string_split(self,str = ""):
        ans = str.split(';')
        for cmd in ans:
            self.get_logger().info(f"ans: {cmd}")
        return ans
        

def main(args=None):
    rclpy.init(args=args)
    node = TestYAMLParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()