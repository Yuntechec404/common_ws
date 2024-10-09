#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
import tf_transformations # using euler_from_quaternion
import csv
from datetime import datetime
import numpy as np
from visp_megapose.msg import Confidence

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_state, initial_uncertainty):
        if len(initial_state) != 10:
            raise ValueError("initial_state 必須包含10個元素")
        self.state = np.array(initial_state)
        self.uncertainty = np.eye(len(self.state)) * initial_uncertainty
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        
        # 狀態轉移矩陣
        self.A = np.eye(len(self.state))
        self.A[0, 1] = 1  # x 和速度 vx
        self.A[2, 3] = 1  # y 和速度 vy
        self.A[4, 5] = 1  # z 和速度 vz

        # 測量矩陣
        self.H = np.zeros((7, 10))
        self.H[0, 0] = 1  # x
        self.H[1, 2] = 1  # y
        self.H[2, 4] = 1  # z
        self.H[3, 6] = 1  # qx
        self.H[4, 7] = 1  # qy
        self.H[5, 8] = 1  # qz
        self.H[6, 9] = 1  # qw

        # 過程噪聲和測量噪聲
        self.Q = np.eye(len(self.state)) * self.process_variance
        self.R = np.eye(7) * self.measurement_variance

    def predict(self):
        self.state = self.A @ self.state
        self.uncertainty = self.A @ self.uncertainty @ self.A.T + self.Q

    def update(self, measurement):
        z = np.array(measurement)
        y = z - self.H @ self.state  # 殘差
        S = self.H @ self.uncertainty @ self.H.T + self.R
        K = self.uncertainty @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.uncertainty = (np.eye(len(self.state)) - K @ self.H) @ self.uncertainty

        # 歸一化四元数
        quaternion = self.state[6:10]  # 取 qx, qy, qz, qw
        quaternion /= np.linalg.norm(quaternion)  # 歸一化
        self.state[6:10] = quaternion

    def get_state(self):
        return self.state

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        self.declare_parameter('object')
        self.declare_parameter('process_variance')
        self.declare_parameter('measurement_variance')
        self.declare_parameter('initial_state')
        self.declare_parameter('initial_uncertainty')
        self.declare_parameter('csv_save')

        self.object_topic = self.get_parameter('object').get_parameter_value().string_value
        self.process_variance = self.get_parameter('process_variance').get_parameter_value().double_value
        self.measurement_variance = self.get_parameter('measurement_variance').get_parameter_value().double_value
        self.initial_state = self.get_parameter('initial_state').get_parameter_value().double_array_value
        self.initial_uncertainty = self.get_parameter('initial_uncertainty').get_parameter_value().double_value

        self.csv_save = self.get_parameter('csv_save').get_parameter_value().bool_value

        self.get_logger().info("object_topic: {}, type: {}".format(self.object_topic, type(self.object_topic)))
        self.get_logger().info("process_variance: {}, type: {}".format(self.process_variance, type(self.process_variance)))
        self.get_logger().info("measurement_variance: {}, type: {}".format(self.measurement_variance, type(self.measurement_variance)))
        self.get_logger().info("initial_uncertainty: {}, type: {}".format(self.initial_uncertainty, type(self.initial_uncertainty)))
        
        self.objectt_confidence = 0.0
        self.object_detection = False
        self.init_kalman_filter()

        # 訂閱來自傳感器的 Pose 信息
        self.object_pose_sub = self.create_subscription(Pose, self.object_topic, self.pose_callback, qos_profile=qos_profile_sensor_data)
        self.object_confidence_sub = self.create_subscription(Confidence, self.object_topic + "_confidence", self.cbObjectConfidence, qos_profile=qos_profile_sensor_data)

        # 發布經過卡爾曼濾波器的 Pose 結果
        self.object_filtered_pub = self.create_publisher(Pose, f"{self.object_topic}_filtered", 1)
        
        if self.csv_save:
            # 创建并打开CSV文件
            TI = datetime.now().strftime('%Y-%m-%d %H-%M')
            self.csv_file = open(f'./{self.object_topic}_filtered_data{TI}.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # 写入CSV文件头
            self.csv_writer.writerow(['Time', 'Raw_Position_X', 'Raw_Position_Y', 'Raw_Position_Z', 'Raw_Orientation_X','Raw_Orientation_Y','Raw_Orientation_Z','Raw_Orientation_W', 
                                    'Filtered_Position_X', 'Filtered_Position_Y', 'Filtered_Position_Z', 'Filtered_Orientation_X', 'Filtered_Orientation_Y', 'Filtered_Orientation_Z', 'Filtered_Orientation_W'])

    def cbObjectConfidence(self, msg):
        self.object_confidence = msg.object_confidence
        self.object_detection = msg.model_detection

    def init_kalman_filter(self):
        # 初始化卡爾曼濾波器
        self.kf = KalmanFilter(
            self.process_variance, 
            self.measurement_variance, 
            self.initial_state, 
            self.initial_uncertainty
        )

    def pose_callback(self, msg):
        # 获取当前时间戳
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        # 收到的測量數據
        measurement = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        # 卡爾曼濾波器的預測和更新
        self.kf.predict()
        self.kf.update(measurement)

        # 得到經過濾波的狀態
        filtered_state = self.kf.get_state()

        # 構建過濾後的 Pose 消息
        filtered_pose = Pose()
        filtered_pose.position.x = filtered_state[0]  # x
        filtered_pose.position.y = filtered_state[2]  # y
        filtered_pose.position.z = filtered_state[4]  # z
        filtered_pose.orientation.x = filtered_state[6]
        filtered_pose.orientation.y = filtered_state[7]
        filtered_pose.orientation.z = filtered_state[8]
        filtered_pose.orientation.w = filtered_state[9]

        # 發布過濾後的 Pose
        self.object_filtered_pub.publish(filtered_pose)
        # self.get_logger().info(f'Published filtered pose: {filtered_pose}')

        if self.csv_save:
            # 写入CSV文件，包括时间戳、原始測量數據和經過濾波的數據
            self.csv_writer.writerow([timestamp, msg.position.x, msg.position.y, msg.position.z,msg.orientation.x,msg.orientation.y, msg.orientation.z,msg.orientation.w, 
                                        filtered_state[0], filtered_state[2], filtered_state[4], filtered_state[6], filtered_state[7], filtered_state[8],filtered_state[9]])

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
