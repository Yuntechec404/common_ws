#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from ekf import KalmanFilter
import tkinter as tk
import circular_server.msg
from forklift_msg.msg import Detection, Confidence

import csv
import os
from datetime import datetime
from dataclasses import dataclass

@dataclass
class DetectionConfidence:
    pose_confidence: float
    pose_detection: bool
    state: str
    tag: str

class Subscriber():
    def __init__(self):
        self.sub_detectionConfidence = DetectionConfidence(
            pose_confidence = 0.0,
            pose_detection = False,
            state = "",
            tag = ""
        )
        odom_topic = rospy.get_param(rospy.get_name() + "/odom_topic", "/odom")
        pose_topic = rospy.get_param(rospy.get_name() + "/pose_topic", "/shelf")
        self.camera_tag_offset_x = rospy.get_param(rospy.get_name() + "/offset_x", 0.0)
        self.camera_tag_offset_z = rospy.get_param(rospy.get_name() + "/offset_z", 0.0)
        self.fileEnble = rospy.get_param(rospy.get_name() + "/fileEnble", True)
        filename = rospy.get_param(rospy.get_name() + "/filename", "shelf_pallet_data.csv")

        self.camera_side = rospy.get_param(rospy.get_name() + "/camera_side", "left").strip().lower()
        if self.camera_side not in ("left", "right"):
            rospy.logwarn("camera_side should be 'left' or 'right'; got %s -> default to 'left'", self.camera_side)
            self.camera_side = "left"
        self.turn_dir_deg = -90 if self.camera_side == "right" else +90

        if self.fileEnble:
            # 設定 CSV 檔案名稱
            self.csv_filename = os.path.join(os.path.expanduser("~"), filename)
            # 如果檔案不存在，則建立並寫入標題
            if not os.path.exists(self.csv_filename):
                with open(self.csv_filename, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Timestamp", "marker_2d_pose_x", "marker_2d_pose_y", "marker_2d_pose_z",
                                    "marker_2d_theta"])
            rospy.loginfo(f"Data will be logged to {self.csv_filename}")

        rospy.loginfo("odom_topic: %s", odom_topic)
        rospy.loginfo("pose_topic: %s", pose_topic)
        rospy.loginfo("offset_x: %s", self.camera_tag_offset_x)
        rospy.loginfo("offset_z: %s", self.camera_tag_offset_z)

        self.sub_pose_topic = rospy.Subscriber(pose_topic, Confidence, self.cbGetObjectConfidence, queue_size = 1)
        self.sub_odom_robot = rospy.Subscriber(odom_topic, Odometry, self.cbGetOdom, queue_size = 1)
        self.ekf_theta = KalmanFilter()
        self.init_parame()
        self.windows()

    def init_parame(self):
        # Odometry_param
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        # AprilTag_param
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_pose_z = 0.0
        self.marker_2d_theta = 0.0
        self.green_line_angle_rad = 0.0

        #ekf
        self.ekf_theta.init(1,1,5)

    def cbGetObjectConfidence(self, msg):
        self.sub_detectionConfidence.pose_confidence = msg.object_IoU
        self.sub_detectionConfidence.pose_detection = msg.object_detection
        self.sub_detectionConfidence.state = msg.state

        state = msg.state.split(':')
        self.sub_detectionConfidence.tag = state[0]
        self.sub_detectionConfidence.state = state[1]

        px, py, pz = msg.position.x, msg.position.y, msg.position.z

        side_sign = -1 if self.camera_side == "right" else +1
        self.marker_2d_pose_x = +pz  
        self.marker_2d_pose_y = side_sign * px + self.camera_tag_offset_x
        self.marker_2d_pose_z = -py + self.camera_tag_offset_z
        self.marker_2d_theta = math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        
        matrix = tf.transformations.quaternion_matrix([qx, qy, qz, qw])
        
        vec_x = matrix[0, 1]  # 畫面左右的偏量 (正 = 右)
        vec_y = matrix[1, 1]  # 畫面上下的偏量 (正 = 下，因為相機Y軸朝下)
        
        # 如果 vec_y < 0，代表這根綠色箭頭在畫面上是「指向上方」的 (例如 168.1 度)
        if vec_y < 0:
            vec_x = -vec_x  # 把向左的箭頭翻轉成向右
            vec_y = -vec_y  # 把向上的箭頭翻轉成向下
            # 翻轉後，原本的 168.1 度，就會瞬間變成 -11.9 度！

        # 以「垂直向下」為 0 度基準！
        self.green_line_angle_rad = math.atan2(vec_x, vec_y)

    def log_data(self):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z,
                             self.marker_2d_theta])

    def cbGetOdom(self, msg):
        if self.is_odom_received == False:
            self.is_odom_received = True 

        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta = theta + math.pi * 2
        if theta > math.pi * 2:
            theta = theta - math.pi * 2

        self.robot_2d_pose_x = msg.pose.pose.position.x
        self.robot_2d_pose_y = msg.pose.pose.position.y
        self.robot_2d_theta = theta

        d_theta = self.robot_2d_theta - self.previous_robot_2d_theta
        if d_theta > math.pi:
            d_theta -= 2 * math.pi
        elif d_theta < -math.pi:
            d_theta += 2 * math.pi

        self.total_robot_2d_theta += d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta

    def windows(self):
        self.window = tk.Tk()
        # 把視窗拉高一點點，以便容納新增加的 Label
        self.window.geometry('800x150')
        self.labels = {
            'robot_pose':  [0,  0],
            'object_pose': [0, 40],
            'object_rot':  [0, 80],
        }
        
        for key, value in self.labels.items():
            if key == 'robot_pose':
                title = "Robot Pose (X, Y, Z, Angle):"
            elif key == 'object_pose':
                title = "Object Pose (X, Y, Z, Angle):"
            elif key == 'object_rot':
                title = "Object Rotation (Roll, Pitch, Yaw):"
            else:
                title = f"{key}:"

            self.labels[key] = [
                tk.Label(self.window, text=title),
                tk.Label(self.window, text="")
            ]
            self.labels[key][0].place(x=value[0], y=value[1])
            self.labels[key][1].place(x=260, y=value[1])

        while not rospy.is_shutdown():
            self.update_window()
            if self.fileEnble:
                self.log_data()
            self.window.update()
        self.window.destroy()

    def update_window(self):
        robot_x = self.robot_2d_pose_x
        robot_y = self.robot_2d_pose_y
        robot_z = 0.0
        robot_deg = math.degrees(self.robot_2d_theta)

        object_x = self.marker_2d_pose_x
        object_y = self.marker_2d_pose_y
        object_z = self.marker_2d_pose_z
        object_deg = self.marker_2d_theta
        
        green_line_deg = math.degrees(getattr(self, "green_line_angle_rad", 0.0))
    
        update_values = {
            'robot_pose':  f"X={robot_x:.3f},  Y={robot_y:.3f},  Z={robot_z:.3f},  θ={robot_deg:.1f}°",
            'object_pose': f"X={object_x:.3f}, Y={object_y:.3f}, Z={object_z:.3f}, θ={object_deg:.1f}°",
            'object_rot':  f"Green Axis (Saw Tilt) = {green_line_deg:.1f}°",
        }
        
        for key, value in update_values.items():
            self.labels[key][1].configure(text=value)

        # self.window.after(50, self.update_window)

if __name__ == '__main__':
    rospy.init_node('gui')
    subscriber = Subscriber()
    rospy.spin()
