#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import math
from forklift_msg.msg import meteorcar
from ekf import KalmanFilter
import tkinter as tk

import csv
import os
from datetime import datetime

class Subscriber():
    def __init__(self):
        odom = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        tag_detections_up = rospy.get_param(rospy.get_name() + "/tag_detections_up", "/tag_detections_up")
        tag_detections_down = rospy.get_param(rospy.get_name() + "/tag_detections_down", "/tag_detections_down")
        forkpos = rospy.get_param(rospy.get_name() + "/forkpos", "/forkpos")

        self.fileEnble = rospy.get_param(rospy.get_name() + "/fileEnble", True)
        filename = rospy.get_param(rospy.get_name() + "/filename", "shelf_pallet_data.csv")

        if self.fileEnble:
            # 設定 CSV 檔案名稱
            self.csv_filename = os.path.join(os.path.expanduser("~"), filename)
            # 如果檔案不存在，則建立並寫入標題
            if not os.path.exists(self.csv_filename):
                with open(self.csv_filename, mode='w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Timestamp", "shelf_2d_theta", "shelf_2d_pose_x", "shelf_2d_pose_y",
                                    "pallet_2d_theta", "pallet_2d_pose_x", "pallet_2d_pose_y"])
            rospy.loginfo(f"Data will be logged to {self.csv_filename}")

        self.sub_info_marker = rospy.Subscriber(tag_detections_up, AprilTagDetectionArray, self.cbGetMarker_up, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber(tag_detections_down, AprilTagDetectionArray, self.cbGetMarker_down, queue_size = 1)
        self.sub_odom_robot = rospy.Subscriber(odom, Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.sub_forwardbackpostion = rospy.Subscriber(forkpos, meteorcar, self.cbGetforkpos, queue_size = 1)
        self.ekf_theta = KalmanFilter()
        self.init_parame()
        self.windows()

    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_param
        self.updown = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Forklift_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        #ekf
        self.ekf_theta.init(1,1,5)

    def cbGetMarker_up(self, msg):
        try:
            # print("up tag")
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            # theta = self.ekf_theta.update(theta)
            self.marker_2d_pose_x = -marker_msg.position.z
            self.marker_2d_pose_y = marker_msg.position.x
            self.marker_2d_theta = -theta

        except:
            pass

    def cbGetMarker_down(self, msg):
        try:
            # print("down tag")
            marker_msg = msg.detections[0].pose.pose.pose
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            # theta = self.ekf_theta.update(theta)
            self.marker_2d_pose_x = -marker_msg.position.z
            self.marker_2d_pose_y = marker_msg.position.x
            self.marker_2d_theta = -theta
        except:
            pass

    def log_data(self):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, self.marker_2d_theta, self.marker_2d_pose_x, self.marker_2d_pose_y])
        
        # rospy.loginfo(f"Data logged at {timestamp}")

    def cbGetRobotOdom(self, msg):
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

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta = self.total_robot_2d_theta + d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta

    def cbGetforkpos(self, msg):
        # self.forwardbackpostion = msg.forwardbackpostion
        self.updownposition = msg.fork_position


    def windows(self):
        self.window = tk.Tk()
        self.window.geometry('220x170+1700+560')
        self.labels = {
            'robot_2d_pose_x': [0, 0],
            'robot_2d_pose_y': [0, 20],
            'robot_2d_theta': [0, 40],
            'marker_2d_pose_x': [0, 100],
            'marker_2d_pose_y': [0, 120],
            'marker_2d_theta': [0, 140],
            'fork_updown_position': [0, 160],
            'fork_forwardback_position': [0, 180]
        }
        for key, value in self.labels.items():
            self.labels[key] = [tk.Label(self.window, text=f"{key.replace('_', ' ').title()}: "), tk.Label(self.window, text="")]
            self.labels[key][0].place(x=value[0], y=value[1])
            self.labels[key][1].place(x=190, y=value[1])
        while not rospy.is_shutdown():
            self.update_window()
            if self.fileEnble:
                self.log_data()
            self.window.update()
            rospy.sleep(0.05) # Set the desired update rate
        self.window.destroy()
        # self.update_window()
        # self.window.mainloop()

    def update_window(self):
        update_values = {
            'robot_2d_pose_x': self.robot_2d_pose_x,
            'robot_2d_pose_y': self.robot_2d_pose_y,
            'robot_2d_theta': math.degrees(self.robot_2d_theta),
            'marker_2d_pose_x': self.marker_2d_pose_x,
            'marker_2d_pose_y': self.marker_2d_pose_y,
            'marker_2d_theta': math.degrees(self.marker_2d_theta),
            'fork_updown_position': self.updownposition,
            'fork_forwardback_position': self.forwardbackpostion
        }
        for key, value in update_values.items():
            self.labels[key][1].configure(text=value)

        # self.window.after(50, self.update_window)

if __name__ == '__main__':
    rospy.init_node('gui')
    subscriber = Subscriber()
    rospy.spin()
