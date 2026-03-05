#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import math
from forklift_msg.msg import meteorcar
from geometry_msgs.msg import PoseStamped, Twist, Pose

import sys
import os
script_dir = os.path.dirname( __file__ )
mymodule_dir = os.path.join( script_dir, '..', 'scripts' )
sys.path.append( mymodule_dir )
from PBVS_minicar import PBVS
from ekf import KalmanFilter

class Subscriber():
    def __init__(self):
        odom = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        tag_detections_up = rospy.get_param(rospy.get_name() + "/tag_detections_up", "/tag_detections_up")
        tag_detections_down = rospy.get_param(rospy.get_name() + "/tag_detections_down", "/tag_detections_down")
        pub_name = rospy.get_param(rospy.get_name() + "/pub_name", "/robot_pose")
        forkpos = rospy.get_param(rospy.get_name() + "/forkpos", "/forkpos")
        self.sub_info_marker = rospy.Subscriber(tag_detections_up, AprilTagDetectionArray, self.cbGetMarker_up, queue_size = 1)
        self.sub_info_marker = rospy.Subscriber(tag_detections_down, AprilTagDetectionArray, self.cbGetMarker_down, queue_size = 1)
        self.sub_map_robot = rospy.Subscriber(pub_name, Pose, self.cbGetRobotMap, queue_size = 1)
        self.sub_odom_robot = rospy.Subscriber(odom, Odometry, self.cbGetRobotOdom, queue_size = 1)
        self.sub_forwardbackpostion = rospy.Subscriber(forkpos, meteorcar, self.cbGetforkpos, queue_size = 1)
        self.ekf_theta = KalmanFilter()
        self.init_parame()

    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        self.robot_map_pose_x = 0.0
        self.robot_map_pose_y = 0.0
        self.robot_map_theta = 0.0
        self.previous_robot_map_theta = 0.0
        self.total_robot_map_theta = 0.0
        self.is_map_pose_received = False # 標記是否已收到 MapToBaselink 的資料
        # AprilTag_param
        self.updown = False
        self.offset_x = 0.0
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Forklift_param
        self.updownposition = 0.0
        #ekf
        self.ekf_theta.init(1,1,5)
    def __del__(self):
        self.window.destroy()

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
    def SpinOnce_fork(self):
        return self.updownposition

    def cbGetMarker_up(self, msg):
        try:
            if self.updown == True:
                # print("up tag")
                marker_msg = msg.detections[0].pose.pose.pose
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                # theta = self.ekf_theta.update(theta)
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x + self.offset_x
                self.marker_2d_theta = -theta
            else:
                pass
        except:
            pass

    def cbGetMarker_down(self, msg):
        try:
            if self.updown == False:
                # print("down tag")
                marker_msg = msg.detections[0].pose.pose.pose
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf.transformations.euler_from_quaternion(quaternion)[1]
                theta = self.ekf_theta.update(theta)
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x + self.offset_x
                self.marker_2d_theta = -theta

            else:
                pass
        except:
            pass

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

    def cbGetRobotMap(self, msg):
        """
        接收 MapToBaselink (/robot_pose) 的資料，轉換並儲存為 2D 座標與角度
        """
        try:
            self.is_map_pose_received = True # 防呆旗標
            
            self.robot_map_pose_x = msg.position.x
            self.robot_map_pose_y = msg.position.y
            
            # 從四元數轉換為 Yaw 角 (Theta)
            quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[2]
            
            # 保持角度在 0 ~ 2*pi 之間
            if theta < 0:
                theta = theta + math.pi * 2
            if theta > math.pi * 2:
                theta = theta - math.pi * 2
                
            self.robot_map_theta = theta
            
            # 計算連續旋轉的總角度 (處理跨越 360 度的跳變)
            if (self.robot_map_theta - self.previous_robot_map_theta) > math.pi:
                d_theta = (self.robot_map_theta - self.previous_robot_map_theta) - 2 * math.pi
            elif (self.robot_map_theta - self.previous_robot_map_theta) < -math.pi:
                d_theta = (self.robot_map_theta - self.previous_robot_map_theta) + 2 * math.pi
            else:
                d_theta = (self.robot_map_theta - self.previous_robot_map_theta)

            self.total_robot_map_theta = self.total_robot_map_theta + d_theta
            self.previous_robot_map_theta = self.robot_map_theta
            
            # 將最終用來計算的角度設為累加後的總角度
            self.robot_map_theta = self.total_robot_map_theta
            
        except Exception as e:
            self.is_map_pose_received = False
            rospy.logwarn(f"Error in cbGetRobotMap: {e}")

    def cbGetforkpos(self, msg):
        self.updownposition = msg.fork_position

 
class PBVSAction():
    def __init__(self, name):
        self.subscriber = Subscriber()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.PBVSAction, execute_cb=self.execute_cb, auto_start = False)
        self._result = forklift_server.msg.PBVSResult()
        self._as.start()

    def execute_cb(self, msg):
        rospy.loginfo('PBVS receive command : %s' % (msg))
        
        self.PBVS = PBVS(self._as, self.subscriber, msg)
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'PBVS Succeeded'
        self.subscriber.updown = True
        self._as.set_succeeded(self._result)
        self.PBVS = None


if __name__ == '__main__':
    rospy.init_node('PBVS_server')
    rospy.logwarn(rospy.get_name() + 'start')
    server = PBVSAction(rospy.get_name())
    rospy.spin()
    