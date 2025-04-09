#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import forklift_server.msg
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
from visp_megapose.msg import Confidence

import sys
import os
script_dir = os.path.dirname( __file__ )
mymodule_dir = os.path.join( script_dir, '..', 'scripts' )
sys.path.append( mymodule_dir )
from PBVS_differential import PBVS

from dataclasses import dataclass
@dataclass
class DetectionConfidence:
    pose_confidence: float
    pose_detection: bool

class Subscriber():
    def __init__(self):
        self.get_parameters()
        self.init_parame()
        self.create_subscriber_publisher()
        self.fnDetectionAllowed(False, 0.0)

    def get_parameters(self):
        # Subscriber Topic setting
        self.odom_topic = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        self.pose_topic = rospy.get_param(rospy.get_name() + "/pose_topic", "/oilpalm")
        self.object_filter = rospy.get_param(rospy.get_name() + "/object_filter", True)
        self.confidence_minimum = rospy.get_param(rospy.get_name() + "/confidence_minimum", 0.5)

        rospy.loginfo("Get subscriber topic parameter")
        rospy.loginfo("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        rospy.loginfo("pose_topic: {}, type: {}".format(self.pose_topic, type(self.pose_topic)))
        rospy.loginfo("object_filter: {}, type: {}".format(self.object_filter, type(self.object_filter)))
        rospy.loginfo("confidence_minimum: {}, type: {}".format(self.confidence_minimum, type(self.confidence_minimum)))

        # camera parking setting
        self.camera_tag_offset_x = rospy.get_param(rospy.get_name() + "/camera_tag_offset_x", 0.0)
        # self.camera_parking_fork_init = rospy.get_param(rospy.get_name() + "/camera_parking_fork_init", 0.0)
        # self.camera_ChangingDirection_threshold = rospy.get_param(rospy.get_name() + "/camera_ChangingDirection_threshold", 0.0)
        self.camera_desired_dist_threshold = rospy.get_param(rospy.get_name() + "/camera_desired_dist_threshold", 0.0)
        
<<<<<<< HEAD
        self.camera_horizon_alignment_threshold = rospy.get_param(rospy.get_name() + "/camera_horizon_alignment_threshold", 0.0)
=======
        self.camera_parking_stop_threshold = rospy.get_param(rospy.get_name() + "/camera_parking_stop_threshold", 0.0)
>>>>>>> 76ed4b50bc3205cd5ad073fc7cf286b6c538684a
        # self.camera_Changingtheta_threshold = rospy.get_param(rospy.get_name() + "/camera_Changingtheta_threshold", 0.0)
        # self.camera_decide_distance = rospy.get_param(rospy.get_name() + "/camera_decide_distance", 0.0)
        # self.camera_back_distance = rospy.get_param(rospy.get_name() + "/camera_back_distance", 0.0)

        rospy.loginfo("Get camera parking parameter")
        rospy.loginfo("camera_tag_offset_x: {}, type: {}".format(self.camera_tag_offset_x, type(self.camera_tag_offset_x)))
        # rospy.loginfo("camera_parking_fork_init: {}, type: {}".format(self.camera_parking_fork_init, type(self.camera_parking_fork_init)))
        # rospy.loginfo("camera_ChangingDirection_threshold: {}, type: {}".format(self.camera_ChangingDirection_threshold, type(self.camera_ChangingDirection_threshold)))
        rospy.loginfo("camera_desired_dist_threshold: {}, type: {}".format(self.camera_desired_dist_threshold, type(self.camera_desired_dist_threshold)))        
<<<<<<< HEAD
        rospy.loginfo("camera_horizon_alignment_threshold: {}, type: {}".format(self.camera_horizon_alignment_threshold, type(self.camera_horizon_alignment_threshold)))
=======
        rospy.loginfo("camera_parking_stop_threshold: {}, type: {}".format(self.camera_parking_stop_threshold, type(self.camera_parking_stop_threshold)))
>>>>>>> 76ed4b50bc3205cd5ad073fc7cf286b6c538684a
        # rospy.loginfo("camera_Changingtheta_threshold: {}, type: {}".format(self.camera_Changingtheta_threshold, type(self.camera_Changingtheta_threshold)))
        # rospy.loginfo("camera_decide_distance: {}, type: {}".format(self.camera_decide_distance, type(self.camera_decide_distance)))
        # rospy.loginfo("camera_back_distance: {}, type: {}".format(self.camera_back_distance, type(self.camera_back_distance)))

    def init_parame(self):
        # Odometry_param
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_param
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Forklift_param
        self.updownposition = 0.0
        # confidence_param
        self.sub_detectionConfidence = DetectionConfidence(
            pose_confidence = 0.0,
            pose_detection = False
        )
    
    def create_subscriber_publisher(self):
        if(self.object_filter):
            object_pose = self.pose_topic + "_filter"
        else:
            object_pose = self.pose_topic
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.cbGetOdom, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1, latch=True)
        self.object_pose_sub = rospy.Subscriber(object_pose, Pose, self.cbGetObject, queue_size = 1)
        self.object_pose_confidence_sub = rospy.Subscriber(self.pose_topic + "_confidence", Confidence, self.cbGetObjectConfidence, queue_size = 1)
        
<<<<<<< HEAD
        #pin
        self.pose_detection_pub = rospy.Publisher(self.pose_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
        # self.pallet_detection_pub = rospy.Publisher(self.pallet_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
        # self.shelf_detection_pub = rospy.Publisher(self.shelf_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
=======
        self.pallet_detection_pub = rospy.Publisher(self.pallet_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
        self.shelf_detection_pub = rospy.Publisher(self.shelf_topic + "_detection", forklift_server.msg.Detection, queue_size = 1, latch=True)
>>>>>>> 76ed4b50bc3205cd5ad073fc7cf286b6c538684a
    
    def fnDetectionAllowed(self, pose_detection, layer):
        pose_msg = forklift_server.msg.Detection()
        pose_msg.detection_allowed = pose_detection
        pose_msg.layer = layer
<<<<<<< HEAD
        # self.shelf_detection_pub.publish(pose_msg)
        self.pose_detection_pub.publish(pose_msg)   #pin
=======
        self.shelf_detection_pub.publish(pose_msg)
>>>>>>> 76ed4b50bc3205cd5ad073fc7cf286b6c538684a

        rospy.sleep(0.2)
        # rospy.loginfo("pose_msg = {}, pallet_msg = {}".format(pose_msg, pallet_msg))

    def __del__(self):
        self.window.destroy()

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
    
    def SpinOnce_confidence(self):
        return self.sub_detectionConfidence

    def cbGetObject(self, msg):
        try:
            marker_msg = msg
            quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
            theta = tf.transformations.euler_from_quaternion(quaternion)[1]
            self.marker_2d_pose_x = -marker_msg.position.z
            self.marker_2d_pose_y = marker_msg.position.x + self.camera_tag_offset_x
            self.marker_2d_theta = -theta
            # rospy.loginfo("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
        except:
            pass

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

    def cbGetObjectConfidence(self, msg):
        self.sub_detectionConfidence.pose_confidence = msg.object_confidence
        self.sub_detectionConfidence.pose_detection = msg.model_detection
 
class PBVSAction():
    def __init__(self, name):
        self.subscriber = Subscriber()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.PBVSMegaposeAction, execute_cb=self.execute_callback, auto_start = False)
        self._result = forklift_server.msg.PBVSResult()
        self._as.start()

    def execute_callback(self, msg):
        # rospy.loginfo('Received goal: Command={}, layer_dist={}'.format(self.command, self.layer_dist))
        rospy.logwarn('PBVS receive command : %s' % (msg))
        self.PBVS = PBVS(self._as, self.subscriber, msg)

        if(msg.command == "parking_camera"):
            self.PBVS.parking_camera()
        elif(msg.command == "odom_front"):
            self.PBVS.odom_front()
        elif(msg.command == "odom_turn"):
            self.PBVS.odom_turn()
        else:
            rospy.logwarn("Unknown command")
            self._result.result = 'fail'
            self._as.set_aborted(self._result)
            return
        
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'PBVS Succeeded'
        self._as.set_succeeded(self._result)
        self.PBVS = None


if __name__ == '__main__':
    rospy.init_node('PBVS_server')
    rospy.logwarn(rospy.get_name() + 'start')
    server = PBVSAction(rospy.get_name())
    rospy.spin()
    