#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import tf
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import circular_server.msg
import math
from forklift_msg.msg import Detection, Confidence
from circular_saw_controller.msg import CircularSawCmd, CircularSawState

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
    state: str
    tag: str

class Subscriber():
    def __init__(self):
        self.get_parameters()
        self.init_parame()
        self.create_subscriber_publisher()
        self.fnDetectionAllowed(False, "score")

    def get_parameters(self):
        # Subscriber Topic setting
        self.odom_topic = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        self.pose_topic = rospy.get_param(rospy.get_name() + "/pose_topic", "/oilpalm")
        self.object_filter = rospy.get_param(rospy.get_name() + "/object_filter", True)
        self.confidence_minimum = rospy.get_param(rospy.get_name() + "/confidence_minimum", 0.5)
        self.arm_status_topic = rospy.get_param(rospy.get_name() + "/arm_status_topic", "/circular_saw")
        self.arm_control_topic = rospy.get_param(rospy.get_name() + "/arm_control_topic", "/cmd_circular_saw")

        rospy.loginfo("Get subscriber topic parameter")
        rospy.loginfo("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        rospy.loginfo("pose_topic: {}, type: {}".format(self.pose_topic, type(self.pose_topic)))
        rospy.loginfo("object_filter: {}, type: {}".format(self.object_filter, type(self.object_filter)))
        rospy.loginfo("confidence_minimum: {}, type: {}".format(self.confidence_minimum, type(self.confidence_minimum)))
        rospy.loginfo("arm_status_topic: {}, type: {}".format(self.arm_status_topic, type(self.arm_status_topic)))
        rospy.loginfo("arm_control_topic: {}, type: {}".format(self.arm_control_topic, type(self.arm_control_topic)))

        # camera parking setting
        self.camera_tag_offset_x = rospy.get_param(rospy.get_name() + "/camera_tag_offset_x", 0.0)
        self.camera_tag_offset_z = rospy.get_param(rospy.get_name() + "/camera_tag_offset_z", 0.0)
        self.camera_desired_dist_threshold = rospy.get_param(rospy.get_name() + "/camera_desired_dist_threshold", 0.0)
        self.camera_horizon_alignment_threshold = rospy.get_param(rospy.get_name() + "/camera_horizon_alignment_threshold", 0.0)
        self.camera_side = rospy.get_param(rospy.get_name() + "/camera_side", "left").strip().lower()
        if self.camera_side not in ("left", "right"):
            rospy.logwarn("camera_side should be 'left' or 'right'; got %s -> default to 'left'", self.camera_side)
            self.camera_side = "left"
        self.turn_dir_deg = -90 if self.camera_side == "right" else +90

        rospy.loginfo("Get camera parking parameter")
        rospy.loginfo("camera_side: {}, type: {}".format(self.camera_side, type(self.camera_side)))
        rospy.loginfo("camera_side=%s, first turn_dir_deg=%d", self.camera_side, self.turn_dir_deg)
        rospy.loginfo("camera_tag_offset_x: {}, type: {}".format(self.camera_tag_offset_x, type(self.camera_tag_offset_x)))
        rospy.loginfo("camera_tag_offset_z: {}, type: {}".format(self.camera_tag_offset_z, type(self.camera_tag_offset_z)))
        rospy.loginfo("camera_desired_dist_threshold: {}, type: {}".format(self.camera_desired_dist_threshold, type(self.camera_desired_dist_threshold)))
        rospy.loginfo("camera_horizon_alignment_threshold: {}, type: {}".format(self.camera_horizon_alignment_threshold, type(self.camera_horizon_alignment_threshold)))

        # Cut pliers (arm) control settings
        self.spin_forward_comp = rospy.get_param(rospy.get_name() + "/spin_forward_comp", 0.0)

        # === Anti-overshoot 參數（mm 單位）===
        self.marker_to_mm     = rospy.get_param(rospy.get_name() + "/marker_to_mm", 1000.0)
        self.z_lead_near_mm   = rospy.get_param(rospy.get_name() + "/z_lead_near_mm", 6.0)
        self.z_lead_far_mm    = rospy.get_param(rospy.get_name() + "/z_lead_far_mm", 12.0)
        self.z_lead_switch_mm = rospy.get_param(rospy.get_name() + "/z_lead_switch_mm", 25.0)
        self.z_step_max_mm    = rospy.get_param(rospy.get_name() + "/z_step_max_mm", 30.0)

        self.x_lead_near_mm   = rospy.get_param(rospy.get_name() + "/x_lead_near_mm", 10.0)
        self.x_lead_far_mm    = rospy.get_param(rospy.get_name() + "/x_lead_far_mm", 20.0)
        self.x_lead_switch_mm = rospy.get_param(rospy.get_name() + "/x_lead_switch_mm", 40.0)
        self.x_step_max_mm    = rospy.get_param(rospy.get_name() + "/x_step_max_mm", 50.0)

        self.zx_stable_frames = rospy.get_param(rospy.get_name() + "/zx_stable_frames", 5)

        # 安全行程（mm）
        self.circular_saw_min_height = rospy.get_param(rospy.get_name() + "/circular_saw_min_height", 0.0)
        self.circular_saw_max_height = rospy.get_param(rospy.get_name() + "/circular_saw_max_height", 0.0)
        self.circular_saw_min_length = rospy.get_param(rospy.get_name() + "/circular_saw_min_length", 0.0)
        self.circular_saw_max_length = rospy.get_param(rospy.get_name() + "/circular_saw_max_length", 0.0)
        self.circular_saw_min_angle = rospy.get_param(rospy.get_name() + "/circular_saw_min_angle", 0.0)
        self.circular_saw_max_angle = rospy.get_param(rospy.get_name() + "/circular_saw_max_angle", 90.0)
        self.circular_saw_max_saw_speed = rospy.get_param(rospy.get_name() + "/circular_saw_max_saw_speed", 6000.0)

        # Z 允許帶 / 自適應步長
        self.z_tolerance_mm = rospy.get_param(rospy.get_name() + "/z_tolerance_mm", 0.0)
        self.z_step_min_mm = rospy.get_param(rospy.get_name() + "/z_step_min_mm", 3.0)
        self.z_step_max_mm = rospy.get_param(rospy.get_name() + "/z_step_max_mm", 20.0)
        self.z_step_gain = rospy.get_param(rospy.get_name() + "/z_step_gain", 0.7)

        # X 目標 / 自適應步長
        self.x_tolerance_mm = rospy.get_param(rospy.get_name() + "/x_tolerance_mm", 0.0)
        self.x_step_min_mm = rospy.get_param(rospy.get_name() + "/x_step_min_mm", 5.0)
        self.x_step_max_mm = rospy.get_param(rospy.get_name() + "/x_step_max_mm", 50.0)
        self.x_step_gain = rospy.get_param(rospy.get_name() + "/x_step_gain", 0.7)
        self.circular_saw_allow_retract = rospy.get_param(rospy.get_name() + "/circular_saw_allow_retract", True)
        self.x_circular_saw_target = rospy.get_param(rospy.get_name() + "/x_circular_saw_target", 0.0)
        self.circular_saw_blind_extend_length = rospy.get_param(rospy.get_name() + "/circular_saw_blind_extend_length", 0.0)
        self.circular_saw_home_length = rospy.get_param(rospy.get_name() + "/circular_saw_home_length", 0.0)
        self.circular_saw_home_height = rospy.get_param(rospy.get_name() + "/circular_saw_home_height", 0.0)

        rospy.loginfo("Get cut pliers (arm) control parameters")
        rospy.loginfo("spin_forward_comp: %s", self.spin_forward_comp)
        rospy.loginfo("marker_to_mm: %s", self.marker_to_mm)
        rospy.loginfo("z_lead_near_mm: %s", self.z_lead_near_mm)
        rospy.loginfo("z_lead_far_mm: %s", self.z_lead_far_mm)
        rospy.loginfo("z_lead_switch_mm: %s", self.z_lead_switch_mm)
        rospy.loginfo("z_step_max_mm: %s", self.z_step_max_mm)
        rospy.loginfo("x_lead_near_mm: %s", self.x_lead_near_mm)
        rospy.loginfo("x_lead_far_mm: %s", self.x_lead_far_mm)
        rospy.loginfo("x_lead_switch_mm: %s", self.x_lead_switch_mm)
        rospy.loginfo("x_step_max_mm: %s", self.x_step_max_mm)
        rospy.loginfo("zx_stable_frames: %s", self.zx_stable_frames)
        rospy.loginfo("circular_saw_min_height: %s", self.circular_saw_min_height)
        rospy.loginfo("circular_saw_max_height: %s", self.circular_saw_max_height)
        rospy.loginfo("circular_saw_min_length: %s", self.circular_saw_min_length)
        rospy.loginfo("circular_saw_max_length: %s", self.circular_saw_max_length)
        rospy.loginfo("circular_saw_min_angle: %s", self.circular_saw_min_angle)
        rospy.loginfo("circular_saw_max_angle: %s", self.circular_saw_max_angle)
        rospy.loginfo("circular_saw_max_saw_speed: %s", self.circular_saw_max_saw_speed)
        rospy.loginfo("z_tolerance_mm: %s", self.z_tolerance_mm)
        rospy.loginfo("z_step_min_mm: %s", self.z_step_min_mm)
        rospy.loginfo("z_step_max_mm: %s", self.z_step_max_mm)
        rospy.loginfo("z_step_gain: %s", self.z_step_gain)
        rospy.loginfo("x_tolerance_mm: %s", self.x_tolerance_mm)
        rospy.loginfo("x_step_min_mm: %s", self.x_step_min_mm)
        rospy.loginfo("x_step_max_mm: %s", self.x_step_max_mm)
        rospy.loginfo("x_step_gain: %s", self.x_step_gain)
        rospy.loginfo("circular_saw_allow_retract: %s", self.circular_saw_allow_retract)
        rospy.loginfo("x_circular_saw_target: %s", self.x_circular_saw_target)
        rospy.loginfo("circular_saw_blind_extend_length: %s", self.circular_saw_blind_extend_length)
        rospy.loginfo("circular_saw_home_length: %s", self.circular_saw_home_length)
        rospy.loginfo("circular_saw_home_height: %s", self.circular_saw_home_height)

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
        self.marker_2d_pose_z = 0.0
        self.marker_2d_theta = 0.0
        self.object_state = ""
        # confidence_param
        self.sub_detectionConfidence = DetectionConfidence(
            pose_confidence = 0.0,
            pose_detection = False,
            state = "",
            tag = ""
        )
        # arm_param
        self.circular_saw_state = None
        self.last_command = None  # 上次命令記錄
    
    def create_subscriber_publisher(self):
        if(self.object_filter):
            object_pose = self.pose_topic + "_filter"
        else:
            object_pose = self.pose_topic
        
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.cbGetOdom, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1, latch=True)
        self.object_pose_sub = rospy.Subscriber(object_pose, Pose, self.cbGetObject, queue_size = 1)
        self.object_pose_confidence_sub = rospy.Subscriber(self.pose_topic + "_confidence", Confidence, self.cbGetObjectConfidence, queue_size = 1)
        
        self.pose_detection_pub = rospy.Publisher(self.pose_topic + "_detection", Detection, queue_size = 1, latch=True)
        
        # 新增手臂相關的 Subscriber 和 Publisher
        self.arm_control_pub = rospy.Publisher(self.arm_control_topic, CircularSawCmd, queue_size=1)
        self.arm_status_sub = rospy.Subscriber(self.arm_status_topic, CircularSawState, self.arm_status_callback, queue_size=1)

    def fnDetectionAllowed(self, pose_detection, det_select_mode):
        pose_msg = Detection()
        pose_msg.detection_allowed = pose_detection
        pose_msg.det_select_mode = det_select_mode
        self.pose_detection_pub.publish(pose_msg)

        rospy.sleep(0.2)
        # rospy.loginfo("pose_msg = {}, pallet_msg = {}".format(pose_msg, pallet_msg))

    def arm_status_callback(self, msg):
        self.circular_saw_state = msg
    
    def cmd_circular_saw(self, msg):

        if self.circular_saw_state is None:
            rospy.logwarn("circular_saw_state尚未初始化")
            return
        
        new_cmd = {
            "x_pos": msg.x_pos,
            "z_pos": msg.z_pos,
            "angle": float(msg.angle),
            "x_speed": msg.x_speed,
            "z_speed": msg.z_speed,
            "saw_speed": msg.saw_speed,
            "stop": msg.stop,
        }

        if (self.last_command is not None and new_cmd == self.last_command):
            rospy.loginfo("指令未變化")
            return

        arm_cmd = CircularSawCmd()
        arm_cmd.x_pos = msg.x_pos
        arm_cmd.z_pos = msg.z_pos
        arm_cmd.angle = msg.angle
        arm_cmd.x_speed = msg.x_speed
        arm_cmd.z_speed = msg.z_speed
        arm_cmd.saw_speed = msg.saw_speed
        arm_cmd.stop = msg.stop

        self.last_command = new_cmd

        # 發布到 /cmd_circular_saw
        self.arm_control_pub.publish(arm_cmd)

        rospy.loginfo(
            f"發送指令:X={arm_cmd.x_pos} mm, Z={arm_cmd.z_pos} mm, angle={arm_cmd.angle:.1f} deg, Vx={arm_cmd.x_speed}, Vz={arm_cmd.z_speed}, Saw={arm_cmd.saw_speed}, stop={arm_cmd.stop}"
        )

    def __del__(self):
        self.window.destroy()

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z, self.marker_2d_theta
    
    def SpinOnce_confidence(self):
        return self.sub_detectionConfidence

    def cbGetObject(self, msg):
        px, py, pz = msg.position.x, msg.position.y, msg.position.z

        side_sign = -1 if self.camera_side == "right" else +1

        self.marker_2d_pose_x = +pz # 伸長：向前為正
        self.marker_2d_pose_y = side_sign * px + self.camera_tag_offset_x  # 橫向（視相機左右翻轉）
        self.marker_2d_pose_z = -py + self.camera_tag_offset_z # 高度：向上為正

        self.marker_2d_theta = math.atan2(self.marker_2d_pose_y, self.marker_2d_pose_x)
        
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
        self.sub_detectionConfidence.pose_confidence = msg.object_IoU
        self.sub_detectionConfidence.pose_detection = msg.object_detection
        self.sub_detectionConfidence.state = msg.state

        state = msg.state.split(':')
        self.sub_detectionConfidence.tag = state[0]
        self.sub_detectionConfidence.state = state[1]
 
class PBVSAction():
    def __init__(self, name):
        self.subscriber = Subscriber()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, circular_server.msg.PBVSFoundationposeAction, execute_cb=self.execute_callback, auto_start = False)
        self._result = circular_server.msg.PBVSFoundationposeResult()
        self._as.start()

    def execute_callback(self, msg):
        rospy.logwarn('PBVS receive command : %s' % (msg))
        self.PBVS = PBVS(self._as, self.subscriber, msg)

        if(msg.command == "fruit_docking"):
            self.PBVS.fruit_docking()
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
    