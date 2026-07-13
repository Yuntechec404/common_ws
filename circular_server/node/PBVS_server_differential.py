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
from std_msgs.msg import Bool

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
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.create_subscriber_publisher()
        self.harvest_done_pub.publish(Bool(data=False))

    def get_parameters(self):
        # Subscriber Topic setting
        self.odom_topic = rospy.get_param(rospy.get_name() + "/odom", "/odom")
        self.pose_topic = rospy.get_param(rospy.get_name() + "/pose_topic", "/oilpalm")
        self.confidence_minimum = rospy.get_param(rospy.get_name() + "/confidence_minimum", 0.5)
        self.arm_status_topic = rospy.get_param(rospy.get_name() + "/arm_status_topic", "/circular_saw")
        self.arm_control_topic = rospy.get_param(rospy.get_name() + "/arm_control_topic", "/cmd_circular_saw")
        # Harvest done signal (separate from pause/resume)
        self.harvest_done_topic = rospy.get_param(rospy.get_name() + "/harvest_done_topic", self.pose_topic + "_harvest_done")
        self.pub_name = rospy.get_param(rospy.get_name() + "/pub_name", "/robot_pose")

        self.camera_base_x = rospy.get_param(rospy.get_name() + "/camera_base_x", 0.3)
        self.camera_base_y = rospy.get_param(rospy.get_name() + "/camera_base_y", 0.1)
        self.camera_base_z = rospy.get_param(rospy.get_name() + "/camera_base_z", -0.05)
        self.camera_base_yaw = rospy.get_param(rospy.get_name() + "/camera_base_yaw", -1.57)
        self.camera_base_pitch = rospy.get_param(rospy.get_name() + "/camera_base_pitch", 0.0)
        self.camera_base_roll = rospy.get_param(rospy.get_name() + "/camera_base_roll", -1.57)

        # PBVS TF frames:
        #   saw_cut_point -> oilpalm / stem
        # 兩個目標 frame 的原點都必須定義在實際切割點。
        self.saw_cut_frame = rospy.get_param(
            rospy.get_name() + "/saw_cut_frame", "saw_cut_point"
        )
        self.bunch_target_frame = rospy.get_param(
            rospy.get_name() + "/bunch_target_frame", "oilpalm"
        )
        self.stem_target_frame = rospy.get_param(
            rospy.get_name() + "/stem_target_frame", "stem"
        )

        rospy.loginfo("Get subscriber topic parameter")
        rospy.loginfo("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        rospy.loginfo("pose_topic: {}, type: {}".format(self.pose_topic, type(self.pose_topic)))
        rospy.loginfo("confidence_minimum: {}, type: {}".format(self.confidence_minimum, type(self.confidence_minimum)))
        rospy.loginfo("arm_status_topic: {}, type: {}".format(self.arm_status_topic, type(self.arm_status_topic)))
        rospy.loginfo("arm_control_topic: {}, type: {}".format(self.arm_control_topic, type(self.arm_control_topic)))
        rospy.loginfo("harvest_done_topic: {}, type: {}".format(self.harvest_done_topic, type(self.harvest_done_topic)))
        rospy.loginfo("pub_name: {}, type: {}".format(self.pub_name, type(self.pub_name)))
        rospy.loginfo("saw_cut_frame: %s", self.saw_cut_frame)
        rospy.loginfo("bunch_target_frame: %s", self.bunch_target_frame)
        rospy.loginfo("stem_target_frame: %s", self.stem_target_frame)

        self.bunch_z_axis_sign = rospy.get_param(rospy.get_name() + "/bunch_z_axis_sign", 0.0)
        self.stem_z_axis_sign = rospy.get_param(rospy.get_name() + "/stem_z_axis_sign", 0.0)

        self.bunch_cut_height_comp_mm = rospy.get_param(rospy.get_name() + "/bunch_cut_height_comp_mm", 0.0)
        self.stem_cut_height_comp_mm = rospy.get_param(rospy.get_name() + "/stem_cut_height_comp_mm", 0.0)
        self.bunch_y_comp_mm = rospy.get_param(rospy.get_name() + "/bunch_y_comp_mm", 0.0)
        self.stem_y_comp_mm = rospy.get_param(rospy.get_name() + "/stem_y_comp_mm", 0.0)

        rospy.loginfo("bunch_z_axis_sign: %s", self.bunch_z_axis_sign)
        rospy.loginfo("stem_z_axis_sign: %s", self.stem_z_axis_sign)
        rospy.loginfo("bunch_cut_height_comp_mm: %s", self.bunch_cut_height_comp_mm)
        rospy.loginfo("stem_cut_height_comp_mm: %s", self.stem_cut_height_comp_mm)
        rospy.loginfo("bunch_y_comp_mm: %s", self.bunch_y_comp_mm)
        rospy.loginfo("stem_y_comp_mm: %s", self.stem_y_comp_mm)

        # camera parking setting
        self.camera_desired_dist_threshold = rospy.get_param(rospy.get_name() + "/camera_desired_dist_threshold", 0.0)
        self.camera_horizon_alignment_threshold = rospy.get_param(rospy.get_name() + "/camera_horizon_alignment_threshold", 0.0)
        self.parking_stable_time_sec = rospy.get_param(
            rospy.get_name() + "/parking_stable_time_sec", 2.0
        )

        # Nearby dead-reckoning pose source.
        # Valid values:
        #   map  -> turn / straight / return turn all use robot_map_*
        #   odom -> turn / straight / return turn all use robot_2d_*
        # This setting is fixed for every Nearby execution; there is no automatic fallback.
        self.nearby_pose_source = str(rospy.get_param(rospy.get_name() + "/nearby_pose_source", "map")).strip().lower()
        if self.nearby_pose_source not in ("map", "odom"):
            rospy.logwarn("Invalid nearby_pose_source=%r; fallback to 'map'.",self.nearby_pose_source,)
            self.nearby_pose_source = "map"

        rospy.loginfo("Get camera parking parameter")
        rospy.loginfo("camera_desired_dist_threshold: {}, type: {}".format(self.camera_desired_dist_threshold, type(self.camera_desired_dist_threshold)))
        rospy.loginfo("camera_horizon_alignment_threshold: {}, type: {}".format(self.camera_horizon_alignment_threshold, type(self.camera_horizon_alignment_threshold)))
        rospy.loginfo("parking_stable_time_sec: {}, type: {}".format(self.parking_stable_time_sec, type(self.parking_stable_time_sec)))
        rospy.loginfo("nearby_pose_source: %s", self.nearby_pose_source)

        # Cut pliers (arm) control settings
        self.spin_forward_comp = rospy.get_param(rospy.get_name() + "/spin_forward_comp", 0.0)

        # X/Z 控制方向與穩定完成判斷
        # cmd_invert：底層控制器的位置命令方向是否需要反向映射
        # visual_sign_invert：視覺誤差方向是否需要反向
        self.x_cmd_invert = rospy.get_param(rospy.get_name() + "/x_cmd_invert", False)
        self.z_cmd_invert = rospy.get_param(rospy.get_name() + "/z_cmd_invert", False)
        self.x_visual_sign_invert = rospy.get_param(rospy.get_name() + "/x_visual_sign_invert", False)
        self.z_visual_sign_invert = rospy.get_param(rospy.get_name() + "/z_visual_sign_invert", False)

        # 進入容許帶後立即停止，且必須連續維持這段時間才算完成
        self.zx_stable_time_sec = rospy.get_param(rospy.get_name() + "/zx_stable_time_sec", 1.0)
        self.axis_cmd_resend_sec = rospy.get_param(rospy.get_name() + "/axis_cmd_resend_sec", 0.3)
        # 判斷上一個小步位置命令是否已經執行完成
        self.axis_motion_tolerance_mm = rospy.get_param(
            rospy.get_name() + "/axis_motion_tolerance_mm", 1.0
        )

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
        self.z_step_max_mm = rospy.get_param(rospy.get_name() + "/z_step_max_mm", 10.0)
        self.z_step_gain = rospy.get_param(rospy.get_name() + "/z_step_gain", 0.30)

        # X 目標 / 自適應步長
        self.x_tolerance_mm = rospy.get_param(rospy.get_name() + "/x_tolerance_mm", 0.0)
        self.x_step_min_mm = rospy.get_param(rospy.get_name() + "/x_step_min_mm", 5.0)
        self.x_step_max_mm = rospy.get_param(rospy.get_name() + "/x_step_max_mm", 12.0)
        self.x_step_gain = rospy.get_param(rospy.get_name() + "/x_step_gain", 0.30)
        self.circular_saw_allow_retract = rospy.get_param(rospy.get_name() + "/circular_saw_allow_retract", True)
        self.x_circular_saw_target = rospy.get_param(rospy.get_name() + "/x_circular_saw_target", 0.0)
        self.circular_saw_blind_extend_length = rospy.get_param(rospy.get_name() + "/circular_saw_blind_extend_length", 0.0)
        self.circular_saw_home_length = rospy.get_param(rospy.get_name() + "/circular_saw_home_length", 0.0)
        self.circular_saw_home_height = rospy.get_param(rospy.get_name() + "/circular_saw_home_height", 0.0)

        # 視覺伺服小步部署
        self.visual_servo_step_max_mm = rospy.get_param(rospy.get_name() + "/visual_servo_step_max_mm", 12.0)

        rospy.loginfo("Get cut pliers (arm) control parameters")
        rospy.loginfo("spin_forward_comp: %s", self.spin_forward_comp)
        rospy.loginfo("x_cmd_invert: %s", self.x_cmd_invert)
        rospy.loginfo("z_cmd_invert: %s", self.z_cmd_invert)
        rospy.loginfo("x_visual_sign_invert: %s", self.x_visual_sign_invert)
        rospy.loginfo("z_visual_sign_invert: %s", self.z_visual_sign_invert)
        rospy.loginfo("zx_stable_time_sec: %s", self.zx_stable_time_sec)
        rospy.loginfo("axis_cmd_resend_sec: %s", self.axis_cmd_resend_sec)
        rospy.loginfo("axis_motion_tolerance_mm: %s", self.axis_motion_tolerance_mm)
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
        rospy.loginfo("visual_servo_step_max_mm: %s", self.visual_servo_step_max_mm)

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
        self.is_map_pose_received = False  # 是否已收到 MapToBaselink 資料
        self.robot_map_pose_x = 0.0
        self.robot_map_pose_y = 0.0
        self.robot_map_theta = 0.0
        self.previous_robot_map_theta = 0.0
        self.total_robot_map_theta = 0.0
    
    def create_subscriber_publisher(self):
        # Tracker is unified now: it publishes ONLY forklift_msg/Confidence on `self.pose_topic`.
        # PBVS should subscribe to that unified topic and obtain pose from Confidence.position/orientation.
        self.sub_map_robot = rospy.Subscriber(self.pub_name, Pose, self.cbGetRobotMap, queue_size = 1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.cbGetOdom, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1, latch=True)

        # Unified Confidence topic: /oilpalm
        self.object_pose_confidence_sub = rospy.Subscriber(self.pose_topic, Confidence, self.cbGetObjectConfidence, queue_size = 1)
        
        self.pose_detection_pub = rospy.Publisher(self.pose_topic + "_detection", Detection, queue_size = 1, latch=True)

        self.harvest_done_pub = rospy.Publisher(self.harvest_done_topic, Bool, queue_size=1)
        
        # 新增手臂相關的 Subscriber 和 Publisher
        self.arm_control_pub = rospy.Publisher(self.arm_control_topic, CircularSawCmd, queue_size=1)
        self.arm_status_sub = rospy.Subscriber(self.arm_status_topic, CircularSawState, self.arm_status_callback, queue_size=1)

    def fnDetectionAllowed(self, pose_detection=False, det_select_mode="nearest_depth", layer=0.0):
        pose_msg = Detection()
        pose_msg.detection_allowed = pose_detection
        pose_msg.det_select_mode = det_select_mode
        pose_msg.layer = layer
        self.pose_detection_pub.publish(pose_msg)

        rospy.sleep(0.2)
        # rospy.loginfo("pose_msg = {}, pallet_msg = {}".format(pose_msg, pallet_msg))

    def fnHarvestDone(self):
        self.harvest_done_pub.publish(Bool(data=True))
        rospy.sleep(0.1)

    def arm_status_callback(self, msg):
        self.circular_saw_state = msg
        
        # ====================================================
        # [動態 TF 核心核心] 根據圓鋸手臂目前的物理位置，即時計算相機新座標
        # ROS 端坐標系定義：X 越大越往前 (mm)； Z 越大越往下 (mm)
        # ====================================================
        dx = msg.x_pos / 1000.0  # 手臂往前伸長的距離 (公尺)
        dz = msg.z_pos / 1000.0  # 手臂往下移動的距離 (公尺)
        
        # 疊加動態位移 (往下移動 dz 代表在車體坐標系中高度 Z 降低)
        current_camera_x = self.camera_base_x + dx
        current_camera_y = self.camera_base_y
        current_camera_z = self.camera_base_z - dz
        
        # 將設定的角度轉為四元數
        quat = tf.transformations.quaternion_from_euler(
            self.camera_base_roll, self.camera_base_pitch, self.camera_base_yaw, 'sxyz'
        )
        
        # 發布動態座標變換關係：base_link -> camera_color_optical_frame
        self.tf_broadcaster.sendTransform(
            (current_camera_x, current_camera_y, current_camera_z),
            quat,
            rospy.Time.now(),
            "camera_color_optical_frame", # 子坐標系名稱
            "base_link"                  # 父坐標系名稱
        )

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

    def get_camera_side_from_tf(self):
        """
            camera_y > 0 : left
            camera_y < 0 : right
        """
        try:
            self.tf_listener.waitForTransform(
                "base_link",
                "camera_color_optical_frame",
                rospy.Time(0),
                rospy.Duration(0.05)
            )

            trans, rot = self.tf_listener.lookupTransform(
                "base_link",
                "camera_color_optical_frame",
                rospy.Time(0)
            )

            camera_y = float(trans[1])

        except Exception as e:
            camera_y = float(getattr(self, "camera_base_y", 0.0))

            rospy.logwarn_throttle(
                1.0,
                f"[PBVS] get_camera_side_from_tf failed, fallback camera_base_y={camera_y:.3f}: {e}"
            )

        if camera_y > 0.01:
            return "left", camera_y
        elif camera_y < -0.01:
            return "right", camera_y
        else:
            return "center", camera_y
    
    def _get_target_config(self, fallback_frame_id=""):
        """
        回傳目前目標的 target frame 與補償設定。

        共同 PBVS 參考座標：
            saw_cut_frame（預設 saw_cut_point）

        bunch:
            saw_cut_point -> oilpalm
            oilpalm 原點必須是果串切割點。

        stem:
            saw_cut_point -> stem
            stem 原點必須是葉莖切割點。

        相機與鋸片的剛性外參只存在於：
            camera_color_optical_frame -> saw_cut_point
        不再把相機到鋸片的偏移掛在 oilpalm / stem 底下。
        """
        target_tag = str(getattr(self.sub_detectionConfidence, "tag", "") or "").strip().lower()
        current_target = str(
            getattr(self, "current_vision_target", fallback_frame_id) or fallback_frame_id or ""
        ).strip().lower()

        bunch_aliases = {
            "bunch", "oilpalm",
            str(getattr(self, "bunch_target_frame", "oilpalm")).strip().lower(),
        }
        stem_aliases = {
            "stem",
            str(getattr(self, "stem_target_frame", "stem")).strip().lower(),
        }

        # Confidence.state 解析出的 tag 優先；frame_id 僅在 tag 尚未建立時備援。
        if target_tag in stem_aliases:
            use_stem = True
        elif target_tag in bunch_aliases:
            use_stem = False
        else:
            use_stem = current_target in stem_aliases

        if use_stem:
            target_kind = "stem"
            target_frame = self.stem_target_frame
            z_axis_sign = float(getattr(self, "stem_z_axis_sign", -1.0))
            z_comp_m = float(getattr(self, "stem_cut_height_comp_mm", 0.0)) * 0.001
            y_comp_m = float(getattr(self, "stem_y_comp_mm", 0.0)) * 0.001
        else:
            # 未知或尚未收到 tag 時，維持原流程的 bunch fallback。
            target_kind = "bunch"
            target_frame = self.bunch_target_frame
            z_axis_sign = float(getattr(self, "bunch_z_axis_sign", -1.0))
            z_comp_m = float(getattr(self, "bunch_cut_height_comp_mm", 0.0)) * 0.001
            y_comp_m = float(getattr(self, "bunch_y_comp_mm", 0.0)) * 0.001

        return target_kind, target_frame, z_axis_sign, z_comp_m, y_comp_m

    def _target_y_axis_angle_from_relative_quaternion(self, rot, target_kind):
        """
        由 saw_cut_frame -> target 的相對旋轉計算目標方向角。

        使用 target frame 的 +Y 軸（綠軸）投影到 saw/camera optical
        的 XY 影像平面：
            +X = 畫面向右
            +Y = 畫面向下

        垂直向下定義為 0 rad。由於切割方向通常沒有箭頭正反之分，
        當 +Y 指向畫面上方時將整條軸翻轉 180 度，使角度維持在
        約 [-90, +90] 度。
        """
        R = tf.transformations.quaternion_matrix(rot)
        vec_x = float(R[0, 1])
        vec_y = float(R[1, 1])

        projection_norm = math.hypot(vec_x, vec_y)
        if projection_norm < 1e-6:
            rospy.logwarn_throttle(
                1.0,
                f"[PBVS ANGLE] {target_kind} 的 +Y 軸接近相機視線方向，"
                "影像平面角度不可靠，暫時沿用上一筆角度。"
            )
            return float(getattr(self, "green_line_angle_rad", 0.0))

        if vec_y < 0.0:
            vec_x = -vec_x
            vec_y = -vec_y

        return math.atan2(vec_x, vec_y)

    def update_target_from_tf(self):
        """
        將果串與葉莖統一轉換為「目標切割點相對鋸片切割點」的 PBVS 誤差。

        TF lookup：
            saw_cut_frame -> oilpalm   (bunch)
            saw_cut_frame -> stem      (stem)

        因 saw_cut_point 與 camera optical 軸方向相同：
            relative +X = 畫面向右
            relative +Y = 畫面向下
            relative +Z = 鋸片前方

        PBVS / Action 使用：
            marker_x = forward distance
            marker_y = lateral error
            marker_z = vertical error
            marker_theta = target +Y axis angle
        """
        target_kind, target_frame, z_axis_sign, z_comp_m, y_comp_m = self._get_target_config()
        reference_frame = self.saw_cut_frame

        try:
            self.tf_listener.waitForTransform(
                reference_frame,
                target_frame,
                rospy.Time(0),
                rospy.Duration(0.05)
            )

            trans, rot = self.tf_listener.lookupTransform(
                reference_frame,
                target_frame,
                rospy.Time(0)
            )

            # target 相對 saw_cut_point 的座標。
            # saw_cut_point 在 launch 中保持與 camera optical 相同軸向。
            ex, ey, ez = map(float, trans)

            # 前後：目標在鋸片前方的距離。
            # X 軸控制仍可用 x_circular_saw_target 保留預停距離。
            self.marker_2d_pose_x = ez

            # 左右：目標相對鋸片的水平誤差。
            # y_comp_m 僅作最後的小量機構校正。
            self.marker_2d_pose_y = ex + y_comp_m

            # 高度：目標相對鋸片的垂直誤差。
            # optical +Y 向下；目前 launch 的 sign=-1 會轉成「向上為正」。
            self.marker_2d_pose_z = z_axis_sign * ey + z_comp_m

            # 姿態：直接使用 target 相對 saw 的 6D / 幾何 pose，
            # 不再用 atan2(position_y, position_z) 代替物件角度。
            target_angle = self._target_y_axis_angle_from_relative_quaternion(
                rot, target_kind
            )
            self.marker_2d_theta = target_angle
            self.green_line_angle_rad = target_angle

            rospy.loginfo_throttle(
                0.5,
                f"[PBVS TARGET] kind={target_kind}, "
                f"tf={reference_frame}->{target_frame}, "
                f"right={ex:.3f}m, down={ey:.3f}m, forward={ez:.3f}m, "
                f"control_x={self.marker_2d_pose_x:.3f}m, "
                f"control_y={self.marker_2d_pose_y:.3f}m, "
                f"control_z={self.marker_2d_pose_z:.3f}m, "
                f"angle={math.degrees(target_angle):.1f}deg, "
                f"y_comp={y_comp_m:.3f}m, "
                f"z_sign={z_axis_sign:+.1f}, z_comp={z_comp_m:.3f}m"
            )

            return True

        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                f"[PBVS TARGET] 尚未取得 TF {reference_frame}->{target_frame}，"
                f"手臂維持安全等待... err={e}"
            )
            return False

    def __del__(self):
        self.window.destroy()

    def SpinOnce(self):
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z, self.marker_2d_theta
    
    def SpinOnce_confidence(self):
        return self.sub_detectionConfidence

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
        self.current_vision_target = msg.frame_id

        self.sub_detectionConfidence.pose_confidence = msg.object_IoU
        self.sub_detectionConfidence.pose_detection = msg.object_detection
        self.sub_detectionConfidence.state = msg.state

        state = msg.state.split(':')
        self.sub_detectionConfidence.tag = state[0]
        self.sub_detectionConfidence.state = state[1]

        target_kind, target_frame, z_axis_sign, z_comp_m, y_comp_m = self._get_target_config(
            fallback_frame_id=msg.frame_id
        )

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
            self.subscriber.fnHarvestDone()
            return
        
        self.subscriber.fnHarvestDone()
        rospy.logwarn('PBVS Succeeded')
        self._result.result = 'PBVS Succeeded'
        self._as.set_succeeded(self._result)
        self.PBVS = None


if __name__ == '__main__':
    rospy.init_node('PBVS_server')
    rospy.logwarn(rospy.get_name() + 'start')
    server = PBVSAction(rospy.get_name())
    rospy.spin()
