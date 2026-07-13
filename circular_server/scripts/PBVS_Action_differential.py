# -*- coding: utf-8 -*-
from sys import flags
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
import statistics
import time
from circular_saw_controller.msg import CircularSawCmd, CircularSawState

class Action():
    def __init__(self, Subscriber):
        # cmd_vel
        self.cmd_vel = cmd_vel(Subscriber)
        self.Subscriber = Subscriber
        self.NearbySequence = Enum( 'NearbySequence', \
                    'initial_dist \
                    turn_right \
                    go_straight \
                    initial_marker \
                    turn_left')
        self.current_nearby_sequence = self.NearbySequence.initial_dist.value
        self.previous_nearby_sequence = None  # 用來記錄上一次的階段
        # Odometry_param
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # 用來避免重複下指令
        self._last_cmd_x = None
        self._last_cmd_z = None
        self._last_cmd_angle = None

        # X 視覺對位若已超過停止距離且禁止後退，
        # 在此鎖定超過量，供後續盲伸距離補償使用。
        self.x_overshoot_mm = 0.0
        self._last_cmd_time_x = 0.0
        self._last_cmd_time_z = 0.0

        # 車體 parking 進入容許帶後的連續穩定計時起點。
        self._parking_in_tolerance_since = None

        # other
        self.check_wait_time = 0
        self.wait_time = 0
        self.is_triggered = False

        # Nearby 專用狀態：pose 來源由 ROS 參數 nearby_pose_source 固定指定。
        # 可選 map 或 odom；Nearby 執行期間與不同輪次都不自動 fallback。
        # 既有 fnseqDeadReckoning* 不修改，其他流程仍維持原本 odom 控制。
        self._nearby_angle_inited = False
        self._nearby_dist_inited = False
        self._nearby_fp_reset_sent = False
        self._nearby_pose_source = None

    def _saturate(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _soft_stop_arm(self):
        self.cmd_vel.fnPauseArm()

    def _axis_limits(self, axis_name):
        if axis_name == "X":
            lo = float(self.Subscriber.circular_saw_min_length)
            hi = float(self.Subscriber.circular_saw_max_length)
        else:
            lo = float(self.Subscriber.circular_saw_min_height)
            hi = float(self.Subscriber.circular_saw_max_height)
        return lo, hi

    def _get_axis_invert(self, axis_name):
        attr = "x_cmd_invert" if axis_name == "X" else "z_cmd_invert"
        return bool(getattr(self.Subscriber, attr, False))

    def _set_axis_invert(self, axis_name, value):
        attr = "x_cmd_invert" if axis_name == "X" else "z_cmd_invert"
        setattr(self.Subscriber, attr, bool(value))
        rospy.logwarn(f"[{axis_name} AUTO] command direction invert -> {bool(value)}")

    def _to_controller_axis_target(self, axis_name, logical_target):
        """
        將 Action 內部 logical target 轉成底層 circular_saw_controller command target。
        若 *_cmd_invert=True，使用 min+max-target 做反向映射。
        """
        lo, hi = self._axis_limits(axis_name)
        logical_target = self._saturate(float(logical_target), lo, hi)

        if self._get_axis_invert(axis_name):
            return self._saturate((lo + hi) - logical_target, lo, hi)

        return logical_target

    def _get_axis_step_params(self, axis_name):
        """
        小步部署參數，避免每一幀直接下完整視覺誤差造成震盪。
        """
        if axis_name == "X":
            step_min = float(getattr(self.Subscriber, "x_step_min_mm", 3.0))
            step_max = float(getattr(self.Subscriber, "x_step_max_mm", 15.0))
            step_gain = float(getattr(self.Subscriber, "x_step_gain", 0.30))
        else:
            step_min = float(getattr(self.Subscriber, "z_step_min_mm", 2.0))
            step_max = float(getattr(self.Subscriber, "z_step_max_mm", 10.0))
            step_gain = float(getattr(self.Subscriber, "z_step_gain", 0.30))

        # 部署預設不要太大，上限再保護一次。
        global_max = float(getattr(self.Subscriber, "visual_servo_step_max_mm", 12.0))
        step_max = min(step_max, global_max)

        return step_min, step_max, step_gain

    def _make_limited_axis_target(self, axis_name, cur_mm, err_mm, sign_positive_err_to_positive_target):
        """
        依誤差產生小步目標。
        sign_positive_err_to_positive_target:
            True  : err>0 時 target = cur + step
            False : err>0 時 target = cur - step
        """
        step_min, step_max, step_gain = self._get_axis_step_params(axis_name)

        step = abs(float(err_mm)) * step_gain
        step = max(step_min, min(step_max, step))

        direction = 1.0 if float(err_mm) > 0.0 else -1.0
        if not sign_positive_err_to_positive_target:
            direction = -direction

        target = float(cur_mm) + direction * step
        lo, hi = self._axis_limits(axis_name)
        target = self._saturate(target, lo, hi)

        return target, step


    def _get_visual_sign_invert(self, axis_name):
        attr = "x_visual_sign_invert" if axis_name == "X" else "z_visual_sign_invert"
        return bool(getattr(self.Subscriber, attr, False))

    def _set_visual_sign_invert(self, axis_name, value):
        attr = "x_visual_sign_invert" if axis_name == "X" else "z_visual_sign_invert"
        setattr(self.Subscriber, attr, bool(value))
        rospy.logwarn(f"[{axis_name} AUTO] visual error sign invert -> {bool(value)}")

    def fnRotateToRelativeLine(self, distance, Kp, v):
        if not getattr(self, "_rot_rel_inited", False):
            self._rot_rel_inited = True
            self._rot_rel_start = time.time()
            self._rot_rel_time_needed = abs(distance / (Kp * v))
            rospy.loginfo(f'time_needed:{self._rot_rel_time_needed}')

        if (time.time() - self._rot_rel_start) < self._rot_rel_time_needed:
            self.cmd_vel.fnGoStraight(Kp, v)
            return False
        else:
            self._soft_stop_arm()
            self._rot_rel_inited = False
            return True

    def fnseqDeadReckoningAngle_Time(self, target_angle, Kp, theta):
        if not getattr(self, "_dead_ang_time_inited", False):
            self._dead_ang_time_inited = True
            self._dead_ang_time_start = time.time()
            target_angle_rad = math.radians(target_angle)
            self._dead_ang_time_needed = target_angle_rad / (Kp * theta)
            rospy.loginfo(f'time_needed:{self._dead_ang_time_needed}')

        if (time.time() - self._dead_ang_time_start) < self._dead_ang_time_needed:
            self.cmd_vel.fnTurn(Kp, theta)
            return False
        else:
            self._soft_stop_arm()
            self._dead_ang_time_inited = False
            return True
    
    def fnseqDeadReckoningAngle(self, target_angle):
        Kp = 0.3
        threshold = 0.015  # 停止的閾值（弧度）
        target_angle_rad = math.radians(target_angle)   # 將目標角度轉換為弧度
        if not self.is_triggered:   # 初始化：如果是第一次調用，記錄初始累積角度
            self.is_triggered = True
            self.initial_total_theta = self.Subscriber.robot_2d_theta  # 使用累積的總角度作為初始角度
        
        current_angle = self.Subscriber.robot_2d_theta - self.initial_total_theta  # 計算當前已旋轉的角度
        remaining_angle = target_angle_rad - current_angle  # 計算剩餘的旋轉角度
        if abs(remaining_angle) < threshold:   # 判斷是否達到目標角度
            self._soft_stop_arm()  # 停止機器人
            self.is_triggered = False  # 重置觸發狀態
            return True
        else:
            self.cmd_vel.fnTurn(Kp, remaining_angle)    # 執行旋轉，正負值決定方向
            return False

    def fnseqDeadReckoning(self, dead_reckoning_dist):  # 使用里程計算移動到指定距離
        Kp = 0.2
        threshold = 0.015  # 停止的閾值
        if self.is_triggered == False:  # 如果還沒啟動，記錄初始位置
            self.is_triggered = True
            self.initial_robot_pose_x = self.Subscriber.robot_2d_pose_x
            self.initial_robot_pose_y = self.Subscriber.robot_2d_pose_y
        # 計算當前移動距離
        current_dist = self.fnCalcDistPoints(self.initial_robot_pose_x, self.Subscriber.robot_2d_pose_x, self.initial_robot_pose_y, self.Subscriber.robot_2d_pose_y)
        # 計算剩餘距離
        remaining_dist = dead_reckoning_dist - math.copysign(1, dead_reckoning_dist) * current_dist
        # 判斷是否達到目標距離
        if abs(remaining_dist) < threshold:  # 進入停止條件
            self._soft_stop_arm()
            self.is_triggered = False
            return True
        else:
            # 計算速度並保持方向
            self.cmd_vel.fnGoStraight(Kp, remaining_dist)
            return False
            
    def fnSeqMarkerDistanceValid(self):
        Kp = 0.02
        dist = math.sqrt(self.Subscriber.marker_2d_pose_x**2 + self.Subscriber.marker_2d_pose_y**2)
        if self.TFConfidence() and dist != 0: #pin &->and
            return True
        else:
            # rospy.logwarn("Confidence Low")
            return False

    def fnSeqChangingtheta(self, threshod): #旋轉到marker的theta值為0, threshod為角度誤差值
        Kp = 0.02
        if self.TFConfidence():
            # self.marker_2d_theta= self.TrustworthyMarker2DTheta(3)
            # print("desired_angle_turn", self.marker_2d_theta)
            # print("threshod", threshod)
            if abs(self.Subscriber.marker_2d_theta) < threshod  :
                self._soft_stop_arm()
                if self.check_wait_time > 20 :
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  + 1
                    return False
            else:
                self.cmd_vel.fnTurn(Kp, -self.Subscriber.marker_2d_theta)
                self.check_wait_time = 0
                return False
        else:
            self.check_wait_time = 0
            return False
        
    def _get_nearby_map_pose(self):
        """取得 map pose；尚未收到時回傳 None。"""
        if not bool(getattr(self.Subscriber, "is_map_pose_received", False)):
            return None

        required = (
            "robot_map_pose_x",
            "robot_map_pose_y",
            "robot_map_theta",
        )
        if not all(hasattr(self.Subscriber, name) for name in required):
            return None

        return (
            float(self.Subscriber.robot_map_pose_x),
            float(self.Subscriber.robot_map_pose_y),
            float(self.Subscriber.robot_map_theta),
        )

    def _get_nearby_odom_pose(self):
        """取得 odom pose；尚未收到時回傳 None。"""
        if not bool(getattr(self.Subscriber, "is_odom_received", False)):
            return None

        required = (
            "robot_2d_pose_x",
            "robot_2d_pose_y",
            "robot_2d_theta",
        )
        if not all(hasattr(self.Subscriber, name) for name in required):
            return None

        return (
            float(self.Subscriber.robot_2d_pose_x),
            float(self.Subscriber.robot_2d_pose_y),
            float(self.Subscriber.robot_2d_theta),
        )

    def _select_nearby_pose_source(self):
        """
        map：整段 Nearby 一律使用 robot_map_*。
        odom：整段 Nearby 一律使用 robot_2d_*。
        """
        configured = str(getattr(self.Subscriber, "nearby_pose_source", "map")).strip().lower()

        if configured not in ("map", "odom"):
            rospy.logerr_throttle(1.0,f"[NEARBY] invalid nearby_pose_source={configured!r}; allowed values are 'map' and 'odom'. Vehicle stopped.")
            self._nearby_pose_source = None
            return None

        # 每次 Nearby 都使用同一個設定；保留 lock 只為避免階段間狀態不一致。
        if self._nearby_pose_source != configured:
            self._nearby_pose_source = configured
            rospy.logwarn(f"[NEARBY] fixed pose source: {configured.upper()}")

        return self._nearby_pose_source

    def _get_nearby_pose(self):
        """依已鎖定的 Nearby pose source 取得目前位置。"""
        if self._nearby_pose_source == "map":
            return self._get_nearby_map_pose()
        if self._nearby_pose_source == "odom":
            return self._get_nearby_odom_pose()
        return None

    def _reset_nearby_motion_state(self, clear_source=False):
        """清除 Nearby dead-reckoning 階段狀態。"""
        self._nearby_angle_inited = False
        self._nearby_dist_inited = False

        if clear_source:
            self._nearby_pose_source = None

    def fnseqDeadReckoningAngleNearby(self, target_angle):
        """Nearby 專用旋轉控制，使用已鎖定的 map 或 odom pose。"""
        pose = self._get_nearby_pose()
        source = self._nearby_pose_source or "unknown"

        if pose is None:
            self.cmd_vel.fnStopVehicle()
            rospy.logwarn_throttle(
                1.0,
                f"[NEARBY {source.upper()}] pose unavailable during angle control; "
                "vehicle stopped. Pose source will not switch mid-motion."
            )
            return False

        _, _, current_theta = pose
        kp = 0.3
        threshold = 0.015
        target_angle_rad = math.radians(float(target_angle))

        if not self._nearby_angle_inited:
            self._nearby_angle_inited = True
            self._nearby_angle_start = current_theta
            rospy.loginfo(
                f"[NEARBY {source.upper()} ANGLE] start={current_theta:.4f}rad, "
                f"target_delta={target_angle_rad:.4f}rad"
            )

        rotated = current_theta - self._nearby_angle_start
        remaining = target_angle_rad - rotated

        rospy.loginfo_throttle(
            0.5,
            f"[NEARBY {source.upper()} ANGLE] current={current_theta:.4f}, "
            f"rotated={rotated:.4f}, remaining={remaining:.4f}rad"
        )

        if abs(remaining) < threshold:
            self.cmd_vel.fnStopVehicle()
            self._nearby_angle_inited = False
            return True

        self.cmd_vel.fnTurn(kp, remaining)
        return False

    def fnseqDeadReckoningNearby(self, target_distance):
        """Nearby 專用直線控制，使用已鎖定的 map 或 odom pose。"""
        pose = self._get_nearby_pose()
        source = self._nearby_pose_source or "unknown"

        if pose is None:
            self.cmd_vel.fnStopVehicle()
            rospy.logwarn_throttle(
                1.0,
                f"[NEARBY {source.upper()}] pose unavailable during distance control; "
                "vehicle stopped. Pose source will not switch mid-motion."
            )
            return False

        current_x, current_y, _ = pose
        kp = 0.2
        threshold = 0.015
        target_distance = float(target_distance)

        if not self._nearby_dist_inited:
            self._nearby_dist_inited = True
            self._nearby_start_x = current_x
            self._nearby_start_y = current_y
            rospy.loginfo(
                f"[NEARBY {source.upper()} DIST] "
                f"start=({current_x:.4f}, {current_y:.4f}), "
                f"target={target_distance:.4f}m"
            )

        travelled = self.fnCalcDistPoints(
            self._nearby_start_x,
            current_x,
            self._nearby_start_y,
            current_y,
        )
        remaining = (
            target_distance
            - math.copysign(1.0, target_distance) * travelled
        )

        rospy.loginfo_throttle(
            0.5,
            f"[NEARBY {source.upper()} DIST] "
            f"current=({current_x:.4f}, {current_y:.4f}), "
            f"travelled={travelled:.4f}, remaining={remaining:.4f}m"
        )

        if abs(remaining) < threshold:
            self.cmd_vel.fnStopVehicle()
            self._nearby_dist_inited = False
            return True

        self.cmd_vel.fnGoStraight(kp, remaining)
        return False

    def fnSeqMovingNearbyParkingLot(self, desired_dist_threshold):
        """
        initial_dist 使用當下 FoundationPose 結果判斷是否需要移車。
        pose 來源由 ROS 參數 nearby_pose_source 固定指定為 map 或 odom。
        FoundationPose reset 與 detection disable 只送一次。
        Nearby 不列入 PBVS 的 vision_required_sequences。
        Nearby 完成後清除來源鎖定；其他流程仍使用原本 odom。
        """
        err = 0.05

        if self.current_nearby_sequence != self.previous_nearby_sequence:
            rospy.loginfo("current_nearby_sequence {0}".format(self.NearbySequence(self.current_nearby_sequence)))
            self.previous_nearby_sequence = self.current_nearby_sequence

        if self.current_nearby_sequence == self.NearbySequence.initial_dist.value:
            # 尚未 reset FoundationPose 前，先用目前視覺結果判斷是否需要移車。
            if not self.TFConfidence():
                self.cmd_vel.fnStopVehicle()
                return False

            self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(5)
            self.initial_marker_pose_x = self.Subscriber.marker_2d_pose_x
            self.initial_marker_pose_y = self.Subscriber.marker_2d_pose_y
            self.desired_dist_diff = (abs(self.initial_marker_pose_x) - float(desired_dist_threshold))

            rospy.loginfo(f"[NEARBY] desired_dist_diff={self.desired_dist_diff:.4f}m")

            # 已經足夠接近，不需要移車，也不重置 FoundationPose。
            if abs(self.initial_marker_pose_x) <= abs(float(desired_dist_threshold)):
                self.cmd_vel.fnStopVehicle()
                self._reset_nearby_motion_state(clear_source=True)
                self._nearby_fp_reset_sent = False
                return True

            # 確定需要移車後，依 nearby_pose_source 使用固定 pose source。
            source = self._select_nearby_pose_source()
            pose = self._get_nearby_pose()

            if source is None or pose is None:
                self.cmd_vel.fnStopVehicle()
                configured = str(getattr(self.Subscriber, "nearby_pose_source", "map")).strip().lower()
                rospy.logwarn_throttle(1.0,f"[NEARBY] selected {configured.upper()} pose is unavailable; vehicle stopped and waiting. No automatic fallback.")
                return False

            pose_x, pose_y, pose_theta = pose
            self.initial_robot_pose_theta = pose_theta
            self.initial_robot_pose_x = pose_x
            self.initial_robot_pose_y = pose_y

            camera_side, camera_y = self.Subscriber.get_camera_side_from_tf()

            if camera_side == "right":
                self.first_turn_deg = 90
            elif camera_side == "left":
                self.first_turn_deg = -90
            else:
                rospy.logwarn(f"[PBVS] camera_y={camera_y:.3f}m near center; use right-side turn strategy.")
                self.first_turn_deg = 90

            self.second_turn_deg = -self.first_turn_deg

            # 確定要進行 Nearby 後，只執行一次 reset / detection disable。
            if not self._nearby_fp_reset_sent:
                self.Subscriber.fnDetectionAllowed(pose_detection=False,det_select_mode="nearest_depth",)
                self.Subscriber.fnHarvestDone()
                self._nearby_fp_reset_sent = True
                rospy.logwarn("[NEARBY] FoundationPose reset sent once; detection disabled.")

            self._reset_nearby_motion_state(clear_source=False)
            self.current_nearby_sequence = self.NearbySequence.turn_right.value

            rospy.loginfo(
                f"[PBVS] Nearby uses locked {source.upper()} pose: "
                f"start=({pose_x:.4f}, {pose_y:.4f}, {pose_theta:.4f}), "
                f"camera_side={camera_side}, camera_y={camera_y:.3f}m, "
                f"first_turn={self.first_turn_deg}, "
                f"second_turn={self.second_turn_deg}"
            )
            return False

        if self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.fnseqDeadReckoningAngleNearby(self.first_turn_deg):
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
            return False

        if self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            fwd = -(self.desired_dist_diff + err + float(self.Subscriber.spin_forward_comp))
            if self.fnseqDeadReckoningNearby(fwd):
                self.current_nearby_sequence = self.NearbySequence.turn_left.value
            return False

        if self.current_nearby_sequence == self.NearbySequence.turn_left.value:
            if self.fnseqDeadReckoningAngleNearby(self.second_turn_deg):
                self.cmd_vel.fnStopVehicle()

                source = self._nearby_pose_source or "unknown"

                self.current_nearby_sequence = self.NearbySequence.initial_dist.value
                self.previous_nearby_sequence = None
                self._reset_nearby_motion_state(clear_source=True)
                self._nearby_fp_reset_sent = False
                self.check_wait_time = 0

                rospy.loginfo(f"[NEARBY {source.upper()}] completed. Subsequent PBVS stages return to their original odom/vision control.")
                return True
            return False

        # 非預期狀態：安全停止並重置 Nearby 狀態機。
        self.cmd_vel.fnStopVehicle()
        self.current_nearby_sequence = self.NearbySequence.initial_dist.value
        self.previous_nearby_sequence = None
        self._reset_nearby_motion_state(clear_source=True)
        self._nearby_fp_reset_sent = False
        return False

    def fnSeqParking(self, tolerance, kp):
        """
        車體 parking 水平對位。

        完成條件：
        1. marker_2d_pose_y 一進入 tolerance，立即發布零速停止車體。
        2. 在停止狀態下，連續維持 parking_stable_time_sec 秒都在容許帶內。
        3. 中途只要離開容許帶，穩定計時立即歸零並重新修正。
        4. TF / Confidence 中斷時立即停止車體，不能沿用上一筆速度。
        """
        tolerance = max(0.0, abs(float(tolerance)))
        stable_time_sec = max(
            0.0,
            float(getattr(self.Subscriber, "parking_stable_time_sec", 2.0))
        )

        # 視覺失效時必須立刻停止底盤，並取消穩定計時。
        if not self.TFConfidence():
            self.cmd_vel.fnStopVehicle()
            self._parking_in_tolerance_since = None
            rospy.logwarn_throttle(
                1.0,
                "[PARKING] TF/Confidence 不足，車體立即停止並等待有效姿態。"
            )
            return False

        err_y_m = float(self.Subscriber.marker_2d_pose_y)
        now = time.monotonic()

        rospy.loginfo_throttle(
            0.5,
            f"[PARKING] err_y={err_y_m:.4f}m, "
            f"tol={tolerance:.4f}m, kp={kp:.3f}"
        )

        # 進入容許帶：立刻停止底盤並開始連續穩定計時。
        if abs(err_y_m) <= tolerance:
            self.cmd_vel.fnStopVehicle()

            if self._parking_in_tolerance_since is None:
                self._parking_in_tolerance_since = now
                rospy.loginfo(
                    f"[PARKING HOLD] 進入容許帶，車體立即停止。"
                    f"err={err_y_m:.4f}m，需穩定 {stable_time_sec:.2f}s"
                )

            held_sec = now - self._parking_in_tolerance_since

            rospy.loginfo_throttle(
                0.25,
                f"[PARKING HOLD] err={err_y_m:.4f}m, "
                f"held={held_sec:.2f}/{stable_time_sec:.2f}s"
            )

            if held_sec >= stable_time_sec:
                rospy.loginfo(
                    f"[PARKING] SUCCESS：連續 {held_sec:.2f}s 位於容許帶內，"
                    f"err={err_y_m:.4f}m, tol={tolerance:.4f}m"
                )
                self.cmd_vel.fnStopVehicle()
                self._parking_in_tolerance_since = None
                return True

            return False

        # 離開容許帶：清除計時，重新發布修正速度。
        if self._parking_in_tolerance_since is not None:
            rospy.logwarn(
                f"[PARKING HOLD] 離開容許帶，穩定計時歸零。"
                f"err={err_y_m:.4f}m, tol={tolerance:.4f}m"
            )
        self._parking_in_tolerance_since = None

        self.cmd_vel.fnGoStraight(kp, err_y_m)
        return False
        
    def fnSeqdecide(self, decide_dist, horizontal_dist):#decide_dist偏離多少公分要後退
        if self.TFConfidence():
            dist_ok = (abs(self.Subscriber.marker_2d_pose_x) <= abs(decide_dist) + 0.05)
            horiz_ok = (abs(self.Subscriber.marker_2d_pose_y) <= abs(horizontal_dist) + 0.05)
            # rospy.loginfo(f'dist: {abs(self.marker_2d_pose_x)-abs(decide_dist)}, horiz: {abs(self.marker_2d_pose_y)-abs(horizontal_dist)}')
            # rospy.loginfo(f'dist_ok: {dist_ok}, horiz_ok: {horiz_ok}')
            return (dist_ok and horiz_ok, dist_ok)

    def fnseqMoveToMarkerDist(self, marker_dist): #(使用marker)前後移動到距離marker_dist公尺的位置
        Kp = 0.2
        if(abs(marker_dist) < 2.0):
            threshold = 0.015
        else:
            threshold = 0.03

        dist = math.sqrt(self.Subscriber.marker_2d_pose_x**2 + self.Subscriber.marker_2d_pose_y**2)
        
        if dist < (abs(marker_dist)-threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        elif dist > (abs(marker_dist)+threshold):
            self.cmd_vel.fnGoStraight(Kp, marker_dist - dist)
            return False
        else:
            self._soft_stop_arm()
            return True
            
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def TrustworthyMarker2DTheta(self, wait_time):
        """在 wait_time 內回傳 None，時間到才計算中位數"""
        if not getattr(self, "_trust_theta_inited", False):
            self._trust_theta_inited = True
            self._trust_theta_start = time.time()
            self._trust_theta_list = []

        if (time.time() - self._trust_theta_start) < wait_time:
            self._trust_theta_list.append(self.Subscriber.marker_2d_theta)
            return None  # 回傳 None 告訴外層「我還在收集」

        # 時間到，重置狀態以供下次使用
        self._trust_theta_inited = False
        data = self._trust_theta_list

        if len(data) == 0:
            return self.Subscriber.marker_2d_theta
        if len(data) == 1:
            return data[0]

        threshold = 0.5
        mean = statistics.mean(data)
        stdev = statistics.stdev(data)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        
        clean_list = [i for i in data if downcutoff <= i <= upcutoff]
               
        if not clean_list:
            rospy.logwarn_throttle(1.0, "[PBVS] Clean list is empty, falling back to raw median.")
            return statistics.median(data)

        return statistics.median(clean_list)
    
    def _mm_step(self, err_m, mm_min, mm_max, k=1.0):
        """
        依誤差自動調整步長（m→mm）：
        - err_m: 距離誤差 (m)，取絕對值
        - 先用線性 k*|err_mm|，最後夾在 [mm_min, mm_max]
        """
        err_mm = abs(float(err_m)) * 1000.0
        raw = k * err_mm
        return int(max(mm_min, min(mm_max, raw)))
    

    def _dead_move_axis(self, axis_name, target, tolerance_mm,
                        get_cur_fn, send_axis_cmd_fn, speed):
        """
        共用 1D 目標控制。

        內部 target 一律使用 logical coordinate，也就是與 circular_saw_state.x_pos / z_pos
        同方向的座標。實際送給 controller 前，會依 x_cmd_invert / z_cmd_invert
        自動轉成 controller command target。

        不做自動反向修正；若方向相反，請在 launch 手動設定 x_cmd_invert / z_cmd_invert。
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn(f"{axis_name}: 尚未接收到手臂狀態，等待中...")
            return False

        lo, hi = self._axis_limits(axis_name)
        target = self._saturate(float(target), lo, hi)
        cur = float(get_cur_fn(st))

        # 不做自動反向修正：X/Z 命令方向只由 launch 內的 x_cmd_invert / z_cmd_invert 決定。
        d = target - cur

        # --- 1) 現在位置已經在目標附近 → 視為完成，送一次 Stop ---
        if abs(d) <= tolerance_mm:
            self._soft_stop_arm()
            if axis_name == "X":
                self._last_cmd_x = None
            else:
                self._last_cmd_z = None
            return True

        # --- 2) 還沒到，但檢查「上一個 logical 目標」是否已經到位 ---
        if axis_name == "X":
            last_cmd = self._last_cmd_x
        else:
            last_cmd = self._last_cmd_z

        MIN_CMD_DELTA = 1.0

        if last_cmd is not None:
            remaining_to_last = float(last_cmd) - cur
            now = time.monotonic()
            if axis_name == "X":
                last_cmd_time = self._last_cmd_time_x
            else:
                last_cmd_time = self._last_cmd_time_z
            resend_sec = float(getattr(self.Subscriber, "axis_cmd_resend_sec", 0.3))
            
            # 上一筆位置還沒到
            if abs(remaining_to_last) > tolerance_mm:
                # 尚未到重送時間，繼續等
                if now - last_cmd_time < resend_sec:
                    return False

                # 重新發布「上一個目標」，不能改送本輪新算的目標
                controller_target = self._to_controller_axis_target(axis_name,last_cmd)

                rospy.logwarn_throttle(
                    0.5,
                    f"[{axis_name} CMD RESEND] "
                    f"cur={cur:.1f}mm, "
                    f"last_target={last_cmd:.1f}mm, "
                    f"remaining={remaining_to_last:.1f}mm"
                )

                send_axis_cmd_fn(int(round(controller_target)),speed)

                if axis_name == "X":
                    self._last_cmd_time_x = now
                else:
                    self._last_cmd_time_z = now

                return False

            if abs(target - float(last_cmd)) < MIN_CMD_DELTA:
                return False

        # --- 3) 第一筆命令，或上一筆完成後的新目標 ---
        controller_target = self._to_controller_axis_target(axis_name, target)

        rospy.loginfo(
            f"[{axis_name} CMD MAP] logical={target:.1f}mm -> "
            f"controller={controller_target:.1f}mm, "
            f"cur={cur:.1f}mm, invert={self._get_axis_invert(axis_name)}"
        )

        send_axis_cmd_fn(int(round(controller_target)), speed)
        now = time.monotonic()

        if axis_name == "X":
            self._last_cmd_x = target
            self._last_cmd_time_x = now
        else:
            self._last_cmd_z = target
            self._last_cmd_time_z = now

        return False

    def _dead_move_rel_axis(self, delta, get_cur_fn):
        st = self.Subscriber.circular_saw_state
        if st is None:
            return None
        cur = float(get_cur_fn(st))
        return cur + float(delta)


    def DeadMoveZ(self, target_z, z_tolerance=2, speed=2000):
        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        Z_MAX = float(self.Subscriber.circular_saw_max_height)
        target_z = self._saturate(float(target_z), Z_MIN, Z_MAX)

        return self._dead_move_axis(
            "Z",
            target_z,
            z_tolerance,
            lambda st: st.z_pos,
            lambda z, spd: self.cmd_vel.fnSawUpDown(z, speed=spd),
            speed
        )

    def DeadMoveZRel(self, delta_z):
        st = self.Subscriber.circular_saw_state
        if st is None:
            return None
        return st.z_pos + float(delta_z)


    def DeadMoveX(self, target_x, x_tolerance=2, speed=2000):
        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)
        target_x = self._saturate(float(target_x), L_MIN, L_MAX)

        return self._dead_move_axis(
            "X",
            target_x,
            x_tolerance,
            lambda st: st.x_pos,
            lambda x, spd: self.cmd_vel.fnSawForwardBackward(x, speed=spd),
            speed
        )

    def DeadMoveXRel(self, delta_x):
        st = self.Subscriber.circular_saw_state
        if st is None:
            return None
        return st.x_pos + float(delta_x)

    def fnControlArmBasedOnFruitZ(self, speed=800):
        """
        Z 軸視覺對位。

        完成條件：
        1. 一進入 z_tolerance_mm 容許帶，立即送停止命令。
        2. 在停止狀態下，連續維持 zx_stable_time_sec 秒都在容許帶內。
        3. 中途只要離開容許帶，穩定計時立即歸零。
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn_throttle(1.0, "Z: 尚未接收到手臂狀態")
            return False

        if not self.TFConfidence():
            # 視覺信心不足時，不允許沿用舊命令繼續前進。
            self._soft_stop_arm()
            self._z_in_tolerance_since = None
            self._last_cmd_z = None
            return False

        if not getattr(self, "_z_inited", False):
            self._z_inited = True
            self._z_in_tolerance_since = None
            self._last_cmd_z = None

            self.cmd_vel.latch_axes_from_status(latch_x=True,latch_z=False,latch_angle=True)
            self._z_hold_x_mm = float(self.cmd_vel.target_l_mm)

            rospy.loginfo(f"[Z] 開始對齊 Z 軸，鎖定 X={self._z_hold_x_mm:.1f}mm")

        z_tol_mm = max(0.0, float(self.Subscriber.z_tolerance_mm))
        stable_time_sec = max(0.0,float(self.Subscriber.zx_stable_time_sec))

        raw_rel_z_mm = float(self.Subscriber.marker_2d_pose_z) * 1000.0
        cur_h = float(st.z_pos)

        # 視覺方向設定只影響移動方向，不影響容許帶判斷。
        rel_z_mm = (
            -raw_rel_z_mm
            if self._get_visual_sign_invert("Z")
            else raw_rel_z_mm
        )

        rospy.loginfo_throttle(
            0.5,
            f"[Z] raw_err={raw_rel_z_mm:.1f}mm, cmd_err={rel_z_mm:.1f}mm, "
            f"tol={z_tol_mm:.1f}mm, cur_z={cur_h:.1f}mm, "
            f"cmd_inv={self._get_axis_invert('Z')}, "
            f"vis_inv={self._get_visual_sign_invert('Z')}"
        )

        now = time.monotonic()

        # ------------------------------------------------------------
        # 1. 進入容許帶：立即停止；維持一段時間後才完成
        # ------------------------------------------------------------
        if abs(raw_rel_z_mm) <= z_tol_mm:
            self._soft_stop_arm()
            self._last_cmd_z = None

            if self._z_in_tolerance_since is None:
                self._z_in_tolerance_since = now
                rospy.loginfo(
                    f"[Z HOLD] 進入容許帶，立即停止。"
                    f"err={raw_rel_z_mm:.1f}mm, "
                    f"需穩定 {stable_time_sec:.2f}s"
                )

            held_sec = now - self._z_in_tolerance_since

            rospy.loginfo_throttle(
                0.25,
                f"[Z HOLD] err={raw_rel_z_mm:.1f}mm, "
                f"held={held_sec:.2f}/{stable_time_sec:.2f}s"
            )

            if held_sec >= stable_time_sec:
                rospy.loginfo(
                    f"[Z] SUCCESS：連續 {held_sec:.2f}s 位於容許帶內，"
                    f"err={raw_rel_z_mm:.1f}mm, tol={z_tol_mm:.1f}mm"
                )
                self._soft_stop_arm()
                self._z_inited = False
                self._z_in_tolerance_since = None
                self._last_cmd_z = None
                return True

            return False

        # 離開容許帶，重新計時。
        if self._z_in_tolerance_since is not None:
            rospy.logwarn(
                f"[Z HOLD] 離開容許帶，穩定計時歸零。"
                f"err={raw_rel_z_mm:.1f}mm, tol={z_tol_mm:.1f}mm"
            )
        self._z_in_tolerance_since = None

        # ------------------------------------------------------------
        # 2. 尚未進入容許帶：採小步限幅部署
        #    rel_z > 0 時，預設往 logical Z 負方向修正。
        # ------------------------------------------------------------
        tgt_h, step = self._make_limited_axis_target(
            "Z",
            cur_h,
            rel_z_mm,
            sign_positive_err_to_positive_target=False
        )

        rospy.loginfo_throttle(
            0.5,
            f"[Z] step={step:.1f}mm, logical_target={tgt_h:.1f}mm, "
            f"cur={cur_h:.1f}mm, speed={speed}"
        )

        self.DeadMoveZ(
            tgt_h,
            z_tolerance=float(self.Subscriber.axis_motion_tolerance_mm),
            speed=speed
        )

        return False

    def fnControlArmBasedOnFruitX(self, speed=800):
        """
        X 軸視覺對位。

        完成條件：
        1. fruit_x_m 與 x_circular_saw_target 的誤差進入
           x_tolerance_mm 後，立即送停止命令。
        2. 在停止狀態下，連續維持 zx_stable_time_sec 秒都在容許帶內。
        3. 中途只要離開容許帶，穩定計時立即歸零。

        circular_saw_allow_retract=False 時：
        若已經超過目標且超出容許帶，立即停止、記錄超過量並直接完成；
        後續盲伸距離會扣除該超過量，不執行反向收回。
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn_throttle(1.0, "X: 尚未接收到手臂狀態")
            return False

        if not self.TFConfidence():
            # 視覺信心不足時，不允許沿用舊命令繼續前進。
            self._soft_stop_arm()
            self._x_in_tolerance_since = None
            self._last_cmd_x = None
            return False

        if not getattr(self, "_x_inited", False):
            self._x_inited = True
            self._x_in_tolerance_since = None
            self._last_cmd_x = None
            self.x_overshoot_mm = 0.0

            self.cmd_vel._init_targets_from_status_once()

            # 鎖定 Z 階段最後下達的目標，X 階段不可再用 Z 回授覆寫。
            self._x_hold_z_mm = float(self.cmd_vel.target_h_mm)
            self.cmd_vel.target_h_mm = self._x_hold_z_mm

            rospy.loginfo(f"[X] 開始對齊 X 軸，鎖定最後 Z 命令={self._x_hold_z_mm:.1f}mm, 目前 Z 回授={float(st.z_pos):.1f}mm")

        x_tol_mm = max(0.0, float(self.Subscriber.x_tolerance_mm))
        stable_time_sec = max(0.0,float(self.Subscriber.zx_stable_time_sec))

        x_stop_m = float(self.Subscriber.x_circular_saw_target)
        allow_retract = bool(self.Subscriber.circular_saw_allow_retract)

        fruit_x_m = float(self.Subscriber.marker_2d_pose_x)
        cur_l = float(st.x_pos)

        # 正值：目標仍然太遠；負值：已經比停止距離更近。
        raw_err_mm = (fruit_x_m - x_stop_m) * 1000.0

        # 視覺方向設定只影響移動方向，不影響容許帶判斷。
        rel_x_mm = (
            -raw_err_mm
            if self._get_visual_sign_invert("X")
            else raw_err_mm
        )

        rospy.loginfo_throttle(
            0.5,
            f"[X] fruit_x={fruit_x_m:.3f}m, stop={x_stop_m:.3f}m, "
            f"raw_err={raw_err_mm:.1f}mm, cmd_err={rel_x_mm:.1f}mm, "
            f"tol={x_tol_mm:.1f}mm, cur_x={cur_l:.1f}mm, "
            f"allow_retract={allow_retract}, "
            f"cmd_inv={self._get_axis_invert('X')}, "
            f"vis_inv={self._get_visual_sign_invert('X')}"
        )

        now = time.monotonic()

        # ------------------------------------------------------------
        # 1. 進入容許帶：立即停止；維持一段時間後才完成
        # ------------------------------------------------------------
        if abs(raw_err_mm) <= x_tol_mm:
            self._soft_stop_arm()
            self._last_cmd_x = None

            if self._x_in_tolerance_since is None:
                self._x_in_tolerance_since = now
                rospy.loginfo(
                    f"[X HOLD] 進入容許帶，立即停止。"
                    f"err={raw_err_mm:.1f}mm, "
                    f"需穩定 {stable_time_sec:.2f}s"
                )

            held_sec = now - self._x_in_tolerance_since

            rospy.loginfo_throttle(
                0.25,
                f"[X HOLD] err={raw_err_mm:.1f}mm, "
                f"held={held_sec:.2f}/{stable_time_sec:.2f}s"
            )

            if held_sec >= stable_time_sec:
                # 即使在容許帶內，若最終位置已比停止距離更近，
                # 仍記錄殘餘超過量，供盲伸距離扣除。
                self.x_overshoot_mm = max(0.0, -raw_err_mm)

                rospy.loginfo(
                    f"[X] SUCCESS：連續 {held_sec:.2f}s 位於容許帶內，"
                    f"err={raw_err_mm:.1f}mm, tol={x_tol_mm:.1f}mm, "
                    f"recorded_overshoot={self.x_overshoot_mm:.1f}mm"
                )
                self._soft_stop_arm()
                self._x_inited = False
                self._x_in_tolerance_since = None
                self._last_cmd_x = None
                return True

            return False

        # 離開容許帶，重新計時。
        if self._x_in_tolerance_since is not None:
            rospy.logwarn(
                f"[X HOLD] 離開容許帶，穩定計時歸零。"
                f"err={raw_err_mm:.1f}mm, tol={x_tol_mm:.1f}mm"
            )
        self._x_in_tolerance_since = None

        # 目標在相機後方屬於異常資料，不可直接判定完成。
        if fruit_x_m < 0.0:
            self._soft_stop_arm()
            self._last_cmd_x = None
            rospy.logwarn_throttle(
                1.0,
                f"[X] fruit_x={fruit_x_m:.3f}m < 0，立即停止並等待有效姿態。"
            )
            return False

        # 已超過停止距離且超出容許帶；若禁止反向修正：
        # 1. 立即停止。
        # 2. 鎖定目前超過量。
        # 3. 直接視為 X 對位完成，讓流程繼續啟動鋸片與盲伸。
        # 4. 後續盲伸距離會扣除這個超過量。
        if raw_err_mm < -x_tol_mm and not allow_retract:
            self._soft_stop_arm()
            self.x_overshoot_mm = max(0.0, -raw_err_mm)

            configured_blind_mm = abs(
                float(self.Subscriber.circular_saw_blind_extend_length)
            )
            effective_blind_mm = max(
                0.0,
                configured_blind_mm - self.x_overshoot_mm
            )

            rospy.logwarn(
                f"[X OVERSHOOT] 已超過停止距離 "
                f"{self.x_overshoot_mm:.1f}mm，且不允許後退。"
                f"記錄超過量並直接完成 X 對位；"
                f"後續盲伸 {configured_blind_mm:.1f} - "
                f"{self.x_overshoot_mm:.1f} = "
                f"{effective_blind_mm:.1f}mm。"
            )

            self._x_inited = False
            self._x_in_tolerance_since = None
            self._last_cmd_x = None
            return True

        # ------------------------------------------------------------
        # 2. 尚未進入容許帶：採小步限幅部署
        #    raw_err > 0 時，預設增加 logical X。
        # ------------------------------------------------------------
        tgt_l, step = self._make_limited_axis_target(
            "X",
            cur_l,
            rel_x_mm,
            sign_positive_err_to_positive_target=True
        )

        rospy.loginfo_throttle(
            0.5,
            f"[X] step={step:.1f}mm, logical_target={tgt_l:.1f}mm, "
            f"cur={cur_l:.1f}mm, speed={speed}"
        )

        self.DeadMoveX(
            tgt_l,
            x_tolerance=float(self.Subscriber.axis_motion_tolerance_mm),
            speed=speed
        )

        return False

    def fnBlindExtendArm(self, speed=2000):
        """
        盲伸，非阻塞。

        流程：
        1. 讀取 X 視覺對位鎖定的 x_overshoot_mm。
        2. 有效盲伸量 = max(0, circular_saw_blind_extend_length - x_overshoot_mm)。
        3. 第一次呼叫時，鎖定 target = 目前 x_pos + 有效盲伸量。
        4. 後續每次呼叫都朝同一個 target 前進。
        5. 到達 target 後回傳 True。
        6. 完成後清除內部狀態，避免下次流程誤用。

        重要修正：
        - 第一次進入盲伸階段時，必須清除 _last_cmd_x。
        - 因為上一階段 X 視覺對位可能留下 _last_cmd_x，
        會讓 _dead_move_axis() 誤判舊命令還在執行，
        導致盲伸 DeadMoveX() 不發布新的 /cmd_circular_saw。
        """
        # 若本輪已經完成盲伸，直接回 True
        if getattr(self, "blind_extend_completed", False):
            return True

        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logerr_throttle(1.0, "fnBlindExtendArm: 尚未取得 arm 狀態")
            return False

        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)

        configured_extra_mm = abs(
            float(self.Subscriber.circular_saw_blind_extend_length)
        )
        overshoot_mm = max(
            0.0,
            float(getattr(self, "x_overshoot_mm", 0.0))
        )
        effective_extra_mm = max(
            0.0,
            configured_extra_mm - overshoot_mm
        )

        # ------------------------------------------------------------
        # 第一次進入盲伸階段：鎖定補償後目標，只算一次
        # ------------------------------------------------------------
        if not hasattr(self, "_blind_extend_target"):
            cur_l0 = float(st.x_pos)

            # 正值代表往前盲伸；超過量只會減少盲伸，不允許變成反向收回。
            raw_tgt = cur_l0 + effective_extra_mm
            self._blind_extend_target = self._saturate(raw_tgt, L_MIN, L_MAX)
            self._blind_extend_effective_extra_mm = effective_extra_mm

            # 關鍵修正：
            # 新 motion phase 開始，不能沿用視覺 X 對位留下的 _last_cmd_x
            self._last_cmd_x = None

            rospy.loginfo(
                f"BlindExtend(init): cur0={cur_l0:.1f}mm, "
                f"configured={configured_extra_mm:.1f}mm, "
                f"x_overshoot={overshoot_mm:.1f}mm, "
                f"effective={effective_extra_mm:.1f}mm "
                f"→ target={self._blind_extend_target:.1f}mm"
            )

        cur_l = float(st.x_pos)
        err = self._blind_extend_target - cur_l

        rospy.loginfo_throttle(
            0.5,
            f"[BlindExtend] cur={cur_l:.1f}mm, "
            f"target={self._blind_extend_target:.1f}mm, "
            f"err={err:.1f}mm, last_cmd_x={self._last_cmd_x}"
        )

        # ------------------------------------------------------------
        # 每次都朝同一個 target 前進
        # ------------------------------------------------------------
        done = self.DeadMoveX(
            self._blind_extend_target,
            x_tolerance=5.0,
            speed=speed
        )

        if done:
            rospy.loginfo(
                f"盲伸完成：到達 {self._blind_extend_target:.1f} mm"
            )

            self.blind_extend_completed = True

            # 清除 target，避免下一輪流程誤用
            if hasattr(self, "_blind_extend_target"):
                delattr(self, "_blind_extend_target")
            if hasattr(self, "_blind_extend_effective_extra_mm"):
                delattr(self, "_blind_extend_effective_extra_mm")

            # 清除 X 指令快取，讓下一階段可以正常發新命令
            self._last_cmd_x = None

            return True

        return False

    def _dead_rotate_angle(self, target_deg, tolerance_deg, speed=2000):
        """
        共用鋸片角度盲轉邏輯（一步到位、不變速）：
        - target_deg: 目標角度 (deg)
        - tolerance_deg: 容許誤差 (deg)
        - 內部會使用 self._last_cmd_angle 避免重複下同一筆 command
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn("ANGLE: 尚未接收到手臂狀態，等待中...")
            return False

        cur = float(st.angle)       # 目前鋸片角度 (deg)
        d = target_deg - cur

        # 1) 現在位置已經在目標附近 → 視為完成，送一次 Stop
        if abs(d) <= tolerance_deg:
            self._soft_stop_arm()
            self._last_cmd_angle = None
            return True

        last_cmd = self._last_cmd_angle
        MIN_CMD_DELTA = 1  # 目標角度與最後一次目標差 < 1deg 就不重發

        if last_cmd is not None:
            remaining_to_last = last_cmd - cur
            # 2a) 離上一個目標還很遠 → 認為上一筆命令還在執行，不要重發
            if abs(remaining_to_last) > tolerance_deg:
                return False

            # 2b) 已經快到上一個目標，且新目標跟上一個幾乎一樣 → 不必重發
            if abs(target_deg - last_cmd) < MIN_CMD_DELTA:
                return False

        # 3) 確定需要新目標 → 發 command
        self.cmd_vel.fnSawRotate(target_deg, speed=speed)
        self._last_cmd_angle = target_deg
        return False

    def DeadRotateAngle(self, target_angle, angle_tolerance=1.0, speed=2000):
        """
        一次性盲轉到指定角度：
        - target_angle: 目標角度 (deg)
        - angle_tolerance: 容許誤差 (deg)
        """
        C_MIN = float(self.Subscriber.circular_saw_min_angle)
        C_MAX = float(self.Subscriber.circular_saw_max_angle)
        target_angle = self._saturate(float(target_angle), C_MIN, C_MAX)

        return self._dead_rotate_angle(
            target_angle,
            angle_tolerance,
            speed=speed
        )

    def fnAlignSawToMarker(self, angle_tolerance=2.0, speed=2000):
        """
        讓鋸片角度與果串角度達成對齊
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn_throttle(1.0, "ANGLE: 尚未接收到手臂狀態")
            return False
        
        if self.TFConfidence():
            if not getattr(self, "_align_saw_inited", False):
                self._align_saw_inited = True
                self._align_success_cnt = 0

                object_angle_rad = float(-self.Subscriber.green_line_angle_rad)
                object_angle_deg = math.degrees(object_angle_rad)
                C_MIN = float(self.Subscriber.circular_saw_min_angle)
                C_MAX = float(self.Subscriber.circular_saw_max_angle)
                
                self._align_target_angle = self._saturate(object_angle_deg, C_MIN, C_MAX)
                rospy.loginfo(f"[ANGLE] align saw to bunch: tilt={object_angle_deg:.2f}deg → target={self._align_target_angle:.2f}deg")

                self.cmd_vel.fnSawRotate(self._align_target_angle, speed=speed)
                return False 

            cur_angle = float(st.angle)
            self.err = self._align_target_angle - cur_angle
        else:
            return False
        
        if abs(self.err) <= angle_tolerance:
            self._align_success_cnt += 1
            if self._align_success_cnt >= 5:  
                rospy.loginfo(f"[ANGLE] SUCCESS cur={cur_angle:.2f}deg target={self._align_target_angle:.2f}deg")
                self._align_saw_inited = False
                return True
        else:
            self._align_success_cnt = 0

        return False
    
    def SawRunStop(self, saw_speed, speed=2000):
        """
        等待鋸片達到目標轉速 (非阻塞式，無超時)
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn_throttle(1.0, "Saw: 尚未接收到手臂狀態")
            return False
        
        if not getattr(self, "_saw_inited", False):
            self._saw_inited = True
            rospy.loginfo(f"[SAW] 設定鋸片目標轉速: {saw_speed}")

        self.cmd_vel.fnSawRunStop(saw_speed)

        if abs(float(st.saw_speed) - saw_speed) <= 5.0:
            self._saw_inited = False
            return True
            
        return False
    
    def CircularStop(self):
        self.cmd_vel.fnCircularStop()
        return True

    def TFConfidence(self):#判斷TF是否可信
        # rospy.loginfo('shelf_detection: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_detection))
        # rospy.loginfo('shelf_confidence: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_confidence))
        # rospy.loginfo('confidence_minimum: {0}'.format(self.Subscriber.confidence_minimum))
        if self.Subscriber.sub_detectionConfidence.state != "STABLE":
            self._soft_stop_arm()
            self.wait_time = 0
            return False
        
        low = (not self.Subscriber.sub_detectionConfidence.pose_detection) or \
          (self.Subscriber.sub_detectionConfidence.pose_confidence < self.Subscriber.confidence_minimum)
        if low:
            self._soft_stop_arm()
            self.wait_time = 0
            return False
        
        if self.wait_time >= 10:
            return True
        else:
            self.wait_time += 1
            return False
     
class cmd_vel():
    def __init__(self, Subscriber):
        self.Subscriber = Subscriber
        self.pub_cmd_vel = self.Subscriber.pub_cmd_vel
        self.arm_control_pub = self.Subscriber.arm_control_pub
        self.front = False

        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        L_MIN = float(self.Subscriber.circular_saw_min_length)

        self.target_h_mm = Z_MIN      # 目標高度
        self.target_l_mm = L_MIN      # 目標長度
        self._target_inited = False   # 是否已經從狀態初始化過

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.angular.z > 0.2:
            twist.angular.z =0.2
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2
        if twist.linear.x > 0 and twist.linear.x < 0.02:
            twist.linear.x =0.01
        elif twist.linear.x < 0 and twist.linear.x > -0.02:
            twist.linear.x =-0.01   

        if twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.09:
            twist.angular.z =0.1
        elif twist.angular.z < 0 and twist.angular.z > -0.09:
            twist.angular.z =-0.1
        self.pub_cmd_vel.publish(twist)

    def fnTurn(self, Kp=0.2, theta=0.):
        twist = Twist()
        twist.angular.z = Kp * theta
        self.cmd_pub(twist)

    def fnGoStraight(self, Kp=0.2, v=0.):
        twist = Twist()
        twist.linear.x = Kp * v

        self.cmd_pub(twist)

    def fnStopVehicle(self):
        """立即發布零速 Twist，停止底盤且覆蓋上一筆 parking 速度。"""
        self.pub_cmd_vel.publish(Twist())

    def fnGoBack(self):
        twist = Twist()
        twist.linear.x = -0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        self.cmd_pub(twist)

    def fnTrackMarker(self, theta, kp):
        twist = Twist()
        twist.linear.x = 0.05
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0

        twist.angular.z = kp * theta
        self.cmd_pub(twist)

    def _saturate(self, v, lo, hi):
        return max(lo, min(hi, v))
    
    def latch_axes_from_status(self, latch_x=True, latch_z=True, latch_angle=True):
        """
        只在「控制階段切換」時，將指定軸的實際回授鎖成保持目標。
        重要：此函式不可在每一筆 X/Z 命令前重複呼叫，否則回授延遲、
        量化誤差或背隙會被反覆寫回絕對位置命令，造成非控制軸漂移。
        """
        self._init_targets_from_status_once()

        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is None:
            rospy.logwarn_throttle(1.0, "[ARM HOLD] 尚未取得手臂回授，無法鎖定軸位置")
            return False

        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        Z_MAX = float(self.Subscriber.circular_saw_max_height)
        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)
        A_MIN = float(self.Subscriber.circular_saw_min_angle)
        A_MAX = float(self.Subscriber.circular_saw_max_angle)

        if latch_x:
            self.target_l_mm = float(int(round(self._saturate(float(st.x_pos), L_MIN, L_MAX))))
        if latch_z:
            self.target_h_mm = float(int(round(self._saturate(float(st.z_pos), Z_MIN, Z_MAX))))
        if latch_angle:
            self.target_angle = self._saturate(float(st.angle), A_MIN, A_MAX)

        rospy.loginfo(
            f"[ARM HOLD LATCH] X={self.target_l_mm:.1f}mm, "
            f"Z={self.target_h_mm:.1f}mm, angle={self.target_angle:.1f}deg, "
            f"latch_x={latch_x}, latch_z={latch_z}, latch_angle={latch_angle}"
        )
        return True

    def _init_targets_from_status_once(self):
        """
        第一次使用時，從 circular_saw_state 把實際位置抄成目標值，
        之後就都用 target_h_mm / target_l_mm 自己維護。
        """
        if self._target_inited:
            return

        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        Z_MAX = float(self.Subscriber.circular_saw_max_height)
        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)
        Angle_MIN = float(self.Subscriber.circular_saw_min_angle)
        Angle_MAX = float(self.Subscriber.circular_saw_max_angle)
        saw_speed_max = float(self.Subscriber.circular_saw_max_saw_speed)

        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            h = float(getattr(st, "z_pos", 0.0))        # Z 位置（mm，0..350）
            l = float(getattr(st, "x_pos", 0.0))        # X 位置（mm，0..400）
            angle = float(getattr(st, "angle", 0.0))        # 鋸片角度（deg）
            saw_speed = float(getattr(st, "saw_speed", 0.0))
            enable_motor1 = bool(getattr(st, "en1", False))
            enable_motor2 = bool(getattr(st, "en2", False))
            enable_motor3 = bool(getattr(st, "en3", False))
            voltage = float(getattr(st, "voltage", 0.0))
        else:
            # 沒有回報，就用安全預設值
            h = 0.0
            l = 0.0
            angle = 0.0
            saw_speed = 0.0
            enable_motor1 = False
            enable_motor2 = False
            enable_motor3 = False
            voltage = 0.0

        self.target_h_mm = self._saturate(h, Z_MIN, Z_MAX)
        self.target_l_mm = self._saturate(l, L_MIN, L_MAX)
        self.target_angle = self._saturate(angle, Angle_MIN, Angle_MAX)
        self.target_saw_speed = int(round(self._saturate(saw_speed, 0.0, float(saw_speed_max))))
        self.target_enable_motor1 = enable_motor1
        self.target_enable_motor2 = enable_motor2
        self.target_enable_motor3 = enable_motor3
        self.target_voltage = voltage

        self._target_inited = True

    def fnSawUpDown(self, target_mm, speed=2000):
        """只動 Z 軸"""
        self._init_targets_from_status_once()

        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        Z_MAX = float(self.Subscriber.circular_saw_max_height)
        self.target_h_mm = float(int(round(self._saturate(float(target_mm), Z_MIN, Z_MAX))))

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        
        msg.x_speed = int(speed)
        msg.z_speed = int(speed)
        msg.saw_speed = int(self.target_saw_speed)
        msg.stop = False

        self.arm_control_pub.publish(msg)

        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            z_feedback = float(st.z_pos)

            rospy.loginfo_throttle(0.5,f"[X ABS CMD] X_cmd={msg.x_pos}mm, Z_hold={msg.z_pos}mm, Z_feedback={z_feedback:.1f}mm, Z_track_err={z_feedback - float(msg.z_pos):+.1f}mm")

    def fnSawForwardBackward(self, target_mm, speed=2000):
        """只動 X 軸"""
        self._init_targets_from_status_once()

        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)
        self.target_l_mm = float(int(round(self._saturate(float(target_mm), L_MIN, L_MAX))))

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        
        msg.x_speed = int(speed)
        msg.z_speed = int(speed)
        msg.saw_speed = int(self.target_saw_speed)
        msg.stop = False

        self.arm_control_pub.publish(msg)

    def fnSawRotate(self, target_angle, speed=2000):
        """只轉角度；X/Z 沿用已鎖定的命令目標。"""
        self._init_targets_from_status_once()

        C_MIN = float(self.Subscriber.circular_saw_min_angle)
        C_MAX = float(self.Subscriber.circular_saw_max_angle)

        self.target_angle = self._saturate(float(target_angle),C_MIN,C_MAX)

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)

        msg.x_speed = int(speed)
        msg.z_speed = int(speed)
        msg.saw_speed = int(self.target_saw_speed)
        msg.stop = False

        self.arm_control_pub.publish(msg)

    def fnSawRunStop(self, saw_speed=0, speed=2000):
        """只控制鋸片轉速"""
        self._init_targets_from_status_once()

        S_MIN = 0
        S_MAX = float(self.Subscriber.circular_saw_max_saw_speed)
        self.target_saw_speed = self._saturate(int(saw_speed), S_MIN, S_MAX)

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        
        msg.x_speed = int(speed)
        msg.z_speed = int(speed)
        msg.saw_speed = int(self.target_saw_speed)
        msg.stop = False

        self.arm_control_pub.publish(msg)

    def fnPauseArm(self, speed=2000):
        """
        不再前進，把所有軸的目標鎖死在現在的物理位置
        保留當前鋸片轉速不變
        """
        self._init_targets_from_status_once()

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        
        msg.x_speed = int(speed)
        msg.z_speed = int(speed)
        msg.saw_speed = int(self.target_saw_speed)
        msg.stop = True

        self.arm_control_pub.publish(msg)

    def fnCircularStop(self):
        """
        全機強制停止：速度全歸零，鋸片轉速歸零，並且送出 stop = True 的強制切斷訊號。
        """
        self._init_targets_from_status_once()
        
        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            self.target_l_mm = st.x_pos
            self.target_h_mm = st.z_pos
            self.target_angle = st.angle

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        msg.x_speed = 0
        msg.z_speed = 0
        msg.saw_speed = 0
        msg.stop = True

        self.arm_control_pub.publish(msg)
