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
        # other
        self.check_wait_time = 0
        self.wait_time = 0
        self.is_triggered = False

    def _saturate(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _soft_stop_arm(self):
        self.cmd_vel.fnPauseArm()
        self._last_cmd_x = None
        self._last_cmd_z = None
        self._last_cmd_angle = None


    def _reset_axis_cmd_cache(self):
        """
        清除上一個 motion phase 的目標記憶。
        """
        self._last_cmd_x = None
        self._last_cmd_z = None
        self._last_cmd_angle = None
        self._axis_probe_x = None
        self._axis_probe_z = None
        self._visual_probe_x = None
        self._visual_probe_z = None

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

    def _auto_check_axis_direction(self, axis_name, cur):
        """
        若實際回授方向與預期方向相反，自動切換 X/Z command invert。
        """
        auto_attr = "x_auto_invert_enable" if axis_name == "X" else "z_auto_invert_enable"
        if not bool(getattr(self.Subscriber, auto_attr, True)):
            return

        probe_attr = "_axis_probe_" + axis_name.lower()
        probe = getattr(self, probe_attr, None)
        if not probe:
            return

        now = time.time()
        delay = float(getattr(self.Subscriber, "axis_auto_delay_sec", 0.35))
        min_move = float(getattr(self.Subscriber, "axis_auto_min_move_mm", 2.0))

        if (now - probe["t0"]) < delay:
            return

        start = float(probe["start"])
        target = float(probe["target"])
        cur = float(cur)

        expected = target - start
        moved = cur - start

        if abs(expected) < 1e-6:
            setattr(self, probe_attr, None)
            return

        # 還沒動夠，不判斷，避免馬達剛起步時誤判。
        if abs(moved) < min_move:
            return

        if np.sign(moved) != np.sign(expected):
            old = self._get_axis_invert(axis_name)
            self._set_axis_invert(axis_name, not old)

            if axis_name == "X":
                self._last_cmd_x = None
            else:
                self._last_cmd_z = None

            rospy.logwarn(
                f"[{axis_name} AUTO] actual motion opposite to expected: "
                f"start={start:.1f}, cur={cur:.1f}, logical_target={target:.1f}, "
                f"moved={moved:.1f}, expected={expected:.1f}. "
                f"Flip invert and resend next cycle."
            )

        setattr(self, probe_attr, None)

    def _remember_axis_probe(self, axis_name, start, target):
        probe_attr = "_axis_probe_" + axis_name.lower()
        setattr(self, probe_attr, {
            "t0": time.time(),
            "start": float(start),
            "target": float(target),
            "invert": self._get_axis_invert(axis_name),
        })

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

    def _filtered_axis_error_mm(self, axis_name, raw_err_mm):
        """
        對 X/Z 視覺誤差做 EMA，降低深度/TF 抖動。
        """
        if axis_name == "X":
            attr = "_x_err_filt_mm"
            alpha = float(getattr(self.Subscriber, "x_err_ema_alpha", 0.35))
        else:
            attr = "_z_err_filt_mm"
            alpha = float(getattr(self.Subscriber, "z_err_ema_alpha", 0.35))

        if not hasattr(self, attr):
            setattr(self, attr, float(raw_err_mm))
        else:
            prev = float(getattr(self, attr))
            setattr(self, attr, alpha * float(raw_err_mm) + (1.0 - alpha) * prev)

        return float(getattr(self, attr))

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

    def _auto_check_visual_error_direction(self, axis_name, raw_err_mm, cur_mm):
        """
        觀察手臂移動後視覺誤差是否反而變大。
        若明顯變大，代表 PBVS 視覺誤差與手臂 logical 方向可能相反，
        自動切換 x_visual_sign_invert / z_visual_sign_invert。
        """
        auto_attr = "x_auto_visual_sign_enable" if axis_name == "X" else "z_auto_visual_sign_enable"
        if not bool(getattr(self.Subscriber, auto_attr, True)):
            return

        probe_attr = "_visual_probe_" + axis_name.lower()
        probe = getattr(self, probe_attr, None)
        if not probe:
            return

        now = time.time()
        delay = float(getattr(self.Subscriber, "axis_auto_delay_sec", 0.35))
        min_move = float(getattr(self.Subscriber, "axis_auto_min_move_mm", 2.0))
        worsen_margin = float(getattr(self.Subscriber, "axis_auto_worsen_margin_mm", 6.0))

        if (now - probe["t0"]) < delay:
            return

        moved = abs(float(cur_mm) - float(probe["start_cur"]))
        if moved < min_move:
            return

        old_abs = abs(float(probe["start_err"]))
        new_abs = abs(float(raw_err_mm))

        if new_abs > old_abs + worsen_margin:
            old = self._get_visual_sign_invert(axis_name)
            self._set_visual_sign_invert(axis_name, not old)

            if axis_name == "X":
                self._last_cmd_x = None
            else:
                self._last_cmd_z = None

            rospy.logwarn(
                f"[{axis_name} AUTO] visual error got worse after motion: "
                f"err_abs {old_abs:.1f} -> {new_abs:.1f}, moved={moved:.1f}mm. "
                f"Flip visual sign and resend next cycle."
            )

        setattr(self, probe_attr, None)

    def _remember_visual_probe(self, axis_name, start_err_mm, start_cur_mm):
        probe_attr = "_visual_probe_" + axis_name.lower()
        setattr(self, probe_attr, {
            "t0": time.time(),
            "start_err": float(start_err_mm),
            "start_cur": float(start_cur_mm),
        })

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
        
    def fnSeqMovingNearbyParkingLot(self,desired_dist_threshold):
        Kp = 0.2
        err = 0.05
        if self.current_nearby_sequence != self.previous_nearby_sequence:
            rospy.loginfo('current_nearby_sequence {0}'.format(self.NearbySequence(self.current_nearby_sequence)))
            self.previous_nearby_sequence = self.current_nearby_sequence  # 更新 previous_sequence

        if self.current_nearby_sequence == self.NearbySequence.initial_dist.value:
            if self.TFConfidence():
                self.initial_robot_pose_theta = self.Subscriber.robot_2d_theta
                self.initial_robot_pose_x = self.Subscriber.robot_2d_pose_x
                self.initial_robot_pose_y = self.Subscriber.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(5)
                self.initial_marker_pose_x = self.Subscriber.marker_2d_pose_x
                self.initial_marker_pose_y = self.Subscriber.marker_2d_pose_y
                self.desired_dist_diff = abs(self.initial_marker_pose_x) - desired_dist_threshold
                rospy.loginfo(f'desired_dist_diff:{self.desired_dist_diff}')
                if abs(self.initial_marker_pose_x) <= desired_dist_threshold:
                    return True
                else:
                    # 依 TF tree 判斷相機側別：
                    # base_link +Y = 左側，-Y = 右側
                    camera_side, camera_y = self.Subscriber.get_camera_side_from_tf()

                    if camera_side == "right":
                        self.first_turn_deg = 90
                    elif camera_side == "left":
                        self.first_turn_deg = -90
                    else:
                        # 相機接近車體中心線時，預設沿用右側策略
                        rospy.logwarn(f"[PBVS] camera_y={camera_y:.3f}m 接近中心線，預設使用 right-side turn strategy")
                        self.first_turn_deg = 90

                    self.second_turn_deg = -self.first_turn_deg
                    rospy.loginfo(
                        f"[PBVS] camera_side from TF = {camera_side}, "
                        f"camera_y={camera_y:.3f}m, "
                        f"first_turn_deg={self.first_turn_deg}, "
                        f"second_turn_deg={self.second_turn_deg}"
                    )

                    self.current_nearby_sequence = self.NearbySequence.turn_right.value
            # 若無觀測信心則維持等待
            return False

        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            # 用 +90 或 -90
            if self.fnseqDeadReckoningAngle(self.first_turn_deg):
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
            return False

        # 前後調整階段
        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            self.Subscriber.fnDetectionAllowed(pose_detection=False, det_select_mode="nearest_depth")
            fwd = -(self.desired_dist_diff + err + float(self.Subscriber.spin_forward_comp))
            if self.fnseqDeadReckoning(fwd):
                self.current_nearby_sequence = self.NearbySequence.turn_left.value
            return False

        elif self.current_nearby_sequence == self.NearbySequence.turn_left.value:
            # 轉回原始朝向：-first_turn_deg
            if self.fnseqDeadReckoningAngle(self.second_turn_deg):
                self.current_nearby_sequence = self.NearbySequence.initial_marker.value
                return True
            return False

        elif self.current_nearby_sequence == self.NearbySequence.initial_marker.value:
            if self.TFConfidence():
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    self.current_nearby_sequence = self.NearbySequence.initial_dist.value
                    return True
                else:
                    self.check_wait_time += 1
            else:
                self.check_wait_time = 0
            return False

        return False

    def fnSeqParking(self, tolerance, kp):
        # rospy.loginfo(f'fnSeqParking: {self.marker_2d_pose_y}')
        if self.TFConfidence():
            if abs(self.Subscriber.marker_2d_pose_y) > tolerance:
                # 若偏差超過設定距離，透過前後移動來修正位置
                self.cmd_vel.fnGoStraight(kp, self.Subscriber.marker_2d_pose_y)
            else:
                # 偏差在容忍範圍內，停止前後運動
                self._soft_stop_arm()
                if self.check_wait_time > 20:
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time += 1
            return False
        else:
            self.check_wait_time = 0
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

        若偵測到實際 state 往預期相反方向移動，會自動切換對應軸的 invert。
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn(f"{axis_name}: 尚未接收到手臂狀態，等待中...")
            return False

        lo, hi = self._axis_limits(axis_name)
        target = self._saturate(float(target), lo, hi)

        cur = float(get_cur_fn(st))

        # 自動方向檢查：若上一筆命令造成反向移動，會 flip invert 並清 last command。
        self._auto_check_axis_direction(axis_name, cur)

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

            # 距離上一次 logical 目標還很遠 → 認為上一筆命令還在執行中，不重發
            if abs(remaining_to_last) > tolerance_mm:
                return False

            if abs(target - float(last_cmd)) < MIN_CMD_DELTA:
                return False

        # --- 3) 確定需要新目標 → 做 command mapping 後發送 ---
        controller_target = self._to_controller_axis_target(axis_name, target)

        rospy.loginfo_throttle(
            0.5,
            f"[{axis_name} CMD MAP] logical={target:.1f}mm -> controller={controller_target:.1f}mm, "
            f"cur={cur:.1f}mm, invert={self._get_axis_invert(axis_name)}"
        )

        send_axis_cmd_fn(int(round(controller_target)), speed)
        self._remember_axis_probe(axis_name, cur, target)

        if axis_name == "X":
            self._last_cmd_x = target
        else:
            self._last_cmd_z = target

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

        設計重點：
        1. 不再一次部署完整 rel_z，改成小步限幅部署，避免震盪。
        2. 若 controller command 方向與 state 方向相反，DeadMoveZ 會自動切換 z_cmd_invert。
        3. 若視覺誤差在移動後反而變大，會自動切換 z_visual_sign_invert。
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn_throttle(1.0, "Z: 尚未接收到手臂狀態")
            return False

        if not self.TFConfidence():
            return False

        if not getattr(self, "_z_inited", False):
            self._z_inited = True
            self._z_success_cnt = 0
            self._z_err_filt_mm = 0.0
            self._last_cmd_z = None
            self._axis_probe_z = None
            self._visual_probe_z = None
            rospy.loginfo("[Z] 開始對齊 Z 軸")

        z_tol_mm = float(self.Subscriber.z_tolerance_mm)
        raw_rel_z_mm = float(self.Subscriber.marker_2d_pose_z) * 1000.0
        cur_h = float(st.z_pos)

        # 若上一次小步移動後視覺誤差變大，自動反轉視覺誤差符號。
        self._auto_check_visual_error_direction("Z", raw_rel_z_mm, cur_h)

        # 視覺方向自動修正後的 error。
        rel_z_for_ctrl = -raw_rel_z_mm if self._get_visual_sign_invert("Z") else raw_rel_z_mm
        rel_z_mm = self._filtered_axis_error_mm("Z", rel_z_for_ctrl)

        rospy.loginfo_throttle(
            0.5,
            f"[Z] raw={raw_rel_z_mm:.1f}mm, ctrl={rel_z_for_ctrl:.1f}mm, "
            f"filt={rel_z_mm:.1f}mm, tol={z_tol_mm:.1f}mm, cur_z={cur_h:.1f}mm, "
            f"cmd_inv={self._get_axis_invert('Z')}, vis_inv={self._get_visual_sign_invert('Z')}"
        )

        # ------------------------------------------------------------
        # 1. 如果 Z 已經在容許範圍內，連續穩定數幀後視為完成
        # ------------------------------------------------------------
        if abs(raw_rel_z_mm) <= z_tol_mm or abs(rel_z_mm) <= z_tol_mm:
            self._z_success_cnt += 1

            if self._z_success_cnt >= 3:
                rospy.loginfo(
                    f"[Z] SUCCESS raw={raw_rel_z_mm:.1f}mm, filt={rel_z_mm:.1f}mm "
                    f"(tol={z_tol_mm:.1f}mm)"
                )

                self._z_inited = False
                self._z_success_cnt = 0
                self._last_cmd_z = None
                self._axis_probe_z = None
                self._visual_probe_z = None
                return True
        else:
            self._z_success_cnt = 0

        # ------------------------------------------------------------
        # 2. 小步限幅部署
        #    原本邏輯是 rel_z > 0 -> target = cur - rel_z，
        #    因此正誤差預設往 logical 負方向修正。
        # ------------------------------------------------------------
        sign_positive_to_positive = False

        tgt_h, step = self._make_limited_axis_target(
            "Z",
            cur_h,
            rel_z_mm,
            sign_positive_err_to_positive_target=sign_positive_to_positive
        )

        rospy.loginfo_throttle(
            0.5,
            f"[Z] step={step:.1f}mm, logical_target={tgt_h:.1f}mm, "
            f"cur={cur_h:.1f}mm, speed={speed}"
        )

        self._remember_visual_probe("Z", raw_rel_z_mm, cur_h)

        motion_tol_mm = float(getattr(self.Subscriber, "axis_motion_tolerance_mm", 1.0))

        self.DeadMoveZ(
            tgt_h,
            z_tolerance=motion_tol_mm,
            speed=speed
        )

        return False


    def fnControlArmBasedOnFruitX(self, speed=800):
        """
        X 軸視覺對位。

        設計重點：
        1. 不再把 fruit_x_m 全部部署成手臂伸長量，而是對
           fruit_x_m - x_circular_saw_target 做小步部署。
        2. 若 controller command 方向與 state 方向相反，DeadMoveX 會自動切換 x_cmd_invert。
        3. 若視覺誤差在移動後反而變大，會自動切換 x_visual_sign_invert。
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn_throttle(1.0, "X: 尚未接收到手臂狀態")
            return False

        if not self.TFConfidence():
            return False

        if not getattr(self, "_x_inited", False):
            self._x_inited = True
            self._x_success_cnt = 0
            self._x_err_filt_mm = 0.0
            self._last_cmd_x = None
            self._axis_probe_x = None
            self._visual_probe_x = None
            rospy.loginfo("[X] 開始對齊 X 軸")

        x_tol_mm = float(self.Subscriber.x_tolerance_mm)
        x_stop_m = float(getattr(self.Subscriber, "x_circular_saw_target", 0.25))

        fruit_x_m = float(self.Subscriber.marker_2d_pose_x)
        cur_l = float(st.x_pos)

        # X 對位真正要消掉的是「目前距離 - 停止距離」。
        raw_err_mm = (fruit_x_m - x_stop_m) * 1000.0

        # 如果已經過近，直接完成，交給下一階段盲伸。
        if fruit_x_m <= x_stop_m:
            rospy.loginfo(
                f"[X] Too close / reached stop distance: {fruit_x_m:.3f}m ≤ {x_stop_m:.3f}m "
                "→ 停止視覺對位"
            )
            self._x_inited = False
            self._x_success_cnt = 0
            self._last_cmd_x = None
            self._axis_probe_x = None
            self._visual_probe_x = None
            return True

        # 若上一次小步移動後視覺誤差變大，自動反轉視覺誤差符號。
        self._auto_check_visual_error_direction("X", raw_err_mm, cur_l)

        err_for_ctrl = -raw_err_mm if self._get_visual_sign_invert("X") else raw_err_mm
        rel_x_mm = self._filtered_axis_error_mm("X", err_for_ctrl)

        rospy.loginfo_throttle(
            0.5,
            f"[X] fruit_x={fruit_x_m:.3f}m, stop={x_stop_m:.3f}m, "
            f"raw_err={raw_err_mm:.1f}mm, ctrl={err_for_ctrl:.1f}mm, "
            f"filt={rel_x_mm:.1f}mm, tol={x_tol_mm:.1f}mm, cur_x={cur_l:.1f}mm, "
            f"cmd_inv={self._get_axis_invert('X')}, vis_inv={self._get_visual_sign_invert('X')}"
        )

        # ------------------------------------------------------------
        # 1. 如果誤差已經在容許範圍內，視為完成
        # ------------------------------------------------------------
        if abs(raw_err_mm) <= x_tol_mm or abs(rel_x_mm) <= x_tol_mm:
            self._x_success_cnt += 1

            if self._x_success_cnt >= 3:
                rospy.loginfo(
                    f"[X] SUCCESS raw_err={raw_err_mm:.1f}mm, filt={rel_x_mm:.1f}mm "
                    f"(tol={x_tol_mm:.1f}mm)"
                )

                self._x_inited = False
                self._x_success_cnt = 0
                self._last_cmd_x = None
                self._axis_probe_x = None
                self._visual_probe_x = None
                return True
        else:
            self._x_success_cnt = 0

        # ------------------------------------------------------------
        # 2. 防呆：目標跑到相機後方
        # ------------------------------------------------------------
        if fruit_x_m < 0:
            rospy.logwarn("[X] fruit_x < 0，目標在相機後方，視為 X 對位完成")
            self._x_inited = False
            self._x_success_cnt = 0
            self._last_cmd_x = None
            self._axis_probe_x = None
            self._visual_probe_x = None
            return True

        # ------------------------------------------------------------
        # 3. 小步限幅部署
        #    預設 err>0 代表目標太遠，需要 logical X 增加。
        # ------------------------------------------------------------
        sign_positive_to_positive = True

        tgt_l, step = self._make_limited_axis_target(
            "X",
            cur_l,
            rel_x_mm,
            sign_positive_err_to_positive_target=sign_positive_to_positive
        )

        rospy.loginfo_throttle(
            0.5,
            f"[X] step={step:.1f}mm, logical_target={tgt_l:.1f}mm, "
            f"cur={cur_l:.1f}mm, speed={speed}"
        )

        self._remember_visual_probe("X", raw_err_mm, cur_l)

        motion_tol_mm = float(getattr(self.Subscriber, "axis_motion_tolerance_mm", 1.0))

        self.DeadMoveX(
            tgt_l,
            x_tolerance=motion_tol_mm,
            speed=speed
        )

        return False

    def fnBlindExtendArm(self, speed=2000):
        """
        盲伸，非阻塞。

        流程：
        1. 第一次呼叫時，鎖定 target = 目前 x_pos + circular_saw_blind_extend_length
        2. 後續每次呼叫都朝同一個 target 前進
        3. 到達 target 後回傳 True
        4. 完成後清除內部狀態，避免下次流程誤用

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

        extra = float(self.Subscriber.circular_saw_blind_extend_length)

        # ------------------------------------------------------------
        # 第一次進入盲伸階段：鎖定目標，只算一次
        # ------------------------------------------------------------
        if not hasattr(self, "_blind_extend_target"):
            cur_l0 = float(st.x_pos)

            # 正值代表往前盲伸
            raw_tgt = cur_l0 + abs(extra)
            self._blind_extend_target = self._saturate(raw_tgt, L_MIN, L_MAX)

            # 關鍵修正：
            # 新 motion phase 開始，不能沿用視覺 X 對位留下的 _last_cmd_x
            self._last_cmd_x = None

            rospy.loginfo(
                f"BlindExtend(init): cur0={cur_l0:.1f}mm, "
                f"extra={extra:.1f}mm → target={self._blind_extend_target:.1f}mm"
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
        if twist.linear.x > 0 and twist.linear.x < 0.03:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.03:
            twist.linear.x =-0.02   

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

        # 鎖死 X 軸與角度
        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            self.target_l_mm = st.x_pos
            self.target_angle = st.angle

        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        Z_MAX = float(self.Subscriber.circular_saw_max_height)
        self.target_h_mm = self._saturate(float(target_mm), Z_MIN, Z_MAX)

        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        
        msg.x_speed = int(speed)
        msg.z_speed = int(speed)
        msg.saw_speed = int(self.target_saw_speed)
        msg.stop = False

        self.arm_control_pub.publish(msg)

    def fnSawForwardBackward(self, target_mm, speed=2000):
        """只動 X 軸"""
        self._init_targets_from_status_once()

        # 鎖死 Z 軸與角度
        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            self.target_h_mm = st.z_pos
            self.target_angle = st.angle

        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)
        self.target_l_mm = self._saturate(float(target_mm), L_MIN, L_MAX)

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
        """只轉角度"""
        self._init_targets_from_status_once()

        # 鎖死 X 軸與 Z 軸
        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            self.target_l_mm = st.x_pos
            self.target_h_mm = st.z_pos

        C_MIN = float(self.Subscriber.circular_saw_min_angle)
        C_MAX = float(self.Subscriber.circular_saw_max_angle)
        self.target_angle = self._saturate(float(target_angle), C_MIN, C_MAX)

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

        # 鎖死所有平移與轉向位置
        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            self.target_l_mm = st.x_pos
            self.target_h_mm = st.z_pos
            self.target_angle = st.angle

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

        st = getattr(self.Subscriber, "circular_saw_state", None)
        if st is not None:
            self.target_l_mm = st.x_pos
            self.target_h_mm = st.z_pos
            self.target_angle = st.angle

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
