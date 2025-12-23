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
        self.is_triggered = False

    def _saturate(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _soft_stop_arm(self):
        self.cmd_vel.fnCircularStop()
        self._last_cmd_x = None
        self._last_cmd_z = None
        self._last_cmd_angle = None

    def fnRotateToRelativeLine(self, distance, Kp, v):
        time_needed = abs(distance / (Kp * v))   # 計算所需的行駛時間
        start_time = rospy.Time.now().secs  # 獲取當前時間（秒）
        rospy.loginfo(f'time_needed:{time_needed}')
        # 開始移動
        while (rospy.Time.now().secs) < (start_time + time_needed):
            self.cmd_vel.fnGoStraight(Kp, v)
            time.sleep(0.1)  # 每 0.1 秒發送一次指令
        self.cmd_vel.fnStop()   # 停止機器人
        return True

    def fnseqDeadReckoningAngle_Time(self, target_angle, Kp, theta):
        target_angle_rad = math.radians(target_angle)   # 計算目標角度（弧度）
        time_needed = target_angle_rad / (Kp * theta)    # 計算所需的行駛時間
        start_time = rospy.Time.now().secs  # 獲取當前時間（秒）
        self.TestAction.get_logger().info(f'time_needed:{time_needed}')
        while (rospy.Time.now().secs) < (start_time + time_needed):
            self.cmd_vel.fnTurn(Kp, theta)
            time.sleep(0.1)  # 每 0.1 秒發送一次指令
        self.cmd_vel.fnStop()   # 停止機器人
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
            self.cmd_vel.fnStop()  # 停止機器人
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
            self.cmd_vel.fnStop()
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
                self.cmd_vel.fnStop()
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

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.Subscriber.marker_2d_pose_x
                self.initial_marker_pose_y = self.Subscriber.marker_2d_pose_y
                self.desired_dist_diff = abs(self.initial_marker_pose_x) - desired_dist_threshold
                rospy.loginfo(f'desired_dist_diff:{self.desired_dist_diff}')
                if abs(self.initial_marker_pose_x) <= desired_dist_threshold:
                    return True
                else:
                    # 依相機側別決定第一個 90° 轉向：右側 => +90；左側 => -90
                    self.first_turn_deg = getattr(self.Subscriber, "turn_dir_deg", -90)  # 預設 left
                    self.second_turn_deg = -self.first_turn_deg
                    self.current_nearby_sequence = self.NearbySequence.turn_right.value  # 名稱沿用，不代表一定「右」
            # 若無觀測信心則維持等待
            return False

        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            # 用 first_turn_deg（+90 或 -90）
            if self.fnseqDeadReckoningAngle(self.first_turn_deg):
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
            return False

        # 前後調整階段
        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            self.Subscriber.fnDetectionAllowed(False, 0.0)  # fnDetectionAllowed(self, pose_detection, layer)
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
                self.cmd_vel.fnGoStraight(kp, -self.Subscriber.marker_2d_pose_y)
            else:
                # 偏差在容忍範圍內，停止前後運動
                self.cmd_vel.fnStop()
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
            dist_ok = (abs(self.Subscriber.marker_2d_pose_x) <= abs(decide_dist))
            horiz_ok = (abs(self.Subscriber.marker_2d_pose_y) <= abs(horizontal_dist))
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
            self.cmd_vel.fnStop()
            return True
            
    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def TrustworthyMarker2DTheta(self, time):
        marker_2d_theta_list = [0.0]
        initial_time = rospy.Time.now().secs
        
        while(abs(initial_time - rospy.Time.now().secs) < time):
            marker_2d_theta_list.append(self.Subscriber.marker_2d_theta)
            # print("self.marker_2d_theta", self.marker_2d_theta)
            rospy.sleep(0.05)
        # print("marker_2d_theta_list", marker_2d_theta_list)
        threshold = 0.5
        mean = statistics.mean(marker_2d_theta_list)
        stdev = statistics.stdev(marker_2d_theta_list)
        upcutoff = mean + threshold * stdev
        downcutoff = mean - threshold * stdev
        clean_list = []
        for i in marker_2d_theta_list:
            if(i > downcutoff and i < upcutoff):
               clean_list.append(i)
               
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
        共用 1D 盲走邏輯（一步到位，不分段、不變速）：
        - axis_name: "X" 或 "Z"
        - target: 目標位置 (mm)
        - tolerance_mm: 誤差容許
        - get_cur_fn(st): 從 circular_saw_state 取出目前位置
        - send_axis_cmd_fn(pos_mm, speed): 實際送 Command 的 function
        """
        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logwarn(f"{axis_name}: 尚未接收到手臂狀態，等待中...")
            return False

        cur = float(get_cur_fn(st))
        d = target - cur

        # --- 1) 現在位置已經在目標附近 → 視為完成，送一次 Stop ---
        if abs(d) <= tolerance_mm:
            self._soft_stop_arm()
            if axis_name == "X":
                self._last_cmd_x = None
            else:
                self._last_cmd_z = None
            return True

        # --- 2) 還沒到，但檢查「上一個目標」是否已經到位 ---
        if axis_name == "X":
            last_cmd = self._last_cmd_x
        else:
            last_cmd = self._last_cmd_z

        MIN_CMD_DELTA = 1.0  # 新目標 vs 上次目標 少於 1mm 就不發新命令

        if last_cmd is not None:
            remaining_to_last = last_cmd - cur
            # 2a) 距離上一次的目標還很遠 → 認為上一次命令還在執行中，不要重發
            if abs(remaining_to_last) > tolerance_mm:
                return False

            # 2b) 離上一次目標已經很近了，如果新目標跟上次差距也很小 → 不必再發
            if abs(target - last_cmd) < MIN_CMD_DELTA:
                return False

        # --- 3) 確定需要新目標 → 發 Command ---
        send_axis_cmd_fn(int(target), speed)

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

    def fnControlArmBasedOnFruitZ(self, timeout=10.0, speed=2000):
        """
        目標：讓 marker_2d_pose_z → 0
        |relative_z_mm| ≤ z_tolerance_mm → 成功
        """
        start = time.time()

        Z_MIN = float(self.Subscriber.circular_saw_min_height)
        Z_MAX = float(self.Subscriber.circular_saw_max_height)
        z_tol_mm = float(self.Subscriber.z_tolerance_mm)

        TARGET_TOL_MM = z_tol_mm  # 也當作 DeadMove 的容許

        success_cnt = 0
        REQUIRED_OK = 3  # 連續 3 次在誤差內才算真正成功

        while time.time() - start < timeout:
            st = self.Subscriber.circular_saw_state
            if st is None:
                rospy.logwarn("Z: 尚未接收到手臂狀態")
                rospy.sleep(0.1)
                continue

            if not self.TFConfidence():
                # rospy.logwarn("Z: TF Confidence low")
                rospy.sleep(0.1)
                continue

            # 相機量測 (m → mm) + 平滑
            rel_z_mm = float(self.Subscriber.marker_2d_pose_z) * 1000.0

            # 1) 若 marker 誤差已在容許範圍內 → 累加成功次數
            if abs(rel_z_mm) <= z_tol_mm:
                success_cnt += 1
                if success_cnt >= REQUIRED_OK:
                    rospy.loginfo(f"[Z] SUCCESS rel_z={rel_z_mm:.1f} mm (tol={z_tol_mm} mm)")
                    return True
                rospy.sleep(0.1)
                continue
            else:
                success_cnt = 0

            # 2) 根據目前高度 + 相對偏差，算目標高度
            cur_h = float(st.z_pos)
            tgt_h = cur_h - rel_z_mm
            tgt_h = self._saturate(tgt_h, Z_MIN, Z_MAX)

            # 3) 交給 DeadMoveZ 一次走到 target，高度內走完
            self.DeadMoveZ(tgt_h, z_tolerance=TARGET_TOL_MM, speed=speed)

            rospy.sleep(0.1)

        rospy.logwarn("Z: 超時")
        return False

    def fnControlArmBasedOnFruitX(self, timeout=10.0, speed=2000):
        """
        目標：讓 marker_2d_pose_x → 0
        |relative_x_mm| ≤ x_tolerance_mm → 成功
        如果太靠近相機 (<= x_stop_m) 或物體變到相機後面 (rel_x < 0)，就停止視覺對位
        """
        start = time.time()

        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)

        x_tol_mm = float(self.Subscriber.x_tolerance_mm)

        # 安全停止距離（m）
        x_stop_m = float(getattr(self.Subscriber, "x_circular_saw_target", 0.25))

        TARGET_TOL_MM = x_tol_mm
        success_cnt = 0
        REQUIRED_OK = 3

        while time.time() - start < timeout:
            st = self.Subscriber.circular_saw_state
            if st is None:
                rospy.logwarn("X: 尚未接收到手臂狀態")
                rospy.sleep(0.1)
                continue

            if not self.TFConfidence():
                # rospy.logwarn("X: TF Confidence low")
                rospy.sleep(0.1)
                continue

            # 相機量測 (m → mm) + 平滑
            fruit_x_m = float(self.Subscriber.marker_2d_pose_x) # m
            rel_x_mm = fruit_x_m * 1000.0 # mm

            # 1) 在允許誤差內 → 累加成功次數
            if abs(rel_x_mm) <= x_tol_mm:
                success_cnt += 1
                if success_cnt >= REQUIRED_OK:
                    rospy.loginfo(f"[X] SUCCESS rel_x={rel_x_mm:.1f}mm (tol={x_tol_mm}mm)")
                    return True
                rospy.sleep(0.1)
                continue
            else:
                success_cnt = 0

            # 2) 物體在相機後面（負） → 直接結束
            if rel_x_mm < 0:
                rospy.logwarn("[X] rel_x < 0 (物體在相機後方) → 視為 OK，停止視覺對位")
                return True
            
            # 3) 太近（距離太小，看不到） → 停止視覺對位
            if fruit_x_m <= x_stop_m:
                rospy.loginfo(f"[X] Too close: {fruit_x_m:.3f}m ≤ {x_stop_m}m → 停止視覺對位")
                return True

            # 4) 正常情況：算目標長度（一次修正）
            cur_l = float(st.x_pos)
            tgt_l = cur_l + rel_x_mm
            # rospy.loginfo(f"[X] cur_l={cur_l:.1f}mm, rel_x={rel_x_mm:.1f}mm → tgt_l={tgt_l:.1f}mm")
            tgt_l = self._saturate(tgt_l, L_MIN, L_MAX)

            # 5) 用 DeadMoveX，一次走到 target，速度固定
            self.DeadMoveX(tgt_l, x_tolerance=TARGET_TOL_MM, speed=speed)

            rospy.sleep(0.1)

        rospy.logwarn("X: 超時")
        return False
    
    def fnBlindExtendArm(self, timeout=7.0, speed=2000):
        """
        盲伸：
        - 基於目前 X 長度 + blind_extend_length
        - 前伸 = X 更負
        - 使用 DeadMoveX 一次走到底
        - DeadMoveX 自己會用 last_cmd_x 避免重複發相同目標
        """
        if getattr(self, "blind_extend_completed", False):
            rospy.logwarn("fnBlindExtendArm 已經做過，略過此次呼叫")
            return True

        st = self.Subscriber.circular_saw_state
        if st is None:
            rospy.logerr("fnBlindExtendArm: 尚未取得 arm 狀態")
            return False

        L_MIN = float(self.Subscriber.circular_saw_min_length)
        L_MAX = float(self.Subscriber.circular_saw_max_length)

        cur_l = float(st.x_pos)

        extra = float(self.Subscriber.circular_saw_blind_extend_length)  # 正值 mm
        raw_tgt = cur_l + abs(extra)   # 往前伸 → X 變大
        tgt_l = self._saturate(raw_tgt, L_MIN, L_MAX)

        rospy.loginfo(f"BlindExtend: cur={cur_l:.1f}mm, extra={extra:.1f}mm → target={tgt_l:.1f}mm")

        start = time.time()
        while time.time() - start < timeout:
            done = self.DeadMoveX(tgt_l, x_tolerance=5.0, speed=speed)
            if done:
                rospy.loginfo(f"盲伸完成：到達 {tgt_l:.1f} mm")
                self.blind_extend_completed = True
                return True
            rospy.sleep(0.1)

        rospy.logerr(f"盲伸超時 (tgt={tgt_l:.1f}mm)")
        return False

        # -----------------------------

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

    def fnAlignSawToMarker(self, timeout=5.0, angle_tolerance=2.0, speed=2000):
        """
        讓鋸片角度和 marker_2d_theta 達成「水平」：
        假設規則：鋸片角度 + marker_2d_theta_deg ≈ 0
        → 目標鋸片角度 = - marker_2d_theta_deg
        - timeout: 最長等待時間 (s)
        - angle_tolerance: 角度誤差容許 (deg)
        """
        start = time.time()

        # 先讀一次 marker_2d_theta（單位 rad），轉成 deg
        marker_theta_rad = float(self.Subscriber.marker_2d_theta)
        marker_theta_deg = math.degrees(marker_theta_rad)

        C_MIN = float(self.Subscriber.circular_saw_min_angle)
        C_MAX = float(self.Subscriber.circular_saw_max_angle)

        # 設計目標：希望「鋸片角度 + marker_theta_deg ≈ 0」
        target_angle = -marker_theta_deg
        target_angle = self._saturate(target_angle, C_MIN, C_MAX)

        rospy.loginfo(f"[ANGLE] align saw to marker: marker={marker_theta_deg:.2f}deg → target={target_angle:.2f}deg")

        success_cnt = 0
        REQUIRED_OK = 5  # 連續 5 次在誤差內才算成功

        while time.time() - start < timeout:
            st = self.Subscriber.circular_saw_state
            if st is None:
                rospy.logwarn("ANGLE: 尚未接收到手臂狀態")
                rospy.sleep(0.1)
                continue

            if not self.TFConfidence():
                # rospy.logwarn("Z: TF Confidence low")
                rospy.sleep(0.1)
                continue

            # 現在鋸片角度
            cur_angle = float(st.angle)
            err = target_angle - cur_angle

            # 1) 在允許誤差內 → 累計成功次數
            if abs(err) <= angle_tolerance:
                success_cnt += 1
                if success_cnt >= REQUIRED_OK:
                    rospy.loginfo(f"[ANGLE] SUCCESS cur={cur_angle:.2f}deg target={target_angle:.2f}deg (tol={angle_tolerance}deg)")
                    # 完成後，把 _last_cmd_angle 清掉，避免影響下一次
                    self._last_cmd_angle = None
                    return True
                rospy.sleep(0.1)
                continue
            else:
                success_cnt = 0

            # 2) 還沒到位 → 用 DeadRotateAngle 盲轉一次
            self.DeadRotateAngle(target_angle, angle_tolerance=angle_tolerance, speed=speed)
            rospy.sleep(0.1)

        rospy.logwarn(f"[ANGLE] 超時: target={target_angle:.2f}deg，最後角度={cur_angle:.2f}deg")
        return False
    
    def SawRunStop(self, saw_speed, timeout=10.0, speed=2000):
        start = time.time()
        while time.time() - start < timeout:
            st = self.Subscriber.circular_saw_state
            if st is None:
                rospy.logwarn("Saw: 尚未接收到手臂狀態")
                continue
            self.cmd_vel.fnSawRunStop(saw_speed)

            if abs(float(st.saw_speed) - saw_speed) <= 5.0:
                # rospy.loginfo(f"Saw speed reached: {st.saw_speed} RPM")
                return True
        return False

    def TFConfidence(self):#判斷TF是否可信
        # rospy.loginfo('shelf_detection: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_detection))
        # rospy.loginfo('shelf_confidence: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_confidence))
        # rospy.loginfo('confidence_minimum: {0}'.format(self.Subscriber.confidence_minimum))
        if self.Subscriber.object_state == "" or self.Subscriber.object_state != "STABLE":
            self.cmd_vel.fnStop()
            self._soft_stop_arm()
            return False
        low = (not self.Subscriber.sub_detectionConfidence.pose_detection) or \
          (self.Subscriber.sub_detectionConfidence.pose_confidence < self.Subscriber.confidence_minimum)
        if low:
            self.cmd_vel.fnStop()
            self._soft_stop_arm()
            return False
        return True
     
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
        if twist.angular.z > 0 and twist.angular.z < 0.09:  #pin 0.05 -> 0.06
            twist.angular.z =0.1
        elif twist.angular.z < 0 and twist.angular.z > -0.09:
            twist.angular.z =-0.1
        self.pub_cmd_vel.publish(twist)

    def fnStop(self):
        twist = Twist()
        self.cmd_pub(twist)

    def fnTurn(self, Kp=0.2, theta=0.):
        # Kp = 0.3 #1.0
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
        # Kp = 4.0 #6.5

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
        """
        絕對高度：只更新 target_h_mm，長度用 target_l_mm，不跟著實際回報抖動。
        """
        self._init_targets_from_status_once()

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
        """
        絕對長度前後：只更新 target_l_mm，height 用 target_h_mm。
        """
        self._init_targets_from_status_once()

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

    def fnSawRunStop(self, saw_speed=0, speed=2000):
        self._init_targets_from_status_once()

        S_MIN = 0
        S_MAX = float(self.Subscriber.circular_saw_max_saw_speed)

        self.target_saw_speed = self._saturate(int(saw_speed), 0, S_MAX)

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
        self._init_targets_from_status_once()

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

    def fnCircularStop(self):
        """
        停止：只重新發一次「目前 target_* 狀態」
        """
        self._init_targets_from_status_once()
        msg = CircularSawCmd()
        msg.x_pos = int(round(self.target_l_mm))
        msg.z_pos = int(round(self.target_h_mm))
        msg.angle = float(self.target_angle)
        msg.x_speed = 0
        msg.z_speed = 0
        msg.saw_speed = 0
        msg.stop = True

        self.arm_control_pub.publish(msg)
