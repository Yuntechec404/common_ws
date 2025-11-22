# -*- coding: utf-8 -*-
from sys import flags
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
import statistics
import time
from cut_pliers_controller.msg import CmdCutPliers

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
        self.is_odom_received = False
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.initial_robot_pose_x = 0.0
        self.initial_robot_pose_y = 0.0
        # AprilTag_param
        self.is_marker_pose_received = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_pose_z = 0.0
        self.marker_2d_theta = 0.0
        self.initial_marker_pose_x = 0.0
        self.initial_marker_pose_y = 0.0
        self.initial_marker_pose_theta = 0.0
        # Fork_param
        self.forwardbackpostion = 0.0
        self.updownposition = 0.0
        self.fork_threshold = 0.005
        # other
        self.check_wait_time = 0
        self.is_triggered = False

    def SpinOnce(self):
        (self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_pose_z, self.marker_2d_theta)=self.Subscriber.SpinOnce()
        
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
        self.SpinOnce()  # 確保獲取到最新位置
        Kp = 0.3
        threshold = 0.015  # 停止的閾值（弧度）
        target_angle_rad = math.radians(target_angle)   # 將目標角度轉換為弧度
        if not self.is_triggered:   # 初始化：如果是第一次調用，記錄初始累積角度
            self.is_triggered = True
            self.initial_total_theta = self.robot_2d_theta  # 使用累積的總角度作為初始角度
        
        current_angle = self.robot_2d_theta - self.initial_total_theta  # 計算當前已旋轉的角度
        remaining_angle = target_angle_rad - current_angle  # 計算剩餘的旋轉角度
        if abs(remaining_angle) < threshold:   # 判斷是否達到目標角度
            self.cmd_vel.fnStop()  # 停止機器人
            self.is_triggered = False  # 重置觸發狀態
            return True
        else:
            self.cmd_vel.fnTurn(Kp, remaining_angle)    # 執行旋轉，正負值決定方向
            return False

    def fnseqDeadReckoning(self, dead_reckoning_dist):  # 使用里程計算移動到指定距離
        self.SpinOnce()  # 確保獲取到最新位置
        Kp = 0.2
        threshold = 0.015  # 停止的閾值
        if self.is_triggered == False:  # 如果還沒啟動，記錄初始位置
            self.is_triggered = True
            self.initial_robot_pose_x = self.robot_2d_pose_x
            self.initial_robot_pose_y = self.robot_2d_pose_y
        # 計算當前移動距離
        current_dist = self.fnCalcDistPoints(self.initial_robot_pose_x, self.robot_2d_pose_x, self.initial_robot_pose_y, self.robot_2d_pose_y)
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
        self.SpinOnce()
        Kp = 0.02
        dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)
        if self.TFConfidence() and dist != 0: #pin &->and
            return True
        else:
            # rospy.logwarn("Confidence Low")
            return False

    def fnSeqChangingtheta(self, threshod): #旋轉到marker的theta值為0, threshod為角度誤差值
        self.SpinOnce()
        Kp = 0.02
        if self.TFConfidence():
            # self.marker_2d_theta= self.TrustworthyMarker2DTheta(3)
            # print("desired_angle_turn", self.marker_2d_theta)
            # print("threshod", threshod)
            if abs(self.marker_2d_theta) < threshod  :
                self.cmd_vel.fnStop()
                if self.check_wait_time > 20 :
                    self.check_wait_time = 0
                    return True
                else:
                    self.check_wait_time =self.check_wait_time  + 1
                    return False
            else:
                self.cmd_vel.fnTurn(Kp, -self.marker_2d_theta)
                self.check_wait_time = 0
                return False
        else:
            self.check_wait_time = 0
            return False
        
        
    def fnSeqMovingNearbyParkingLot(self,desired_dist_threshold):
        self.SpinOnce()
        Kp = 0.2
        err = 0.05
        if self.current_nearby_sequence != self.previous_nearby_sequence:
            rospy.loginfo('current_nearby_sequence {0}'.format(self.NearbySequence(self.current_nearby_sequence)))
            self.previous_nearby_sequence = self.current_nearby_sequence  # 更新 previous_sequence

        if self.current_nearby_sequence == self.NearbySequence.initial_dist.value:
            if self.TFConfidence():
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y

                self.initial_marker_pose_theta = self.TrustworthyMarker2DTheta(3)
                self.initial_marker_pose_x = self.marker_2d_pose_x
                self.initial_marker_pose_y = self.marker_2d_pose_y
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
        self.SpinOnce()
        # rospy.loginfo(f'fnSeqParking: {self.marker_2d_pose_y}')
        if self.TFConfidence():
            if abs(self.marker_2d_pose_y) > tolerance:
                # 若偏差超過設定距離，透過前後移動來修正位置
                self.cmd_vel.fnGoStraight(kp, -self.marker_2d_pose_y)
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
        self.SpinOnce()
        if self.TFConfidence():
            dist_ok = (abs(self.marker_2d_pose_x) <= abs(decide_dist))
            horiz_ok = (abs(self.marker_2d_pose_y) <= abs(horizontal_dist))
            # rospy.loginfo(f'dist: {abs(self.marker_2d_pose_x)-abs(decide_dist)}, horiz: {abs(self.marker_2d_pose_y)-abs(horizontal_dist)}')
            # rospy.loginfo(f'dist_ok: {dist_ok}, horiz_ok: {horiz_ok}')
            return (dist_ok and horiz_ok, dist_ok)

    def fnseqMoveToMarkerDist(self, marker_dist): #(使用marker)前後移動到距離marker_dist公尺的位置
        self.SpinOnce()
        Kp = 0.2
        if(abs(marker_dist) < 2.0):
            threshold = 0.015
        else:
            threshold = 0.03

        dist = math.sqrt(self.marker_2d_pose_x**2 + self.marker_2d_pose_y**2)
        
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
            self.SpinOnce()
            marker_2d_theta_list.append(self.marker_2d_theta)
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
    
    def fnControlArmBasedOnFruitZ(self, timeout=10.0):
        """
        |relative_z_mm| ≤ z_tolerance_mm → 成功
        """
        start = time.time()

        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)

        z_tol_mm = float(self.Subscriber.z_tolerance_mm)

        Z_STEP_MIN = float(self.Subscriber.z_step_min_mm)
        Z_STEP_MAX = float(self.Subscriber.z_step_max_mm)
        Z_K = float(self.Subscriber.z_step_gain)

        TARGET_TOL_MM = 5.0  # 實際位置與目標位置的±誤差允許

        while time.time() - start < timeout:
            self.SpinOnce()
            st = self.Subscriber.current_arm_status
            if st is None:
                rospy.logwarn("Z: 尚未接收到手臂狀態")
                rospy.sleep(0.2)
                continue

            if not self.TFConfidence():
                rospy.logwarn("Z: TF Confidence low")
                rospy.sleep(0.2)
                continue

            # 當前高度（mm）
            cur_h = float(st.height1 if self.Subscriber.arm_ID == 1 else st.height2)

            # 相對位移 (m → mm)
            rel_z_mm = float(self.marker_2d_pose_z) * 1000.0

            # 1) 成功條件
            if abs(rel_z_mm) <= z_tol_mm:
                rospy.loginfo(f"[Z] SUCCESS rel_z={rel_z_mm:.1f}mm (tol={z_tol_mm}mm)")
                return True

            # 計算目標高度（絕對高度）
            tgt_h = max(Z_MIN, min(Z_MAX, cur_h + rel_z_mm))

            # 與目標差距
            err_mm = tgt_h - cur_h

            # 若已在 ±5mm 內 → 視為停止
            if abs(err_mm) <= TARGET_TOL_MM:
                rospy.loginfo(f"[Z] REACH cur={cur_h:.1f} tgt={tgt_h:.1f}")
                return True

            # 動態步長
            step = self._mm_step(err_mm / 1000.0, Z_STEP_MIN, Z_STEP_MAX, Z_K)
            cmd_h = cur_h + (step if err_mm > 0 else -step)
            cmd_h = max(Z_MIN, min(Z_MAX, cmd_h))

            # rospy.loginfo(f"[Z] cur={cur_h:.1f} rel={rel_z_mm:.1f} tgt={tgt_h:.1f} cmd={cmd_h:.1f}")

            self.cmd_vel.fnClawUpDown(int(cmd_h))
            rospy.sleep(0.15)

        rospy.logwarn("Z: 超時")
        return False


    def fnControlArmBasedOnFruitX(self, timeout=10.0):
        """
        |relative_x_mm| ≤ x_tolerance_mm → 成功
        若距離相機太近 (marker_2d_pose_x <= x_stop_m)：
        → 停止視覺對位，return True
        """
        start = time.time()

        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        x_tol_mm = float(self.Subscriber.x_tolerance_mm)

        X_STEP_MIN = float(self.Subscriber.x_step_min_mm)
        X_STEP_MAX = float(self.Subscriber.x_step_max_mm)
        X_K = float(self.Subscriber.x_step_gain)

        allow_retract = bool(self.Subscriber.cut_pliers_allow_retract)

        TARGET_TOL_MM = 5.0  # 目標位置與實際位置的允許誤差 ±5 mm

        # ⭐ 新增：太近會看不到的「安全停止距離」（單位 m）
        x_stop_m = float(getattr(self.Subscriber, "x_cut_pliers_target", 0.25))

        if not hasattr(self, "last_valid_length"):
            self.last_valid_length = L_MIN

        while time.time() - start < timeout:
            self.SpinOnce()
            st = self.Subscriber.current_arm_status
            if st is None:
                rospy.logwarn("X: 尚未接收到手臂狀態")
                rospy.sleep(0.2)
                continue

            if not self.TFConfidence():
                rospy.logwarn("X: TF Confidence low")
                rospy.sleep(0.2)
                continue

            cur_l = float(st.length1 if self.Subscriber.arm_ID == 1 else st.length2)

            fruit_x_m  = float(self.marker_2d_pose_x)       # m（forward 為正）
            fruit_x_mm = fruit_x_m * 1000.0                # 轉成 mm

            # 1) 已經在允許誤差範圍內 → 視為完成
            if abs(fruit_x_mm) <= x_tol_mm:
                rospy.loginfo(f"[X] SUCCESS rel_x={fruit_x_mm:.1f}mm (tol={x_tol_mm}mm)")
                return True

            # 2) 還是正值但太近，停止視覺對位
            if fruit_x_m <= x_stop_m:
                rospy.loginfo(f"[X] Too close to camera (rel_x={fruit_x_m:.3f}m ≤ {x_stop_m}m) 停止視覺對位")
                return True

            # 3) 物體相對距離變成負的（已經超過目標、在相機後面），且不允許後退 → 直接結束
            if fruit_x_mm < 0 and not allow_retract:
                rospy.logwarn("[X] Too close (rel_x<0) & no retract allowed → 視為 OK，停止視覺對位")
                return True

            # --- 正常情況：還在可觀測距離內，繼續用視覺往前/往後調 ---
            err_mm = fruit_x_mm
            step = self._mm_step(err_mm / 1000.0, X_STEP_MIN, X_STEP_MAX, X_K)
            direction = 1 if err_mm > 0 else -1
            tgt_l = max(L_MIN, min(L_MAX, cur_l + direction * step))

            # 如果這一步變化太小（≤5mm），也直接認為到位
            if abs(tgt_l - cur_l) <= TARGET_TOL_MM:
                rospy.loginfo(f"[X] REACH cur={cur_l:.1f} tgt={tgt_l:.1f}")
                return True

            # rospy.loginfo(f"[X] cur={cur_l:.1f} rel_x={fruit_x_mm:.1f}mm tgt={tgt_l:.1f}mm")

            if direction > 0:
                self.cmd_vel.fnClawForward(int(tgt_l))
            else:
                self.cmd_vel.fnClawBackward(int(tgt_l))

            rospy.sleep(0.15)

        rospy.logwarn("X: 超時")
        return False

    
    def DeadMoveZ(self, target_z, z_tolerance=3):  # 盲走Z（固定速）
        st = self.Subscriber.current_arm_status
        if st is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        current_z = st.height1 if self.Subscriber.arm_ID == 1 else st.height2
        dz = target_z - current_z
        # rospy.loginfo(f'Current Z: {current_z}, Target Z: {target_z}, Delta Z: {dz}')
        if abs(dz) <= z_tolerance:
            self.cmd_vel.fnClawStop()
            return True

        self.cmd_vel.fnClawUpDown(int(target_z))
        return False
    
    def DeadMoveZRel(self, delta_z):
        """計算 target，不送命令"""
        st = self.Subscriber.current_arm_status
        if st is None:
            return None
        current_z = st.height1 if self.Subscriber.arm_ID == 1 else st.height2
        return current_z + float(delta_z)
    
    def fnBlindExtendArm(self, timeout=7.0):
        """
        盲伸（相機停止後的小幅度前伸）：
        - 基於目前長度 + blind_extend_length
        - 使用 DeadMoveX() 方式前伸，保持高度不變
        """

        # 防重複執行
        if hasattr(self, "blind_extend_completed") and self.blind_extend_completed:
            rospy.logwarn("⚠ `fnBlindExtendArm()` 已執行過，跳過此次呼叫")
            return True

        st = self.Subscriber.current_arm_status
        if st is None:
            rospy.logerr("無法獲取手臂狀態")
            return False

        # 取得目前長度
        if self.Subscriber.arm_ID == 1:
            cur_l = st.length1
        else:
            cur_l = st.length2

        if cur_l is None or cur_l <= 0:
            rospy.logerr("當前長度無效，盲伸中止")
            return False

        extra = float(self.Subscriber.cut_pliers_blind_extend_length)
        tgt_l = min(cur_l + extra, float(self.Subscriber.cut_pliers_max_length))

        # rospy.loginfo(f"BlindExtend: cur={cur_l:.1f}mm, extra={extra:.1f} → target={tgt_l:.1f}mm")

        # 使用 DeadMoveX 盲走
        start = time.time()
        while time.time() - start < timeout:
            done = self.DeadMoveX(target_x=tgt_l, x_tolerance=5.0)
            if done:
                rospy.loginfo(f"盲伸完成：到達 {tgt_l:.1f} mm")
                self.blind_extend_completed = True
                return True

            rospy.sleep(0.1)

        rospy.logerr(f"盲伸超時 (tgt={tgt_l:.1f}mm)")
        return False

    def DeadMoveX(self, target_x, x_tolerance=5.0):
        st = self.Subscriber.current_arm_status
        if st is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        current_x = st.length1 if self.Subscriber.arm_ID == 1 else st.length2
        dx = target_x - current_x
        # rospy.loginfo(f"[DeadMoveX] cur={current_x:.1f}mm, tgt={target_x:.1f}mm, dx={dx:.1f}mm, tol={x_tolerance:.1f}mm")

        if abs(dx) <= x_tolerance:
            self.cmd_vel.fnClawStop()
            return True

        if dx >= 0:
            self.cmd_vel.fnClawForward(int(target_x))
        else:
            self.cmd_vel.fnClawBackward(int(target_x))
        return False
    
    def DeadMoveXRel(self, delta_x):
        """計算 target，不送命令"""
        st = self.Subscriber.current_arm_status
        if st is None:
            return None
        current_x = st.length1 if self.Subscriber.arm_ID == 1 else st.length2
        # rospy.loginfo(f'Current X: {current_x}, Delta X: {delta_x}')
        return current_x + float(delta_x)

    def fnControlClaw(self, claw_state, timeout=5.0):
        """
        控制剪鉗開/關（阻塞直到到達目標或 timeout）。
        claw_state: 1/True = 打開, 0/False = 關閉。
        """
        start_time = time.time()
        want_close = bool(claw_state)      # True=開、False=關
        target_open = (not want_close)     # CmdCutPliers.clawX → True=打開（依你 fnClawSet 的命名）

        # 等待 arm status 初始化
        while self.Subscriber.current_arm_status is None and time.time() - start_time < 1.0:
            rospy.logwarn("等待手臂狀態初始化...")
            rospy.sleep(0.05)

        if self.Subscriber.current_arm_status is None:
            rospy.logerr("未接收到手臂狀態，無法控制剪鉗")
            return False

        def get_cur_open():
            st = self.Subscriber.current_arm_status
            if st is None:
                return None
            if self.Subscriber.arm_ID == 1:
                return st.claw1
            else:
                return st.claw2

        last_send = 0.0
        send_interval = 0.2  # 5 Hz 重送一次命令

        while time.time() - start_time < timeout:
            now = time.time()
            # 週期性送開/關命令
            if now - last_send > send_interval:
                self.cmd_vel.fnClawSet(open_=target_open)
                last_send = now
                if want_close:
                    rospy.loginfo("➡ 送出開剪鉗命令")
                else:
                    rospy.loginfo("➡ 送出關剪鉗命令")

            cur_open = get_cur_open()
            if cur_open is not None and cur_open == target_open:
                if want_close:
                    rospy.loginfo("剪鉗打開完成")
                else:
                    rospy.loginfo("剪鉗閉合完成")
                self.cmd_vel.fnClawStop()
                return True

            rospy.sleep(0.05)

        rospy.logerr("剪鉗動作超時，無法確認到達目標狀態")
        self.cmd_vel.fnClawStop()
        return False
    
    def TFConfidence(self):#判斷TF是否可信
        # rospy.loginfo('shelf_detection: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_detection))
        # rospy.loginfo('shelf_confidence: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_confidence))
        # rospy.loginfo('confidence_minimum: {0}'.format(self.Subscriber.confidence_minimum))
        # if (not self.Subscriber.sub_detectionConfidence.pose_detection) or self.Subscriber.sub_detectionConfidence.pose_confidence < self.Subscriber.confidence_minimum:
        #     self.cmd_vel.fnStop()
        #     return False
        if (not self.Subscriber.sub_detectionConfidence.pose_detection) or self.Subscriber.sub_detectionConfidence.pose_confidence < self.Subscriber.confidence_minimum:
            self.cmd_vel.fnStop()
            return False
        return True
     
class cmd_vel():
    def __init__(self, Subscriber):
        self.Subscriber = Subscriber
        self.pub_cmd_vel = self.Subscriber.pub_cmd_vel
        self.arm_pub_cmd_vel = self.Subscriber.arm_control_pub
        self.front = False

        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        L_MIN = float(self.Subscriber.cut_pliers_min_length)

        self.target_h_mm = Z_MIN      # 目標高度
        self.target_l_mm = L_MIN      # 目標長度
        self.target_claw = False      # 目標夾爪狀態
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

    def arm_cmd_pub(self, msg):
        if self.Subscriber.arm_ID == 1:
            msg.enable_motor1 = True
            msg.enable_motor2 = True
            msg.enable_motor3 = False
            msg.enable_motor4 = False
        elif self.Subscriber.arm_ID == 2:
            msg.enable_motor1 = False
            msg.enable_motor2 = False
            msg.enable_motor3 = True
            msg.enable_motor4 = True
            
        self.arm_pub_cmd_vel.publish(msg)

    def _saturate(self, v, lo, hi):
        return max(lo, min(hi, v))
    
    def _init_targets_from_status_once(self):
        """
        第一次使用時，從 current_arm_status 把實際位置抄成目標值，
        之後就都用 target_h_mm / target_l_mm 自己維護。
        """
        if self._target_inited:
            return

        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)
        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        st = getattr(self.Subscriber, "current_arm_status", None)
        if st is not None:
            if self.Subscriber.arm_ID == 1:
                h = float(getattr(st, "height1", Z_MIN))
                l = float(getattr(st, "length1", L_MIN))
                claw = bool(getattr(st, "claw1", False))
            else:
                h = float(getattr(st, "height2", Z_MIN))
                l = float(getattr(st, "length2", L_MIN))
                claw = bool(getattr(st, "claw2", False))
        else:
            h, l, claw = Z_MIN, L_MIN, False

        self.target_h_mm = self._saturate(h if h > 0 else Z_MIN, Z_MIN, Z_MAX)
        self.target_l_mm = self._saturate(l if l > 0 else L_MIN, L_MIN, L_MAX)
        self.target_claw = claw
        self._target_inited = True

    def fnClawSet(self, open_: bool):
        """
        只改夾爪目標，其餘高度/長度用目前 target_*。
        """
        self._init_targets_from_status_once()

        self.target_claw = bool(open_)

        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.height1 = int(round(self.target_h_mm))
            msg.length1 = int(round(self.target_l_mm))
            msg.claw1 = self.target_claw
        else:
            msg.height2 = int(round(self.target_h_mm))
            msg.length2 = int(round(self.target_l_mm))
            msg.claw2 = self.target_claw

        self.arm_cmd_pub(msg)

    def fnClawUpDown(self, target_mm):
        """
        絕對高度：只更新 target_h_mm，長度用 target_l_mm，不跟著實際回報抖動。
        """
        self._init_targets_from_status_once()

        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)

        self.target_h_mm = self._saturate(float(target_mm), Z_MIN, Z_MAX)

        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.height1 = int(round(self.target_h_mm))
            msg.length1 = int(round(self.target_l_mm))
            msg.claw1 = self.target_claw
        else:
            msg.height2 = int(round(self.target_h_mm))
            msg.length2 = int(round(self.target_l_mm))
            msg.claw2 = self.target_claw

        self.arm_cmd_pub(msg)

    def fnClawForward(self, target_mm):
        """
        絕對長度前伸：只更新 target_l_mm，height 用 target_h_mm。
        mode=0 (前進)
        """
        self._init_targets_from_status_once()

        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        self.target_l_mm = self._saturate(abs(float(target_mm)), L_MIN, L_MAX)

        msg = CmdCutPliers()
        msg.mode = 0  # forward
        if self.Subscriber.arm_ID == 1:
            msg.length1 = int(round(self.target_l_mm))
            msg.height1 = int(round(self.target_h_mm))
            msg.claw1 = self.target_claw
        else:
            msg.length2 = int(round(self.target_l_mm))
            msg.height2 = int(round(self.target_h_mm))
            msg.claw2 = self.target_claw

        self.arm_cmd_pub(msg)

    def fnClawBackward(self, target_mm):
        """
        絕對長度後退：只更新 target_l_mm，height 用 target_h_mm。
        mode=1 (後退)
        """
        self._init_targets_from_status_once()

        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        self.target_l_mm = self._saturate(abs(float(target_mm)), L_MIN, L_MAX)

        msg = CmdCutPliers()
        msg.mode = 1  # backward
        if self.Subscriber.arm_ID == 1:
            msg.length1 = int(round(self.target_l_mm))
            msg.height1 = int(round(self.target_h_mm))
            msg.claw1 = self.target_claw
        else:
            msg.length2 = int(round(self.target_l_mm))
            msg.height2 = int(round(self.target_h_mm))
            msg.claw2 = self.target_claw

        self.arm_cmd_pub(msg)

    def fnClawStop(self):
        """
        停止：只重新發一次「目前 target_* 狀態」，不送 0。
        """
        self._init_targets_from_status_once()
        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.height1 = int(round(self.target_h_mm))
            msg.length1 = int(round(self.target_l_mm))
            msg.claw1 = self.target_claw
        else:
            msg.height2 = int(round(self.target_h_mm))
            msg.length2 = int(round(self.target_l_mm))
            msg.claw2 = self.target_claw

        self.arm_cmd_pub(msg)
