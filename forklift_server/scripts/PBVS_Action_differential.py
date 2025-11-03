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
        # arm
        self.current_arm_status = self.Subscriber.current_arm_status
        # 初始化 y_pose_history 和窗口大小
        # self.y_pose_history = []
        # self.moving_average_window = 5
        # self.arm_control_pub = rospy.Publisher("/cmd_cut_pliers", CmdCutPliers, queue_size=10)
        # 用於儲存最新的手臂狀態
        
        # 訂閱 /arm_current_status 話題
        # self.arm_status_sub = rospy.Subscriber("/arm_current_status", CmdCutPliers, self.arm_status_callback, queue_size=1)


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
    
    def fnControlArmBasedOnFruitZ(self, timeout=10.0, tolerance=4):
        """
        距離越近步距越小：根據與允許帶的距離自動縮小步長。
        """
        start = time.time()

        # 允許帶、動作限制
        lower_z = float(self.Subscriber.cut_pliers_lower_z)   # m
        upper_z = float(self.Subscriber.cut_pliers_upper_z)   # m
        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)

        # 自動步長（mm）範圍與斜率
        Z_STEP_MIN = float(getattr(self.Subscriber, "z_step_min_mm", 3.0))
        Z_STEP_MAX = float(getattr(self.Subscriber, "z_step_max_mm", 30.0))
        Z_K = float(getattr(self.Subscriber, "z_step_gain", 0.6))  # 步長 = K * |誤差mm|

        while time.time() - start < timeout:
            self.SpinOnce()
            st = self.Subscriber.current_arm_status
            if st is None:
                rospy.logwarn("Z: 尚未接收到手臂狀態")
                rospy.sleep(0.2)
                continue

            fruit_z = float(self.marker_2d_pose_z)          # m（向上為正）
            cur_h = float(st.height1)                      # mm
            conf = self.TFConfidence()
            if conf is None or conf < 0.5:
                rospy.logwarn("Z: 信心指數不足，暫停調整")
                rospy.sleep(0.5)
                continue

            # 在允許帶內 → 完成
            if lower_z <= fruit_z <= upper_z:
                rospy.loginfo(f"Z: cur={cur_h:.0f}mm, tgt=IN-BAND({lower_z:.3f}~{upper_z:.3f}m)")
                return True

            # 計算與允許帶的最近距離（m）
            if fruit_z < lower_z:
                err_m = (lower_z - fruit_z)   # 太高 → 要下降 (高度減少)
                direction = -1
            else:
                err_m = (fruit_z - upper_z)   # 太低 → 要上升 (高度增加)
                direction = +1

            step = self._mm_step(err_m, Z_STEP_MIN, Z_STEP_MAX, Z_K)
            tgt_h = max(Z_MIN, min(Z_MAX, cur_h + direction * step))

            rospy.loginfo(f"Z: cur={cur_h:.0f}mm, tgt={tgt_h:.0f}mm")
            if abs(tgt_h - cur_h) <= tolerance:
                # 微小變化不下發，避免抖動
                rospy.sleep(0.2)
                continue

            self.cmd_vel.fnClawUpDown(int(tgt_h))

            # 等待到位/收斂
            reach_t0 = time.time()
            while time.time() - reach_t0 < 3.0:
                self.SpinOnce()
                st2 = self.Subscriber.current_arm_status
                if st2 is None:
                    break
                cur_h2 = float(st2.height1)
                if abs(cur_h2 - tgt_h) <= tolerance:
                    break
                rospy.sleep(0.1)

            rospy.sleep(0.2)

        rospy.logwarn("Z: 超時未達標")
        return False

    def fnControlArmBasedOnFruitX(self, timeout=10.0):
        """
        只保留：現在長度 / 目標長度 / 異常訊息。
        絕對長度控制；距離越近步距越小。達標條件：fruit_x <= target_x + x_tol。
        fruit_x > target_x → 前伸；fruit_x < target_x → 允許時後退。
        """
        start = time.time()

        # 參數
        target_x_m = float(self.Subscriber.cut_pliers_target_x)        # m
        x_tol_m = float(self.Subscriber.x_tolerance_m)  # m
        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)
        allow_retract= bool(self.Subscriber.cut_pliers_allow_retract)

        # 自動步長（mm）範圍與斜率
        X_STEP_MIN = float(self.Subscriber.x_step_min_mm)
        X_STEP_MAX = float(self.Subscriber.x_step_max_mm)
        X_K = float(self.Subscriber.x_step_gain)

        if not hasattr(self, "last_valid_length"):
            self.last_valid_length = L_MIN

        while time.time() - start < timeout:
            self.SpinOnce()
            st = self.Subscriber.current_arm_status
            if st is None:
                rospy.logwarn("X: 尚未接收到手臂狀態")
                rospy.sleep(0.2)
                continue

            fruit_x = float(self.marker_2d_pose_x)    # m（forward 為正）
            cur_l = float(st.length1)               # mm
            conf = self.TFConfidence()
            if conf is None or conf < 0.5:
                rospy.logwarn("X: 信心指數不足，暫停調整")
                rospy.sleep(0.5)
                continue

            # 失真/回退防呆
            if cur_l <= 0:
                cur_l = self.last_valid_length
            else:
                self.last_valid_length = max(self.last_valid_length, cur_l)

            # 達標（含容差）
            if fruit_x <= (target_x_m + x_tol_m):
                rospy.loginfo(f"X: cur={cur_l:.0f}mm, tgt=MEET({target_x_m:.3f}m±{x_tol_m:.3f})")
                return True

            # 誤差與方向
            err_m = fruit_x - target_x_m
            if err_m > 0:          # 太遠 → 前伸
                direction = +1
            else:                  # 太近 → 後退（需允許）
                if not allow_retract:
                    rospy.logwarn("X: 過近且不允許後退，停止調整")
                    return True
                direction = -1

            step = self._mm_step(err_m, X_STEP_MIN, X_STEP_MAX, X_K)
            tgt_l = max(L_MIN, min(L_MAX, cur_l + direction * step))

            rospy.loginfo(f"X: cur={cur_l:.0f}mm, tgt={tgt_l:.0f}mm")
            if direction > 0:
                self.cmd_vel.fnClawForward(int(tgt_l))
            else:
                self.cmd_vel.fnClawBackward(int(tgt_l))

            # 到位等候
            reach_t0 = time.time()
            while time.time() - reach_t0 < 3.0:
                self.SpinOnce()
                st2 = self.Subscriber.current_arm_status
                if st2 is None:
                    break
                cur_l2 = float(st2.length1)
                if cur_l2 <= 0:
                    cur_l2 = self.last_valid_length
                else:
                    self.last_valid_length = max(self.last_valid_length, cur_l2)
                if abs(cur_l2 - tgt_l) <= 10.0:
                    break
                rospy.sleep(0.1)

            rospy.sleep(0.2)

        rospy.logwarn("X: 超時未達標")
        return False

    def ClawAlignZX(self, z_tolerance=3, x_tolerance=3):
        """
        以固定速硬體對齊 ZX（全部 mm 單位）：
        - 相機給的是「相對位移」(Δz, Δx)；馬達命令為「絕對位置」
        => 目標絕對 = 目前位置 + 相對位移(公尺→mm)
        - anti-overshoot：前視量(lead) + 單步上限(step cap)
        - hysteresis：連續 N 次在容差內才判定完成
        """

        # 讀感測
        self.SpinOnce()
        if not self.TFConfidence():
            rospy.logwarn("TF confidence is low, cannot align claw.")
            return False
        if self.current_arm_status is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        # 相機位移(相對) 由 m 轉 mm（維持你在 cbGetObject 的軸向：forward=+pz、up=-py）
        scale = float(getattr(self.Subscriber, "marker_to_mm", 1000.0))
        rel_z_mm = float(self.marker_2d_pose_z) * scale   # Δz (mm), 上為正
        rel_x_mm = float(self.marker_2d_pose_x) * scale   # Δx (mm), 前為正

        # 目前（arm 回報單位即 mm）
        if self.Subscriber.arm_ID == 1:
            current_z_mm = float(self.current_arm_status.height1)
            current_x_mm = float(self.current_arm_status.length1)
        else:
            current_z_mm = float(self.current_arm_status.height2)
            current_x_mm = float(self.current_arm_status.length2)

        # 目標絕對位置 = 目前 + 相對位移
        target_z_mm = current_z_mm + rel_z_mm
        target_x_mm = current_x_mm + rel_x_mm

        dz = target_z_mm - current_z_mm    # = rel_z_mm
        dx = target_x_mm - current_x_mm    # = rel_x_mm

        # 連續命中判定（抑制邊界抖動）
        if not hasattr(self, "_zx_stable_cnt"):
            self._zx_stable_cnt = 0
        STABLE_N = int(getattr(self.Subscriber, "zx_stable_frames", 5))

        in_z = abs(dz) <= z_tolerance
        in_x = abs(dx) <= x_tolerance
        if in_z and in_x:
            self._zx_stable_cnt += 1
            self.cmd_vel.fnStop()
            self.cmd_vel.fnClawStop()
            if self._zx_stable_cnt >= STABLE_N:
                self._zx_stable_cnt = 0
                return True
            return False
        else:
            self._zx_stable_cnt = 0

        # 參數（mm）
        Z_LEAD_NEAR = float(getattr(self.Subscriber, "z_lead_near_mm", 6.0))
        Z_LEAD_FAR = float(getattr(self.Subscriber, "z_lead_far_mm", 12.0))
        Z_SWITCH = float(getattr(self.Subscriber, "z_lead_switch_mm", 25.0))
        Z_STEP_MAX = float(getattr(self.Subscriber, "z_step_max_mm", 30.0))
        Z_MIN = float(getattr(self.Subscriber, "cut_pliers_min_height", 0.0))
        Z_MAX = float(getattr(self.Subscriber, "cut_pliers_max_height", 280.0))

        X_LEAD_NEAR = float(getattr(self.Subscriber, "x_lead_near_mm", 10.0))
        X_LEAD_FAR = float(getattr(self.Subscriber, "x_lead_far_mm", 20.0))
        X_SWITCH = float(getattr(self.Subscriber, "x_lead_switch_mm", 40.0))
        X_STEP_MAX = float(getattr(self.Subscriber, "x_step_max_mm", 50.0))
        X_MIN = 10.0
        X_MAX = float(getattr(self.Subscriber, "cut_pliers_max_length", 440.0))

        def _clip(v, lo, hi): return max(lo, min(hi, v))
        def _sgn(v): return 1.0 if v >= 0.0 else -1.0

        # Z：預停 + 限步（mm）— 用「目標絕對」做預停
        if not in_z:
            z_lead = Z_LEAD_FAR if abs(dz) > Z_SWITCH else Z_LEAD_NEAR
            # 往目標方向提前預停 z_pre（仍是絕對座標）
            z_pre = _clip(target_z_mm - _sgn(dz) * z_lead, Z_MIN, Z_MAX)
            z_to_pre = z_pre - current_z_mm
            z_delta = dz if abs(dz) < abs(z_to_pre) else z_to_pre
            z_step_target = _clip(current_z_mm + _clip(z_delta, -Z_STEP_MAX, Z_STEP_MAX), Z_MIN, Z_MAX)
            rospy.loginfo(f"ClawAlignZX[Z]: rel={rel_z_mm:.1f}, target={target_z_mm:.1f}, current={current_z_mm:.1f}, dz={dz:.1f}, z_pre={z_pre:.1f}, z_to_pre={z_to_pre:.1f}, step_target={z_step_target:.1f}")
            self.cmd_vel.fnClawUpDown(int(round(z_step_target)))

        # X：預停 + 限步（mm）— 同上
        if not in_x:
            x_lead = X_LEAD_FAR if abs(dx) > X_SWITCH else X_LEAD_NEAR
            x_pre = _clip(target_x_mm - _sgn(dx) * x_lead, X_MIN, X_MAX)
            x_to_pre = x_pre - current_x_mm
            x_delta = dx if abs(dx) < abs(x_to_pre) else x_to_pre
            x_step_target = _clip(current_x_mm + _clip(x_delta, -X_STEP_MAX, X_STEP_MAX), X_MIN, X_MAX)
            rospy.loginfo(f"ClawAlignZX[X]: rel={rel_x_mm:.1f}, target={target_x_mm:.1f}, current={current_x_mm:.1f}, dx={dx:.1f}, x_pre={x_pre:.1f}, x_to_pre={x_to_pre:.1f}, step_target={x_step_target:.1f}")
            if x_step_target >= current_x_mm:
                self.cmd_vel.fnClawForward(int(round(x_step_target)))
            else:
                self.cmd_vel.fnClawBackward(int(round(x_step_target)))

        return False
    
    def DeadMoveZ(self, target_z, z_tolerance=3):  # 盲走Z（固定速）
        if self.current_arm_status is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        current_z = self.current_arm_status.height1 if self.Subscriber.arm_ID == 1 else self.current_arm_status.height2
        dz = target_z - current_z
        if abs(dz) <= z_tolerance:
            self.cmd_vel.fnClawStop()
            return True

        self.cmd_vel.fnClawUpDown(int(target_z))
        return False

    def DeadMoveX(self, target_x, x_tolerance=3):  # 盲走X（固定速）
        if self.current_arm_status is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        current_x = self.current_arm_status.length1 if self.Subscriber.arm_ID == 1 else self.current_arm_status.length2
        dx = target_x - current_x
        if abs(dx) <= x_tolerance:
            self.cmd_vel.fnClawStop()
            return True

        if dx >= 0:
            self.cmd_vel.fnClawForward(int(target_x))
        else:
            self.cmd_vel.fnClawBackward(int(target_x))
        return False

    def fnControlClaw(self, claw_state, timeout=3):
        start_time = time.time()

        # 確保 claw_state 為 bool
        claw_state = bool(claw_state)

        # 等待初始手臂狀態
        while self.current_arm_status is None and time.time() - start_time < 1.0:
            rospy.logwarn("等待手臂狀態初始化...")
            rospy.sleep(0.1)
        if self.current_arm_status is None:
            rospy.logerr("❌ 未接收到手臂狀態，無法控制剪鉗")
            return False

        # 發送剪鉗控制指令
        if self.Subscriber.arm_ID == 1:
            claw_state = self.current_arm_status.claw1 if claw_state else not self.current_arm_status.claw1
        elif self.Subscriber.arm_ID == 2:
            claw_state = self.current_arm_status.claw2 if claw_state else not self.current_arm_status.claw2

        # 等待剪鉗狀態變更
        while time.time() - start_time < timeout:
            self.SpinOnce()  # 處理 ROS 回傳的狀態
            if self.current_arm_status.claw1 == claw_state:
                if claw_state:  # 閉合
                    rospy.loginfo(f"✅ 剪鉗閉合成功，等待2秒以穩定狀態...")
                    rospy.sleep(5)  # 閉合後等待2秒
                else:  # 打開
                    rospy.loginfo(f"✅ 剪鉗打開成功，等待10秒以穩定狀態...")
                    rospy.sleep(25)  # 打開後等待10秒
                return True
            rospy.logwarn(f"⏳ 剪鉗動作中... 目標: {claw_state}, 當前: {self.current_arm_status.claw1}")
            rospy.sleep(0.1)
        
        rospy.logerr(f"⏰ 剪鉗動作超時: 目標 {claw_state}, 當前: {self.current_arm_status.claw1}")
        return False
    
    def TFConfidence(self):#判斷TF是否可信
        # rospy.loginfo('shelf_detection: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_detection))
        # rospy.loginfo('shelf_confidence: {0}'.format(self.Subscriber.sub_detectionConfidence.shelf_confidence))
        # rospy.loginfo('confidence_minimum: {0}'.format(self.Subscriber.confidence_minimum))
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
    
    def _get_current_hl(self):
        """讀目前高度/長度；若尚未有回報，用參數下限做保底。"""
        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)
        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        h_cur, l_cur = Z_MIN, L_MIN
        st = getattr(self.Subscriber, "current_arm_status", None)
        if st is not None:
            if self.Subscriber.arm_ID == 1:
                h_cur = float(getattr(st, "height1", Z_MIN))
                l_cur = float(getattr(st, "length1", L_MIN))
                claw_cur = getattr(st, "claw1", False)
            else:
                h_cur = float(getattr(st, "height2", Z_MIN))
                l_cur = float(getattr(st, "length2", L_MIN))
                claw_cur = getattr(st, "claw2", False)

        h_cur = self._saturate(h_cur if h_cur > 0 else Z_MIN, Z_MIN, Z_MAX)
        l_cur = self._saturate(l_cur if l_cur > 0 else L_MIN, L_MIN, L_MAX)
        return int(round(h_cur)), int(round(l_cur)), claw_cur
    
    def fnClawSet(self, open_: bool):
        """只改夾爪，其他軸帶目前值（絕對位置控制避免 0）。"""
        h_cur, l_cur, claw_cur = self._get_current_hl()
        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.height1 = h_cur
            msg.length1 = l_cur
            msg.claw1 = bool(open_)
        else:
            msg.height2 = h_cur
            msg.length2 = l_cur
            msg.claw2 = bool(open_)
        self.arm_pub_cmd_vel.publish(msg)
    
    def _fill_claw(self, msg, claw_cur):
        """把目前爪子狀態（若有）放進訊息裡，避免意外改變爪子。"""
        if claw_cur is None:
            # 讀不到就維持既有；某些韌體會忽略未設定欄位
            return
        if self.Subscriber.arm_ID == 1:
            msg.claw1 = bool(claw_cur)
            msg.claw2 = False
        else:
            msg.claw1 = False
            msg.claw2 = bool(claw_cur)

    def fnClawUpDown(self, target_mm):
        """絕對高度：同時帶入目前長度，避免另一軸送 0。"""
        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)

        h_cur, l_cur, claw_cur = self._get_current_hl()
        target_h = int(round(self._saturate(float(target_mm), Z_MIN, Z_MAX)))
        target_l = int(round(l_cur))  # 未改動，帶目前值

        msg = CmdCutPliers()
        # 長度也要一併填入目前值（避免 0）
        if self.Subscriber.arm_ID == 1:
            msg.height1 = target_h
            msg.length1 = target_l
        else:
            msg.height2 = target_h
            msg.length2 = target_l

        self._fill_claw(msg, claw_cur)
        self.arm_pub_cmd_vel.publish(msg)

    def fnClawForward(self, target_mm):
        """絕對長度前伸：同時帶入目前高度，並設定 mode=0。"""
        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)
        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        h_cur, l_cur, claw_cur = self._get_current_hl()
        target_l = int(round(self._saturate(abs(float(target_mm)), L_MIN, L_MAX)))
        keep_h   = int(round(self._saturate(h_cur, Z_MIN, Z_MAX)))

        msg = CmdCutPliers()
        msg.mode = 0  # forward
        if self.Subscriber.arm_ID == 1:
            msg.length1 = target_l
            msg.height1 = keep_h
        else:
            msg.length2 = target_l
            msg.height2 = keep_h

        self._fill_claw(msg, claw_cur)
        self.arm_pub_cmd_vel.publish(msg)

    def fnClawBackward(self, target_mm):
        """絕對長度後退：同時帶入目前高度，並設定 mode=1。"""
        Z_MIN = float(self.Subscriber.cut_pliers_min_height)
        Z_MAX = float(self.Subscriber.cut_pliers_max_height)
        L_MIN = float(self.Subscriber.cut_pliers_min_length)
        L_MAX = float(self.Subscriber.cut_pliers_max_length)

        h_cur, l_cur, claw_cur = self._get_current_hl()
        target_l = int(round(self._saturate(abs(float(target_mm)), L_MIN, L_MAX)))
        keep_h   = int(round(self._saturate(h_cur, Z_MIN, Z_MAX)))

        msg = CmdCutPliers()
        msg.mode = 1  # backward
        if self.Subscriber.arm_ID == 1:
            msg.length1 = target_l
            msg.height1 = keep_h
        else:
            msg.length2 = target_l
            msg.height2 = keep_h

        self._fill_claw(msg, claw_cur)
        self.arm_pub_cmd_vel.publish(msg)

    def fnClawStop(self):
        msg = CmdCutPliers()
        self.arm_cmd_pub(msg)
