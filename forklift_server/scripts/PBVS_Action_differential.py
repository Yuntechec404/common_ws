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
                    self.current_nearby_sequence = self.NearbySequence.turn_right.value

        # 水平對準階段
        elif self.current_nearby_sequence == self.NearbySequence.turn_right.value:
            if self.fnseqDeadReckoningAngle(90):
                self.current_nearby_sequence = self.NearbySequence.go_straight.value
            else:
                # rospy.logwarn("turn right failed")
                return False

        # 前後調整階段
        elif self.current_nearby_sequence == self.NearbySequence.go_straight.value:
            if self.fnseqDeadReckoning(-(self.desired_dist_diff + err)):
                self.current_nearby_sequence = self.NearbySequence.turn_left.value
            else:
                # rospy.logwarn("go straight failed")
                return False
        
        # 恢復原始朝向階段
        elif self.current_nearby_sequence == self.NearbySequence.turn_left.value:
            if self.fnseqDeadReckoningAngle(-90):
                self.current_nearby_sequence = self.NearbySequence.initial_marker.value
                return True
            else:
                # rospy.logwarn("turn left failed")
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
            # 正值表示偏右、負值表示偏左，目標為 0（中心位）
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
            dist = self.marker_2d_pose_x
            horizontal = self.marker_2d_pose_y
            if  abs(dist) <= abs(decide_dist) and abs(horizontal) <= abs(horizontal_dist):
                return True
            else:
                return False
        else:
            return False

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

    def ClawAlignZX(self, z_tolerance=3, x_tolerance=3):
        # 讀取當前 marker 與 arm 狀態
        self.SpinOnce()
        if not self.TFConfidence():
            rospy.logwarn("TF confidence is low, cannot align claw.")
            return False
        target_z = self.marker_2d_pose_z
        target_x = self.marker_2d_pose_x
        if self.current_arm_status is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False
        if self.Subscriber.arm_ID ==1:
            current_z = self.current_arm_status.height1
            current_x = self.current_arm_status.length1
        elif self.Subscriber.arm_ID ==2:
            current_z = self.current_arm_status.height2
            current_x = self.current_arm_status.length2

        dz = target_z - current_z
        dx = target_x - current_x

        in_z = abs(dz) <= z_tolerance
        in_x = abs(dx) <= x_tolerance

        if in_z and in_x:
            self.cmd_vel.fnStop()
            self.cmd_vel.fnClawStop()
            return True

        # 速度隨距離調整
        z_speed = self.cmd_vel._clawZ_speed(0.2 * abs(dz))
        x_speed = self.cmd_vel._clawX_speed(0.2 * abs(dx))

        # 發送命令
        if not in_z:
            self.cmd_vel.fnClawUpDown(int(target_z), z_speed)
        if not in_x:
            self.cmd_vel.fnClawForward(int(target_x), x_speed)
        return False
    
    def DeadMoveZ(self, target_z, speed_k=0.5, z_tolerance=3): # 盲走Z
        if self.current_arm_status is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        if self.Subscriber.arm_ID == 1:
            current_z = self.current_arm_status.height1
        elif self.Subscriber.arm_ID == 2:
            current_z = self.current_arm_status.height2
        
        dz = min(target_z + current_z, self.Subscriber.cut_pliers_max_height)
        if abs(dz) <= z_tolerance:
            self.cmd_vel.fnClawStop()
            return True

        if dz > 0:
            z_speed = self.cmd_vel._clawZ_speed(abs(dz) * speed_k + 1)
            self.cmd_vel.fnClawUpDown(int(target_z), z_speed)
            # rospy.loginfo(f"Z 上升: {current_z:.1f}→{target_z:.1f} (dz={dz:.1f}, speed={z_speed})")
        else:
            z_speed = self.cmd_vel._clawZ_speed(abs(dz) * speed_k + 1)
            self.cmd_vel.fnClawUpDown(int(target_z), z_speed)
            # rospy.loginfo(f"Z 下降: {current_z:.1f}→{target_z:.1f} (dz={dz:.1f}, speed={z_speed})")
        return False

    def DeadMoveX(self, target_x, speed_k=0.5, x_tolerance=3):  # 盲走X
        if self.current_arm_status is None:
            rospy.logwarn("尚未接收到手臂狀態，等待中...")
            return False

        if self.Subscriber.arm_ID == 1:
            current_x = self.current_arm_status.length1
        elif self.Subscriber.arm_ID == 2:
            current_x = self.current_arm_status.length2

        dx = min(target_x + current_x, self.Subscriber.cut_pliers_max_length)
        if abs(dx) <= x_tolerance:
            self.cmd_vel.fnClawStop()
            return True

        if dx > 0:
            x_speed = self.cmd_vel._clawX_speed(abs(dx) * speed_k + 1)
            self.cmd_vel.fnClawForward(int(target_x), x_speed)
            # rospy.loginfo(f"X 前伸: {current_x:.1f}→{target_x:.1f} (dx={dx:.1f}, speed={x_speed})")
        else:
            x_speed = self.cmd_vel._clawX_speed(abs(dx) * speed_k + 1)
            self.cmd_vel.fnClawBackward(int(target_x), x_speed)
            # rospy.loginfo(f"X 縮回: {current_x:.1f}→{target_x:.1f} (dx={dx:.1f}, speed={x_speed})")
        return False

    def fnRetractArm(self, timeout=12.0):
            """
            後退手臂到指定的目標長度。
            參數從 self.Subscriber 讀取。
            """
            if hasattr(self, "retract_executed") and self.retract_executed:
                rospy.logwarn("已執行過後退，忽略此次請求")
                return False

            target_length = self.Subscriber.cut_pliers_retract_length
            rospy.loginfo(f"正在執行 fnRetractArm(), 目標長度: {target_length}")

            start_time = time.time()
            if self.Subscriber.arm_ID == 1:
                current_length = self.current_arm_status.length1
            elif self.Subscriber.arm_ID == 2:
                current_length = self.current_arm_status.length2

            if current_length is None:
                rospy.logerr("無法獲取當前手臂長度，後退失敗")
                return False

            if target_length > current_length:
                rospy.logwarn(f"目標長度 {target_length} mm 大於當前長度 {current_length} mm，忽略請求")
                return False

            # 設置為已執行後退
            self.retract_executed = True

            # 發送後退訊息
            
            self.arm_control_pub.publish(msg)
            rospy.loginfo(f"🔵 已發送後退指令: {msg}")

            while time.time() - start_time < timeout:
                self.SpinOnce()
                current_length = self.current_arm_status.length1

                if abs(current_length - target_length_1) <= 10:
                    rospy.loginfo(f"✅ 手臂已成功縮回至 {current_length} mm")
                    return True

                rospy.logwarn(f"⏳ 目前長度 {current_length} mm，目標 {target_length_1} mm，等待中...")
                rospy.sleep(0.5)

            rospy.logerr(f"⏰ 手臂後退超時: 目標 {target_length_1} mm 未達成，當前 {current_length} mm")
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
        self.arm_pub_cmd_vel = self.Subscriber.arm_control_topic
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
    
    def _clawZ_speed(self, speed):
        SPEED_MIN = 1
        SPEED_MAX = 10
        return max(SPEED_MIN, min(SPEED_MAX, int(speed)))
    
    def _clawX_speed(self, speed):
        SPEED_MIN = 1
        SPEED_MAX = 10
        return max(SPEED_MIN, min(SPEED_MAX, int(speed)))

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
            msg.enable_motor4 = True\
            
        self.arm_pub_cmd_vel.publish(msg)

    def fnClawClose(self): # 關閉剪鉗
        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.claw1 = True
            msg.claw2 = False
        elif self.Subscriber.arm_ID == 2:
            msg.claw1 = False
            msg.claw2 = True
        self.arm_cmd_pub(msg)
    
    def fnClawOpen(self): # 打開剪鉗
        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.claw1 = False
            msg.claw2 = True
        elif self.Subscriber.arm_ID == 2:
            msg.claw1 = True
            msg.claw2 = False

    def fnClawUpDown(self, target=10, speed=1): # 上升，target為增加的高度，速度自選
        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.speed1 = self._clawZ_speed(speed)
            msg.height1 = target
        elif self.Subscriber.arm_ID == 2:
            msg.speed3 = self._clawZ_speed(speed)
            msg.height2 = target
        self.arm_cmd_pub(msg)

    def fnClawForward(self, target=10, speed=1): # 前伸，target為增加的長度（正數），速度自選
        msg = CmdCutPliers()
        msg.mode = 0
        if self.Subscriber.arm_ID == 1:
            msg.speed2 = self._clawX_speed(speed)
            msg.length1 = abs(target)
        elif self.Subscriber.arm_ID == 2:
            msg.speed4 = self._clawX_speed(speed)
            msg.length2 = abs(target)
        self.arm_cmd_pub(msg)

    def fnClawBackward(self, target=10, speed=1): # 後退，target為減少的長度（正數），速度自選
        msg = CmdCutPliers()
        msg.mode = 1
        if self.Subscriber.arm_ID == 1:
            msg.speed2 = self._clawX_speed(speed)
            msg.length1 = abs(target)
        elif self.Subscriber.arm_ID == 2:
            msg.speed4 = self._clawX_speed(speed)
            msg.length2 = abs(target)
        self.arm_cmd_pub(msg)

    def fnClawStop(self):
        msg = CmdCutPliers()
        if self.Subscriber.arm_ID == 1:
            msg.speed1 = 0
            msg.speed2 = 0
        elif self.Subscriber.arm_ID == 2:
            msg.speed3 = 0
            msg.speed4 = 0
        self.arm_cmd_pub(msg)
