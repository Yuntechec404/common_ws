# -*- coding: utf-8 -*-
from sys import flags
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from enum import Enum
import statistics
import time

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
         self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta)=self.Subscriber.SpinOnce()
        
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
        threshold = 0.035  # 停止的閾值（弧度）
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
        self.front = False

    def cmd_pub(self, twist):
        if not self.front:
            twist.linear.x = -twist.linear.x

        if twist.angular.z > 0.2:
            twist.angular.z =0.2
        elif twist.angular.z < -0.2:
            twist.angular.z =-0.2
        if twist.linear.x > 0 and twist.linear.x < 0.01:
            twist.linear.x =0.02
        elif twist.linear.x < 0 and twist.linear.x > -0.01:
            twist.linear.x =-0.02   

        if twist.linear.x > 0.2:
            twist.linear.x =0.2
        elif twist.linear.x < -0.2:
            twist.linear.x =-0.2                     
        if twist.angular.z > 0 and twist.angular.z < 0.06:  #pin 0.05 -> 0.06
            twist.angular.z =0.06
        elif twist.angular.z < 0 and twist.angular.z > -0.06:
            twist.angular.z =-0.06
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

  
