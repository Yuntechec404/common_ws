# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
from PBVS_Action_minicar import Action

# 定義各個模式的獨立狀態機序列
ParkingBodyCameraSequence = Enum('ParkingBodyCameraSequence', \
                                 'init_fork \
                                 changing_direction \
                                 move_nearby_parking_lot \
                                 parking \
                                 changingtheta \
                                 decide \
                                 back \
                                 stop \
                                 error')

ParkingForkCameraSequence = Enum('ParkingForkCameraSequence', \
                                 'init_fork \
                                 parking \
                                 changingtheta \
                                 decide \
                                 back \
                                 stop \
                                 error')

RaisePalletSequence = Enum('RaisePalletSequence', \
                           'init_fork \
                           dead_reckoning \
                           fork_updown \
                           back \
                           going \
                           stop \
                           error')

DropPalletSequence = Enum('DropPalletSequence', \
                          'init_fork \
                          dead_reckoning \
                          fork_updown \
                          back \
                          going \
                          stop \
                          error')

class PBVS():
    def __init__(self, _as, subscriber, goal):
        self._as = _as
        self._feedback = forklift_server.msg.PBVSFeedback()
        self._result = forklift_server.msg.PBVSResult()
        self.subscriber = subscriber
        
        # 簡化後的 Input，只取 command 和 layer
        self.command = goal.command
        self.layer = goal.layer
        
        self.Action = Action(self.subscriber)
        self.check_wait_time = 0
        self.is_sequence_finished = False

        # 執行主程式選擇器
        self.dispatch_action()

    def dispatch_action(self):
        """根據 command 決定執行哪個函式"""
        rospy.loginfo(f"PBVS Start: Command={self.command}, Layer={self.layer}")
        
        if self.command == "parking_bodycamera":
            self.parking_bodycamera()
        elif self.command == "parking_forkcamera":
            self.parking_forkcamera()
        elif self.command == "raise_pallet":
            self.raise_pallet()
        elif self.command == "drop_pallet":
            self.drop_pallet()
        else:
            rospy.logwarn(f"Unknown command: {self.command}")
            self._result.result = 'fail'
            self._as.set_succeeded(self._result)

    def parking_bodycamera(self):
        # 1. 載入參數
        self.subscriber.updown = True # Body camera 需要控制貨叉上下嗎? 參考原code是True
        self.subscriber.offset_x = rospy.get_param(rospy.get_name() + "/bodycamera_tag_offset_x")
        init_fork = rospy.get_param(rospy.get_name() + "/bodycamera_parking_fork_init")
        ChangingDirection_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_ChangingDirection_threshold")
        desired_dist_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_desired_dist_threshold")
        desired_angle_threshold = rospy.get_param(rospy.get_name() + "/bodycamera_desired_angle_threshold")
        Parking_distance = rospy.get_param(rospy.get_name() + "/bodycamera_parking_stop")
        Changingtheta_threshod = rospy.get_param(rospy.get_name() + "/bodycamera_Changingtheta_threshold")
        decide_distance = rospy.get_param(rospy.get_name() + "/bodycamera_decide_distance")
        back_distance = rospy.get_param(rospy.get_name() + "/bodycamera_back_distance")

        current_sequence = ParkingBodyCameraSequence.init_fork.value
        previous_sequence = None

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.logwarn('PBVS Preempted')
                self._as.set_preempted()
                return

            # Log sequence change
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(ParkingBodyCameraSequence(current_sequence)))
                self._feedback.feedback = str(ParkingBodyCameraSequence(current_sequence))
                self._as.publish_feedback(self._feedback)
                previous_sequence = current_sequence

            # --- State Machine ---
            if current_sequence == ParkingBodyCameraSequence.init_fork.value:
                self.is_sequence_finished = self.Action.fork_updown(init_fork)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.changing_direction.value
                    self.is_sequence_finished = False
            
            elif current_sequence == ParkingBodyCameraSequence.changing_direction.value:
                self.is_sequence_finished = self.Action.fnSeqChangingDirection(ChangingDirection_threshold)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.move_nearby_parking_lot.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingBodyCameraSequence.move_nearby_parking_lot.value:
                self.is_sequence_finished = self.Action.fnSeqMovingNearbyParkingLot(desired_dist_threshold, desired_angle_threshold)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingBodyCameraSequence.parking.value:
                self.is_sequence_finished = self.Action.fnSeqParking(Parking_distance)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.changingtheta.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingBodyCameraSequence.changingtheta.value:
                self.is_sequence_finished = self.Action.fnSeqChangingtheta(Changingtheta_threshod)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.decide.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingBodyCameraSequence.decide.value:
                self.is_sequence_finished = self.Action.fnSeqdecide(decide_distance)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.stop.value
                    self.is_sequence_finished = False
                else: # 如果 decide 回傳 False，代表需要後退重來
                    current_sequence = ParkingBodyCameraSequence.back.value
                    self.is_sequence_finished = False
            
            elif current_sequence == ParkingBodyCameraSequence.back.value:
                self.is_sequence_finished = self.Action.fnseqmove_to_marker_dist(back_distance)
                if self.is_sequence_finished:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingBodyCameraSequence.stop.value:
                if self.check_wait_time > 20 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
                
                # self._result.result = 'success'
                # self._as.set_succeeded(self._result)

            rospy.Rate(10).sleep()

    def parking_forkcamera(self):
        # 1. 載入參數
        self.subscriber.updown = False
        self.subscriber.offset_x = rospy.get_param(rospy.get_name() + "/forkcamera_tag_offset_x")
        Changingtheta_threshod = rospy.get_param(rospy.get_name() + "/forkcamera_Changingtheta_threshold")
        Parking_distance = rospy.get_param(rospy.get_name() + "/forkcamera_parking_stop")
        decide_distance = rospy.get_param(rospy.get_name() + "/forkcamera_decide_distance")
        back_distance = rospy.get_param(rospy.get_name() + "/forkcamera_back_distance")
        
        # 根據 Layer 選擇參數
        if self.layer == 2:
            init_fork = rospy.get_param(rospy.get_name() + "/forkcamera_parking_fork_layer2")
        else:
            init_fork = rospy.get_param(rospy.get_name() + "/forkcamera_parking_fork_layer1")

        current_sequence = ParkingForkCameraSequence.init_fork.value
        previous_sequence = None

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.logwarn('PBVS Preempted')
                self._as.set_preempted()
                return

            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(ParkingForkCameraSequence(current_sequence)))
                self._feedback.feedback = str(ParkingForkCameraSequence(current_sequence))
                self._as.publish_feedback(self._feedback)
                previous_sequence = current_sequence

            # --- State Machine ---
            if current_sequence == ParkingForkCameraSequence.init_fork.value:
                self.is_sequence_finished = self.Action.fork_updown(init_fork)
                if self.is_sequence_finished:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False
            
            elif current_sequence == ParkingForkCameraSequence.parking.value:
                self.is_sequence_finished = self.Action.fnSeqParking(Parking_distance)
                if self.is_sequence_finished:
                    current_sequence = ParkingForkCameraSequence.changingtheta.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingForkCameraSequence.changingtheta.value:
                self.is_sequence_finished = self.Action.fnSeqChangingtheta(Changingtheta_threshod)
                if self.is_sequence_finished:
                    current_sequence = ParkingForkCameraSequence.decide.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingForkCameraSequence.decide.value:
                self.is_sequence_finished = self.Action.fnSeqdecide(decide_distance)
                if self.is_sequence_finished:
                    current_sequence = ParkingForkCameraSequence.stop.value
                    self.is_sequence_finished = False
                else:
                    current_sequence = ParkingForkCameraSequence.back.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingForkCameraSequence.back.value:
                self.is_sequence_finished = self.Action.fnseqmove_to_marker_dist(back_distance)
                if self.is_sequence_finished:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif current_sequence == ParkingForkCameraSequence.stop.value:
                if self.check_wait_time > 20 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
                # self._result.result = 'success'
                # self._as.set_succeeded(self._result)
                return

            rospy.Rate(10).sleep()

    def raise_pallet(self):
        # 1. 載入參數
        self.subscriber.updown = False
        dead_reckoning_dist = rospy.get_param(rospy.get_name() + "/raise_pallet_dead_reckoning_dist")
        back_distance = rospy.get_param(rospy.get_name() + "/raise_pallet_back_distance")
        navigation_helght = rospy.get_param(rospy.get_name() + "/raise_pallet_navigation_helght")
        
        if self.layer == 2:
            init_fork = rospy.get_param(rospy.get_name() + "/raise_pallet_fork_init_layer2")
            raise_height = rospy.get_param(rospy.get_name() + "/raise_pallet_raise_height_layer2")
        else:
            init_fork = rospy.get_param(rospy.get_name() + "/raise_pallet_fork_init_layer1")
            raise_height = rospy.get_param(rospy.get_name() + "/raise_pallet_raise_height_layer1")

        current_sequence = RaisePalletSequence.init_fork.value
        previous_sequence = None

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.logwarn('PBVS Preempted')
                self._as.set_preempted()
                return

            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(RaisePalletSequence(current_sequence)))
                self._feedback.feedback = str(RaisePalletSequence(current_sequence))
                self._as.publish_feedback(self._feedback)
                previous_sequence = current_sequence

            # --- State Machine ---
            if current_sequence == RaisePalletSequence.init_fork.value:
                self.is_sequence_finished = self.Action.fork_updown(init_fork)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    self.is_sequence_finished = False

            elif current_sequence == RaisePalletSequence.dead_reckoning.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(dead_reckoning_dist)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif current_sequence == RaisePalletSequence.fork_updown.value:
                self.is_sequence_finished = self.Action.fork_updown(raise_height)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = RaisePalletSequence.back.value
                    self.is_sequence_finished = False

            elif current_sequence == RaisePalletSequence.back.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(back_distance)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = RaisePalletSequence.going.value
                    self.is_sequence_finished = False

            elif current_sequence == RaisePalletSequence.going.value:
                self.is_sequence_finished = self.Action.fork_updown(navigation_helght)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = RaisePalletSequence.stop.value
                    self.is_sequence_finished = False

            elif current_sequence == RaisePalletSequence.stop.value:
                rospy.sleep(1)
                # self._result.result = 'success'
                # self._as.set_succeeded(self._result)
                return

            rospy.Rate(10).sleep()

    def drop_pallet(self):
        # 1. 載入參數
        self.subscriber.updown = True
        dead_reckoning_dist = rospy.get_param(rospy.get_name() + "/drop_pallet_dead_reckoning_dist")
        back_distance = rospy.get_param(rospy.get_name() + "/drop_pallet_back_distance")
        navigation_helght = rospy.get_param(rospy.get_name() + "/drop_pallet_navigation_helght")
        
        if self.layer == 2:
            init_fork = rospy.get_param(rospy.get_name() + "/drop_pallet_fork_init_layer2")
            drop_height = rospy.get_param(rospy.get_name() + "/drop_pallet_drop_height_layer2")
        else:
            # 預設 layer 1 (雖然原code只有寫layer2, 但為了安全補上預設)
            init_fork = rospy.get_param(rospy.get_name() + "/drop_pallet_fork_init_layer1")
            drop_height = rospy.get_param(rospy.get_name() + "/drop_pallet_drop_height_layer1")

        current_sequence = DropPalletSequence.init_fork.value
        previous_sequence = None

        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.logwarn('PBVS Preempted')
                self._as.set_preempted()
                return

            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(DropPalletSequence(current_sequence)))
                self._feedback.feedback = str(DropPalletSequence(current_sequence))
                self._as.publish_feedback(self._feedback)
                previous_sequence = current_sequence

            # --- State Machine ---
            if current_sequence == DropPalletSequence.init_fork.value:
                self.is_sequence_finished = self.Action.fork_updown(init_fork)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = DropPalletSequence.dead_reckoning.value
                    self.is_sequence_finished = False

            elif current_sequence == DropPalletSequence.dead_reckoning.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(dead_reckoning_dist)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = DropPalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif current_sequence == DropPalletSequence.fork_updown.value:
                self.is_sequence_finished = self.Action.fork_updown(drop_height)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = DropPalletSequence.back.value
                    self.is_sequence_finished = False
            
            elif current_sequence == DropPalletSequence.back.value:
                self.is_sequence_finished = self.Action.fnseqdead_reckoning(back_distance)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = DropPalletSequence.going.value
                    self.is_sequence_finished = False

            elif current_sequence == DropPalletSequence.going.value:
                self.is_sequence_finished = self.Action.fork_updown(navigation_helght)
                if self.is_sequence_finished:
                    rospy.sleep(0.05)
                    current_sequence = DropPalletSequence.stop.value
                    self.is_sequence_finished = False

            elif current_sequence == DropPalletSequence.stop.value:
                rospy.sleep(1)
                # self._result.result = 'success'
                # self._as.set_succeeded(self._result)
                return

            rospy.Rate(10).sleep()

    def __del__(self):
        rospy.logwarn('delete PBVS')
