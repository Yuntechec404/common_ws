# -*- coding: utf-8 -*-
import rospy
import forklift_server.msg
from enum import Enum
from PBVS_Action_differential import Action
# from forklift_msg.msg import meteorcar
ParkingCameraSequence = Enum( 'ParkingCameraSequence', \
                    'initial_marker \
                    move_nearby_parking_lot \
                    parking \
                    decide \
                    cut_pliers_align_Z \
                    cut_pliers_align_X \
                    cut_pliers_dead_reckoning_up \
                    cut_pliers_dead_reckoning_down \
                    cut_pliers_dead_reckoning_extend \
                    cut_pliers_dead_reckoning \
                    cut_pliers_home_length \
                    cut_pliers_home_height \
                    cut_pliers_down \
                    close_pliers \
                    open_pliers \
                    stop \
                    error')
FrontSequence = Enum( 'FrontSequence', \
                        'Front \
                        stop \
                        error')
TurnSequence = Enum( 'TurnSequence', \
                        'Turn \
                        stop \
                        error')
    
class PBVS():
    def __init__(self, _as, subscriber, mode):
        self._as = _as
        self._feedback = forklift_server.msg.PBVSMegaposeFeedback()
        self._result = forklift_server.msg.PBVSMegaposeResult()
        self.subscriber = subscriber
        self.command = mode.command
        self.layer_dist = mode.layer_dist
        self.check_wait_time = 0
        self.Action = Action(self.subscriber)

    def fruit_docking(self):
        current_sequence = ParkingCameraSequence.initial_marker.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(ParkingCameraSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence

            if(current_sequence == ParkingCameraSequence.initial_marker.value):
                self.subscriber.fnDetectionAllowed(True, self.layer_dist)  # fnDetectionAllowed(self, pose_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqMarkerDistanceValid()
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingCameraSequence.move_nearby_parking_lot.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingCameraSequence.move_nearby_parking_lot.value):
                self.is_sequence_finished = self.Action.fnSeqMovingNearbyParkingLot(self.subscriber.camera_desired_dist_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingCameraSequence.parking.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingCameraSequence.parking.value):
                self.subscriber.fnDetectionAllowed(True, self.layer_dist)  # fnDetectionAllowed(self, pose_detection, layer)
                self.is_sequence_finished = self.Action.fnSeqParking(self.subscriber.camera_horizon_alignment_threshold, 0.1)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingCameraSequence.decide.value):
                ok_all, dist_ok = self.Action.fnSeqdecide(self.subscriber.camera_desired_dist_threshold, self.subscriber.camera_horizon_alignment_threshold )
                
                if ok_all == True:
                    current_sequence = ParkingCameraSequence.cut_pliers_align_Z.value
                    self.is_sequence_finished = False
                    ok_all = False
                else:
                    current_sequence = (ParkingCameraSequence.parking.value if dist_ok else ParkingCameraSequence.move_nearby_parking_lot.value)
                    self.is_sequence_finished = False
                    dist_ok = False
                    ok_all = False

            elif(current_sequence == ParkingCameraSequence.cut_pliers_align_Z.value):
                # self.is_sequence_finished = self.Action.ClawAlignZX()
                self.is_sequence_finished = self.Action.fnControlArmBasedOnFruitZ()
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.cut_pliers_align_X.value  
                    self.is_sequence_finished = False  

            elif(current_sequence == ParkingCameraSequence.cut_pliers_align_X.value):
                self.is_sequence_finished = self.Action.fnControlArmBasedOnFruitX()
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.cut_pliers_dead_reckoning.value  
                    self.is_sequence_finished = False  

            elif current_sequence == ParkingCameraSequence.cut_pliers_dead_reckoning.value:
                self.subscriber.fnDetectionAllowed(False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                self.is_sequence_finished = self.Action.fnBlindExtendArm()
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.close_pliers.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingCameraSequence.close_pliers.value):
                self.is_sequence_finished = self.Action.fnControlClaw(0)  # 關閉剪鉗
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.cut_pliers_dead_reckoning_up.value  
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingCameraSequence.cut_pliers_dead_reckoning_up.value):
                self.is_sequence_finished = self.Action.DeadMoveZ(120.0, z_tolerance=5.0)
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.cut_pliers_home_length.value  
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingCameraSequence.cut_pliers_home_length.value):
                self.is_sequence_finished = self.Action.DeadMoveX(self.subscriber.cut_pliers_home_length)
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.cut_pliers_down.value  
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingCameraSequence.cut_pliers_down.value):
                self.is_sequence_finished = self.Action.DeadMoveZ(80.0, z_tolerance=5.0)
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.open_pliers.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingCameraSequence.open_pliers.value):
                self.is_sequence_finished = self.Action.fnControlClaw(1)  # 剪鉗
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.cut_pliers_home_height.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingCameraSequence.cut_pliers_home_height.value):
                self.is_sequence_finished = self.Action.DeadMoveZ(self.subscriber.cut_pliers_home_height, z_tolerance=7.0)
                if self.is_sequence_finished:
                    current_sequence = ParkingCameraSequence.stop.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingCameraSequence.stop.value):
                self.subscriber.fnDetectionAllowed(False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            
            else:
                rospy.logerr('Error: {0} does not exist'.format(current_sequence))
                self.subscriber.fnDetectionAllowed(False, self.layer_dist)  # fnDetectionAllowed(self, shelf_detection, pallet_detection, layer)
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
    
    def odom_front(self):
        current_sequence = FrontSequence.Front.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(FrontSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence
            self.subscriber.fnDetectionAllowed(False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)

            if(current_sequence == FrontSequence.Front.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoning(-self.layer_dist)
                if self.is_sequence_finished == True:
                    current_sequence = FrontSequence.stop.value
                    self.is_sequence_finished = False

            elif(current_sequence == FrontSequence.stop.value):
                    if self.check_wait_time > 15 :
                        self.check_wait_time = 0
                        return
                    else:
                        self.check_wait_time =self.check_wait_time  +1
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
            
    def odom_turn(self):
        current_sequence = TurnSequence.Turn.value
        previous_sequence = None  # 用來記錄上一次的階段

        while(not rospy.is_shutdown()):
            # 如果 current_sequence 發生變化，記錄 log
            if current_sequence != previous_sequence:
                rospy.loginfo('Current Sequence: {0}'.format(TurnSequence(current_sequence)))
                previous_sequence = current_sequence  # 更新 previous_sequence
            self.subscriber.fnDetectionAllowed(False, self.layer_dist)  # fnDetectionAllowed(self, shelf_string, pallet_string)

            if(current_sequence == TurnSequence.Turn.value):
                self.is_sequence_finished = self.Action.fnseqDeadReckoningAngle(self.layer_dist)
                if self.is_sequence_finished == True:
                    current_sequence = TurnSequence.stop.value
                    self.is_sequence_finished = False
                
                elif(current_sequence == TurnSequence.stop.value):
                    if self.check_wait_time > 15 :
                        self.check_wait_time = 0
                        return
                    else:
                        self.check_wait_time =self.check_wait_time  +1
            else:
                rospy.loginfo('Error: {0} does not exist'.format(current_sequence))
                if self.check_wait_time > 15 :
                    self.check_wait_time = 0
                    return
                else:
                    self.check_wait_time =self.check_wait_time  +1
