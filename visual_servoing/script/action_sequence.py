# -*- coding: utf-8 -*-
import rclpy
from enum import Enum
from action import Action
ParkingBodyCameraSequence = Enum( 'ParkingSequence', \
                        'init_fork \
                        changing_direction_1 \
                        moving_nearby_parking_lot \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop')
    
class ActionSequence():
    def __init__(self, VisualServoingActionServer):
        self.visual_servoing_action_server = VisualServoingActionServer  # access function {SpinOnce(), SpinOnce_Fork()} and parameter
        self.action = Action()
        self.is_sequence_finished = False

    def parking_bodycamera(self, goal_handle, layer):
        current_sequence = ParkingBodyCameraSequence.init_fork.value

        if(current_sequence == ParkingBodyCameraSequence.init_fork.value):
            self.is_sequence_finished = self.action.fork_updown(self.visual_servoing_action_server.bodycamera_parking_fork_init)
            
            if self.is_sequence_finished == True:
                current_sequence = ParkingBodyCameraSequence.changing_direction_1.value
                self.is_sequence_finished = False
        
        elif(current_sequence == ParkingBodyCameraSequence.changing_direction_1.value):
            self.is_sequence_finished = self.action.fnSeqChangingDirection(self.visual_servoing_action_server.bodycamera_ChangingDirection_threshold)
            
            if self.is_sequence_finished == True:
                current_sequence = ParkingBodyCameraSequence.changingtheta.value
                self.is_sequence_finished = False
        
        elif(current_sequence == ParkingBodyCameraSequence.move_nearby_parking_lot.value):
            self.is_sequence_finished = self.action.fnSeqMovingNearbyParkingLot()
            
            if self.is_sequence_finished == True:
                current_sequence = ParkingBodyCameraSequence.parking.value
                self.is_sequence_finished = False
        elif(current_sequence == ParkingBodyCameraSequence.parking.value):
            self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.bodycamera_parking_stop)
            
            if self.is_sequence_finished == True:
                current_sequence = ParkingBodyCameraSequence.changingtheta.value
                self.is_sequence_finished = False
        elif(current_sequence == ParkingBodyCameraSequence.changingtheta.value):
            self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.bodycamera_Changingtheta_threshold)
            
            if self.is_sequence_finished == True:
                current_sequence = ParkingBodyCameraSequence.decide.value
                self.is_sequence_finished = False
        elif(current_sequence == ParkingBodyCameraSequence.decide.value):
            self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.bodycamera_decide_distance)
            
            if self.is_sequence_finished == True:
                return
            elif self.is_sequence_finished == False:
                current_sequence = ParkingBodyCameraSequence.back.value
                self.is_sequence_finished = False
        elif(current_sequence == ParkingBodyCameraSequence.back.value):
            self.is_sequence_finished = self.action.fnseqmove_to_marker_dist(self.visual_servoing_action_server.bodycamera_back_distance)
            
            if self.is_sequence_finished == True:
                current_sequence = ParkingBodyCameraSequence.parking.value
                self.is_sequence_finished = False
 
