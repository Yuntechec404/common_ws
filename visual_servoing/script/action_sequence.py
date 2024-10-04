#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from enum import Enum
from action import Action
import time
import rclpy
ParkingBodyCameraSequence = Enum( 'ParkingBodyCameraSequence', \
                        'init_fork \
                        changing_direction \
                        move_nearby_parking_lot \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
ParkingForkCameraSequence = Enum( 'ParkingForkCameraSequence', \
                        'init_fork \
                        changing_direction \
                        parking \
                        changingtheta \
                        decide \
                        back \
                        stop \
                        error')
RaisePalletSequence = Enum( 'RaisePalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
                        stop \
                        error')
DropPalletSequence = Enum( 'DropPalletSequence', \
                        'init_fork \
                        dead_reckoning \
                        fork_updown \
                        back \
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
class ActionSequence():
    def __init__(self, VisualServoingActionServer):
        self.visual_servoing_action_server = VisualServoingActionServer  # access function {SpinOnce(), SpinOnce_Fork()} and parameter and Node{ROS2 API}
        self.action = Action(VisualServoingActionServer)
        self.is_sequence_finished = False

    def parking_bodycamera(self, goal_handle, layer):
        current_sequence = ParkingBodyCameraSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(ParkingBodyCameraSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == ParkingBodyCameraSequence.init_fork.value):
                self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.bodycamera_parking_fork_init)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changing_direction.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.changing_direction.value):
                self.visual_servoing_action_server.fnDetectionAllowed("allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                self.is_sequence_finished = self.action.fnSeqChangingDirection(self.visual_servoing_action_server.bodycamera_ChangingDirection_threshold, "bodycamera")

                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.move_nearby_parking_lot.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingBodyCameraSequence.move_nearby_parking_lot.value):
                self.is_sequence_finished = self.action.fnSeqMovingNearbyParkingLot(self.visual_servoing_action_server.bodycamera_desired_dist_threshold)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.parking.value):
                self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.bodycamera_parking_stop, 1.0, "bodycamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.changingtheta.value):
                self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.bodycamera_Changingtheta_threshold, "bodycamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.decide.value):
                self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.bodycamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingBodyCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingBodyCameraSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.bodycamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingBodyCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingBodyCameraSequence.stop.value):
                self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                return
            
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                return
            
    def parking_forkcamera(self, goal_handle, layer):
        current_sequence = ParkingForkCameraSequence.init_fork.value

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            feedback = str(ParkingForkCameraSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == ParkingForkCameraSequence.init_fork.value):
                if layer == 1.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.forkcamera_parking_fork_layer1)
                elif layer == 2.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.forkcamera_parking_fork_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False
            
            elif(current_sequence == ParkingForkCameraSequence.parking.value):
                self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                self.is_sequence_finished = self.action.fnSeqParking(self.visual_servoing_action_server.forkcamera_parking_stop, 1.0, "forkcamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.changingtheta.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.changingtheta.value):
                self.is_sequence_finished = self.action.fnSeqChangingtheta(self.visual_servoing_action_server.forkcamera_Changingtheta_threshold, "forkcamera")
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.decide.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.decide.value):
                self.is_sequence_finished = self.action.fnSeqdecide(self.visual_servoing_action_server.forkcamera_decide_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.stop.value
                    self.is_sequence_finished = False
                elif self.is_sequence_finished == False:
                    current_sequence = ParkingForkCameraSequence.back.value
                    self.is_sequence_finished = False
            elif(current_sequence == ParkingForkCameraSequence.back.value):
                self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.forkcamera_back_distance)
                
                if self.is_sequence_finished == True:
                    current_sequence = ParkingForkCameraSequence.parking.value
                    self.is_sequence_finished = False

            elif(current_sequence == ParkingForkCameraSequence.stop.value):
                self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                return
            
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
                return

    def raise_pallet(self, goal_handle, layer):
        current_sequence = RaisePalletSequence.init_fork.value
        time.sleep(5)

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
            feedback = str(RaisePalletSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == RaisePalletSequence.init_fork.value):
                if layer == 1.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_fork_init_layer1)
                elif layer == 2.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_fork_init_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.dead_reckoning.value
                    feedback = str(RaisePalletSequence(current_sequence))
                    self.visual_servoing_action_server.get_logger().info('fnseqDeadReckoning change to:{0}'.format(feedback))
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.dead_reckoning.value):
                self.visual_servoing_action_server.get_logger().info('fnseqDeadReckoning: {0}'.format(self.visual_servoing_action_server.raise_pallet_dead_reckoning_dist))
                self.is_sequence_finished = self.action.fnseqDeadReckoning(-self.visual_servoing_action_server.raise_pallet_dead_reckoning_dist)
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.fork_updown.value):
                if layer == 1.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_raise_height_layer1)
                elif layer == 2.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.raise_pallet_raise_height_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = RaisePalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == RaisePalletSequence.back.value):
                self.is_sequence_finished = self.action.fnseqDeadReckoning(-self.visual_servoing_action_server.raise_pallet_back_dist)

                if self.is_sequence_finished == True:
                    return
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
              
    def drop_pallet(self, goal_handle, layer):
        current_sequence = DropPalletSequence.init_fork.value
        time.sleep(5)

        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
            if(current_sequence == DropPalletSequence.init_fork.value):
                if layer == 1.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_fork_init_layer1)
                elif layer == 2.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_fork_init_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.dead_reckoning.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.dead_reckoning.value):
                if(self.visual_servoing_action_server.shelf_format):
                    self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.drop_pallet_dead_reckoning_dist)
                else:
                    self.is_sequence_finished = self.action.fnseqDeadReckoning(-self.visual_servoing_action_server.drop_pallet_dead_reckoning_dist)
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.fork_updown.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.fork_updown.value):
                if layer == 1.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_drop_height_layer1)
                elif layer == 2.0:
                    self.is_sequence_finished = self.action.fnForkUpdown(self.visual_servoing_action_server.drop_pallet_drop_height_layer2)
                else:
                    self.visual_servoing_action_server.get_logger().info('Layer is not defined')
                    return
                
                if self.is_sequence_finished == True:
                    current_sequence = DropPalletSequence.back.value
                    self.is_sequence_finished = False

            elif(current_sequence == DropPalletSequence.back.value):
                if(self.visual_servoing_action_server.shelf_format):
                    self.is_sequence_finished = self.action.fnseqMoveToMarkerDist(self.visual_servoing_action_server.drop_pallet_back_distance)
                else:
                    self.is_sequence_finished = self.action.fnseqDeadReckoning(-self.visual_servoing_action_server.drop_pallet_back_distance)
                if self.is_sequence_finished == True:
                    return
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
    
    def odom_front(self, goal_handle, dist):
        current_sequence = FrontSequence.Front.value
        self.is_sequence_finished = False
        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
            feedback = str(FrontSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == FrontSequence.Front.value):
                self.is_sequence_finished = self.action.fnseqDeadReckoning(dist)
                if self.is_sequence_finished == True:
                    return
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return
            
    def odom_turn(self, goal_handle, dist):
        current_sequence = TurnSequence.Turn.value
        self.is_sequence_finished = False
        while not goal_handle.is_cancel_requested:
            time.sleep(0.1)
            self.visual_servoing_action_server.fnDetectionAllowed("not_allowed","not_allowed")  # fnDetectionAllowed(self, shelf_string, pallet_string)
            feedback = str(TurnSequence(current_sequence))
            self.visual_servoing_action_server.get_logger().info('Feedback: {0}'.format(feedback))

            if(current_sequence == TurnSequence.Turn.value):
                self.is_sequence_finished = self.action.fnseqDeadReckoningAngle_2(dist)
                # if(dist < 0):
                #     self.is_sequence_finished = self.action.fnseqDeadReckoningAngle_2(dist,0.1, -2)
                # else:
                #     self.is_sequence_finished = self.action.fnseqDeadReckoningAngle(dist,0.1, 2)
                if self.is_sequence_finished == True:
                    return
                # else:
                #     self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                #     return 
            else:
                self.visual_servoing_action_server.get_logger().info('Error: {0} does not exist'.format(current_sequence))
                return