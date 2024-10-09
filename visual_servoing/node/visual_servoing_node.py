#!/usr/bin/env python3
# third party
import math # using math.pi
import tf_transformations # using euler_from_quaternion
# message
from visual_servoing.action import VisualServoing
from geometry_msgs.msg import PoseArray, Pose, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from forklift_driver.msg import Meteorcar
from visp_megapose.msg import Confidence
import numpy as np
# ROS2 
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
# The MultiThreadedExecutor can be used to execute multiple callbacks groups in multiple threads.
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
# The ReentrantCallbackGroup creates a group of reentrant callbacks that can be called from multiple threads.
# The MutuallyExclusiveCallbackGroup creates a group of mutually exclusive callbacks that can be called from multiple threads.
# 動作流程
from action_sequence import ActionSequence
from action import Action
from dataclasses import dataclass

@dataclass
class DetectionConfidence:
    pallet_confidence: float
    pallet_detection: bool
    shelf_confidence: float
    shelf_detection: bool

class VisualServoingActionServer(Node):
    def __init__(self):
        super().__init__('VisualServoing_action_server')
        self.callback_group = ReentrantCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()
        self.init_parame()
        self.get_parameters()
        self.create_subscriber()
        self.init_kalman_filter()
        self.action_sequence = ActionSequence(self)
        self.action = Action(self)
        
        self._action_server = ActionServer(self, VisualServoing, 'VisualServoing', self.execute_callback, callback_group=self.callback_group2)
        self.fnDetectionAllowed("not_allowed","not_allowed")
        
    def fnDetectionAllowed(self, shelf_string, pallet_string):
        shelf_msg = String()
        shelf_msg.data = shelf_string
        self.shelf_detection_allowed_pub.publish(shelf_msg)
        
        pallet_msg = String()
        pallet_msg.data = pallet_string
        self.pallet_detection_allowed_pub.publish(pallet_msg)
        # self.get_logger().info("shelf_msg = {}, pallet_msg = {}".format(shelf_msg, pallet_msg))

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: Command={}, layer_dist={}'.format(goal_handle.request.command, goal_handle.request.layer_dist))
        if(goal_handle.request.command == "parking_bodycamera"):
            self.shelf_or_pallet = True  # True: shelf, False: pallet
            self.action_sequence.parking_bodycamera(goal_handle, goal_handle.request.layer_dist)
        elif(goal_handle.request.command == "parking_forkcamera"):
            self.shelf_or_pallet = False  # True: shelf, False: pallet
            self.action_sequence.parking_forkcamera(goal_handle, goal_handle.request.layer_dist)
        elif(goal_handle.request.command == "raise_pallet"):
            self.shelf_or_pallet = True
            self.action_sequence.raise_pallet(goal_handle, goal_handle.request.layer_dist)
        elif(goal_handle.request.command == "drop_pallet"):
            self.shelf_or_pallet = False
            self.action_sequence.drop_pallet(goal_handle, goal_handle.request.layer_dist)
        elif(goal_handle.request.command == "odom_front"):
            self.shelf_or_pallet = True
            self.action_sequence.odom_front(goal_handle, goal_handle.request.layer_dist)
        elif(goal_handle.request.command == "odom_turn"):
            self.shelf_or_pallet = True
            self.action_sequence.odom_turn(goal_handle, goal_handle.request.layer_dist)
        else:
            self.get_logger().info("Unknown command")
            goal_handle.abort()
            return VisualServoing.Result()
        
        goal_handle.succeed()
        result = VisualServoing.Result()
        result.result = "success"
        self.get_logger().info('Goal execution succeeded')
        return result
    
    def init_parame(self):
        # Odometry_variable
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_variable
        self.shelf_or_pallet = True   # True: shelf, False: pallet
        self.offset_x = 0.0
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        # Forklift_variable
        self.updownposition = 0.0
        # confidence_variable
        self.detectionConfidence = DetectionConfidence(
            pallet_confidence = 0.0,
            pallet_detection = False,
            shelf_confidence = 0.0,
            shelf_detection = False
        )

    def get_parameters(self):
        # get subscriber topic parameter
        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.declare_parameter('shelf_topic', '/tag_detections')
        self.shelf_topic = self.get_parameter('shelf_topic').get_parameter_value().string_value
        self.declare_parameter('pallet_topic', '/pallet_detection')
        self.pallet_topic = self.get_parameter('pallet_topic').get_parameter_value().string_value
        self.declare_parameter('forkpose_topic', '/fork_pose')
        self.forkpose_topic = self.get_parameter('forkpose_topic').get_parameter_value().string_value
        self.declare_parameter('shelf_format', True)
        self.shelf_format = self.get_parameter('shelf_format').get_parameter_value().bool_value
        self.declare_parameter('confidence_minimum', 0.5)
        self.confidence_minimum = self.get_parameter('confidence_minimum').get_parameter_value().double_value

        self.get_logger().info("Get subscriber topic parameter")
        self.get_logger().info("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        self.get_logger().info("shelf_topic: {}, type: {}".format(self.shelf_topic, type(self.shelf_topic)))
        self.get_logger().info("pallet_topic: {}, type: {}".format(self.pallet_topic, type(self.pallet_topic)))
        self.get_logger().info("forkpose_topic: {}, type: {}".format(self.forkpose_topic, type(self.forkpose_topic)))
        self.get_logger().info("shelf_format: {}, type: {}".format(self.shelf_format, type(self.shelf_format)))
        self.get_logger().info("confidence_minimum: {}, type: {}".format(self.confidence_minimum, type(self.confidence_minimum)))

        # get kalman filter parameter
        self.declare_parameter('kalman_enble', True)
        self.kalman_enble = self.get_parameter('kalman_enble').get_parameter_value().bool_value
        self.declare_parameter('process_variance', 2.5)
        self.process_variance = self.get_parameter('process_variance').get_parameter_value().double_value
        self.declare_parameter('measurement_variance', 0.2)
        self.measurement_variance = self.get_parameter('measurement_variance').get_parameter_value().double_value
        self.declare_parameter('initial_state', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.initial_state = self.get_parameter('initial_state').get_parameter_value().double_array_value
        self.declare_parameter('initial_uncertainty', 10.0)
        self.initial_uncertainty = self.get_parameter('initial_uncertainty').get_parameter_value().double_value

        self.get_logger().info("kalman_enble: {}, type: {}".format(self.kalman_enble, type(self.kalman_enble)))
        self.get_logger().info("process_variance: {}, type: {}".format(self.process_variance, type(self.process_variance)))
        self.get_logger().info("measurement_variance: {}, type: {}".format(self.measurement_variance, type(self.measurement_variance)))
        self.get_logger().info("initial_state: {}, type: {}".format(self.initial_state, type(self.initial_state)))
        self.get_logger().info("initial_uncertainty: {}, type: {}".format(self.initial_uncertainty, type(self.initial_uncertainty)))

        # get bodycamera parking parameter
        self.declare_parameter('bodycamera_tag_offset_x', 0.0)
        self.bodycamera_tag_offset_x = self.get_parameter('bodycamera_tag_offset_x').get_parameter_value().double_value
        self.declare_parameter('bodycamera_parking_fork_init', 0.0)
        self.bodycamera_parking_fork_init = self.get_parameter('bodycamera_parking_fork_init').get_parameter_value().double_value
        self.declare_parameter('bodycamera_ChangingDirection_threshold', 0.0)
        self.bodycamera_ChangingDirection_threshold = self.get_parameter('bodycamera_ChangingDirection_threshold').get_parameter_value().double_value
        self.declare_parameter('bodycamera_desired_dist_threshold', 0.0)
        self.bodycamera_desired_dist_threshold = self.get_parameter('bodycamera_desired_dist_threshold').get_parameter_value().double_value
        self.declare_parameter('bodycamera_parking_stop', 0.0)
        self.bodycamera_parking_stop = self.get_parameter('bodycamera_parking_stop').get_parameter_value().double_value
        self.declare_parameter('bodycamera_Changingtheta_threshold', 0.0)
        self.bodycamera_Changingtheta_threshold = self.get_parameter('bodycamera_Changingtheta_threshold').get_parameter_value().double_value
        self.declare_parameter('bodycamera_decide_distance', 0.0)
        self.bodycamera_decide_distance = self.get_parameter('bodycamera_decide_distance').get_parameter_value().double_value
        self.declare_parameter('bodycamera_back_distance', 0.0)
        self.bodycamera_back_distance = self.get_parameter('bodycamera_back_distance').get_parameter_value().double_value
        
        self.get_logger().info("Get bodycamera parking parameter")
        self.get_logger().info("bodycamera_tag_offset_x: {}, type: {}".format(self.bodycamera_tag_offset_x, type(self.bodycamera_tag_offset_x)))
        self.get_logger().info("bodycamera_parking_fork_init: {}, type: {}".format(self.bodycamera_parking_fork_init, type(self.bodycamera_parking_fork_init)))
        self.get_logger().info("bodycamera_ChangingDirection_threshold: {}, type: {}".format(self.bodycamera_ChangingDirection_threshold, type(self.bodycamera_ChangingDirection_threshold)))
        self.get_logger().info("bodycamera_desired_dist_threshold: {}, type: {}".format(self.bodycamera_desired_dist_threshold, type(self.bodycamera_desired_dist_threshold)))
        self.get_logger().info("bodycamera_parking_stop: {}, type: {}".format(self.bodycamera_parking_stop, type(self.bodycamera_parking_stop)))
        self.get_logger().info("bodycamera_Changingtheta_threshold: {}, type: {}".format(self.bodycamera_Changingtheta_threshold, type(self.bodycamera_Changingtheta_threshold)))
        self.get_logger().info("bodycamera_decide_distance: {}, type: {}".format(self.bodycamera_decide_distance, type(self.bodycamera_decide_distance)))
        self.get_logger().info("bodycamera_back_distance: {}, type: {}".format(self.bodycamera_back_distance, type(self.bodycamera_back_distance)))

        # get forkcamera parking parameter
        self.declare_parameter('forkcamera_parking_fork_layer1', 0.0)
        self.forkcamera_parking_fork_layer1 = self.get_parameter('forkcamera_parking_fork_layer1').get_parameter_value().double_value
        self.declare_parameter('forkcamera_parking_fork_layer2', 0.0)
        self.forkcamera_parking_fork_layer2 = self.get_parameter('forkcamera_parking_fork_layer2').get_parameter_value().double_value
        self.declare_parameter('forkcamera_tag_offset_x', 0.0)
        self.forkcamera_tag_offset_x = self.get_parameter('forkcamera_tag_offset_x').get_parameter_value().double_value
        self.declare_parameter('forkcamera_ChangingDirection_threshold', 0.0)
        self.forkcamera_ChangingDirection_threshold = self.get_parameter('forkcamera_ChangingDirection_threshold').get_parameter_value().double_value
        self.declare_parameter('forkcamera_desired_dist_threshold', 0.0)
        self.forkcamera_desired_dist_threshold = self.get_parameter('forkcamera_desired_dist_threshold').get_parameter_value().double_value
        self.declare_parameter('forkcamera_parking_stop', 0.0)
        self.forkcamera_parking_stop = self.get_parameter('forkcamera_parking_stop').get_parameter_value().double_value
        self.declare_parameter('forkcamera_Changingtheta_threshold', 0.0)
        self.forkcamera_Changingtheta_threshold = self.get_parameter('forkcamera_Changingtheta_threshold').get_parameter_value().double_value
        self.declare_parameter('forkcamera_decide_distance', 0.0)
        self.forkcamera_decide_distance = self.get_parameter('forkcamera_decide_distance').get_parameter_value().double_value
        self.declare_parameter('forkcamera_back_distance', 0.0)
        self.forkcamera_back_distance = self.get_parameter('forkcamera_back_distance').get_parameter_value().double_value

        self.get_logger().info("Get forkcamera parking parameter")
        self.get_logger().info("forkcamera_parking_fork_layer1: {}, type: {}".format(self.forkcamera_parking_fork_layer1, type(self.forkcamera_parking_fork_layer1)))
        self.get_logger().info("forkcamera_parking_fork_layer2: {}, type: {}".format(self.forkcamera_parking_fork_layer2, type(self.forkcamera_parking_fork_layer2)))
        self.get_logger().info("forkcamera_tag_offset_x: {}, type: {}".format(self.forkcamera_tag_offset_x, type(self.forkcamera_tag_offset_x)))
        self.get_logger().info("forkcamera_ChangingDirection_threshold: {}, type: {}".format(self.forkcamera_ChangingDirection_threshold, type(self.forkcamera_ChangingDirection_threshold)))
        self.get_logger().info("forkcamera_parking_stop: {}, type: {}".format(self.forkcamera_parking_stop, type(self.forkcamera_parking_stop)))
        self.get_logger().info("forkcamera_Changingtheta_threshold: {}, type: {}".format(self.forkcamera_Changingtheta_threshold, type(self.forkcamera_Changingtheta_threshold)))
        self.get_logger().info("forkcamera_decide_distance: {}, type: {}".format(self.forkcamera_decide_distance, type(self.forkcamera_decide_distance)))
        self.get_logger().info("forkcamera_back_distance: {}, type: {}".format(self.forkcamera_back_distance, type(self.forkcamera_back_distance)))

        # get raise_pallet parameter
        self.declare_parameter('raise_pallet_fork_init_layer1', 0.0)
        self.raise_pallet_fork_init_layer1 = self.get_parameter('raise_pallet_fork_init_layer1').get_parameter_value().double_value   
        self.declare_parameter('raise_pallet_fork_init_layer2', 0.0)
        self.raise_pallet_fork_init_layer2 = self.get_parameter('raise_pallet_fork_init_layer2').get_parameter_value().double_value
        self.declare_parameter('raise_pallet_dead_reckoning_dist', 0.0)
        self.raise_pallet_dead_reckoning_dist = self.get_parameter('raise_pallet_dead_reckoning_dist').get_parameter_value().double_value
        self.declare_parameter('raise_pallet_raise_height_layer1', 0.0)
        self.raise_pallet_raise_height_layer1 = self.get_parameter('raise_pallet_raise_height_layer1').get_parameter_value().double_value
        self.declare_parameter('raise_pallet_raise_height_layer2', 0.0)
        self.raise_pallet_raise_height_layer2 = self.get_parameter('raise_pallet_raise_height_layer2').get_parameter_value().double_value
        self.declare_parameter('raise_pallet_back_dist', 0.0)
        self.raise_pallet_back_dist = self.get_parameter('raise_pallet_back_dist').get_parameter_value().double_value
        self.declare_parameter('raise_pallet_navigation_helght', 0.0)
        self.raise_pallet_navigation_helght = self.get_parameter('raise_pallet_navigation_helght').get_parameter_value().double_value

        self.get_logger().info("Get raise_pallet parameter")
        self.get_logger().info("raise_pallet_fork_init_layer1: {}, type: {}".format(self.raise_pallet_fork_init_layer1, type(self.raise_pallet_fork_init_layer1)))
        self.get_logger().info("raise_pallet_fork_init_layer2: {}, type: {}".format(self.raise_pallet_fork_init_layer2, type(self.raise_pallet_fork_init_layer2)))
        self.get_logger().info("raise_pallet_dead_reckoning_dist: {}, type: {}".format(self.raise_pallet_dead_reckoning_dist, type(self.raise_pallet_dead_reckoning_dist)))
        self.get_logger().info("raise_pallet_raise_height_layer1: {}, type: {}".format(self.raise_pallet_raise_height_layer1, type(self.raise_pallet_raise_height_layer1)))
        self.get_logger().info("raise_pallet_raise_height_layer2: {}, type: {}".format(self.raise_pallet_raise_height_layer2, type(self.raise_pallet_raise_height_layer2)))
        self.get_logger().info("raise_pallet_back_dist: {}, type: {}".format(self.raise_pallet_back_dist, type(self.raise_pallet_back_dist)))
        self.get_logger().info("raise_pallet_navigation_helght: {}, type: {}".format(self.raise_pallet_navigation_helght, type(self.raise_pallet_navigation_helght)))

        # get drop_pallet parameter
        self.declare_parameter('drop_pallet_fork_init_layer1', 0.0)
        self.drop_pallet_fork_init_layer1 = self.get_parameter('drop_pallet_fork_init_layer1').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_fork_init_layer2', 0.0)
        self.drop_pallet_fork_init_layer2 = self.get_parameter('drop_pallet_fork_init_layer2').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_dead_reckoning_dist', 0.0)
        self.drop_pallet_dead_reckoning_dist = self.get_parameter('drop_pallet_dead_reckoning_dist').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_fork_forward_distance', 0.0)
        self.drop_pallet_fork_forward_distance = self.get_parameter('drop_pallet_fork_forward_distance').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_drop_height_layer1', 0.0)
        self.drop_pallet_drop_height_layer1 = self.get_parameter('drop_pallet_drop_height_layer1').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_drop_height_layer2', 0.0)
        self.drop_pallet_drop_height_layer2 = self.get_parameter('drop_pallet_drop_height_layer2').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_back_distance', 0.0)
        self.drop_pallet_back_distance = self.get_parameter('drop_pallet_back_distance').get_parameter_value().double_value
        self.declare_parameter('drop_pallet_navigation_helght', 0.0)
        self.drop_pallet_navigation_helght = self.get_parameter('drop_pallet_navigation_helght').get_parameter_value().double_value

        self.get_logger().info("Get drop_pallet parameter")
        self.get_logger().info("drop_pallet_fork_init_layer1: {}, type: {}".format(self.drop_pallet_fork_init_layer1, type(self.drop_pallet_fork_init_layer1)))
        self.get_logger().info("drop_pallet_fork_init_layer2: {}, type: {}".format(self.drop_pallet_fork_init_layer2, type(self.drop_pallet_fork_init_layer2)))
        self.get_logger().info("drop_pallet_dead_reckoning_dist: {}, type: {}".format(self.drop_pallet_dead_reckoning_dist, type(self.drop_pallet_dead_reckoning_dist)))
        self.get_logger().info("drop_pallet_fork_forward_distance: {}, type: {}".format(self.drop_pallet_fork_forward_distance, type(self.drop_pallet_fork_forward_distance)))
        self.get_logger().info("drop_pallet_drop_height_layer1: {}, type: {}".format(self.drop_pallet_drop_height_layer1, type(self.drop_pallet_drop_height_layer1)))
        self.get_logger().info("drop_pallet_drop_height_layer2: {}, type: {}".format(self.drop_pallet_drop_height_layer2, type(self.drop_pallet_drop_height_layer2)))
        self.get_logger().info("drop_pallet_back_distance: {}, type: {}".format(self.drop_pallet_back_distance, type(self.drop_pallet_back_distance)))
        self.get_logger().info("drop_pallet_navigation_helght: {}, type: {}".format(self.drop_pallet_navigation_helght, type(self.drop_pallet_navigation_helght)))

    def create_subscriber(self):
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.pallet_sub = self.create_subscription(Pose, self.pallet_topic, self.pallet_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.forkpose_sub = self.create_subscription(Meteorcar, self.forkpose_topic, self.cbGetforkpos, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.pallet_confidence_sub = self.create_subscription(Confidence, self.pallet_topic + "_confidence", self.cbPalletConfidence, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1, callback_group=self.callback_group)
        self.fork_pub = self.create_publisher(Meteorcar, "/cmd_fork", 1, callback_group=self.callback_group)
        self.pallet_detection_allowed_pub = self.create_publisher(String, self.pallet_topic + "_detection_allowed", 1, callback_group=self.callback_group)
        
        if self.kalman_enble == True:
            self.pallet_kalman_pub = self.create_publisher(Pose, f"{self.pallet_topic}_filtered", 1, callback_group=self.callback_group)
            if(self.shelf_format == False):
                self.shelf_kalman_pub = self.create_publisher(Pose, f"{self.shelf_topic}_filtered", 1, callback_group=self.callback_group)

        if(self.shelf_format == True):
            self.shelf_sub = self.create_subscription(PoseArray, self.shelf_topic, self.shelf_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        else:
            self.shelf_confidence_sub = self.create_subscription(Confidence, self.shelf_topic + "_confidence", self.cbShelfConfidence, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
            self.shelf_detection_allowed_pub = self.create_publisher(String, self.shelf_topic + "_detection_allowed", 1, callback_group=self.callback_group)
            self.shelf_sub = self.create_subscription(Pose, self.shelf_topic, self.shelf_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)

    def init_kalman_filter(self):
        self.check_wait_time = 0

        self.kf = KalmanFilter_(
            self.process_variance, 
            self.measurement_variance,  
            self.initial_state, 
            self.initial_uncertainty
        )

    def log_info(self):
        rclpy.spin_once(self)
        self.get_logger().info("Odom: x={}, y={}, theta={}".format(
        self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta))
        self.get_logger().info("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, (self.marker_2d_theta*180/math.pi)))
        self.get_logger().info("Fork position: {}".format(self.updownposition))

    def SpinOnce(self):
        # rclpy.spin_once(self)
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
    
    def SpinOnce_fork(self):
        # rclpy.spin_once(self)
        return self.updownposition
    
    def SpinOnce_confidence(self):
        # rclpy.spin_once(self)
        return self.detectionConfidence

    def odom_callback(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf_transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta = theta + math.pi * 2
        if theta > math.pi * 2:
            theta = theta - math.pi * 2

        self.robot_2d_pose_x = msg.pose.pose.position.x
        self.robot_2d_pose_y = msg.pose.pose.position.y
        self.robot_2d_theta = theta

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta = self.total_robot_2d_theta + d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta
        # if self.action.vel_state_getter():
        #     self.get_logger().info("Robot Moving")
        # else :
        #     self.get_logger().info("Robot Stop")

    def shelf_callback(self, msg):
        # self.get_logger().info("Shelf callback")
        try:
            if self.shelf_or_pallet == True:
                if(self.shelf_sub.msg_type == PoseArray):
                    marker_msg = msg.poses[0]
                else:
                    if self.kalman_enble:
                        msg_filter = self.kalman_execute(msg, 'shelf')
                    marker_msg = msg_filter
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf_transformations.euler_from_quaternion(quaternion)[1]
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x + self.offset_x
                self.marker_2d_theta = -theta
                # self.get_logger().info("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
            else:
                pass
        except:
            pass

    def pallet_callback(self, msg):
        # self.get_logger().info("Pallet callback")
        try:
            if self.shelf_or_pallet == False:
                if self.kalman_enble:
                    msg_filter = self.kalman_execute(msg, 'pallet')
                marker_msg = msg_filter
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf_transformations.euler_from_quaternion(quaternion)[1]
                self.marker_2d_pose_x = -marker_msg.position.z
                self.marker_2d_pose_y = marker_msg.position.x + self.offset_x
                self.marker_2d_theta = -theta
                # self.get_logger().info("pallet Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
            else:
                pass
        except:
            pass
  
    def cbGetforkpos(self, msg):
        # self.get_logger().info("cbGetforkpos")
        self.updownposition = msg.fork_position
        
    def cbPalletConfidence(self, msg):
        self.detectionConfidence.pallet_confidence = msg.object_confidence
        self.detectionConfidence.pallet_detection = msg.model_detection

    def cbShelfConfidence(self, msg):
        self.detectionConfidence.shelf_confidence = msg.object_confidence
        self.detectionConfidence.shelf_detection = msg.model_detection

    def kalman_execute(self, msg, object_name):
        # timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        # 收到的測量數據
        measurement = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

        # 濾波器的預測和更新
        self.kf.predict()
        self.kf.update(measurement)

        # 得到經過濾波的狀態
        filtered_state = self.kf.get_state()

        # 構建過濾後的 Pose 消息
        filtered_pose = Pose()
        filtered_pose.position.x = filtered_state[0]  # x
        filtered_pose.position.y = filtered_state[2]  # y
        filtered_pose.position.z = filtered_state[4]  # z
        filtered_pose.orientation.x = filtered_state[6]
        filtered_pose.orientation.y = filtered_state[7]
        filtered_pose.orientation.z = filtered_state[8]
        filtered_pose.orientation.w = filtered_state[9]

        # 發布過濾後的 Pose
        if object_name == 'pallet':
            self.pallet_kalman_pub.publish(filtered_pose)
        elif object_name == 'shelf':
            self.shelf_kalman_pub.publish(filtered_pose)
        return filtered_pose

class KalmanFilter_:
    def __init__(self, process_variance, measurement_variance, initial_state, initial_uncertainty):
        if len(initial_state) != 10:
            raise ValueError("initial_state 必須包含10個元素")
        self.state = np.array(initial_state)
        self.uncertainty = np.eye(len(self.state)) * initial_uncertainty
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        
        # 狀態轉移矩陣
        self.A = np.eye(len(self.state))
        self.A[0, 1] = 1  # x 和速度 vx
        self.A[2, 3] = 1  # y 和速度 vy
        self.A[4, 5] = 1  # z 和速度 vz

        # 測量矩陣
        self.H = np.zeros((7, 10))
        self.H[0, 0] = 1  # x
        self.H[1, 2] = 1  # y
        self.H[2, 4] = 1  # z
        self.H[3, 6] = 1  # qx
        self.H[4, 7] = 1  # qy
        self.H[5, 8] = 1  # qz
        self.H[6, 9] = 1  # qw

        # 過程噪聲和測量噪聲
        self.Q = np.eye(len(self.state)) * self.process_variance
        self.R = np.eye(7) * self.measurement_variance

    def predict(self):
        self.state = self.A @ self.state
        self.uncertainty = self.A @ self.uncertainty @ self.A.T + self.Q

    def update(self, measurement):
        z = np.array(measurement)
        y = z - self.H @ self.state  # 殘差
        S = self.H @ self.uncertainty @ self.H.T + self.R
        K = self.uncertainty @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.uncertainty = (np.eye(len(self.state)) - K @ self.H) @ self.uncertainty

        # 歸一化四元数
        quaternion = self.state[6:10]  # 取 qx, qy, qz, qw
        quaternion /= np.linalg.norm(quaternion)  # 歸一化
        self.state[6:10] = quaternion

    def get_state(self):
        return self.state

def main(args=None):
    rclpy.init(args=args)

    VisualServoing_action_server = VisualServoingActionServer()
    executor = MultiThreadedExecutor(num_threads=3)

    try:
        executor.add_node(VisualServoing_action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        VisualServoing_action_server.destroy_node()
    # try:
    #     rclpy.spin(VisualServoing_action_server)
    # except KeyboardInterrupt:
    #     pass


if __name__ == '__main__':
    main()