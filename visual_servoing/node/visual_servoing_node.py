#!/usr/bin/env python3
# third party
import time
import math # math.pi
import tf_transformations # euler_from_quaternion
# message
from visual_servoing.action import VisualServoing
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from forklift_driver.msg import Meteorcar
# ROS2 
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# 動作流程
from action_sequence import ActionSequences

class VisualServoingActionServer(Node):
    def __init__(self):
        super().__init__('VisualServoing_action_server')
        self.init_parame()
        self.get_parameters()
        self.create_subscriber()
        self.action_sequence = ActionSequences(self)
        self._action_server = ActionServer(self, VisualServoing, 'VisualServoing', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: Command={}, Layer={}'.format(goal_handle.request.command, goal_handle.request.layer))
        if(goal_handle.request.command == "parking_bodycamera"):
            self.action_sequence.parking_bodycamera(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "parking_forkcamera"):
            self.action_sequence.parking_forkcamera(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "raise_pallet"):
            self.action_sequence.raise_pallet(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "drop_pallet"):
            self.action_sequence.drop_pallet(goal_handle, goal_handle.request.layer)

        goal_handle.succeed()
        result = VisualServoing.Result()
        result.result = "success"
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

        self.get_logger().info("Get subscriber topic parameter")
        self.get_logger().info("odom_topic: {}".format(self.odom_topic))
        self.get_logger().info("shelf_topic: {}".format(self.shelf_topic))
        self.get_logger().info("pallet_topic: {}".format(self.pallet_topic))
        self.get_logger().info("forkpose_topic: {}".format(self.forkpose_topic))

        # get bodycamera parking parameter
        self.declare_parameter('bodycamera_tag_offset_x', -0.03)
        self.bodycamera_tag_offset_x = self.get_parameter('bodycamera_tag_offset_x').get_parameter_value().double_value
        self.declare_parameter('bodycamera_parking_fork_init', -0.1)
        self.bodycamera_parking_fork_init = self.get_parameter('bodycamera_parking_fork_init').get_parameter_value().double_value
        self.declare_parameter('bodycamera_ChangingDirection_threshold', 0.01)
        self.bodycamera_ChangingDirection_threshold = self.get_parameter('bodycamera_ChangingDirection_threshold').get_parameter_value().double_value
        self.declare_parameter('bodycamera_parking_stop', 0.5)
        self.bodycamera_parking_stop = self.get_parameter('bodycamera_parking_stop').get_parameter_value().double_value
        self.declare_parameter('bodycamera_Changingtheta_threshold', 0.025)
        self.bodycamera_Changingtheta_threshold = self.get_parameter('bodycamera_Changingtheta_threshold').get_parameter_value().double_value
        self.declare_parameter('bodycamera_decide_distance', 0.025)
        self.bodycamera_decide_distance = self.get_parameter('bodycamera_decide_distance').get_parameter_value().double_value
        self.declare_parameter('bodycamera_back_distance', 1.10)
        self.bodycamera_back_distance = self.get_parameter('bodycamera_back_distance').get_parameter_value().double_value
        # get forkcamera parking parameter
        self.declare_parameter('forkcamera_parking_fork_layer1', 0.0)
        self.forkcamera_parking_fork_layer1 = self.get_parameter('forkcamera_parking_fork_layer1').get_parameter_value().double_value
        self.declare_parameter('forkcamera_parking_fork_layer2', 0.3)
        self.forkcamera_parking_fork_layer2 = self.get_parameter('forkcamera_parking_fork_layer2').get_parameter_value().double_value
        self.declare_parameter('forkcamera_tag_offset_x', 0.0)

    def create_subscriber(self):
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data)
        self.shelf_sub = self.create_subscription(PoseArray, self.shelf_topic, self.shelf_callback, qos_profile=qos_profile_sensor_data)
        self.pallet_sub = self.create_subscription(Pose, self.pallet_topic, self.pallet_callback, qos_profile=qos_profile_sensor_data)
        self.forkpose_sub = self.create_subscription(Meteorcar, self.forkpose_topic, self.cbGetforkpos, qos_profile=qos_profile_sensor_data)
    
    def log_info(self):
        rclpy.spin_once(self)
        # self.get_logger().info("Odom: x={}, y={}, theta={}".format(
        #     self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta))
        self.get_logger().info("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, (self.marker_2d_theta*180/math.pi)))
        # self.get_logger().info("Fork position: {}".format(self.updownposition))
        
    def SpinOnce(self):
        rclpy.spin_once(self)
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta
    
    def SpinOnce_fork(self):
        rclpy.spin_once(self)
        return self.updownposition

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

    def shelf_callback(self, msg):
        # self.get_logger().info("Shelf callback")
        try:
            if self.shelf_or_pallet == True:
                marker_msg = msg.poses[0]
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
                marker_msg = msg.pose
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

    def cbGetforkpos(self, msg):
        self.updownposition = msg.fork_position

def main(args=None):
    rclpy.init(args=args)

    VisualServoing_action_server = VisualServoingActionServer()
    try:
        rclpy.spin(VisualServoing_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()