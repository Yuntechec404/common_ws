#!/usr/bin/env python3
# third party
import math # using math.pi
import tf_transformations # using euler_from_quaternion
# message
from visual_servoing.action import VisualServoing
from geometry_msgs.msg import PoseArray, Pose, Twist
from nav_msgs.msg import Odometry
from forklift_driver.msg import Meteorcar
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

class VisualServoingActionServer(Node):
    def __init__(self):
        super().__init__('VisualServoing_action_server')
        self.callback_group = ReentrantCallbackGroup()
        self.callback_group2 = MutuallyExclusiveCallbackGroup()
        self.init_parame()
        self.get_parameters()
        self.create_subscriber()
        self.action_sequence = ActionSequence(self)
        
        self._action_server = ActionServer(self, VisualServoing, 'VisualServoing', self.execute_callback, callback_group=self.callback_group2)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal: Command={}, Layer={}'.format(goal_handle.request.command, goal_handle.request.layer))
        if(goal_handle.request.command == "parking_bodycamera"):
            self.shelf_or_pallet = True  # True: shelf, False: pallet
            self.action_sequence.parking_bodycamera(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "parking_forkcamera"):
            self.shelf_or_pallet = False  # True: shelf, False: pallet
            self.action_sequence.parking_forkcamera(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "raise_pallet"):
            self.shelf_or_pallet = False
            self.action_sequence.raise_pallet(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "drop_pallet"):
            self.shelf_or_pallet = True
            self.action_sequence.drop_pallet(goal_handle, goal_handle.request.layer)
        elif(goal_handle.request.command == "fruit_docking"):
            self.shelf_or_pallet = False
            self.action_sequence.fruit_docking(goal_handle, goal_handle.request.layer)
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
        self.marker_2d_pose_z = 0.0

        self.marker_2d_theta = 0.0
        # pallet variable
        self.pallet_2d_pose_x = 0.0
        self.pallet_2d_pose_y = 0.0
        self.pallet_2d_theta = 0.0
        self.pallet_2d_pose_z = 0.0  # 新增的z轴属性
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
        self.get_logger().info("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        self.get_logger().info("shelf_topic: {}, type: {}".format(self.shelf_topic, type(self.shelf_topic)))
        self.get_logger().info("pallet_topic: {}, type: {}".format(self.pallet_topic, type(self.pallet_topic)))
        self.get_logger().info("forkpose_topic: {}, type: {}".format(self.forkpose_topic, type(self.forkpose_topic)))

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

        # get fruit_docking parameter
        self.declare_parameter('forkcamera_x_pose_hreshold', 0.0)
        self.forkcamera_x_pose_hreshold = self.get_parameter('forkcamera_x_pose_hreshold').get_parameter_value().double_value
        self.declare_parameter('fruit_dead_reckoning_dist', 0.0)
        self.fruit_dead_reckoning_dist = self.get_parameter('fruit_dead_reckoning_dist').get_parameter_value().double_value



    def create_subscriber(self):
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.shelf_sub = self.create_subscription(PoseArray, self.shelf_topic, self.shelf_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.pallet_sub = self.create_subscription(Pose, self.pallet_topic, self.pallet_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.forkpose_sub = self.create_subscription(Meteorcar, self.forkpose_topic, self.cbGetforkpos, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1, callback_group=self.callback_group)
        self.fork_pub = self.create_publisher(Meteorcar, "/cmd_fork", 1, callback_group=self.callback_group)

    def log_info(self):
        rclpy.spin_once(self)
        self.get_logger().info("Odom: x={}, y={}, theta={}".format(
        self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta))
        self.get_logger().info("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, (self.marker_2d_theta*180/math.pi)))
        self.get_logger().info("Fork position: {}".format(self.updownposition))

    def SpinOnce(self):
        # rclpy.spin_once(self)
        return self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta, \
               self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta ,self.pallet_2d_pose_x ,self.pallet_2d_pose_y ,self.pallet_2d_pose_z
    
    def SpinOnce_fork(self):
        # rclpy.spin_once(self)
        return self.updownposition

    def odom_callback(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        theta = tf_transformations.euler_from_quaternion(quaternion)[2]
        if theta < 0:
            theta = theta + math.pi * 2
        if theta > math.pi * 2:        print(f"Target pallet_2d_pose_z: {self.marker_2d_pose_x:.10f}")


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
        # self.get_logger().info("Shelf callbpallet_callbackack")
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
                marker_msg = msg
                quaternion = (marker_msg.orientation.x, marker_msg.orientation.y, marker_msg.orientation.z, marker_msg.orientation.w)
                theta = tf_transformations.euler_from_quaternion(quaternion)[1]
                self.pallet_2d_pose_x = -marker_msg.position.z
                self.pallet_2d_pose_y = marker_msg.position.x + self.offset_x
                self.pallet_2d_pose_z = marker_msg.position.y  # 更新z轴信息

                self.marker_2d_theta = -theta
                # self.get_logger().info("pallet Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
            else:
                pass
        except:
            pass

    def cbGetforkpos(self, msg):
        # self.get_logger().info("cbGetforkpos")
        self.updownposition = msg.fork_position

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