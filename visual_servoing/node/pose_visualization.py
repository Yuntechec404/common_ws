#!/usr/bin/env python3
# third party
import math # using math.pi
import tf_transformations # using euler_from_quaternion
import tkinter as tk # using Tkinter to visualize pose
# message
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from forklift_driver.msg import Meteorcar
# ROS2 
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
# The MultiThreadedExecutor can be used to execute multiple callbacks groups in multiple threads.
from rclpy.callback_groups import ReentrantCallbackGroup
# The ReentrantCallbackGroup creates a group of reentrant callbacks that can be called from multiple threads.


class PoseVisualization(Node):
    def __init__(self):
        super().__init__('VisualServoing_action_server')
        self.callback_group = ReentrantCallbackGroup()
        self.init_parame()
        self.get_parameters()
        self.create_subscriber()
    
        self.root = tk.Tk()
        self.root.title("Robot Visualization")

        self.robot_pose_label = tk.Label(self.root, text="Robot Pose: x=0.0, y=0.0, theta=0.0")
        self.robot_pose_label.pack()
        
        self.marker_pose_label = tk.Label(self.root, text="Marker Pose: x=0.0, y=0.0, theta=0.0")
        self.marker_pose_label.pack()
        
        self.fork_pose_label = tk.Label(self.root, text="Fork Position: 0.0")
        self.fork_pose_label.pack()
        
        self.update_gui()
        self.root.mainloop() 

    def __del__(self):
        self.destroy_node()
        rclpy.shutdown()

    def update_canvas(self):
        self.canvas.coords(self.robot_marker, self.robot_2d_pose_x * 50 + 250, self.robot_2d_pose_y * 50 + 250, self.robot_2d_pose_x * 50 + 260, self.robot_2d_pose_y * 50 + 260)
        self.canvas.coords(self.marker_marker, self.marker_2d_pose_x * 50 + 250, self.marker_2d_pose_y * 50 + 250, self.marker_2d_pose_x * 50 + 260, self.marker_2d_pose_y * 50 + 260)
        self.root.after(100, self.update_canvas)

    def update_gui(self):
        #self.log_info()
        rclpy.spin_once(self)
        self.robot_pose_label.config(text="Robot Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(
            self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta))
        self.marker_pose_label.config(text="Marker Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(
            self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta))
        self.fork_pose_label.config(text="Fork Position: {:.3f}".format(self.updownposition))
        self.root.after(100, self.update_gui)

    def init_parame(self):
        # Odometry_variable
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0
        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0
        # AprilTag_variable
        self.shelf_or_pallet = True   # True: apriltag, False: pallet
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
        self.declare_parameter('apriltag_topic', '/tag_detections')
        self.apriltag_topic = self.get_parameter('apriltag_topic').get_parameter_value().string_value
        self.declare_parameter('pallet_topic', '/pallet_detection')
        self.pallet_topic = self.get_parameter('pallet_topic').get_parameter_value().string_value
        self.declare_parameter('forkpose_topic', '/fork_pose')
        self.forkpose_topic = self.get_parameter('forkpose_topic').get_parameter_value().string_value

        self.get_logger().info("Get subscriber topic parameter")
        self.get_logger().info("odom_topic: {}, type: {}".format(self.odom_topic, type(self.odom_topic)))
        self.get_logger().info("apriltag_topic: {}, type: {}".format(self.apriltag_topic, type(self.apriltag_topic)))
        self.get_logger().info("pallet_topic: {}, type: {}".format(self.pallet_topic, type(self.pallet_topic)))
        self.get_logger().info("forkpose_topic: {}, type: {}".format(self.forkpose_topic, type(self.forkpose_topic)))

    def create_subscriber(self):
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.apriltag_sub = self.create_subscription(PoseArray, self.apriltag_topic, self.apriltag_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.pallet_sub = self.create_subscription(Pose, self.pallet_topic, self.pallet_callback, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)
        self.forkpose_sub = self.create_subscription(Meteorcar, self.forkpose_topic, self.cbGetforkpos, qos_profile=qos_profile_sensor_data, callback_group=self.callback_group)

    def log_info(self):
        rclpy.spin_once(self)
        self.get_logger().info("Odom: x={}, y={}, theta={}".format(
        self.robot_2d_pose_x, self.robot_2d_pose_y, self.robot_2d_theta))
        self.get_logger().info("Pose: x={:.3f}, y={:.3f}, theta={:.3f}".format(self.marker_2d_pose_x, self.marker_2d_pose_y, (self.marker_2d_theta*180/math.pi)))
        self.get_logger().info("Fork position: {}".format(self.updownposition))

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

    def apriltag_callback(self, msg):
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
        # self.get_logger().info("cbGetforkpos")
        self.updownposition = msg.fork_position

def main(args=None):
    rclpy.init(args=args)

    VisualServoing_action_server = PoseVisualization()
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
