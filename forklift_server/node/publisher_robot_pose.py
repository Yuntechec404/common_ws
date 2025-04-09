#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import Pose

class MapToBaselink:
    def __init__(self):
        self.pub = rospy.Publisher(pub_name, Pose, queue_size=1)
        self.listener = tf.TransformListener()
        self.pose_msg = Pose()
        self.buf = [0]*7

    def get_transform(self):
        try:
            listener_time = rospy.Time(0)
            self.listener.waitForTransform('map', 'base_link', listener_time, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform('map', 'base_link', listener_time)
            transform_pose = trans.copy() + rot.copy()
            self.publish_pose(pose=transform_pose)
            self.buf = transform_pose

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.publish_pose(pose=self.buf)

    def publish_pose(self, pose = [0]*7):
        self.pose_msg.position.x = pose[0]
        self.pose_msg.position.y = pose[1]
        self.pose_msg.position.z = pose[2]
        self.pose_msg.orientation.x = pose[3]
        self.pose_msg.orientation.y = pose[4]
        self.pose_msg.orientation.z = pose[5]
        self.pose_msg.orientation.w = pose[6]

        self.pub.publish(self.pose_msg)

if __name__ == "__main__":
    rospy.init_node("Robot_pose")
    pub_name = rospy.get_param('~publish_name', '/robot_pose')
    rate = int(rospy.get_param('~rate', 10))
    robot = MapToBaselink()
    r = rospy.Rate(rate)    
    while not rospy.is_shutdown():
        robot.get_transform()
        r.sleep()