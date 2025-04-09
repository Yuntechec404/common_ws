#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import geometry_msgs.msg
from forklift_msg.msg import meteorcar
import tf_conversions

class ForkliftTFBroadcaster:
    def __init__(self):
        rospy.init_node("dynamic_forklift_tf")
        
        self.forkpos = rospy.get_param(rospy.get_name() + "/forkpos", "/forkpos")
        self.parent_frame_id = rospy.get_param(rospy.get_name() + "/parent_frame_id", "base_link")
        self.child_frame_id = rospy.get_param(rospy.get_name() + "/child_frame_id", "camera_link")
        self.translation = eval(rospy.get_param(rospy.get_name() + "/translation", "[0.0, 0.0, 0.0]"))
        self.rotation = eval(rospy.get_param(rospy.get_name() + "/rotation", "[0.0, 0.0, 0.0]"))
        self.rate = rospy.get_param(rospy.get_name() + "/rate", 10)

        rospy.loginfo("forkpos: {}, type: {}".format(self.forkpos, type(self.forkpos)))
        rospy.loginfo("parent_frame_id: {}, type: {}".format(self.parent_frame_id, type(self.parent_frame_id)))
        rospy.loginfo("child_frame_id: {}, type: {}".format(self.child_frame_id, type(self.child_frame_id)))
        rospy.loginfo("translation: {}, type: {}".format(self.translation, type(self.translation)))
        rospy.loginfo("rotation: {}, type: {}".format(self.rotation, type(self.rotation)))
        rospy.loginfo("rate: {}, type: {}".format(self.rate, type(self.rate)))

        self.updownposition = 0.0

        self.forkpose_sub = rospy.Subscriber(self.forkpos, meteorcar, self.cbGetforkpos, queue_size = 1)

        # 建立 TF 廣播器
        self.br = tf2_ros.TransformBroadcaster()
        self.publish_transform()

    def cbGetforkpos(self, msg):
        self.updownposition = msg.fork_position

    def publish_transform(self):
        """持續發布 TF 變換"""
        rate = rospy.Rate(self.rate)  # 10 Hz 更新 TF
        while not rospy.is_shutdown():
            transform_stamped = geometry_msgs.msg.TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = self.parent_frame_id
            transform_stamped.child_frame_id = self.child_frame_id

            # 設定位移 (XYZ)
            transform_stamped.transform.translation.x = self.translation[0]
            transform_stamped.transform.translation.y = self.translation[1]
            transform_stamped.transform.translation.z = self.translation[2] + self.updownposition   # Z 軸隨 /forklift_pose 變化

            # 設定旋轉 (RPY → Quaternion)
            q = tf_conversions.transformations.quaternion_from_euler(self.rotation[0], self.rotation[1], self.rotation[2])
            transform_stamped.transform.rotation.x = q[0]
            transform_stamped.transform.rotation.y = q[1]
            transform_stamped.transform.rotation.z = q[2]
            transform_stamped.transform.rotation.w = q[3]

            # 發布 TF 變換
            self.br.sendTransform(transform_stamped)

            rate.sleep()

if __name__ == "__main__":
    try:
        ForkliftTFBroadcaster()
    except rospy.ROSInterruptException:
        pass
