#!/usr/bin/env python3
import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ackermann_static_tf2:
    def __init__(self):
        rospy.init_node('ackermann_static_tf2')
        self.broadcaster_tf = StaticTransformBroadcaster()

    def make_transforms(self):
        odom_to_map = TransformStamped()
        odom_to_map.header.stamp = rospy.Time.now()
        odom_to_map.header.frame_id = "map"
        odom_to_map.child_frame_id = "odom"
        odom_to_map.transform.translation.x = 0.0
        odom_to_map.transform.translation.y = 0.0
        odom_to_map.transform.translation.z = 0.0
        odom_to_map.transform.rotation.w = 1.0
        odom_to_map.transform.rotation.x = 0.0
        odom_to_map.transform.rotation.y = 0.0
        odom_to_map.transform.rotation.z = 0.0

        return [
                odom_to_map
        ]

    def run(self):
        self.broadcaster_tf.sendTransform(self.make_transforms())
        rospy.spin()

if __name__ == '__main__':
    ack_static_tf2 = ackermann_static_tf2()
    try:
        ack_static_tf2.run()
    except rospy.ROSInterruptException:
        pass