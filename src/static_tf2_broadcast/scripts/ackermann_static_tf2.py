#!/usr/bin/env python3
import rospy
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ackermann_static_tf2:
    def __init__(self):
        rospy.init_node('ackermann_static_tf2')
        print("Static TF is Working ...")
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

        base_link_to_base_footprint = TransformStamped()
        base_link_to_base_footprint.header.stamp = rospy.Time.now()
        base_link_to_base_footprint.header.frame_id = "base_footprint"
        base_link_to_base_footprint.child_frame_id = "base_link"
        base_link_to_base_footprint.transform.translation.x = 0.0
        base_link_to_base_footprint.transform.translation.y = 0.0
        base_link_to_base_footprint.transform.translation.z = 0.0
        base_link_to_base_footprint.transform.rotation.w = 1.0
        base_link_to_base_footprint.transform.rotation.x = 0.0
        base_link_to_base_footprint.transform.rotation.y = 0.0
        base_link_to_base_footprint.transform.rotation.z = 0.0

        imu_link_to_base_link = TransformStamped()
        imu_link_to_base_link.header.stamp = rospy.Time.now()
        imu_link_to_base_link.header.frame_id = "base_link"
        imu_link_to_base_link.child_frame_id = "bno055"
        imu_link_to_base_link.transform.translation.x = 4.10
        imu_link_to_base_link.transform.translation.y = 0.14
        imu_link_to_base_link.transform.translation.z = 0.585
        imu_link_to_base_link.transform.rotation.w = 1.0
        imu_link_to_base_link.transform.rotation.x = 0.0
        imu_link_to_base_link.transform.rotation.y = 0.0 
        imu_link_to_base_link.transform.rotation.z = 0.0

        gps_front_to_base_link = TransformStamped()
        gps_front_to_base_link.header.stamp = rospy.Time.now()     
        gps_front_to_base_link.header.frame_id = "base_link"
        gps_front_to_base_link.child_frame_id = "gps_front"
        gps_front_to_base_link.transform.translation.x = 5.26 #3.165 base at center
        gps_front_to_base_link.transform.translation.y = 1.345
        gps_front_to_base_link.transform.translation.z = 0.6975
        gps_front_to_base_link.transform.rotation.w = 1.0
        gps_front_to_base_link.transform.rotation.x = 0.0
        gps_front_to_base_link.transform.rotation.y = 0.0
        gps_front_to_base_link.transform.rotation.z = 0.0

        rslidar_front_to_base_link = TransformStamped()
        rslidar_front_to_base_link.header.stamp = rospy.Time.now()     
        rslidar_front_to_base_link.header.frame_id = "base_link"
        rslidar_front_to_base_link.child_frame_id = "rslidar_front"
        rslidar_front_to_base_link.transform.translation.x = 5.26 #3.165 base at center
        rslidar_front_to_base_link.transform.translation.y = 1.345
        rslidar_front_to_base_link.transform.translation.z = 0.6125
        rslidar_front_to_base_link.transform.rotation.w = 0.0
        rslidar_front_to_base_link.transform.rotation.x = -0.707
        rslidar_front_to_base_link.transform.rotation.y = -0.707
        rslidar_front_to_base_link.transform.rotation.z = 0.0

        laser_to_base_link = TransformStamped()
        laser_to_base_link.header.stamp = rospy.Time.now()
        laser_to_base_link.header.frame_id = "base_link"
        laser_to_base_link.child_frame_id = "laser"
        laser_to_base_link.transform.translation.x = 0.0
        laser_to_base_link.transform.translation.y = 0.0
        laser_to_base_link.transform.translation.z = 0.0 #0.6125
        laser_to_base_link.transform.rotation.w = 1.0
        laser_to_base_link.transform.rotation.x = 0.0
        laser_to_base_link.transform.rotation.y = 0.0
        laser_to_base_link.transform.rotation.z = 0.0

        gps_back_to_base_link = TransformStamped()
        gps_back_to_base_link.header.stamp = rospy.Time.now()     
        gps_back_to_base_link.header.frame_id = "base_link"
        gps_back_to_base_link.child_frame_id = "gps_back"
        gps_back_to_base_link.transform.translation.x = -1.07
        gps_back_to_base_link.transform.translation.y = -1.345
        gps_back_to_base_link.transform.translation.z = 0.6975
        gps_back_to_base_link.transform.rotation.w = 1.0
        gps_back_to_base_link.transform.rotation.x = 0.0
        gps_back_to_base_link.transform.rotation.y = 0.0
        gps_back_to_base_link.transform.rotation.z = 0.0

        rslidar_back_to_base_link = TransformStamped()
        rslidar_back_to_base_link.header.stamp = rospy.Time.now()     
        rslidar_back_to_base_link.header.frame_id = "base_link"
        rslidar_back_to_base_link.child_frame_id = "rslidar_back"
        rslidar_back_to_base_link.transform.translation.x = -1.07
        rslidar_back_to_base_link.transform.translation.y = -1.345
        rslidar_back_to_base_link.transform.translation.z = 0.6125
        rslidar_back_to_base_link.transform.rotation.w = 0.0
        rslidar_back_to_base_link.transform.rotation.x = 0.707
        rslidar_back_to_base_link.transform.rotation.y = -0.707
        rslidar_back_to_base_link.transform.rotation.z = 0.0

        return [
                odom_to_map,
                base_link_to_base_footprint,
                imu_link_to_base_link,
                gps_front_to_base_link,
                rslidar_front_to_base_link,
                laser_to_base_link,
                gps_back_to_base_link,
                rslidar_back_to_base_link
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