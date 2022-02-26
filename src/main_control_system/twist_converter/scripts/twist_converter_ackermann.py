#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class TwistToAckermann:
    def __init__(self):
        # Init node
        rospy.init_node('twist_converter_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        self.sub_twist_cmd = rospy.Subscriber(self.twist_cmd_topic, Twist, self.twist_callback, queue_size=1)
        self.pub_acker_cmd = rospy.Publisher(self.ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)

    def tiwst_convert_to_steering(self, v, omega):
        if omega == 0 or v == 0:
            return 0
        radius = v / omega
        return math.atan(self.wheelbase / radius)

    def twist_callback(self, twist_msg):
        # reference https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/scripts/cmd_vel_to_ackermann_drive.py
        v = twist_msg.linear.x
        if self.cmd_angle_instead_rotvel:
            steering = twist_msg.angular.z
        else:
            steering = self.tiwst_convert_to_steering(v, twist_msg.angular.z)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = v
        self.pub_acker_cmd.publish(msg)

    def get_ros_params(self):
        self.twist_cmd_topic = rospy.get_param(self.node_name + '/twist_cmd_topic', 'cmd_vel')
        self.ackermann_cmd_topic = rospy.get_param(self.node_name + '/ackermann_cmd_topic', 'ackermann_cmd')
        self.wheelbase = rospy.get_param(self.node_name + '/wheelbase', 1.0)
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'odom')
        self.cmd_angle_instead_rotvel = rospy.get_param('/move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel', False)
    
    def run():
        rospy.spin()

if __name__ == '__main__':
    twist_to_ack = TwistToAckermann()
    try:
      twist_to_ack.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('TwistToAckermann_node exit.')
