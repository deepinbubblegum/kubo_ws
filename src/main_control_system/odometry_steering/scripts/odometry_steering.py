#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import SensorEncoderStamped
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import sin, cos, pi

class Odometry_ackermann:
    def __init__(self):
        # Init node
        rospy.init_node('drive_control_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        self.set_variable()

        # create topic
        self.create_topic_node()

    def get_ros_params(self):
        self.base_frame_id = rospy.get_param(self.node_name + '/base_frame_id', 'base_link')
        self.odom_frame_id = rospy.get_param(self.node_name + '/odom_frame_id', 'odom')
        self.tf = rospy.get_param(self.node_name + '/tf', 'True')
        self.wheelbase = rospy.get_param(self.node_name + '/wheelbase', 4.19)
        self.ticks_min = rospy.get_param(self.node_name + '/ticks_min', -2147483648)
        self.ticks_max = rospy.get_param(self.node_name + '/ticks_max', 2147483648)

    def set_variable(self):
        self.steer_position = 0.0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def callback_front_encoder(self, enc_msg):
        self.steer_position = enc_msg.sensor.position

    def update(self):
        self.virtual_steering = self._encoder_wheel_steering(self.steering_position)

    def create_topic_node(self):
        self.sub_steering_enc = rospy.Subscriber(
            'front_encoder',
            SensorEncoderStamped,
            self.callback_front_encoder,
            queue_size=1)

    def _encoder_wheel_steering(self, rad_enc):
        if rad_enc > 0.0:
            front_angle_wheel_l = rad_enc 
            front_angle_wheel_r = rad_enc / 1.15
        elif rad_enc < 0.0:
            front_angle_wheel_l = rad_enc / 1.15
            front_angle_wheel_r = rad_enc
        else:
            front_angle_wheel_l = 0.0
            front_angle_wheel_r = 0.0
        return (front_angle_wheel_l + front_angle_wheel_r) / 2

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    odom = Odometry_ackermann()
    try:
      odom.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminated Odometry_ackermann')