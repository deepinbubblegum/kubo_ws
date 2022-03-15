#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import SensorEncoderStamped, SensorEncoderDistanStamped
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from math import sin, cos, tan, pi

class Odometry_ackermann:
    def __init__(self):
        # Init node
        rospy.init_node('odometry_steering_node', anonymous=False)

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
        self.ticks_min = rospy.get_param(self.node_name + '/ticks_min', -2147483648.0)
        self.ticks_max = rospy.get_param(self.node_name + '/ticks_max', 2147483648.0)
        self.ticks_meter = rospy.get_param(self.node_name + '/ticks_meter', 5542)

    def set_variable(self):
        self.steer_position = 0.0
        # initial velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # distanc ticker
        self.ticker = 0.0
        self.prev_ticker = 0.0
        self.prev_ticker_enc = 0
        self.multi_ticker = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0

        # distanc mater
        self.distanc = 0.0

        # frist loop
        self.frist_loop = True

    def callback_front_encoder(self, enc_msg):
        self.steer_position = self._encoder_wheel_steering(enc_msg.sensor.position)

    def callback_distanc_enc(self, enc_msg):
        self.ticker = self.multi_counting(enc_msg.sensor.ticker) / self.ticks_meter
        self.current_time = enc_msg.header.stamp
        self.update()

    def update(self):
        if self.frist_loop:
            self.last_time = self.current_time
            self.prev_ticker = self.ticker
            self.frist_loop = False
        else:
            self.distanc = self.ticker - self.prev_ticker
            elapsed = (self.current_time - self.last_time).to_sec()
            self.current_linear_velocity = self.distanc / elapsed # m/s
            self.current_angular_velocity = self.current_linear_velocity * tan(self.steer_position) / self.wheelbase
            # propigate odometry
            self.x += self.current_linear_velocity * cos(self.theta + (self.current_angular_velocity * elapsed) / 2)
            self.y += self.current_linear_velocity * sin(self.theta + (self.current_angular_velocity * elapsed) / 2)
            self.theta += self.current_angular_velocity * elapsed

            q = quaternion_from_euler(0, 0, self.theta)
            quaternion = Quaternion(*q)

            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = self.current_time
            transform_stamped_msg.header.frame_id = self.odom_frame_id 
            transform_stamped_msg.child_frame_id = self.base_frame_id
            transform_stamped_msg.transform.translation.x = self.x
            transform_stamped_msg.transform.translation.y = self.y
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation = quaternion

            if self.tf:
                self.odom_broadcaster.sendTransform(transform_stamped_msg)

            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion

            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.current_linear_velocity
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.current_angular_velocity
            self.pub_odom_wheel.publish(odom)
            self.last_time = self.current_time
            self.prev_ticker = self.ticker

    def create_topic_node(self):
        self.sub_steering_enc = rospy.Subscriber(
            'front_encoder',
            SensorEncoderStamped,
            self.callback_front_encoder,
            queue_size=1)

        self.sub_distanc_enc = rospy.Subscriber(
            'recv_enc_topic',
            SensorEncoderDistanStamped,
            self.callback_distanc_enc,
            queue_size=1)

        self.pub_odom_wheel = rospy.Publisher(
            'odometry/wheel', 
            Odometry, 
            queue_size=1)

        if self.tf:
            self.odom_broadcaster = TransformBroadcaster()

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

    def multi_counting(self, ticker):
        if ticker < self.ticks_min and self.prev_ticker_enc > self.ticks_max:
            self.multi_ticker += 1
        if ticker > self.ticks_max and self.prev_ticker_enc < self.ticks_min:
            self.multi_ticker -= 1
        distanc_ticker = 1.0 * (ticker + self.multi_ticker * (self.ticks_max - self.ticks_min))
        self.prev_ticker_enc = ticker
        return distanc_ticker

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    odom = Odometry_ackermann()
    try:
      odom.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminated Odometry_ackermann')