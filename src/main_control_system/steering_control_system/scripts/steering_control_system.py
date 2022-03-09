#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import SteeringControlStamped, SensorEncoderStamped
from ackermann_msgs.msg import AckermannDriveStamped

class steering_control_system:
    def __init__(self):
        # Init node
        rospy.init_node('central_control_system_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()

        # set variable
        self.set_steering_position = 0.0
        self.steering_position = 0.0
        self.virtual_steering = 0.0
        self.steering_position_limit = 0.32288591162

        # Get ros params
        self.get_ros_params()

        # create topic
        self.pub_plc_control = rospy.Publisher(
            'steering_control',
            SteeringControlStamped,
            queue_size=1)

        self.sub_acker_cmd = rospy.Subscriber(
            'ackermann_cmd', 
            AckermannDriveStamped, 
            self.acker_callback, 
            queue_size=1)

        self.sub_enc_steer = rospy.Subscriber(
            'front_encoder', 
            SensorEncoderStamped, 
            self.enc_callback,
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

    def acker_callback(self, ack_msg):
        self.set_steering_position = ack_msg.drive.steering_angle

    def enc_callback(self, enc_msg):
        self.steering_position = enc_msg.sensor.position
        self.cal_steer()

    def cal_steer(self):
        msg = SteeringControlStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        self.virtual_steering = self._encoder_wheel_steering(self.steering_position)
        
        # limit steering pisition
        if self.set_steering_position < (self.steering_position_limit * -1):
            self.set_steering_position = (self.steering_position_limit * -1)
        elif self.set_steering_position > self.steering_position_limit:
            self.set_steering_position = self.steering_position_limit

        dist_posistion = self.set_steering_position - self.virtual_steering
        if dist_posistion > self.offset:
            msg.steer.SteerCMD = -1
        elif dist_posistion < (self.offset * -1):
            msg.steer.SteerCMD = 1
        else:
            msg.steer.SteerCMD = 0
        self.pub_plc_control.publish(msg)
        
    def get_ros_params(self):
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'steer_control_sys')
        self.offset = rospy.get_param(self.node_name + '/offset', 0.00872664626) #rad

    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    steer_control = steering_control_system()
    try:
      steer_control.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('exit steering_control_system node ')