#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import VelocityCMDStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class velocity_control_system:
    def __init__(self):
        rospy.init_node('velocity_control_system_node', anonymous=False)

        # Get ros params
        self.get_ros_params()

        # initial_variable
        self.initial_variable()

        # Create topic
        self.sub_Odometry_feedback = rospy.Subscriber(
            'odometry/wheel',
            Odometry,
            self.callback_odometry_velocity,
            queue_size=1
        )

        self.sub_ackermann_topic = rospy.Subscriber(
            'ackermann_cmd',
            AckermannDriveStamped,
            self.callback_ackermann_vel,
            queue_size=1
        )

        self.pub_velocity_control = rospy.Publisher(
            'velocity_control',
            VelocityCMDStamped,
            queue_size=1
        )

    def get_ros_params(self):
        self.Kp = rospy.get_param(self.node_name + '/Kp', 1) # P = Kp * e(t) ใช้สำหรับยกกราฟขึ้นแบบรวดเร็ว (ถ้าใส่เยอะไปจะ overshoot)
        self.Ki = rospy.get_param(self.node_name + '/Ki', 1) # I = Ki * ∫(0->t) (e)dt การรวม error ใช้เพื่อปรับเข้าจุด setpoint
        self.Kd = rospy.get_param(self.node_name + '/Kd', 1) # D = Kd * de(t) / dt ใช้ลดอัดตราเร็งของ feedback เมื่อเข้าใกล้จุด setpoint (ลด overshoot)
        self.frequency = rospy.get_param(self.node_name + '/frequency', 50)

    def PID_Controller(self, setpoint, dt):
        error = setpoint - self.pv_speed
        proportional = error
        self.integral = self.integral + error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
        brake = self.Ki * self.integral
        self.previous_error = error
        return output, brake

    def update(self):
        self.current_time = rospy.Time.now()
        if self.frist_loop:
            self.last_time = self.current_time
            # self.last_setpoint_time = self.current_setpoint_time # setpoint time use for check timeout
            self.frist_loop = False
        else:
            dt = (self.current_time - self.last_time).to_sec()
            torque, brake = self.PID_Controller(self.sp_speed, dt)
            velocity_msg = VelocityCMDStamped()
            velocity_msg.header.stamp = rospy.Time.now()
            velocity_msg.header.frame_id = 'velocity_control'
            velocity_msg.velocity.torque = torque
            velocity_msg.velocity.brake = brake
            self.last_time = self.current_time
        self.pub_velocity_control.publish(velocity_msg) # publish topic
        setpoint_time_distanc = (self.current_setpoint_time - self.last_setpoint_time).to_sec()
        if setpoint_time_distanc >= 2.5:
            self.sp_speed = 0.0

    def callback_odometry_velocity(self, odom_msg):
        self.pv_speed = odom_msg.twist.twist.linear.x
        # self.current_time = odom_msg.header.stamp

    def callback_ackermann_vel(self, ackermann_msg):
        self.sp_speed = ackermann_msg.drive.speed
        self.current_setpoint_time = rospy.Time.now()
        self.last_setpoint_time = self.current_setpoint_time

    def initial_variable(self):
        self.sp_speed = 0.0
        self.pv_speed = 0.0

        # timeout variable init
        self.last_setpoint_time = rospy.Time.now()
        self.current_setpoint_time = rospy.Time.now()
         
        # pid init variable
        self.previous_error = 0.0
        self.integral = 0.0
        
        # init time variable
        self.frist_loop = True
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    velocity_ctl_sys = velocity_control_system()
    try:
      velocity_ctl_sys.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('terminated velocity control system node')