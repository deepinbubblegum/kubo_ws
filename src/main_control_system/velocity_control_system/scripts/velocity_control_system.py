#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import VelocityCMDStamped, DriveMCU1Stamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class velocity_control_system:
    def __init__(self):
        rospy.init_node('velocity_control_system_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()
        
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

        self.sub_mcu_1 = rospy.Subscriber(
            'MCU1_Topic',
            DriveMCU1Stamped,
            self.callback_mcu_1,
            queue_size=1
        )

        self.pub_velocity_control = rospy.Publisher(
            'velocity_control',
            VelocityCMDStamped,
            queue_size=1
        )

    def get_ros_params(self):
        self.frequency = rospy.get_param(self.node_name + '/frequency', 30.0)
        self.torque_limit = rospy.get_param(self.node_name + '/torque_limit', 1000.0)
        self.Kp = rospy.get_param(self.node_name + '/Kp', 1.0) # P = Kp * e(t) ใช้สำหรับยกกราฟขึ้นแบบรวดเร็ว (ถ้าใส่เยอะไปจะ overshoot)
        self.Ki = rospy.get_param(self.node_name + '/Ki', 1.0) # I = Ki * ∫(0->t) (e)dt การรวม error ใช้เพื่อปรับเข้าจุด setpoint
        self.Kd = rospy.get_param(self.node_name + '/Kd', 1.0) # D = Kd * de(t) / dt ใช้ลดอัดตราเร็งของ feedback เมื่อเข้าใกล้จุด setpoint (ลด overshoot)

    def PID_Controller(self, setpoint, dt):
        error = setpoint - self.pv_speed
        proportional = error
        self.integral = self.integral + error * dt
        derivative = ((error - self.previous_error) / dt ) if dt != 0 else 0
        output = self.Kp * proportional + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        if output > self.torque_limit: #limit torque pid
            output = self.torque_limit
        if output < (self.torque_limit * -1):
            output = (self.torque_limit * -1)
        return output

    def output_filter(self, sp_speed, output, pv_speed):
        if sp_speed > 0:
            if output > 0:
                throttle = output
                self.auto_brake = 0
            else:
                throttle = 0
                self.auto_brake += 4
        elif sp_speed < 0:
            if output < 0:
                throttle = output
                self.auto_brake = 0
            else:
                throttle = 0
                self.auto_brake += 4
        else:
            throttle = 0
            if pv_speed != 0:
                self.auto_brake += 10
        if self.auto_brake < 0:
            self.auto_brake = 0
        
        return throttle, self.auto_brake

    def update(self):
        velocity_msg = VelocityCMDStamped()
        velocity_msg.header.frame_id = 'velocity_control'
        velocity_msg.header.stamp = rospy.Time.now()
        self.current_time = rospy.Time.now()
        
        gear_ready = False
        
        if self.frist_loop:
            self.last_time = self.current_time
            self.frist_loop = False
        else:
            
            if self.sp_speed > 0 :
                if self.last_sp_speed <= 0:
                    if self.pv_speed == 0:
                        self.stop_sleep += 1
                    else:
                        self.stop_sleep = 0
                    if self.stop_sleep >= 5:
                        gear_ready = True
                    else:
                        gear_ready = False
                else:
                    gear_ready = True
            
            if self.sp_speed < 0 :
                if self.last_sp_speed >= 0:
                    if self.pv_speed == 0:
                        self.stop_sleep += 1
                    else:
                        self.stop_sleep = 0
                    if self.stop_sleep >= 5:
                        gear_ready = True
                    else:
                        gear_ready = False
                else:
                    gear_ready = True  
            
            if self.sp_speed == 0 :
                gear_ready = True
                self.previous_error = 0.0
                self.integral = 0.0
                   
            if gear_ready == True:
                self.last_sp_speed = self.sp_speed
                
                
            if self.drive_ready_state is False or gear_ready is False: # mcu 1 topic is not ready
                self.sp_speed = 0.0
                self.previous_error = 0.0
                self.integral = 0.0
            dt = (self.current_time - self.last_time).to_sec()
            output = self.PID_Controller(self.sp_speed, dt)
            torque, brake = self.output_filter(self.sp_speed, output, self.pv_speed)

            velocity_msg.velocity.torque = torque
            percent_brake = (brake/ self.torque_limit) * 100
            if percent_brake > 0:
                percent_brake + 30
            if percent_brake > 100.0:
                percent_brake = 100.0
            velocity_msg.velocity.brake = percent_brake

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

    def callback_mcu_1(self, mcu_msg):
        self.drive_ready_state = bool(mcu_msg.MCU1.Ready)

    def initial_variable(self):
        self.sp_speed = 0.0
        self.pv_speed = 0.0

        self.auto_brake = 0
        self.flag_forward = 0
        self.flag_reward = 0
        
        self.stop_sleep = self.last_sp_speed = self.sp_speed = 0

        # drive mcu status
        self.drive_ready_state = False

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