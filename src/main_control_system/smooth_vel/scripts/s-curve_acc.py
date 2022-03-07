#!/usr/bin/env python3
import os
import rospy
from math import sin, pi
from geometry_msgs.msg import Twist

class SCurve:
    def __init__(self):
        rospy.init_node('smooth_vel_node', anonymous=False)

        # get node name
        self.node_name = rospy.get_name()

        # get ros params
        self.get_ros_params()

        # init variable
        self.isLock = True
        self.cmd_speed = [0.0, 0.0]
        self.last_cmd_speed = [0.0, 0.0]
        self.speed_sp = [0.0, 0.0]
        self.speed_pv = [0.0, 0.0]
        self.speed_last_sp = [0.0, 0.0]
        self.speed_error = [0.0, 0.0]
        self.speed_error_max = 0.0

        self.sprofile_T = self.sprofile_t = self.sprofile
        
        # init
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.time_last_cmd = rospy.Time.now()

        # Create topics
        self.pub_cmd_vel = rospy.Publisher('cmd_vel_smooth', Twist, queue_size=1)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.callback_cmd_vel)

        self.timer = rospy.Timer(rospy.Duration(self.time_out), self.timeout_callback)


    def update(self):
        scurve_vel = Twist()
        self.current_time = rospy.Time.now()
        if self.sprofile_t <= self.sprofile_T:
            for i in range(2):
                if self.speed_sp[i] > self.speed_pv[i]:
                    self.speed_pv[i] = self.speed_last_sp[i] + self.sCurves_accel_decel(self.speed_error[i], self.sprofile_t)
                elif self.speed_sp[i] < self.speed_pv[i]:
                    self.speed_pv[i] = self.speed_last_sp[i] - self.sCurves_accel_decel(self.speed_error[i], self.sprofile_t)
            if self.sprofile_t >= self.sprofile_T:
                for i in range(2):
                    self.speed_pv[i] = self.speed_sp[i]
        scurve_vel.linear.x = self.speed_pv[0]
        scurve_vel.angular.z = self.speed_pv[1]
        self.sprofile_t += 2.5 
        self.pub_cmd_vel.publish(scurve_vel)

    def sCurves_accel_decel(self, V, t):
        res = V * (2 * pi * t / self.sprofile_T - sin(2 * pi * t / self.sprofile_T)) / 2 / pi
        return res

    def even_cmd_vel_set(self):
        for i in range(2):
            self.speed_last_sp[i] = self.speed_pv[i]
            self.speed_sp[i] = self.cmd_speed[i]
            self.speed_error[i] = abs(self.speed_sp[i] - self.speed_pv[i]) 
            if self.speed_error[i] > self.speed_error_max:
                self.speed_error_max = self.speed_error[i]
        self.sprofile_T = (self.speed_error_max/self.sprofile) * 100.0
        if self.sprofile_T < 100.0:
            self.sprofile_T = 100.0
        if self.sprofile_t >= self.sprofile_T:
            self.sprofile_t = 0.0

    def timeout_callback(self, time):
        if self.current_time - self.last_time >= rospy.Duration.from_sec(self.time_out):
            set_vel_timeout = Twist()
            set_vel_timeout.linear.x = 0.0
            set_vel_timeout.angular.z = 0.0
            self.callback_cmd_vel(set_vel_timeout)

    def callback_cmd_vel(self, twist):
        self.current_time = self.last_time = rospy.Time.now()
        if self.last_cmd_speed[0] != twist.linear.x or self.last_cmd_speed[1] != twist.angular.z:
            self.sprofile_t = self.sprofile_T
            self.last_cmd_speed[0] = self.cmd_speed[0] = twist.linear.x
            self.last_cmd_speed[1] = self.cmd_speed[1] = twist.angular.z
            self.even_cmd_vel_set()
        

    def get_ros_params(self):
        self.frequency = rospy.get_param(self.node_name + '/frequency', 30)
        self.time_out = rospy.get_param(self.node_name + '/time_out', 1.2)
        self.sprofile = rospy.get_param(self.node_name + '/sprofile', 1000)

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    s_curve = SCurve()
    try:
        s_curve.run()
    except rospy.ROSInterruptException:
        pass
