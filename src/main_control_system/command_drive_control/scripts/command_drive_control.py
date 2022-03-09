#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import VelocityCMDStamped, DriveVCUStamped, DriveMCU1Stamped

class command_drive_control:
    def __init__(self):
        rospy.init_node('command_drive_control_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()
        
        # Get ros params
        self.get_ros_params()
        
        # initial_variable
        self.initial_variable()

        # create ros topic
        self.sub_velocity_troque = rospy.Subscriber(
            'velocity_control',
            VelocityCMDStamped,
            self.callback_velocity,
            queue_size=1
        )

        self.sub_mcu_1 = rospy.Subscriber(
            'MCU1_Topic',
            DriveMCU1Stamped,
            self.callback_mcu1,
            queue_size=1
        )

        self.pub_drive_vcu = rospy.Publisher(
            'VCU',
            DriveVCUStamped,
            queue_size=1
        )

    def get_ros_params(self):
        self.inverse = rospy.get_param(self.node_name + '/inverse', False)

    def callback_velocity(self, velocity_msg):
        self.torque = velocity_msg.velocity.torque
        self.update()

    def callback_mcu1(self, mcu_msg):
        self.drive_ready = bool(mcu_msg.MCU1.Ready)

    def drive_working_mode(self, torque):
        if self.inverse:
            torque = torque * -1
            
        if torque > 0:
            working_mode = 0x92 # Forward drive
        elif torque < 0:
            working_mode = 0x8A # Reverse drive
        else:
            working_mode = 0x81 # Stop
        return working_mode

    def update(self):
        if self.drive_ready is False: # vcu send ready
            self.set_torque = 0 #N⋅m
            self.drive_working = 0x80 # 1000 0000 = Ready
            self.send_topic()
            self.second_torque = False
        else:
            if self.second_torque is False:
                self.set_torque = 0
                self.dc_limit_current = 100 #A
                self.drive_working = 0x92
                self.send_topic()
                self.second_torque = True
            else:
                self.set_torque = abs(self.torque)
                self.dc_limit_current = 100 #A
                self.drive_working = self.drive_working_mode(self.torque)
                self.send_topic()
                self.second_torque = False

    def initial_variable(self):
        # second torque 0 set
        self.second_torque = False

        self.brake = 0
        self.drive_ready = False
        self.set_torque = 0

        # vcu
        self.torque = 0
        self.dc_limit_voltage = 700 #V
        self.dc_limit_current = 0 #A
        self.drive_working = 0
        self.vcu_life = 0

    def send_topic(self):
        if self.vcu_life >= 255:
            self.vcu_life = 0
        else:
            self.vcu_life += 1
        vcu_msg = DriveVCUStamped()
        vcu_msg.header.stamp = rospy.Time.now()
        vcu_msg.header.frame_id = 'vcu'
        vcu_msg.drive.Torque = self.set_torque
        vcu_msg.drive.DCVoltageLimit = self.dc_limit_voltage
        vcu_msg.drive.DCCurrentLimit = self.dc_limit_current
        vcu_msg.drive.DriveWorkingMode = self.drive_working
        vcu_msg.drive.VCULife = self.vcu_life
        self.pub_drive_vcu.publish(vcu_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    command_drive = command_drive_control()
    try:
      command_drive.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminated command drive control node.')