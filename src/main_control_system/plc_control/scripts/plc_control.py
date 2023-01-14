#!/usr/bin/env python3
import rospy
import socket
from sensor_custom_msgs.msg import SensorPLCStamped

class PLC:
    def __init__(self):
        # Init node
        rospy.init_node('plc_control_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()
        
        # Get ros params
        self.get_ros_params()
        
        self.Dump = 0
        
        # variable package data for send to server ctl
        self.ctl_custom_package = [
            0x00, 0x00, #[0] lift not use
            0x00, 0x00, #[2] angle not use
            0x00, 0x00, #[4] front led
            0x00, 0x00, #[6] left front led
            0x00, 0x00, #[8] left rear led
            0x00, 0x00, #[10] rear led
            0x00, 0x00, #[12] right rear led
            0x00, 0x00, #[14] right front led
            0x00, 0x00, #[16] front brake
            0x00, 0x00, #[18] real brake
            0x00, 0x00, #[20] high volt enable
            0x00, 0x00, #[22] park brake
            0x00, 0x00, #[24] pallet 
            0x03, 0x04, #check sum
            0x05, 0x06,  #check sum
        ]
        
        # socket plc
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_connected = False
        
        # call connect network plc
        self.connect_network()

        # create topic plc data
        self.sub_control_plc = rospy.Subscriber('plc_control', SensorPLCStamped, self.callback_control_plc, queue_size=1)
        
    def connect_network(self):
        try:
            self.s.connect((self.plc_ip, self.plc_port))
            self.socket_connected = True
            rospy.loginfo( "connection successful")
        except:
            self.socket_connected = False
            try:
                self.s.close()
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except:
                rospy.loginfo('socket error pipeline.')
            
    def get_ros_params(self):
        self.plc_ip = rospy.get_param(self.node_name + '/plc_ip', '10.1.100.140')
        self.plc_port = rospy.get_param(self.node_name + '/plc_port', 2000)
        self.buffer_size = rospy.get_param(self.node_name + '/buffer_size', 1024)
        self.frequency = rospy.get_param(self.node_name + '/frequency', 50)

    def send(self, package):
        if self.socket_connected:
            try:
                self.s.send(bytearray(package))
            except:
                self.socket_connected = False
        else:
            print("try re-connection...")
            self.connect_network()

    def steering(self, msg_data):
        if msg_data == -1:
            self.ctl_custom_package[2] &= 0b11111100
            self.ctl_custom_package[2] |= 0b00000001
        elif msg_data == 1:
            self.ctl_custom_package[2] &= 0b11111100
            self.ctl_custom_package[2] |= 0b00000010
        else:
            self.ctl_custom_package[2] &= 0b11111100

    def callback_control_plc(self, control_msg):
        self.ctl_custom_package[0] = control_msg.PLC.UpDown
        self.steering(control_msg.PLC.WheelAngleLR)
        self.ctl_custom_package[4] = control_msg.PLC.StackLED0
        self.ctl_custom_package[6] = control_msg.PLC.StackLED1
        self.ctl_custom_package[8] = control_msg.PLC.StackLED2
        self.ctl_custom_package[10] = control_msg.PLC.StackLED3
        self.ctl_custom_package[12] = control_msg.PLC.StackLED4
        self.ctl_custom_package[14] = control_msg.PLC.StackLED5
        self.ctl_custom_package[16] = control_msg.PLC.PercentBrake
        self.ctl_custom_package[18] = control_msg.PLC.PercentBrake
        self.ctl_custom_package[20] = control_msg.PLC.Parking
        self.ctl_custom_package[22] = control_msg.PLC.Mode
        self.ctl_custom_package[24] = control_msg.PLC.Pallet
        self.Dump = control_msg.PLC.Dump
        self.Dumper_control()

    def Dumper_control(self):
        if self.Dump == 1:
            self.ctl_custom_package[2] &= 0b11110011
            self.ctl_custom_package[2] |= 0b00001000
        elif self.Dump == -1:
            self.ctl_custom_package[2] &= 0b11110011
            self.ctl_custom_package[2] |= 0b00000100
        else:
            self.ctl_custom_package[2] &= 0b11110011

    def update(self):
        self.send(self.ctl_custom_package)

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
        self.s.close()
    
if __name__ == "__main__":
    plc = PLC()
    try:
      plc.run()
    except rospy.ROSInterruptException():
        rospy.loginfo('exit plc node')