#!/usr/bin/env python3
import rospy
import socket
from sensor_custom_msgs.msg import SensorPLC, SensorPLCStamped

class PLC:
    def __init__(self):
        # Init node
        rospy.init_node('plc_control_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()
        
        # Get ros params
        self.get_ros_params()
        
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
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'plc')
        self.frequency = rospy.get_param(self.node_name + '/frequency', 50)

    def update(self):
        if self.socket_connected:
            try:
                self.s.send(bytearray(self.ctl_custom_package))
            except:
                self.socket_connected = False
                print("try re-connection...")
                self.connect_network()
        else:
            print("try re-connection...")
            time.sleep(0.5)
            self.connect_network()

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