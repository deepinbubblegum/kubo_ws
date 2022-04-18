#!/usr/bin/env python3
import rospy
import socket
import json
from sensor_custom_msgs.msg import SensorEncoderDistanStamped

class recive_encoder:
    def __init__(self):
        # Init node
        rospy.init_node('recive_encoder_node', anonymous=False)
        
        # Get node name
        self.node_name = rospy.get_name()

        # init variable
        self.set_init_variable()

        # Get ros params
        self.get_ros_params()

        # create topic
        self.pub_enc_msg = rospy.Publisher(
            self.recive_enc_topic,
            SensorEncoderDistanStamped,
            queue_size=1
        )
    
    def set_init_variable(self):
        UDP_IP_ADDRESS = ""
        UDP_PORT_NO = 10000
        self.serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))

    def get_ros_params(self):
        self.recive_enc_topic = rospy.get_param(self.node_name + '/recv_topic', 'recv_enc_topic')
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'recv_encoder')
        self.frequency = rospy.get_param(self.node_name + '/frequency', 100)

    def update(self):
        try:
            encoder, addr = self.serverSock.recvfrom(1024)
            res = encoder.decode('utf-8')
            json_string = json.loads(res)
            detEncode = int(json_string["enc0"])
            msg_enc = SensorEncoderDistanStamped()
            msg_enc.header.frame_id = self.frame_id
            msg_enc.header.stamp = rospy.Time.now()
            msg_enc.sensor.ticker = detEncode
            self.pub_enc_msg.publish(msg_enc)
        except:
          rospy.loginfo('someting wrong receiver distanc.')

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    recive = recive_encoder()
    try:
      recive.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminated recive encoder node.')