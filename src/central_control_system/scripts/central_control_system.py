#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import SensorPLCStamped

class central_control_system:
    def __init__(self):
        # Init node
        rospy.init_node('central_control_system_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # create topic
        self.pub_plc_control = rospy.Publisher(
            'plc_control', 
            SensorPLCStamped, 
            queue_size=1)

    def get_ros_params(self):
        self.frequency = rospy.get_param(self.node_name + '/frequency', 50)
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'central_control')

    def update(self):
        msg = SensorPLCStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
    
    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    central_control = central_control_system()
    try:
      central_control.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('central control exit.')