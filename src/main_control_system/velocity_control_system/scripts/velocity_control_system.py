#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import DriveVCUStamped

class velocity_control_system:
    def __init__(self):
        rospy.init_node('velocity_control_system_node', anonymous=False)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    velocity_ctl_sys = velocity_control_system()
    try:
      velocity_ctl_sys.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('terminated velocity control system node')