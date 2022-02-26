#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from sensor_custom_msgs.msg import EventControl

class event_command_system:
    def __init__(self):
        # Init node
        rospy.init_node('event_command_system_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

    def get_ros_params(self):
        self.PowerOn_topic = rospy.get_param(self.node_name + '/PowerOn_topic', 'power_on')
        self.PowerOff_topic = rospy.get_param(self.node_name + '/PowerOff_topic', 'power_off')
        self.BatteryCharging_topic = rospy.get_param(self.node_name + '/BatteryCharging_topic', 'battery_charging')
        self.ParkingOn_topic = rospy.get_param(self.node_name + '/ParkingOn_topic', 'parking_on')
        self.ParkingOff_topic = rospy.get_param(self.node_name + '/ParkingOff_topic', 'parking_off')
        self.UpPallet_topic = rospy.get_param(self.node_name + '/UpPallet_topic', 'up_pallet')
        self.DownPallet_topic = rospy.get_param(self.node_name + '/DownPallet_topic', 'down_pallet')
        self.PalletForward_topic = rospy.get_param(self.node_name + '/PalletForward_topic', 'pallet_forward')
        self.PalletBackward_topic = rospy.get_param(self.node_name + '/PalletBackward_topic', 'pallet_backward')
        self.PalletCloseDoor_topic = rospy.get_param(self.node_name + '/PalletCloseDoor_topic', 'pallet_close_door')
        self.PalletOpenDoor_topic = rospy.get_param(self.node_name + '/PalletOpenDoor_topic', 'pallet_open_door')

    def update(self):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    event_cmd_sys = event_command_system()
    try:
      event_cmd_sys.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminate event command system node')