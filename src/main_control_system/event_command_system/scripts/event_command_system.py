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

        # create topic
        self.sub_PowerOn_topic = rospy.Subscriber(
            self.PowerOn_topic,
            Bool,
            self.callback_PowerOn_topic,
            queue_size=1)

        self.sub_PowerOff_topic = rospy.Subscriber(
            self.PowerOff_topic,
            Bool,
            self.callback_PowerOff_topic,
            queue_size=1)

        self.sub_BatteryCharging_topic = rospy.Subscriber(
            self.BatteryCharging_topic,
            Bool,
            self.callback_BatteryCharging_topic,
            queue_size=1)

        self.sub_ParkingOn_topic = rospy.Subscriber(
            self.ParkingOn_topic,
            Bool,
            self.callback_ParkingOn_topic,
            queue_size=1)

        self.sub_ParkingOff_topic = rospy.Subscriber(
            self.ParkingOff_topic,
            Bool,
            self.callback_ParkingOff_topic,
            queue_size=1)

        self.sub_UpPallet_topic = rospy.Subscriber(
            self.UpPallet_topic,
            Bool,
            self.callback_UpPallet_topic,
            queue_size=1)
            
        self.sub_DownPallet_topic = rospy.Subscriber(
            self.DownPallet_topic,
            Bool,
            self.callback_DownPallet_topic,
            queue_size=1)

        self.sub_PalletForward_topic = rospy.Subscriber(
            self.PalletForward_topic,
            Bool,
            self.callback_PalletForward_topic,
            queue_size=1)

        self.sub_PalletBackward_topic = rospy.Subscriber(
            self.PalletBackward_topic,
            Bool,
            self.callback_PalletBackward_topic,
            queue_size=1)

        self.sub_PalletCloseDoor_topic = rospy.Subscriber(
            self.PalletCloseDoor_topic,
            Bool,
            self.callback_PalletCloseDoor_topic,
            queue_size=1)

        self.sub_PalletOpenDoor_topic = rospy.Subscriber(
            self.PalletOpenDoor_topic,
            Bool,
            self.callback_PalletOpenDoor_topic,
            queue_size=1)

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

    # callback funtion
    def callback_PowerOn_topic(self, msg):
        pass

    def callback_PowerOff_topic(self, msg):
        pass

    def callback_BatteryCharging_topic(self, msg):
        pass

    def callback_ParkingOn_topic(self, msg):
        pass

    def callback_ParkingOff_topic(self, msg):
        pass

    def callback_UpPallet_topic(self, msg):
        pass

    def callback_DownPallet_topic(self, msg):
        pass

    def callback_PalletForward_topic(self, msg):
        pass

    def callback_PalletBackward_topic(self, msg):
        pass

    def callback_PalletCloseDoor_topic(self, msg):
        pass

    def callback_PalletOpenDoor_topic(self, msg):
        pass
    
    # end callback funtion

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