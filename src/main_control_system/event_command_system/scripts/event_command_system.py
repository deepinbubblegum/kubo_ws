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
        self.create_ros_topic()

    def init_variable(self):
        self.PowerOn = False
        self.PowerOff = False
        self.BatteryCharging = False
        self.ParkingOn = True
        self.ParkingOff = False
        self.UpPallet = False
        self.DownPallet = False
        self.PalletForward = False
        self.PalletBackward = False
        self.PalletCloseDoor = False
        self.PalletOpenDoor = False
        self.Brake = False

    def create_ros_topic(self):
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

        self.sub_brake = rospy.Subscriber(
            self.Brake_topic,
            Bool,
            self.callback_brake_topic,
            queue_size=1
        )

        # create topic publisher
        self.pub_event_cmd = rospy.Publisher(
            self.pub_event_cmd_topic,
            EventControl,
            queue_size=1)

    def get_ros_params(self):
        self.pub_event_cmd_topic = rospy.get_param(self.node_name + '/event_cmd_pub', 'event_cmd')
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
        self.Brake_topic = rospy.get_param(self.node_name + '/Brake_topic', 'brake_topic')

    # callback funtion
    def callback_PowerOn_topic(self, msg):
        self.PowerOn = msg.data
        self.update()

    def callback_PowerOff_topic(self, msg):
        self.PowerOff = msg.data
        self.update()

    def callback_BatteryCharging_topic(self, msg):
        self.BatteryCharging = msg.data
        self.update()

    def callback_ParkingOn_topic(self, msg):
        self.ParkingOn = msg.data
        self.update()

    def callback_ParkingOff_topic(self, msg):
        self.ParkingOff  = msg.data
        self.update()

    def callback_UpPallet_topic(self, msg):
        self.UpPallet = msg.data
        self.update()

    def callback_DownPallet_topic(self, msg):
        self.DownPallet = msg.data
        self.update()

    def callback_PalletForward_topic(self, msg):
        self.PalletForward = msg.data
        self.update()

    def callback_PalletBackward_topic(self, msg):
        self.PalletBackward = msg.data
        self.update()

    def callback_PalletCloseDoor_topic(self, msg):
        self.PalletCloseDoor = msg.data
        self.update()

    def callback_PalletOpenDoor_topic(self, msg):
        self.PalletOpenDoor = msg.data
        self.update()

    def callback_brake_topic(self, msg):
        self.Brake = msg.data
        self.update()

    # end callback funtion
    def mode_event(self):
        mode = 0x00
        if self.PowerOff:
            self.PowerOn = False
            self.BatteryCharging = False
            mode = 0x00 # (power off)
        elif self.PowerOn:
            self.PowerOff = False
            self.BatteryCharging = False
            mode = 0x01  # (power on)
        elif self.BatteryCharging:
            self.PowerOff = False
            self.PowerOn = False
            mode = 0x02 # (BatteryCharging)
        return mode

    def parking_event(self):
        parking = 0x01
        if self.ParkingOn == True and self.ParkingOff == False:
            parking = 0x00 # (ParkBrakeOn)
        else:
            parking = 0x01 # (ParkBrakeOff)
        return parking

    def updown_event(self):
        updown = 0x00
        if self.UpPallet == True and self.DownPallet == False:
            updown = 0x1E # (Up)
        elif self.UpPallet == False and self.DownPallet == True:
            updown = 0x01 # (Down)
        else:
            updown = 0x00 # (Stop)
        return updown

    def pallet_event(self):
        pallet = 0x00
        if self.PalletForward == True and self.PalletBackward == False and self.PalletCloseDoor == False and self.PalletOpenDoor == False:
            pallet = 0x01 # (PalletForward)
        elif self.PalletForward == False and self.PalletBackward == True and self.PalletCloseDoor == False and self.PalletOpenDoor == False:
            pallet = 0x02 # (PalletBackward)
        elif self.PalletForward == False and self.PalletBackward == False and self.PalletCloseDoor == True and self.PalletOpenDoor == False:
            pallet = 0x03 # (CloseDoor)
        elif self.PalletForward == False and self.PalletBackward == False and self.PalletCloseDoor == False and self.PalletOpenDoor == True:
            pallet = 0x04 # (CloseDoor)
        return pallet

    def brake_event(self):
        Brake = 0
        if self.Brake:
            Brake = 100
        return Brake

    def update(self):
        msg = EventControl()
        # event process funtions
        msg.Mode = self.mode_event() # 0x00(power off), 0x01(power on), 0x02(BatteryCharging)
        msg.Parking = self.parking_event() # 0x00(ParkBrakeOn), 0x01(ParkBrakeOff)
        msg.UpDown = self.updown_event() # 0x1E(Up), 0x01(Down) 0x00(Stop)
        msg.Pallet = self.pallet_event()# 0x00(Stop), 0x01(PalletForward), 
                                        # 0x02(PalletBackward), 0x03(CloseDoor), 
                                        # 0x04(OpenDoor)
        msg.PercentBrake = self.brake_event()
        self.pub_event_cmd.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    event_cmd_sys = event_command_system()
    try:
      event_cmd_sys.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminate event command system node')