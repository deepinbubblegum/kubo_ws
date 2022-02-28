#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import SteeringControlStamped, SensorPLCStamped, EventControl

class central_control_system:
    def __init__(self):
        # Init node
        rospy.init_node('central_control_system_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # initial_variable
        self.initial_variable()

        # create topic
        self.sub_steering_control = rospy.Subscriber(
            'steering_control',
            SteeringControlStamped,
            self.steering_control_callback,
            queue_size=1
        )

        self.sub_event_cmd =  rospy.Subscriber(
            'event_cmd',
            EventControl,
            self.event_cmd_callback,
            queue_size=1
        )

        self.pub_plc_control = rospy.Publisher(
            'plc_control', 
            SensorPLCStamped, 
            queue_size=1
        )

    # init variable 
    def initial_variable(self):
        self.steer_control = 0  # -1, 0 , 1 (left, stop, right)
        self.mode = 0x00 # 0x00(power off), 0x01(power on), 0x02(BatteryCharging)
        self.Parking = 0x00 # 0x00(ParkBrakeOn), 0x01(ParkBrakeOff)
        self.UpDown = 0x00 # 0x1E(Up), 0x01(Down) 0x00(Stop)
        self.Pallet = 0x00 # 0x00(Stop), 0x01(PalletForward), 
                            # 0x02(PalletBackward), 0x03(CloseDoor), 
                            # 0x04(OpenDoor)

    # callback funtion
    def steering_control_callback(self, steer_control_msg):
        self.steer_control = steer_control_msg.steer.SteerCMD
        self.update()

    def event_cmd_callback(self, event_cmd_msg):
        self.mode = event_cmd_msg.Mode
        self.Parking = event_cmd_msg.Parking
        self.UpDown = event_cmd_msg.UpDown
        self.Pallet = event_cmd_msg.Pallet
        self.update()

    # end callback funtion

    # get ros params
    def get_ros_params(self):
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'central_control')

    # update rate pub to plc node
    def update(self):
        msg = SensorPLCStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.PLC.WheelAngleLR = self.steer_control
    
    # fun main
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    central_control = central_control_system()
    try:
      central_control.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('central control exit.')