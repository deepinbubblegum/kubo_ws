#!/usr/bin/env python3
import rospy
from sensor_custom_msgs.msg import LEDState

class led_state_control:
    def __init__(self):
        # Init node
        rospy.init_node('led_state_control_node', anonymous=False)

        self.node_name = rospy.get_name()

        # Get ros params                  
        self.get_ros_params()

        # init variable
        self.init_variable()

        # Create topic
        self.pub_led_state = rospy.Publisher(
            'led_state',
            LEDState,
            queue_size=1
        )

    def update(self):
        led_msg = LEDState()
        if self.i == 0:
            self.data = 0x01
        if self.i == 1:
            self.data = 0x02
        if self.i == 2:
            self.data = 0x04
        if self.i == 3:
            self.data = 0x08
        if self.i == 4:
            self.data = 0x10
        if self.i == 5:
            self.data = 0x20
        if self.i == 6:
            self.data = 0x40
        if self.i == 7:
            self.data = 0x80

        if self.j == 0:
            led_msg.StackLED0 = self.data
            led_msg.StackLED1 = 0
            led_msg.StackLED2 = 0
            led_msg.StackLED3 = self.data
            led_msg.StackLED4 = 0
            led_msg.StackLED5 = 0
        if self.j == 1:
            led_msg.StackLED0 = 0
            led_msg.StackLED1 = self.data
            led_msg.StackLED2 = 0
            led_msg.StackLED3 = 0
            led_msg.StackLED4 = self.data
            led_msg.StackLED5 = 0
        if self.j == 2:
            led_msg.StackLED0 = 0
            led_msg.StackLED1 = 0
            led_msg.StackLED2 = self.data
            led_msg.StackLED3 = 0
            led_msg.StackLED4 = 0
            led_msg.StackLED5 = self.data

        if self.i < 7:
            self.i += 1
        else:
            self.i = 0
            self.j += 1
        
        if self.j > 2:
            self.j = 0
        self.pub_led_state.publish(led_msg)

    def get_ros_params(self):
        self.frequency = rospy.get_param(self.node_name + '/frequency', 2)

    def init_variable(self):
        self.data = 0x00
        self.j = 0
        self.i = 0

    def run(self):
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    led_state = led_state_control()
    try:
      led_state.run()
    except rospy.ROSInterruptException:
      rospy.loginfo('terminate led state node')