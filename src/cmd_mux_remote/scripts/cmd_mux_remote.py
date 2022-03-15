#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class cmd_mux_remote:
    def __init__(self):
        # Init node
        rospy.init_node('cmd_mux_remote_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # Create init variable
        self.linear_x = 0.0
        self.angular_z = 0.0

        # create topic 
        self.sub_twist_cmd_angular = rospy.Subscriber(
            'tab_vel_angular',
            Twist,
            self.twist_cmd_angular,
            queue_size=1
        )

        self.sub_twist_cmd_linear = rospy.Subscriber(
            'tab_vel_linear',
            Twist,
            self.twist_cmd_linear,
            queue_size=1
        )

        self.pub_twist_cmd = rospy.Publisher(
            'tab_vel',
            Twist,
            queue_size=1
        )

    def get_ros_params(self):
        self.frequency = rospy.get_param(self.node_name + '/frequency', 25)

    def twist_cmd_angular(self, twist_cmd):
        self.angular_z = twist_cmd.angular.z
        self.update()

    def twist_cmd_linear(self, twist_cmd):
        self.linear_x = twist_cmd.linear.x
        self.update()

    def update(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_x
        twist_msg.angular.z = self.angular_z
        self.pub_twist_cmd.publish(twist_msg)

    def run(self):
        rospy.spin()
        # rate = rospy.Rate(self.frequency)
        # while not rospy.is_shutdown():
        #     self.update()
        #     rate.sleep()

if __name__ == '__main__':
    cmd_remote = cmd_mux_remote()
    try:
      cmd_remote.run()
    except:
      rospy.loginfo('Terminated ros cmd mux remote.')
