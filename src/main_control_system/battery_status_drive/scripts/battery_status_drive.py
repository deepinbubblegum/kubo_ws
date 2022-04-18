#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState
from sensor_custom_msgs.msg import DriveMCU2Stamped

class battery_status_drive:
    def __init__(self):
        # Init node
        rospy.init_node('battery_status_drive_node', anonymous=False)

        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()
        
        # init variable
        self.init_variable()
        
        # create topic
        self.sub_mcu2_status = rospy.Subscriber(
            'MCU2_Topic', 
            DriveMCU2Stamped,
            self.mcu2_update,
            queue_size=1
        )
        
        self.pub_battery_state = rospy.Publisher(
            self.pub_status_topic, 
            BatteryState,
            queue_size=1
        )
        
    def get_ros_params(self):
        self.pub_status_topic = rospy.get_param(self.node_name + '/pub_status_topic', 'battery_status')
        self.voltage_min = rospy.get_param(self.node_name + '/voltage_min', 512.0)
        self.voltage_max = rospy.get_param(self.node_name + '/voltage_max', 540.0)
    
    def mcu2_update(self, mcu2_msg):
        battery_state = BatteryState()
        self.total_voltage = mcu2_msg.MCU2.Total_Voltage
        # limit total voltage
        
        if self.total_voltage < self.voltage_min:
            self.total_voltage = self.voltage_min
        elif self.total_voltage > self.voltage_max:
            self.total_voltage = self.voltage_max
        self.current_voltage = self.total_voltage - self.voltage_min
        self.current_voltage_max = self.voltage_max - self.voltage_min
        self.battery_percent = round((self.cal_percent(self.current_voltage, self.current_voltage_max)),2)

        battery_state.header.stamp = rospy.Time.now()
        battery_state.header.frame_id = 'battery_state'
        battery_state.voltage = self.total_voltage
        battery_state.percentage = self.battery_percent
        battery_state.present = True
        self.pub_battery_state.publish(battery_state)
        
    def cal_percent(self, current_voltage, current_voltage_max):
        if current_voltage > 0:
            percent = (current_voltage / current_voltage_max) * 100
        else:
            percent = 0.0
        return percent
        
    def init_variable(self):
        self.total_voltage = 0.0
        self.current_voltage = 0.0
        self.current_voltage_max = 0.0
        self.current_percent_voltage = 0.0

        
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    battery_state_node = battery_status_drive()
    try:
        battery_state_node.run()
    except rospy.ROSInterruptException():
        rospy.loginfo('exit battery_state drive node.')