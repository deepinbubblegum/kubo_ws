#!/usr/bin/env python3
import rospy
import can
from sensor_custom_msgs.msg import DriveMCU1Stamped, DriveMCU2Stamped, DriveMCU3Stamped, DriveMCU4Stamped, DriveVCUStamped

class drive_control:
    def __init__(self):
        # Init node
        rospy.init_node('drive_control_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # init canbus device
        self.init_canbus()

        # init variable
        self.init_variable()

        # create topic
        self.create_topic_node()

    def create_topic_node(self):
        if self.MCU1_Topic:
            self.pub_mcu1_topic = rospy.Publisher(
                'MCU1_Topic',
                DriveMCU1Stamped,
                queue_size=1
            )

        if self.MCU2_Topic:
            self.pub_mcu2_topic = rospy.Publisher(
                'MCU2_Topic',
                DriveMCU2Stamped,
                queue_size=1
            )

        if self.MCU3_Topic:
            self.pub_mcu3_topic = rospy.Publisher(
                'MCU3_Topic',
                DriveMCU3Stamped,
                queue_size=1
            )

        if self.MCU4_Topic:
            self.pub_mcu4_topic = rospy.Publisher(
                'MCU4_Topic',
                DriveMCU4Stamped,
                queue_size=1
            )

        self.sub_vcu_topic = rospy.Subscriber(
            'VCU',
            DriveVCUStamped,
            self.callback_vcu,
            queue_size=1
        )

    def init_variable(self):
        self.VCU1_ID = 0x0C19F0A7
        self.MCU1_ID = 0x0C08A7F0
        self.MCU2_ID = 0x0C09A7F0
        self.MCU3_ID = 0x0C0AA7F0
        self.MCU4_ID = 0x0C0BA7F0

        # MCU STATUS 1 Offset / Scale
        self.MCU1_ST_TORQUE_OFFSET = -32000
        self.MCU1_ST_TORQUE_SCALE = 1  # NM/bit
        self.MCU1_ST_SPEED_OFFSET = 0
        self.MCU1_ST_SPEED_SCALE = 0.5  # RPM/bit
        self.MCU1_ST_DC_INPUT_Current_OFFSET = -10000
        self.MCU1_ST_DC_INPUT_Current_SCALE = 0.1  # A/bit

        # MCU STATUS 2 Offset / Scale
        self.MCU2_ST_TOTAL_VOLTAGE_OFFSET = -10000
        self.MCU2_ST_TOTAL_VOLTAGE_SCALE = 0.1  # V/bit
        self.MCU2_ST_MOTOR_TEMPERATURE_OFFSET = -40
        self.MCU2_ST_MOTOR_TEMPERATURE_SCALE = 1  # ํC/bit
        self.MCU2_ST_MCU_TEMPERATURE_OFFSET = -40
        self.MCU2_ST_MCU_TEMPERATURE_SCALE = 1  # ํC/bit

        # MCU STATUS 4 Offset / Scale
        self.MCU4_ST_MOTOR_PEAK_POWER_OFFSET = 0
        self.MCU4_ST_MOTOR_PEAK_POWER_SCALE = 1 #kw/bit
        self.MCU4_ST_MOTOR_PEAK_SPEED_OFFSET = 0
        self.MCU4_ST_MOTOR_PEAK_SPEED_SCALE = 100 #rpm/bit
        self.MCU4_ST_MOTOR_PEAK_TORQUE_OFFSET = 0
        self.MCU4_ST_MOTOR_PEAK_TORQUE_SCALE = 1 #NM/bit

        self.driveTorque_Offset = -32000
        self.dc_limitVoltage_Offset = -10000
        self.dc_limitCurrent_Offset = -10000

    def init_canbus(self):
        self.canbus = can.interface.Bus(
            bustype = self.bustype, 
            channel = self.channel,
            ttyBaudrate = self.ttyBaudrate
        )

    def get_ros_params(self):
        self.pub_status_topic = rospy.get_param(self.node_name + '/pub_status_topic', 'drive_status')
        self.bustype = rospy.get_param(self.node_name + '/bustype', 'robotell')
        self.channel = rospy.get_param(self.node_name + '/channel', '/dev/ttyUSB0')
        self.ttyBaudrate = rospy.get_param(self.node_name + '/ttyBaudrate', 115200)
        self.frequency = rospy.get_param(self.node_name + '/frequency', 50)
        self.MCU1_Topic = rospy.get_param(self.node_name + '/MCU1_Topic', True)
        self.MCU2_Topic = rospy.get_param(self.node_name + '/MCU2_Topic', True)
        self.MCU3_Topic = rospy.get_param(self.node_name + '/MCU3_Topic', True)
        self.MCU4_Topic = rospy.get_param(self.node_name + '/MCU4_Topic', True)

    def Status_Scale_Offset(self, Offset, Value, Scale):
        return (Value + (Offset)) * Scale

    def MCU1_Process_Status(self, rx_msg):
        RES_ST_Torque = (rx_msg.data[1] << 8) | (rx_msg.data[0])
        RES_ST_Speed = (rx_msg.data[3] << 8) | (rx_msg.data[2])
        RES_ST_DC_INPUT_Current = (rx_msg.data[5] << 8) | (rx_msg.data[4])
        RES_ST_MCU1_STATUS = rx_msg.data[6]
        RES_ST_MCU_LIFE = rx_msg.data[7]
        drive = DriveMCU1Stamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = 'MCU1'
        drive.MCU1.Torque = self.Status_Scale_Offset(
            self.MCU1_ST_TORQUE_OFFSET,
            RES_ST_Torque,
            self.MCU1_ST_TORQUE_SCALE
        )
        drive.MCU1.Speed = self.Status_Scale_Offset(
            self.MCU1_ST_SPEED_OFFSET,
            RES_ST_Speed,
            self.MCU1_ST_SPEED_SCALE
        )
        # rospy.loginfo(rx_msg)
        drive.MCU1.DCInputCurrent = self.Status_Scale_Offset(
            self.MCU1_ST_DC_INPUT_Current_OFFSET,
            RES_ST_DC_INPUT_Current,
            self.MCU1_ST_DC_INPUT_Current_SCALE
        )
        drive.MCU1.Ready = (RES_ST_MCU1_STATUS & 0b10000000) >> 7
        drive.MCU1.AllowForPowerDown = (RES_ST_MCU1_STATUS & 0b01000000) >> 6
        drive.MCU1.Mode = (RES_ST_MCU1_STATUS & 0b00100000) >> 5
        drive.MCU1.Forward = (RES_ST_MCU1_STATUS & 0b00010000) >> 4
        drive.MCU1.Reverse = (RES_ST_MCU1_STATUS & 0b00001000) >> 3
        drive.MCU1.Brake = (RES_ST_MCU1_STATUS & 0b00000100) >> 2
        drive.MCU1.Drive = (RES_ST_MCU1_STATUS & 0b00000010) >> 1
        drive.MCU1.Stop = (RES_ST_MCU1_STATUS & 0b00000001) >> 0
        drive.MCU1.MCU_LIFE = RES_ST_MCU_LIFE
        if self.MCU1_Topic:
            self.pub_mcu1_topic.publish(drive)

    def MCU2_Process_Status(self, rx_msg):
        RES_ST_MCU2_TOTAL_VOLTAGE = (rx_msg.data[1] << 8) | (rx_msg.data[0])
        RES_ST_MOTOR_TEMPERATURE = rx_msg.data[2]
        RES_ST_MCU2_TEMPERATURE = rx_msg.data[3]
        RES_ST_MCU2_STATUS = rx_msg.data[6]
        RES_ST_DRIVING_SYSTEM_ERROR_CODE = rx_msg.data[7]
        drive = DriveMCU2Stamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = 'MCU2'
        drive.MCU2.Total_Voltage = self.Status_Scale_Offset(
            self.MCU2_ST_TOTAL_VOLTAGE_OFFSET,
            RES_ST_MCU2_TOTAL_VOLTAGE,
            self.MCU2_ST_TOTAL_VOLTAGE_SCALE
        )
        drive.MCU2.MotorTemperature = self.Status_Scale_Offset(
            self.MCU2_ST_MOTOR_TEMPERATURE_OFFSET,
            RES_ST_MOTOR_TEMPERATURE,
            self.MCU2_ST_MOTOR_TEMPERATURE_SCALE
        )
        drive.MCU2.MCUTemperature = self.Status_Scale_Offset(
            self.MCU2_ST_MCU_TEMPERATURE_OFFSET,
            RES_ST_MCU2_TEMPERATURE,
            self.MCU2_ST_MCU_TEMPERATURE_SCALE
        )
        drive.MCU2.DrivingSystemFailure = (RES_ST_MCU2_STATUS >> 2)
        drive.MCU2.CloseTheAuxiliaryContactor = (RES_ST_MCU2_STATUS & 0b00000010) >> 1
        drive.MCU2.CloseTheMainContactor = (RES_ST_MCU2_STATUS & 0b00000001) >> 0
        drive.MCU2.DrivingSystemErrorCode = RES_ST_DRIVING_SYSTEM_ERROR_CODE
        if self.MCU2_Topic:
            self.pub_mcu2_topic.publish(drive)

    def MCU3_Process_Status(self, rx_msg):
        RES_ST_SW_VER_CTL_TY_First = rx_msg.data[1]
        RES_ST_SW_VER_ctl_TY_Second = rx_msg.data[2]
        RES_ST_SW_VER_MOTOR = (rx_msg.data[4] << 8) | (rx_msg.data[3])
        RES_ST_SW_VER_ADDITIONAL = int(rx_msg.data[5])
        RES_ST_SW_VER_MAIN = rx_msg.data[6]
        RES_ST_SW_VER_SUB = rx_msg.data[7]
        drive = DriveMCU3Stamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = 'MCU3'
        drive.MCU3.SOFTWARE_VERSION_CTL_TYPE = RES_ST_SW_VER_CTL_TY_First
        drive.MCU3.SOFTWARE_VERSION_CTL_VER_NO = RES_ST_SW_VER_ctl_TY_Second
        drive.MCU3.SOFTWARE_VERSION_MOTOR = RES_ST_SW_VER_MOTOR
        drive.MCU3.SOFTWARE_VERSION_ADDITIONAL = RES_ST_SW_VER_ADDITIONAL
        drive.MCU3.SOFTWARE_VERSION_MAIN_INFO = RES_ST_SW_VER_MAIN
        drive.MCU3.SOFTWARE_VERSION_SUB_INFO = RES_ST_SW_VER_SUB
        if self.MCU3_Topic:
            self.pub_mcu3_topic.publish(drive)

    def MCU4_Process_Status(self, rx_msg):
        RES_ST_MOTOR_PEAK_POWER = rx_msg.data[0]
        RES_ST_MOTOR_PEAK_SPEED = rx_msg.data[1]
        RES_ST_MOTOR_PEAK_TORQUE = (rx_msg.data[3] << 8) | (rx_msg.data[2])
        RES_ST_MOTOR_FAILURE_1 = rx_msg.data[5]
        RES_ST_MOTOR_FAILURE_2 = rx_msg.data[4]
        RES_ST_MOTOR_FAILURE_3 = rx_msg.data[6]
        RES_ST_MOTOR_FAILURE_4 = rx_msg.data[7]
        drive = DriveMCU4Stamped()
        drive.header.stamp = rospy.Time.now()
        drive.header.frame_id = 'MCU4'
        drive.MCU4.MotorPeakPower = self.Status_Scale_Offset(
            self.MCU4_ST_MOTOR_PEAK_POWER_OFFSET, 
            RES_ST_MOTOR_PEAK_POWER, 
            self.MCU4_ST_MOTOR_PEAK_POWER_SCALE
        )
        drive.MCU4.MotorPeakSpeed = self.Status_Scale_Offset(
            self.MCU4_ST_MOTOR_PEAK_SPEED_OFFSET, 
            RES_ST_MOTOR_PEAK_SPEED, 
            self.MCU4_ST_MOTOR_PEAK_SPEED_SCALE
        )
        drive.MCU4.MotorPeakTorque = self.Status_Scale_Offset(
            self.MCU4_ST_MOTOR_PEAK_TORQUE_OFFSET,
            RES_ST_MOTOR_PEAK_TORQUE, 
            self.MCU4_ST_MOTOR_PEAK_TORQUE_SCALE
        )
        drive.MCU4.MotorFailure = f"{RES_ST_MOTOR_FAILURE_1}, {RES_ST_MOTOR_FAILURE_2}, {RES_ST_MOTOR_FAILURE_3}, {RES_ST_MOTOR_FAILURE_4}"
        if self.MCU4_Topic:
            self.pub_mcu4_topic.publish(drive)

    def canbus_receive(self):
        rx_msg = self.canbus.recv(0.1)
        if rx_msg is not None:
            
            MCU_Message_ID = format(rx_msg.arbitration_id, "#X")
            
            if MCU_Message_ID == format(self.MCU1_ID, "#X") and self.MCU1_Topic:
                #rospy.loginfo("rx1===============")
                self.MCU1_Process_Status(rx_msg)
            elif MCU_Message_ID == format(self.MCU2_ID, "#X") and self.MCU2_Topic:
                #rospy.loginfo("rx2===============")
                self.MCU2_Process_Status(rx_msg)
            elif MCU_Message_ID == format(self.MCU3_ID, "#X") and self.MCU3_Topic:
                #rospy.loginfo("rx3===============")
                self.MCU3_Process_Status(rx_msg)
            elif MCU_Message_ID == format(self.MCU4_ID, "#X") and self.MCU4_Topic:
                #rospy.loginfo("rx4===============")
                self.MCU4_Process_Status(rx_msg)

    def _1Word_extract_2Byte(self, Word):
        byte_HIGH = Word >> 8 # is mean slide bit righ 8 position
        byte_LOW = Word & 255 # is mean Word & 0b11111111
        return byte_LOW, byte_HIGH
        
    def cmd_scale_offset(self, Offset, Value):
        return round((Value - (Offset)))

    def callback_vcu(self, vcu_msg):
        TorqueLow, TorqueHigh = self._1Word_extract_2Byte(
            self.cmd_scale_offset(
                self.driveTorque_Offset,
                vcu_msg.drive.Torque
            )
        )

        VoltageLimitLow, VoltageLimitHigh = self._1Word_extract_2Byte(
            self.cmd_scale_offset(
                self.dc_limitVoltage_Offset,
                vcu_msg.drive.DCVoltageLimit
            )
        )

        CurrentLimitLow, CurrentLimitHigh = self._1Word_extract_2Byte(
            self.cmd_scale_offset(
                self.dc_limitCurrent_Offset,
                vcu_msg.drive.DCCurrentLimit
            )
        )

        DriveWorkingMode = vcu_msg.drive.DriveWorkingMode
        VCULife = vcu_msg.drive.VCULife
        tx_msg = can.Message(
            arbitration_id=self.VCU1_ID,
            data=[
                TorqueLow, TorqueHigh,
                VoltageLimitLow, VoltageLimitHigh,
                CurrentLimitLow, CurrentLimitHigh,
                DriveWorkingMode, VCULife
            ],
            is_extended_id=True
        )
        #rospy.loginfo("tx==============================")
        self.canbus.send(tx_msg)

    def update(self):
        self.canbus_receive()

    def run(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

if __name__ == '__main__':
    drive = drive_control()
    try:
      drive.run()
    except rospy.ROSInterruptException():
        rospy.loginfo('exit drive node')