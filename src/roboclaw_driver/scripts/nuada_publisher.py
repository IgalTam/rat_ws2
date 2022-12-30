#!/usr/bin/python3
# license removed for brevity
import rospy
import time
from configparser import ConfigParser
from roboclaw_driver.msg import armCmd
from math import isclose
from roboclaw_python.roboclaw_3 import Roboclaw
from math import pi
import RPi.GPIO as gpio

class RoboclawPublisher:

    BAUDRATE = 115200
    FLOAT_EPSILON = 0.0001 # motor angle tolerance
    same_msg_cnt = 0
    BASE_IDX = 0
    ELBOW_IDX = 1
    WRIST_IDX = 2
    CLAW_IDX = 3
    HALLPIN_1 = 17  # base hall effect
    HALLPIN_2 = 18  # claw hall effect


    def __init__(self):
        # get roboclaw configuration data
        parser = ConfigParser()
        parser.read("/home/pi/ros_stuff/rat_ws2/src/roboclaw_driver/scripts/joint_config.ini")
        
        self.num_joints = int(parser['JOINTS']['num_joints'])
        self.joint_addresses= parser['JOINTS']['addresses'].split(", ") # address of roboclaw for that joint
        self.joint_channels = parser['JOINTS']['channels'].split(", ")
        self.joint_comports = parser['JOINTS']['comports'].split(", ")
        self.joint_cnts_per_rev = parser['JOINTS']['cnts_per_rev'].split(", ")
        self.joint_direction = parser['JOINTS']['flip_direction'].split(", ")

        self.old_data = armCmd()
        self.old_data.position_rads = [0, 0, 0, 1.57]
        self.writtendata = armCmd()
        self.writtendata.position_rads = [0, 0, 0, 1.57]

        # initialize hall effect sensors
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)
        gpio.setup(self.HALLPIN_1, gpio.IN)
        gpio.setup(self.HALLPIN_2, gpio.IN)

        # initialize roboclaw
        self.rc = Roboclaw(self.joint_comports[0], self.BAUDRATE) # comports are not gonna change for us
        self.rc.Open()


    def rads_enc_cnts_conv(self, cnts_per_rev, rads=None, enc_counts=None):
        """
        If input is an angle in radians:
        Takes angle in radians and the enocder counts per revolution 
        and converts radians into encoder counts for that motor.
        If input is encoder counts:
        Takes encoder counts and the enocder counts per revolution 
        and converts encoder counts into radians for that motor.
        NOTE: encoder cnts   <-> angle 
              cnts_per_rev   <-> 0 rads
              0              <-> -2pi rads
              2*cnts_per_rev <-> 2pi rads 
        args:
            cnts_per_rev(uint): encoder counts equivalent to 0 radians for particular motor
            rads(float): desired angle 
        """
        if rads:
            return int(cnts_per_rev + (rads * (cnts_per_rev/(2*pi))))
        elif enc_counts:
            return int((enc_counts - cnts_per_rev) * ((2*pi)/cnts_per_rev))


    def read_halls(self):
        """uses RPi.GPIO to read hall effect sensor values"""


    def talker(self):
        cmd_pub = rospy.Publisher('roboclaw_cmd', armCmd, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        
        while not rospy.is_shutdown():
            msg = armCmd()

            # store positions in msg
            base_pos = self.rads_enc_cnts_conv(self.joint_cnts_per_rev[self.BASE_IDX], self.rc.ReadEncM2(self.joint_addresses[self.BASE_IDX]))
            elbow_pos = self.rads_enc_cnts_conv(self.joint_cnts_per_rev[self.ELBOW_IDX], self.rc.ReadEncM1(self.joint_addresses[self.ELBOW_IDX]))
            wrist_pos = self.rads_enc_cnts_conv(self.joint_cnts_per_rev[self.WRIST_IDX], self.rc.ReadEncM2(self.joint_addresses[self.WRIST_IDX]))
            claw_pos = self.rads_enc_cnts_conv(self.joint_cnts_per_rev[self.CLAW_IDX], self.rc.ReadEncM1(self.joint_addresses[self.CLAW_IDX]))
            msg.position_rads = [base_pos, elbow_pos, wrist_pos, claw_pos]
            
            # store speeds in msg
            base_spd = self.rc.ReadSpeedM2(self.joint_addresses[self.BASE_IDX])
            elbow_spd = self.rc.ReadSpeedM1(self.joint_addresses[self.ELBOW_IDX])
            wrist_spd = self.rc.ReadSpeedM2(self.joint_addresses[self.WRIST_IDX])
            claw_spd = self.rc.ReadEncM1(self.joint_addresses[self.CLAW_IDX])
            msg.speed = [base_spd, elbow_spd, wrist_spd, claw_spd]

            # store accel/deccel in msg
            msg.accel_deccel = [0, 0, 0, 0]

            # store hall effect values in msg
            msg.hall_vals[0], msg.hall_vals[1] = gpio.input(self.HALLPIN_1), gpio.input(self.HALLPIN_2)
            
            rospy.loginfo(msg)
            cmd_pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    roboclaw_node = RoboclawPublisher()
    try:
        roboclaw_node.talker()
    except rospy.ROSInterruptException:
        pass
