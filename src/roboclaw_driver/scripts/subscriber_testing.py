#!/usr/bin/python3
import rospy
import time
from math import isclose
from roboclaw_python.roboclaw_3 import Roboclaw
from math import pi
from roboclaw_driver.msg import armCmd, ratTelemetry
from configparser import ConfigParser

class RoboclawNode:

    # config file maps address/channel to index in armCmd.msg
    # number of joints is specified in armCmd.msg AND in joint_config.ini
    parser = ConfigParser()
    parser.read("/home/pi/ros_stuff/rat_ws2/src/roboclaw_driver/scripts/joint_config.ini")
    num_joints = int(parser['JOINTS']['num_joints'])
    joint_addresses= parser['JOINTS']['addresses'].split(", ") # address of roboclaw for that joint
    joint_channels = parser['JOINTS']['channels'].split(", ")
    joint_comports = parser['JOINTS']['comports'].split(", ")
    joint_cnts_per_rev = parser['JOINTS']['cnts_per_rev'].split(", ")
    joint_direction = parser['JOINTS']['flip_direction'].split(", ")
    BAUDRATE = 115200
    FLOAT_EPSILON = 0.0001 # motor angle tolerance
    old_data = armCmd()
    old_data.position_rads = [0, 0, 0, 1.57]
    writtendata = armCmd()
    writtendata.position_rads = [0, 0, 0, 1.57]

    same_msg_cnt = 0
    def rads_to_enc_cnts(self, cnts_per_rev, rads):
        """
        Takes angle in radians and the enocder counts per revolution 
        and converts radians into encoder counts for that motor.
        NOTE: encoder cnts   <-> angle 
              cnts_per_rev   <-> 0 rads
              0              <-> -2pi rads
              2*cnts_per_rev <-> 2pi rads 
        args:
            cnts_per_rev(uint): encoder counts equivalent to 0 radians for particular motor
            rads(float): desired angle 
        """
        return int(cnts_per_rev + (rads * (cnts_per_rev/(2*pi))))

    def __init__(self):
        
        self.rc = Roboclaw(self.joint_comports[0], self.BAUDRATE) # comports are not gonna change for us
        self.rc.Open()
        self.center_motors()                                                  # flagged movement

    def center_motors(self):
        for i in range(self.num_joints):
            address = int(self.joint_addresses[i]) # default: 0x80 or 128
            channel = int(self.joint_channels[i])
            cnts_per_rev = int(self.joint_cnts_per_rev[i])
            if i == 2: # need to set the wrist to 90 degrees
                cnts_per_rev += cnts_per_rev//4
            if channel == 1:
                self.rc.SetEncM1(address,cnts_per_rev)
            if channel == 2:
                self.rc.SetEncM2(address,cnts_per_rev)

    def float_list_cmp(self, l1, l2):
        # return False if lists are different
        assert len(l1) == len(l2)

        for i in range(len(l1)):
            if not isclose(l1[i], l2[i]):
                return False
        return True

    # limit propositions for limbs:
    # base:
    # elbow: 0 rad -> pi rad
    #

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
        # init message to publish for hardware interface telem_callback
        telem_msg = ratTelemetry()
        # if input("Send commands to RoboClaws? (Yes/no): ") != "Yes":
        #     rospy.loginfo(rospy.get_caller_id() + "ignoring last message")
        #     return
        for i in range(0, self.num_joints):
            # fetch data from config file
            address = int(self.joint_addresses[i]) # default: 0x80 or 128
            channel = int(self.joint_channels[i])
            cnts_per_rev = int(self.joint_cnts_per_rev[i])

            # fetch data from armCmd.msg
            rads = data.position_rads[i]
            speed = data.speed[i]
            # accel = data.accel_deccel[i]

            # Linux comport name
            #rc.Open()
            
            flip_direction = int(self.joint_direction[i])
            buf = 1
            accel1 = 0
            if i == 0:
                speed1 = 300
                speed2 = 300
            else:
                speed1 = 150
                speed2 = 150
            deccel1 = 0
            cnts1 = 0
            accel2 = 0
            deccel2 = 0
            cnts2 = 0

## Doing this seperate based on the channels seems kind of dumb, but roboclaw lib
### uses different funcs for m1 and m2, there is a func for both m1 and m2 at the same time
### so that could replace this later

            if channel == 1:
                # get current encoder val
                cur_enc_val = self.rc.ReadEncM1(address)[1] # ReadEnc returns (success_code, enc_val, address)
                cur_angle = ((cur_enc_val-cnts_per_rev)/cnts_per_rev) * 2 * pi
                rospy.loginfo(f'Roboclaw {address} Motor {i} {cur_enc_val}')
                # populate telemetry message
                telem_msg.angle[i] = cur_angle
                # if isclose(cur_angle, rads, rel_tol=self.FLOAT_EPSILON):
                #     continue # done write an unneccesary value to motor
                # calculate encoder counts to write to motor
                cnts1 = self.rads_to_enc_cnts(cnts_per_rev, rads)
                if flip_direction:
                        print(f'precnts1:{cnts1}')
                        cnts1 = (2*cnts_per_rev) - cnts1
                        print(f'postcnts1:{cnts1}')
                if cnts1 == cur_enc_val:
                    rospy.loginfo(f'{cnts1} == {cur_enc_val}')
                    continue # dont need to write val if motor is already there
                self.rc.SpeedAccelDeccelPositionM1(address,accel1,speed1,deccel1,cnts1,buf) # flagged movement
                rospy.loginfo(f'Writing cnts1 {cnts1}')
            elif channel == 2:
                # get current encoder val
                cur_enc_val = self.rc.ReadEncM2(address)[1]
                cur_angle = ((cur_enc_val-cnts_per_rev)/cnts_per_rev) * 2 * pi
                rospy.loginfo(f'Roboclaw {address} Motor {i} {cur_enc_val}')
                # populate telemetry message
                telem_msg.angle[i] = cur_angle
                cnts2 = self.rads_to_enc_cnts(cnts_per_rev, rads)
                if flip_direction:
                        print(f'precnts2:{cnts2}')
                        cnts2 = (2*cnts_per_rev) - cnts2
                        print(f'postcnts2:{cnts2}')
                # calculate encoder counts to write to motor
                
                if cnts2 == cur_enc_val:
                    rospy.loginfo(f'{cnts1} == {cur_enc_val}')
                    continue # dont need to write val if motor is already there
                rospy.loginfo(f'Writing cnts2 {cnts2}')
                self.rc.SpeedAccelDeccelPositionM2(address,accel2,speed2,deccel2,cnts2,buf) # flagged movement
            else:
                rospy.loginfo(rospy.get_caller_id() + "Invalid motor channel: " + channel)
                return  
            time.sleep(0.5)
            
        #rospy.loginfo(f'telem_msg: {telem_msg}')
        self.telem_pub.publish(telem_msg)
        self.writtendata = data
        #self.rc.SpeedAccelDeccelPositionM1M2(address,accel1,speed1,deccel1,cnts1,
            #                                         accel2,speed2,deccel2,cnts2,buf)
        # read encoder values
        #for i in range(self.num_joints):
        #    address = int(self.joint_addresses[i]) # default: 0x80 or 128
        #    channel = int(self.joint_channels[i])
        #    if channel == 1:
        #        enc_val = rc.ReadEncM1(address)


        #rc.ForwardM1(address,32)#1/4 power forward
        #time.sleep(3)
        #rc.ForwardM1(address,0)	#1/4 power forward

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'telemetry' node so that multiple listeners can
        # run simultaneously.
        self.telem_pub = rospy.Publisher('roboclaw_telemetry', ratTelemetry, queue_size=5)
        rospy.init_node('roboclaw_node', anonymous=True)
        rospy.Subscriber('roboclaw_cmd', armCmd, self.callback, queue_size=256, buff_size=128)
        print(f'Num Joints: {self.num_joints}\n Joint Config:\n  Addresses: {self.joint_addresses}\n  Channels: {self.joint_channels}\n  Comports: {self.joint_comports}\n  Counts per Revolution: {self.joint_cnts_per_rev}\n  Flip direction? : {self.joint_direction}')
        rospy.loginfo('Listening...\n')


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    roboclaw_node = RoboclawNode()
    roboclaw_node.listener()
