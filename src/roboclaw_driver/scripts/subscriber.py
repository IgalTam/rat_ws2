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
    parser.read("/home/pi/rat_ws2/src/roboclaw_driver/scripts/joint_config.ini")
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
    claw_pos = 0

    # Note: we are working under the assumption that the claw will be closed at startup/after homing
    # 0 means closed, 1 means open
    claw_status = 0   
    

    same_msg_cnt = 0
    def rads_to_enc_cnts(self, cnts_per_rev, rads, centered=True):
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
        if centered:
            return int(cnts_per_rev + (rads * (cnts_per_rev/(2*pi))))
        else:
            return int(rads * (cnts_per_rev/(2*pi)))

    def __init__(self):
        
        self.rc = Roboclaw(self.joint_comports[0], self.BAUDRATE) # comports are not gonna change for us
        self.rc.Open()
        self.center_motors()

    def center_motors(self):
        for i in range(self.num_joints-1): # dont center claw
            address = int(self.joint_addresses[i]) # default: 0x80 or 128
            channel = int(self.joint_channels[i])
            cnts_per_rev = int(self.joint_cnts_per_rev[i])
            # if i == 2: # need to set the wrist to 90 degrees
            #     # cnts_per_rev += cnts_per_rev//4
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

    def limit_joints(self, data_arr):
        """restricts incoming joint angles to preset limits
        implemented for hardware safety
        NOTE: will break the rest of the ROS operation if engaged
        due to offsetting the actual encoders from where Moveit thinks
        they are"""
        for joint_idx in range(len(data_arr)-1):
            # set to minimum if below minimum
            if data_arr[joint_idx] < self.JOINT_LIMIT_ARR[joint_idx][0]:
                data_arr[joint_idx] = self.JOINT_LIMIT_ARR[joint_idx][0]
                print(f'motor {joint_idx} limited to {data_arr[joint_idx]}')
            # set to maximum if below maximum
            elif data_arr[joint_idx] > self.JOINT_LIMIT_ARR[joint_idx][1]:
                data_arr[joint_idx] = self.JOINT_LIMIT_ARR[joint_idx][1]
                print(f'motor {joint_idx} limited to {data_arr[joint_idx]}')
        return data_arr

    def actuate_claw(self):
        """this function actuates the claw, either opens or closes the claw
        and toggles a bit to indicate whether the """
        print("Actuating Claw...")
        self.rc.SpeedAccelDeccelPositionM1(129, 0, 200, 0, 58, 1)
        time.sleep(1)
        self.rc.SetEncM1(int(self.joint_addresses[self.num_joints - 1]), 0) # reset this encoder
        self.claw_status += 1
        self.claw_status = self.claw_status % 2
        

    def callback(self, data):
        if data.position_rads[self.num_joints - 1] < 0: # check if claw needs to be actuated, hacky as urdf does not know about claw
            self.actuate_claw()
            # print("Actuating Claw...")
            # self.rc.SpeedAccelDeccelPositionM1(129, 0, 200, 0, 58, 1)
            # time.sleep(1)
            # self.rc.SetEncM1(int(self.joint_addresses[self.num_joints - 1]), 0) # reset this encoder
            # self.claw_status += 1
            # self.claw_status = self.claw_status % 2
            return
        elif data.position_rads[self.num_joints - 1] > 0: # check if claw needs to be actuated, hacky as urdf does not know about claw
            print("Rotating Claw...")
            started_open = self.claw_status
            if (started_open == 1): # check if claw is open. If so, close for rotation
                self.actuate_claw()
            radian_angle = data.position_rads[self.num_joints - 1] # radian_angle is the intended position the claw needs to rotate to
            print(f"radian angle: {radian_angle}")
            # due to the mechanism of the claw, it cannot be rotated backwards -- it 
            # can only move forward. So, if the new angle passed is less than the angle that was
            # previously passed in, the claw must rotate to home position (0 radians) and then move
            # forward from home to the passed in radian_angle.
            if (radian_angle < self.claw_pos):
                #claw isn't centered like other joints, so set centered to False to avoid unneccesarry offset
                encoder_counts = -1*self.rads_to_enc_cnts(int(self.joint_cnts_per_rev[self.num_joints - 1]), 6.28319 - self.claw_pos + radian_angle, False)
                print(f"move attempt: {self.rc.SpeedAccelDeccelPositionM1(129, 0, 200, 0, encoder_counts, 1)}")
                time.sleep(1)
                print(f"actual loc of claw: {self.rc.ReadEncM1(129)}")
                time.sleep(1)
                self.claw_pos = radian_angle
                print(f"after rotation enc counts: {encoder_counts}, claw pos: {self.claw_pos}")
            elif (radian_angle >= self.claw_pos):
                encoder_counts = -1*(self.rads_to_enc_cnts(int(self.joint_cnts_per_rev[self.num_joints - 1]), radian_angle - self.claw_pos)-int(self.joint_cnts_per_rev[self.num_joints - 1]))
                print(f"move attempt: {self.rc.SpeedAccelDeccelPositionM1(129, 0, 200, 0, encoder_counts, 1)}")
                time.sleep(1)
                print(f"actual loc of claw: {self.rc.ReadEncM1(129)}")
                time.sleep(1)
                self.claw_pos = radian_angle
                print(f"after rotation enc counts: {encoder_counts}, claw pos: {self.claw_pos}")
            self.rc.SetEncM1(int(self.joint_addresses[self.num_joints - 1]), 0) # reset this encoder
            if (started_open == 1):
                self.actuate_claw()
            return

        # # upon getting a msg, check if previous msg is the same
        # if self.float_list_cmp(self.old_data.position_rads, data.position_rads):
        #     self.same_msg_cnt += 1
        # else:
        #     print("new message")
        #     self.same_msg_cnt = 0
        #     self.old_data = data
        #     return

        # if self.same_msg_cnt > 3: # data has stabilized write it
        #     self.same_msg_cnt = 0
        #    # print("data stabilized")
            
        # else:
        #    # print("same data recieved")
        #     return
        # if self.float_list_cmp(self.writtendata.position_rads, data.position_rads):
        #     return
        print("writing")
        
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        # init message to publish for hardware interface telem_callback
        telem_msg = ratTelemetry()
        # if input("Send commands to RoboClaws? (Yes/no): ") != "Yes":
        #     rospy.loginfo(rospy.get_caller_id() + "ignoring last message")
        #     return
        for i in range(0, self.num_joints-1):
            # fetch data from config file
            address = int(self.joint_addresses[i]) # default: 0x80 or 128
            channel = int(self.joint_channels[i])
            cnts_per_rev = int(self.joint_cnts_per_rev[i])

            # fetch data from armCmd.msg
            rads = data.position_rads[i]
            speed = data.speed[i] # not used
            # accel = data.accel_deccel[i]

            if i == 1: # add elbow offset
                rads -= 0.0175 # ~1 degrees


            # Linux comport name
            #rc.Open()
            flip_direction = int(self.joint_direction[i])
            buf = 1
            accel1 = 0
            if i == 0: # base
                speed1 = 370
                speed2 = 370
            elif i == 1: # elbow
                speed1 = 115
                speed2 = 115
            else: # wrist
                speed1 = 90
                speed2 = 90
            deccel1 = 0
            cnts1 = 0
            accel2 = 0
            #speed2 = 100
            deccel2 = 0
            cnts2 = 0

            ## Doing this seperate based on the channels seems kind of dumb, but roboclaw lib
            ### uses different funcs for m1 and m2, there is a func for both m1 and m2 at the same time
            ### so that could replace this later
            print(self.rc.ReadEncM2(128))
            if channel == 1:
                # get current encoder val
                cur_enc_val = self.rc.ReadEncM1(address)[1] # ReadEnc returns (success_code, enc_val, address)
                cur_angle = ((cur_enc_val-cnts_per_rev)/cnts_per_rev) * 2 * pi
                rospy.loginfo(f'Motor {i} {cur_enc_val}')
                # populate telemetry message
                telem_msg.angle[i] = cur_angle
                # if isclose(cur_angle, rads, rel_tol=self.FLOAT_EPSILON):
                #     continue # done write an unneccesary value to motor
                # calculate encoder counts to write to motor
                cnts1 = self.rads_to_enc_cnts(cnts_per_rev, rads)
                if flip_direction:
                        #print(f'precnts1:{cnts1}')
                        cnts1 = (2*cnts_per_rev) - cnts1
                        #print(f'postcnts1:{cnts1}')
                if cnts1 == cur_enc_val:
                    rospy.loginfo(f'{cnts1} == {cur_enc_val}')
                    continue # dont need to write val if motor is already there
                self.rc.SpeedAccelDeccelPositionM1(address,accel1,speed1,deccel1,cnts1,buf)
                rospy.loginfo(f'Writing {cnts1}')

            elif channel == 2:
                # get current encoder val
                print(self.rc.ReadEncM2(address))
                cur_enc_val = self.rc.ReadEncM2(address)[1]
                cur_angle = ((cur_enc_val-cnts_per_rev)/cnts_per_rev) * 2 * pi
                rospy.loginfo(f'Motor {i} {cur_enc_val}')
                # populate telemetry message
                telem_msg.angle[i] = cur_angle
                cnts2 = self.rads_to_enc_cnts(cnts_per_rev, rads)
                if flip_direction:
                        #print(f'precnts2:{cnts2}')
                        cnts2 = (2*cnts_per_rev) - cnts2
                        #print(f'postcnts2:{cnts2}')
                # calculate encoder counts to write to motor
                
                if cnts2 == cur_enc_val:
                    rospy.loginfo(f'{cnts1} == {cur_enc_val}')
                    continue # dont need to write val if motor is already there
                rospy.loginfo(f'Writing {cnts2}')
                self.rc.SpeedAccelDeccelPositionM2(address,accel2,speed2,deccel2,cnts2,buf)
            else:
                rospy.loginfo(rospy.get_caller_id() + "Invalid motor channel: " + channel)
                return  
            time.sleep(0.5)
            
        #rospy.loginfo(f'telem_msg: {telem_msg}')
        self.telem_pub.publish(telem_msg)
        self.writtendata = data

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'telemetry' node so that multiple listeners can
        # run simultaneously.

        self.rc.SetPinFunctions(128, 0, 0, 0) # turn off homing pin for base
        self.rc.SetPinFunctions(129, 0, 0, 0) # turn off homing pin for claw

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
