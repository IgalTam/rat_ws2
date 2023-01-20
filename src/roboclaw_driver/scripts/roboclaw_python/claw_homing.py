#!/usr/bin/python3
import time
from sys import exit
from roboclaw_3 import Roboclaw

# This file is the final claw homing that will be 
# ready to integrate into with the ARM HOMING code


# constants
CLAW_FORWARDROT = 60
CLAW_BACKWARDROT = 240
ELBOW_FULLROT = 3250
WRIST_FULLROT = 1000
BASE_FULLROT = 15000
FULLROT = 360

ROBOCLAW_1 = 128
ROBOCLAW_2 = 129

# This function carries out the homing procedure for the claw. The claw 
# will continue to rotate at a constant speed until the s4 pin is read high 
# i.e. the Hall-effect sensor is turned on.
def home_claw(homed):

    while(homed != True):
        # check pins
        err = rc.ReadError(ROBOCLAW_2)
        print(err)
        print(n)
        n += 1
        # if s4 pin high, exit loop
        if ((err[1] & 0x400000) == 0x400000):
            break
        time.sleep(1)

    return 0

if __name__ == "__main__":
    #Linux comport name -> for UART3 on the Nuada Pi4, this is ttyAMA1)
    rc = Roboclaw("/dev/ttyAMA1", 115200)
    print(f"port open is {rc.Open()}")

    # these values are defaulted
    accel = 0
    speed = 200
    deccel = 0
    mod_settings = True
    
    # To ensure s4 settings are correct, they are manually 
    # configured in code below
    n=0

    # setting s4 to default: off
    rc.SetPinFunctions(ROBOCLAW_2, 0, 0, 0)
    time.sleep(1)

    # setting s4 to setting motor home (user)
    rc.SetPinFunctions(ROBOCLAW_2, 0, 0x62, 0)
    print("s4 set to 0x62")
    time.sleep(2)

    # check/print that s4 pin was set correctly
    print(rc.ReadPinFunctions(ROBOCLAW_2))
    time.sleep(2)

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    homed = ((rc.ReadError(ROBOCLAW_2)[1] & 0x400000) == 0x400000)

    homing_length = -460
    rc.SpeedAccelDeccelPositionM1(ROBOCLAW_2,0,80,0,homing_length,1)
    time.sleep(2)

    # CLAW HOMING
    # Run the function to home the claw
    claw_homed = home_claw(homed)
    time.sleep(1)

    # turn off homing by setting s4 to default: off
    rc.SetPinFunctions(ROBOCLAW_2, 0, 0, 0)
    time.sleep(1)

    # affirm homing turned off
    print(rc.ReadPinFunctions(ROBOCLAW_2))
    time.sleep(1)

    # read position after homing completed
    enc_val = int(rc.ReadEncM1(ROBOCLAW_2)[1])
    print(f'Encoder value after homing = {enc_val}')
    time.sleep(1)

    # CLAW CALIBRATION 
    # Since the claw will be slighly off from the "home position"
    # We determine the correct "0/home" position for the claw
    rc.SpeedAccelDeccelPositionM1(ROBOCLAW_2,0,100,0,-230-enc_val,1)
    time.sleep(1)

    print("Claw has been homed")
    

        





