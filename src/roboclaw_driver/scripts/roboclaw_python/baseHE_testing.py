# This file is for testing the hall effect for 
# functionality through the ROBOCLAW using s4 and s5 pins

#!/usr/bin/python3
import time
from sys import exit
from roboclaw_3 import Roboclaw


# constants
CLAW_FORWARDROT = 60
CLAW_BACKWARDROT = 240
ELBOW_FULLROT = 3250
WRIST_FULLROT = 1000
BASE_FULLROT = 15000
FULLROT = 360

ROBOCLAW_1 = 128
ROBOCLAW_2 = 129

def main ():

    #Linux comport name -> for UART3 on the Nuada Pi4, this is ttyAMA1)
    rc = Roboclaw("/dev/ttyAMA1", 115200)
    print(f"port open is {rc.Open()}")

    # these values are defaulted
    accel = 0
    speed = 200
    deccel = 0
    mod_settings = True

    # setting s4 to default: off
    rc.SetPinFunctions(ROBOCLAW_1, 0, 0, 0)
    time.sleep(1)

    # setting s4 to setting motor home (user)
    rc.SetPinFunctions(ROBOCLAW_1, 0, 0x62, 0)
    print("s4 set to 0x62")
    time.sleep(1)

    # check/print that s4 pin was set correctly
    print(rc.ReadPinFunctions(129))
    time.sleep(1)
    print("here")

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    homed = ((rc.ReadError(ROBOCLAW_1)[1] & 0x400000) == 0x400000)

    print("here1")
    val = 0
    # rc.SpeedAccelDeccelPositionM1(ROBOCLAW_1, accel, speed, deccel, val, 1)
    
    speed = 50

    while (homed != True):
        # val -= 20
        val -= 80
        rc.SpeedAccelDeccelPositionM1(ROBOCLAW_1, accel, speed, deccel, val, 1)
        time.sleep(1)
        
        homed = ((rc.ReadError(ROBOCLAW_1)[1] & 0x400000) == 0x400000)
        # time.sleep(1)

    print("Base Homed")

    forward = input("move forward? ")
    if forward == "y" or forward == "yes":
        rc.SpeedAccelDeccelPositionM1(ROBOCLAW_1, accel, speed, deccel, 400, 1)
        time.sleep(1)


if __name__ == "__main__":
    main()

