import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
import time

"""non-ROS homing protocol implementation"""

# macros
BASE_HALL = 17
BASE_ADDR = 128
BASE_MOTOR = 2
BASE_FULLROT = 15000

ELBOW_ADDR = 128
ELBOW_MOTOR = 1
ELBOW_FULLROT = 3250

WRIST_ADDR = 129
WRIST_MOTOR = 2
WRIST_FULLROT = 1000
 
CLAW_ADDR = 129
CLAW_MOTOR = 1
CLAW_FORWARDROT = 60
CLAW_BACKWARDROT = -230

FULLROT = 360

TEST_WRIST_ADDR = 130
TEST_WRIST_MOTOR = 2
TEST_WRIST_ENC_DEG = 3
TEST_WRIST_FULLROT = 1000

TEST_ELBOW_ADDR = 130
TEST_ELBOW_MOTOR = 1
TEST_ELBOW_FULLROT = 3250

TEST_SPEED = 25

def rotate_to_deg(rc: Roboclaw, motor, address, deg):
    # rotates the motor until it reaches deg or the encoder
    # value stops changing
    if address == 128 and motor == 2:
        """moving base motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * BASE_FULLROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM2(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM2(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM2(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2
    elif address == 128 and motor == 1:
        """moving elbow motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * ELBOW_FULLROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM1(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM1(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM1(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2
    elif address == 129 and motor == 2:
        """moving wrist motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * WRIST_FULLROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM2(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM2(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM2(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2
    elif address == 129 and motor == 1:
        """moving claw motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * CLAW_BACKWARDROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM1(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM1(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM1(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2


def rotate_till_stopM2(rc: Roboclaw, address):
    # OBJECTIVE: Will rotate a M2 on any roboclaw a given ammount till it reaches a physical stop
    # This is used for nonROS homing, running under the assumption that the motor does not know
    # its position
    oldPos = 10000
    newPos = rc.ReadEncM2(address)[1]
    #newPos needs to be set to a value that will not break out of while loop right away

    if(newPos != 0):
        # CASE: if while homing the first movement should start a 0...
        print("Should never be here...\n")
    while(oldPos != newPos):
        # Waiting for the ecoder new and old value to be the same after a movement 
        oldPos = newPos
        #updating to check with new movement
        rc.SpeedAccelDeccelPositionM2(address, 0, TEST_SPEED, 0, (newPos - TEST_WRIST_ENC_DEG), 1)
        time.sleep(1)
        newPos = rc.ReadEncM2(address)[1]
    



def adjust_past_base(rc):
    """concluding sequence to home the arm"""
    # move elbow motor to -180 degrees
    rotate_to_deg(rc, ELBOW_MOTOR, ELBOW_ADDR, -180)
    # rotate claw to 0 position
    rotate_to_deg(rc, CLAW_MOTOR, CLAW_ADDR, 0)
    # close claw [pending hardware]
    # ----------code goes here---------------
    # move wrist motor to 90 degrees 
    rotate_to_deg(rc, WRIST_MOTOR, WRIST_ADDR, 90)
    

    # home is achieved

def main():
    # configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
    # generate/open port
    rc.Open()

    tempVal = rc.ReadEncM2(TEST_WRIST_ADDR)[1]
    print("Encoder M2 Before Any Movement: ",  tempVal, "\n")

    # Single motor(WRIST)Testing to a hard stop
    rotate_till_stopM2(rc, TEST_ELBOW_ADDR)
    tempVal = rc.ReadEncM2(TEST_WRIST_ADDR)[1]
    print("Encoder M2 before Zero: ",  tempVal, "\n")

    rc.SetEncM2(TEST_WRIST_ADDR, 0)
    rc.SpeedAccelDeccelPositionM2(TEST_WRIST_ADDR, 0, TEST_SPEED, 0, 30, 1)

    tempVal = rc.ReadEncM2(TEST_WRIST_ADDR)[1]
    print("Encoder M2 After Roll Back: ", tempVal, "\n")


    

if __name__ == "__main__":
    main()
