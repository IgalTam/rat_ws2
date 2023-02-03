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
TEST_WRIST_ENC_DEG = 9
TEST_WRIST_FULLROT = 1000
TEST_WRIST_HOME = 250
#   TODO: Needs to be confirmed....not actually but in test bed
TEST_WRIST_FINE = 15

TEST_ELBOW_ADDR = 130
TEST_ELBOW_MOTOR = 1
TEST_ELBOW_ENC_DEG = 27
TEST_ELBOW_FULLROT = 3250
TEST_ELBOW_FINE = 35
#   TODO: Needs to be confirmed....

TEST_SPEED = 80

def rotate_wrist_till_stopM2(rc: Roboclaw, address):
    """OBJECTIVE: Will rotate a M2 on any roboclaw for the Wrist to move given ammount till 
    it reaches a physical stop"""
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    oldPos = 10000
    newPos = rc.ReadEncM2(address)[1]
#   newPos needs to be set to a value that will not break out of while loop right away

    while not (oldPos+2 >= newPos >= oldPos-2):
        print("Old: ", oldPos, " New: ", newPos, "\n")

#       Moves arm position at a slow rate towrds its desired physical stop
        rc.SpeedAccelDeccelPositionM2(address, 0, TEST_SPEED, 0, (newPos - TEST_WRIST_ENC_DEG), 1)
        time.sleep(0.5)

#       Updating position by setting the current position(newPos) to oldPos for next move 
        oldPos = newPos
        newPos = rc.ReadEncM2(address)[1]
        print("Move Complete -> newPos: ", newPos, "\n\n")


def rotate_elbow_till_stopM1(rc: Roboclaw, address):
    """OBJECTIVE: Will rotate a M1 on any roboclaw for the Elbow to move given ammount till 
    it reaches a physical stop"""
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    oldPos = 10000
    newPos = rc.ReadEncM1(address)[1]
#   newPos needs to be set to a value that will not break out of while loop right away

    while not (oldPos+2 >= newPos >= oldPos-2):
        print("Old: ", oldPos, " New: ", newPos, "\n")

#   Moves arm position at a slow rate towrds its desired physical stop
        rc.SpeedAccelDeccelPositionM1(address, 0, TEST_SPEED, 0, (newPos - TEST_ELBOW_ENC_DEG), 1)
#       TODO: Ensure Speed and amount moved is correct AND DIRECTION!!!!!  
        time.sleep(0.5)

#       Updating position by setting the current position(newPos) to oldPos for next move 
        oldPos = newPos
        newPos = rc.ReadEncM1(address)[1]
        print("Move Complete -> newPos: ", newPos, "\n\n")
    

def homing_procedure(rc: Roboclaw, rc1Address, rc2Address):
    """OBJECTIVE: General procedure to get the arm back from any position back to the home position"""
#   rc1Address - roboclaw address for base and claw
#   rc2Address - roboclaw address forthe elbow and wrist

#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    rotate_wrist_till_stopM2(rc, rc2Address)
#   moving the wrist motor till it hits the mechanical stop in the vertical position

#   TODO: Check if the Hall Effect sensor has been triggered yet
    hallEffect = 0
    if not (hallEffect):
#       CASE: The base Hall Effect Sensor is triggered...the arm should be close to the home position
        baseMotorGood = 1
#       TODO: Run Base motor till it hits the base motor(could turn this to a while loop checking if
#       Hall Effect has been triggered) 
    
    rotate_elbow_till_stopM1(rc, rc2Address)
#   Moving elbow motor to bring the arm back till it reaches the mechanical stop on the base


#   TODO: Home claw to correct position and close claw

#   rc.SpeedAccelDeccelPositionM1(rc2Address, 0, TEST_SPEED, 0, TEST_WRIST_HOME, 1)
#   code to move ARM back to the downwards position
#   Should the encoder be reset at this point?

#   Homed?




def test_wrist_homing(rc: Roboclaw, address):
    """Fuction to allow for easier testing of wrist homing"""
#   Moved most of the code from main to here in order to be able to test homing in sectionals

    currentPos = rc.ReadEncM2(TEST_WRIST_ADDR)[1]
    print("Testing Wrist Homing on Roboclaw: ",  address, " M2\n")
    print("Current Encoder count: ",  currentPos, "\n\n")
#   Getting current information on wrist
    
    if (input("Attempt to Zero? y/n: ") == "y"):
#       Asking user if they want to attempt basic homing
        rotate_wrist_till_stopM2(rc, TEST_WRIST_ADDR)
        currentPos = rc.ReadEncM2(TEST_WRIST_ADDR)[1]
        print("\nCurrent Encoder count: ",  currentPos, "\n")
        
    if(input("\n\nFinetune stop? y/n: ") == "y"):
#   Adjust the physical stop to back off the stop
        currentPos = rc.ReadEncM2(TEST_WRIST_ADDR)[1]
        rc.SpeedAccelDeccelPositionM2(TEST_WRIST_ADDR, 0, TEST_SPEED, 0, (currentPos + TEST_WRIST_FINE), 1)
#       TODO: ensure this is the correct location to go to

    print("\n\nCurrent encoder position: ",  currentPos, "\n")
    if (input("Set current Encoder position to zero? y/n: ") == "y"):
#       Asking user if they want to actually zero the position
        rc.SetEncM2(TEST_WRIST_ADDR, 0)
        print("\nCurrent encoder position: ",  currentPos)

        if (input("\n\nAttempt to move to projected home? y/n: ") == "y"):
#           Will attemt to move the arm to the home position
            rc.SpeedAccelDeccelPositionM2(TEST_WRIST_ADDR, 0, TEST_SPEED, 0, TEST_WRIST_HOME, 1)

            if (input("\n\nSet this new psoition as the zero? y/n: ") == "y"):
                rc.SetEncM2(TEST_WRIST_ADDR, 0)
                print("\nWrist Should be Homed and Zeroed\n")

    time.sleep(1)
    input("Testing System holding...Press any key to leave test\n")
#   Waiting to be ready to exit testing


def test_elbow_homing(rc: Roboclaw, address):
    """Fuction to allow for easier testing of elbow homing"""
#   Moved most of the code from main to here in order to be able to test homing in sectionals

    currentPos = rc.ReadEncM1(TEST_WRIST_ADDR)[1]
    print("Testing Elbow Homing on Roboclaw: ",  address, " M2\n")
    print("Current Encoder count: ",  currentPos, "\n\n")
#   Getting current information on elbow
    
    if (input("Attempt to Zero? y/n: ") == "y"):
#       Asking user if they want to attempt basic homing
        rotate_elbow_till_stopM1(rc, TEST_WRIST_ADDR)
        currentPos = rc.ReadEncM1(TEST_WRIST_ADDR)[1]
        print("\nCurrent Encoder count: ",  currentPos, "\n")
        
    if(input("\n\nFinetune stop? y/n: ") == "y"):
#   Adjust the physical stop to back off the stop
        currentPos = rc.ReadEncM1(TEST_WRIST_ADDR)[1]
        rc.SpeedAccelDeccelPositionM1(TEST_WRIST_ADDR, 0, TEST_SPEED, 0, (currentPos + TEST_ELBOW_FINE), 1)
#       TODO: ensure this is the correct location to go to

    print("\n\nCurrent encoder position: ",  currentPos, "\n")
    if (input("Set current Encoder position to zero? y/n: ") == "y"):
#       Asking user if they want to actually zero the position
        rc.SetEncM1(TEST_WRIST_ADDR, 0)
        print("\nCurrent encoder position: ",  currentPos)

    time.sleep(1)
    input("Testing System holding...Press any key to leave test\n")
#   Waiting to be ready to exit testing



def test_homing(rc: Roboclaw, rc1Address, rc2Address):
    """Testing bothe the wrist and elbow at same time"""
#   Expected demo

def main():
#   configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
#   generate/open port
    rc.Open()

    test_wrist_homing(rc, TEST_WRIST_ADDR)

    

    


    

if __name__ == "__main__":
    main()
