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
ELBOW_MOTOR = 2
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
TEST_WRIST_ENC_DEG = -6
TEST_WRIST_FULLROT = 1000
TEST_WRIST_HOME = 250
#   TODO: Needs to be confirmed....not actually but in test bed
TEST_WRIST_FINE = 10

TEST_ELBOW_ADDR = 130
TEST_ELBOW_MOTOR = 1
TEST_ELBOW_ENC_DEG = 9
TEST_ELBOW_FULLROT = 3250
TEST_ELBOW_FINE = -50
#   TODO: Needs to be confirmed....

TEST_SPEED = 40

def turn_by_encoder(rc: Roboclaw, address, motorNum, ecoderVal, motorSpd, printFlag):
#   Purpose of this fuction is smplify the roboclaw movements between M1 and M2,
#   CASE: the sleep time on this function is for short movements
#   RETURNS: the new ecoder postion the motor is at 

    if(motorNum == 1):
#       CASE: moving M1 on addressed roboclaw
        currentPos = rc.ReadEncM1(address)[1]
        rc.SpeedAccelDeccelPositionM1(address, 0, motorSpd, 0, (currentPos + ecoderVal), 1)
#       sleep to ensure the move is complete
        time.sleep(0.5)
        finalPos = rc.ReadEncM1(address)[1]
    elif(motorNum == 2):
#       CASE: moving M2 on addressed roboclaw
        currentPos = rc.ReadEncM2(address)[1]
        rc.SpeedAccelDeccelPositionM2(address, 0, motorSpd, 0, (currentPos + ecoderVal), 1)
#       sleep to ensure the move is complete
        time.sleep(0.5)
        finalPos = rc.ReadEncM2(address)[1]
    else:
#       CASE: Invalid entry 
        return -1
    
    if(printFlag == 1):
#       CASE: if the print flag is set, main for debugging
        print("Move Complete \n")
        print("Expected Distance: ", ecoderVal)
        print("\nActual Distance: ", abs(currentPos - finalPos), "\n\n")
    
    return finalPos

    




def rotate_wrist_till_stop(rc: Roboclaw, address):
    """OBJECTIVE: Will rotate a M2 on any roboclaw for the Wrist to move given ammount till 
    it reaches a physical stop"""
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    oldPos = 10000
    disMoved = 100
    newPos = rc.ReadEncM2(address)[1]
#   newPos needs to be set to a value that will not break out of while loop right away

    while not (disMoved <= 4):
#       Moves arm position at a slow rate towrds its desired physical stop       
        oldPos = newPos
        newPos = turn_by_encoder(rc, address, 2, TEST_WRIST_ENC_DEG, TEST_SPEED, 1)
#       Updating position by setting the current position(newPos) to oldPos for next move 

        disMoved = abs(oldPos - newPos)
        



def rotate_elbow_till_stop(rc: Roboclaw, address):
    """OBJECTIVE: Will rotate a M1 on any roboclaw for the Elbow to move given ammount till 
    it reaches a physical stop"""
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    oldPos = 10000
    disMoved = 100
    newPos = rc.ReadEncM2(address)[1]
#   newPos needs to be set to a value that will not break out of while loop right away

    while not (disMoved <= 6 ):
#       Moves arm position at a slow rate towrds its desired physical stop       
        oldPos = newPos
        newPos = turn_by_encoder(rc, address, 2, TEST_ELBOW_ENC_DEG, TEST_SPEED, 1)
#       TODO: Ensure Speed and amount moved is correct AND DIRECTION!!!!!  

#       Updating position by setting the current position(newPos) to oldPos for next move 

        disMoved = abs(oldPos - newPos)
    

def homing_procedure(rc: Roboclaw, rc1Address, rc2Address):
    """OBJECTIVE: General procedure to get the arm back from any position back to the home position"""
#   rc1Address - roboclaw address for base and claw
#   rc2Address - roboclaw address forthe elbow and wrist

#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    rotate_wrist_till_stop(rc, rc2Address)
#   moving the wrist motor till it hits the mechanical stop in the vertical position

#   TODO: Check if the Hall Effect sensor has been triggered yet
    hallEffect = 0
    if not (hallEffect):
#       CASE: The base Hall Effect Sensor is triggered...the arm should be close to the home position
        baseMotorGood = 1
#       TODO: Run Base motor till it hits the base motor(could turn this to a while loop checking if
#       Hall Effect has been triggered) 
    
    rotate_elbow_till_stop(rc, rc2Address)
#   Moving elbow motor to bring the arm back till it reaches the mechanical stop on the base


#   TODO: Home claw to correct position and close claw

#   rc.SpeedAccelDeccelPositionM1(rc2Address, 0, TEST_SPEED, 0, TEST_WRIST_HOME, 1)
#   code to move ARM back to the downwards position
#   Should the encoder be reset at this point?

#   Homed?




def test_wrist_homing(rc: Roboclaw, address):
    """Fuction to allow for easier testing of wrist homing"""
#   Moved most of the code from main to here in order to be able to test homing in sectionals

    currentPos = rc.ReadEncM2(address)[1]
    print("Testing Wrist Homing on Roboclaw: ",  address, " M2\n")
    print("Current Encoder count: ",  currentPos, "\n")
#   Getting current information on wrist
    
    if (input("Attempt to Zero? y/n: ") == "y"):
#       Asking user if they want to attempt basic homing
        rotate_wrist_till_stop(rc, address)
        currentPos = rc.ReadEncM2(address)[1]
        print("\nCurrent Encoder count: ",  currentPos, "\n")
        
    if(input("\n\nFinetune stop? y/n: ") == "y"):
#   Adjust the physical stop to back off the stop
        currentPos = rc.ReadEncM2(address)[1]
        rc.SpeedAccelDeccelPositionM2(address, 0, TEST_SPEED, 0, (currentPos + TEST_WRIST_FINE), 1)
#       TODO: ensure this is the correct location to go to

    print("\n\nCurrent encoder position: ",  currentPos, "\n")
    if (input("Set current Encoder position to zero? y/n: ") == "y"):
#       Asking user if they want to actually zero the position
        rc.SetEncM2(address, 0)
        print("\nCurrent encoder position: ",  currentPos)

        if (input("\n\nAttempt to move to projected home? y/n: ") == "y"):
#           Will attemt to move the arm to the home position
            rc.SpeedAccelDeccelPositionM2(address, 0, TEST_SPEED, 0, TEST_WRIST_HOME, 1)

            if (input("\n\nSet this new psoition as the zero? y/n: ") == "y"):
                rc.SetEncM2(address, 0)
                print("\nWrist Should be Homed and Zeroed\n")

    time.sleep(1)
    input("Testing System holding...Press any key to leave test\n")
#   Waiting to be ready to exit testing


def test_elbow_homing(rc: Roboclaw, address):
    """Fuction to allow for easier testing of elbow homing"""
#   Moved most of the code from main to here in order to be able to test homing in sectionals

    currentPos = rc.ReadEncM1(address)[1]
    print("Testing Elbow Homing on Roboclaw: ",  address, " M2\n")
    print("Current Encoder count: ",  currentPos, "\n\n")
#   Getting current information on elbow
    
    if (input("Attempt to Zero? y/n: ") == "y"):
#       Asking user if they want to attempt basic homing
        rotate_elbow_till_stop(rc, address)
        currentPos = rc.ReadEncM1(address)[1]
        print("\nCurrent Encoder count: ",  currentPos, "\n")
        
    if(input("\n\nFinetune stop? y/n: ") == "y"):
#   Adjust the physical stop to back off the stop
        currentPos = rc.ReadEncM1(address)[1]
        rc.SpeedAccelDeccelPositionM1(address, 0, TEST_SPEED, 0, (currentPos + TEST_ELBOW_FINE), 1)
#       TODO: ensure this is the correct location to go to

    print("\n\nCurrent encoder position: ",  currentPos, "\n")
    if (input("Set current Encoder position to zero? y/n: ") == "y"):
#       Asking user if they want to actually zero the position
        rc.SetEncM1(address, 0)
        print("\nCurrent encoder position: ",  currentPos)

    time.sleep(1)
    input("Testing System holding...Press any key to leave test\n")
#   Waiting to be ready to exit testing

def step_back(rc: Roboclaw):
    motorNum = 0
    encoderval = 0
    address = 0
    print("Enter the address of the roboclaw you want to stepback? \n")
    input(address)
    print("Enter the motor you want to stepback? \n")
    print("\n1 for M1\n")
    print("2 for M2\n")
    input(motorNum)
    print("How many Encoder turns? \n")
    input(encoderval)

    turn_by_encoder(rc, address, motorNum, encoderval, TEST_SPEED, 1)
    


def test_homing(rc: Roboclaw, rc1Address, rc2Address):
    """Testing both the wrist and elbow at same time"""
#   Expected demo
#   turn_by_encoder(rc, TEST_ELBOW_ADDR, 1, -150, TEST_SPEED, 1)
    test_wrist_homing(rc, TEST_WRIST_ADDR)
    test_elbow_homing(rc, TEST_ELBOW_ADDR)

def main():
#   configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
#   generate/open port
    rc.Open()

    step_back(rc)
    test_homing(rc, TEST_WRIST_ADDR, TEST_ELBOW_ADDR)
 


    



if __name__ == "__main__":
    main()
