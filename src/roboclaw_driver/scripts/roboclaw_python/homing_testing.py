import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
from homing_protocol import *
import time


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
        
    
    currentPos = rc.ReadEncM2(address)[1]
    rc.SpeedAccelDeccelPositionM2(address, 0, TEST_SPEED, 0, (currentPos + TEST_WRIST_FINE), 1)
#   TODO: ensure this is the correct location to go to

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
 

def test_setup(rc: Roboclaw):
    while(input("Want to change an arm starting position? y/n\n") == "y"):

        motorNum = 0
        encoderVal = 0
        address = 0
        address = int(input("Enter the address of the roboclaw you want to stepback?\n"))
        print("Enter the motor you want to stepback? \n")
        print("1 for M1")
        motorNum = int(input("2 for M2\n"))
        encoderVal = int(input("How many Encoder turns? \n"))

        print("Values: ", address, " ", motorNum, " ", encoderVal, "\n")
        turn_by_encoder(rc, address, motorNum, encoderVal, TEST_SPEED, 1)

        while(input("Want to make another move with the same setup? y/n\n") == "y"):
            encoderVal = int(input("How many Encoder turns? \n"))
            print("Values: ", address, " ", motorNum, " ", encoderVal, "\n")
            turn_by_encoder(rc, address, motorNum, encoderVal, TEST_SPEED, 1)
        
    print("SetUp complete\nMoving into normal testing\n\n")





def main():
    #   configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
#   generate/open port
    rc.Open()

    test_setup(rc)
    home_base(0, rc)

    double_run_homing(rc, WRIST_ADDR, WRIST_MOTOR, TEST_WRIST_ENC_DEG, 4)
    double_run_homing(rc, ELBOW_ADDR, ELBOW_MOTOR, TEST_ELBOW_ENC_DEG, 6)

if __name__ == "main":
    main()