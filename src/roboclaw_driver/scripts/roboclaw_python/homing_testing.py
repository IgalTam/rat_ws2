# AUTHOR: Sebastian Rosso (NUADA Team)
# DATE: 2/20/2023
#   PURPOSE: This program is designed to be able to safely test the Homing
#            Protocol for reliablity without making modifications to the
#            homing_protocol.py file 
#################################################


import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
from homing_protocol import *
import time


# macros for Roboclaws
# WARNING: DO NOT CHANGE THESE MACROS UNLESS USER IS COMFORTABLE
#          WITH ARM. Macros are set up for current test arm mounted 
#          to rover 

# BASE Motor info
BASE_ADDR = 128
BASE_MOTOR = 2
BASE_FULLROT = 15000
# Number of encoder values to make on full rotation
BASE_SPEED = 50
# Speed of BASE while homing


# ELBOW Motor info
ELBOW_ADDR = 128
ELBOW_MOTOR = 2
ELBOW_FULLROT = 3250
# Number of encoder values to make on full rotation
ELBOW_ENC_DEG = 9
# Amount and direction the homing will step the wrist per move
ELBOW_ENC_BREAK = 6
# The max encoder values that will cause a home detection for the ELBOW 
ELBOW_SPEED = 40
# Speed of ELBOW while homing



# WRIST Motor info
WRIST_ADDR = 129
WRIST_MOTOR = 2
WRIST_FULLROT = 1000
# Number of encoder values to make on full rotation
WRIST_ENC_DEG = -6
# Amount and direction the homing will step the WRIST per move
WRIST_ENC_BREAK = 4
# The max encoder values that will cause a home detection for the WRIST
WRIST_SPEED = 40
# Speed of WRIST while homing



# CLAW Motor info
CLAW_ADDR = 129
CLAW_MOTOR = 1
CLAW_ROT_OPENCLOSE = 60
# Amount to run forward to open or close the claw 
CLAW_BACKWARD_FULLROT = -230
# Amount to run backward to complete a full rotation of claw
CLAW_SPEED = 100
# Speed of CLAW while homing




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

def basic_testing(rc):
    test_setup(rc)
    home_base_setup_run(rc)
    
    double_run_homing(rc, WRIST_ADDR, WRIST_MOTOR, WRIST_ENC_DEG, WRIST_ENC_BREAK)
    double_run_homing(rc, ELBOW_ADDR, ELBOW_MOTOR, ELBOW_ENC_DEG, ELBOW_ENC_BREAK)


def full_testing(rc):
#   Program will run the homing protocol a given number of time as specified below
#   and produce range 
    val = 1

def print_motor_addresses():
    print("CURRENT ARM ROBOCLAW INFORMATION\n\n")
    time.sleep(0.5)
    print("BASE Motor Roboclaw Address", BASE_ADDR, "\n")
    print("BASE Motor Number: ", BASE_MOTOR, "\n\n")
    time.sleep(0.5)
    print("ELBOW Motor Roboclaw Address", ELBOW_ADDR, "\n")
    print("ELBOW Motor Number: ", ELBOW_MOTOR, "\n\n")
    time.sleep(0.5)
    print("WRIST Motor Roboclaw Address", WRIST_ADDR, "\n")
    print("WRIST Motor Number: ", WRIST_MOTOR, "\n\n")
    time.sleep(0.5)
    print("CLAW Motor Roboclaw Address", CLAW_ADDR, "\n")
    print("CLAW Motor Number: ", CLAW_MOTOR, "\n\n")
    time.sleep(0.25)
    print("Please ensure values are are correct before moving forward with testing\n")
    input("Press any key to Continue\n\n")


def test_homing(rc):
#   Main location for testing the homing protocol 
    print("CURRENTLY TESTING HOMING PROCEDURE\n\n")
    print_motor_addresses()
#   Printing out information on motor before going into normal testing
    if(input("Do you want to only find the home position once? y/n" == "y")):
#       CASE: only the basic procedure should be run here to find the home
        basic_testing(rc)
    else:
#       CASE: Run full testing procedure to test acuracy of arm
        full_testing(rc)





def main():
#   configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
#   generate/open port
    rc.Open()
    
    basic_testing(rc)
if __name__ == "__main__":
    main()