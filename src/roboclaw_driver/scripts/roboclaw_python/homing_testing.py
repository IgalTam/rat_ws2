# AUTHOR: Sebastian Rosso (NUADA Team)
# DATE: 2/20/2023
#   PURPOSE: This program is designed to be able to safely test the Homing
#            Protocol for reliablity without making modifications to the
#            homing_protocol.py file 
#################################################


import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
from homing_protocol import *
from encoders import configSettings
import time


# macros for Roboclaws
# WARNING: DO NOT CHANGE THESE MACROS UNLESS USER IS COMFORTABLE
#          WITH ARM. Macros are set up for current test arm mounted 
#          to rover 

# BASE Motor info
BASE_ADDR = 128
BASE_MOTOR = 1
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
ELBOW_ENC_BREAK = 7
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
CLAW_BACKWARD_FULLROT = -240
# Amount to run backward to complete a full rotation of claw
CLAW_SPEED = 100
# Speed of CLAW while homing



def single_move(rc: Roboclaw):
    while(input("Want to change an arm starting position? y/n\n") == "y"):

        motorNum = 0
        encoderVal = 0
        address = 0
        value = 0
        print("Enter the first letter of the joint you want to move")
        print("c for claw")
        print("w for wrist")
        print("e for elbow")
        print("b for base")
        value = int(input("input: "))
        if(value == "c"):
            address = CLAW_ADDR
            motorNum = CLAW_MOTOR
        elif(value == "w"):
            address = WRIST_ADDR
            motorNum = WRIST_MOTOR
        elif(value == "e"):
            address = ELBOW_ADDR
            motorNum = ELBOW_MOTOR
        elif(value == "b"):
            address = BASE_ADDR
            motorNum = BASE_MOTOR
        else:
            print("INVALID ENTRY: ", value, "\nReturning to main menu\n\n")
            return
        
        encoderVal = input("Enter the the value for the ammount of encoder position you want to move: ")
        print("Values: ", address, " ", motorNum, " ", encoderVal)
        turn_by_encoder(rc, address, motorNum, encoderVal, 40, 1)

        while(input("Want to make another move with the same setup? y/n\n") == "y"):
            encoderVal = int(input("How many Encoder turns?"))
            print("Values: ", address, " ", motorNum, " ", encoderVal)
            turn_by_encoder(rc, address, motorNum, encoderVal, 40, 1)
        
    print("SetUp complete\nMoving into normal testing\n\n")

def single_homing_test(rc):
    print("Enter the first letter of the joint you want to home")
    print("c for claw")
    print("w for wrist")
    print("e for elbow")
    print("b for base")
    value = int(input("input: "))
    if(value == "c"):
        home_claw_setup_run(rc)
    elif(value == "w"):
        double_run_homing(rc, WRIST_ADDR, WRIST_MOTOR, WRIST_ENC_DEG, WRIST_ENC_BREAK, WRIST_SPEED)
    elif(value == "e"):
        double_run_homing(rc, ELBOW_ADDR, ELBOW_MOTOR, ELBOW_ENC_DEG, ELBOW_ENC_BREAK, ELBOW_SPEED)
    elif(value == "b"):
        home_base_setup_run(rc)
    else:
        print("INVALID ENTRY: ", value, "\nReturning to main menu\n\n")
        return

def move_away(rc):
    turn_by_encoder(rc, BASE_ADDR, BASE_MOTOR, 2000, 100, 0, 1)
    turn_by_encoder(rc, ELBOW_ADDR, ELBOW_MOTOR, -400, 80, 0, 1)
    turn_by_encoder(rc, WRIST_ADDR, WRIST_MOTOR, 200, 80, 0, 1)
    time.sleep(20)
    print("Movements Complete\n")


def confirm_motor_addresses():
    print("CURRENT ARM ROBOCLAW INFORMATION")
    print("--------------------------------")
    time.sleep(0.5)
    print("BASE Motor Roboclaw Address", BASE_ADDR)
    print("BASE Motor Number: ", BASE_MOTOR)
    print("--------------------------------")
    time.sleep(0.5)
    print("ELBOW Motor Roboclaw Address", ELBOW_ADDR)
    print("ELBOW Motor Number: ", ELBOW_MOTOR)
    print("--------------------------------")
    time.sleep(0.5)
    print("WRIST Motor Roboclaw Address", WRIST_ADDR)
    print("WRIST Motor Number: ", WRIST_MOTOR)
    print("--------------------------------")
    time.sleep(0.5)
    print("CLAW Motor Roboclaw Address", CLAW_ADDR)
    print("CLAW Motor Number: ", CLAW_MOTOR)
    print("--------------------------------")
    time.sleep(0.25)
    print("Please ensure values are are correct before moving forward with testing")
    input("Press any key to Continue\n\n")


def test_main(rc):
#   Main location for testing the homing protocol 
    print("###############################\nRUNNING ARM TESTING ENVIRONMENT\n###############################\n")
    confirm_motor_addresses()
#   Printing out information on motor before going into normal testing

    while(1 == 1):
#   Test Env Main loop
        print("Please enter the number of the type of testing you want to complete")
        print("--------------------------------")
        print("1 to move the arm out of the home position")
        print("2 for a single homing attempt")
        print("3 to test a signle homing feature")
        print("4 to exit testing envirorment\n")
        menuNav = int(input("Enter choice Here: "))
#       Offering choice to user to run different types of tests 
        if(menuNav == 1):
#           CASE: Single Movements for arm
            print("\n#############\nSINGLE MOVEMENTS\n#############")
            print("WARNING: the addresses used in config settings are not bound to Macros in homing_testing.py\n\n")
            single_move(rc)                
        elif(menuNav == 2):
#           CASE: Run a single homing run
            print("\n#################\nSINGLE HOMING\n#################")
            if(input("\nARM WILL TRY TO FIND HOME, confirm? y/n\n\n") == "y"):
                homing_procedure(rc)
        elif(menuNav == 3):
#               CASE: Run full testing procedure to test accuracy of arm
            print("\n##############\nTEST PART OF HOMING\n##############")
            
        elif(menuNav == 4):
#           CASE: Leave Testing Environment
            print("\n########################\nLeaving Test Environment\n########################")
            break
        else:
#           CASE: Invalid entry
            print("\nINVALID ENTRY: ", menuNav, "\n\n")





def main():
#   configure Roboclaws
    rc = Roboclaw("/dev/ttyS0", 115200)
#   generate/open port
    rc.Open()
    
    test_main(rc)
if __name__ == "__main__":
    main()