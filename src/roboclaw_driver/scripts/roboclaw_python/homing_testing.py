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


def kill_all_motors(rc):
#   Should only be used in urgent situations will stop the movement of the whole arm
    turn_by_encoder(rc, BASE_ADDR, BASE_MOTOR, 0, BASE_SPEED, 0, 0.1)
    turn_by_encoder(rc, ELBOW_ADDR, ELBOW_MOTOR, 0, ELBOW_SPEED, 0, 0.1)
    turn_by_encoder(rc, WRIST_ADDR, WRIST_MOTOR, 0, WRIST_SPEED, 0, 0.1)
    turn_by_encoder(rc, CLAW_ADDR, CLAW_MOTOR, 0, CLAW_SPEED, 0, 0.1)


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
#   Base homing
    home_base_setup_run(rc)
#   Wrist Homing
    double_run_homing(rc, WRIST_ADDR, WRIST_MOTOR, WRIST_ENC_DEG, WRIST_ENC_BREAK)
#   Elbow Homing
    double_run_homing(rc, ELBOW_ADDR, ELBOW_MOTOR, ELBOW_ENC_DEG, ELBOW_ENC_BREAK)


def full_testing(rc):
#   Program will run the homing protocol a given number of time as specified below
#   and produce range 
    numRuns = int(input("Enter the number of times you want to run homing: "))
    print("\nREADY TO RUN TESTING FOR HOMING\n")
    print("NOTE: at any point during testing, using ctrl + C will stop the arm in any position and return to the menu\n")
    input("Press any key to continue: ")
#   Start of Testing Run
    try:
        val = 1
    except KeyboardInterrupt:
        print("FORCED OUT OF TESTING LOOP\n")
        kill_all_motors(rc)
















def confirm_motor_addresses():
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


def test_main(rc):
#   Main location for testing the homing protocol 
    print("RUNNING ARM TESTING ENVIRONMENT\n\n")
    confirm_motor_addresses()
#   Printing out information on motor before going into normal testing

    while(1 == 1):
#   Test Env Main loop
        print("Please enter the number of the type of testing you want to complete\n")
        print("1 for User entered Signle Motor Movments\n")
        print("2 for a single homing attempt\n")
        print("3 for the Full Homing Procedure Testing\n")
        print("4 to exit testing envirorment\n")
        menuNav = input("Enter choice Here: ")
#       Offering choice to user to run different types of tests 
        match(menuNav):
            case 1:
#               CASE: Single Movements for arm
                print("\nWARNING: the addresses used in config settings are not bound to Macros in homing_testing.py\n\n")
                configSettings()
                
            case 2:
#               CASE: Run a single homing run
                if(input("\nARM WILL TRY TO FIND HOME, confirm? y/n\n\n") == "y"):
                    basic_testing(rc)
            case 3:
#               CASE: Run full testing procedure to test accuracy of arm
                if(input("\nARM WILL RUN FULL HOMING TEST, confirm? y/n\n\n") == "y"):
                    print(("WARNING: Before Testing, please confirm arm is in the ideal home position\n"))
                    print("This is extremley important for safe and acurate test results\n")
                    if(input("Do you want to continue? y/n\n") == "y"):
                        full_testing(rc)
            case 4:
#               CASE: Leave Testing Environment
                print("\nLeaving Test Environment\n")
                break
            case _:
#               CASE: Invalid entry
                print("\nINVALID ENTRY: ", menuNav, "\n\n")





def main():
#   configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
#   generate/open port
    rc.Open()
    
    test_main(rc)
if __name__ == "__main__":
    main()