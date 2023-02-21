import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
import time


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



class Roboclaw_Motor:
    address = 0
    motorNum = 0
    baseSpd = 0

    
    def __init__(addressInit, motorNumInit, baseSpdInit):
        address = addressInit
        motorNum = motorNumInit
        baseSpd = baseSpdInit





def main():
    print("new val\n\n")
 


    



if __name__ == "__main__":
    main()
