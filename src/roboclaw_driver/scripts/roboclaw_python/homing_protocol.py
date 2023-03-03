# AUTHOR: Sebastian Rosso (NUADA Team)
# DATE: 1/15/2023
#   PURPOSE: This is the file containing the functions to complete the homing.
#            DO NOT MAKE CHANGES TO THE FILE IF ATTEMPTING TESTING. go to the 
#            homing_testing.py 
#################################################

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

ROBOCLAW_1 = 128
ROBOCLAW_2 = 129

TEST_SPEED = 40

def turn_by_encoder(rc: Roboclaw, address, motorNum, encoderVal, motorSpd, printFlag, sleepTime):
#   Purpose of this fuction is smplify the roboclaw movements between M1 and M2,
#   CASE: the sleep time on this function is for short movements
#   RETURNS: the new ecoder postion the motor is at


    if(motorNum == 1):
#       CASE: moving M1 on addressed roboclaw
        currentPos = rc.ReadEncM1(address)[1]
        rc.SpeedAccelDeccelPositionM1(address, 0, motorSpd, 0, (currentPos + encoderVal), 1)
#       sleep to ensure the move is complete
        time.sleep(sleepTime)
        finalPos = rc.ReadEncM1(address)[1]
    elif(motorNum == 2):
#       CASE: moving M2 on addressed roboclaw
        currentPos = rc.ReadEncM2(address)[1]
        rc.SpeedAccelDeccelPositionM2(address, 0, motorSpd, 0, (currentPos + encoderVal), 1)
#       sleep to ensure the move is complete
        time.sleep(sleepTime)
        finalPos = rc.ReadEncM2(address)[1]
    else:
#       CASE: Invalid entry
#        print("Are we here? : ", motorNum)
        return -1
    
    if(printFlag == 1):
#       CASE: if the print flag is set, main for debugging
        print("Move Complete \n")
        print("Expected Distance: ", encoderVal)
        print("\nActual Distance: ", abs(currentPos - finalPos), "\n\n")
    
    return finalPos

def read_encoder(rc: Roboclaw, address, motorNum):
#   saving time by getting rid of the m1 and m2 reads
    encoderVal = 0
    if(motorNum == 1):
        encoderVal = rc.ReadEncM1(address)[1]
    elif(motorNum == 2):
        encoderVal = rc.ReadEncM2(address)[1]
    return encoderVal




def step_till_stop(rc: Roboclaw, address, motorNum, encoderVal, breakVal):
#   OBJECTIVE: Will rotate on any roboclaw to move given ammount till it reaches a physical stop
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position
    oldPos = 10000
    disMoved = 100
    newPos = read_encoder(rc, address, motorNum)
#   newPos needs to be set to a value that will not break out of while loop right away

    while not (disMoved <= breakVal ):
#       Moves arm position at a slow rate towrds its desired physical stop       
        oldPos = newPos
        newPos = turn_by_encoder(rc, address, motorNum, encoderVal, TEST_SPEED, 1, 0.75)
#       TODO: Ensure Speed and amount moved is correct AND DIRECTION!!!!!  

#       Updating position by setting the current position(newPos) to oldPos for next move 

        disMoved = abs(oldPos - newPos)
    
    time.sleep(0.75)
    return read_encoder(rc, address, motorNum) 


    

def solid_move_homing(rc: Roboclaw, address, motorNum, encoderVal, breakVal):
    """OBJECTIVE: Will rotate a motor with a single move to find a stop"""
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    oldPos = 10000
    disMoved = 100
    newPos = read_encoder(rc, address, motorNum)
#   newPos needs to be set to a value that will not break out of while loop right away

    newPos = turn_by_encoder(rc, address, motorNum, encoderVal, TEST_SPEED, 0, 0.75)
    time.sleep(0.3)
    try:
        while not (disMoved <= breakVal):
#           Moves arm position at a slow rate towrds its desired physical stop       
            time.sleep(0.3)
            oldPos = newPos
            newPos = read_encoder(rc, address, motorNum)
            disMoved = abs(oldPos - newPos)

            print("Move Complete \n")
            print("Expected Distance: ", abs(encoderVal))
            print("\nActual Distance: ", abs(oldPos - newPos), "\n\n")
    except KeyboardInterrupt:
        print("FORCED OUT OF WHILE LOOP\n\n")
        finetune2 = 75
        if(address == ELBOW_ADDR):
            finetune2 = -75
        turn_by_encoder(rc, address, motorNum, finetune2, TEST_SPEED, 0, 0.75)
            


    newPos = read_encoder(rc, address, motorNum) 
    finetune = 75
    if(address == ELBOW_ADDR):
        finetune = -75
    newPos = turn_by_encoder(rc, address, motorNum, finetune, TEST_SPEED, 0, 0.75)



def home_base_setup_run(rc):

    # To ensure s4 settings are correct, they are manually 
    # configured in code below
    n=0

    # setting s4 to default: off
    rc.SetPinFunctions(ROBOCLAW_1, 0, 0, 0)
    time.sleep(1)
    # rc.SetPinFunctions(ROBOCLAW_2, 0, 0, 0)
    # time.sleep(1)

    # setting s4 to setting motor home (user)
    rc.SetPinFunctions(ROBOCLAW_1, 0, 0x62, 0)
    print("s4 set to 0x62 for ROBOCLAW 1")
    time.sleep(1)

    # rc.SetPinFunctions(ROBOCLAW_2, 0, 0x62, 0)
    # print("s4 set to 0x62 for ROBOCLAW 2")
    # time.sleep(2)

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    homed_base = ((rc.ReadError(ROBOCLAW_1)[1] & 0x400000) == 0x400000)
    # homed_claw = ((rc.ReadError(ROBOCLAW_2)[1] & 0x400000) == 0x400000)

    home_base(homed_base, rc)
    print("BASE HOMED!!")

    # home_claw(homed_claw, rc)
    # print("CLAW HOMED")

    return 0

def home_claw_setup_run(rc):

    # To ensure s4 settings are correct, they are manually 
    # configured in code below
    n=0

    # setting s4 to default: off
    # rc.SetPinFunctions(ROBOCLAW_1, 0, 0, 0)
    # time.sleep(1)
    rc.SetPinFunctions(ROBOCLAW_2, 0, 0, 0)
    time.sleep(1)

    # setting s4 to setting motor home (user)
    # rc.SetPinFunctions(ROBOCLAW_1, 0, 0x62, 0)
    # print("s4 set to 0x62 for ROBOCLAW 1")
    # time.sleep(1)

    rc.SetPinFunctions(ROBOCLAW_2, 0, 0x62, 0)
    print("s4 set to 0x62 for ROBOCLAW 2")
    time.sleep(2)

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    # homed_base = ((rc.ReadError(ROBOCLAW_1)[1] & 0x400000) == 0x400000)
    homed_claw = ((rc.ReadError(ROBOCLAW_2)[1] & 0x400000) == 0x400000)

    # Rotating claw maximum 4 full rotations for homing
    # print("homing")
    homing_length = 4 * -230
    # rc.SpeedAccelDistanceM1(ROBOCLAW_2,0,40,homing_length,0)
    rc.SpeedAccelDeccelPositionM1(ROBOCLAW_2,0,100,0,homing_length,1)
    time.sleep(2)
    # print("homed before 360")

    home_claw(homed_claw, rc)
    print("CLAW HOMED")

    rc.SetPinFunctions(ROBOCLAW_2, 0, 0, 0)
    time.sleep(1)
    


    return 0

def home_base(homed, rc):
    val = 0
    speed = 50 ; accel = 0; deccel = 0
    step = 100
    prev_pos = rc.ReadEncM1(ROBOCLAW_1)[1]

    while (homed != True):
        # determine new position toward home
        val -= step
        print(val)
        # move to new position
        rc.SpeedAccelDeccelPositionM1(ROBOCLAW_1, accel, speed, deccel, val, 1)
        time.sleep(3)
        
        # read in new position after move
        cur_pos = rc.ReadEncM1(ROBOCLAW_1)[1]
        print(f"prev_pos: {prev_pos}, cur_pos: {cur_pos}")
        
        #check if hall effect sensor hit
        homed = ((rc.ReadError(ROBOCLAW_1)[1] & 0x400000) == 0x400000)

        if homed:
            break
        else:
            # Home not hit and read yet
            # fail-safe
            if (abs(cur_pos - prev_pos) < (step * 0.9)):
                # arm could be breaking / Hall effect missed
                rc.SpeedAccelDeccelPositionM1(ROBOCLAW_1, accel, 50, deccel, cur_pos + 500, 1)
                val = cur_pos + 500
                print("\n\n**********\nBASE MISSED HALL EFFECT\n**********\n")
                time.sleep(10)


        prev_pos = cur_pos

        # time.sleep(1)

    return 0


def home_claw(homed, rc):

    while(homed != True):
        # check pins
        err = rc.ReadError(ROBOCLAW_2)
        # if s4 pin high, exit loop
        if ((err[1] & 0x400000) == 0x400000):
            break
        time.sleep(1)

    return 0


def double_run_homing(rc: Roboclaw, address, motorNum, encoderVal, breakVal):
#   OBJECTIVE: To have the desired homing method run till find a stop, then have a back off and test again
#   TODO: Run possible 3 checks or mind a way to prevent a second false positive from getting through
#   TODO: Make sure the stepback wont hit something by insuring the stepback is less than the amount moved already
    trueHome = False

    firstStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal)
#   First attempt at homing
    secondStop = 0
    while not (trueHome):
        stepBack = (-1 * encoderVal * 5)
#       the ammount the arm will step back before attempting the homing again
        turn_by_encoder(rc, address, motorNum, stepBack, TEST_SPEED, 1, 0.75)

        time.sleep(2)

        secondStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal)
        print("HOME FOUND!!!\n\n\n\n\n\n\n\n\n")
#       attempting to finding stop again

        if(abs(secondStop - firstStop) <= breakVal * 2):
#           CASE: found that the new position is similar to the last one
            trueHome = True
        else:
#           CASE: there is some discreptency between the two values
            firstStop = secondStop
    
#   Step back after finding home
    turn_by_encoder(rc, address, motorNum, (encoderVal * -1 * 2), TEST_SPEED, 1, 0.75)
   



def multi_run_homing(rc: Roboclaw, address, motorNum, encoderVal, breakVal):
#   OBJECTIVE: To have the desired homing method run till find a stop, then have a back off and test again multiple times to ensure its working
#   TODO: Run possible 3 checks or mind a way to prevent a second false positive from getting through
#   TODO: Make sure the stepback wont hit something by insuring the stepback is less than the amount moved already
    trueHome = False
    recordedHomes = list[5]

    firstStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal)
#   First attempt at homing
    secondStop = 0
    while not (trueHome):
        stepBack = (-1 * encoderVal * 5)
#       the ammount the arm will step back before attempting the homing again
        turn_by_encoder(rc, address, motorNum, stepBack, TEST_SPEED, 1, 0.75)

        time.sleep(2)

        secondStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal)
        print("HOME FOUND!!!\n\n\n\n\n\n\n\n\n")
#       attempting to finding stop again

        if(abs(secondStop - firstStop) <= breakVal * 2):
#           CASE: found that the new position is similar to the last one
            trueHome = True
        else:
#           CASE: there is some discreptency between the two values
            firstStop = secondStop
    
#   Step back after finding home
    turn_by_encoder(rc, address, motorNum, (encoderVal * -1 * 2), TEST_SPEED, 1, 0.75)







# anirudh and trenten claw protocol for testing main
def main():
    #   configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)
#   generate/open port
    rc.Open()
    # print("here\n\n")
    # test_setup(rc)
    home_base_setup_run(rc)
    # home_claw_setup_run(rc)

    # double_run_homing(rc, WRIST_ADDR, WRIST_MOTOR, TEST_WRIST_ENC_DEG, 4)
    # double_run_homing(rc, ELBOW_ADDR, ELBOW_MOTOR, TEST_ELBOW_ENC_DEG, 6)


# sebastian homing protocol main
# def main():
# #   configure Roboclaws
#     rc = Roboclaw("/dev/ttyAMA1", 115200)
# #   generate/open port
#     rc.Open()


if __name__ == "__main__":
    main()
