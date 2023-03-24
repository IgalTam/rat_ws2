# AUTHOR: Sebastian Rosso (NUADA Team)
# DATE: 1/15/2023
#   PURPOSE: This is the file containing the functions to complete the homing.
#            DO NOT MAKE CHANGES TO THE FILE IF ATTEMPTING TESTING. go to the 
#            homing_testing.py 
#################################################

import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
import time
import sys

"""non-ROS homing protocol implementation"""

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
ELBOW_SPEED = 60
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
WRIST_SPEED = 60
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
        print("Move Complete")
        print("Expected Distance: ", abs(encoderVal))
        print("Actual Distance: ", abs(currentPos - finalPos), "\n\n")
    
    return finalPos

def read_encoder(rc: Roboclaw, address, motorNum):
#   saving time by getting rid of the m1 and m2 reads
    encoderVal = 0
    if(motorNum == 1):
        encoderVal = rc.ReadEncM1(address)[1]
    elif(motorNum == 2):
        encoderVal = rc.ReadEncM2(address)[1]
    return encoderVal


def kill_all_motors(rc):
#   Should only be used in urgent situations will stop the movement of the whole arm
    turn_by_encoder(rc, BASE_ADDR, BASE_MOTOR, 0, BASE_SPEED, 0, 0.1)
    turn_by_encoder(rc, ELBOW_ADDR, ELBOW_MOTOR, 0, ELBOW_SPEED, 0, 0.1)
    turn_by_encoder(rc, WRIST_ADDR, WRIST_MOTOR, 0, WRIST_SPEED, 0, 0.1)
    turn_by_encoder(rc, CLAW_ADDR, CLAW_MOTOR, 0, CLAW_SPEED, 0, 0.1)

#   def set_enc_zero(rc):
#   Used to zero all motors an RAT arm(used at end of homing sequence)


def step_till_stop(rc: Roboclaw, address, motorNum, encoderVal, breakVal, speed):
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
        newPos = turn_by_encoder(rc, address, motorNum, encoderVal, speed, 1, 0.75)
#       TODO: Ensure Speed and amount moved is correct AND DIRECTION!!!!!  

#       Updating position by setting the current position(newPos) to oldPos for next move 

        disMoved = abs(oldPos - newPos)
    
    time.sleep(0.75)
    return read_encoder(rc, address, motorNum) 


    

def solid_move_homing(rc: Roboclaw, address, motorNum, encoderVal, breakVal, speed):
    """OBJECTIVE: Will rotate a motor with a single move to find a stop"""
#   This is used for nonROS homing, running under the assumption that the motor does not know
#   its position

    oldPos = 10000
    disMoved = 100
    newPos = read_encoder(rc, address, motorNum)
#   newPos needs to be set to a value that will not break out of while loop right away

    newPos = turn_by_encoder(rc, address, motorNum, encoderVal, speed, 0, 0.75)
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
        turn_by_encoder(rc, address, motorNum, finetune2, speed, 0, 0.75)
            


    newPos = read_encoder(rc, address, motorNum) 
    finetune = 75
    if(address == ELBOW_ADDR):
        finetune = -75
    newPos = turn_by_encoder(rc, address, motorNum, finetune, speed, 0, 0.75)



def home_base_setup_run(rc):
    """OBJECTIVE: Will make necessary function calls to set up the base for roboclaw
    limit switch options and then run the code to home the base"""
    # To ensure s4 settings are correct, they are manually 
    # configured in code below

    # setting s4 to default: off
    rc.SetPinFunctions(BASE_ADDR, 0, 0, 0)
    time.sleep(1)

    # setting s4 to setting motor home (user)
    rc.SetPinFunctions(BASE_ADDR, 0, 0x62, 0)
    time.sleep(1)

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    homed_base = ((rc.ReadError(BASE_ADDR)[1] & 0x400000) == 0x400000)

    # Call the function that will home the base
    home_base(homed_base, rc)
    print("BASE HOMED!!")

    return 0

def home_claw_setup_run(rc):
    """OBJECTIVE: Will make necessary function calls to set up the claw for roboclaw
    limit switch options and then run the code to home the claw"""
    # To ensure s4 settings are correct, they are manually 
    # configured in code below

    # setting s4 to default: off
    rc.SetPinFunctions(CLAW_ADDR, 0, 0, 0)
    time.sleep(1)

    # setting s4 to setting motor home (user)
    rc.SetPinFunctions(CLAW_ADDR, 0, 0x62, 0)
    time.sleep(2)

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    homed_claw = ((rc.ReadError(CLAW_ADDR)[1] & 0x400000) == 0x400000)

    # Rotating claw maximum 4 full rotations for homing
    homing_length = 4 * -230
    rc.SpeedAccelDeccelPositionM1(CLAW_ADDR,0,100,0,homing_length,1)
    time.sleep(2)

    # call the function to home the claw
    home_claw(homed_claw, rc)
    print("CLAW HOMED")

    rc.SetPinFunctions(CLAW_ADDR, 0, 0, 0)
    time.sleep(1)

    return 0

def home_base(homed, rc):
    """OBJECTIVE: Will run through procedure to home the base"""
    val = 0
    speed = 50 ; accel = 0; deccel = 0
    step = 100
    prev_pos = rc.ReadEncM1(BASE_ADDR)[1]

    while (homed != True):
        # determine new position toward home
        val -= step
        print(val)
        # move to new position
        rc.SpeedAccelDeccelPositionM1(BASE_ADDR, accel, speed, deccel, val, 1)
        rc.SpeedAccelDeccelPositionM1(BASE_ADDR, accel, speed, deccel, val, 1) # doubled to eliminate runs that don't go through
        time.sleep(2.5)
        
        # read in new position after move
        cur_pos = rc.ReadEncM1(BASE_ADDR)[1]
        print(f"prev_pos: {prev_pos}, cur_pos: {cur_pos}")
        
        #check if hall effect sensor hit
        homed = ((rc.ReadError(BASE_ADDR)[1] & 0x400000) == 0x400000)

        if homed:
            break
        else:
            # Home not hit and read yet
            # we have entered fail-safe
            if (abs(cur_pos - prev_pos) < (step * 0.9)):
                # arm could be breaking / Hall effect missed
                rc.SpeedAccelDeccelPositionM1(BASE_ADDR, accel, 50, deccel, cur_pos + 500, 1)
                val = cur_pos + 500
                print("\n\n**********\nBASE MISSED HALL EFFECT\n**********\n")
                time.sleep(10)

        prev_pos = cur_pos
    rc.SpeedAccelDeccelPositionM1(BASE_ADDR, accel, speed, deccel, 0, 1)
    time.sleep(5)

    return 0


def home_claw(homed, rc):
    """OBJECTIVE: Will run through procedure to home the claw"""
    while(homed != True):
        # check pins
        err = rc.ReadError(CLAW_ADDR)
        # if s4 pin high, exit loop
        if ((err[1] & 0x400000) == 0x400000):
            break
        time.sleep(1)

    return 0


def double_run_homing(rc: Roboclaw, address, motorNum, encoderVal, breakVal, speed):
#   OBJECTIVE: To have the desired homing method run till find a stop, then have a back off and test again
#   TODO: Run possible 3 checks or mind a way to prevent a second false positive from getting through
#   TODO: Make sure the stepback wont hit something by insuring the stepback is less than the amount moved already
    trueHome = False

    firstStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal, speed)
#   First attempt at homing
    secondStop = 0
    while not (trueHome):
        stepBack = (-1 * encoderVal * 10)
#       the ammount the arm will step back before attempting the homing again
        turn_by_encoder(rc, address, motorNum, stepBack, speed, 1, 3)

        time.sleep(2)

        print("A HOME HAS BEEN FOUND\n")
        secondStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal, speed)
#       attempting to finding stop again

        if(abs(secondStop - firstStop) <= breakVal * 2):
#           CASE: found that the new position is similar to the last one
            trueHome = True
        else:
#           CASE: there is some discreptency between the two values
            firstStop = secondStop
    
#   Step back after finding home
    print("RECENT HOME POSITION MATCH PREVIOUS HOME POSITION\n")
    turn_by_encoder(rc, address, motorNum, (encoderVal * -1 * 2), speed, 1, 5)
   



def multi_run_homing(rc: Roboclaw, address, motorNum, encoderVal, breakVal, speed):
#   UNDER CONSTRUCTION
#   OBJECTIVE: To have the desired homing method run till find a stop, then have a back off and test again multiple times to ensure its working
#   TODO: Run possible 3 checks or mind a way to prevent a second false positive from getting through
#   TODO: Make sure the stepback wont hit something by insuring the stepback is less than the amount moved already
    trueHome = False
    # recordedHomes = list[5]

    firstStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal, speed)
#   First attempt at homing
    secondStop = 0
    while not (trueHome):
        stepBack = (-1 * encoderVal * 5)
#       the ammount the arm will step back before attempting the homing again
        turn_by_encoder(rc, address, motorNum, stepBack, speed, 1, 0.75)

        time.sleep(2)

        secondStop = step_till_stop(rc, address, motorNum, encoderVal, breakVal, speed)
        print("HOME FOUND!!!\n\n\n\n\n\n\n\n\n")
#       attempting to finding stop again

        if(abs(secondStop - firstStop) <= breakVal * 2):
#           CASE: found that the new position is similar to the last one
            trueHome = True
        else:
#           CASE: there is some discreptency between the two values
            firstStop = secondStop
    
#   Step back after finding home
    turn_by_encoder(rc, address, motorNum, (encoderVal * -1 * 2), speed, 1, 0.75)



def homing_procedure(rc):
#   Wrist Homing
    double_run_homing(rc, WRIST_ADDR, WRIST_MOTOR, WRIST_ENC_DEG, WRIST_ENC_BREAK, WRIST_SPEED)
#   Base homing
    home_base_setup_run(rc)
#   Elbow Homing
    double_run_homing(rc, ELBOW_ADDR, ELBOW_MOTOR, ELBOW_ENC_DEG, ELBOW_ENC_BREAK, ELBOW_SPEED)
#   Claw Homing
    home_claw_setup_run(rc)
#   Turn Claw 90 degrees for alignment
    turn_by_encoder(rc, CLAW_ADDR, CLAW_MOTOR, CLAW_BACKWARD_FULLROT//4, CLAW_SPEED, 1, 3)
#   Moving wrist to actual home at end (45 degrees off from base)
    turn_by_encoder(rc, WRIST_ADDR, WRIST_MOTOR, (3*WRIST_FULLROT)//8, 100, 1, 7)


def arm_setup():
#   OBJECTIVE: setup procedure when it is time to wake up the arm. The is no case where the arm
#              can return a failed attempt to home/setup. If ot fails it will be stuck in the homing loop

#   TODO: Turning on a pin for the roboclaw power
    time.sleep(3)
#   waiting 3 seconds to ensure the arm is powered up
    rc = Roboclaw("/dev/ttyS0", 115200)
#   configure Roboclaws
    rc.Open()
#   generate/open port
    time.sleep(1)
    homing_procedure(rc)
#   Running of basic homing procedure
#   TODO: Set all roboclaw encoder values to zero

    return rc
#   returns the rc object to call to the UART channels

def arm_shutdown(rc):
#   OBJECTIVE: Called when it is time to power down the arm 
    kill_all_motors(rc)

    return 0

# anirudh and trenten claw protocol for testing main
def main(cmd_call=None):
    #   configure Roboclaws
    rc = Roboclaw("/dev/ttyS0", 115200)
#   generate/open port
    rc.Open()
    if cmd_call == "one":
        homing_procedure(rc)


if __name__ == "__main__":
    if len(sys.argv) == 2:
        main(sys.argv[1])