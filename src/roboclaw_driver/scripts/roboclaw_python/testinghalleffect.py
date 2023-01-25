#!/usr/bin/python3
import time
from sys import exit
from roboclaw_3 import Roboclaw

# constants
CLAW_FORWARDROT = 60
CLAW_BACKWARDROT = 240
ELBOW_FULLROT = 3250
WRIST_FULLROT = 1000
BASE_FULLROT = 15000
FULLROT = 360

ROBOCLAW_1 = 128
ROBOCLAW_2 = 129

def updateMotorPos(address, accel, speed, deccel, motor):
    if motor == 1:
        print(f'Current Encoder value = {rc.ReadEncM1(address)}')
    elif motor == 2:
        print(f'Current Encoder value = {rc.ReadEncM2(address)}')
    angle = input("Enter angle: ")
    if angle.isnumeric() or (angle[1:].isnumeric() and angle[0] == '-'):
        val = angleToEncoders(address, motor, int(angle))
        if motor == 1:
            rc.SpeedAccelDeccelPositionM1(address, accel, speed, deccel, val, 1)
        elif motor == 2:
            rc.SpeedAccelDeccelPositionM2(address, accel, speed, deccel, val, 1)

def angleToEncoders(address, motor, angle):
    if (address == ROBOCLAW_1):
        if (motor == 1):
            return (angle * ELBOW_FULLROT) // FULLROT
        elif (motor == 2):
            return (angle * BASE_FULLROT) // FULLROT
    elif (address == ROBOCLAW_2):
        if (motor == 1):
            if (angle >= 0):
                return (angle * CLAW_FORWARDROT) // FULLROT
            else:
                return (angle * CLAW_BACKWARDROT) // FULLROT
        elif (motor == 2):
            return (angle * WRIST_FULLROT) // FULLROT
    return 0


def configSettings():
    address = 0
    motor = 0
    motor_query = input("Base (b), Elbow (e), Wrist (w), or Claw (c)? ").lower()
    if motor_query == 'b':
        address = 128
        motor = 2
    elif motor_query == 'e':
        address = 128
        motor = 1
    elif motor_query == 'w':
        address = 129
        motor = 2
    elif motor_query == 'c':
        address = 129
        motor = 1
    else:
        print('Invalid input')
        exit()
    accel = input("Acceleration (0 is default): ")
    if not accel or not accel.isnumeric():
        accel = 0
    else:
        accel = int(accel)
    speed = input("Speed/Velocity (200 is default): ")
    if not speed or not speed.isnumeric():
        speed = 200
    else:
        speed = int(speed)
    deccel = input("deccel (0 is default): ")
    if not deccel or not deccel.isnumeric():
        deccel = 0
    else:
        deccel = int(deccel)

    return address, accel, speed, deccel, motor

if __name__ == "__main__":
    #Linux comport name -> for UART3 on the Nuada Pi4, this is ttyAMA1)
    rc = Roboclaw("/dev/ttyAMA1", 115200)
    print(f"port open is {rc.Open()}")

    # these values are defaulted
    accel = 0
    speed = 200
    deccel = 0
    mod_settings = True
    
    # To ensure s4 settings are correct, they are manually 
    # configured in code below
    n=0

    # setting s4 to default: off
    rc.SetPinFunctions(129, 0, 0, 0)
    time.sleep(1)

    # setting s4 to setting motor home (user)
    rc.SetPinFunctions(129, 0, 0x62, 0)
    print("s4 set to 0x62")
    time.sleep(2)

    # check/print that s4 pin was set correctly
    print(rc.ReadPinFunctions(129))
    time.sleep(2)

    # check if in homed state by reading status of all pins 
    # (using ReadError function from roboclaw library) and then
    # doing a bitwise AND with the value desired (in this case 0x400000)
    homed = ((rc.ReadError(129)[1] & 0x400000) == 0x400000)

    homing_length = -460
    rc.SpeedAccelDeccelPositionM1(129,0,80,0,homing_length,1)
    time.sleep(2)

    # This while loop carries out the homing procedure for the claw. The claw 
    # will continue to rotate at a constant speed until the s4 pin is read high 
    # i.e. the Hall-effect sensor is turned on.
    while(homed != True):
        # check pins
        err = rc.ReadError(129)
        print(err)
        print(n)
        n += 1
        # if s4 pin high, exit loop
        if ((err[1] & 0x400000) == 0x400000):
            break
        time.sleep(1)

    time.sleep(1)

    # turn off homing by setting s4 to default: off
    rc.SetPinFunctions(129, 0, 0, 0)
    time.sleep(1)

    # affirm homing turned off
    print("homing setting")
    print(rc.ReadPinFunctions(129))
    time.sleep(1)

    # read position after homing completed
    enc_val = rc.ReadEncM1(129)
    print(f'Encoder value after homing = {enc_val}')
    time.sleep(1)
    
    # Since the claw will be slighly off from the "home position"
    # For motor calibration the following code is run 
    # determine position after homing
    enc_1 = int(enc_val[1])
    time.sleep(1)

    # move in reverse to equivalent pos to homed 0
    if (homed == False):
        rc.SpeedAccelDeccelPositionM1(129,0,100,0,-230-enc_1,1)
        time.sleep(1)
    
    # claw rotation position variable
    rot_pos = 0
    
    #claw open/close variable
    grab_pos = 0

    while (True):
        time.sleep(2)
        # print(f'Encoder pos before change = {rc.ReadEncM1(129)[1]}')
        print(f'claw rotation before change: {rot_pos}')
        print(f'claw grab position before change: {grab_pos}')
        rc.ResetEncoders(129)
        cont = input("new enc val? (n to exit) ")
        if (cont == "n"):
            rc.ResetEncoders(129)
            break
        enc = int(cont)
        
        time.sleep(1)
        rc.SpeedAccelDeccelPositionM1(129,0,200,0,enc,1)
        time.sleep(3)
        
        new_enc = rc.ReadEncM1(129)[1]
        
#         print(f'Encoder pos after change = {new_enc}\n')

        if (new_enc <= 0):
            rot_pos += new_enc
            while (rot_pos < -230):
                rot_pos += 230
            print(f'current claw rotation: {rot_pos}')
            
        elif (new_enc > 0):
            grab_pos += new_enc
            while (grab_pos > 120):
                grab_pos -= 120
                
            print(f'claw grabber position: {grab_pos}')
        





