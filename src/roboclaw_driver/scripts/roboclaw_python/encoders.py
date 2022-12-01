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
    print(f'Current Encoder value = {rc.ReadEncM1(address)}')
    print(f"motor is {motor}")
    val = angleToEncoders(address, motor, int(input("Enter angle: ")))
    print(f"val is set to {val}")
    print(f"motor after val is {motor}")
#    val = angleToEncoders(address, motor, val)
    if motor == 1:
        rc.SpeedAccelDeccelPositionM1(address, accel, speed, deccel, val, 1)
    elif motor == 2:
        rc.SpeedAccelDeccelPositionM2(address, accel, speed, deccel, val, 1)

def angleToEncoders(address, motor, angle):
    print(f"motor in angleToEncoders is {motor}")
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

if __name__ == "__main__":
    #Linux comport name
    rc = Roboclaw("/dev/ttyAMA1", 115200)
    print(f"port open is {rc.Open()}")
    address = int(input("Roboclaw Address 128 or 129: "))
    # 0 just uses default values
    accel = 0
    accel = int(input("Acceleration (0 is default): "))
    if not accel:
        accel = 0
    speed = 100
    speed = int(input("Speed/Velocity (100 is default): "))
    if not speed:
        speed = 100
    deccel = 0
    deccel = int(input("deccel (0 is default): "))
    if not deccel:
        deccel = 0
    motor = 2
    motor = int(input("Motor 1 or 2: "))
    if motor != 1 and motor != 2:
            print(f'Bad motor value: {motor}')
            exit()

    # speed = int(input("Enter speed (1 to 2^16): "))
    # if speed < 1 or speed > 2**16:
    #         print(f'Bad speed value: {speed}')
    #         exit()

    #rc.ResetEncoders(address)
    print(f'Addr: {address} Motor: {motor} Speed: {speed}')
    while(True):
        updateMotorPos(address, accel, speed, deccel, motor)
        # if motor == 1:
        #     updateMotorPos(address, accel, speed, decel, motor)
            # print(f'Current Encoder value = {rc.ReadEncM1(address)}')
            # val = int(input("Enter angle: "))
            # val = angleToEncoders(address, 1, val)
            # rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,val,1)
        # if motor == 2:
        #     updateMotorPos(address, accel, speed, decel, motor)
            # print(f'Current Encoder value = {rc.ReadEncM2(address)}')
            # val = int(input("Enter angle: "))
            # val = angleToEncoders(address, 2, val)
            # rc.SpeedAccelDeccelPositionM2(address,accel,speed,deccel,val,1)   
        #rc.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
        time.sleep(1)





