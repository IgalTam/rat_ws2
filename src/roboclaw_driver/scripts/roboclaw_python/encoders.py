#!/usr/bin/python3
import time
from sys import exit
from sys import argv
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
        if (motor == 2):
            return (angle * ELBOW_FULLROT) // FULLROT
        elif (motor == 1):
            return (angle * BASE_FULLROT) // FULLROT
        elif (motor == 2):
            return (angle * ELBOW_FULLROT) // FULLROT
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
        motor = 1
    elif motor_query == 'e':
        address = 128
        motor = 2
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

def nuada_demo(accel, speed, deccel):
    """arm demo sequence"""

    # Move to Nuada Position
    address = 0x80
    motor = 1
    val = angleToEncoders(address, motor, -90)  # get angle for elbow
    rc.SpeedAccelDeccelPositionM1(address, accel, speed, deccel, val, 1)
    time.sleep(2)
    speed = 300
    address = 0x80
    motor = 2
    val = angleToEncoders(address, motor, 45)   # get angle for base
    rc.SpeedAccelDeccelPositionM2(address, accel, speed, deccel, val, 1)
    time.sleep(5)
    speed = 200
    address = 0x81
    motor = 2
    val = angleToEncoders(address, motor, 90)   # get angle for wrist
    rc.SpeedAccelDeccelPositionM2(address, accel, speed, deccel, val, 1)
    time.sleep(2)
    address = 0x81
    motor = 1
    val = 120
    rc.SpeedAccelDeccelPositionM1(address, accel, speed, deccel, val, 1)    # actuate claw
    time.sleep(5)

    #move to home position
    address = 0x81
    motor = 2
    val = angleToEncoders(address, motor, 0)
    rc.SpeedAccelDeccelPositionM2(address, accel, speed, deccel, val, 1)
    time.sleep(3)
    speed = 300
    address = 0x80
    motor = 2
    val = angleToEncoders(address, motor, 0)
    rc.SpeedAccelDeccelPositionM2(address, accel, speed, deccel, val, 1)
    time.sleep(4)
    speed = 200
    address = 0x80
    motor = 1
    val = angleToEncoders(address, motor, 0)
    rc.SpeedAccelDeccelPositionM1(address, accel, speed, deccel, val, 1)
    time.sleep(3)
    address = 0x81
    motor = 1
    val = 0
    rc.SpeedAccelDeccelPositionM1(address, accel, speed, deccel, val, 1)
    # time.sleep(5)

    print("demo completed\n")

if __name__ == "__main__":
    #Linux comport name -> for UART3 on the Nuada Pi4, this is ttyAMA1)
#    rc = Roboclaw("/dev/ttyAMA1", 115200)
    rc = Roboclaw("/dev/ttyS0", 115200)
    print(f"port open is {rc.Open()}")

    accel = 0
    speed = 200
    deccel = 0

    if len(argv) > 1 and argv[1] == 'demo':
        nuada_demo(accel, speed, deccel)

    mod_settings = True

    while(True):
        #rc.ResetEncoders(address)
        if mod_settings:
            address, accel, speed, deccel, motor = configSettings()
            mod_settings = False
        print(f'Addr: {address} Motor: {motor} Speed: {speed} Accel: {accel} Deccel: {deccel}')
        updateMotorPos(address, accel, speed, deccel, motor)
        poll_string = input("Change the configurations? [Y \ N]").lower()
        if poll_string == "y":
            mod_settings = True
        time.sleep(1)



    # address = int(input("Roboclaw Address 128 or 129: "))
    # # 0 just uses default values
    # accel = 0
    # accel = int(input("Acceleration (0 is default): "))
    # if not accel:
    #     accel = 0
    # speed = 100
    # speed = int(input("Speed/Velocity (100 is default): "))
    # if not speed:
    #     speed = 100
    # deccel = 0
    # deccel = int(input("deccel (0 is default): "))
    # if not deccel:
    #     deccel = 0
    # motor = 2
    # motor = int(input("Motor 1 or 2: "))
    # if motor != 1 and motor != 2:
    #         print(f'Bad motor value: {motor}')
    #         exit()

    # # speed = int(input("Enter speed (1 to 2^16): "))
    # # if speed < 1 or speed > 2**16:
    # #         print(f'Bad speed value: {speed}')
    # #         exit()

    # #rc.ResetEncoders(address)
    # print(f'Addr: {address} Motor: {motor} Speed: {speed}')
    # while(True):
    #     updateMotorPos(address, accel, speed, deccel, motor)
    #     # if motor == 1:
    #     #     updateMotorPos(address, accel, speed, decel, motor)
    #         # print(f'Current Encoder value = {rc.ReadEncM1(address)}')
    #         # val = int(input("Enter angle: "))
    #         # val = angleToEncoders(address, 1, val)
    #         # rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,val,1)
    #     # if motor == 2:
    #     #     updateMotorPos(address, accel, speed, decel, motor)
    #         # print(f'Current Encoder value = {rc.ReadEncM2(address)}')
    #         # val = int(input("Enter angle: "))
    #         # val = angleToEncoders(address, 2, val)
    #         # rc.SpeedAccelDeccelPositionM2(address,accel,speed,deccel,val,1)   
    #     #rc.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
    #     time.sleep(1)





