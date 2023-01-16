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

    accel = 0
    speed = 200
    deccel = 0


    mod_settings = True
    
    n=0
    #setting s4 to default: off
    rc.SetPinFunctions(129, 0, 0, 0)
    time.sleep(1)
    #setting s4 to motor home (user)
    rc.SetPinFunctions(129, 0, 0x62, 0)
    print("set_s4")
    time.sleep(2)
    #checking written settings
    print(rc.ReadPinFunctions(129))
    time.sleep(2)
    #run claw backwards to home
    rc.BackwardM1(129, 40)
    time.sleep(2)

    while(True):
        #rc.ResetEncoders(address)
#         if mod_settings:
#             address, accel, speed, deccel, motor = configSettings()
#             mod_settings = False
#         print(f'Addr: {address} Motor: {motor} Speed: {speed} Accel: {accel} Deccel: {deccel}')
#         updateMotorPos(address, accel, speed, deccel, motor)
#         poll_string = input("Change the configurations? [Y \ N]").lower()
#         if poll_string == "y":
#             mod_settings = True
#         time.sleep(1)

        #check pins
        err = rc.ReadError(129)
        print(err)
        print(n)
        n += 1
        #if s4 pin high, exit loop
        if ((err[1] & 0x400000) == 0x400000):
            break
        time.sleep(1)



    time.sleep(1)
    #turn off homing
    rc.SetPinFunctions(129, 0, 0, 0)
    time.sleep(1)
    #affirm homing turned off
    print(rc.ReadPinFunctions(129))
    time.sleep(1)
    #read position after homing completed
    enc_val = rc.ReadEncM1(129)
    print(f'Encoder value after homing= {enc_val[1]}')
    time.sleep(1)
    
    
    # determine position after homing
    enc_1 = int(enc_val[1])
    time.sleep(1)
    #move in reverse to equivalent pos to homed 0
    rc.SpeedAccelDeccelPositionM1(129,0,200,0,-240-enc_1,1)
    time.sleep(1)
    


    #claw rotation position variable
    rot_pos = 0
    
    #claw open/close variable
    grab_pos = 0
    

    while (True):
        time.sleep(2)
#         print(f'Encoder pos before change = {rc.ReadEncM1(129)[1]}')
        print(f'claw rotation before change: {rot_pos}')
        print(f'claw grab position before change: {grab_pos}')
        rc.ResetEncoders(129)
        cont = input("new enc val? ")
        enc = int(cont)
        
        time.sleep(1)
        rc.SpeedAccelDeccelPositionM1(129,0,200,0,enc,1)
        time.sleep(3)
        
        new_enc = rc.ReadEncM1(129)[1]
        
#         print(f'Encoder pos after change = {new_enc}\n')

        if (new_enc <= 0):
            rot_pos += new_enc
            while (rot_pos < -240):
                rot_pos += 240
            print(f'current claw rotation: {rot_pos}')
            
        elif (new_enc > 0):
            grab_pos += new_enc
            while (grab_pos > 120):
                grab_pos -= 120
                
            print(f'claw grabber position: {grab_pos}')
        
        


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





