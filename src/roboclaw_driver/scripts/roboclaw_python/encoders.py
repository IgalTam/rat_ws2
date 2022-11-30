#!/usr/bin/python3
import time
from sys import exit
from roboclaw_3 import Roboclaw

#Linux comport name
# rc = Roboclaw("/dev/ttyS0", 115200)
rc = Roboclaw("/dev/ttyAMA1", 115200)
print(f"port open is {rc.Open()}")
address = int(input("Roboclaw Address 128 or 129: "))
# 0 just uses default values
accel = 0
speed = 200
deccel = 0
motor = 2
motor = int(input("Motor 1 or 2: "))
#motor = 1
if motor != 1 and motor != 2:
    print(f"Bad motor value: {motor}")
    #print("bad motor")
    exit()

# speed = int(input("Enter speed (1 to 2^16): "))
# if speed < 1 or speed > 2**16:
#         print(f'Bad speed value: {speed}')
#         exit()



# rc.ResetEncoders(address)
print(f'Addr: {address} Motor: {motor} Speed: {speed}')
while(True):
    if motor == 1:
        print(f'Current Encoder value = {rc.ReadEncM1(address)}')
        val = int(input("Enter encoder counts: "))
        rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,val,1)
    if motor == 2:
        print(f'Current Encoder value = {rc.ReadEncM2(address)}')
        val = int(input("Enter encoder counts: "))
        rc.SpeedAccelDeccelPositionM2(address,accel,speed,deccel,val,1)   
    #rc.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
    time.sleep(1)
