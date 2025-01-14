#!/usr/bin/python3
import time
from sys import exit
from roboclaw_3 import Roboclaw

#Linux comport name
rc = Roboclaw("/dev/ttyS0", 115200)
rc.Open()
address = 0x80
# 0 just uses default values
accel = 0
speed = 100
deccel = 0
motor = 2
#motor = int(input("Motor 1 or 2: "))
#if motor != 1 and motor != 2:
#        print(f'Bad motor value: {motor}')
#        exit()
#
# speed = int(input("Enter speed (1 to 2^16): "))
# if speed < 1 or speed > 2**16:
#         print(f'Bad speed value: {speed}')
#         exit()



rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,0,1)   
#rc.ResetEncoders(address)
print(f'Addr: {address} Motor: {motor} Speed: {speed}')
while(True):
    print(f'Current Encoder value = {rc.ReadEncM1(address)}')
    val = int(input("Enter encoder counts: "))
    rc.SpeedAccelDeccelPositionM2(address,accel,speed,deccel,val,1)
    #rc.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
    time.sleep(1)
