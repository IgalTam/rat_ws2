#!/usr/bin/python3
import time
from roboclaw_3 import Roboclaw

#Linux comport name
rc = Roboclaw("/dev/ttyACM0", 115200)

rc.Open()
address = 0x80
accel = 0
speed = 0
deccel = 0

#rc.ResetEncoders(address)
while(True):
    print(f'Current Encoder value = {rc.ReadEncM1(address)}')
    val = int(input("Enter encoder counts: "))
    rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,val,1)
    #rc.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
    time.sleep(1)
