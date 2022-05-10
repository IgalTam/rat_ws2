#!/usr/bin/python3
import time
from roboclaw_3 import Roboclaw

#Linux comport name
rc = Roboclaw("/dev/ttyACM0", 115200)

rc.Open()
addresses = [0x80, 0x81]

rc.ResetEncoders(addresses[0])
rc.ResetEncoders(addresses[1])

print(f'Encoder1 = {rc.ReadEncM1(addresses[0])}')
print(f'Encoder2 = {rc.ReadEncM2(addresses[0])}')
print(f'Encoder3 = {rc.ReadEncM1(addresses[1])}')
print(f'Encoder4 = {rc.ReadEncM2(addresses[1])}')
#     val = int(input("Enter encoder counts: "))
#     rc.SpeedAccelDeccelPositionM1(address,accel,speed,deccel,val,1)
#     #rc.SpeedAccelDeccelPositionM1(0x80,10000,2000,10000,15000,1)
#     time.sleep(1)
