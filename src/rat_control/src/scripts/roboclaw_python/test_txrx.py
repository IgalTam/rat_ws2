import time
from sys import exit
from roboclaw_3 import Roboclaw

ROBO_ADDRESS = 129


# program used to test the roboclaw's read and write capabilities
# through the serial port or via proper commands

rc = Roboclaw("/dev/ttyAMA1", 115200)
print("Testing Roboclaw functionallity:\n")
print("Roboclaw address: ", ROBO_ADDRESS)   
print(f"port open is {rc.Open()}")
i = 0
motor = 0
print("Which motor you want to test? \n")
print("1 for M1\n")
print("2 for M2\n")
val = 0
val = input("Enter Motor Number: ")
if(val == 1):
    while True:
        if i == -200:
            i = 0
        else:
            i -= 10
    # rc._sendcommand(129, rc.Cmd.M1SPEEDACCELDECCELPOS)
    # rc._writelong(0)
    # rc._writelong(200)
    # rc._writelong(0)
    # rc._writelong(i)
    # rc._writebyte(1)
    # rc._port.read(1)
    # time.sleep(0.25)
        rc.SpeedAccelDeccelPositionM1(ROBO_ADDRESS, 0, 200, 0, i, 1)
        print(f'Current Encoder value = {rc.ReadEncM2(ROBO_ADDRESS)}')

        time.sleep(1)   # this is required for the RoboClaw to fully process commands
elif(val == 2):
    while True:
        if i == -200:
            i = 0
        else:
            i -= 10
    # rc._sendcommand(129, rc.Cmd.M1SPEEDACCELDECCELPOS)
    # rc._writelong(0)
    # rc._writelong(200)
    # rc._writelong(0)
    # rc._writelong(i)
    # rc._writebyte(1)
    # rc._port.read(1)
    # time.sleep(0.25)
        rc.SpeedAccelDeccelPositionM2(ROBO_ADDRESS, 0, 200, 0, i, 1)
        print(f'Current Encoder value = {rc.ReadEncM2(ROBO_ADDRESS)}')

        time.sleep(1)   # this is required for the RoboClaw to fully process commands