import time
from sys import exit
from roboclaw_3 import Roboclaw

rc = Roboclaw("/dev/ttyAMA1", 115200)
print(f"port open is {rc.Open()}")
i = 0
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
    rc.SpeedAccelDeccelPositionM1(129, 0, 200, 0, i, 1)
    time.sleep(1)   # this is required for the RoboClaw to fully process commands