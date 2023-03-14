# This code tests base hall effect
# using the RASPBERRY PI

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

# set up raspberry pi
import RPi.GPIO as gpio
gpio.setmode(gpio.BCM)
HALLPIN = 2
gpio.setup(HALLPIN, gpio.IN)

def main ():

    #Linux comport name -> for UART3 on the Nuada Pi4, this is ttyAMA1)
    rc = Roboclaw("/dev/ttyAMA1", 115200)
    print(f"port open is {rc.Open()}")

    # these values are defaulted
    accel = 0
    speed = 200
    deccel = 0
    mod_settings = True

    val = 0
    while True:
        rc.SpeedAccelDeccelPositionM1(ROBOCLAW_1, accel, speed, deccel, val, 1)
        val -= 10
        time.sleep(1)

        if(gpio.input(HALLPIN) == False):

            print("Base Homed")
            break

if __name__ == "__main__":
    main()
        





