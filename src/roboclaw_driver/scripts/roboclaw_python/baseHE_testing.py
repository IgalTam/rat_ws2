# This file is for testing the hall effect for 
# functionality through the raspberry pi to see if 
# it even works

#!/usr/bin/python3
import time
from sys import exit
from roboclaw_3 import Roboclaw

# set up raspberry pi
import RPi.GPIO as gpio
gpio.setmode(gpio.BCM)
HALLPIN = 2
gpio.setup(HALLPIN, gpio.IN)

while (True):
    if(gpio.input(HALLPIN) == True):
        print("magnet detected")
    else:
        print(" ")
