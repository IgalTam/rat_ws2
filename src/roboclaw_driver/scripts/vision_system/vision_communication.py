import time
import math
import threading
import sys

from typing import Union
#import array as arr
import numpy as np
# import RPi.GPIO as GPIO

#from .rover import *

class Vision_Communication:

    def __init__(self, data_packets, x, y, z, theta, xFlag, angle, zFlag):
        self.data_packets = data_packets
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.xFlag = xFlag
        self.angle = angle
        self.zFlag = zFlag
        
    def parse_data(self, data_packets, x, y, z, theta):
        
        data_packets = [1, 2, 3, 4]

        x = data_packets[0]
        y = data_packets[1]
        z = data_packets[2]
        theta = data_packets[3]
    
    def horizontal_view(self, x, xFlag):
        if x != 0:
            xFlag = False

            if x > 0:
                turn_direction = "left"
            elif x < 0:
                turn_direction = "right"
            
            # turn(turn_direction, angle)
            
        else:
            xFlag = True