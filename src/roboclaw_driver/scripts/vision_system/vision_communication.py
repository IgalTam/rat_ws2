import time
import math
import threading
import sys

from typing import Union
#import array as arr
import numpy as np
# import RPi.GPIO as GPIO

#from .rover import Rover

class Vision_Communication:

    def __init__(self, data_packets, x, y, z, theta, xFlag, zFlag):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.data_packets = data_packets
        self.xFlag = xFlag;
        self.zFlag = zFlag;
        
    def parse_data(self, data_packets, x, y, z, theta):
        
        data_packets = [5, 5, 5, 5]

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
            
        else:
            xFlag = True