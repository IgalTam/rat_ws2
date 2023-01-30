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

    def __init__(self, data_packets, x, y, z, theta, xFlag, angle, zFlag, setFlag):
        self.data_packets = data_packets
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.xFlag = xFlag
        self.angle = angle
        self.zFlag = zFlag
        self.setFlag = setFlag
        
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
            
            # Call function to calculate angle
            # Tank Turn Function:
            # turn(turn_direction, angle)
            
        else:
            xFlag = True
    
    def distance_view(self, z, zFlag):
        if z < 3 or z > 10:
            zFlag = False

            if z < 3:
                # Calculate distance for going forward
                move_distance = 3 - z;
            elif z > 10:
                # Calculate distance for going backward (negative)
                move_distance = 10 - z;
            
            # Call function to move rover forward or backward
            # move_straight(z)
            
        else:
            zFlag = True
    
    def position_set(self, xFlag, zFlag, setFlag):
        if (not xFlag) or (not zFlag):
            # Power back on vision system
            Vision_Communication.power_vision_system()
        
        else:
            setFlag = True
    
    def power_vision_system(self):
        # Power up vision system
        x = 5
    
    def vision_system_ros(self, setFlag, y, z, theta):
        if setFlag:
            # Send y, z, and theta into ROS
            x = 5