import time
import math
import threading
import sys

from typing import Union
import numpy as np
# import RPi.GPIO as GPIO
#from smbus2 import SMBus
#import Rovor.src.rovor.jetson as Vision
import rat_control.src.scripts.move_group_interface as mgi

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
    
    def vision_system(self, data_packets, x, y, z, theta):
        # Power up vision system
        # Vision.Jetson.power_on();

        # Start vision system pipeline
        # Vision.Jetson.power_start();

        # Wait 30 seconds for Vision System to Send Data
        time.sleep(30)

        # Power up vision system
        # Vision.Jetson.get_depth_image();

        # Get Data from vision system
        # Vision.Jetson.get_object(data_packets);

        # Stop vision system pipeline
        # Vision.Jetson.power_stop();

        # Power down vision system
        # Vision.Jetson.power_off();

        x = data_packets[0]
        y = data_packets[1]
        z = data_packets[2]
        theta = data_packets[3]
    
    def horizontal_view(self, x, xFlag, z):
        if x != 0:
            xFlag = False

            if x > 0:
                turnDirection = "left"
            elif x < 0:
                turnDirection = "right"
                x = abs(x)
            
            turnAngle = math.asin(x / z)
            # Call function to calculate angle
            # Tank Turn Function:
            # turn(turnDirection, turnAngle)
            
        else:
            xFlag = True
    
    def distance_view(self, z, zFlag):
        if z < 3 or z > 10:
            zFlag = False

            if z < 3:
                # Calculate distance for going forward
                move_distance = 3 - z
            elif z > 10:
                # Calculate distance for going backward (negative)
                move_distance = 10 - z
            
            # Call function to move rover forward or backward
            # move_straight(z)
            
        else:
            zFlag = True
    
    def position_set(self, xFlag, zFlag, y, z, theta):
        if (not xFlag) or (not zFlag):
            # Power back on vision system
            Vision_Communication.power_vision_system()
        
        else:
            # Send y, z, and theta into ROS
            mgi.main_cmd(z, y, theta)