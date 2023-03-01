import time
import math
import threading
import sys

from typing import Union
import numpy as np
# import RPi.GPIO as GPIO
# from smbus2 import SMBus
# import Rovor.src.rovor.jetson as Visioncle
from move_group_interface import MoveGroupInterface

#from .rover import *

class VisionCommunication:

    def __init__(self):
        self.xFlag = False
        self.zFlag = False
        self.maxDistanceArm = 30
        self.mgi = MoveGroupInterface()
        pass
    
    def vision_system(self):
        # Power up vision system
        # Vision.Jetson.power_on();

        # Start vision system pipeline
        # Vision.Jetson.power_start();

        # Wait 30 seconds for Vision System to Send Data
        # time.sleep(30)

        # Power up vision system
        # Vision.Jetson.get_depth_image();

        # Get Data from vision system
        # Vision.Jetson.get_object(data_packets);

        # Stop vision system pipeline
        # Vision.Jetson.power_stop();

        # Power down vision system
        # Vision.Jetson.power_off();

        # Test
        data_packets = "x12y13z15a120"

        x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
        y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')])
        z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')])
        theta: int = int(data_packets[data_packets.index('a') + 1: ])

        print(f"Data:\n {x}, {y}, {z}, {theta}")

        self.horizontal_view(x, z)
        self.distance_view(z)
        self.position_set(y, z, theta)

    def horizontal_view(self, x, z):
        if x != 0:
            self.xFlag = False

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
            self.xFlag = True
    
    def distance_view(self, z):
        if z > self.maxDistanceArm:
            self.zFlag = False

            # Calculate distance for going forward
            move_distance = ((z - self.maxDistanceArm) + self.maxDistanceArm) / 2
            
            # Call function to move rover forward
            # move_straight(z)
            
        else:
            self.zFlag = True
    
    def position_set(self, y, z, theta):
        if (not self.xFlag) or (not self.zFlag):
            # Power back on vision system
            self.vision_system()
        
        else:
            # Functionality for interfacing with ROS:
            # Send y, z, and theta into ROS
            self.mgi.actuate_claw()          # open/close claw
            self.mgi.rotate_claw(theta)      # rotate claw
            self.mgi.vision_to_moveit(z, y)  # move to coordinate location (270-315 deg. angle of approach)


if __name__ == "__main__":
    vc = VisionCommunication()
    vc.vision_system()
    # zflag = None
    # vc = VisionCommunication()
    # vc.distance_view(0, zflag)
    # print(zflag)

    # data_packets = "x12.5y13z15a120"

    # x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
    # y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')])
    # z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')])
    # theta: int = int(data_packets[data_packets.index('a') + 1: ])

    # print(f"Data:\n {x}, {y}, {z}, {theta}")