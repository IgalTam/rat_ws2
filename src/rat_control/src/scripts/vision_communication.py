#!/usr/bin/python3


import time
import math
import threading
import sys

from typing import Union
import numpy as np

# import Rovor.src.rovor.jetson as Visioncle
from move_group_interface import MoveGroupInterface

from i2c_bus import *

sys.path.insert(0, "/home/pi/goscout/Rover/src/rover")

# from rover import main as rov
# from .rover import *

class VisionCommunication:
    """Communication and Commands for Arm-Vision Rover"""

    def __init__(self):
        self.xFlag = False
        self.zFlag = False
        self.maxDistanceArm = 30

        self.mgi = MoveGroupInterface()
        self.bus = I2CBus()

        # self.i2cBus = main()
        pass

    def send_i2c_cmd(self):
        """I2C communication with the Vision team's Jetson Nano to the Arm team's
        Raspberry Pi."""

        # bus = I2CBus()

        # test writing a command to get cordinates
        # bus.write_pkt(b'cord', 'c', 0)

        # wait for response
        while True:
            pkt = self.bus.wait_response()
            
            if not pkt:
                continue

            if(pkt[I2CPacket.id_index].decode() != self.bus.pkt_targ_id) or (pkt[I2CPacket.stat_index] != b'd'):
                continue

            # print received data
            data = pkt[I2CPacket.data_index].decode().strip('\0')
            if data is not None:
                print(data)
                return data
            
    def i2c_image(self):
        """reads and saves image file sent over I2C"""
        # bus = I2CBus()
        data = self.bus.read_file()
    
    def vision_system(self):
        """Main function for the vision system interface. The function takes
        the data packets from the Vision team and then parses the data values
        for x, y, z, and theta. Then the functions needed for the rover's arm
        to successfully move to a desired location are called.
        
        Data Values:
        - x -         Horizontal Coordinate (cm)
        - y -         Depth Coordinate (cm)
        - z -         Height Coordinate (cm)
        - theta -     Angle of the test tube (degrees)
        """

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

        
        while True:
            data_packets = self.send_i2c_cmd()
            if data_packets == "Ready":
                break
            print("Ready not detected, exiting")

        self.bus.write_pkt(b'cord', 'c', 0)
        print("Ready detected, waiting for coordinates")
        data_packets = self.send_i2c_cmd()

        # Test
        # data_packets = "x12y13z15a120"

        x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
        y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')]) - 6.9
        z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')]) - 18.2
        theta = int(float(data_packets[data_packets.index('a') + 1: ]))

        print(f"Data:\n {x}, {y}, {z}, {theta}")

        self.horizontal_view(x, y)
        self.distance_view(y)
        self.position_set(y, z, theta)

        # return x, y, z, theta

    def horizontal_view(self, x, y):
        """If x is not zero (x = 0 means arm is directly in front of sample tube),
        have the rover do a tank turn."""

        if x != 0:
            self.xFlag = True

            if x > 0:
                # Turning Left
                pass
            elif x < 0:
                # Turning Right
                x = abs(x)
            
            turnAngle = math.asin(x / y)
            # Call function to calculate angle
            # Tank Turn Function:
            # rov.do_tank_turn(angle)
            
        else:
            self.xFlag = False
    
    def distance_view(self, y):
        """Checks to see if the same tube is too far for the rover's arm to reach. 
        If the rover's arm can't reach, then the rover will move forward."""

        if y > self.maxDistanceArm:
            self.zFlag = True

            # Calculate distance for going forward
            move_distance = ((y - self.maxDistanceArm) + self.maxDistanceArm) / 2
            
            # Call function to move rover forward
            # rov.move_forward(move_distance)
            
        else:
            self.zFlag = False
    
    def position_set(self, y, z, theta):
        """If the rover had to do any tank turns or move forward, turn back on 
        the vision system to obtain new data packets. Otherwise, send y, z, and 
        theta values to ROS."""

        # if (self.xFlag) or (self.zFlag):
            # Power back on vision system
            # self.vision_system()
        
        # else:
            # Functionality for interfacing with ROS:
            # Send y, z, and theta into ROS
        self.mgi.actuate_claw()          # open/close claw
        self.mgi.rotate_claw(theta)      # rotate claw
        self.mgi.vision_to_moveit(y, z)  # move to coordinate location (270-315 deg. angle of approach)

        # print(f"data received at end: y {y} z {z} theta {theta}")

    def main(self):
        """Vision System Interface"""

        vc = VisionCommunication()
        command = True

        print ('\n***** Welcome User! *****\n')
        
        while(command == True):
            a1 = input('Would you like to turn on the Vision System? (Type y or n): ')

            if (a1 == 'y' or a1 == 'Y'):
                print('\nTurning on Vision System...')
                data_packets = vc.vision_system()

                if (data_packets[0] != 0):
                    a2 = input('\nThe rover is not facing the sample tube. ' 
                        'Would you like to do a tank turn? (Type y or n): ')
                    
                    if (a2 == 'y' or a2 == 'Y'):
                        print('\nTurning Rover...')
                        self.horizontal_view(data_packets[0], data_packets[1])
                
                if (data_packets[2] > self.maxDistanceArm):
                    a3 = input('\nThe rover is too far away from the sample tube. ' 
                        'Would you like to move the rover forward? (Type y or n): ')
                    
                    if (a3 == 'y' or a3 == 'Y'):
                        print('\nMoving Rover Forward...')
                        self.distance_view(data_packets[1])
                        
                if (self.xFlag) or (self.zFlag):
                    a4 = input('\nDetected the rover attempting to do either a '
                           'tank turn or moving forward. Would you like to take another picture '
                           'with the vision system? (Type y or n): ')
                    
                    self.xFlag = False
                    self.zFlag = False
                    
                    if (a4 != 'y' or a4 != 'Y'):
                        print('help\n')
                
                else:
                    a5 = input('\nWould you like to move the arm to the desired '
                               'location? (Type y or n): ')
                    
                    if (a5 == 'y' or a5 == 'Y'):
                        print('\nMoving Arm...')
                        self.position_set(data_packets[2], data_packets[1], data_packets[3])
                    
                    a6 = input('\nWould you like to use the vision system again? '
                               '(Type y or n): ')
                    
                    if (a6 != 'y' or a6 != 'Y'):
                        command = False
            else:
                command = False
                
        print("\n***** Logging Off *****")

if __name__ == "__main__":

    vc = VisionCommunication()
    vc.vision_system()
    
    # vc.main()

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