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

    # Default Coordinates to move the arm close to the original position
    DEFAULT_X = 0
    DEFAULT_Y = 5
    DEFAULT_Z = 0
    DEFAULT_THETA = 0

    # Preset Coordinates after claw picks up tube
    PRESET_Y = 5
    PRESET_Z = 25

    def __init__(self):
        self.xFlag = False
        self.zFlag = False
        self.maxDistanceArm = 30
        self.previousCord = None
        self.tubeFlag = False
        self.looped = False
        self.firstPickUp = False

        self.mgi = MoveGroupInterface()
        self.bus = I2CBus()

    def send_i2c_cmd(self):
        """I2C communication with the Vision team's Jetson Nano to the Arm team's
        Raspberry Pi."""

        # Wait for response
        while True:
            pkt = self.bus.wait_response()
            
            if not pkt:
                if self.firstPickUp is True:
                    self.tubeFlag = True
                    self.firstPickUp = False
                continue

            if(pkt[I2CPacket.id_index].decode() != self.bus.pkt_targ_id) or (pkt[I2CPacket.stat_index] != b'd'):
                continue

            # print received data
            data = pkt[I2CPacket.data_index].decode().strip('\0')
            
            if data is not None:
                print(data)

                # if (self.previousCord is not None and self.previousCord == data):
                #     self.tubeFlag = True
                
                self.previousCord = data

                return data
            
    def i2c_image(self):
        """reads and saves image file sent over I2C"""

        data = self.bus.read_file()
    
    def vision_ready(self, data_packets):
        """Checks to see if the vision system is ready to receive commands."""

        while True:
            data_packets = self.send_i2c_cmd()
            if data_packets == "Ready":
                break
            print("Ready not detected, exiting")        

        self.bus.write_pkt(b'cord', 'c', 0)
        print("Ready detected, waiting for coordinates")
    
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
        
        self.vision_ready()

        data_packets = self.send_i2c_cmd()

        # Test
        # data_packets = "x12y13z15a120"

        x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
        y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')]) - 6.9
        z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')]) - 18.2
        theta = int(float(data_packets[data_packets.index('a') + 1: ]))

        print(f"Data:\n {x}, {y}, {z}, {theta}")

        # self.horizontal_view(x, y)
        # self.distance_view(y)
        # self.position_set(y, z, theta)

        return x, y, z, theta
    
    def vision_power_on(self):
        """Power on the vision system."""

        # Power up vision system
        # Vision.Jetson.power_on();

        # Start vision system pipeline
        # Vision.Jetson.power_start();

        pass

    def vision_power_off(self):
        """Power off the vision system."""

        # Stop vision system pipeline
        # Vision.Jetson.power_stop();

        # Power down vision system
        # Vision.Jetson.power_off();

        pass

    def horizontal_view(self, x, y):
        """If x is not zero (x = 0 means arm is directly in front of sample tube),
        have the rover do a tank turn."""

        if x != 0:
            self.xFlag = True

            if x > 0:
                # Turning Left
                turnDirection = 'Left'
            elif x < 0:
                # Turning Right
                turnDirection = 'Right'
                x = abs(x)
            
            turnAngle = math.degrees(math.atan(x / y))

            print(f"Turn {turnAngle} degrees to the {turnDirection}")
            
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
            move_distance = y - self.maxDistanceArm
            move_distance = ((move_distance + y) / 2)

            print(f"Move forward {move_distance} cm")
            
            # Call function to move rover forward
            # rov.move_forward(move_distance)
            
        else:
            self.zFlag = False
    
    def position_set(self, y, z, theta):
        """If the rover had to do any tank turns or move forward, turn back on 
        the vision system to obtain new data packets. Otherwise, send y, z, and 
        theta values to ROS."""

        if self.xFlag or self.zFlag:
            # Power back on vision system
            self.vision_system()
        
        else:
            # Functionality for interfacing with ROS
            # Send y, z, and theta into ROS
            self.mgi.actuate_claw()          # open/close claw
            self.mgi.rotate_claw(theta)      # rotate claw
            self.mgi.vision_to_moveit(y, z)  # move to coordinate location (270-315 deg. angle of approach)
    
    def move_tube(self):
        """Moves the sample tube when in the rover's arm."""

        self.mgi.vision_to_moveit(self.PRESET_Y, self.PRESET_Z)  # move to coordinate location (270-315 deg. angle of approach)
    
    def reset_arm(self):
        """Resets the rover's arm to its original position."""

        self.mgi.rotate_claw(self.DEFAULT_THETA)
        self.mgi.vision_to_moveit(self.DEFAULT_Y, self.DEFAULT_Z)
    
    def verify_pickup(self):
        """Verifies that the rover has picked up the sample tube."""

        # If the vision system can't find the tube in a minute, then there is no tube
        # data_packets = self.vision_system()
        # if data_packets[0] == 1000:
        #     self.tubeFlag = True
        
        if self.tubeFlag is True:
            print("\nSample tube has been picked up.")
            self.tubeFlag = False
        else:
            print("\nSample tube has not been picked up.")

    def vision_interactive(self):
        """Vision System Interface"""

        done = False
        
        while True:
            if not self.looped:
                print ('\n***** Welcome User! *****\n')
                a1 = input('Would you like to use the Vision System? (Type y or n): ')
            else:
                a1 = 'y'

            if a1 == 'y' or a1 == 'Y':
                print('\nUsing Vision System...')
                data_packets = self.vision_system()

                if data_packets[0] != 0:
                    a2 = input('\nThe rover is not facing the sample tube. ' 
                        'Would you like to do a tank turn? (Type y or n): ')
                    
                    if a2 == 'y' or a2 == 'Y':
                        print('\nTurning Rover...')
                        self.horizontal_view(data_packets[0], data_packets[1])
                
                if data_packets[1] > self.maxDistanceArm:
                    a3 = input('\nThe rover is too far away from the sample tube. ' 
                        'Would you like to move the rover forward? (Type y or n): ')
                    
                    if a3 == 'y' or a3 == 'Y':
                        print('\nMoving Rover Forward...')
                        self.distance_view(data_packets[1])
                        
                if self.xFlag or self.zFlag:
                    a4 = input('\nDetected the rover attempting to do either a '
                           'tank turn or moving forward. Would you like to take another picture '
                           'with the vision system? (Type y or n): ')
                    
                    self.xFlag = False
                    self.zFlag = False
                    
                    if a4 == 'y' or a4 == 'Y':
                        self.looped = True
                        done = False
                        break
                
                a5 = input('\nWould you like to move the arm to the desired '
                            'location? (Type y or n): ')
                
                if a5 == 'y' or a5 == 'Y':
                    print('\nMoving Arm...')
                    self.position_set(data_packets[1], data_packets[2], data_packets[3])

                    self.firstPickUp = True

                    a6 = input('\nWould you like to use check if the tube was picked up? '
                            '(Type y or n): ')
                    
                    if a6 == 'y' or a6 == 'Y':
                        print('\nVerifying Pickup...')
                        self.move_tube()
                        self.verify_pickup()
                
                a7 = input('\nWould you like to use the vision system again? '
                            '(Type y or n): ')
                
                if a7 == 'y' or a7 == 'Y':
                    self.looped = True
                    done = False
                else:
                    self.looped = False
                    done = True
                    break

            else:
                done = True
                break

        if (done):
            print('\nResetting Arm Position...')
            
            self.reset_arm()
            
            print("\n***** Logging Off *****")

        else:
            self.vision_interactive()

if __name__ == "__main__":

    vc = VisionCommunication()

    vc.vision_interactive()