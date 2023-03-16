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
    DEFAULT_Z = -6
    DEFAULT_THETA = 0

    # Preset Coordinates after claw picks up tube
    PRESET_Y1 = 30
    PRESET_Z1 = 0
    PRESET_Y2 = 20
    PRESET_Z2 = 25

    # Coordinate Offsets
    Y_OFF = 7.4
    Z_OFF = 21

    def __init__(self):
        self.xFlag = False
        self.zFlag = False
        self.maxDistanceArm = 30
        self.previousCord = None
        self.tubeFlag = False
        self.looped = False
        self.firstPickUp = False
        self.data = ()
        self.readyFlag = False
        self.dataFlag = False

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
                print(f"data: {data}")

                # if (self.previousCord is not None and self.previousCord == data):
                #     self.tubeFlag = True
                
                self.previousCord = data

                return data
            
    def i2c_image(self):
        """reads and saves image file sent over I2C"""

        data = self.bus.read_file()
    
    def vision_ready(self):
        """Checks to see if the vision system is ready to receive commands."""

        while True:
            data_packets = self.send_i2c_cmd()
            if data_packets == "Ready":
                self.readyFlag = True
                print("\nReady detected, waiting for coordinates.")
                break
            print("\nReady not detected, exiting")        

        # self.bus.write_pkt(b'cord', 'c', 0)
        # print("Ready detected, waiting for coordinates")
    
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
        self.bus.write_pkt(b'cord', 'c', 0)

        data_packets = None
        while data_packets is None:
            data_packets = self.send_i2c_cmd()

        # Test
        # data_packets = "x12y13z15a120"

        x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
        y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')]) - self.Y_OFF
        z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')]) - self.Z_OFF
        theta = int(float(data_packets[data_packets.index('a') + 1: ]))

        self.dataFlag = True

        print(f"\nData: x: {x}, y: {y}, z: {z}, theta: {theta}\n")

        # self.horizontal_view(x, y)
        # self.distance_view(y)
        # self.position_set(y, z, theta)

        return x, y, z, theta
    
    def vision_power_on(self):
        """Power on the vision system."""

        # GPIO17 pin 11 is for controlling power

        # Power up vision system
        # Vision.Jetson.power_on();

        pass

    def vision_power_off(self):
        """Power off the vision system."""

        # GPIO17 pin 11 is for controlling power

        # Power down vision system
        # Vision.Jetson.power_off();

        pass

    def horizontal_view(self, x, y):
        """If x is not zero (x = 0 means arm is directly in front of sample tube),
        have the rover do a tank turn."""

        if x != 0:
            self.xFlag = True
            turnDirection = None

            if x > 0:
                # Turning Left
                turnDirection = 'Left'
            elif x < 0:
                # Turning Right
                turnDirection = 'Right'
                x = abs(x)
            
            turnAngle = math.degrees(math.atan(x / y))

            print(f"Turn {turnAngle} degrees to the {turnDirection}.\n")
            
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

            print(f"Move forward {move_distance} cm.\n")
            
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
        self.mgi.vision_to_moveit(self.PRESET_Y1, self.PRESET_Z1)  # move to coordinate location (270-315 deg. angle of approach)
        self.mgi.vision_to_moveit(self.PRESET_Y2, self.PRESET_Z2)  # move to coordinate location (270-315 deg. angle of approach)
    
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

        movedArm = False

        print ('\n***** Welcome User! *****\n')

        while True:
            print('What command would you like to do with the Vision System?')

            print('*************************************************')

            print('1. Power On Vision System')

            print('2. Power Off Vision System')

            print('3. Check for Ready Command')

            print('4. Take a Picture and Get Data Packets')

            print('5. Tank Turn')

            print('6. Move Forward')

            print('7. Move Arm to Sample Tube')

            print('8. Verify Pickup')

            print('9. Reset Arm')

            print('10. Exit')

            print('*************************************************')
            
            user = input('Enter the command here (number): ')

            if user == '3':
                if self.readyFlag is True:
                    print('\nVision System is already ready.')
                else:
                    print('\nChecking for Ready Command...')
                    self.vision_ready()

            # if user == '4' and self.readyFlag is True:
            if user == '4':
                print('\nTaking a picture and getting data packets...')
                # self.readyFlag = False
                self.xFlag = False
                self.zFlag = False
                self.data = self.vision_system()

            if self.readyFlag is False and self.dataFlag is False:
                print('\nVision System is not ready. Please try again.\n')
            
            if self.dataFlag is True:

                if user == '1':
                    print('\nUnavailable feature at this time.\n')
                    # print('Powering on Vision System...')
                    # self.vision_power_on()
                if user == '2':
                    print('\nUnavailable feature at this time.\n')
                    # print('Powering off Vision System...')
                    # self.vision_power_off()
                if user == '5':
                    if self.data[0] != 0:
                        print('\nTurning rover...')
                        self.horizontal_view(self.data[0], self.data[1])
                        print('Take a picture again.\n')
                    else:
                        print('\nThe rover is already in front of the sample tube.\n')
                if user == '6':
                    if self.data[1] > self.maxDistanceArm:
                        print('\nMoving rover forward...')
                        self.distance_view(self.data[1])
                        print('Take a picture again.\n')
                    else:
                        print('\nThe rover is already close enough to the sample tube.\n')
                if user == '7':
                    if self.xFlag is False and self.zFlag is False:
                        print('\nMoving arm to sample tube...\n')
                        self.position_set(self.data[1], self.data[2], self.data[3])
                        self.mgi.actuate_claw() # Pick up sample tube
                        self.firstPickUp = True
                        movedArm = True
                    else:
                        print('\nDetected the rover attempting to do either a '
                        'tank turn or moving forward. Take another picture.\n')
                if user == '8':
                    if self.firstPickUp is True:
                        print('\nVerifying pickup...\n')
                        self.move_tube()
                        self.send_i2c_cmd()
                        time.sleep(30)
                        self.verify_pickup()
                    else:
                        print('\nThe arm has not tried to pick up a sample tube\n')
                if user == '9':
                    if movedArm is True:
                        print('\nResetting arm...\n')
                        self.reset_arm()
                        movedArm = False
                    else:
                        print('\nThe arm has not moved.\n')
                if user == '10':
                    print('\nExiting...')
                    break
            
            if self.dataFlag is False and self.readyFlag is True:
                print('\nPlease take a picture first.\n')

    def vision_script(self):
        """Vision System Script"""

        done = False
        self.vision_ready()
        
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
                    self.mgi.actuate_claw() # pick up sample tube
                    self.firstPickUp = True

                    a6 = input('\nWould you like to use check if the tube was picked up? '
                            '(Type y or n): ')
                    
                    if a6 == 'y' or a6 == 'Y':
                        print('\nVerifying Pickup...')
                        self.move_tube()
                        self.send_i2c_cmd()
                        time.sleep(60)
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
                # done = True
                break

        if done:
            print('\nResetting Arm Position...')
            
            self.reset_arm()
            
            print("\n***** Logging Off *****")

        else:
            self.vision_interactive()

if __name__ == "__main__":

    vc = VisionCommunication()

    vc.vision_interactive()
