#!/usr/bin/python3

import time
import math
import sys

from typing import Union
import numpy as np

from move_group_interface import MoveGroupInterface

from i2c_bus import *

sys.path.insert(0, "/home/pi/goscout/Rover/src/rover")

# from rover import testbench as rov

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

    # Coordinate Offsets from camera to arm
    Y_OFF = 9
    Z_OFF = 21

    def __init__(self):
        self.xFlag = False
        self.yFlag = False
        self.maxDistanceArm = 30
        self.tubeFlag = False
        self.looped = False
        self.firstPickUp = False
        self.movedArm = False
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
                continue

            if(pkt[I2CPacket.id_index].decode() != self.bus.pkt_targ_id) or (pkt[I2CPacket.stat_index] != b'd'):
                continue

            # print received data
            data = pkt[I2CPacket.data_index].decode().strip('\0')
            
            if data != 'none':
                print(f"data: {data}")

                # Return coordinates
                return data
            else:
                # For when the vision system has not picked up a tube yet
                self.tubeFlag = True
            
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

    def vision_none(self):
        self.bus.write_pkt(b'cord', 'c', 0)

        data_packets = None
        while data_packets is None:
            data_packets = self.send_i2c_cmd()
        
        return data_packets
    
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

        # Parse coordinate values
        x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
        y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')]) - self.Y_OFF
        z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')]) - self.Z_OFF
        theta = int(float(data_packets[data_packets.index('a') + 1: ]))

        # Flag for when the vision system has picked up a tube
        self.dataFlag = True

        print(f"\nData: x: {x}, y: {y}, z: {z}, theta: {theta}\n")

        return x, y, z, theta
    
    def vision_power_on(self):
        """Power on the vision system."""

        print('Powering on Vision System...')

        # GPIO17 Pin 11 is for controlling power for the Jetson Nono

    def vision_power_off(self):
        """Power off the vision system."""

        print('Powering off Vision System...')

        # GPIO17 Pin 11 is for controlling power to the Jetson Nano

    def horizontal_view(self, x, y):
        """If x is not zero (x = 0 means arm is directly in front of sample tube),
        have the rover do a tank turn."""

        if x != 0:
            print('\nTurning rover...')

            # Flag is true when the rover is turning
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
            
            # Tank Turn Function:
            # rov.do_tank_turn(angle)

            print('Take a picture again.\n')
            
        else:
            print('\nThe rover is already in front of the sample tube.\n')
            self.xFlag = False
    
    def distance_view(self, y):
        """Checks to see if the same tube is too far for the rover's arm to reach. 
        If the rover's arm can't reach, then the rover will move forward."""

        if y > self.maxDistanceArm:
            print('\nMoving rover forward...')

            # Flag is true when the rover is moving forward
            self.yFlag = True

            # Calculate distance for going forward
            move_distance = y - self.maxDistanceArm
            move_distance = ((move_distance + y) / 2)

            print(f"Move forward {move_distance} cm.\n")
            
            # Call function to move rover forward
            # rov.move_forward(move_distance)

            print('Take a picture again.\n')
            
        else:
            print('\nThe rover is already close enough to the sample tube.\n')
            self.yFlag = False
    
    def position_set(self, y, z, theta):
        """If the rover had to do any tank turns or move forward, turn back on 
        the vision system to obtain new data packets. Otherwise, send y, z, and 
        theta values to ROS."""

        if self.xFlag is False and self.yFlag is False:
            print('\nMoving arm to sample tube...\n')

            # Functionality for interfacing with ROS
            # Send y, z, and theta into ROS
            self.mgi.actuate_claw()             # open/close claw
            self.mgi.rotate_claw(theta)         # rotate claw
            self.mgi.vision_to_moveit(y, z)     # move to coordinate location (270-315 deg. angle of approach)
            self.mgi.actuate_claw()             # open/close claw

            self.firstPickUp = True
            self.movedArm = True
        else:
            print('\nDetected the rover attempting to do either a tank turn or moving forward. Take another picture.\n')
    
    def move_tube(self):
        """Moves the sample tube when in the rover's arm."""

        self.mgi.vision_to_moveit(self.PRESET_Y1, self.PRESET_Z1)   # move to coordinate location (270-315 deg. angle of approach)
        self.mgi.vision_to_moveit(self.PRESET_Y2, self.PRESET_Z2)   # move to coordinate location (270-315 deg. angle of approach)
    
    def reset_arm(self):
        """Resets the rover's arm to its original position."""

        if self.movedArm is True:
            print('\nResetting arm...\n')

            self.mgi.rotate_claw(self.DEFAULT_THETA)
            self.mgi.vision_to_moveit(self.DEFAULT_Y, self.DEFAULT_Z)
            self.movedArm = False

        else:
            print('\nThe arm has not moved.\n')
    
    def verify_pickup(self):
        """Verifies that the rover has picked up the sample tube."""

        if self.firstPickUp is True:
            print('\nVerifying pickup...\n')
            self.move_tube()
            self.send_i2c_cmd()
            time.sleep(30)

            # If the vision system can't find the tube in a minute, then there is no tube
            if self.tubeFlag is True:
                print("\nSample tube has been picked up.")
                self.tubeFlag = False
            else:
                print("\nSample tube has not been picked up.")

        else:
            print('\nThe arm has not tried to pick up a sample tube\n')

    def vision_interactive(self):
        """Vision System Interface"""

        print ('\n***** Welcome User! *****\n')

        while True:
            print('What command would you like to do with the Vision System?')

            print('**********************************************************')

            print('1. Power On Vision System')

            print('2. Power Off Vision System')

            print('3. Check for Ready Command')

            print('4. Take a Picture and Get Coordinates')

            print('5. Tank Turn')

            print('6. Move Forward')

            print('7. Move Arm to Sample Tube')

            print('8. Verify Pickup')

            print('9. Reset Arm')

            print('10. Exit')

            print('**********************************************************')
            
            user = input('Enter the command here (number): ')

            if user == '3':
                if self.readyFlag is True:
                    print('\nVision System is already ready.')
                else:
                    print('\nChecking for Ready Command...')
                    self.vision_ready()

            if user == '4':         # and self.readyFlag is True <-- Does not work anymore since Jetson Nano has a boot file now
                print('\nTaking a picture and getting data packets...')
                self.xFlag = False
                self.yFlag = False
                self.data = self.vision_system()

            if self.readyFlag is False and self.dataFlag is False:
                print('\nVision System is not ready. Please try again.\n')
            
            if self.dataFlag is True:

                # Python 3.7 doesn't support switch statements, so had to use if/elif statements

                if user == '1':
                    print('\nUnavailable feature at this time.\n')
                    # self.vision_power_on()
                elif user == '2':
                    print('\nUnavailable feature at this time.\n')
                    # self.vision_power_off()
                elif user == '5':
                    self.horizontal_view(self.data[0], self.data[1])
                elif user == '6':
                    self.distance_view(self.data[1])
                elif user == '7':
                    self.position_set(self.data[1], self.data[2], self.data[3])
                elif user == '8':
                    self.verify_pickup()
                elif user == '9':
                    self.reset_arm()
                elif user == '10':
                    print('\nExiting...')
                    break
            
            if self.dataFlag is False and self.readyFlag is True:
                print('\nPlease take a picture first.\n')

if __name__ == "__main__":

    vc = VisionCommunication()

    vc.vision_interactive()
