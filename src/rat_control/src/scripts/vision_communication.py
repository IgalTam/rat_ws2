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

from i2c_bus import *

sys.path.insert(0, "/home/pi/goscout/Rover/src/rover")

# from rover import main as rov
# from .rover import *

class VisionCommunication:

    def __init__(self):
        self.xFlag = False
        self.zFlag = False
        self.maxDistanceArm = 30
        self.mgi = MoveGroupInterface()
        #self.i2cBus = main()
        pass

    def send_i2c_cmd(self):
        """I2C communication with the Vision team's Jetson Nano to the Arm team's
        Raspberry Pi."""

        bus = I2CBus()

        # test writing a command to get cordinates
        bus.write_pkt(b'cord', 'c', 0)

        # wait for response
        while True:
            pkt = bus.wait_response()
            
            if not pkt:
                continue

            if(pkt[I2CPacket.id_index].decode() != bus.pkt_targ_id) or (pkt[I2CPacket.stat_index] != b'd'):
                continue

            # print received data
            data = pkt[I2CPacket.data_index].decode().strip('\0')
            if data:
                print(data)
                return data
    
    def vision_system(self):
        """Main function for the vision system interface. The function takes
        the data packets from the Vision team and then parses the data values
        for x, y, z, and theta. Then the functions needed for the rover's arm
        to successfully move to a desired location are called.
        
        Data Values:
        x           - Horizontal Coordinate (cm)
        y           - Height Coordinate (cm)
        z           - Depth Coordinate (cm)
        theta       - Angle of the test tube (degrees)
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

        print('start of vision system')

        data_packets = self.send_i2c_cmd()

        # Test
        # data_packets = "x12y13z15a120"

        x: float = float(data_packets[data_packets.index('x') + 1: data_packets.index('y')])
        y: float = float(data_packets[data_packets.index('y') + 1: data_packets.index('z')])
        z: float = float(data_packets[data_packets.index('z') + 1: data_packets.index('a')])
        theta: int = int(data_packets[data_packets.index('a') + 1: ])

        print(f"Data:\n {x}, {y}, {z}, {theta}")

        self.horizontal_view(x, z)
        self.distance_view(z)
        self.position_set(y, z, theta)

    def horizontal_view(self, x, z):
        """If x is not zero (x = 0 means arm is directly in front of sample tube),
        have the rover do a tank turn."""

        if x != 0:
            self.xFlag = False

            if x > 0:
                # Turning Left
                pass
            elif x < 0:
                # Turning Right
                x = abs(x)
            
            turnAngle = math.asin(x / z)
            # Call function to calculate angle
            # Tank Turn Function:
            # rov.do_tank_turn(angle)
            
        else:
            self.xFlag = True
    
    def distance_view(self, z):
        """Checks to see if the same tube is too far for the rover's arm to reach. 
        If the rover's arm can't reach, then the rover will move forward."""

        if z > self.maxDistanceArm:
            self.zFlag = False

            # Calculate distance for going forward
            move_distance = ((z - self.maxDistanceArm) + self.maxDistanceArm) / 2
            
            # Call function to move rover forward
            # rov.move_forward(move_distance)
            
        else:
            self.zFlag = True
    
    def position_set(self, y, z, theta):
        """If the rover had to do any tank turns or move forward, turn back on 
        the vision system to obtain new data packets. Otherwise, send y, z, and 
        theta values to ROS."""

        # if (not self.xFlag) or (not self.zFlag):
        #     # Power back on vision system
        #     self.vision_system()
        
        # else:
            # Functionality for interfacing with ROS:
            # Send y, z, and theta into ROS
        self.mgi.actuate_claw()          # open/close claw
        self.mgi.rotate_claw(theta)      # rotate claw
        self.mgi.vision_to_moveit(z, y)  # move to coordinate location (270-315 deg. angle of approach)

        # print(f"data received at end: y {y} z {z} theta {theta}")


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