import RPi.GPIO as gpio
from roboclaw_3 import Roboclaw
import time

"""non-ROS homing protocol implementation"""

# macros
BASE_HALL = 17
BASE_ADDR = 128
BASE_MOTOR = 2
BASE_FULLROT = 15000

ELBOW_ADDR = 128
ELBOW_MOTOR = 1
ELBOW_FULLROT = 3250

WRIST_ADDR = 129
WRIST_MOTOR = 2
WRIST_FULLROT = 1000

CLAW_ADDR = 129
CLAW_MOTOR = 1
CLAW_FORWARDROT = 60
CLAW_BACKWARDROT = 240

FULLROT = 360

def rotate_to_deg(rc: Roboclaw, motor, address, deg):
    # rotates the motor until it reaches deg or the encoder
    # value stops changing
    if address == 128 and motor == 2:
        """moving base motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * BASE_FULLROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM2(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM2(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM2(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2
    elif address == 128 and motor == 1:
        """moving elbow motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * ELBOW_FULLROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM1(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM1(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM1(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2
    elif address == 129 and motor == 2:
        """moving wrist motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * WRIST_FULLROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM2(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM2(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM2(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2
    elif address == 129 and motor == 1:
        """moving claw motor"""
        # calculate encoder counts from deg
        enc_counts = (deg * CLAW_BACKWARDROT) // FULLROT

        # move motor one encoder count at a time
        temp_val = rc.ReadEncM1(address)[1]
        while(temp_val != enc_counts):
            rc.SpeedAccelDeccelPositionM1(address,0,200,0,1,1)
            temp_val2 = rc.ReadEncM1(address)[1]
            if temp_val2 ==  temp_val:
                break
            temp_val = temp_val2

def adjust_past_base(rc):
    """concluding sequence to home the arm"""
    # move elbow motor to -180 degrees
    rotate_to_deg(rc, ELBOW_MOTOR, ELBOW_ADDR, -180)
    # rotate claw to 0 position
    rotate_to_deg(rc, CLAW_MOTOR, CLAW_ADDR, 0)
    # close claw [pending hardware]
    # ----------code goes here---------------
    # move wrist motor to 90 degrees 
    rotate_to_deg(rc, WRIST_MOTOR, WRIST_ADDR, 90)

    # home is achieved

def main():
    # configure Roboclaws
    rc = Roboclaw("/dev/ttyAMA1", 115200)

    # configure base Hall Effect sensor
    gpio.setmode(gpio.BCM)
    gpio.setwarnings(False)
    hallpin = BASE_HALL
    gpio.setup(hallpin, gpio.IN)

    if gpio.input(hallpin):
        # if the base HE sensor is triggered, then the base motor is already homed
        # in this case move the wrist motor to 180 deg and run adjust_past_base()
        rotate_to_deg(rc, WRIST_MOTOR, WRIST_ADDR, 180)
        adjust_past_base(rc)
    else:
        # if base HE is not triggered, extend wrist and elbow, then rotate base
        # motor until the HE is triggered
        # after, calibrate with adjust_past_base()
        rotate_to_deg(rc, WRIST_MOTOR, WRIST_ADDR, 180)
        rotate_to_deg(rc, ELBOW_MOTOR, ELBOW_ADDR, 180)
        while(not gpio.input(hallpin)):
            rotate_to_deg(rc, BASE_MOTOR, BASE_ADDR, 1)
        adjust_past_base(rc)

if __name__ == "__main__":
    main()

