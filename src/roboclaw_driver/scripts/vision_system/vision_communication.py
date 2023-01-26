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
    
    def __init__(self, x, y, z, theta):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
    
    def parse_data(self, x, y, z, theta):
        a = 5