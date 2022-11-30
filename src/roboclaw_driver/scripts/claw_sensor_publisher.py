#!/usr/bin/python3
# license removed for brevity
import rospy
from roboclaw_driver.msg import hallSense
import RPi.GPIO as gpio

def talker():
    cmd_pub = rospy.Publisher('hallsensor_cmd', hallSense, queue_size=10)
    rospy.init_node('hallsense', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    # initialize hall effect sensor
    gpio.setmode(gpio.BCM)
    gpio.setwarnings(False)
    hallpin = 17
    gpio.setup(hallpin, gpio.IN)

    while not rospy.is_shutdown():
        msg = hallSense()
        test_val = gpio.input(hallpin)
        if not test_val:
            msg.hallHigh = "Magnet detected"
        else:
            msg.hallHigh = "Magnet not detected"
        msg.hallVal = test_val
        
        rospy.loginfo(msg)
        cmd_pub.publish(msg)
        rate.sleep()
    
    gpio.cleanup()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

