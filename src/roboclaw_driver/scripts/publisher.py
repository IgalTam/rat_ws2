#!/usr/bin/python3
# license removed for brevity
import rospy
from roboclaw_driver.msg import armCmd

def talker():
    cmd_pub = rospy.Publisher('roboclaw_cmd', armCmd, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        msg = armCmd()
        msg.position_rads = [0, 0, 0, 0]
        msg.speed = [0, 0, 0, 0]
        msg.accel_deccel = [0, 0, 0, 0]

        for i in range(0,4):
            rads= float(input("enter position in radians: "))
            if rads < -6.283184 or rads > 6.283184:
                rospy.loginfo("please enter a valid position in radians (0 to 6.283184...)") 
                continue
            msg.position_rads[i] = rads
        
        rospy.loginfo(msg)
        cmd_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
