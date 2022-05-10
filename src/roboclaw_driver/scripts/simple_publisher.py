#!/usr/bin/python3
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32MultiArray

def talker():
    pub = rospy.Publisher('motors', Int32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cnts = input("enter position in encoder counts: ")
        arr_msg = Int32MultiArray()
        arr_msg.data = [0, 0, int(cnts)]
        pub.publish(arr_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
