#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64MultiArray


def talker():
    pub = rospy.Publisher('human/doppler', Int64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        msg = Int64MultiArray()
        A = 100000
        B = 150000
        C = 200000
        msg.data = [A, B, C]
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
