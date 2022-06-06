#!/usr/bin/env python3

import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from vicon_bridge.msg import Markers
from vicon_bridge.msg import Marker
from geometry_msgs.msg import Point


class marker:
    def __init__(self):
        rospy.Subscriber("/vicon/markers", Markers, self.callback, queue_size=1)
        self.pub = rospy.Publisher("point", Twist, queue_size=1)
        rospy.loginfo("Initialized")
        self.P0 = np.array((3, 1))
        self.P = np.array((3, 1))
        self.msgTwist = Twist()
        self.isFirst = True

    def callback(self, data):

        # rospy.loginfo(data.markers[0])
        mark1 = Marker()
        point1 = Point()
        mark1 = data.markers[0]
        point1 = mark1.translation
        x = point1.x
        y = point1.y
        z = point1.z

        self.P = np.array([[x], [y], [z]])
        if self.isFirst:
            self.P0 = np.array([[x], [y], [z]])

        dist = self.P - self.P0
        norm = np.linalg.norm(dist) / 1000.0
        # rospy.loginfo("X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        rospy.loginfo(norm)
        self.publishTwist(norm)
        self.isFirst = False

    def publishTwist(self, x):
        x = self.clip(x, 0.0, 1.0)
        self.msgTwist.linear.x = x
        self.pub.publish(self.msgTwist)
        time.sleep(0.05)

    def clip(self, input, min, max):
        if input > max:
            return max
        if input < min:
            return min
        return input


def main():
    rospy.init_node("marker_reader", anonymous=True)
    obj = marker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
