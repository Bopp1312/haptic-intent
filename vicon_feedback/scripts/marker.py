#!/usr/bin/env python3

import rospy
import numpy as np
import time
from vicon_bridge.msg import Markers
from vicon_bridge.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


class marker:
    def __init__(self):
        # Subscribe to all markers from vicon
        rospy.Subscriber("/vicon/markers", Markers, self.callback, queue_size=1)

        # Create publisher for actuators
        self.actuator1_pub = rospy.Publisher("haptic/actuator_1/position", Vector3, queue_size=1)

        # Create publisher for target
        self.intention_pub = rospy.Publisher("robot/intention/position", Vector3, queue_size=1)

        rospy.loginfo("Initialized")

        self.msgPosition = Vector3()
        self.msgPosition0 = Vector3()

        self.isFirst = True

    def callback(self, data):

        # rospy.loginfo(data.markers[0])
        act1 = Vector3()

        # Marker for actuator 1
        mark1 = Marker()
        mark1 = data.markers[0]
        x = mark1.translation.x
        y = mark1.translation.y
        z = mark1.translation.z

        act1 = self.vector3ToNumpy(mark1.translation)

        if self.isFirst:
            self.P0 = act1

        dist = act1 - self.P0
        norm = np.linalg.norm(dist) / 1000.0

        # rospy.loginfo("X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        rospy.loginfo(norm)
        self.publishActuator1(act1)
        self.publishIntention(intention1)
        self.isFirst = False

    def vector3ToNumpy(self, vec):
        return np.array([[vec.x],[vec.y],[vec.z]])

    def NumpyToVector3(self, array):
        vec = Vector3()
        vec.x = array[0]
        vec.y = array[1]
        vec.z = array[2]
        return vec

    def publishTwist(self, x):
        x = self.clip(x, 0.0, 1.0)
        self.msgTwist.linear.x = x
        self.pub.publish(self.msgTwist)
        time.sleep(0.05)

    def publishActuator(self, vec):
        self.actuator1_pub.publish(vec)

    def publishIntention(self, vec):
        self.intention_pub.publish(vec)

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
