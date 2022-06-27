#!/usr/bin/env python3

import rospy
import numpy as np
import time
import threading
import logging

from vicon_bridge.msg import Markers
from vicon_bridge.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


class marker:
    def __init__(self):
        # Subscribe to all markers from vicon
        rospy.Subscriber("/vicon/markers", Markers, self.callback, queue_size=1)

        # Create publisher for actuators
        self.actuator1_pub = rospy.Publisher(
            "human/actuator_1/position", Vector3, queue_size=1
        )

        # Create publisher for target
        self.intention_pub = rospy.Publisher(
            "robot/intention/position", Vector3, queue_size=1
        )

        rospy.loginfo("Initialized")

        self.msgPosition = Vector3()
        self.msgPosition0 = Vector3()

        # Position of finger
        self.finger = Marker()

        self.isFirst = True

        self.simulate = rospy.get_param("/simulate", False)

        # Allows for local testing by simulating position data over ros
        if self.simulate == True:
            rospy.loginfo("Running in simulate Mode")
            rosRate = rospy.Rate(50)
            while not rospy.is_shutdown():
                finger = Vector3()
                origin = Vector3()
                t = time.time()
                x_pos = 250 + 250 * np.sin(t * 2 * np.pi * 0.15)
                y_pos = 250 * np.cos(t * 2 * np.pi * 0.15)
                finger.x = x_pos
                finger.y = y_pos
                finger.z = np.sqrt(x_pos ** 2 + y_pos ** 2)
                finger_np = self.vector3ToNumpy(finger)
                self.publishActuator(finger_np)
                origin_np = self.vector3ToNumpy(origin)
                self.publishIntention(origin_np)
                rosRate.sleep()

    def callback(self, data):

        # rospy.loginfo(data.markers[0])
        act1 = Vector3()
        intention1 = np.zeros((3, 1))

        # Marker for actuator 1
        mark1 = Marker()

        # Find oring finger marker_reader
        for marker in data.markers:
            if marker.marker_name == "finger11":
                # Update finger marker otherwise use last
                self.finger = marker

        x = self.finger.translation.x
        y = self.finger.translation.y
        z = self.finger.translation.z

        act1 = self.vector3ToNumpy(self.finger.translation)

        if self.isFirst:
            self.P0 = act1

        dist = act1 - intention1
        norm = np.linalg.norm(dist) / 1000.0

        # rospy.loginfo("X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        rospy.loginfo("Distance start: " + str(norm))
        self.publishActuator(act1)
        self.publishIntention(intention1)
        self.isFirst = False

    def vector3ToNumpy(self, vec):
        return np.array([[vec.x], [vec.y], [vec.z]])

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
        msg = Vector3()
        msg.x = vec[0]
        msg.y = vec[1]
        msg.z = vec[2]
        self.actuator1_pub.publish(msg)

    def publishIntention(self, vec):
        msg = Vector3()
        msg.x = vec[0]
        msg.y = vec[1]
        msg.z = vec[2]
        self.intention_pub.publish(msg)

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
