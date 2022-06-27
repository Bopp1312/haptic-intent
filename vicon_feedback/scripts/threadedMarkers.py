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
from std_msgs.msg import Float32MultiArray


class marker:
    def __init__(self):
        # Subscribe to all markers from vicon
        rospy.Subscriber("/vicon/finger/finger", Markers, self.callback, queue_size=1)

        self.actuator_pub = rospy.Publisher("human/actuator", Float32MultiArray, queue_size=1)
        self.debug_sensor_pub = rospy.Publisher("debug/sensor/position", Vector3, queue_size=1)

        rospy.loginfo("Initialized")

        self.rosRate = rospy.Rate(120)

        self.sensor = np.zeros((3, 1))
        self.goal = np.zeros((3, 1))

        # Position of finger
        self.finger = Marker()

        self.isFirst = True

        self.simulate = rospy.get_param("/simulate", False)

        if self.simulate == False:
            self.thread = threading.Thread(target=self.loop)
            self.thread.start()

        # Allows for local testing by simulating position data over ros
        if self.simulate == True:
            rospy.loginfo("Running in simulate Mode")
            while not rospy.is_shutdown():
                finger = np.zeros((3, 1))
                origin = np.zeros((3, 1))
                t = time.time()
                x_pos = 250 + 250 * np.sin(t * 2 * np.pi * 0.10)
                y_pos = 250 * np.cos(t * 2 * np.pi * 0.1)
                finger[0] = x_pos
                finger[1] = y_pos
                finger[2] = np.sqrt(x_pos ** 2 + y_pos ** 2)
                self.sensor = finger
                self.update_actuator()
                self.update_debug()
                self.rosRate.sleep()

    def update_debug(self):
        msg = Vector3()
        msg.x = self.sensor[0]
        msg.y = self.sensor[1]

        self.debug_sensor_pub.publish(msg)

    def loop(self):
        while rospy.is_shutdown() == False:
            self.update_actuator()
            self.rosRate.sleep()
            # rospy.loginfo("Here")

    def callback(self, data):
        # Find oring finger marker_reader
        for marker in data.markers:
            if marker.marker_name == "finger11":
                # Update finger marker otherwise use last
                self.finger = marker

        self.sensor = self.vector3ToNumpy(self.finger.translation)

        if self.isFirst:
            self.P0 = self.sensor

        norm = np.linalg.norm(self.goal - self.sensor)
        # rospy.loginfo("X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        rospy.loginfo("Distance start: " + str(norm))
        self.isFirst = False

    def vector3ToNumpy(self, vec):
        return np.array([[vec.x], [vec.y], [vec.z]])

    def NumpyToVector3(self, array):
        vec = Vector3()
        vec.x = array[0]
        vec.y = array[1]
        vec.z = array[2]
        return vec

    def update_actuator(self):
        # Any validation or casting happens here
        #self.texture_hot_cold()
        self.cartesian_detents(10)
        msg = Float32MultiArray()
        msg.data = self.actuator_states 
        self.actuator_pub.publish(msg)

    def clip(self, input, min, max):
        if input > max:
            return max
        if input < min:
            return min
        return input

    def texture_hot_cold(self):
        # This texture starts off with a intensity at mid range 
        # gets more intense the closer to the goal it is
        max_radius = 200 #mm
        frequency = 60 #hz

        zeta = 2/(max_radius**2)

        mag_error = np.absolute(np.linalg.norm(self.goal - self.sensor))
        if mag_error > max_radius:
            mag_error = max_radius        
        # Scale distance to 0 and 127
        intensity = (1.0-(mag_error/max_radius))*127
        intensity =(1.0-0.5*zeta*(mag_error)**2)*127

        self.actuator_states = [intensity,frequency]

    def cartesian_detents(self, ticks):
        frequency = 60 #Hz
        distance_thresh = 5 #mm
        intensity = 0
        grid_space = 100
        detent = Falsed
        remainder = np.mod(self.sensor,grid_space)
        for axis in remainder:
            
            if (np.abs(axis) < distance_thresh) or (np.abs(axis - grid_space) < distance_thresh):
                detent = True
                break

        if detent:
            intensity = 60

        self.actuator_states = [intensity, frequency]

def main():
    rospy.init_node("marker_reader", anonymous=True)
    obj = marker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
