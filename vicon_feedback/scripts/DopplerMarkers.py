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
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int64MultiArray

class marker:
    def __init__(self):
        # Subscribe to all markers from vicon
        rospy.Subscriber("/vicon/finger/1", TransformStamped, self.sensor_cb_1, queue_size=1)
        rospy.Subscriber("/vicon/finger/2", TransformStamped, self.sensor_cb_2, queue_size=1)
        rospy.Subscriber("/vicon/finger/3", TransformStamped, self.sensor_cb_3, queue_size=1)

        rospy.Subscriber("/vicon/target", TransformedStamped, self.target_cb, queue_size=1)

        self.actuator_pub = rospy.Publisher("human/actuator", Float32MultiArray, queue_size=1)
        self.doppler_pub = rospy.Publisher("human/doppler", Int64MultiArray, queue_size=1)

        self.debug_sensor_pub = rospy.Publisher("debug/sensor/position", Vector3, queue_size=1)

        rospy.loginfo("Initialized")

        self.rosRate = rospy.Rate(60)

        self.sensor_1 = np.zeros((3, 1))
        self.sensor_2 = np.zeros((3, 1))
        self.sensor_3 = np.zeros((3, 1))

        # Initialize goal
        self.goal = np.zeros((3, 1))
        self.goal[0] = 0.2
        self.goal[1] = 0.04

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
                x_pos = 0.250 + 0.250 * np.sin(t * 2 * np.pi * 0.10)
                y_pos = 0.250 * np.cos(t * 2 * np.pi * 0.1)
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
        self.sensor[0] = data.transform.translation.x
        self.sensor[1] = data.transform.translation.y
        self.sensor[2] = data.transform.translation.z
        #print(self.sensor)

        #self.sensor = self.vector3ToNumpy(data.transform.translation)

        if self.isFirst:
            self.P0 = self.sensor

        norm = np.linalg.norm(self.goal - self.sensor)
        # rospy.loginfo("X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
        rospy.loginfo("Distance start: " + str(norm))
        self.isFirst = False

    def target_cb(self, data):
       self.goal[0] = data.transform.translation.x
       self.goal[1] = data.transform.translation.y
       self.goal[2] = data.transform.translation.z 

    def sensor_cb_1(self, data):
        self.sensor_1[0] = data.transform.translation.x
        self.sensor_1[1] = data.transform.translation.y
        self.sensor_1[2] = data.transform.translation.z

    def sensor_cb_2(self, data):
        self.sensor_2[0] = data.transform.translation.x
        self.sensor_2[1] = data.transform.translation.y
        self.sensor_2[2] = data.transform.translation.z

    def sensor_cb_3(self, data):
        self.sensor_3[0] = data.transform.translation.x
        self.sensor_3[1] = data.transform.translation.y
        self.sensor_3[2] = data.transform.translation.z

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
        #self.binary_hot_cold()
        #self.cartesian_detents(10)
        #self.periodic_detents()
        self.doppler()
        #msg = Float32MultiArray()
        #msg.data = self.actuator_states 
        #self.actuator_pub.publish(msg)
    
    def doppler(self):
        wave_velocity = 100 # m/s

        # Calculate distance to target
        dist_1 = np.linalg.norm(self.goal - self.sensor_1)
        dist_2 = np.linalg.norm(self.goal - self.sensor_2)
        dist_3 = np.linalg.norm(self.goal - self.sensor_3)

        # Calculate time to target for wave
        time_1 = dist_1/wave_velocity
        time_2 = dist_2/wave_velocity
        time_3 = dist_3/wave_velocity

        time_us_1 = int(time_1*10**6)
        time_us_2 = int(time_2*10**6)
        time_us_3 = int(time_3*10**6)

        msg = Int64MultiArray()
        msg.data = [time_us_1, time_us_2, time_us_3]
        self.doppler_pub(msg)

    def clip(self, input, min, max):
        if input > max:
            return max
        if input < min:
            return min
        return input

    def texture_hot_cold(self):
        # This texture starts off with a intensity at mid range 
        # gets more intense the closer to the goal it is
        max_radius = 0.200 #m
        frequency = 60 #hz

        zeta = 2/(max_radius**2)

        mag_error = np.absolute(np.linalg.norm(self.goal - self.sensor))
        if mag_error > max_radius:
            mag_error = max_radius        
        # Scale distance to 0 and 127
        #intensity = (1.0-(mag_error/max_radius))*127
        intensity =(1.0-0.5*zeta*(mag_error)**2)*127

        self.actuator_states = [intensity,frequency]

    def binary_hot_cold(self):
        # This texture starts off with a intensity at mid range 
        # gets more intense the closer to the goal it is
        max_radius = 0.200 #m
        frequency = 200 #hz

        mag_error = np.absolute(np.linalg.norm(self.goal - self.sensor))
        if mag_error > max_radius:
            intensity = 50
        else: 
            intensity = 10       

        self.actuator_states = [intensity,frequency]

    def cartesian_detents(self, ticks):
        frequency = 60 #Hz
        distance_thresh = 0.005 #m
        intensity = 0
        grid_space = 0.25 #m
        detent = False
        remainder = np.mod(self.sensor,grid_space)
        for axis in remainder:
            
            if (np.abs(axis) < distance_thresh) or (np.abs(axis - grid_space) < distance_thresh):
                detent = True
                break

        if detent:
            intensity = 60

        self.actuator_states = [intensity, frequency]

    def periodic_detents(self):
        frequency = 200 #Hz
        intensity = 0
        wavelength=  0.010 #m
        amplitude = 60
        offset = amplitude/2.0 
        dist = np.linalg.norm(self.sensor)
        t = time.time()
        velocity = -0.5 # m/s

        intensity = amplitude*np.sin(dist*np.pi/wavelength+velocity*np.pi*2*t) + offset

        self.actuator_states = [intensity, frequency]

    def __del__(self):
        msg = Float32MultiArray()
        msg.data = [0,0]
        self.actuator_pub.publish(msg)

def main():
    rospy.init_node("marker_reader", anonymous=True)
    obj = marker()
    try:
        rospy.spin()
    except KeyboardInterrupt as e:
        obj.shutddown()
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
