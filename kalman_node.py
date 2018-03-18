#!/usr/bin/env python
"""
Used for ROS communication. It allows real-time 'filtering' of position data
on the bike by subscribing to data from different sensors (GPS + IMU). When
testing, use "bash start.sh run_with_kalman" to run this node.
"""

import numpy as np
from std_msgs.msg import Float32
import rospy
import time
from localization.kalman import kalman
import geometry
import requestHandler
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from util.sensor_data import GPSData, BikeData
from util.location import global_to_local

gps_data = []
kalman_state = []


class Kalman(object):
    def __init__(self):
        self.ready = False
        self.last_timestamp = None

        self.k1_state = None
        self.k1_P = np.identity(4)

        self.k2_state = None
        self.k2_P = np.identity(2)

        # TODO: shouldn't need this
        self.gps_yaw = None
        self.gps_speed = None

        self.pub = rospy.Publisher('kalman_pub', Float32MultiArray, queue_size=10)
        rospy.init_node('kalman')
        rospy.Subscriber("bike_state", Float32MultiArray, self.bike_state_listener)
        rospy.Subscriber("gps", Float32MultiArray, self.gps_listener)


    def bike_state_listener(self, data):
        """ROS callback for the bike_state topic"""
        if not self.ready:
            return

        steer = data.data[4]
        timestamp = data.data[0] / 1.e9

        delta_time = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        bike_sensor_data = BikeData(
                steer       = steer,
                yaw         = self.gps_yaw,
                speed       = self.gps_speed,
                timestamp   = None)

        output = kalman(
                delta_time,
                self.k1_state,
                self.k1_P,
                self.k2_state,
                self.k2_P,
                bike_sensor=bike_sensor_data)

        self.k1_state   = output[0]
        self.k1_P       = output[1]
        self.k2_state   = output[2]
        self.k2_P       = output[3]


    def gps_listener(self, data):
        """ROS callback for the gps topic"""
        latitude = data.data[0] # In degrees
        longitude = data.data[1]
        velocity = data.data[8]
        yaw = np.deg2rad(data.data[7])
        timestamp = data.data[0] / 1.e9
       
        # Converts lat long to x,y using FIXED origin
        x, y = global_to_local(float(latitude), float(longitude))

        if not self.ready:
            # TODO: don't assume initial xdot and ydot are zero
            self.k1_state = np.matrix([[x], [y], [0], [0]])
            self.k2_state = np.matrix([[yaw], [0]])
            self.ready = True
            self.last_timestamp = timestamp

        delta_time = timestamp - self.last_timestamp
        self.last_timestamp = timestamp
        
        gps_sensor_data = GPSData(
                x           = x,
                y           = y,
                yaw         = yaw,
                speed       = velocity,
                timestamp   = None)

        output = kalman(
                delta_time,
                self.k1_state,
                self.k1_P,
                self.k2_state,
                self.k2_P,
                gps_sensor=gps_sensor_data)

        self.k1_state   = output[0]
        self.k1_P       = output[1]
        self.k2_state   = output[2]
        self.k2_P       = output[3]
        

    def main_loop(self):
        rate = rospy.Rate(100)

        #Run until the nodes are shutdown (end.sh run OR start.sh was killed)
        while not rospy.is_shutdown():
            dim = [MultiArrayDimension('data', 1, 4)]
            layout = MultiArrayLayout(dim, 0)
            
            #wait here until GPS has been called
            while not self.ready:
                time.sleep(0.001)
            
            #Change output_matrix to a standard array for publishing
            kalman_state = [
                    self.k1_state[0, 0],
                    self.k1_state[1, 0],
                    self.k1_state[2, 0],
                    self.k1_state[3, 0]
            ]
            
            self.pub.publish(layout, kalman_state)
            rate.sleep()


if __name__ == '__main__':
    Kalman().main_loop()
