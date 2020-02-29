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
from localization.kalman import KalmanFilter
from localization.sensor_fusion import SensorFusion
import geometry
import requestHandler
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
from std_msgs.msg import Empty
from std_msgs.msg import String
from util.sensor_data import GPSData, BikeData
from util.location import global_to_local

gps_data = []
kalman_state = []

OUTLIER_THRESHOLD = 1000 # 1km from origin

class Kalman(object):
    def __init__(self):
        self.ready = False
        self.last_timestamp = 0

        # TODO: shouldn't need this
        self.gps_yaw = None
        self.gps_speed = None

        self.position_filter = SensorFusion(0, 0, 0, 0, 0)

        self.pub = rospy.Publisher('kalman_pub', Float32MultiArray, queue_size=10)
        self.pub_debug = rospy.Publisher('kalman_debug', Float32MultiArray, queue_size=10)

        rospy.init_node('kalman')
        rospy.Subscriber("bike_state", Float32MultiArray, self.bike_state_listener)
        rospy.Subscriber("gps", Float32MultiArray, self.gps_listener)
        rospy.Subscriber("nav_reset", Empty, self.reset_listener)


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
                speed       = data.data[6],
                timestamp   = None)

        self.position_filter.update(delta_time, bike_sensor=bike_sensor_data)

    def gps_listener(self, data):
        """ROS callback for the gps topic"""
        latitude = data.data[0] # In degrees
        longitude = data.data[1]

        # Converts lat long to x,y using FIXED origin
        x, y = global_to_local(float(latitude), float(longitude))

        #if abs(x) > OUTLIER_THRESHOLD or abs(y) > OUTLIER_THRESHOLD:
            # return

        self.gps_speed = data.data[8]
        self.gps_yaw = np.deg2rad(data.data[7])
        timestamp = data.data[0] / 1.e9

        # Converts lat long to x,y using FIXED origin
        if not self.ready:
            # TODO: don't assume initial xdot and ydot are zero
            self.k1_state = np.matrix([[x], [y], [0], [0]])
            self.k2_state = np.matrix([[self.gps_yaw], [0]])
            self.ready = True
            self.last_timestamp = timestamp

        delta_time = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        self.time_step = [data.data[10]]

        # Converts lat long to x,y using FIXED origin
        x, y = global_to_local(float(latitude), float(longitude))

        if abs(x) > OUTLIER_THRESHOLD or abs(y) > OUTLIER_THRESHOLD:
            return

        self.gps_speed = data.data[8]
        self.gps_yaw = np.deg2rad(data.data[7])
        timestamp = data.data[0] / 1.e9

        # Converts lat long to x,y using FIXED origin
        dim = [MultiArrayDimension('data', 1, 2)]
        layout = MultiArrayLayout(dim, 0)

        self.pub_debug.publish(layout, gps_debug_state)

        gps_sensor_data = GPSData(
                x           = x,
                y           = y,
                yaw         = self.gps_yaw,
                speed       = self.gps_speed,
                timestamp   = None)

        self.position_filter.update(delta_time, gps_sensor=gps_sensor_data)

    def reset_listener(self, data):
        # so that the next time we get GPS info, we just refresh our state
        self.ready = False
        self.position_filter.reset()

    def main_loop(self):
        rate = rospy.Rate(100)

        #Run until the nodes are shutdown (end.sh run OR start.sh was killed)
        while not rospy.is_shutdown():
            dim = [MultiArrayDimension('data', 1, 4)]
            layout = MultiArrayLayout(dim, 0)

            if self.ready:
                kalman_state = [
                        self.position_filter.get_x(),
                        self.position_filter.get_xdot(),
                        self.position_filter.get_y(),
                        self.position_filter.get_ydot()
                ]
                
                self.pub.publish(layout, kalman_state)

            rate.sleep()


if __name__ == '__main__':
    Kalman().main_loop()
