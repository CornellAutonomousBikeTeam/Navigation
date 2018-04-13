import numpy as np
from math import sin, cos
from util.sensor_data import GPSData, BikeData
from util.math_helpers import align_radians, dist_radians
from util.constants import BIKE_LENGTH


class SensorFusion:
    def __init__(self, x, y, speed, yaw, yawdot):
        self.x      = x
        self.y      = y
        self.speed  = speed
        self.yaw    = yaw
        self.yawdot = yawdot

        self.gps_last_x = None
        self.gps_last_y = None

        self.bike_speed_gain    = 0.1
        self.bike_yaw_gain      = 0.1
        self.bike_yawdot_gain   = 1.0
        self.gps_gain           = 0.001
        self.gps_edge_gain      = 0.05
        self.gps_res_x          = 0.627
        self.gps_res_y          = 0.424


    def get_x(self):        return self.x

    def get_y(self):        return self.y

    def get_xdot(self):     return self.speed * cos(self.yaw)

    def get_ydot(self):     return self.speed * sin(self.yaw)

    def get_yaw(self):      return self.yaw

    def get_yawdot(self):   return self.yawdot


    def update(self, dt, gps_sensor=None, bike_sensor=None):
        if bike_sensor is not None:
            self.speed += self.bike_speed_gain * (bike_sensor.speed - self.speed)
            # if bike_sensor.timestamp < 55:
            self.yaw += self.bike_yaw_gain * dist_radians(bike_sensor.yaw, self.yaw)
            self.yawdot = self.speed * np.tan(bike_sensor.steer) / BIKE_LENGTH

        if gps_sensor is not None:
            # if gps_sensor.timestamp < 55:
            if self.gps_last_x is not None and self.gps_last_y is not None:
                # Check if the gps jumped, if so assume we are on the edge of that jump
                if abs(gps_sensor.x - self.gps_last_x) > 0.01 * self.gps_res_x:
                    gps_x = 0.5 * (gps_sensor.x + self.gps_last_x)
                    self.x += self.gps_edge_gain * (gps_x - self.x)

                if abs(gps_sensor.y - self.gps_last_y) > 0.01 * self.gps_res_y:
                    gps_y = 0.5 * (gps_sensor.y + self.gps_last_y)
                    self.y += self.gps_edge_gain * (gps_y - self.y)

                # Regardless of a jump, if position is too far away then adjust it
                if abs(gps_sensor.x - self.x) > self.gps_res_x:
                    self.x += self.gps_gain * (gps_sensor.x - self.x)

                if abs(gps_sensor.y - self.y) > self.gps_res_y:
                    self.y += self.gps_gain * (gps_sensor.y - self.y)

            self.gps_last_x = gps_sensor.x
            self.gps_last_y = gps_sensor.y

        self.yaw += dt * self.yawdot
        self.x += dt * self.speed * cos(self.yaw)
        self.y += dt * self.speed * sin(self.yaw)
