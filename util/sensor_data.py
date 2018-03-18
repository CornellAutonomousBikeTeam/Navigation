import csv
import math
import numpy as np
from collections import namedtuple
from util.location import global_to_local
from util.math_helpers import align_radians


# Thresholds for rejecting GPS outliers.
GPS_RANGE_THRESHOLD = 20000 # 20 kilometers
GPS_SPEED_THRESHOLD = 100   # 100 m/s


GPSData = namedtuple('GPSData', ['x', 'y', 'yaw', 'speed', 'timestamp'])
BikeData = namedtuple('BikeData', ['steer', 'yaw', 'speed', 'timestamp'])


class SensorData:
    def __init__(self, gps_filename=None, bike_filename=None):
        if gps_filename is not None:
            self.gps = self.load_gps_state(gps_filename)

        if bike_filename is not None:
            self.bike = self.load_bike_state(bike_filename)


    def load_gps_state(self, gps_filename):
        x_data = []
        y_data = []
        yaw_data = []
        speed_data = []
        timestamp_data = []

        
        with open(gps_filename) as gps_file:
            csv_reader = list(csv.reader(gps_file, delimiter=','))[1:]

            for i, row in enumerate(csv_reader):
            	x, y = global_to_local(float(row[2]), float(row[3]))
                yaw = align_radians(-float(row[9]) * np.pi / 180. + np.pi / 2)
                speed = float(row[10])
                timestamp = float(row[0]) / 1.e9

                # Remove outliers.
                if (np.linalg.norm([x, y]) > GPS_RANGE_THRESHOLD or
                        speed > GPS_SPEED_THRESHOLD):
                    print 'ignoring outlier gps[{}] = ({}, {})'.format(i, x, y)
                    continue

                x_data.append(x)
                y_data.append(y)
                yaw_data.append(yaw)
                speed_data.append(speed)
                timestamp_data.append(timestamp)

        # Shift timestamps to get time relative to start.
        start_time = min(timestamp_data)
        timestamp_data = [t - start_time for t in timestamp_data]
        
        return GPSData(
                x           = np.array(x_data),
                y           = np.array(y_data),
                yaw         = np.array(yaw_data),
                speed       = np.array(speed_data),
                timestamp   = np.array(timestamp_data))


    def load_bike_state(self, bike_state_filename):
        steer_data = []
        speed_data = []
        yaw_data = []
        timestamp_data = []

        with open(bike_state_filename) as bike_state_file:
            csv_reader = list(csv.reader(bike_state_file, delimiter=','))[1:]
            time_offset = None

            # skip header row
            for row in list(csv_reader)[1:]:
                steer = float(row[4])
                speed = float(row[8])
                yaw = align_radians(-float(row[11]) + np.pi / 2. + 0.9)
                timestamp = float(row[0]) / 1.e9

                steer_data.append(steer)
                speed_data.append(speed)
                yaw_data.append(yaw)
                timestamp_data.append(timestamp)

        # Shift timestamps to get time relative to start.
        start_time = min(timestamp_data)
        timestamp_data = [t - start_time for t in timestamp_data]

        return BikeData(
                steer       = np.array(steer_data),
                yaw         = np.array(yaw_data),
                speed       = np.array(speed_data),
                timestamp   = np.array(timestamp_data))


    # Makes a copy of this object with the subset of sensor data in the given time span.
    def range(self, start, end):
        subrange = SensorData()
        gps_start, gps_end = np.searchsorted(self.gps.timestamp, [start, end])
        bike_start, bike_end = np.searchsorted(self.bike.timestamp, [start, end])

        subrange.gps = GPSData(
                x           = self.gps.x[gps_start : gps_end],
                y           = self.gps.y[gps_start : gps_end],
                yaw         = self.gps.yaw[gps_start : gps_end],
                speed       = self.gps.speed[gps_start : gps_end],
                timestamp   = self.gps.timestamp[gps_start : gps_end])

        subrange.bike = BikeData(
                steer       = self.bike.steer[bike_start : bike_end],
                yaw         = self.bike.yaw[bike_start : bike_end],
                speed       = self.bike.speed[bike_start : bike_end],
                timestamp   = self.bike.timestamp[bike_start : bike_end])

        return subrange
