"""
Used to plot raw GPS data vs. kalman-filtered data. In order to use this, you must
download the correct csv files (see testing instructions), and change the relevant filepaths.

At the bottom of the file, uncomment the files and function you want to use for plotting:

Use gps_only_retro() to plot kalman filter data made retroactively using only the gps.
You must uncomment GPS_FILE for this.

Use gps_imu_retro() to plot kalman filter data made retroactively using gps and imu data.
You must uncomment GPS_FILE and BIKE_STATE for this.

Use real_time() to plot kalman filter data that was made in real time during a test.
You must uncomment GPS_FILE and KALMAN_FILE for this.
"""

import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt
from matplotlib import image

import kalman
from util.sensor_data import SensorData
from util.location import global_to_local
from util.gmaps_plot import GMapsPlot
from util.math_helpers import dist_radians


def kalman_odometry_retro(gps_filename, bike_filename, show_plots=True):
    # Load data from csv file.
    sensors = SensorData(gps_filename=gps_filename, bike_filename=bike_filename)
    # sensors = sensors.range(0, 80)

    # Run the Kalman filter
    output = kalman.kalman_retro(sensors)

    if show_plots:
        # Plot GPS data and filter output.
        plt.figure(1)
        plt.scatter(sensors.gps.x, sensors.gps.y, c='r')
        plt.scatter(output['x'], output['y'], c='b')

        # Plot some filter values for debugging.
        plt.figure(2)
        plt.subplot(3, 1, 1)

        yaw_deltas = []
        
        for gps, imu in zip(output['last_gps_yaw'], output['last_yaw']):
            if gps is not None and imu is not None:
                yaw_deltas.append(dist_radians(gps, imu))

        realized_yaw = []
        px, py = output['x'][0], output['y'][0]
        for x, y in zip(output['x'], output['y']):
            realized_yaw.append(np.arctan2(y - py, x - px))
            px, py = x, y

        plt.scatter(output['timestamp'], output['last_yaw'], c='b')
        plt.scatter(output['timestamp'], output['last_gps_yaw'], c='r')
        #plt.scatter(output['timestamp'], realized_yaw, c='g')
        plt.legend(['IMU Yaw', 'GPS Yaw'])

        plt.subplot(3, 1, 2)
        plt.scatter(output['timestamp'], output['last_x'], c='r')
        plt.scatter(output['timestamp'], output['last_y'], c='y')
        plt.legend(['GPS X', 'GPS Y'])

        plt.subplot(3, 1, 3)
        plt.scatter(list(range(0, len(yaw_deltas))), np.rad2deg(yaw_deltas), c='r')
        #plt.legend(['Yaw Offset'])

        plt.show()

    # Draw google maps plot to html.
    gmp = GMapsPlot()
    gmp.add_plot(sensors.gps.x, sensors.gps.y, color='#FF0000')
    gmp.add_plot(output['x'], output['y'], color='#0000FF')
    gmp.render()


def kalman_compare_retro(gps_filename, bike_filename):
    gps_data = []
    with open(gps_filename) as gps_file:
        gps_reader = list(csv.reader(gps_file, delimiter=","))

        for row in gps_reader[1:]:

            # field0 is lat, field1 is long, field7 is yaw in degrees,
            # field8 is speed from the gps (meters per second)
            # field10 is timestep (miliseconds)
            x, y = global_to_local(float(row[2]), float(row[3]))
            if -0.0001 < x < 0.0001:
              continue
            # print '{:6.3f} {:6.3f}'.format(x, y)
            gps_data.append([x, y, float(row[9])*np.pi/180, float(row[10]), float(row[12])])
        
    # The Kalman filter wants the GPS data in matrix form
    gps_matrix = np.matrix(gps_data)

    # Run the Kalman filter
    output_old = kalman.kalman_retro_old(gps_matrix)

    # Run updated Kalman filter
    sensors = SensorData(gps_filename=gps_filename, bike_filename=bike_filename)
    output_new = kalman.kalman_retro(sensors)

    # Plot the GPS data
    plt.scatter(sensors.gps.x, sensors.gps.y, c='r')

    # Plot the old and new filter output
    plt.scatter([output_old[:,0]], [output_old[:,1]], c='b')
    plt.scatter(output_new['x'], output_new['y'], c='g')

    # Show everything
    plt.show()
