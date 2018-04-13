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
from kalman import KalmanFilter
from sensor_fusion import SensorFusion
from util.sensor_data import SensorData, GPSData, BikeData
from util.location import global_to_local
from util.gmaps_plot import GMapsPlot
from util.math_helpers import dist_radians
from util.constants import BIKE_LENGTH


def compute_retro(sensors, dt=0.01, filter_type='SF'):
    # Get start and end time.
    start_time = min(sensors.gps.timestamp[0], sensors.bike.timestamp[0])
    end_time = min(sensors.gps.timestamp[-1], sensors.bike.timestamp[-1])

    # Keep track of index into sensor arrays so it's easier to know when the simulation time
    # passes though a sensor data point timestamp.
    gps_index = 0
    bike_index = 0

    # Get initial state vectors for both filters.
    x_init = sensors.gps.x[0]
    y_init = sensors.gps.y[0]
    speed_init = sensors.gps.speed[0]
    xdot_init = speed_init * np.cos(sensors.gps.yaw[0])
    ydot_init = speed_init * np.sin(sensors.gps.yaw[0])
    k1_state = np.matrix([[x_init], [y_init], [xdot_init], [ydot_init]])

    yaw_init = sensors.bike.yaw[0]
    yawdot_init = sensors.gps.speed[0] * np.tan(sensors.bike.steer[0]) / BIKE_LENGTH
    k2_state = np.matrix([[yaw_init], [yawdot_init]])

    # State estimate covariance matrix.
    k1_P = np.identity(4)
    k2_P = np.identity(2)    

    if filter_type == 'KF':
        position_filter = KalmanFilter(k1_state, k1_P, k2_state, k2_P)
    elif filter_type == 'SF':
        position_filter = SensorFusion(x_init, y_init, speed_init, yaw_init, yawdot_init)
    else:
        raise Exception('Filter type unrecognized')

    # Observed values for output.
    last_obs_x         = None
    last_obs_y         = None
    last_obs_yaw       = None
    last_obs_gps_yaw   = None
    last_obs_yawdot    = None
    last_obs_speed     = None
    integrated_yaw = yaw_init

    raw_output = []

    # Run the filters.
    for t in np.arange(start_time, end_time - dt, dt):
        gps_sensor = None
        bike_sensor = None

        # Check for sensor data during this timestep.
        if sensors.gps.timestamp[gps_index] < t:
            gps_index += 1

            gps_sensor = GPSData(
                x           = sensors.gps.x         [gps_index],
                y           = sensors.gps.y         [gps_index],
                yaw         = sensors.gps.yaw       [gps_index],
                speed       = sensors.gps.speed     [gps_index],
                timestamp   = sensors.gps.timestamp [gps_index]
            )

            last_obs_x = sensors.gps.x[gps_index]
            last_obs_y = sensors.gps.y[gps_index]
            last_obs_gps_yaw = sensors.gps.yaw[gps_index]

            # Catch up to most recent sensor measurement.
            while sensors.gps.timestamp[gps_index] < t:
                gps_index += 1
                #print 'Skipping sensor data, timestep might be too small.'

        if sensors.bike.timestamp[bike_index] < t:
            bike_index += 1

            bike_sensor = BikeData(
                steer       = sensors.bike.steer        [bike_index],
                yaw         = sensors.gps.yaw[gps_index], # TODO: fix when imu works
                speed       = sensors.gps.speed[gps_index], # TODO: fix when hall sensor works
                timestamp   = sensors.bike.timestamp    [bike_index]
            )

            last_obs_yaw = sensors.bike.yaw[bike_index]
            last_obs_speed = sensors.gps.speed[gps_index]
            last_obs_yawdot = (last_obs_speed * 
                    np.tan(sensors.bike.steer[bike_index]) / BIKE_LENGTH)

            # Catch up to most recent sensor measurement.
            while sensors.bike.timestamp[bike_index] < t:
                bike_index += 1
                #print 'Skipping sensor data, timestep might be too small.'

        position_filter.update(dt, gps_sensor=gps_sensor, bike_sensor=bike_sensor)

        raw_output.append({
            'timestamp':    t,
            'x':            position_filter.get_x(),
            'y':            position_filter.get_y(),
            'xdot':         position_filter.get_xdot(),
            'ydot':         position_filter.get_ydot(),
            'yaw':          position_filter.get_yaw(),
            'yawdot':       position_filter.get_yawdot(),
            'last_x':       last_obs_x,
            'last_y':       last_obs_y,
            'last_yaw':     last_obs_yaw,
            'last_gps_yaw': last_obs_gps_yaw,
            'last_yawdot':  last_obs_yawdot,
            'last_speed':   last_obs_speed
        })

    output = {
        'timestamp':    [p['timestamp']     for p in raw_output],
        'x':            [p['x']             for p in raw_output],
        'y':            [p['y']             for p in raw_output],
        'xdot':         [p['xdot']          for p in raw_output],
        'ydot':         [p['ydot']          for p in raw_output],
        'yaw':          [p['yaw']           for p in raw_output],
        'yawdot':       [p['yawdot']        for p in raw_output],
        'last_x':       [p['last_x']        for p in raw_output],
        'last_y':       [p['last_y']        for p in raw_output],
        'last_yaw':     [p['last_yaw']      for p in raw_output],
        'last_gps_yaw': [p['last_gps_yaw']  for p in raw_output],
        'last_yawdot':  [p['last_yawdot']   for p in raw_output],
        'last_speed':   [p['last_speed']    for p in raw_output]
    }

    return output


def draw_retro(gps_filename, bike_filename, show_plots=True):
    # Load data from csv file.
    sensors = SensorData(gps_filename=gps_filename, bike_filename=bike_filename)
    # sensors = sensors.range(0, 80)

    # Run the Kalman filter
    output = compute_retro(sensors, filter_type='SF')
    output_kf = compute_retro(sensors, filter_type='KF')

    if show_plots:
        # Plot GPS data and filter output.
        plt.figure(1)
        plt.scatter(sensors.gps.x, sensors.gps.y, c='r')
        plt.scatter(output['x'], output['y'], c='g')
        plt.scatter(output_kf['x'], output_kf['y'], c='b')
        plt.axes().set_aspect('equal')

        # Plot some filter values for debugging.
        plt.figure(2)
        plt.subplot(3, 1, 1)

        yaw_deltas = []
        
        for gps, imu in zip(output['last_gps_yaw'], output['last_yaw']):
            if gps is not None and imu is not None:
                yaw_deltas.append(dist_radians(gps, imu))

        plt.scatter(output['timestamp'], output['last_yaw'], c='b')
        # plt.scatter(output['timestamp'], output['integrated_yaw'], c='r')
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
