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


def kalman_odometry_retro(gps_filename, bike_filename, show_plots=True):
    # Load data from csv file.
    sensors = SensorData(gps_filename=gps_filename, bike_filename=bike_filename)
    sensors = sensors.range(0, 80)

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
        plt.scatter(output['timestamp'], output['last_yaw'], c='r')
        plt.scatter(output['timestamp'], output['yaw'], c='b')
        plt.legend(['IMU Yaw', 'Filter Yaw'])

        plt.subplot(3, 1, 2)
        plt.scatter(output['timestamp'], output['last_x'], c='r')
        plt.scatter(output['timestamp'], output['last_y'])
        plt.legend(['GPS X', 'GPS Y'])

        plt.subplot(3, 1, 3)
        plt.scatter(output['timestamp'], output['last_gps_yaw'], c='r')
        plt.legend(['GPS Yaw'])

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

        for row in gps_reader[1:3000]:

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
    output_new_x = [el['x'] for el in output_new]
    output_new_y = [el['y'] for el in output_new]

    # Plot the GPS data
    # plt.scatter(sensors.gps.x, sensors.gps.y, c='r', edgecolors="none")

    # plt.imshow(quad_image, extent=[92.229, 141.543, -442.444, -390.070])

    # Plot the Kalman output
    # plt.scatter([output_old[:,0]], [output_old[:,1]], c='b', edgecolors="none")
    # plt.scatter(output_new_x, output_new_y, c='g', edgecolors='none')

    # plt.scatter(sensors.bike.timestamp, sensors.bike.speed)

    # Show everything
    plt.show()

  
# def real_time(gps_file, kalman_file):
  
#   gps_data = []
#   count = 0
#   orig_lat = 0
#   orig_long = 0
#   with open(gps_file) as gps_file:
#       gps_reader = csv.reader(gps_file, delimiter=",")
#       header_row = True
#       for row in gps_reader:
#           if header_row:
  
#               # Skip the first row, which contains the headers
#               header_row = False
#               continue
  
#           # field0 is lat, field1 is long
#           # field8 is speed from the gps (meters per second)
#           # field10 is timestep (miliseconds)
#           #Uncomment if not using relative
#           # x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), float(row[2]), float(row[3]))

#           #Uncomment up 'else' if using relative
#           if count == 0:
#             orig_lat = float(row[2])
#             orig_long = float(row[3])
#             x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), orig_lat , orig_long )
#             count = 1
#           else:
#             x, y = requestHandler.math_convert_relative(float(row[2]), float(row[3]), orig_lat, orig_long)
#           gps_data.append([x, y, float(row[10]), float(row[12])])
  
#   kalman_data = []    
#   with open(kalman_file) as kalman_file:
#     kalman_reader = csv.reader(kalman_file, delimiter=",")
#     header_row = True
#     for row in kalman_reader:
#         if header_row:

#             # Skip the first row, which contains the headers
#             header_row = False
#             continue

#         # field5 is lat converted to x, field6 is long converted to y - cartesian
#         kalman_data.append([float(row[5]), float(row[6])])
        
#   # The Kalman filter wants the GPS data in matrix form
#   gps_matrix = np.matrix(gps_data)
#   kalman_matrix = np.matrix(kalman_data)
  
#   # Plot the GPS data
#   plt.scatter([gps_matrix[:,0]], [gps_matrix[:,1]], c='r', edgecolors="none")
  
#   # Plot the Kalman output
#   plt.scatter([kalman_matrix[:,0]], [kalman_matrix[:,1]], edgecolors="none")
  
#   #PLOT WAYPOINTS ONCE WE HAVE THE CHANCE
#   waypoints = [(0.0, 0.0), (0.0, 10.0), (0.0, 20.0), (0.0, 30.0)]
  
#   plt.scatter([i[0] for i in waypoints], [i[1] for i in waypoints], c='g')
  
#   # Show everything
#   plt.show()
  
if __name__ == '__main__':
  GPS_FILE = "data/gps_data_new.csv"
  BIKE_STATE = "data/bike_data_new.csv"
  #KALMAN_FILE = "/Users/joshua/Desktop/kalman_pub_2017-11-07~~07-27-22-PM.csv"
  
  # kalman_odometry_retro(GPS_FILE, BIKE_STATE)
  # kalman_compare_retro(GPS_FILE, BIKE_STATE)
  plot_maps()
