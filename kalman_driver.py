"""
Script that is used to plot raw GPS data vs. kalman-filtered data that was created
retroactively after a test. In order to use this script, you must
download the correct gps csv (see testing instructions), and
GPS_FILE below must be changed to the correct filepath.

Position, speed, and yaw are all collected from GPS.
"""

import csv
import matplotlib
import numpy as np

import matplotlib
from matplotlib import pyplot as plt

import kalman
import requestHandler

GPS_FILE = "/Users/joshuasones/Desktop/School/Bike Team/gps_2017-07-18~~02-36-30-PM-copy.csv"

gps_data = []

with open(GPS_FILE) as gps_file:
    gps_reader = csv.reader(gps_file, delimiter=",")
    header_row = True
    for row in gps_reader:
        if header_row:

            # Skip the first row, which contains the headers
            header_row = False
            continue

        # field0 is lat, field1 is long, field7 is yaw in degrees,
        # field8 is speed from the gps (meters per second)
        # field10 is timestep (miliseconds)
        x, y = requestHandler.math_convert(float(row[2]), float(row[3]))
        gps_data.append([x, y, float(row[9])*np.pi/180, float(row[10]), float(row[12])])

# The Kalman filter wants the GPS data in matrix form
gps_matrix = np.matrix(gps_data)

# Plot the GPS data
plt.scatter(gps_matrix[:,0], gps_matrix[:,1], c='r', edgecolors="none")

# Run the Kalman filter
output_matrix = kalman.kalman_retro(gps_matrix)

# Plot the Kalman output
plt.scatter(output_matrix[:,0], output_matrix[:,1], edgecolors="none")

# Show everything
plt.show()
