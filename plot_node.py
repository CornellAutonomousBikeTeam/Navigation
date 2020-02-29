#!/usr/bin/env python
import numpy as np
import rospy
import geometry
import time
from util.location import global_to_local
import mapModel

from bike.msg import Location

import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib import collections as mc
from matplotlib.path import Path
from matplotlib.patches import Wedge, PathPatch, Circle

class Plotter(object):

    def loc_listener_old(self, data):
            #if hasattr(self, 'path_patch') and self.path_patch:
            #    self.path_patch.remove()
            #if len(self.bike_trajectory):
            #    self.path_patch = PathPatch(Path(self.bike_trajectory), fill=False,
            #                       linewidth=2)
            #    axes.add_patch(self.path_patch)

            # Plot the bike as a wedge pointing in the direction bike.psi

            bike_x, bike_y = global_to_local(data.lat, data.lng)
            #rospy.logerr("%f,%f"%(bike_x,bike_y))
            self.circle.center = (bike_x,bike_y)
            #if self.prev_bike_patch: self.prev_bike_patch.remove()
            #wedge_angle = 45 # The angle covered by the wedge
            #bike_heading = data.yaw
            #bike_polygon = Wedge((bike_x, bike_y), 1,
            #                     bike_heading - wedge_angle / 2 + 180,
            #                     bike_heading + wedge_angle / 2 + 180, fc="black")
            #rospy.logerr(dir(bike_polygon))
            #self.axes.add_patch(bike_polygon)
            #self.prev_bike_patch = bike_polygon
            #plt.show(block=True)
            #plt.pause(0.00000000000001)
            self.bike_trajectory.append((bike_x, bike_y))
            plt.draw()

    def loc_listener(self, data):
            self.circle.center = global_to_local(data.lat, data.lng)
            plt.draw()

    def loc2_listener(self, data):
            self.circle2.center = global_to_local(data.lat, data.lng)
            plt.draw()

    def main(self):
        rospy.init_node("plotter")
        while not rospy.has_param('path'):
            time.sleep(1)

        self.path = rospy.get_param("path")
	plt.ion() # enables interactive plotting
        waypoints = map(lambda p: global_to_local(*p), self.path)
        rospy.logerr(waypoints)
	map_model = mapModel.Map_Model(None, waypoints, [], [])
	paths = map_model.paths
	fig = plt.figure()
	#ax = plt.axes(**find_display_bounds(map_model.waypoints))
        ax = plt.axes(xlim=(-100,100), ylim=(-100,100))
	lc = mc.LineCollection(paths, linewidths=2, color = "blue")
	ax.add_collection(lc)

	# For plotting the bicycle
	self.axes = plt.gca()

        self.circle = Circle((0,0),1,ec='r', fc='none')
        self.axes.add_patch(self.circle)

        self.circle2 = Circle((0,0),1,ec='b', fc='none')
        self.axes.add_patch(self.circle2)

	# Holds past locations of the bike, for plotting
	self.bike_trajectory = []

	# We need to keep this around to clear it after path updates
	#self.path_patch = None

	self.prev_bike_patch = None

        rospy.Subscriber("/android/gps/fused", Location, self.loc_listener) # RED
        rospy.Subscriber("/android/gps/gps", Location, self.loc2_listener) # BLUE
	plt.show(block=True)

if __name__ == '__main__':
    Plotter().main()
