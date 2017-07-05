# Navigation- UNSTABLE
Master branch should have stable code only. If unstable, write unstable at the top.
Repo for the nav code- .gitignore only has python

**nav.py** contains the navigation algorithms with steering angle as output.  
Run **loop.py** to see simulation.  

All files with "_node" are used for ROS communication.  

 ## Known Bugs
  - [ ] Divide by 0 errors in geometry.py methods
  - [ ] Verify all return types using python's .dtype() method
 
 ## Last commited by
 > Kenneth Fang
 
 ## Reason for commit
> Working on documentation for the code embedded in the README.md, and removing some repeat/useless cde.

---
## Documentation
* [bikeSim.py](#bikeSim)
* [bikeState.py](#bikeState)
* [constants.py](#constants)
* [geometry.py](#geometry)
* [loop.py](#loop)
* [mapModel.py](#mapModel)
* [nav.py](#nav)
* [requestHandler.py](#requestHandler)
* [gps_assisted_simulator_node.py](#gps_assisted_simulator_node)
* [navigation_node.py](#navigation_node)
* [map_node.py](#map_node)
* [simulator_node.py](#simulator_node)
* [vis_node.py](#vis_node)

All files with "_node" are used for ROS communication.

### <a name="bikeSim"></a> bikeSim.py
Module that simulates the bikes dynamics.

  <details>  
    <summary><small>bikeSim.py-Click to Expand</small></summary><p>  
    
		from constants import *
		import bikeState

		def new_state(bike, steerD):
		""" Returns new bike state object after it passes through bike dynamics """
		# Get navigation command
		# steerD = nav.command(bike.xB, bike.yB, bike.psi, bike.v, waypoints)
		rhs_result = bike.rhs(steerD)
		u = rhs_result[0]
		values = rhs_result[1]
		new_xB = bike.xB + values[0]*TIMESTEP
		new_yB = bike.yB + values[1]*TIMESTEP
		new_phi = bike.phi + values[2]*TIMESTEP
		new_psi = bike.psi+ values[3]*TIMESTEP
		new_delta = bike.delta + values[4]*TIMESTEP
		new_w_r = bike.w_r + values[5]*TIMESTEP
		new_v = bike.v + values[6]*TIMESTEP
		new_state = bikeState.Bike(new_xB, new_yB, new_phi, new_psi, new_delta, new_w_r, new_v)
		return new_state

    
  </p></details>   
  
Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
Bike object| new_state(bike, steerD)| Returns: next bikeState given current state [bike] and direction [steerD]

---

### <a name="bikeState"></a> bikeState.py
Defines the Bike object which represents the autonomous bike for the purposes of navigation. Not to be confused with Bike_State class in Arduino code.


  <details>  
    <summary><small>bikeState.py-Click to Expand</small></summary><p>  
    
    		import numpy as np
		from constants import *
		""" This module contains a class to represent the bike's state """


		class Bike(object):

			def __init__(self, xB, yB, phi, psi, delta, w_r, v):
				""" State representation in Matlab """

				#xB, yB initial position coordinates of bicycle
				self.xB = xB
				self.yB = yB
				self.phi = phi #lean
				self.psi = psi #direction (aka thetaB)
				self.delta = delta # Steer Angle!!!!!!
				self.w_r = w_r #lean rate
				self.v = v  #speed
				self.turning_r = 2


			@property
			def vector(self):
				b = np.array([math.cos(self.psi), math.sin(self.psi)])
				return b/np.linalg.norm(b)

			def update(self, bike):
				self.xB = bike.xB
				self.yB = bike.yB
				self.phi = bike.phi
				self.psi = bike.psi
				self.delta = bike.delta
				self.w_r = bike.w_r
				self.v = bike.v
				self.turning_r = bike.turning_r


			def rhs(self, steerD):
				""" Equivalent to rhs in Matlab. Modifies the state object to turn it into the next state """

				deltaD = steerD

				u = K1 * self.phi + K2 * self.w_r + K3 * (self.delta - deltaD)

				if u > 10:
					u = 10
				elif u < -10:
					u = -10

				xdot = self.v * np.cos(self.psi)
				ydot = self.v * np.sin(self.psi)
				phi_dot = self.w_r
				psi_dot = (self.v/L)*(np.tan(self.delta)/np.cos(self.phi))
				delta_dot = u # ideal steer rate
				v_dot = 0
				# wr_dot = (((-(self.v)**2)*self.delta) - B * self.v * u + G * L * self.phi)/(H*L)
				wr_dot = (1/H)*(G*np.sin(self.phi) - np.tan(self.delta)*((self.v**2)/L + B*v_dot/L + np.tan(self.phi)*((B/L)*self.v*phi_dot -   (H/(L**2)*(self.v**2)*np.tan(self.delta))))-B*self.v*delta_dot/(L*np.cos(self.delta)**2))
				# Returns u which is the motor command and the zdot vector in the form of a list
				return (u, [xdot, ydot, phi_dot, psi_dot, delta_dot, wr_dot, v_dot])
  </p></details>  
  
  Class Definition:

Type | Instance Attribute | Description
:-------------: |:-------------:| :-----:
float | xB | Bike position's x-coordinate
float | yB | Bike position's y-coordinate
float | phi | Lean angle (radians)
float | psi | Direction/heading/yaw (radians)
float | delta | Steer angle (radians)
float | w_r | Lean rate (m/s)
float | v | Speed (m/s)
  
  Methods: //TODO

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
array? | vector(self) | Returns the bike's unit vector
void | update(self, bike) | Updates the bike state
array? | rhs(self, steerD) | Modifies state object to turn it into the next state using bike dynamics. Equivalent to rhs in Matlab


---

### <a name="constants"></a> constants.py
Definitions of various constants for Navigation code. //TODO

  <details>
    <summary><small>constants.py-Click to Expand</small></summary><p>  
    
  </p></details>   
  
Constant Names | Significance | Used In
:-------------: |:-------------:| :-----:
G, L, B, H | Constants from the Matlab bike dynamics simulation | bikeSim.py
C | Trail? | ?
K1 | Gain from old nav algorithm | Not used
K2 | Gain from old nav algorithm | Not used
K3 | Gain from old nav algorithm | Not used
TIMESTEP | ? | ?
PAUSE | ? | ?
MAX_Steer| Bike's maximumum steer angle, used in clamping steerD | nav.py
PID_DIST_GAIN | Gain used for distance contribution of PID | nav.py
PID_ANGLE_GAIN | Gain used for angle contribution of PID | Not used
RAD_TO_DEG | Ratio of radians to degrees | nav.py
TURN_LOOKAHEAD_DIST | A lookahead distance | Not used
NEXT_TURN_GAIN | Gain used for next turn contribution of PID| nav.py
BIKE_LENGTH | Bike length used in quintic | nav.py
QUINTIC_LOOKAHEAD | Lookahead distance used in quintic | nav.py
QUINTIC_SAMPLE_LENGTH | Constant used in quintic polynomial | nav.py
BASE_LOOKAHEAD | ? | Not used
LOOKAHEAD_ANGLE_GAIN| Gain used for lookahead angle | Not used
MIN_TURN_RADIUS| Bike's minimum turning radius, determined using MAX_STEER | nav.py
ANIM_INTERVAL| Miliseconds between blit frames | loop.py

---
### <a name="geometry"></a> geometry.py
Module that contains many convenience functions that perform mathematical calculations. Used in __

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
array? | unit_vector(p1, p2) |  Returns: the unit vector given 2D points [p1] and [p2]
array? | threeD_unit_vector(p1, p2) | Returns: the unit vector given points [p1] and [p2]
float? | dist2(p1, p2) |  Returns: the square of the distance between [p1] and [p2]
float? | distance(p1, p2) | Returns: the distance between points [p1] and [p2]
int? | get_sign(v) | Returns: the sign of the number [v]
array? | nearest_point_on_path(path, point) | Returns: the nearest point on the [path] to the [point]
float? | angle_from_path(bike_direction, p1, p2) | Returns: angle that the bicycle has to turn to approach line [p1] to [p2]
float? | distance_from_path(point, target_path) | Returns: calculates the distance from [point] to [target_path]
float? | line_slope(line) | Returns: slope of [line]
float? | angle_between_vectors(v1, v2) | Returns: angle between vectors [v1] and [v2]. Used in angle_between_two_lines
float? | angle_between_two_lines(line1, line2) | Returns: angle between [line1] and [line2]
float? | dot_product(v1, v2) | Returns: mathematical dot product of vectors [v1] and [v2]
array? | intersect_circle_path(path, r, c) | Returns: points of intersection between [path] and circle with radius [r] and center [c]
---
### <a name="loop"></a> loop.py
Main Navigation file. Runs a loop that gets navigation command, passes it thorugh simulation, and gets the new updated state.

Main loop:

  <details>
    <summary><small>loop.py-Click to Expand</small></summary><p>  
    
		import sys
		import nav
		import mapModel
		import math
		import bikeState
		import bikeSim
		from constants import *
		import geometry
		import requestHandler

		import numpy as np

		# Decide which visualization function to use based on imports.
		get_loop_function = lambda: lambda nav, bike, map_model: None
		try:
			import pyqtgraph as pg
			from pyqtgraph.Qt import QtGui, QtCore
			get_loop_function = lambda: loop_pyqtgraph
		except ImportError:
			try:
				import matplotlib

				from matplotlib import pyplot as plt
				from matplotlib import animation
				from matplotlib import collections as mc
				from matplotlib.path import Path
				from matplotlib.patches import Wedge, PathPatch, Circle

				# Use a different graphics backend and function for OS X
				if sys.platform == "darwin":
					matplotlib.use("TkAgg")
					get_loop_function = lambda: loop_matplotlib
				else:
					get_loop_function = lambda: loop_matplotlib_blitting
			except ImportError:
				print("No animation libraries usable! Quitting.")
				sys.exit(1)

		def loop_pyqtgraph(nav, bike, map_model):
			traces = [dict()]

			# Set the background color of all plots to white for legibility
			pg.setConfigOption('background', 'w')

			qt_win = pg.GraphicsWindow(title="Bike Simulator 2017")

			# Stores every item in the "trajectory" plot
			plot_items = [dict()]

			# This ViewBox will hold the bike and trajectory
			viewbox = qt_win.addPlot(col=0, row=0, lockAspect=1.0)
			viewbox.sigResized = viewbox.sigRangeChanged # Axes need this method
			viewbox.showAxis("bottom", show=True)

			# Make an item for the bike
			bike_polygon = QtGui.QPolygonF()
			for _ in xrange(3): bike_polygon.append(QtCore.QPointF(0, 0))
			bikeItem = QtGui.QGraphicsPolygonItem(bike_polygon)
			plot_items[0]["bikeitem"] = bikeItem
			plot_items[0]["bike"] = bike_polygon
			bikeItem.setPen(pg.mkPen(None))
			bikeItem.setBrush(pg.mkBrush('r'))
			viewbox.addItem(bikeItem)

			# Graphics helper
			def setPenWidth(pen, width):
				pen.setWidth(width)
				return pen

			# Make an item for the static (given) path
			paths = map_model.paths
			static_path = QtGui.QPainterPath()
			static_path.moveTo(paths[0][0][0], paths[0][0][1])
			for each_segment in paths:
				static_path.lineTo(*each_segment[1])
			static_path_item = QtGui.QGraphicsPathItem(static_path)
			static_path_item.setPen(setPenWidth(pg.mkPen('g'), 2))
			static_path_item.setBrush(pg.mkBrush(None))
			viewbox.addItem(static_path_item)

			# Make an item for the trajectory
			traj_path = QtGui.QPainterPath()
			traj_path.moveTo(bike.xB, bike.yB)
			plot_items[0]["traj"] = traj_path
			traj_path_item = QtGui.QGraphicsPathItem(traj_path)
			traj_path_item.setPen(setPenWidth(pg.mkPen('b'), 2))
			traj_path_item.setBrush(pg.mkBrush(None))
			plot_items[0]["trajitem"] = traj_path_item
			viewbox.addItem(traj_path_item)

			def traj_update():
				plot_items[0]["traj"].lineTo(bike.xB, bike.yB)
				plot_items[0]["trajitem"].setPath(plot_items[0]["traj"])
			traj_timer = QtCore.QTimer()
			traj_timer.timeout.connect(traj_update)
			traj_timer.start(100)

			# This bit simulates the algo receiving delayed information
			delayed_bike = bikeState.Bike(0, 0, 0, 0, 0, 0, 0)
			nav.map_model.bike = delayed_bike
			def delayed_update():
				delayed_bike.update(bike)
			delayed_timer = QtCore.QTimer()
			delayed_timer.timeout.connect(delayed_update)
			delayed_timer.start(1)

			get_steering_angle = nav.get_steering_angle
			simulation_step = lambda angle: bike.update(bikeSim.new_state(bike, angle))
			def update():
				simulation_step(get_steering_angle())
				x1, y1 = bike.xB, bike.yB
				x2, y2 = bike.xB + 2.0 * math.cos(bike.psi + 13 * math.pi / 12), bike.yB + 2.0 * math.sin(bike.psi + 13 * math.pi / 12)
				x3, y3 = bike.xB + 2.0 * math.cos(bike.psi + 11 * math.pi / 12), bike.yB + 2.0 * math.sin(bike.psi + 11 * math.pi / 12)
				#x1, y1, x2, y2, x3, y3 = tuple(int(num) for num in (x1, y1, x2, y2, x3, y3))
				new_polygon = QtGui.QPolygonF()
				for each_point in ((x1, y1), (x2, y2), (x3, y3)): new_polygon.append(QtCore.QPointF(*each_point))
				plot_items[0]["bikeitem"].setPolygon(new_polygon)

			anim_timer = QtCore.QTimer()
			anim_timer.timeout.connect(update)
			anim_timer.start(0)

			QtGui.QApplication.instance().exec_()

		def loop_matplotlib(nav, bike, map_model):
			"""This function uses adding and removing patches to animate the bike."""
			plt.ion() # enables interactive plotting
			paths = map_model.paths
			fig = plt.figure()
			ax = plt.axes(**find_display_bounds(map_model.waypoints))
			lc = mc.LineCollection(paths, linewidths=2, color = "blue")
			ax.add_collection(lc)
			plt.show()

			# For plotting the bicycle
			axes = plt.gca()

			# Holds past locations of the bike, for plotting
			bike_trajectory = [(bike.xB, bike.yB)]

			# We need to keep this around to clear it after path updates
			path_patch = None

			prev_bike_patch = None
			prev_lookahead_patch = None

			# Main animation loop
			while True:

				if path_patch:
					path_patch.remove()
				path_patch = PathPatch(Path(bike_trajectory), fill=False,
						       linewidth=2)
				axes.add_patch(path_patch)

			# Plot the bike as a wedge pointing in the direction bike.psi
				if prev_bike_patch:
					prev_bike_patch.remove()
				bike_heading = bike.psi * (180/math.pi) # Converted to degrees
				wedge_angle = 45 # The angle covered by the wedge
				bike_polygon = Wedge((bike.xB, bike.yB), 0.3,
						     bike_heading - wedge_angle / 2 + 180,
						     bike_heading + wedge_angle / 2 + 180, fc="black")
				axes.add_patch(bike_polygon)
				prev_bike_patch = bike_polygon
				plt.show()
				plt.pause(0.00000000000001)

				bike_trajectory.append((bike.xB, bike.yB))

				steerD = nav.get_steering_angle()
				# if new state has new target path then recalculate delta
				bike.update(bikeSim.new_state(bike, steerD))

		def loop_matplotlib_blitting(nav, bike, map_model, blitting=True):
			"""This code uses blitting and callbacks to simulate the
			bike."""
			figure, axes = plt.figure(), plt.axes(**find_display_bounds(map_model.waypoints))

			# Square aspect ratio for the axes
			axes.set_aspect("equal")

			paths = new_map_model.paths

			# Draw the paths
			lc = mc.LineCollection(paths, linewidths=2, color = "blue")
			axes.add_collection(lc)

			# Paths won't change, so capture them
			figure.canvas.draw()
			background = [figure.canvas.copy_from_bbox(axes.bbox)]

			# Create bike polygon
			bike_heading = bike.psi * (180/math.pi) # heading is psi, but in degrees
			wedge_angle = 45 # The angle covered by the wedge (degrees)
			theta1 = bike_heading - wedge_angle / 2 + 180
			theta2 = bike_heading + wedge_angle / 2 + 180
			bike_polygon = Wedge((bike.xB, bike.yB), 1, theta1, theta2, fc="black")
			bike_polygon.set_zorder(10)
			axes.add_artist(bike_polygon)

			# Create bike trajectory
			bike_trajectory_polygon = axes.plot([0, 0], [0, 0], "g")[0]

			# Set up trajectory data
			bike_traj_x = [bike.xB] # Just the x-coords
			bike_traj_y = [bike.yB] # Just the y-coords
			add_traj_x = bike_traj_x.append
			add_traj_y = bike_traj_y.append

			# Create lookahead point
			lookahead_polygon = Circle((bike.xB, bike.yB), 1)
			axes.add_artist(lookahead_polygon)

			# Create dropped point
			dropped_polygon = Circle((bike.xB, bike.yB), 1, fc="red")
			axes.add_artist(dropped_polygon)

			# Create current line highlight
			current_line = axes.plot([0, 0], [0, 0], "r")[0]
			axes.add_artist(current_line)

			# Set up resizing handlers
			listener_id = [None]
			def safe_draw():
				canvas = figure.canvas
				if listener_id[0]: canvas.mpl_disconnect(listener_id[0])
				canvas.draw()
				listener_id[0] = canvas.mpl_connect("draw_event", grab_background)
			def grab_background(event=None):
				transient_polygons = (bike_polygon, lookahead_polygon,
						      current_line, dropped_polygon)
				for polygon in transient_polygons:
					polygon.set_visible(False)
				safe_draw()
				background[0] = figure.canvas.copy_from_bbox(figure.bbox)
				for polygon in transient_polygons:
					polygon.set_visible(True)
				blit()
			def blit():
				figure.canvas.restore_region(background[0])
				axes.draw_artist(bike_polygon)
				figure.canvas.blit(axes.bbox)
			listener_id[0] = figure.canvas.mpl_connect("draw_event", grab_background)

			# This timer runs simulation steps and draws the results
			figure_restore = figure.canvas.restore_region
			get_steering_angle = nav.get_steering_angle
			simulation_step = lambda angle: bike.update(bikeSim.new_state(bike, angle))
			figure_blit = figure.canvas.blit
			def full_step():
				figure_restore(background[0])
				simulation_step(get_steering_angle())

				# Update bike polygon properties and redraw it
				wedge_dir = bike.psi * (180/math.pi) + 180
				bike_pos = (bike.xB, bike.yB)
				bike_polygon.set(center = bike_pos,
						 theta1 = wedge_dir - wedge_angle / 2,
						 theta2 = wedge_dir + wedge_angle / 2)
				axes.draw_artist(bike_polygon)

				# Update trajectory and redraw it
				add_traj_x(bike.xB)
				add_traj_y(bike.yB)
				bike_trajectory_polygon.set_xdata(bike_traj_x)
				bike_trajectory_polygon.set_ydata(bike_traj_y)
				axes.draw_artist(bike_trajectory_polygon)

				# Update and redraw lookahead point
				lookahead_polygon.center = nav.lookahead_point
				#axes.draw_artist(lookahead_polygon)

				# Update and redraw dropped point
				dropped_polygon.center = nav.dropped_point
				#axes.draw_artist(dropped_polygon)

				# Update and redraw highlight for current closest line
				curr_path_segment = paths[nav.closest_path_index]
				current_line.set_xdata([curr_path_segment[0][0], curr_path_segment[1][0]])
				current_line.set_ydata([curr_path_segment[0][1], curr_path_segment[1][1]])
				axes.draw_artist(current_line)

				# Redraw bike
				figure_blit(axes.bbox)

			# Start the update & refresh timer
			if blitting:
				figure.canvas.new_timer(interval=ANIM_INTERVAL, callbacks=[(full_step, [], {})]).start()
			else:
				FuncAnimation(figure, full_step, frames=xrange(0,200))

			# Display the window with the simulation
			plt.show()

		def find_display_bounds(waypoints):
			"""Given a set of waypoints, return {xlim, ylim} that can fit them."""
			xlim = [99999, -99999] # min, max
			ylim = [99999, -99999] # min, max
			padding = 5
			for waypoint in waypoints:
				if waypoint[0] < xlim[0]:
					xlim[0] = waypoint[0]
				elif waypoint[0] > xlim[1]:
					xlim[1] = waypoint[0]

				if waypoint[1] < ylim[0]:
					ylim[0] = waypoint[1]
				elif waypoint[1] > ylim[1]:
					ylim[1] = waypoint[1]
			xlim, ylim = (xlim[0] - padding, xlim[1] + padding), (ylim[0] - padding, ylim[1] + padding)
			return {"xlim": xlim, "ylim": ylim}

		if __name__ == '__main__':
			new_bike = bikeState.Bike(0, 0, 0.1, 0, 0, 0, 3.57)
			# waypoints = requestHandler.parse_json(True)
			#waypoints = [(0,0), (20, 5), (40, 5)]
			#waypoints = [(0,0), (50, 0)]
			#waypoints = [(0,0), (20, 5), (40, -5), (60, 10), (100, -20), (40, -50), (0,-10), (0, 0)]
			#waypoints = [(40, 0), (20, -10), (0, 0)]
			#waypoints = [(0,0), (50, 0), (50, 50), (0, 50), (0,0)]
			#waypoints = [(0, 0), (10, 0), (20, 5), (25, 15), (12, 20), (0, 15), (0, 0)]
			waypoints = [(0, 0), (20, 0), (40, 10), (50, 30), (24, 40), (0, 30), (0, 0)]
			new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
			new_nav = nav.Nav(new_map_model)
			# print "PATHS", new_nav.map_model.paths

			get_loop_function()(new_nav, new_bike, new_map_model)
</p></details>

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | loop_pyqtgraph(nav, bike, map_model) | Animation using PyQtGraph
void | loop_matplotlib(nav, bike, map_model) | Animation using Matplotlib with adding/removing patches
void | loop_matplotlib_blitting(nav, bike, map_model, blitting=True) | Animation using Matplotlib with blitting and callbacks
dict? | find_display_bounds(waypoints) | Given a set of waypoints, return {xlim, ylim} that can fit them on a graph

			
---

### <a name="mapModel"></a> mapModel.py
Virtual model of the map in which the simulated bike is navigating. Every Route is made up of paths, which are lines between two points.


Class Definition:

Type | Instance Attribute | Description
:-------------: |:-------------:| :-----:
Bike | bike | Bike object
array | paths | Array of path segments. Each element is an array of 2 waypoints that make up a path segment.
array | waypoints | Waypoints that make up the route
array? | obstacles | Obstacles on the map?


  <details>
    <summary><small>bikeState.py-Click to Expand</small></summary><p>  
import bikeState
		import numpy as np
		""" This module contains a class to represent the Map Model and all its functions """

		
		
		class Map_Model(object):


			def __init__(self, bike, waypoints, obstacles, paths = []):
				""" Initializes the Map Model """
				self.bike = bike
				self.paths = self.init_paths(waypoints)
				self.waypoints = waypoints
				self.obstacles = obstacles

			def init_paths(self, waypoints):
				""" Initializes paths fron input waypoints """
				paths = []
				if len(waypoints) < 2:
					return paths
				else:
					for i in range(1, len(waypoints)):
						paths.append((waypoints[i-1], waypoints[i]))
					return paths

			def add_path(self, p1, p2):
				""" Adds a new path from point p1 to point p2 at the end of the path list """
				self.paths.append([p1,p2])
				self.waypoints[0].append(p1[0])
				self.waypoints[0].append(p2[0])
				self.waypoints[1].append(p1[1])
				self.waypoints[1].append(p2[1])


			def add_point(self, p):
				""" Adds a new point p to the list of waypoints. If it is not the first point added in 
				the waypoints list, then a path is also added from the last point in waypoints to the 
				new point p that we add """
				if (len(self.paths) != 0):
					previous_point = self.paths[-1][1]
					self.paths.append([previous_point, p])
				elif (len(self.waypoints[0])==1):
					#If the first point had been added we create the first path from the first point to p
					self.paths.append([(self.waypoints[0][0], self.waypoints[1][0]), p])
				self.waypoints[0].append(p[0])
				self.waypoints[1].append(p[1])


			def close_path(self):
				""" Adds a path from the last point to the first point in the waypoints list """
				self.add_point(self.paths[0][0])


			def draw_circle(self, center, r, n_points, degrees = 2*np.pi):
				""" Draws a circle with given characteristics """
				deg_inc = float(degrees)/n_points
				theta = deg_inc
				p0 = np.array(center) + np.array([r, 0])
				p1 = np.array(center) + np.array([r*np.cos(theta), r*np.sin(theta)])
				self.add_path(p0, p1)
				for i in range(2, n_points+1):
					next_point = np.array(center) + np.array([r*np.cos(i*theta), r*np.sin(i*theta)])
					self.add_point(next_point)


  </p></details>

Methods:

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
void | init_paths(self, waypoints) | Initializes paths fron input waypoints
void | add_path(self, p1, p2) | Adds a new path from point p1 to point p2 at the end of the path list
void | add_point(self, p) | Adds a new point p to the list of waypoints. If it is not the first point, appends a new path
void | close_path(self)| Adds a path from the last point to the first point in the waypoints list
void | draw_circle(self, center, r, n_points, degrees = 2*np.pi) | Draws a circle with given characteristics




---
### <a name="nav"></a> nav.py
Navigation algorithms. Contains methods that return the desired steering angle given a bike state and list of paths.

Methods:

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
void | clamp_steer_angle(self, steerD) | Limits a steer angle to within [-MAX_STEER, +MAX_STEER]
int? | find_closest_path(self, point) | Returns the index of the nearest path to the given point from the list of paths (stored in self.map_model.paths)
float? | pid_controller(self) | Returns a desired steering angle for the bike. Two parts: a regular PID controller, and a component that detects when a turn is coming up so we can start turning early
float? | create_lookahead_correction(self, current_path, bike)| Looks at the path ahead and returns a steering angle correction
float? | quintic_steering_angle(self, dist_error, angle_error, curvature_error, L, s)| Returns the y-coordinate of the quintic polynomial given s as the x-coordinate
float? | quintic(self) | Returns a desired steering angle for the bike based on quintic algorithm
float? | get_steering_angle(self) | Calls another function to calculate the steering angle. This function is part of the external interface of this class. All external users of this class should call this method instead of the other steering-angle-calculation methods.


---
### <a name="requestHandler"></a> requestHandler.py
File that handles obtaining GPS waypoints from website. Users can submit waypoints on a Google Maps model on the website, and this file makes requests to the website to obtain those waypoints.

Team members can sign in and submit waypoints at: https://abserver-168813.appspot.com

Functions: //TODO

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
array? | convert(latitude, longitude) | Mercator map projection
array? | convert2(latitude, longitude) | 
array? | math_convert(latitude, longitude) | Converts latitude and longitude to local coordinates that bike can use to navigate.
void | bearing_from_origin(origin, latitude, longitude)| potato
void | parse_json(presets=False) | Parses JSON response from http request


---
### <a name="gps_assisted_simulator_node"></a> gps_assisted_simulator_node.py
This file is simply simulator_node.py, but it reads from the GPS ROS stream and updates its internal state based on that.

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_bike_from_gps(data) | Takes the incoming data from the GPS and updates our state with it
void | update_graph(data) | 
void | path_parse(data) |
void | listener() |


---
### <a name="navigation_node"></a> navigation_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | callback(data) | 
void | path_parse(data) |
void | update_bike_state(data) |
void | update_bike_xy(data) |
void | keyboard_update(data) |
void | talker() |


---
### <a name="map_node"></a> map_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
array? | setup_dimension() | 
void | map_server() |


---
### <a name="simulator_node"></a> simulator_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_graph(data) |
void | path_parse(data) |
void | listener() | 

---
### <a name="vis_node"></a> vis_node.py

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_graph(data) |
void | path_parse(data) |
void | listener() | 
