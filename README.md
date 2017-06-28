# Navigation- UNSTABLE
Master branch should have stable code only. If unstable, write unstable at the top.
Repo for the nav code- .gitignore only has python

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

Methods:

 Return Type  | Method Signature | Description 
:-------------: |:-------------:| :-----:
Bike object| new_state(bike, steerD)| Returns: next bikeState given current state [bike] and direction [steerD]

---

### <a name="bikeState"></a> bikeState.py
Defines the Bike object which represents the autonomous bike for the purposes of navigation. Not to be confused with Bike_State class in Arduino code.

Class Definition:

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
  
  Methods: //TODO

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
array? | vector(self) | Returns the bike's unit vector
void | update(self, bike) | Updates the bike state
 | rhs(self, steerD) | Modifies state object to turn it into the next state using bike dynamics. Equivalent to rhs in Matlab


---

### <a name="constants"></a> constants.py
Definitions of various constants for Navigation code. //TODO

Constant Names | Significance | Used In
:-------------: |:-------------:| :-----:
G, L, B, H | Constants from the Matlab bike dynamics simulation | bikeSim.py
C | |
K1 | |
K2 | |
K3 | |
TIMESTEP | |
PAUSE | | 
MAX_Steer| Bike's maximumum steer angle, used in clamping steerD | nav.py
PID_DIST_GAIN | | nav.py
PID_ANGLE_GAIN | | nav.py
RAD_TO_DEG | Ratio of degrees to radians to degrees | nav.py
TURN_LOOKAHEAD_DIST | | nav.py
NEXT_TURN_GAIN | | nav.py
BIKE_LENGTH | | nav.py
QUINTIC_LOOKAHEAD | | nav.py
QUINTIC_SAMPLE_LENGTH | | nav.py
BASE_LOOKAHEAD | | nav.py
LOOKAHEAD_ANGLE_GAIN| | nav.py
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
float? | angle_between_vectors(v1, v2) | Returns: angel between vectors [v1] and [v2]. Used in angle_between_two_lines
float? | angle_between_two_lines(line1, line2) | Returns: angle between [line1] and [line2]
float? | dot_product(v1, v2) | Returns: mathematical dot product of vectors [v1] and [v2]
---
### <a name="loop"></a> loop.py
Main Navigation file. Runs a loop that gets navigation command, passes it thorugh simulation, and gets the new updated state.

Main loop:

  <details>
    <summary><small>loop.py-Click to Expand</small></summary><p>
		
		
		
		
		import nav
		import mapModel
		import math
		import bikeState
		import bikeSim
		from constants import *
		import geometry
		import requestHandler

		import matplotlib
		from matplotlib import pyplot as plt
		from matplotlib import animation
		from matplotlib import collections  as mc

		def main_loop(nav, bike):
			""" This is the main loop that gets the nav command, passes it through bike dynamics
			to get the updated state and plots the new state """
			k = 0 
			steerD = 0
			iters = 0

			while (k < 2000):

				#plotting
				plt.scatter(bike.xB, bike.yB)
				plt.show()
				plt.pause(0.0000001)

				# Steer vs Time
				# plt.scatter((k+1)*TIMESTEP, bike.delta)
				# plt.show()
				# plt.pause(0.00001)

				# if (nav.close_enough()):
				# 	steerD = nav.controller_direction_to_turn() #pd cotnroller takes over
				# 	print "pd Controller takes over"
				# else:
				# 	steerD = nav.direction_to_turn()
				# # 	steerD = steerD * MAX_STEER * (-1)
				# if iters == 85:
				# 	iters = 0
				# 	steerD = nav.controller_direction_to_turn()
				# iters+=1
				print "STEER D IS", steerD
				# steerD = nav.controller_direction_to_turn() #pd cotnroller takes over
				steerD = nav.controller_direction_to_turn()
				# print steerD
				# if new state has new target path then recalculate delta
				bike.update(bikeSim.new_state(bike, steerD))
				# if k == 20:
				# 	print "HELLOOOO", nav.calc_overshoot()
					# print "HELLOOOO", nav.calc_overshoot()
				# path_angle = geometry.line_angle(nav.map_model.paths[nav.target_path])
				# bike_angle = nav.map_model.bike.pi

				# print "ANGLE BETWEEN", math.fabs(path_angle - bike_angle)

				k = k + 1

				# When it crosses the line... ?


		if __name__ == '__main__':
			
			new_bike = bikeState.Bike(0, 0, 0.1, 0, math.pi/6.0, 0, 3.57)
			# waypoints = requestHandler.parse_json(True)
			# waypoints = [(0,0), (20, 5), (40, 5)]
			waypoints = [(0,0), (50, 5)]
			new_map_model = mapModel.Map_Model(new_bike, waypoints, [], [])
			new_nav = nav.Nav(new_map_model)
			print "PATHS", new_nav.map_model.paths
			# print new_nav.direction_to_turn()
			# print new_bike.rhs(new_nav.direction_to_turn())
			# PLOTTING
			plt.ion() # enables interactive plotting
			paths = new_map_model.paths
			fig = plt.figure()
			fig.set_dpi(100) #dots per inch
		 	ax = plt.axes(xlim=(0, 20), ylim=(0, 20)) 
		 	lc = mc.LineCollection(paths, linewidths=2, color = "black")
			ax.add_collection(lc)
			plt.show() 
			main_loop(new_nav, new_bike)
</p></details>
			
---

### <a name="mapModel"></a> mapModel.py
Virtual model of the map in which the simulated bike is navigating. Every Route is made up of paths, which are lines between two points.


Class Definition:

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

Methods: //TODO

Return Type | Method Signature | Description
:-------------: |:-------------:| :-----:
array? | math_convert(latitude, longitude) | Converts latitude and longitude to local coordinates that bike can use to navigate.
void | bearing_from_origin(origin, latitude, longitude)| potato
void | parse_json(presets=False) | Parses JSON response from http request


---
### <a name="gps_assisted_simulator_node"></a> gps_assisted_simulator_node.py
This file is simply simulator_node.py, but it reads from the GPS ROS stream and updates its internal state based on that.

Functions:

Return Type | Function Signature | Description
:-------------: |:-------------:| :-----:
void | update_bike_from_gps(data) | Takes the incoming daata from the GPS and updates our state with it
void | update_graph(data) | 
void | path_parse(data) |
void | listener() |


---
### <a name="navigation_node"></a> navigation_node.py


---
### <a name="map_node"></a> map_node.py


---
### <a name="simulator_node"></a> simulator_node.py


---
### <a name="vis_node"></a> vis_node.py
