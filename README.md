# Navigation
Repo for the nav code- .gitignore only has python

## Documentation
* [bikeSim.py](#bikeSim)
* [bikeState.py](#bikeState)
* [constants.py](#constants)
* [geometry.py](#geometry)
* [loop.py](#loop)
* [mapModel.py](#mapModel)
* [nav.py](#nav)
* [requestHandler.py](#requestHandler)
* [navigation_node.py](#navigation_node)
* [map_node.py](#map_node)
* [simulator_node.py](#simulator_node)
* [vis_node.py](#vis_node)

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
    <summary><small>Bike_State.h-Click to Expand</small></summary><p>
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

---

### <a name="constants"></a> constants.py
Definitions of various constants for Navigation code.

Constant Name | Significance | Usage
:-------------: |:-------------:| :-----:
G | | 
L | |
B | | 
H | |
C | |
K1 | |
K2 | |
K3 | |
TIMESTEP | |
PAUSE | | 
MAX_Steer| |

---
### <a name="geometry"></a> geometry.py
Module that contains many convenience methods that perform mathematical calculations. Used in __

Methods:

Return Type | Method Signature | Description
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


---

### <a name="mapModel"></a> mapModel.py


---
### <a name="nav"></a> nav.py


---
### <a name="requestHandler"></a> requestHandler.py


---
### <a name="navigation_node"></a> navigation_node.py


---
### <a name="map_node"></a> map_node.py


---
### <a name="simulator_node"></a> simulator_node.py


---
### <a name="vis_node"></a> vis_node.py
