"""
Contains kalman filter functions that take in raw data and output filtered data.
Originally translated from MATLAB. Math used is explained in great detail at:
https://home.wlu.edu/~levys/kalman_tutorial/
"""

import math
import numpy as np
import geometry
from util.sensor_data import GPSData, BikeData
from util.math_helpers import align_radians
from util.constants import BIKE_LENGTH


# Runs two concurrent Kalman Filters k1 and k2.
# k1 state is (x, y, xdot, ydot).
# k2 state is (yaw, yawdot).


class KalmanFilter:
    def __init__(self, k1_state, k1_P, k2_state, k2_P):
        self.k1_state = k1_state
        self.k1_P = k1_P
        self.k2_state = k2_state
        self.k2_P = k2_P


    def get_x(self):        return self.k1_state[0, 0]

    def get_y(self):        return self.k1_state[1, 0]

    def get_xdot(self):     return self.k1_state[2, 0]

    def get_ydot(self):     return self.k1_state[3, 0]

    def get_yaw(self):      return self.k2_state[0, 0]

    def get_yawdot(self):   return self.k2_state[1, 0]


    def update(self, dt, gps_sensor=None, bike_sensor=None):
        # Sensor covariance matrix.
        k1_R = np.matrix([[200, 0, 0, 0], [0, 50, 0, 0], [0,0,0.01,0], [0,0,0,0.01]])
        k2_R = np.matrix([[2, 0], [0, 0.02]])

        # Hidden state to observed state transformation.
        k1_C = np.identity(4)
        k2_C = np.identity(2)

        # State update matrix.
        k1_A = np.identity(4)
        k1_A[0, 2] = dt
        k1_A[1, 3] = dt

        k2_A = np.identity(2)
        k2_A[0, 1] = dt

        # Predict new state.
        self.k1_state = k1_A * self.k1_state
        self.k1_P = k1_A * self.k1_P * k1_A.T + np.identity(4) * dt

        self.k2_state = k2_A * self.k2_state
        self.k2_P = k2_A * self.k2_P * k2_A.T + np.identity(2) * dt

        # Update state if there are sensor observations.
        if gps_sensor is not None:
            yaw_obs = self.k2_state[0, 0]
            xdot_obs = gps_sensor.speed * np.cos(yaw_obs)
            ydot_obs = gps_sensor.speed * np.sin(yaw_obs)

            k1_z = np.matrix([[gps_sensor.x], [gps_sensor.y], [xdot_obs], [ydot_obs]])

            # Compute kalman gain.
            k1_G = self.k1_P * k1_C.T * (k1_C * self.k1_P * k1_C.T + k1_R).I
            self.k1_state += k1_G * (k1_z - k1_C * self.k1_state)
            self.k1_P = (np.identity(4) - k1_G * k1_C) * self.k1_P

        if bike_sensor is not None:
            yaw_obs = bike_sensor.yaw
            speed_obs = bike_sensor.speed
            yawdot_obs = speed_obs * np.tan(bike_sensor.steer) / BIKE_LENGTH

            # Wrap yaw around in case we've done a full circle.
            self.k2_state[0, 0] = align_radians(self.k2_state[0, 0], center=yaw_obs)

            k2_z = np.matrix([[yaw_obs], [yawdot_obs]])

            # Compute kalman gain.
            k2_G = self.k2_P * k2_C.T * (k2_C * self.k2_P * k2_C.T + k2_R).I
            self.k2_state += k2_G * (k2_z - k2_C * self.k2_state)
            self.k2_P = (np.identity(2) - k2_G * k2_C) * self.k2_P


def kalman_retro_old(raw_state):
    """Kalman filter that is used to retroactively filter previously collected
    raw data. Input raw_state is a 5x1 matrix of raw x, y, yaw, velocity, and time step data.
    
    Returns kalman state, which is a 4x1 matrix of filtered x, y, x_dot, and y_dot data.
    """
    
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    raw_state = np.matrix(raw_state)
    rows = raw_state.shape[0]

    x_pos = raw_state[:,0]
    y_pos = raw_state[:,1]
    yaw = raw_state[:,2]
    v = raw_state[:,3]
    t_step = raw_state[:,4]
    
    v_0 = v.item(0)
    yaw_0 = yaw.item(0)
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    s_current = np.matrix([[x_pos.item(0)], [y_pos.item(0)], [x_dot_0], [y_dot_0]])
    
    P_current = eye4
    #Try this for later tuning
    R = np.matrix([[6.25, 0, 0, 0], [0, 6.25, 0, 0], [0,0,100,0], [0,0,0,100]])
    C = eye4
 
    kalman_state = np.matrix(np.zeros((4,rows), dtype=float))
    
    old_xy = (raw_state.item(0,0), raw_state.item(0,1))
    for i in range(rows):
        
        A = np.matrix([[1, 0 , t_step.item(i)/1000., 0], [0, 1, 0, t_step.item(i)/1000.], [0, 0, 1, 0], [0, 0, 0, 1]]) 
        
        kalman_state[:, i] = s_current
        
        s_new = A*s_current
        P_new = A*P_current*A.T + (np.matrix([[.0001, 0, 0 ,0], [0, .0001, 0, 0], [0,0,.0001,0], [0,0,0,.0001]]))
        
        #Check for outliers
        if geometry.distance((raw_state.item(i,0),raw_state.item(i,1)), old_xy) < 20:
            x_actual = raw_state.item(i, 0)
            y_actual = raw_state.item(i, 1)
            old_xy = (x_actual, y_actual)
        else:
            x_actual = old_xy[0]
            y_actual = old_xy[1]
        
        psi_actual = raw_state.item(i, 2)
        v_actual = raw_state.item(i, 3)
        x_dot_actual = v_actual*np.cos(psi_actual)
        y_dot_actual = v_actual*np.sin(psi_actual)

        z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
        
        G = P_new*C.T*((C*P_new*C.T + R).I)
        s_new = s_new + G*(z - C*s_new)
        P_new = (eye4 - G*C)*P_new
        
        s_current = s_new
        P_current = P_new
    
    return kalman_state.T

def kalman_real_time(raw_state, s_current, P_current):
    """
    Kalman filter that can be run in real time. Input raw_state is a 5x1
    matrix of raw x, y, yaw, velocity, and time step data. s_current is
    the kalman state, usually taken from the previous call to this
    function, and P_current is the observation, also taken from previous
    call to function.

    Returns the new s and P after filter has been run, which can be used
    in the next call to the function as its s_current and P_current. The
    first entry of the tuple is the Kalman state, as a row matrix (x, y,
    x_dot, y_dot). The second entry of the tuple is the prediction
    error, as a 4x4 square matrix.
    """

    #4x4 identity matrix
    eye4 = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    #make sure raw_state is a matrix
    raw_state = np.matrix(raw_state)

    x_pos = raw_state[:,0]
    y_pos = raw_state[:,1]
    yaw = raw_state[:,2]
    v = raw_state[:,3]
    t_step = raw_state[:,4]
    
    v_0 = v.item(0) # initial velocity
    yaw_0 = yaw.item(0) #initial yaw
    x_dot_0 = v_0*np.cos(yaw_0)
    y_dot_0 = v_0*np.sin(yaw_0)
    
    A = np.matrix([[1, 0 , t_step/1000., 0], [0, 1, 0, t_step/1000.], [0, 0, 1, 0], [0, 0, 0, 1]])
    C = eye4 
    R = 4*eye4
 
    s_new = A*s_current
    P_new = A*P_current*A.T + (np.matrix([[.0001, 0, 0 ,0], [0, .0001, 0, 0], [0,0,.0001,0], [0,0,0,.0001]]))
    
    #Check for outliers
    old_xy = (s_current.item(0), s_current.item(1))
    if geometry.distance((raw_state.item(0),raw_state.item(1)), old_xy) < 20:
        x_actual = raw_state.item(0)
        y_actual = raw_state.item(1)
    else:
        x_actual = old_xy[0]
        y_actual = old_xy[1]
        
    psi_actual = raw_state.item(2)
    v_actual = raw_state.item(3)
    x_dot_actual = v_actual*np.cos(psi_actual)
    y_dot_actual = v_actual*np.sin(psi_actual)

    z = np.matrix([[x_actual], [y_actual], [x_dot_actual], [y_dot_actual]])
    
    
    G = P_new*C.T*((C*P_new*C.T + R).I)
    s_new = s_new + G*(z - C*s_new)
    P_new = (eye4 - G*C)*P_new
        
    return (s_new, P_new)
