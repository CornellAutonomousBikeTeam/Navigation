"""
Contains kalman filter functions that take in raw data and output filtered data.
Originally translated from MATLAB. Math used is explained in great detail at:
https://home.wlu.edu/~levys/kalman_tutorial/
"""

import numpy as np
import geometry


# Runs two concurrent Kalman Filters k1 and k2.
# k1 state is (x, y, xdot, ydot).
# k2 state is (yaw, yawdot).
def kalman_retro(sensors, dt=0.01, speed=1.30, bike_length=1.0):
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
    xdot_init = sensors.gps.speed[0] * np.cos(sensors.gps.yaw[0])
    ydot_init = sensors.gps.speed[0] * np.sin(sensors.gps.yaw[0])
    k1_state = np.matrix([[x_init], [y_init], [xdot_init], [ydot_init]])

    yaw_init = sensors.bike.yaw[0]
    yawdot_init = speed * np.tan(sensors.bike.steer[0]) / bike_length
    k2_state = np.matrix([[yaw_init], [yawdot_init]])

    # State estimate covariance matrix.
    k1_P = np.identity(4)
    k2_P = np.identity(2)

    # Sensor covariance matrix.
    k1_R = np.matrix([[50, 0, 0, 0], [0, 50, 0, 0], [0,0,0.01,0], [0,0,0,0.01]])
    k2_R = np.matrix([[2, 0], [0, 0.02]])

    # Hidden state to observed state transformation.
    k1_C = np.identity(4)
    k2_C = np.identity(2)

    # Observed values for output.
    last_observed_x         = None
    last_observed_y         = None
    last_observed_yaw       = None
    last_observed_gps_yaw   = None
    last_observed_yawdot    = None
    last_observed_speed     = None

    raw_output = []

    # Run the filters.
    for t in np.arange(start_time, end_time-dt, dt):
        # State update matrix.
        k1_A = np.identity(4)
        k1_A[0, 2] = dt
        k1_A[1, 3] = dt

        k2_A = np.identity(2)
        k2_A[0, 1] = dt

        # Predict new state.
        # TODO: why do we need to add the identity here?
        k1_state = k1_A * k1_state
        k1_P = k1_A * k1_P * k1_A.T + np.identity(4) * dt

        k2_state = k2_A * k2_state
        k2_P = k2_A * k2_P * k2_A.T + np.identity(2) * dt

        # Check for sensor data during this timestep.
        if sensors.gps.timestamp[gps_index] < t:
            gps_index += 1

            # Update state using sensor observations.
            x_observed = sensors.gps.x[gps_index]
            y_observed = sensors.gps.y[gps_index]
            yaw_observed = k2_state[0, 0]
            xdot_observed = sensors.gps.speed[gps_index] * np.cos(yaw_observed)
            ydot_observed = sensors.gps.speed[gps_index] * np.sin(yaw_observed)

            last_observed_x = x_observed
            last_observed_y = y_observed
            last_observed_gps_yaw = sensors.gps.yaw[gps_index]

            k1_z = np.matrix([[x_observed], [y_observed], [xdot_observed], [ydot_observed]])

            # Compute kalman gain.
            k1_G = k1_P * k1_C.T * (k1_C * k1_P * k1_C.T + k1_R).I
            k1_state += k1_G * (k1_z - k1_C * k1_state)
            k1_P = (np.identity(4) - k1_G * k1_C) * k1_P

            # Catch up to most recent sensor measurement.
            while sensors.gps.timestamp[gps_index] < t:
                gps_index += 1
                #print 'Skipping sensor data, timestep might be too small.'

        if sensors.bike.timestamp[bike_index] < t:
            bike_index += 1

            # Update state using sensor observations.
            yaw_observed = sensors.bike.yaw[bike_index]
            speed_observed = sensors.gps.speed[gps_index]
            yawdot_observed = speed_observed * np.tan(sensors.bike.steer[bike_index]) / bike_length

            last_observed_yaw = yaw_observed
            last_observed_yawdot = yawdot_observed
            last_observed_speed = speed_observed

            # Wrap yaw around in case we've done a full circle.
            delta = k2_state[0, 0] - yaw_observed
            if delta > np.pi:
                k2_state -= 2 * np.pi
            elif delta < -np.pi:
                k2_state += 2 * np.pi

            k2_z = np.matrix([[yaw_observed], [yawdot_observed]])

            # Compute kalman gain.
            k2_G = k2_P * k2_C.T * (k2_C * k2_P * k2_C.T + k2_R).I
            k2_state += k2_G * (k2_z - k2_C * k2_state)
            k2_P = (np.identity(2) - k2_G * k2_C) * k2_P

            # Catch up to most recent sensor measurement.
            while sensors.bike.timestamp[bike_index] < t:
                bike_index += 1
                #print 'Skipping sensor data, timestep might be too small.'

        raw_output.append({
            'timestamp':    t,
            'x':            k1_state[0, 0],
            'y':            k1_state[1, 0],
            'xdot':         k1_state[2, 0],
            'ydot':         k1_state[3, 0],
            'yaw':          k2_state[0, 0],
            'yawdot':       k2_state[1, 0],
            'last_x':       last_observed_x,
            'last_y':       last_observed_y,
            'last_yaw':     last_observed_yaw,
            'last_gps_yaw': last_observed_gps_yaw,
            'last_yawdot':  last_observed_yawdot,
            'last_speed':   last_observed_speed
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
