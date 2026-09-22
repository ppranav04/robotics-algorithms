import numpy as np
import math

# Motion model:
'''
xt+1 = xt + v.dt.cos(yaw)
yt+1 = yt + v.dt.sin(yaw)
yaw t+1 = yaw + ang.vel*dt
vt+1 = vcmd
'''

dt = 1.0 # step size (time)

def motion_model(x, u):
    F = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 0]])

    B = np.array([[dt*math.cos(x[2,0]), 0],
                  [dt*math.sin(x[2,0]), 0],
                  [0, dt],
                  [1, 0]])

    x = F @ x + B @ u

    return x

# Observation Model
'''
zt = [[x], -> measurement
      [y]]
    
zt = h(x) 
'''

def observation_model(x):
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])

    x = H @ x

    return x

def jacob_f(x, u):
    v = x[3,0]
    yaw = x[2,0]
    '''We take the jacobian of the motion model which by handwritten calculations
        comes to be:
        [1, 0, -v*dt*math.sin(yaw), dt*math.cos(yaw)],
        [0, 1 , v*dt*math.cos(yaw), dt*math.sin(yaw)]
        [0, 0, 1, 0],
        [0, 0, 0, 0],
    '''
    jF = np.array([[1, 0, -v*dt*math.sin(yaw), dt*math.cos(yaw)],
                   [0, 1 , v*dt*math.cos(yaw), dt*math.sin(yaw)],
                   [0, 0, 1, 0],
                   [0, 0, 0, 0]])

    return jF

def jacob_h():
    '''
        We take the jacobian of the observation model and it's a 4x2 matrix 
        H = [1, 0, 0, 0],
            [0, 1, 0, 0]
        Alright so it is a 2x4 matrix because we are doing a state (input):
        [x,
         y]
        to measurement (output) that is the state variables
    '''
    jH = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])

    return jH

# Now the pre-requisites are done now let's start EKF
