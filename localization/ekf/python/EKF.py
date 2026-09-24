import numpy as np
import math

dt = 0.1 # timestep

def motion_model(state, control):
    f = np.array([ [1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])

    b = np.array([[dt*math.cos(state[2,0]), 0],
                  [dt*math.sin(state[2,0]), 0],
                  [0, dt]])

    m_model = f @ state + b @ control

    return m_model


def observation_model(state, meas):
    x = state[0,0]
    y = state[1,0]
    theta = state[2,0]

    Ix = meas[0,0]
    Iy = meas[1,0]
    

    r = math.sqrt(((Ix-x)**2)+((Iy-y)**2))
    phi = math.atan2(Iy-y,Ix-x) - theta

    o_model = np.array([[r],
                        [phi]])

    return o_model

def jacob_f(state, control):
    F = np.array([[1, 0, -control[0,0]*math.sin(state[2,0])*dt],
                  [0, 1,  control[0,0]*math.cos(state[2,0])*dt],
                  [0, 0, 1]])

    return F

def jacob_h(state, meas):
    x = state[0,0]
    y = state[1,0]
    theta = state[2,0]
    
    Ix = meas[0,0]
    Iy = meas[1,0]
    

    r = math.sqrt(((Ix-x)**2)+((Iy-y)**2))
    phi = math.atan2(Iy-y,Ix-x) - theta

    H = np.array([[(x-Ix)/r , (y-Iy)/r, 0],
                  [(Iy-y)/r**2 , (x-Ix)/r**2, -1]])

    return H

def EKF_estimate(xEst, PEst, u, z, meas):
    jF = jacob_f(xEst, u)
    # Predict
    xPred = motion_model(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    jH = jacob_h(xPred, meas)
    #Estimation
    zPred = observation_model(xPred, meas)
    y = z - zPred
    y[1, 0] = math.atan2(math.sin(y[1, 0]), math.cos(y[1, 0]))
    s = jH @ PPred @ jH.T + R
    k = PPred @ jH.T @ np.linalg.inv(s)

    xEst = xPred + k @ y
    PEst = (np.eye(len(xEst)) - k @ jH) @ PPred

    return xEst, PEst