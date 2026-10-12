import numpy as np
import math

dt = 0.1
n=3
Q = np.diag((0.1,0.1,0.1))

# Motion Model
"""state: (3,1) [x, y, theta]. control: (2,1) [v, w]. Returns (3,1)."""
def motion_model(state, control):
    U = np.array([
                [dt*math.cos(state[2,0]),0],
                [dt*math.sin(state[2,0]),0],
                [0, dt]
                ])

    result = state + U @ control
    return result

# Observation Model
'''
Observation model = [[r, phi]]

r = sqrt((Ix-x)**2 + (Iy-y)**2)
phi = atan2(Iy-y,Ix-x) - theta
'''
def observation_model(state, landmark):
    Ix = landmark[0]
    Iy = landmark[1]

    x = state[0,0]
    y = state[1,0]
    theta = state[2,0]

    r = math.sqrt(((Ix-x)**2) + ((Iy-y)**2))
    phi = math.atan2(Iy-y,Ix-x) - theta
    phi = math.atan2(math.sin(phi), math.cos(phi))
    return np.array([[r],
                     [phi]])

# Generate sigma points
def generate_sigma_points(state, covariance, lamda):
    result = np.empty([n,2*n+1])

    gamma = math.sqrt(n+lamda)

    LLT = np.linalg.cholesky(covariance)
    result[:,0] = state[:,0]
    for i in range(n):
        result[:,i+1] = state[:,0] + gamma*LLT[:,i]
        result[:,i + n + 1] = state[:,0] - gamma*LLT[:,i]

    return result

# Generate weights
def generate_weights(lamda, alpha, beta):
    result = np.zeros([2,2*n+1])

    result[0,0] = lamda/(n+lamda)
    result[1,0] = result[0,0] + (1 - alpha**2 + beta)

    for i in range(1, 2*n+1):
        result[0,i] = result [1,i] = 1/(2*(n+lamda))

    return result

# PREDICTION

# Propogate sigma points through motion model
def propagate_sigma_points(sigmas, control):
    result = np.zeros([n, 2*n+1])
    for i in range(2*n+1):
        result[:,i:i+1] = motion_model(sigmas[:,i:i+1], control)
        result[2,i] = math.atan2(math.sin(result[2,i]), math.cos(result[2,i]))

    return result


# Find Predicted Mean
def predict_Mean(weights, propagated_sigmas):
    result = np.zeros((3,1))
    sin_sum = 0
    cos_sum = 0

    for i in range(2*n+1):

        result += weights[0,i] * propagated_sigmas[:, i:i+1]
        sin_sum += weights[0,i] * math.sin(propagated_sigmas[2,i])
        cos_sum += weights[0,i] * math.cos(propagated_sigmas[2,i])
        
    result[2,0] = math.atan2(sin_sum, cos_sum)
    return result

# Find Predicted Covariance
def predict_Covariance(weights, propagated_sigmas, predicted_Mean):
    result = np.zeros((3,3))
    residual = np.zeros((3,1))

    for i in range(2*n+1):
        residual = propagated_sigmas[:, i:i+1] - predicted_Mean
        residual[2,0] = math.atan2(math.sin(residual[2,0]), math.cos(residual[2,0]))
        result += weights[1,i] * residual @ residual.T

    result += Q

    return result


# ESTIMATION
# Propogate sigma points through observation model
# Find innvoation
# Find innovation covariance
# Find cross covariance
# FInd Kalman gain
# Finf esimated mean
# FInd estimated covariance
