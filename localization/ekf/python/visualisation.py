import math 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import EKF # unmodified

EKF.Q = np.diag([0.05, 0.05, np.deg2rad(1.0)]) ** 2 # process noise
EKF.R = np.diag([0.03, np.deg2rad(3.0)]) ** 2  # measurement noise

INPUT_NOISE = np.diag([0.3, np.deg2rad(5.0)]) ** 2
SENSOR_RANGE = 9.0
SIM_TIME = 40.0

LANDMARKS = np.array([
    [5.0, 5.0],
    [-3.0, 6.0],
    [4.0, -4.0],
    [-5.0, -3.0]
])

def calc_input():
    v = 1.0
    omega = 0.15
    return np.array([[v],
                     [omega]])

def nearest_visible_landmark(xTrue):
    x, y = xTrue[0,0], xTrue[1,0]
    best, best_dist = None, None
    for Ix, Iy in LANDMARKS:
        d = math.hypot(Ix-x, Iy-y)
        if d <= SENSOR_RANGE and (best_dist is None or d < best_dist):
            best, best_dist = np.array([[Ix], [Iy]]), d

    return best

def simulate_measurement(xTrue, landmark_xy):
    z_true = EKF.observation_model(xTrue, landmark_xy)
    noise = np.array([
        [np.random.randn() * math.sqrt(EKF.R[0, 0])],
        [np.random.randn() * math.sqrt(EKF.R[1, 1])],
        ])

    return z_true + noise

def plot_covariance_ellipse(x, y, cov_xy, ax, n_std=3.0, **kwargs):
    eigvals, eigvecs = np.linalg.eigh(cov_xy)
    order = eigvals.argsort()[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]
    angle = math.degrees(math.atan2(eigvecs[1, 0], eigvecs[0, 0]))
    width, height = 2 * n_std * np.sqrt(np.maximum(eigvals, 0))
    patch = Ellipse((x, y), width, height, angle=angle, fill=False, **kwargs)
    ax.add_patch(patch)
    return patch  
 
 
def main():
    time = 0.0
 
    xTrue = np.zeros((3, 1))
    xEst = np.zeros((3, 1))
    xDR = np.zeros((3, 1))
    PEst = np.eye(3) * 0.1
 
    hxTrue, hxEst, hxDR = xTrue.copy(), xEst.copy(), xDR.copy()
 
    fig, ax = plt.subplots()
 
    while time <= SIM_TIME:
        time += EKF.dt
        u = calc_input()
 
        xTrue = EKF.motion_model(xTrue, u)
 

        u_noisy = u + np.sqrt(INPUT_NOISE) @ np.random.randn(2, 1)
        xDR = EKF.motion_model(xDR, u_noisy)
 
        landmark_xy = nearest_visible_landmark(xTrue)
 
        if landmark_xy is None:
            # predict-only: EKF_estimate is not called at all
            jF = EKF.jacob_f(xEst, u_noisy)
            xEst = EKF.motion_model(xEst, u_noisy)
            PEst = jF @ PEst @ jF.T + EKF.Q
        else:
            z = simulate_measurement(xTrue, landmark_xy)
            xEst, PEst = EKF.EKF_estimate(xEst, PEst, u_noisy, z, landmark_xy)
 
        hxTrue = np.hstack((hxTrue, xTrue))
        hxEst = np.hstack((hxEst, xEst))
        hxDR = np.hstack((hxDR, xDR))
 
        ax.cla()
        ax.plot(LANDMARKS[:, 0], LANDMARKS[:, 1], "^k", markersize=10, label="landmarks")
        ax.plot(hxTrue[0, :], hxTrue[1, :], "-b", label="ground truth")
        ax.plot(hxDR[0, :], hxDR[1, :], "-k", alpha=0.5, label="dead reckoning")
        ax.plot(hxEst[0, :], hxEst[1, :], "-r", label="EKF estimate")
        ellipse_patch = plot_covariance_ellipse(
            xEst[0, 0], xEst[1, 0], PEst[0:2, 0:2], ax,
            edgecolor="r", linestyle="--", label="EKF covariance (3σ)")
        if landmark_xy is not None:
            ax.plot([xTrue[0, 0], landmark_xy[0, 0]],
                   [xTrue[1, 0], landmark_xy[1, 0]], "g--", alpha=0.4,
                   label="active landmark sighting")
        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
        ax.set_aspect("equal")
        ax.grid(True)
 
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles, labels, loc="upper right", fontsize=8)
 
        plt.pause(0.001)
 
    plt.show()
 
 
if __name__ == "__main__":
    main()