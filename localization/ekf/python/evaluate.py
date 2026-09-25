

import math
import numpy as np
import matplotlib.pyplot as plt

import EKF  # unmodified

EKF.Q = np.diag([0.05, 0.05, np.deg2rad(1.0)]) ** 2
EKF.R = np.diag([0.3, np.deg2rad(3.0)]) ** 2

INPUT_NOISE = np.diag([0.3, np.deg2rad(5.0)]) ** 2
SENSOR_RANGE = 8.0
SIM_TIME = 80.0

LANDMARKS = np.array([
    [5.0, 5.0],
    [-3.0, 6.0],
    [4.0, -4.0],
    [-5.0, -3.0],
])


def calc_input():
    return np.array([[1.0], [0.15]])


def nearest_visible_landmark(xTrue):
    x, y = xTrue[0, 0], xTrue[1, 0]
    best, best_dist = None, None
    for lx, ly in LANDMARKS:
        d = math.hypot(lx - x, ly - y)
        if d <= SENSOR_RANGE and (best_dist is None or d < best_dist):
            best, best_dist = np.array([[lx], [ly]]), d
    return best


def simulate_measurement(xTrue, landmark_xy):
    z_true = EKF.observation_model(xTrue, landmark_xy)
    noise = np.array([
        [np.random.randn() * math.sqrt(EKF.R[0, 0])],
        [np.random.randn() * math.sqrt(EKF.R[1, 1])],
    ])
    return z_true + noise


def position_error(est, true):
    """Euclidean distance between estimated and true (x, y) -- ignores
    heading, since heading error isn't directly comparable in meters."""
    return math.hypot(est[0, 0] - true[0, 0], est[1, 0] - true[1, 0])


def run_once(seed=None):
    if seed is not None:
        np.random.seed(seed)

    time = 0.0
    xTrue = np.zeros((3, 1))
    xEst = np.zeros((3, 1))
    xDR = np.zeros((3, 1))
    PEst = np.eye(3) * 0.1

    times, ekf_err, dr_err = [], [], []

    while time <= SIM_TIME:
        time += EKF.dt
        u = calc_input()

        xTrue = EKF.motion_model(xTrue, u)

        u_noisy = u + np.sqrt(INPUT_NOISE) @ np.random.randn(2, 1)
        xDR = EKF.motion_model(xDR, u_noisy)

        landmark_xy = nearest_visible_landmark(xTrue)
        if landmark_xy is None:
            jF = EKF.jacob_f(xEst, u_noisy)
            xEst = EKF.motion_model(xEst, u_noisy)
            PEst = jF @ PEst @ jF.T + EKF.Q
        else:
            z = simulate_measurement(xTrue, landmark_xy)
            xEst, PEst = EKF.EKF_estimate(xEst, PEst, u_noisy, z, landmark_xy)

        times.append(time)
        ekf_err.append(position_error(xEst, xTrue))
        dr_err.append(position_error(xDR, xTrue))

    return np.array(times), np.array(ekf_err), np.array(dr_err)


def summarize(name, err):
    rmse = math.sqrt(np.mean(err ** 2))
    print(f"{name:14s}  RMSE = {rmse:6.3f} m   "
          f"max = {err.max():6.3f} m   final = {err[-1]:6.3f} m")
    return rmse


def main():
    times, ekf_err, dr_err = run_once(seed=0)

    print(f"\nSingle run over {SIM_TIME:.0f}s ({len(times)} steps):\n")
    summarize("EKF", ekf_err)
    summarize("Dead reckoning", dr_err)

    fig, ax = plt.subplots()
    ax.plot(times, dr_err, "-k", alpha=0.6, label="dead reckoning error")
    ax.plot(times, ekf_err, "-r", label="EKF error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error vs. ground truth [m]")
    ax.set_title("Localization error over time: EKF vs. dead reckoning")
    ax.legend()
    ax.grid(True)
    plt.show()


if __name__ == "__main__":
    main()