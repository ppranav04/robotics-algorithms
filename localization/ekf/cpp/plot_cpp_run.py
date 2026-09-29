"""
Plots the trajectory written by the C++ demo (ekf_cpp_run.csv), in the
same style as visualize_ekf.py, so the C++ run can be checked by eye
against the Python one on the same scenario.

Usage:
    python plot_cpp_run.py [path/to/ekf_cpp_run.csv]
"""

import sys
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

LANDMARKS = np.array([
    [5.0, 5.0],
    [-3.0, 6.0],
    [4.0, -4.0],
    [-5.0, -3.0],
])


def plot_covariance_ellipse(x, y, p00, p11, ax, n_std=3.0, **kwargs):
    # demo.cpp only logs the diagonal of P, not the off-diagonal terms,
    # so this draws an axis-aligned ellipse (no tilt) -- a simplification
    # relative to visualize_ekf.py's full-covariance ellipse.
    width = 2 * n_std * math.sqrt(max(p00, 0))
    height = 2 * n_std * math.sqrt(max(p11, 0))
    ax.add_patch(Ellipse((x, y), width, height, fill=False, **kwargs))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "ekf_cpp_run.csv"
    data = np.genfromtxt(path, delimiter=",", names=True)

    x_true = data["x_true"]
    y_true = data["y_true"]
    x_dr = data["x_dr"]
    y_dr = data["y_dr"]
    x_est = data["x_est"]
    y_est = data["y_est"]
    p00 = data["p00"]
    p11 = data["p11"]
    landmark_idx = data["landmark"]

    # Position RMSE against ground truth, same metric evaluate_ekf.py uses.
    ekf_err = np.hypot(x_est - x_true, y_est - y_true)
    dr_err = np.hypot(x_dr - x_true, y_dr - y_true)
    print(f"C++ EKF   RMSE = {math.sqrt(np.mean(ekf_err**2)):.3f} m   "
          f"max = {ekf_err.max():.3f} m   final = {ekf_err[-1]:.3f} m")
    print(f"Dead reck RMSE = {math.sqrt(np.mean(dr_err**2)):.3f} m   "
          f"max = {dr_err.max():.3f} m   final = {dr_err[-1]:.3f} m")

    fig, ax = plt.subplots()
    ax.plot(LANDMARKS[:, 0], LANDMARKS[:, 1], "^k", markersize=10, label="landmarks")
    ax.plot(x_true, y_true, "-b", label="ground truth")
    ax.plot(x_dr, y_dr, "-k", alpha=0.5, label="dead reckoning")
    ax.plot(x_est, y_est, "-r", label="EKF estimate (C++)")

    plot_covariance_ellipse(x_est[-1], y_est[-1], p00[-1], p11[-1], ax,
                            edgecolor="r", linestyle="--",
                            label="EKF covariance (3σ, diagonal only)")

    # Mark timesteps where a landmark was actually seen.
    seen = landmark_idx >= 0
    ax.plot(x_true[seen], y_true[seen], ".", color="green", alpha=0.3,
            markersize=3, label="landmark sighting")

    ax.set_xlim(-12, 12)
    ax.set_ylim(-12, 12)
    ax.set_aspect("equal")
    ax.grid(True)
    ax.set_title("C++ EKF run")
    ax.legend(loc="upper right", fontsize=8)
    plt.show()


if __name__ == "__main__":
    main()