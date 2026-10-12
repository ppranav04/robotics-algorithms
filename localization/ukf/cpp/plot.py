import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

HERE = Path(__file__).resolve().parent          # .../ukf/cpp
IMG = HERE.parent / "img"                       # .../ukf/img
IMG.mkdir(exist_ok=True)

RUNS = 50                                       # must match kRuns in demo.cpp
LANDMARKS = np.array([(8, 3), (-8, 3), (0, 10), (0, -5)])   # must match demo.cpp

d = np.genfromtxt(HERE / "run0.csv", delimiter=",", names=True)
n = np.genfromtxt(HERE / "nees.csv", delimiter=",", names=True)

# ---- 1) trajectory: truth vs estimate -------------------------------------
fig, ax = plt.subplots(figsize=(6, 6))
ax.plot(d["true_x"], d["true_y"], lw=2, label="ground truth")
ax.plot(d["est_x"], d["est_y"], "--", lw=1.5, label="UKF estimate")
ax.scatter(LANDMARKS[:, 0], LANDMARKS[:, 1], marker="*", s=160, c="k", zorder=3, label="landmarks")
ax.scatter(d["true_x"][0], d["true_y"][0], c="green", s=60, zorder=4, label="start")
ax.set_aspect("equal")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("UKF trajectory estimate (run 0)")
ax.grid(alpha=0.3)
ax.legend(loc="lower right")
fig.tight_layout()
fig.savefig(IMG / "ukf_trajectory.png", dpi=150)

# ---- 2) consistency: mean NEES over runs ----------------------------------
# Approximate 95% band for the mean of RUNS chi-square(3) samples (normal approximation)
half = 1.96 * np.sqrt(2 * 3 / RUNS)

fig, ax = plt.subplots(figsize=(7, 3.8))
ax.axhspan(3 - half, 3 + half, color="gray", alpha=0.2, label="approx. 95% band")
ax.plot(n["k"], n["mean_nees"], lw=1.2, label=f"mean NEES ({RUNS} runs)")
ax.axhline(3, c="k", ls="--", lw=1, label="expected = 3")
ax.set_ylim(0, 6)
ax.set_xlabel("step")
ax.set_ylabel("NEES")
ax.set_title("Filter consistency (NEES)")
ax.grid(alpha=0.3)
ax.legend(loc="upper right")
fig.tight_layout()
fig.savefig(IMG / "ukf_nees.png", dpi=150)

print("saved:", IMG / "ukf_trajectory.png", "and", IMG / "ukf_nees.png")
plt.show()