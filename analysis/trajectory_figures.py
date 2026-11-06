# Saves x-y plots of the parabola and sigmoid runs against their reference curves to plots/
#
# Usage (run this from root):
#   python3 analysis/trajectory_figures.py

import os

import matplotlib.pyplot as plt
import numpy as np

from plot_run import read_csv
from point_figures import AQUA, BLUE, ORANGE, OUTPUT_DIR, ROOT  # also applies the shared plot style

AXIS_LIMITS = (-0.1, 2.6)


def plot_trajectory(title, folder, reference_label, reference, x_max, filename):
    pose = read_csv(os.path.join(ROOT, folder, "robot_pose.csv"))
    x, y = pose["x"], pose["y"]
    reference_x = np.linspace(0, x_max, 200)

    fig, ax = plt.subplots(figsize=(7, 7), layout="constrained")
    ax.plot(x, y, color=BLUE, label="Robot path")
    ax.plot(reference_x, reference(reference_x), color="black", linestyle=":", label=reference_label)
    ax.plot(x[0], y[0], "o", color=AQUA, markersize=9, markeredgecolor="white", label="Start")
    ax.plot(x[-1], y[-1], "s", color=ORANGE, markersize=9, markeredgecolor="white", label="End")

    ax.set_title(title, fontsize=15, fontweight="bold")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(AXIS_LIMITS)
    ax.set_ylim(AXIS_LIMITS)
    ax.set_aspect("equal")
    ax.legend(loc="lower right")

    fig.savefig(os.path.join(OUTPUT_DIR, filename))
    plt.close(fig)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    plot_trajectory(
        "Parabola trajectory: x vs y", "data/trajectory/parabola", "Reference: $y = x^2$",
        lambda x: x**2, 1.5, "parabola_trajectory.png",
    )
    plot_trajectory(
        "Sigmoid trajectory: x vs y", "data/trajectory/sigmoid", r"Reference: $y = \frac{2}{1 + e^{-2x}} - 1$",
        lambda x: 2 / (1 + np.exp(-2 * x)) - 1, 2.5, "sigmoid_trajectory.png",
    )

    print(f"Saved trajectory plots to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
