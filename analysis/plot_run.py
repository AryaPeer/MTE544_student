# Plots the logs of a run of decisions.py:
#   Figure 1: linear and angular error over time   (from linear.csv and angular.csv)
#   Figure 2: robot trajectory in the x-y plane    (from robot_pose.csv)
#
# Usage:
#   python3 plot_point_tracking.py                         # logs in this folder
#   python3 plot_point_tracking.py --prefix "sim data/P_"  # uses P_linear.csv, P_angular.csv, P_pose.csv
#   python3 plot_point_tracking.py --parabola              # also draws the reference parabola on the x-y plot
#   python3 plot_point_tracking.py --sigmoid               # also draws the reference sigmoid on the x-y plot

import argparse
import os

import matplotlib.pyplot as plt

import planner as pl


def reference_trajectory(name):
    # Gets the waypoints from planner.py so the reference matches what the robot was given
    pl.TRAJECTORY = name
    waypoints = pl.planner(pl.TRAJECTORY_PLANNER).plan()
    return [p[0] for p in waypoints], [p[1] for p in waypoints]


def read_csv(filename):
    # Reads the Logger format: a header line, then rows ending with a trailing ", "
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip() != ""]

    headers = [h.strip() for h in lines[0].split(',') if h.strip() != ""]
    columns = {h: [] for h in headers}

    for line in lines[1:]:
        values = [v.strip() for v in line.split(',') if v.strip() != ""]
        if len(values) != len(headers):
            continue  # skip a partially written last line
        for h, v in zip(headers, values):
            columns[h].append(float(v))

    return columns


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Plot linear/angular error and the robot trajectory.")
    parser.add_argument("--prefix", default="", help='Prefix of the CSV files, e.g. "sim data/P_"')
    reference = parser.add_mutually_exclusive_group()
    reference.add_argument("--parabola", action="store_true", help="Draw the reference parabola from planner.py on the x-y plot")
    reference.add_argument("--sigmoid", action="store_true", help="Draw the reference sigmoid from planner.py on the x-y plot")
    args = parser.parse_args()

    lin = read_csv(args.prefix + "linear.csv")
    ang = read_csv(args.prefix + "angular.csv")

    # The live run writes robot_pose.csv, the sim logs were saved as <prefix>pose.csv
    pose_file = args.prefix + "robot_pose.csv"
    if not os.path.isfile(pose_file):
        pose_file = args.prefix + "pose.csv"
    pose = read_csv(pose_file)

    # Stamps are in nanoseconds; convert to seconds from the start of the run
    t0 = min(lin["stamp"][0], ang["stamp"][0])
    t_lin = [(s - t0) / 1e9 for s in lin["stamp"]]
    t_ang = [(s - t0) / 1e9 for s in ang["stamp"]]

    # Figure 1: errors over time (separate axes since linear is in m and angular in rad)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("Errors over time")

    axes[0].plot(t_lin, lin["e"], color="#2a78d6", linewidth=2, label="linear error")
    axes[0].set_title("Linear error vs time")
    axes[0].set_ylabel("e [m]")

    axes[1].plot(t_ang, ang["e"], color="#eb6834", linewidth=2, linestyle="-.", label="angular error")
    axes[1].set_title("Angular error vs time")
    axes[1].set_ylabel("e [rad]")
    axes[1].set_xlabel("time [s]")

    for ax in axes:
        ax.axhline(0.0, color="#8a8984", linewidth=1, linestyle="--", label="zero error")
        ax.grid(True)
        ax.legend()
    fig.tight_layout()

    # Figure 2: trajectory of the robot in the x-y plane
    fig, ax = plt.subplots(figsize=(7, 7))
    if args.parabola or args.sigmoid:
        name = "parabola" if args.parabola else "sigmoid"
        ref_x, ref_y = reference_trajectory(name)
        ax.plot(ref_x, ref_y, color="black", linewidth=1.5, linestyle="--", marker=".", markersize=4, label=f"reference {name}")
    ax.plot(pose["x"], pose["y"], color="#2a78d6", linewidth=2, label="robot trajectory")
    ax.plot(pose["x"][0], pose["y"][0], marker="o", markersize=9, color="#1baf7a", linestyle="none", label="start")
    ax.plot(pose["x"][-1], pose["y"][-1], marker="s", markersize=9, color="#eb6834", linestyle="none", label="end")
    ax.set_title("Robot trajectory (x-y)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()

    plt.show()
