# run this from root with: python3 analysis/plot_run.py --prefix data/point/pid_tuned/

import argparse
import os
import sys

import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import planner as pl


def reference_trajectory(name):
    pl.TRAJECTORY = name
    waypoints = pl.planner(pl.TRAJECTORY_PLANNER).plan()
    return [p[0] for p in waypoints], [p[1] for p in waypoints]


def read_csv(filename):
    with open(filename, 'r') as file:
        lines = [line.strip() for line in file if line.strip() != ""]

    headers = [h.strip() for h in lines[0].split(',') if h.strip() != ""]
    columns = {h: [] for h in headers}

    for line in lines[1:]:
        values = [v.strip() for v in line.split(',') if v.strip() != ""]
        if len(values) != len(headers):
            continue
        for h, v in zip(headers, values):
            columns[h].append(float(v))

    return columns


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Plot linear/angular error and the robot trajectory.")
    parser.add_argument("--prefix", default="", help='Prefix of the CSV files, e.g. "data/sim/P_"')
    reference = parser.add_mutually_exclusive_group()
    reference.add_argument("--parabola", action="store_true", help="Draw the reference parabola on the x-y plot")
    reference.add_argument("--sigmoid", action="store_true", help="Draw the reference sigmoid on the x-y plot")
    args = parser.parse_args()

    lin = read_csv(args.prefix + "linear.csv")
    ang = read_csv(args.prefix + "angular.csv")

    pose_file = args.prefix + "robot_pose.csv"
    if not os.path.isfile(pose_file):
        pose_file = args.prefix + "pose.csv"
    pose = read_csv(pose_file)

    t0 = min(lin["stamp"][0], ang["stamp"][0])
    t_lin = [(s - t0) / 1e9 for s in lin["stamp"]]
    t_ang = [(s - t0) / 1e9 for s in ang["stamp"]]

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
