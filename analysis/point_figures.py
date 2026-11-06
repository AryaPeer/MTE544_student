# run this from root with: python3 analysis/point_figures.py

import os
import re

import matplotlib.pyplot as plt

from plot_run import ROOT, read_csv

OUTPUT_DIR = os.path.join(ROOT, "plots")
GOAL = (0.75, 0.75)

RUNS = [
    {"name": "P controller", "folder": "data/point/p_tuned", "terms": "p"},
    {"name": "Tuned PID controller", "folder": "data/point/pid_tuned", "terms": "pvi"},
]

BLUE, ORANGE, AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK = "#0b0b0b"
MUTED = "#52514e"

plt.rcParams.update({
    "axes.grid": True,
    "axes.edgecolor": "#c3c2b7",
    "axes.spines.top": False,
    "axes.spines.right": False,
    "grid.color": "#e1e0d9",
    "lines.linewidth": 1.8,
    "figure.titlesize": 15,
    "figure.titleweight": "bold",
    "savefig.dpi": 200,
})


def load_run(run):
    folder = os.path.join(ROOT, run["folder"])
    data = {name: read_csv(os.path.join(folder, f"{name}.csv")) for name in ("linear", "angular", "robot_pose")}

    t0 = data["robot_pose"]["stamp"][0]
    for log in data.values():
        log["t"] = [(stamp - t0) / 1e9 for stamp in log["stamp"]]

    with open(os.path.join(folder, "gains.txt")) as file:
        gains = {motion + term: value for motion, term, value in re.findall(r"k([la])([pvi])=([\d.]+)", file.read())}

    return {**run, **data, "gains": gains}


def gains_text(run, motions):
    lines = []
    for motion in motions:
        terms = [rf"$K_{term}$ = {run['gains'][motion[0] + term]}" for term in run["terms"]]
        lines.append(f"{motion.capitalize()}: " + ",  ".join(terms))
    return "\n".join(lines)


def new_figure(title, runs, motions, figsize):
    fig, axes = plt.subplots(1, len(runs), figsize=figsize, sharex=True, sharey=True, layout="constrained")
    fig.suptitle(title)

    for ax, run in zip(axes, runs):
        ax.set_title(run["name"], fontweight="bold", pad=10 + 14 * len(motions))
        ax.text(0.5, 1.02, gains_text(run, motions), transform=ax.transAxes, ha="center", va="bottom", color=MUTED, fontsize=10)

    return fig, axes


def save(fig, filename):
    fig.savefig(os.path.join(OUTPUT_DIR, filename))
    plt.close(fig)


def mark_points(ax, x, y, target, target_label):
    ax.plot(x[0], y[0], "o", color=AQUA, markersize=9, markeredgecolor="white", label="Start")
    ax.plot(x[-1], y[-1], "s", color=ORANGE, markersize=9, markeredgecolor="white", label="End")
    ax.plot(*target, "X", color=INK, markersize=10, markeredgecolor="white", label=target_label)


def plot_vs_time(runs, title, log_name, series, ylabel, motions, filename):
    fig, axes = new_figure(title, runs, motions, figsize=(12, 5))

    for ax, run in zip(axes, runs):
        log = run[log_name]
        for column, label, color, style in series:
            ax.plot(log["t"], log[column], color=color, linestyle=style, label=label)
        ax.axhline(0, color="#898781", linewidth=0.8)
        ax.set_xlabel("Time [s]")
        ax.legend()

    axes[0].set_ylabel(ylabel)
    save(fig, filename)


def plot_phase(runs, title, log_name, unit, filename):
    fig, axes = new_figure(title, runs, [log_name], figsize=(12, 5.5))

    for ax, run in zip(axes, runs):
        e, e_dot = run[log_name]["e"], run[log_name]["e_dot"]
        ax.plot(e, e_dot, color=BLUE, label="Error state")
        mark_points(ax, e, e_dot, (0, 0), "Zero error")
        ax.set_xlabel(f"Error $e$ [{unit}]")
        ax.legend()

    axes[0].set_ylabel(rf"Error rate $\dot{{e}}$ [{unit}/s]")
    save(fig, filename)


def plot_trajectory(runs, filename):
    fig, axes = new_figure("Robot trajectory: x vs y", runs, ["linear", "angular"], figsize=(11, 6.5))

    for ax, run in zip(axes, runs):
        x, y = run["robot_pose"]["x"], run["robot_pose"]["y"]
        ax.plot(x, y, color=BLUE, label="Robot path")
        mark_points(ax, x, y, GOAL, "Goal")
        ax.set_xlabel("x [m]")
        ax.set_aspect("equal")
        ax.legend(loc="upper left")

    axes[0].set_ylabel("y [m]")
    save(fig, filename)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    runs = [load_run(run) for run in RUNS]

    plot_vs_time(
        runs, "Angular error and error rate vs time", "angular",
        [("e", "$e$ [rad]", BLUE, "-"), ("e_dot", r"$\dot{e}$ [rad/s]", ORANGE, "-")],
        "Error [rad], error rate [rad/s]", ["angular"], "angular_error_vs_time.png",
    )
    plot_vs_time(
        runs, "Linear error and error rate vs time", "linear",
        [("e", "$e$ [m]", BLUE, "-"), ("e_dot", r"$\dot{e}$ [m/s]", ORANGE, "-")],
        "Error [m], error rate [m/s]", ["linear"], "linear_error_vs_time.png",
    )
    plot_vs_time(
        runs, "Robot pose vs time", "robot_pose",
        [("x", "x [m]", BLUE, "-"), ("y", "y [m]", ORANGE, "-"), ("theta", r"$\theta$ [rad]", AQUA, "-")],
        "Position [m], heading [rad]", ["linear", "angular"], "pose_vs_time.png",
    )
    plot_trajectory(runs, "x_vs_y.png")
    plot_phase(runs, "Angular error rate vs angular error", "angular", "rad", "angular_phase.png")
    plot_phase(runs, "Linear error rate vs linear error", "linear", "m", "linear_phase.png")

    print(f"Saved figures to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
