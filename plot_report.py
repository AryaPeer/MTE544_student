"""
Plots for the Lab 3 report: EKF estimates vs sensor measurements.

Output (in figures/report/):
  <run>.png                 one 2x3 figure per run: path, heading, v, w, ax, ay
  compare_Q_variation.png   spiral with R = 0.5 and Q = 0.1, 0.5, 1.0
  compare_R_variation.png   spiral with Q = 0.5 and R = 0.1, 0.5, 1.0
  rms_summary.csv           RMS of (measured - estimated) for each run

Run with:  python plot_report.py
"""

import csv
import os

import matplotlib
matplotlib.use("Agg")  # save to files, don't open windows
import matplotlib.pyplot as plt
import numpy as np

OUT_DIR = "figures/report"

# Line styles: measured = solid blue, estimated = dashed orange
MEAS = dict(color="#2a78d6", linestyle="-", linewidth=1.0)
EST = dict(color="#eb6834", linestyle="--", linewidth=1.5)

plt.rcParams.update({
    "font.size": 8,
    "axes.titlesize": 9,
    "axes.labelsize": 8,
    "legend.fontsize": 7,
    "figure.titlesize": 10,
    "axes.grid": True,
    "grid.alpha": 0.35,
})

FINAL = "Q = 0.5, R = 0.01 for odometry and 0.5 for IMU"
KIDNAP = "Q = 0.5, R = 0.5 for odometry and 0.1 for IMU"

# (csv file, output name, figure title)
RUNS = [
    ("data/spiral_QR_variations/spiral_Q0.5_R0.5_baseline.csv", "spiral_Q0.5_R0.5_baseline",
     "Spiral, Q = 0.5, R = 0.5 (baseline)"),
    ("data/spiral_QR_variations/spiral_Q0.1_R0.5.csv", "spiral_Q0.1_R0.5",
     "Spiral, Q = 0.1, R = 0.5"),
    ("data/spiral_QR_variations/spiral_Q1.0_R0.5.csv", "spiral_Q1.0_R0.5",
     "Spiral, Q = 1.0, R = 0.5"),
    ("data/spiral_QR_variations/spiral_Q0.5_R0.1.csv", "spiral_Q0.5_R0.1",
     "Spiral, Q = 0.5, R = 0.1"),
    ("data/spiral_QR_variations/spiral_Q0.5_R1.0.csv", "spiral_Q0.5_R1.0",
     "Spiral, Q = 0.5, R = 1.0"),
    ("data/spiral_final/spiral_final_run1.csv", "spiral_final_run1",
     f"Spiral, final tuning, run 1 ({FINAL})"),
    ("data/spiral_final/spiral_final_run2_long.csv", "spiral_final_run2_long",
     f"Spiral, final tuning, run 2 ({FINAL})"),
    ("data/spiral_final/spiral_final_run3.csv", "spiral_final_run3",
     f"Spiral, final tuning, run 3 ({FINAL})"),
    ("data/point_final/point_final_run1.csv", "point_final_run1",
     f"Point controller, final tuning ({FINAL})"),
    ("data/spiral_kidnap/spiral_kidnap_Q0.5_R0.5-0.5-0.1-0.1_run1.csv", "spiral_kidnap_run1",
     f"Spiral with kidnapping, run 1 ({KIDNAP})"),
    ("data/spiral_kidnap/spiral_kidnap_Q0.5_R0.5-0.5-0.1-0.1_run2.csv", "spiral_kidnap_run2",
     f"Spiral with kidnapping, run 2 ({KIDNAP})"),
]


def read_log(filename):
    """Load a csv from the Logger into a dict of numpy arrays, keyed by column name."""
    with open(filename) as f:
        headers = [h.strip() for h in f.readline().split(",") if h.strip()]
        rows = []
        for line in f:
            values = [v.strip() for v in line.split(",") if v.strip()]
            if len(values) == len(headers):  # skip a half-written last line
                rows.append([float(v) for v in values])

    data = np.array(rows)
    log = {name: data[:, i] for i, name in enumerate(headers)}
    log["t"] = log["stamp"] - log["stamp"][0]
    return log


def plot_path(ax, log, title="Path"):
    # the first spiral run was logged before odom_x/odom_y were added
    if "odom_x" in log:
        ax.plot(log["odom_x"], log["odom_y"], label="Odometry", **MEAS)
    else:
        ax.text(0.03, 0.03, "odometry not logged", transform=ax.transAxes,
                fontsize=7, color="gray", style="italic")
    ax.plot(log["kf_x"], log["kf_y"], label="EKF", **EST)
    ax.plot(0, 0, "ko", markersize=4, label="Start")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="lower right", framealpha=0.9)


def plot_vs_time(ax, log, meas_key, est_key, meas_label, est_label, title, ylabel):
    t = log["t"]
    if meas_key in log:
        ax.plot(t, log[meas_key], label=meas_label, **MEAS)
    else:
        ax.text(0.03, 0.03, "odometry not logged", transform=ax.transAxes,
                fontsize=7, color="gray", style="italic")
    ax.plot(t, log[est_key], label=est_label, **EST)

    ax.set_title(title)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    ax.legend(loc="best", framealpha=0.9)


def plot_run(log, title):
    fig, axes = plt.subplots(2, 3, figsize=(10, 5.6), constrained_layout=True)
    fig.suptitle(title)

    plot_path(axes[0, 0], log)

    # unwrap so the angle doesn't jump between -pi and pi
    if "odom_th" in log:
        log["odom_th_unwrapped"] = np.unwrap(log["odom_th"])
    if "kf_th" in log:
        log["kf_th_unwrapped"] = np.unwrap(log["kf_th"])
        plot_vs_time(axes[1, 0], log, "odom_th_unwrapped", "kf_th_unwrapped",
                     "Odometry", "EKF", "Heading", r"$\theta$ [rad]")
    else:
        axes[1, 0].set_axis_off()

    plot_vs_time(axes[0, 1], log, "odom_v", "kf_vx", "Odometry", "EKF",
                 "Linear velocity", "v [m/s]")
    plot_vs_time(axes[0, 2], log, "odom_w", "kf_w", "Odometry", "EKF",
                 "Angular velocity", r"$\omega$ [rad/s]")
    plot_vs_time(axes[1, 1], log, "imu_ax", "kf_ax", "IMU", r"EKF $\dot{v}$",
                 "Forward acceleration", r"$a_x$ [m/s$^2$]")
    plot_vs_time(axes[1, 2], log, "imu_ay", "kf_ay", "IMU", r"EKF $v\omega$",
                 "Lateral acceleration", r"$a_y$ [m/s$^2$]")
    return fig


def plot_comparison(runs, title):
    """Top row: paths. Bottom row: linear velocity. One column per covariance value."""
    fig, axes = plt.subplots(2, 3, figsize=(10, 5.6), constrained_layout=True)
    fig.suptitle(title)

    for col, (log, label) in enumerate(runs):
        plot_path(axes[0, col], log, f"Path, {label}")
        plot_vs_time(axes[1, col], log, "odom_v", "kf_vx", "Odometry", "EKF",
                     f"Linear velocity, {label}", "v [m/s]")

    # same y range on every velocity plot so they can be compared
    low = min(ax.get_ylim()[0] for ax in axes[1])
    high = max(ax.get_ylim()[1] for ax in axes[1])
    for ax in axes[1]:
        ax.set_ylim(low, high)
    return fig


def rms(error):
    return round(float(np.sqrt(np.mean(error ** 2))), 4)


def rms_summary(log, name):
    row = {"run": name, "duration_s": round(float(log["t"][-1]), 1)}
    for meas, est in [("odom_v", "kf_vx"), ("odom_w", "kf_w"),
                      ("imu_ax", "kf_ax"), ("imu_ay", "kf_ay")]:
        row["rms_" + meas] = rms(log[meas] - log[est]) if meas in log else ""

    if "odom_x" in log:
        dist = np.hypot(log["kf_x"] - log["odom_x"], log["kf_y"] - log["odom_y"])
        row["final_pos_diff_m"] = round(float(dist[-1]), 3)
        row["max_pos_diff_m"] = round(float(dist.max()), 3)
    else:
        row["final_pos_diff_m"] = ""
        row["max_pos_diff_m"] = ""
    return row


def save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name + ".png"), dpi=200)
    plt.close(fig)
    print("saved", name + ".png")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    logs = {}
    summary = []
    for path, name, title in RUNS:
        logs[name] = read_log(path)
        summary.append(rms_summary(logs[name], name))
        save(plot_run(logs[name], title), name)

    save(plot_comparison([(logs["spiral_Q0.1_R0.5"], "Q = 0.1"),
                          (logs["spiral_Q0.5_R0.5_baseline"], "Q = 0.5"),
                          (logs["spiral_Q1.0_R0.5"], "Q = 1.0")],
                         "Spiral: effect of changing Q (R = 0.5)"),
         "compare_Q_variation")
    save(plot_comparison([(logs["spiral_Q0.5_R0.1"], "R = 0.1"),
                          (logs["spiral_Q0.5_R0.5_baseline"], "R = 0.5"),
                          (logs["spiral_Q0.5_R1.0"], "R = 1.0")],
                         "Spiral: effect of changing R (Q = 0.5)"),
         "compare_R_variation")

    with open(os.path.join(OUT_DIR, "rms_summary.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=summary[0].keys())
        writer.writeheader()
        writer.writerows(summary)
    print("saved rms_summary.csv")


if __name__ == "__main__":
    main()
