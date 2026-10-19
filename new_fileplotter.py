import argparse
import csv
import os

import numpy as np
import matplotlib.pyplot as plt

ANGLE_MIN = -np.pi
RANGE_MIN = 0.15
SMOOTH_SECONDS = 1.0


def read_csv(filename):
    with open(filename, newline="") as f:
        reader = csv.reader(f, skipinitialspace=True)
        headers = [h.strip() for h in next(reader) if h.strip()]
        rows = [[float(c) for c in row if c.strip()] for row in reader if row]
    return headers, np.array(rows)


def seconds_since_start(stamps):
    return (stamps - stamps[0]) * 1e-9


def moving_average(values, time):
    window = max(int(round(SMOOTH_SECONDS / np.median(np.diff(time)))), 1) | 1
    kernel = np.ones(window)
    return np.convolve(values, kernel, "same") / np.convolve(np.ones_like(values), kernel, "same")


def save(fig, filename, suffix):
    fig.tight_layout()
    out = os.path.splitext(filename)[0] + "_" + suffix + ".png"
    fig.savefig(out, dpi=150)
    print("saved", out)


def plot_imu(filename, headers, data):
    time = seconds_since_start(data[:, -1])
    signals = [
        ("acc_x", "Linear acceleration x", "Acceleration [m/s²]"),
        ("acc_y", "Linear acceleration y", "Acceleration [m/s²]"),
        ("angular_z", "Angular velocity z", "Angular velocity [rad/s]"),
    ]

    fig, axes = plt.subplots(len(signals), 1, sharex=True, figsize=(9, 8))
    for ax, (name, title, ylabel) in zip(axes, signals):
        values = data[:, headers.index(name)]
        ax.plot(time, values, alpha=0.35, linewidth=0.8, label=name + " (raw)")
        ax.plot(time, moving_average(values, time), color="tab:red", linewidth=2,
                label=name + " (moving average)")
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("IMU data: " + os.path.basename(filename))
    save(fig, filename, "plot")


def plot_odom(filename, headers, data):
    time = seconds_since_start(data[:, -1])
    x = data[:, headers.index("x")]
    y = data[:, headers.index("y")]
    theta = np.unwrap(data[:, headers.index("th")])

    signals = [
        (x, "x", "Position x", "x [m]"),
        (y, "y", "Position y", "y [m]"),
        (theta, "θ", "Orientation (yaw)", "θ [rad]"),
    ]

    fig, axes = plt.subplots(len(signals), 1, sharex=True, figsize=(9, 8))
    for ax, (values, label, title, ylabel) in zip(axes, signals):
        ax.plot(time, values, label=label)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Odometry data: " + os.path.basename(filename))
    save(fig, filename, "timeseries")

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(x, y, color="tab:blue", s=8)
    ax.plot(x[0], y[0], "go", markersize=10, label="start")
    ax.plot(x[-1], y[-1], "8", color="red", markeredgecolor="white", markeredgewidth=2, markersize=14, label="end")
    ax.set_title("Odometry trajectory: " + os.path.basename(filename))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.legend()
    save(fig, filename, "trajectory")


def plot_laser(filename, data, scan_index):
    ranges = data[:, :-2]
    angle_increment = data[0, -2]
    time = seconds_since_start(data[:, -1])

    if scan_index is None:
        scan_index = len(data) // 2
    scan = ranges[scan_index]

    angles = ANGLE_MIN + np.arange(len(scan)) * angle_increment
    in_range = np.isfinite(scan) & (scan >= RANGE_MIN)
    x = scan[in_range] * np.cos(angles[in_range])
    y = scan[in_range] * np.sin(angles[in_range])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))

    ax1.plot(np.degrees(angles[in_range]), scan[in_range], ".", markersize=3, label="In range")
    ax1.plot(np.degrees(angles[~in_range]), np.zeros((~in_range).sum()), "|",
             color="tab:red", alpha=0.5, label="Out of range")
    ax1.set_title("Raw scan (range vs. beam angle)")
    ax1.set_xlabel("Beam angle [deg]")
    ax1.set_ylabel("Range [m]")
    ax1.grid(True)
    ax1.legend(loc="upper right")

    ax2.scatter(x, y, color="tab:blue", s=8)
    ax2.plot(0, 0, "r^", markersize=12, label="Robot")
    ax2.set_title("Cartesian points, scan %d (t = %.1f s)" % (scan_index, time[scan_index]))
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    ax2.set_aspect("equal", adjustable="datalim")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    fig.suptitle("Laser scan: " + os.path.basename(filename))
    save(fig, filename, "scan%d" % scan_index)


def plot_file(filename, scan_index):
    headers, data = read_csv(filename)
    if "ranges" in headers:
        plot_laser(filename, data, scan_index)
    elif "acc_x" in headers:
        plot_imu(filename, headers, data)
    elif "th" in headers:
        plot_odom(filename, headers, data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot logged sensor data.")
    parser.add_argument("--files", nargs="+", required=True, help="CSV files to plot")
    parser.add_argument("--scan-index", type=int, default=None, help="Laser scan (row) to plot")
    args = parser.parse_args()

    for filename in args.files:
        plot_file(filename, args.scan_index)
    plt.show()