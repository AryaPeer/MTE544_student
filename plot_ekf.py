"""
Plotting script for the Lab 3 EKF logs written by localization.py.

For every CSV it produces:
  1. Path figure    - odometry path vs EKF path in the x-y plane (start/end marked)
  2. States figure  - measurement vs estimate over time for v, w, ax, ay, heading,
                      plus the EKF-vs-odom position difference
and prints a short numeric summary (RMS of measurement - estimate per channel).

With --compare, it also draws one small-multiples figure with the path of every
file side by side (useful for the Q/R variations in the report).

Older logs that only have the 9 template columns (no odom_* columns) still work:
the odometry series are simply skipped.

Usage:
  python3 plot_ekf.py --files test2.csv
  python3 plot_ekf.py --files spiral_q05_r05.csv spiral_q01_r05.csv --labels "Q=0.5, R=0.5" "Q=0.1, R=0.5" --compare
  python3 plot_ekf.py --files test2.csv --save figures/      # write PNGs instead of opening windows
"""

import argparse
import os

import numpy as np
import matplotlib.pyplot as plt

# measurement / odometry = blue solid, EKF estimate = orange dashed (colour + line style,
# so the two remain distinguishable in grayscale print)
MEAS = dict(color="#2a78d6", linestyle="-", linewidth=1.4)
EST = dict(color="#eb6834", linestyle="--", linewidth=2.0)
ERR = dict(color="#1baf7a", linestyle="-", linewidth=1.6)
RUN_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]


def read_log(filename):
    """Read a Logger CSV into {header: np.array}. Tolerates the trailing ', ' and a cut-off last line."""
    with open(filename) as f:
        headers = [h.strip() for h in f.readline().split(",") if h.strip()]
        rows = []
        for line in f:
            vals = [v.strip() for v in line.split(",") if v.strip()]
            if len(vals) != len(headers):
                continue  # partial line (e.g. node killed mid-write)
            try:
                rows.append([float(v) for v in vals])
            except ValueError:
                continue
    if not rows:
        raise ValueError(f"{filename}: no complete data rows")
    data = np.array(rows)
    log = {h: data[:, i] for i, h in enumerate(headers)}
    log["t"] = log["stamp"] - log["stamp"][0]
    return log


def has(log, *keys):
    return all(k in log for k in keys)


def style_axis(ax, title, xlabel, ylabel):
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.35)


def draw_path(ax, log, odom_label="Odometry path", ekf_label="EKF path", ekf_style=EST):
    if has(log, "odom_x", "odom_y"):
        ax.plot(log["odom_x"], log["odom_y"], label=odom_label, **MEAS)
    ax.plot(log["kf_x"], log["kf_y"], label=ekf_label, **ekf_style)
    ax.plot(log["kf_x"][0], log["kf_y"][0], marker="o", markersize=8, color="#444444",
            linestyle="none", label="Start")
    ax.plot(log["kf_x"][-1], log["kf_y"][-1], marker="X", markersize=9, color="#444444",
            linestyle="none", label="End (EKF)")
    ax.set_aspect("equal", adjustable="datalim")


def plot_path(log, name):
    fig, ax = plt.subplots(figsize=(7, 7), constrained_layout=True)
    draw_path(ax, log)
    style_axis(ax, f"Robot path: odometry vs EKF ({name})", "x [m]", "y [m]")
    ax.legend(loc="best")
    return fig


def plot_states(log, name):
    t = log["t"]
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=True, constrained_layout=True)
    fig.suptitle(f"EKF estimates vs measurements ({name})", fontsize=14)

    # linear velocity: odom v (measured) vs EKF v
    ax = axes[0, 0]
    if has(log, "odom_v"):
        ax.plot(t, log["odom_v"], label="Odometry v (measured)", **MEAS)
    ax.plot(t, log["kf_vx"], label="EKF v (estimated)", **EST)
    style_axis(ax, "Linear velocity", "", "v [m/s]")
    ax.legend(loc="best")

    # angular velocity: odom w vs EKF w
    ax = axes[0, 1]
    if has(log, "odom_w"):
        ax.plot(t, log["odom_w"], label="Odometry ω (measured)", **MEAS)
    ax.plot(t, log["kf_w"], label="EKF ω (estimated)", **EST)
    style_axis(ax, "Angular velocity", "", "ω [rad/s]")
    ax.legend(loc="best")

    # forward acceleration: imu ax vs EKF vdot
    ax = axes[1, 0]
    ax.plot(t, log["imu_ax"], label="IMU aₓ (measured)", **MEAS)
    ax.plot(t, log["kf_ax"], label="EKF v̇ (estimated)", **EST)
    style_axis(ax, "Forward acceleration", "", "aₓ [m/s²]")
    ax.legend(loc="best")

    # lateral (centripetal) acceleration: imu ay vs EKF v*w
    ax = axes[1, 1]
    ax.plot(t, log["imu_ay"], label="IMU a_y (measured)", **MEAS)
    ax.plot(t, log["kf_ay"], label="EKF v·ω (estimated)", **EST)
    style_axis(ax, "Lateral (centripetal) acceleration", "", "a_y [m/s²]")
    ax.legend(loc="best")

    # heading: odom yaw (unwrapped so it is continuous) vs EKF theta
    ax = axes[2, 0]
    if has(log, "kf_th"):
        if has(log, "odom_th"):
            ax.plot(t, np.unwrap(log["odom_th"]), label="Odometry θ (unwrapped)", **MEAS)
        ax.plot(t, log["kf_th"], label="EKF θ (estimated)", **EST)
        ax.legend(loc="best")
    else:
        ax.text(0.5, 0.5, "kf_th / odom_th not logged in this file", ha="center", va="center",
                transform=ax.transAxes)
    style_axis(ax, "Heading", "time [s]", "θ [rad]")

    # position: EKF vs odom difference, or x/y vs time if odom pose is missing
    ax = axes[2, 1]
    if has(log, "odom_x", "odom_y"):
        diff = np.hypot(log["kf_x"] - log["odom_x"], log["kf_y"] - log["odom_y"])
        ax.plot(t, diff, label="‖EKF − odom‖ position", **ERR)
        style_axis(ax, "Position difference, EKF vs odometry", "time [s]", "distance [m]")
    else:
        ax.plot(t, log["kf_x"], label="EKF x", **EST)
        ax.plot(t, log["kf_y"], label="EKF y", color="#1baf7a", linestyle=":", linewidth=2.0)
        style_axis(ax, "EKF position vs time (odom pose not logged)", "time [s]", "position [m]")
    ax.legend(loc="best")
    return fig


def plot_compare(logs, labels):
    n = len(logs)
    cols = min(n, 3)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 5.2 * rows), squeeze=False,
                             constrained_layout=True)
    fig.suptitle("Path comparison across runs: odometry vs EKF", fontsize=14)
    for i, (log, label) in enumerate(zip(logs, labels)):
        ax = axes[i // cols][i % cols]
        draw_path(ax, log)
        style_axis(ax, label, "x [m]", "y [m]")
        ax.legend(loc="best", fontsize=8)
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].set_visible(False)
    return fig


def rms(a):
    return float(np.sqrt(np.mean(np.square(a))))


def summary(log, name):
    dt = np.diff(log["t"])
    print(f"\n=== {name} ===")
    print(f"  samples: {len(log['t'])}, duration: {log['t'][-1]:.1f} s, "
          f"dt mean {dt.mean():.3f} s (min {dt.min():.3f}, max {dt.max():.3f})")
    print("  RMS(measurement - estimate):")
    pairs = [("odom_v", "kf_vx", "v  [m/s]"), ("odom_w", "kf_w", "w  [rad/s]"),
             ("imu_ax", "kf_ax", "ax [m/s^2]"), ("imu_ay", "kf_ay", "ay [m/s^2]")]
    for meas, est, unit in pairs:
        if has(log, meas, est):
            print(f"    {unit:11s} {rms(log[meas] - log[est]):.4f}")
    if has(log, "odom_x", "odom_y"):
        d = np.hypot(log["kf_x"] - log["odom_x"], log["kf_y"] - log["odom_y"])
        print(f"  EKF vs odom position: final {d[-1]:.3f} m, max {d.max():.3f} m")
    else:
        print("  (no odom columns in this log - re-record with the updated localization.py "
              "to compare against odometry)")


def main():
    parser = argparse.ArgumentParser(description="Plot EKF logs: odometry vs EKF path and states.")
    parser.add_argument("--files", nargs="+", required=True, help="CSV logs from localization.py")
    parser.add_argument("--labels", nargs="+", help="name for each file in titles (e.g. 'Q=0.5, R=0.5')")
    parser.add_argument("--compare", action="store_true", help="also draw all paths side by side")
    parser.add_argument("--save", metavar="DIR", help="save PNGs to DIR instead of showing windows")
    args = parser.parse_args()

    labels = args.labels or [os.path.splitext(os.path.basename(f))[0] for f in args.files]
    if len(labels) != len(args.files):
        parser.error("--labels must have one entry per file")

    logs = [read_log(f) for f in args.files]
    figs = []
    for log, label in zip(logs, labels):
        summary(log, label)
        figs.append((f"{label}_path", plot_path(log, label)))
        figs.append((f"{label}_states", plot_states(log, label)))
    if args.compare and len(logs) > 1:
        figs.append(("compare_paths", plot_compare(logs, labels)))

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        for fname, fig in figs:
            safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in fname)
            path = os.path.join(args.save, safe + ".png")
            fig.savefig(path, dpi=200)
            print(f"saved {path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
