# Plot the robot's trajectory from the logged odometry, so you can see how it moved.
#
# Usage (run in the folder with the odom_content_<motion>.csv files):
#   python3 plot_trajectory.py                      # all three motions, static plots
#   python3 plot_trajectory.py --motion spiral      # just one
#   python3 plot_trajectory.py --motion circle --animate            # watch it drive
#   python3 plot_trajectory.py --motion line --animate --gif        # also save a .gif
#
# Static plot: path coloured by time, arrows showing which way the robot faced,
# robot outline drawn every few seconds, start/end marked.

import argparse
import os
from math import cos, sin, pi

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation

ROBOT_RADIUS = 0.17   # [m] approx. TurtleBot 4 footprint, only used for drawing


def read_odom(path):
    """Returns t [s from first sample], x, y, th from odom_content_<motion>.csv."""
    t, x, y, th = [], [], [], []
    with open(path) as f:
        next(f)  # header
        for line in f:
            v = [float(s) for s in line.strip().split(',') if s.strip()]
            if len(v) < 4:
                continue
            x.append(v[0]); y.append(v[1]); th.append(v[2]); t.append(v[3])
    t0 = t[0]
    t = [(s - t0) / 1e9 for s in t]
    return t, x, y, th


def draw_robot(ax, x, y, th, color, alpha=1.0):
    """Robot as a circle with a line showing the direction it faces."""
    body = Circle((x, y), ROBOT_RADIUS, fill=False, ec=color, lw=1.2, alpha=alpha)
    ax.add_patch(body)
    nose, = ax.plot([x, x + ROBOT_RADIUS * cos(th)], [y, y + ROBOT_RADIUS * sin(th)],
                    '-', color=color, lw=1.5, alpha=alpha)
    return body, nose


def static_plot(motion, t, x, y, th, every_s=3.0):
    fig, ax = plt.subplots(figsize=(7, 7))

    # path coloured by time
    pts = list(zip(x, y))
    segs = [[pts[k], pts[k + 1]] for k in range(len(pts) - 1)]
    lc = LineCollection(segs, cmap='viridis', linewidths=2.5)
    lc.set_array(t[:-1])
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label('time [s]')

    # robot outline + heading every `every_s` seconds
    # arrows every `every_s` seconds, robot outline only on every 3rd arrow (less clutter)
    next_t, n = 0.0, 0
    for k in range(len(t)):
        if t[k] >= next_t:
            if n % 3 == 0:
                draw_robot(ax, x[k], y[k], th[k], 'grey', alpha=0.4)
            n += 1
            ax.annotate('', xy=(x[k] + 0.12 * cos(th[k]), y[k] + 0.12 * sin(th[k])),
                        xytext=(x[k], y[k]),
                        arrowprops=dict(arrowstyle='->', color='black', lw=1.2))
            next_t += every_s

    ax.plot(x[0], y[0], 'o', color='tab:green', ms=10, label='start', zorder=5)
    ax.plot(x[-1], y[-1], 's', color='tab:red', ms=10, label='end', zorder=5)
    ax.plot([], [], '->', color='black', label=f'heading (every {every_s:g} s)')
    ax.plot([], [], 'o', mfc='none', mec='grey', ms=12, label=f'robot outline (every {3 * every_s:g} s)')

    ax.set_title(f'Robot trajectory from odometry ({motion})')
    ax.set_xlabel('x [m] (odom frame)'); ax.set_ylabel('y [m] (odom frame)')
    ax.set_aspect('equal', adjustable='box')
    pad = ROBOT_RADIUS + 0.1
    ax.set_xlim(min(x) - pad, max(x) + pad); ax.set_ylim(min(y) - pad, max(y) + pad)
    ax.grid(True)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2)
    fig.tight_layout()
    out = f'trajectory_{motion}.png'
    fig.savefig(out, dpi=200)
    print('saved', out)
    return fig


def animate(motion, t, x, y, th, speedup=4.0, save_gif=False):
    """Replays the run. speedup=4 means 4x real time."""
    fig, ax = plt.subplots(figsize=(7, 7))
    pad = ROBOT_RADIUS + 0.1
    ax.set_xlim(min(x) - pad, max(x) + pad); ax.set_ylim(min(y) - pad, max(y) + pad)
    ax.set_aspect('equal', adjustable='box'); ax.grid(True)
    ax.set_xlabel('x [m] (odom frame)'); ax.set_ylabel('y [m] (odom frame)')
    ax.plot(x, y, ':', color='lightgrey', label='full path')
    trail, = ax.plot([], [], '-', color='tab:blue', lw=2, label='path so far')
    ax.plot(x[0], y[0], 'o', color='tab:green', ms=9, label='start')
    body = Circle((x[0], y[0]), ROBOT_RADIUS, fill=False, ec='tab:red', lw=2)
    ax.add_patch(body)
    nose, = ax.plot([], [], '-', color='tab:red', lw=2, label='robot (line = heading)')
    title = ax.set_title('')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2)
    fig.subplots_adjust(bottom=0.2)

    fps = 20
    frame_dt = speedup / fps            # seconds of data per frame
    n_frames = int(t[-1] / frame_dt) + 1
    idx = []                            # odom sample to show in each frame
    k = 0
    for f in range(n_frames):
        while k < len(t) - 1 and t[k + 1] <= f * frame_dt:
            k += 1
        idx.append(k)

    def update(f):
        k = idx[f]
        trail.set_data(x[:k + 1], y[:k + 1])
        body.center = (x[k], y[k])
        nose.set_data([x[k], x[k] + ROBOT_RADIUS * cos(th[k])],
                      [y[k], y[k] + ROBOT_RADIUS * sin(th[k])])
        title.set_text(f'Robot trajectory ({motion})   t = {t[k]:5.1f} s   ({speedup:g}x speed)')
        return trail, body, nose, title

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000 / fps, blit=False)
    if save_gif:
        out = f'trajectory_{motion}.gif'
        anim.save(out, writer='pillow', fps=fps)
        print('saved', out)
    return fig, anim


if __name__ == '__main__':
    p = argparse.ArgumentParser(description='Plot the robot trajectory from odometry logs.')
    p.add_argument('--motion', choices=['circle', 'spiral', 'line', 'all'], default='all')
    p.add_argument('--animate', action='store_true', help='replay the run as an animation')
    p.add_argument('--gif', action='store_true', help='with --animate: also save a .gif')
    p.add_argument('--speed', type=float, default=4.0, help='animation speed-up (default 4x)')
    p.add_argument('--every', type=float, default=3.0, help='seconds between robot outlines')
    p.add_argument('--no-show', action='store_true', help='only save files, no windows')
    a = p.parse_args()

    motions = ['line', 'circle', 'spiral'] if a.motion == 'all' else [a.motion]
    keep = []   # keep animations alive until plt.show()
    for m in motions:
        path = f'odom_content_{m}.csv'
        if not os.path.exists(path):
            print('missing', path); continue
        t, x, y, th = read_odom(path)
        static_plot(m, t, x, y, th, a.every)
        if a.animate:
            keep.append(animate(m, t, x, y, th, a.speed, a.gif))
    if not a.no_show:
        plt.show()