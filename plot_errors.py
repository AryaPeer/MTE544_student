import matplotlib.pyplot as plt
from math import cos, sin
from utilities import FileReader


MAX_SPEED=0.4   # top linear speed our TurtleBot4 actually reached in the spiral logs [m/s]


# ideal path from the spiral command in decisions.py (v += 0.01 every 0.1 s up to 1.0, w = 1.0)
# the robot can't reach 1 m/s, so v and w are scaled down together to MAX_SPEED:
# this keeps the commanded turning radius v/w, which is what the robot does in the logs
def ideal_spiral(duration, dt=0.1):

    x=0.0; y=0.0; th=0.0
    v_cmd=0.0
    xs=[x]; ys=[y]

    for _ in range(int(duration/dt)):
        v_cmd+=0.01 if v_cmd < 1.0 else 0.0
        w_cmd=1.0

        scale=min(1.0, MAX_SPEED/v_cmd)
        v=v_cmd*scale; w=w_cmd*scale

        x+=v*cos(th)*dt
        y+=v*sin(th)*dt
        th+=w*dt
        xs.append(x); ys.append(y)

    return xs, ys


def plot_errors(filename, spiral=False):

    headers, values=FileReader(filename).read_file()

    # grab a column by its header name (None if that column was not logged)
    def col(name):
        if name not in headers:
            return None
        i=headers.index(name)
        return [lin[i] for lin in values]

    time_list=[]

    first_stamp=values[0][-1]

    for val in values:
        time_list.append(val[-1] - first_stamp)


    # xy trajectory on the left, the four signals on the right
    fig, axes = plt.subplot_mosaic([["xy", "v",  "w"],
                                    ["xy", "ax", "ay"]], figsize=(18,8), constrained_layout=True)
    fig.suptitle(f"Sensor measurements vs EKF estimates ({filename})")


    # xy trajectory: odometry (sensor) vs EKF, plus the ideal spiral if asked for
    if spiral:
        xs, ys = ideal_spiral(time_list[-1])
        axes["xy"].plot(xs, ys, "g:", linewidth=2, label="ideal spiral")
    if col("odom_x") is not None:
        axes["xy"].plot(col("odom_x"), col("odom_y"), "b-", label="odom")
    axes["xy"].plot(col("kf_x"), col("kf_y"), "r--", label="EKF")
    axes["xy"].set_title("xy trajectory")
    axes["xy"].set_xlabel("x [m]")
    axes["xy"].set_ylabel("y [m]")
    axes["xy"].axis("equal")
    axes["xy"].legend()
    axes["xy"].grid()


    # each signal: (subplot, sensor column, sensor label, EKF column, title, y label)
    signals=[("v",  "odom_v", "odom v",   "kf_vx", "Linear velocity",       "v [m/s]"),
             ("w",  "odom_w", "odom w",   "kf_w",  "Angular velocity",      "w [rad/s]"),
             ("ax", "imu_ax", "imu ax",   "kf_ax", "Forward acceleration",  "ax [m/s^2]"),
             ("ay", "imu_ay", "imu ay",   "kf_ay", "Lateral acceleration",  "ay [m/s^2]")]

    for key, sensor, sensor_label, ekf, title, ylabel in signals:
        ax=axes[key]
        if col(sensor) is not None:
            ax.plot(time_list, col(sensor), "b-", label=sensor_label)
        ax.plot(time_list, col(ekf), "r--", label="EKF")
        ax.set_title(title)
        ax.set_xlabel("time [s]")
        ax.set_ylabel(ylabel)
        ax.legend()
        ax.grid()

    plt.show()






import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--spiral', action='store_true', help='overlay the ideal spiral on the xy plot')

    args = parser.parse_args()

    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename, args.spiral)


