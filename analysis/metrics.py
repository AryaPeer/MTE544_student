# run this from root with: python3 analysis/metrics.py

import os
from math import copysign, cos, nan

from plot_run import ROOT, read_csv

RUNS = {"P": "data/point/p_tuned", "PID": "data/point/pid_tuned"}
SETTLING_BAND = 0.05


def load_errors(folder):
    linear = read_csv(os.path.join(ROOT, folder, "linear.csv"))
    angular = read_csv(os.path.join(ROOT, folder, "angular.csv"))

    n = min(len(linear["e"]), len(angular["e"]))
    t = [(stamp - linear["stamp"][0]) / 1e9 for stamp in linear["stamp"][:n]]
    e_lin = [copysign(d, cos(th)) for d, th in zip(linear["e"][:n], angular["e"][:n])]
    e_ang = angular["e"][:n]
    return t, e_lin, e_ang


def first_time_below(t, r, level):
    return next((ti for ti, ri in zip(t, r) if ri <= level), nan)


def step_metrics(t, e):
    r = [ei / e[0] for ei in e]

    rise_time = first_time_below(t, r, 0.1) - first_time_below(t, r, 0.9)

    outside = [i for i, ri in enumerate(r) if abs(ri) > SETTLING_BAND]
    if not outside:
        settling_time = 0.0
    elif outside[-1] == len(r) - 1:
        settling_time = nan
    else:
        settling_time = t[outside[-1] + 1]

    overshoot = max(0.0, -min(r)) * 100

    return rise_time, settling_time, abs(e[-1]), overshoot


def main():
    print(f"{'Run':<14}{'Rise (s)':>10}{'Settle (s)':>12}{'SS error':>12}{'OS (%)':>9}")
    for name, folder in RUNS.items():
        t, e_lin, e_ang = load_errors(folder)
        for signal, e, unit in (("linear", e_lin, "m"), ("angular", e_ang, "rad")):
            rise, settle, ss, os_pct = step_metrics(t, e)
            print(f"{name + ' ' + signal:<14}{rise:>10.2f}{settle:>12.2f}{ss:>8.4f} {unit:<3}{os_pct:>9.1f}")


if __name__ == "__main__":
    main()
