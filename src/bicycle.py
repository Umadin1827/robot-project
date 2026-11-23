"""My own re-implementation of the PythonRobotics pure pursuit example.

Source (MIT licence): Atsushi Sakai et al., PythonRobotics,
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

The source steers a car (bicycle model).  I wrote the same algorithm again
with the same numbers, to check that my pure pursuit gives the same result.
The idea (curvature from alpha and Ld) is identical to the robot version;
only the last step differs: the car converts curvature into a steering angle,
my robot converts it into two wheel speeds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .controllers import curvature, find_target_index
from .paths import wrap_angle


@dataclass
class CarParams:
    """Parameters of the PythonRobotics example (copied from the source file)."""
    k: float = 0.1                  # look-ahead gain (Ld grows with speed)
    Lfc: float = 2.0                # base look-ahead distance [m]
    Kp: float = 1.0                 # speed P gain
    dt: float = 0.1                 # time step [s]
    WB: float = 2.9                 # wheel base [m]
    max_steer: float = math.pi / 4  # max steering angle [rad]


def source_course() -> tuple[np.ndarray, np.ndarray]:
    """The target course used in the source's main()."""
    cx = np.arange(0, 50, 0.5)
    cy = np.array([math.sin(ix / 5.0) * ix / 2.0 for ix in cx])
    return cx, cy


def simulate_car(cx, cy, p: CarParams = CarParams(), target_speed: float = 10.0 / 3.6,
                 x0: float = 0.0, y0: float = -3.0, T: float = 100.0) -> np.ndarray:
    """Pure pursuit + P speed control for the bicycle model.

    Returns an array with columns [t, x, y, yaw, v, steer].
    """
    x, y, yaw, v = x0, y0, 0.0, 0.0
    rear = lambda: (x - (p.WB / 2) * math.cos(yaw), y - (p.WB / 2) * math.sin(yaw))
    rx, ry = rear()
    nearest = int(np.argmin(np.hypot(cx - rx, cy - ry)))      # first search: global
    Ld = p.k * v + p.Lfc
    nearest, target = find_target_index(cx, cy, rx, ry, nearest, Ld)
    rows = [[0.0, x, y, yaw, v, 0.0]]
    t, last = 0.0, len(cx) - 1
    while T >= t and last > target:
        rx, ry = rear()
        Ld = p.k * v + p.Lfc                                    # speed dependent Ld
        nearest, new_target = find_target_index(cx, cy, rx, ry, nearest, Ld)
        target = max(target, new_target)                        # never go backwards
        alpha = math.atan2(cy[target] - ry, cx[target] - rx) - yaw
        kappa = curvature(alpha, Ld)
        steer = float(np.clip(math.atan(p.WB * kappa), -p.max_steer, p.max_steer))
        a = p.Kp * (target_speed - v)
        # state update (same order as the source: old yaw and v move the car)
        x += v * math.cos(yaw) * p.dt
        y += v * math.sin(yaw) * p.dt
        yaw = wrap_angle(yaw + v / p.WB * math.tan(steer) * p.dt)
        v += a * p.dt
        t += p.dt
        rows.append([t, x, y, yaw, v, steer])
    return np.array(rows)
