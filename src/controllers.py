"""Two controllers for the robot: my waypoint P controller and pure pursuit."""
from __future__ import annotations

import math
from typing import Protocol

import numpy as np

from .motor import K_E
from .paths import wrap_angle
from .robot import RobotParams


class Controller(Protocol):
    """What simulate() needs from a controller (Technique: typing.Protocol)."""
    finished: bool

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float]: ...


class ConstantVoltage:
    """Test controller: always the same two voltages (it never finishes)."""

    def __init__(self, u_l: float, u_r: float):
        self.u = (u_l, u_r)
        self.finished = False

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        return self.u


# ---------------------------------------------------------------- baseline
class WaypointP:
    """Waypoint controller of the course notebook (given by the professor): drive to waypoint after waypoint.

        u_l = 8*distance - 1*heading_error
        u_r = 8*distance + 1*heading_error
    The robot goes to the next waypoint when it is closer than `tol` metres.
    """

    def __init__(self, wx, wy, tol: float = 0.05, k_dist: float = 8.0, k_head: float = 1.0):
        self.wx, self.wy, self.tol = wx, wy, tol
        self.k_dist, self.k_head = k_dist, k_head
        self.k = 1                  # waypoint 0 is the start point, so begin with 1
        self.finished = False

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        dist = math.hypot(self.wx[self.k] - x, self.wy[self.k] - y)
        if dist <= self.tol:                      # reached this waypoint
            if self.k == len(self.wx) - 1:
                self.finished = True
                return 0.0, 0.0
            self.k += 1
            dist = math.hypot(self.wx[self.k] - x, self.wy[self.k] - y)
        bearing = math.atan2(self.wy[self.k] - y, self.wx[self.k] - x)
        heading_error = wrap_angle(bearing - yaw)
        return (self.k_dist * dist - self.k_head * heading_error,
                self.k_dist * dist + self.k_head * heading_error)


# ------------------------------------------------------------ pure pursuit
def find_target_index(cx, cy, px: float, py: float, nearest: int, Ld: float) -> tuple[int, int]:
    """Find the look-ahead target on the path.

    1. From the old nearest index go forward while the next point is closer
       (the index never goes backwards, so loops of the path cannot confuse us).
    2. From the nearest point walk forward until a point is at least Ld away.
    Returns (nearest_index, target_index).
    """
    last = len(cx) - 1
    d = math.hypot(cx[nearest] - px, cy[nearest] - py)
    while nearest < last:
        d_next = math.hypot(cx[nearest + 1] - px, cy[nearest + 1] - py)
        if d_next >= d:
            break
        nearest, d = nearest + 1, d_next
    target = nearest
    while target < last and math.hypot(cx[target] - px, cy[target] - py) < Ld:
        target += 1
    return nearest, target


def curvature(alpha: float, Ld: float) -> float:
    """Pure pursuit formula: curvature of the circle through robot and target.

    alpha = angle between robot heading and the line to the target [rad]
    Ld    = look-ahead distance [m]
        kappa = 2*sin(alpha) / Ld        (kappa = 1/R, positive = turn left)
    """
    return 2.0 * math.sin(alpha) / Ld


def wheel_speeds(v: float, kappa: float, B: float) -> tuple[float, float]:
    """Left and right wheel linear speeds [m/s] for speed v and curvature kappa.

    Written by me (Umadin1827). The turning rate is omega = v * kappa, and the
    wheels are B/2 left and right of the centre, so they differ by +-omega*B/2:
        v_left  = v - omega*B/2  =  v * (1 - kappa*B/2)
        v_right = v + omega*B/2  =  v * (1 + kappa*B/2)
    """
    omega = v * kappa
    v_left = v - (omega * B) / 2
    v_right = v + (omega * B) / 2
    return v_left, v_right


def speed_to_voltage(v_wheel: float, p: RobotParams) -> float:
    """Wheel speed [m/s] -> motor voltage [V] with the steady-state motor law.

    omega = u / K_E  and  v_wheel = omega * r   =>   u = K_E * v_wheel / r
    """
    return K_E * v_wheel / p.r


class PurePursuit:
    """Pure pursuit path tracking for the differential drive robot."""

    def __init__(self, cx, cy, p: RobotParams, Ld: float = 0.3, v_ref: float = 0.25,
                 goal_tol: float = 0.05):
        self.cx, self.cy, self.p = cx, cy, p
        self.Ld, self.v_ref, self.goal_tol = Ld, v_ref, goal_tol
        self.nearest = 0
        self.target = 0
        self.finished = False
        self.alpha_history: list[float] = []

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        d_goal = math.hypot(self.cx[-1] - x, self.cy[-1] - y)
        if self.target == len(self.cx) - 1 and d_goal < self.goal_tol:
            self.finished = True
            return 0.0, 0.0
        self.nearest, self.target = find_target_index(
            self.cx, self.cy, x, y, self.nearest, self.Ld)
        tx, ty = self.cx[self.target], self.cy[self.target]
        alpha = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        kappa = curvature(alpha, self.Ld)
        # slow down when the goal is close, so the robot can stop on it
        v = self.v_ref * min(1.0, max(d_goal / self.Ld, 0.2))
        v_l, v_r = wheel_speeds(v, kappa, self.p.B)
        self.alpha_history.append(alpha)
        return speed_to_voltage(v_l, self.p), speed_to_voltage(v_r, self.p)
