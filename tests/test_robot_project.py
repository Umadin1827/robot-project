"""Automated checks. The same checks are also run with assert inside lecture.ipynb."""
import math
from pathlib import Path

import numpy as np
import pytest

from src.bicycle import simulate_car, source_course
from src.controllers import (ConstantVoltage, PurePursuit, WaypointP, curvature, speed_to_voltage,
                             wheel_speeds)
from src.metrics import cross_track_error
from src.motor import motor_step_exact, motor_step_ivp
from src.paths import spiral_path, straight_path, wrap_angle
from src.robot import RobotParams, simulate

ROOT = Path(__file__).resolve().parent.parent


def test_motor_exact_equals_solve_ivp():
    """Check 1: analytic motor step == scipy solve_ivp (same ODE)."""
    state, dt = [0.3, 2.0], 0.01
    exact = motor_step_exact(tuple(state), u=5.0, dt=dt)
    ivp = motor_step_ivp(state, 5.0, 0.0, dt)
    np.testing.assert_allclose(exact, ivp, rtol=1e-5)


def test_curvature_on_a_circle():
    """Check 2: robot on a circle of radius R, heading along the tangent.
    The target point on the circle at chord length Ld has alpha = asin(Ld/(2R)),
    so the pure pursuit formula must return exactly 1/R."""
    for R in (0.5, 1.0, 3.0):
        Ld = 0.4
        alpha = math.asin(Ld / (2 * R))
        np.testing.assert_allclose(curvature(alpha, Ld), 1 / R, rtol=1e-12)


def test_constant_voltages_give_a_circle():
    """Check 3: u_l != u_r, constant -> circle with R = B/2 * (v_r+v_l)/(v_r-v_l)."""
    p = RobotParams()
    u_l, u_r = 4.0, 6.0
    res = simulate(ConstantVoltage(u_l, u_r), p, t_max=20.0)
    v_l, v_r = u_l / 0.5 * p.r, u_r / 0.5 * p.r
    R_expected = p.B / 2 * (v_r + v_l) / (v_r - v_l)
    # measured curvature in the last 5 seconds = change of heading / distance driven
    d_yaw = res.yaw[-1] - res.yaw[-500]
    dist = np.sum(np.hypot(np.diff(res.x[-500:]), np.diff(res.y[-500:])))
    np.testing.assert_allclose(dist / d_yaw, R_expected, rtol=1e-3)


def test_straight_line_converges():
    """Check 4: start 0.5 m beside a straight line -> pure pursuit goes onto it."""
    p = RobotParams()
    sx, sy = straight_path(6, 600)
    res = simulate(PurePursuit(sx, sy, p, Ld=0.3), p, t_max=100, pose0=(0, 0.5, 0))
    assert res.finished
    assert abs(res.y[-1]) < 0.01


def test_wrap_angle():
    np.testing.assert_allclose(wrap_angle(3 * np.pi / 2), -np.pi / 2)
    assert -np.pi <= wrap_angle(100.0) < np.pi


def test_wheel_speeds_and_voltage():
    p = RobotParams()
    v_l, v_r = wheel_speeds(0.2, 0.0, p.B)           # straight: both equal
    assert v_l == v_r == 0.2
    v_l, v_r = wheel_speeds(0.2, 2.0, p.B)           # left turn: right wheel faster
    assert v_r > v_l
    np.testing.assert_allclose((v_r - v_l) / p.B, 0.2 * 2.0)   # omega = v*kappa
    np.testing.assert_allclose(speed_to_voltage(p.v_max, p), p.u_max)


def test_matches_pythonrobotics_reference():
    """Check 5: my re-implementation == trajectory of the original PythonRobotics code."""
    ref = np.loadtxt(ROOT / "data" / "pythonrobotics_reference.csv", delimiter=",", skiprows=1)
    mine = simulate_car(*source_course())
    assert mine.shape == ref.shape
    np.testing.assert_allclose(mine, ref, atol=1e-9)


def test_same_seed_same_result():
    """Check 6: all randomness is seeded -> identical results on rerun."""
    p = RobotParams()
    cx, cy = spiral_path(400)
    a = simulate(PurePursuit(cx, cy, p), p, noise_std=0.05, seed=3)
    b = simulate(PurePursuit(cx, cy, p), p, noise_std=0.05, seed=3)
    np.testing.assert_array_equal(a.x, b.x)


def test_pure_pursuit_is_better_than_waypoint_p():
    """Regression: on the spiral, pure pursuit is faster and closer to the path."""
    p = RobotParams()
    cx, cy = spiral_path(800)
    wx, wy = spiral_path(21)
    old = simulate(WaypointP(wx, wy), p)
    new = simulate(PurePursuit(cx, cy, p, Ld=0.3, v_ref=0.25), p)
    assert old.finished and new.finished
    rms = lambda r: np.sqrt(np.mean(cross_track_error(r.x, r.y, cx, cy) ** 2))
    assert new.t[-1] < old.t[-1]
    assert rms(new) < rms(old)
