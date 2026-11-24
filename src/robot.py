"""Differential drive robot: parameters, kinematics and the simulation loop."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .motor import motor_step_exact, K_E


@dataclass          # Technique: dataclass - parameters in one clean object
class RobotParams:
    """All robot numbers in one place (values from the course notebook given by the professor)."""
    B: float = 0.15        # distance between the wheels [m]
    r: float = 0.026       # wheel radius [m]
    dt: float = 0.01       # simulation time step [s]
    u_max: float = 8.0     # voltage limit [V]  (original: +-8)

    @property
    def v_max(self) -> float:
        """Top speed of one wheel when the motor gets u_max [m/s]."""
        return self.u_max / K_E * self.r


@dataclass
class Result:
    """Everything the simulation records (arrays have one entry per step)."""
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    u_l: np.ndarray
    u_r: np.ndarray
    finished: bool = False
    extra: dict = field(default_factory=dict)


def update_pose(theta_l: float, theta_r: float, prev_path: float,
                p: RobotParams, x: float, y: float, yaw0: float = 0.0):
    """Kinematics from wheel angles (formulas from the course notebook given by the professor).

    path   = (theta_r + theta_l) * r / 2     distance travelled
    course = (theta_r - theta_l) * r / B     heading angle
    The robot moves by (path - prev_path) in the new heading direction.
    """
    path = (theta_r + theta_l) * p.r / 2
    course = yaw0 + (theta_r - theta_l) * p.r / p.B
    x_new = x + (path - prev_path) * np.cos(course)
    y_new = y + (path - prev_path) * np.sin(course)
    return x_new, y_new, course, path


def simulate(controller, p: RobotParams, t_max: float = 400.0,
             pose0: tuple[float, float, float] = (0.0, 0.0, 0.0),
             noise_std: float = 0.0, seed: int = 0) -> Result:
    """Run the robot until the controller says it is finished (or t_max).

    controller.command(x, y, yaw) -> (u_l, u_r)   voltages for the two motors
    controller.finished                           True when the goal is reached
    noise_std: standard deviation [m] of Gaussian noise added to the measured
               position (the controller sees the noisy one); 0 = perfect sensor.
    """
    rng = np.random.default_rng(seed)         # Technique: seeded random generator
    n = int(round(t_max / p.dt))
    x0, y0, yaw0 = pose0
    t = np.arange(n + 1) * p.dt
    xs, ys, yaws = np.zeros(n + 1), np.zeros(n + 1), np.zeros(n + 1)
    uls, urs = np.zeros(n + 1), np.zeros(n + 1)
    xs[0], ys[0], yaws[0] = x0, y0, yaw0

    state_l = (0.0, 0.0)       # (theta, omega) of the left motor
    state_r = (0.0, 0.0)       # (theta, omega) of the right motor
    path_prev = 0.0
    i = 0
    while i < n:
        x, y, yaw = xs[i], ys[i], yaws[i]
        mx, my = x, y
        if noise_std > 0:
            mx, my = x + rng.normal(0, noise_std), y + rng.normal(0, noise_std)
        u_l, u_r = controller.command(mx, my, yaw)
        if controller.finished:
            break
        u_l = float(np.clip(u_l, -p.u_max, p.u_max))      # limit the voltage
        u_r = float(np.clip(u_r, -p.u_max, p.u_max))
        state_l = motor_step_exact(state_l, u_l, p.dt)
        state_r = motor_step_exact(state_r, u_r, p.dt)
        xs[i + 1], ys[i + 1], yaws[i + 1], path_prev = update_pose(
            state_l[0], state_r[0], path_prev, p, x, y, yaw0)
        uls[i], urs[i] = u_l, u_r
        i += 1

    k = i + 1
    return Result(t[:k], xs[:k], ys[:k], yaws[:k], uls[:k], urs[:k],
                  finished=bool(controller.finished))
