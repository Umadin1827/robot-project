"""DC motor model (from the course notebook given by the professor) and a fast exact version."""
from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp

# Motor constants from the course notebook
T_M = 0.08   # electromechanical time constant [s]
K_E = 0.5    # motor constant [V*s/rad]  (steady-state speed = u / K_E)


def dc_motor(t: float, y: list[float], u: float) -> list[float]:
    """Right-hand side of the DC motor ODE (from the course notebook given by the professor).

    State y = [theta, omega]  (angle [rad], angular speed [rad/s]).
    Input u = control voltage [V].
        d(theta)/dt = omega
        d(omega)/dt = u / (T_M*K_E) - omega / T_M
    """
    omega = y[1]
    dtheta_dt = omega
    domega_dt = u / (T_M * K_E) - omega / T_M
    return [dtheta_dt, domega_dt]


def motor_step_ivp(state: list[float], u: float, t0: float, t1: float) -> list[float]:
    """One time step of the motor with scipy's solve_ivp (slow, but trusted)."""
    # Technique: scipy solve_ivp - a ready ODE solver
    sol = solve_ivp(dc_motor, (t0, t1), state, args=(u,), t_eval=[t0, t1])
    return list(sol.y[:, -1])


def motor_step_exact(state: tuple[float, float], u: float, dt: float) -> tuple[float, float]:
    """One time step with the exact analytic solution (fast).

    For a constant voltage u during dt the ODE is linear, so
        omega(dt) = w_ss + (omega0 - w_ss) * exp(-dt/T_M),   w_ss = u / K_E
        theta(dt) = theta0 + w_ss*dt + (omega0 - w_ss) * T_M * (1 - exp(-dt/T_M))
    """
    theta0, omega0 = state
    w_ss = u / K_E
    decay = np.exp(-dt / T_M)
    omega = w_ss + (omega0 - w_ss) * decay
    theta = theta0 + w_ss * dt + (omega0 - w_ss) * T_M * (1.0 - decay)
    return float(theta), float(omega)
