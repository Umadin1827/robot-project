"""Paths the robot should follow, and small angle / geometry helpers."""
from __future__ import annotations

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle in radians into the range [-pi, pi).

    Example: 3*pi/2 (270 degrees) becomes -pi/2 (-90 degrees).
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def spiral_path(n_points: int) -> tuple[np.ndarray, np.ndarray]:
    """Moving spiral from my original notebook (two loops while moving right).

    x = -1 + cos(theta) + 0.5*theta,  y = sin(theta),  theta in [0, 4*pi]
    Use n_points=21 for the sparse waypoints (original notebook) and a large
    number (for example 800) for the dense path used by pure pursuit.
    """
    theta = np.linspace(0, 4 * np.pi, n_points)
    return -1 + np.cos(theta) + 0.5 * theta, np.sin(theta)


def straight_path(length: float, n_points: int) -> tuple[np.ndarray, np.ndarray]:
    """Straight line along the x axis, used for the verification tests."""
    return np.linspace(0, length, n_points), np.zeros(n_points)


def path_curvature(cx: np.ndarray, cy: np.ndarray) -> np.ndarray:
    """Curvature kappa = 1/R of a path (1/m), from finite differences."""
    dx, dy = np.gradient(cx), np.gradient(cy)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    return (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5


def path_length(cx: np.ndarray, cy: np.ndarray) -> float:
    """Total length of the path in metres."""
    return float(np.sum(np.hypot(np.diff(cx), np.diff(cy))))
