"""Small plotting helper so notebook cells stay short."""
from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from .robot import Result


def plot_run(ax: plt.Axes, cx: np.ndarray, cy: np.ndarray, res: Result,
             label: str, color: str = "tab:blue") -> None:
    """Draw the wanted path (red dashed) and the robot track on one axis."""
    ax.plot(cx, cy, "r--", lw=2, alpha=0.6, label="wanted path")
    ax.plot(res.x, res.y, color=color, lw=2, label=label)
    ax.plot(res.x[0], res.y[0], "ks", ms=9, label="start")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3, fontsize=11)
