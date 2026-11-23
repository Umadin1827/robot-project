"""How good was the tracking? Cross-track error (distance to the path)."""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def cross_track_error(x: np.ndarray, y: np.ndarray,
                      cx: np.ndarray, cy: np.ndarray, spacing: float = 0.001) -> np.ndarray:
    """Distance from every robot position to the closest point of the path [m].

    The path is first refined to points `spacing` metres apart (linear
    interpolation), otherwise the error would only be measured to the path
    points, which are 1-2 cm apart.
    Technique: scipy cKDTree - fast nearest neighbour search instead of
    comparing every robot point with every path point.
    """
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(cx), np.diff(cy)))])   # distance along path
    s_fine = np.linspace(0.0, s[-1], int(s[-1] / spacing) + 1)
    fine = np.column_stack([np.interp(s_fine, s, cx), np.interp(s_fine, s, cy)])
    dist, _ = cKDTree(fine).query(np.column_stack([x, y]))
    return dist


def summary(cte: np.ndarray, t: np.ndarray) -> dict[str, float]:
    """RMS error, max error and the time needed."""
    return {"rms_cte": float(np.sqrt(np.mean(cte**2))),
            "max_cte": float(cte.max()),
            "time": float(t[-1])}
