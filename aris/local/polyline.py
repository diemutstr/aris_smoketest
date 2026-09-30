"""A line as a polyline with arc length: where the layers go and where the exact samples go."""
from __future__ import annotations

import numpy as np

DUP_TOL = 1e-9          # m; consecutive points closer than this are one point
MIN_GAP = 1e-5          # m; the closest two exact samples may be


def clean(points) -> tuple[np.ndarray, np.ndarray]:
    """Drop non-finite points and repeated points.  -> points (M,3), arc length at each (M,)."""
    p = np.asarray(points, float).reshape(-1, 3)
    p = p[np.all(np.isfinite(p), axis=1)]
    if len(p) > 1:
        keep = np.concatenate([[True], np.linalg.norm(np.diff(p, axis=0), axis=1) > DUP_TOL])
        p = p[keep]
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    return p, s


def at(points: np.ndarray, s_points: np.ndarray, s) -> np.ndarray:
    """Positions on the polyline at arc lengths s.  Each lies on a segment of the polyline."""
    s = np.asarray(s, float)
    return np.stack([np.interp(s, s_points, points[:, i]) for i in range(3)], axis=-1)


def layer_positions(length: float, step: float) -> np.ndarray:
    """Evenly spaced arc lengths from 0 to `length`, no further apart than `step`, both ends in."""
    n = max(1, int(np.ceil(length / step - 1e-9)))
    s = length * np.arange(n + 1) / n
    s[-1] = length
    return s


def dense_positions(s0: float, s1: float, s_points: np.ndarray, step: float) -> np.ndarray:
    """Arc lengths of the exact samples from s0 to s1: both ends, every corner of the polyline
    in between, and evenly spaced samples at most `step` apart.  A corner within MIN_GAP of an
    end is left to the end; an even sample within a tenth of a step of a corner, to the corner.
    """
    n = max(1, int(np.ceil((s1 - s0) / step - 1e-9)))
    uniform = s0 + (s1 - s0) * np.arange(n + 1) / n
    uniform[-1] = s1
    corners = s_points[(s_points > s0 + MIN_GAP) & (s_points < s1 - MIN_GAP)]
    if len(corners):
        i = np.clip(np.searchsorted(corners, uniform), 1, len(corners)) - 1
        j = np.clip(i + 1, 0, len(corners) - 1)
        d = np.minimum(np.abs(uniform - corners[i]), np.abs(uniform - corners[j]))
        near = d < 0.1 * step
        near[[0, -1]] = False
        uniform = uniform[~near]
    return np.union1d(uniform, corners)
