"""Raise an end: move the pen tip straight up from the paper, the hand keeping its orientation.

An end of a free-space move has the pen tip a few millimetres above the paper, the hardest
place for a tree to grow from.  If there is a paper plane among the obstacles, the tip is moved
along the paper normal to a comfortable height by a short straight tip move.  The arm keeps its
shape: joint 7 stays fixed and every sample takes the IK answer nearest the previous one, so the
joints change continuously; a jump means the shape could not be followed and the lift is dropped.

The lift is an attempt, not a rule (lesson L7: lifting can bring the elbow closer to a
neighbour).  If any piece of it is not free, the caller keeps the original end.
"""
from __future__ import annotations

import numpy as np

from aris.types import Obstacles, Plane

STEP = 0.012          # m of tip rise between IK samples (fewer samples, fewer corners to time)
MAX_JUMP = 0.1        # rad; a larger joint change between samples is a change of arm shape
SAME = 1e-6           # rad; the IK answer that reproduces the end itself


def paper_plane(obstacles: Obstacles) -> Plane | None:
    for p in obstacles.planes:
        if p.kind == "paper":
            return p
    return None


def lift_path(arm, q: np.ndarray, paper: Plane, height: float) -> np.ndarray | None:
    """Joint samples from q (first row) to the raised configuration (last row), or None if the
    tip is already at `height` or the arm shape cannot be followed."""
    n = np.asarray(paper.normal, float)
    T0 = arm.fk(q[None])[0]
    h0 = float(n @ arm.tip(q[None])[0] - paper.offset)
    rise = height - h0
    if rise <= STEP:
        return None
    k = int(np.ceil(rise / STEP)) + 1
    T = np.repeat(T0[None], k, axis=0)
    T[:, :3, 3] += np.linspace(0.0, rise, k)[:, None] * n
    Q, ok = arm.ik(T, np.full(k, q[6]))
    out = [q]
    for i in range(1, k):
        if not ok[i].any():
            return None
        d = np.where(ok[i], np.linalg.norm(np.nan_to_num(Q[i] - out[-1], nan=1e9), axis=1), np.inf)
        b = int(np.argmin(d))
        if d[b] > MAX_JUMP:
            return None
        out.append(Q[i, b])
    first = np.where(ok[0], np.linalg.norm(np.nan_to_num(Q[0] - q, nan=1e9), axis=1), np.inf)
    if first.min() > SAME:                     # the solver does not know this end: no lift
        return None
    return np.array(out)


def raise_end(arm, checker, q: np.ndarray, paper: Plane | None, height: float | None):
    """The lift from q as a joint path (first row q), if it exists and is free; else None."""
    if paper is None or height is None:
        return None
    path = lift_path(arm, q, paper, height)
    if path is None or not checker.edges(path[:-1], path[1:]).all():
        return None
    return path
