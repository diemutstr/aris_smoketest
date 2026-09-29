"""The pen on the paper during a drawing motion.

Read at the driver's rate (1 kHz) on the flown curve:
  height     how far the tip is from the paper plane (before the controller presses)
  off line   how far the tip is from the line it was asked to draw (`motion.tip_base`, the
             polyline through the planned tips), and how far each sample's tip is from its
             own planned point
  backwards  how far the tip ever falls back along the line, measured from the furthest point
             it had reached
  slowest    the slowest speed along the line between the moment the pen first gets going and
             the moment it starts its final stop (a stop halfway reads near zero)
  fastest    the fastest tip speed, against the drawing speed
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.check.model import ArmModel, tip

GOING = 0.25          # of the fastest speed along the line: the pen "is going" above this


@dataclass(frozen=True)
class PenReport:
    height: float        # m, largest |tip z - paper z|
    off_line: float      # m, largest distance from the planned line
    backwards: float     # m
    slowest: float       # m/s along the line, between getting going and the final stop
    fastest: float       # m/s, tip speed
    length: float        # m, how far along the line the tip got


def _project(p, a, b):
    """Closest points of points p (M,3) on segments [a,b] (M,S,3): distance (M,S), fraction."""
    d = b - a
    dd = np.einsum("msi,msi->ms", d, d)
    u = np.clip(np.einsum("msi,msi->ms", p[:, None] - a, d) / np.where(dd > 0, dd, 1.0), 0, 1)
    return np.linalg.norm(a + u[..., None] * d - p[:, None], axis=-1), u


def pen_report(model: ArmModel, T_table_base, paper_z, traj, tip_base, t_rate, q_rate,
               window: int = 2) -> PenReport:
    """`t_rate`, `q_rate`: the flown curve sampled at the driver's rate (holding samples at the
    ends included).  The line is followed locally: a sample between planned points i and i+1
    is compared with the stretch of line from point i - window to i + 1 + window."""
    P = np.asarray(tip_base, float)
    tip_base_now = tip(model, q_rate, np.eye(4))
    tip_table = tip_base_now @ T_table_base[:3, :3].T + T_table_base[:3, 3]
    height = float(np.max(np.abs(tip_table[:, 2] - paper_z)))

    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_at = np.concatenate([[0.0], np.cumsum(seg)])
    n_seg = len(P) - 1
    i = np.clip(np.searchsorted(traj.t, t_rate, side="right") - 1, 0, n_seg - 1)
    idx = np.clip(i[:, None] + np.arange(-window, window + 2), 0, n_seg - 1)          # (M,S)
    dist, u = _project(tip_base_now, P[idx], P[idx + 1])
    best = np.argmin(dist, axis=1)
    rows = np.arange(len(idx))
    off = dist[rows, best]
    s = s_at[idx[rows, best]] + u[rows, best] * seg[idx[rows, best]]
    at_knots = np.linalg.norm(tip(model, traj.q, np.eye(4)) - P, axis=1)
    off_line = float(max(off.max(), at_knots.max()))

    backwards = float(np.max(np.maximum.accumulate(s) - s))
    dt = np.diff(t_rate)
    along = np.diff(s) / dt
    speed = np.linalg.norm(np.diff(tip_base_now, axis=0), axis=1) / dt
    going = np.flatnonzero(along >= GOING * along.max()) if along.max() > 0 else []
    slowest = float(along[going[0]:going[-1] + 1].min()) if len(going) else 0.0
    return PenReport(height, off_line, backwards, slowest, float(speed.max()), float(s[-1]))
