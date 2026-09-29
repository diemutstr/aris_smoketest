"""Step 7: is the timed trajectory free as it will be flown?

A `Trajectory` means: between two samples, the cubic that matches position and velocity at both.
Timing keeps the straight pieces of the path exactly and rounds only its corners.  So:

- A cubic piece whose two samples lie on one straight piece of the path, with velocities along
  it, lies on that straight piece (everything in the cubic's formula is a point of the line or a
  vector along it).  If, along the line, it also stays between the straight piece's ends (the
  cubic's extremes are computed exactly), it is covered by the kernel's bound on that straight
  piece: the one the search proved (with room to spare), or a fresh one.  Rounding error is
  measured and charged.
- The other pieces (the rounded corners) are checked themselves.  A cubic strays from the
  straight chord between its ends by at most  max|q''| * dt^2 / 8  per joint; that is charged,
  weighted by how far each joint can move each capsule.  Runs of pieces are merged while the
  charge stays under `sag_tol`; a piece that bends more is cut at points of the cubic itself.

The verdict is the smallest clearance over both kinds, from `collide.edges_clearance_q`, with the
real margins (nothing to spare) and the arm against itself.
"""
from __future__ import annotations

import numpy as np

from aris.kernel.retime import sample
from aris.types import Trajectory

SAG_TOL = 3e-4        # m, the most any capsule may be charged for the cubic's bend per interval
LOOKAHEAD = 256       # samples looked at per kept interval (keeps each step one numpy call)
ON_LINE = 1e-7        # rad, the most a piece may stray from its straight piece and still count
                      # as lying on it (charged in full)


def piece_accel(traj: Trajectory) -> np.ndarray:
    """(N-1, 7): the largest |q''| on each cubic piece.  q'' is linear on a cubic piece, so it
    is largest at one of the piece's ends."""
    h = np.diff(traj.t)[:, None]
    dq = np.diff(traj.q, axis=0)
    v0, v1 = traj.qd[:-1], traj.qd[1:]
    a0 = (6.0 * dq - h * (4.0 * v0 + 2.0 * v1)) / h ** 2
    a1 = (-6.0 * dq + h * (2.0 * v0 + 4.0 * v1)) / h ** 2
    return np.maximum(np.abs(a0), np.abs(a1))


def on_path(traj: Trajectory, path: np.ndarray):
    """-> (segment index per piece, -1 if the piece is not on one straight piece of the path;
    how far each covered piece can stray from it, rad)."""
    A, U = path[:-1], np.diff(path, axis=0)                          # (M,7)
    L2 = np.maximum(np.einsum("mj,mj->m", U, U), 1e-300)
    rel = traj.q[:, None] - A[None]                                  # (N,M,7)
    s = np.einsum("nmj,mj->nm", rel, U) / L2                         # (N,M) along the line
    off = np.linalg.norm(rel - s[..., None] * U, axis=-1)            # (N,M) off the line
    sv = np.einsum("nj,mj->nm", traj.qd, U) / L2                     # d s / dt
    voff = np.linalg.norm(traj.qd[:, None] - sv[..., None] * U, axis=-1)
    h = np.diff(traj.t)[:, None]
    # off the line: the end residuals, plus the tangents' sideways parts times the most the
    # cubic's tangent weights reach (4/27)
    stray = np.maximum(off[:-1], off[1:]) + (4.0 / 27.0) * h * (voff[:-1] + voff[1:])
    lo, hi = _cubic_range(s[:-1], s[1:], h * sv[:-1], h * sv[1:])
    stray = stray + np.sqrt(L2) * np.maximum(0.0, np.maximum(-lo, hi - 1.0))
    seg = np.argmin(stray, axis=1)
    best = stray[np.arange(len(seg)), seg]
    return np.where(best <= ON_LINE, seg, -1), best


def _cubic_range(s0, s1, m0, m1):
    """Exact smallest and largest value on [0, 1] of the 1-D cubic Hermite through s0, s1 with
    end slopes m0, m1 (all arrays of one shape)."""
    a = 6 * s0 + 3 * m0 - 6 * s1 + 3 * m1
    b = -6 * s0 - 4 * m0 + 6 * s1 - 2 * m1
    c = m0
    val = lambda t: ((2 * t ** 3 - 3 * t ** 2 + 1) * s0 + (t ** 3 - 2 * t ** 2 + t) * m0
                     + (-2 * t ** 3 + 3 * t ** 2) * s1 + (t ** 3 - t ** 2) * m1)
    lo, hi = np.minimum(s0, s1), np.maximum(s0, s1)
    disc = b * b - 4 * a * c
    with np.errstate(divide="ignore", invalid="ignore"):
        r = np.sqrt(np.maximum(disc, 0.0))
        for t in ((-b + r) / (2 * a), (-b - r) / (2 * a), -c / b):
            ok = np.isfinite(t) & (t > 0) & (t < 1)
            v = np.where(ok, val(np.where(ok, t, 0.0)), s0)
            lo, hi = np.minimum(lo, v), np.maximum(hi, v)
    return lo, hi


def check_points(traj: Trajectory, reach: np.ndarray, todo: np.ndarray, sag_tol: float):
    """Intervals on the flown curve covering the pieces marked `todo`: -> (t_a, t_b, sag), the
    interval ends' times and each interval's per-joint sag bound."""
    acc = piece_accel(traj)
    t = traj.t
    ta, tb, sags = [], [], []
    i = 0
    n = len(t)
    while i < n - 1:
        if not todo[i]:
            i += 1
            continue
        m = min(LOOKAHEAD, n - 1 - i)
        run_end = np.flatnonzero(~todo[i:i + m])
        m = int(run_end[0]) if len(run_end) else m
        run = np.maximum.accumulate(acc[i:i + m], axis=0)             # (m, 7)
        span = (t[i + 1:i + m + 1] - t[i])[:, None] ** 2 / 8.0
        sag = run * span                                             # (m, 7) per joint, rad
        charge = (sag @ reach).max(axis=1)                           # (m,) worst capsule, m
        fits = np.flatnonzero(charge <= sag_tol)
        if len(fits):
            step = int(fits[-1]) + 1
            ta.append(t[i:i + 1]), tb.append(t[i + step:i + step + 1])
            sags.append(sag[step - 1][None])
        else:                                        # one piece bends too much: cut it
            step = 1
            parts = int(np.ceil(np.sqrt(charge[0] / sag_tol)))
            cut = np.linspace(t[i], t[i + 1], parts + 1)
            ta.append(cut[:-1]), tb.append(cut[1:])
            sags.append(np.repeat(sag[0][None] / parts ** 2, parts, axis=0))
        i += step
    if not ta:
        return np.zeros(0), np.zeros(0), np.zeros((0, 7))
    return np.concatenate(ta), np.concatenate(tb), np.concatenate(sags)


def flown_verdict(checker, arm, res, gates, path) -> tuple[str | None, tuple]:
    """(None if the timed trajectory is free as flown, else why not; (clearance as flown,)), the
    clearance counting obstacles and the arm against itself.  `path`: the waypoints it was
    timed from.

    The joint limits are read from retime's report, the exact extremes of the cubic pieces,
    with the gate's margin.
    """
    traj, rep = res.traj, res.report
    if not rep.inside:
        return "the timing check is over a limit", ()
    lo = arm.limits.q_min + gates.limit_margin
    hi = arm.limits.q_max - gates.limit_margin
    if np.any(rep.q_low < lo) or np.any(rep.q_high > hi):
        return "the flown path comes closer to a joint limit than the gate's margin", ()
    seg, stray = on_path(traj, path)
    covered = seg >= 0
    lever = 2.0 * float(np.max(np.linalg.norm(checker.reach, axis=0)))   # m/rad, pairs too
    margin = np.inf
    used = np.unique(seg[covered])
    if len(used):
        charge = np.zeros(len(used))
        np.maximum.at(charge, np.searchsorted(used, seg[covered]), stray[covered] * lever)
        # A straight piece the search already proved free with room to spare needs no second
        # look; any other is bounded now.
        room = min(checker.need_obst, checker.need_self)
        known = np.array([checker.was_proven(path[k], path[k + 1]) for k in used])
        margin = float(np.min(room - charge[known], initial=np.inf))
        if not known.all():
            margin = min(margin, float(np.min(checker.edge_margins(
                path[used[~known]], path[used[~known] + 1], charge[~known]))))
    # The corners; a failure is looked at again with ten times finer points before it counts.
    for sag_tol in (SAG_TOL, 0.1 * SAG_TOL):
        ta, tb, sag = check_points(traj, checker.reach, ~covered, sag_tol)
        if not len(ta):
            corner = np.inf
            break
        corner = float(np.min(checker.edge_margins(sample(traj, ta)[0], sample(traj, tb)[0],
                                                   None, sag)))
        if corner >= 0.0:
            break
    margin = min(margin, corner)
    if margin < 0.0:
        return f"flown clearance {margin * 1e3:.2f} mm", (margin,)
    return None, (margin,)
