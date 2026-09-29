"""Step 7: is the timed trajectory free as it will be flown?

A `Trajectory` means: between two samples, the cubic that matches position and velocity at both.
The collision kernel's path bound covers straight joint-space pieces between samples.  A cubic
strays from the straight piece between its ends by at most  max|q''| * dt^2 / 8  per joint (the
usual bound for linear interpolation), so that much is charged on top, per joint, weighted by how
far each joint can move each capsule.

Checking every sample (one per 2 to 5 ms, about a thousand for a 4 s move) costs more than the
whole search.  Where the arm cruises, q'' is zero and the cubic is the straight piece, so a run of
samples can be checked as one straight piece at no extra charge.  `thin` keeps a subset of the
trajectory's own samples such that the charge over each kept interval, computed from the real
q'' of the cubics inside it, stays below `sag_tol`.  The verdict on the kept samples plus the
charge is a lower bound on the clearance of the flown curve.
"""
from __future__ import annotations

import numpy as np

from aris.types import Trajectory

SAG_TOL = 3e-4        # m, the most any capsule may be charged for the cubic's bend per interval
TIGHT_TOL = 1e-4      # m, the kernel's path-bound tolerance when a verdict is close
LOOKAHEAD = 256       # samples looked at per kept interval (keeps each step one numpy call)


def piece_accel(traj: Trajectory) -> np.ndarray:
    """(N-1, 7): the largest |q''| on each cubic piece.  q'' is linear on a cubic piece, so it
    is largest at one of the piece's ends."""
    h = np.diff(traj.t)[:, None]
    dq = np.diff(traj.q, axis=0)
    v0, v1 = traj.qd[:-1], traj.qd[1:]
    a0 = (6.0 * dq - h * (4.0 * v0 + 2.0 * v1)) / h ** 2
    a1 = (-6.0 * dq + h * (2.0 * v0 + 4.0 * v1)) / h ** 2
    return np.maximum(np.abs(a0), np.abs(a1))


def thin(traj: Trajectory, reach: np.ndarray, sag_tol: float = SAG_TOL):
    """-> (indices of the kept samples, (M-1, 7) per-joint sag bound of each kept interval)."""
    acc = piece_accel(traj)
    t = traj.t
    n = len(t)
    keep, sags = [0], []
    i = 0
    while i < n - 1:
        m = min(LOOKAHEAD, n - 1 - i)
        run = np.maximum.accumulate(acc[i:i + m], axis=0)             # (m, 7)
        span = (t[i + 1:i + m + 1] - t[i])[:, None] ** 2 / 8.0
        sag = run * span                                             # (m, 7) per joint, rad
        charge = (sag @ reach).max(axis=1)                           # (m,) worst capsule, m
        fits = np.flatnonzero(charge <= sag_tol)
        step = int(fits[-1]) + 1 if len(fits) else 1                 # one piece always goes
        keep.append(i + step)
        sags.append(sag[step - 1])
        i += step
    return np.array(keep), np.array(sags).reshape(-1, 7)


def flown_verdict(checker, arm, res, gates) -> tuple[str | None, tuple]:
    """(None if the timed trajectory is free as flown, else why not; (obstacle, self) clearance).

    The joint limits are read at 1 kHz (retime's own report) with the gate's margin; between two
    readings the cubic can stray by at most qdd_max * (1 ms)^2 / 8 = 1.3 microrad.
    """
    traj, rep = res.traj, res.report
    if not rep.inside:
        return "the 1 kHz check is over a limit", ()
    lo = arm.limits.q_min + gates.limit_margin
    hi = arm.limits.q_max - gates.limit_margin
    if np.any(rep.q_low < lo) or np.any(rep.q_high > hi):
        return "the flown path comes closer to a joint limit than the gate's margin", ()
    # First the thinned samples with a loose kernel tolerance (cheap when there is room); a
    # failure is looked at again on every sample with a tight tolerance before it counts.
    idx, sag = thin(traj, checker.reach)
    obst, own = checker.path_margins(traj.q[idx], sag,
                                     tol=max(TIGHT_TOL, 0.5 * checker.need_obst))
    if min(obst, own) < 0.0:
        idx, sag = thin(traj, checker.reach, sag_tol=0.0)
        obst, own = checker.path_margins(traj.q[idx], sag, tol=TIGHT_TOL)
    if obst < 0.0:
        return f"flown clearance to obstacles {obst * 1e3:.2f} mm", (obst, own)
    if own < 0.0:
        return f"flown clearance to itself {own * 1e3:.2f} mm", (obst, own)
    return None, (obst, own)
