"""Curves for the timing step: smoothing a joint polyline, and the cubic that a Trajectory means.

A path made of straight pieces has corners, and at a corner the acceleration is an impulse
however slowly the corner is flown (lesson L44).  So before any timing, the polyline is replaced
by a smooth path that stays within a given distance of it:

    each point of the path is replaced by a weighted average of its neighbours along the path.

On a straight piece the average of a straight line is the line itself, so straight pieces are
kept exactly; only the corners are rounded.  The weights are three box averages in a row, which
makes the result smooth up to its third derivative (acceleration and jerk stay finite).  The
window is as wide as the deviation budget allows at each place: the narrowest width that fits
everywhere is found first, then wider ones (4x, 16x, ...) are blended in wherever they fit too.  The two ends are handled by mirroring the path through its end
points, which keeps both end points exactly where they were.

Deviation is measured pointwise at equal path parameter: the smooth path at parameter u against
the polyline at the same u, as a Euclidean distance in joint space (radians).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import correlate1d, minimum_filter1d, uniform_filter1d

WIDTH_LEVELS = 8       # window widths tried: the narrowest that fits everywhere, times 4^k


# --------------------------------------------------------------------------- the smooth path


@dataclass(frozen=True)
class SmoothPath:
    """A smooth joint path sampled on a fine uniform grid of the path parameter u in [0, L]."""
    u: np.ndarray          # (M,) grid, u[0] = 0, u[-1] = L, spacing h
    q: np.ndarray          # (M, 7)
    dq: np.ndarray         # (M, 7) dq/du
    ddq: np.ndarray        # (M, 7) d2q/du2
    dddq: np.ndarray       # (M, 7) d3q/du3
    width: np.ndarray      # (M,) the averaging window used at each point, in units of u
    deviation: float       # largest joint-space distance to the polyline at equal u, rad

    @property
    def h(self) -> float:
        return float(self.u[1] - self.u[0])


def polyline_at(u_knots: np.ndarray, q_knots: np.ndarray, u: np.ndarray) -> np.ndarray:
    """The input polyline evaluated at parameters u (clamped to its ends)."""
    return np.stack([np.interp(u, u_knots, q_knots[:, j]) for j in range(q_knots.shape[1])],
                    axis=1)


def smooth_path(u_knots: np.ndarray, q_knots: np.ndarray, deviation: float,
                oversample: int = 8, max_points: int = 1_000_000,
                min_points: int = 2000) -> SmoothPath | str:
    """Round the corners of a polyline, staying within `deviation` of it.

    u_knots must be strictly increasing, starting at 0.  Returns a reason string if the corners
    are so sharp, for the length of the path, that the grid would exceed `max_points`.  The grid
    has at least `min_points`, so that the speed can be chosen finely along a straight path too.
    """
    L = float(u_knots[-1])
    slopes = np.diff(q_knots, axis=0) / np.diff(u_knots)[:, None]
    turn = np.linalg.norm(np.diff(slopes, axis=0), axis=1)
    # A single rounded corner deviates by about 0.2 * turn * width; start just below that.
    width = L / 3.0
    if turn.size and turn.max() > 0.0:
        width = min(width, 4.0 * deviation / turn.max())
    for _ in range(60):
        grid = _grid(L, width, oversample, max_points, min_points)
        if grid is None:
            return (f"corners too sharp for the length of the path: a window of {width:.3g} "
                    f"over a length of {L:.3g} needs more than {max_points} grid points")
        h, n_box = grid
        width = h * n_box
        n = int(round(L / h))
        pad = 3 * (n_box // 2) + 4
        u, q_lin = _extend(u_knots, q_knots, h, n, pad)
        q = _box3(q_lin, n_box)
        err = float(np.max(np.linalg.norm((q - q_lin)[pad:pad + n + 1], axis=1)))
        if err <= deviation:
            return _widen(u_knots, q_knots, h, n, n_box, deviation)
        width *= max(0.3, 0.95 * deviation / err)
    return "smoothing did not converge"


def _widen(u_knots, q_knots, h, n, n_box0, deviation) -> SmoothPath:
    """Smooth at widths n_box0 * 4^k and, wherever a wider window stays inside the budget, blend
    it in gradually.  Each blend is a weighted average of two paths that are both inside the
    budget there, so the result is too.  One sharp corner then does not force a narrow window
    (and tight, slow curves) on the whole path."""
    boxes = [n_box0]
    while len(boxes) < WIDTH_LEVELS and (4 * boxes[-1] + 1) * h <= u_knots[-1] / 3.0:
        boxes.append(4 * boxes[-1] + 1)
    pad = 3 * (boxes[-1] // 2) + 4
    u, q_lin = _extend(u_knots, q_knots, h, n, pad)
    q = _box3(q_lin, n_box0)
    width = np.full(len(u), n_box0 * h)
    for n_box in boxes[1:]:
        wide = _box3(q_lin, n_box)
        ok = (np.linalg.norm(wide - q_lin, axis=1) <= deviation).astype(float)
        step = (2 * n_box // 3) | 1                 # blend over about two window widths
        reach = 3 * (step // 2) + 1
        alpha = minimum_filter1d(ok, 2 * reach + 1, mode="nearest")
        for _ in range(3):
            alpha = uniform_filter1d(alpha, step, mode="nearest")
        alpha = np.clip(alpha, 0.0, 1.0)
        q += alpha[:, None] * (wide - q)
        width += alpha * (n_box * h - width)
    d1 = np.gradient(q, h, axis=0)
    d2 = np.gradient(d1, h, axis=0)
    d3 = np.gradient(d2, h, axis=0)
    keep = slice(pad, pad + n + 1)
    qk = q[keep].copy()
    qk[0], qk[-1] = q_knots[0], q_knots[-1]
    err = float(np.max(np.linalg.norm(qk - q_lin[keep], axis=1)))
    return SmoothPath(u[keep] * 1.0, qk, d1[keep], d2[keep], d3[keep], width[keep], err)


def _grid(L: float, width: float, oversample: int, max_points: int, min_points: int):
    """Grid spacing h (dividing L exactly) and box length in samples; None if too many points."""
    for k in range(oversample, 2, -1):
        n_box = 2 * k + 1
        n = int(np.ceil(L / (width / n_box)))
        if n < min_points:                      # short or straight path: longer box, same width
            n = min_points
            n_box = max(n_box, int(width / (L / n)) // 2 * 2 + 1)
        if n + 3 * n_box + 10 <= max_points:
            return L / n, n_box
    return None


def _extend(u_knots, q_knots, h, n, pad):
    """The polyline on the grid, continued past both ends by mirroring through the end points
    (odd reflection): a symmetric average at an end point is then the end point itself."""
    L = float(u_knots[-1])
    i = np.arange(-pad, n + pad + 1)
    u = i * h
    q = polyline_at(u_knots, q_knots, np.abs(u))
    lo, hi = i < 0, i > n
    q[lo] = 2.0 * q_knots[0] - q[lo]
    q[hi] = 2.0 * q_knots[-1] - polyline_at(u_knots, q_knots, 2.0 * L - u[hi])
    return u, q


def box3_kernel(n: int) -> np.ndarray:
    """Weights of three box averages of n samples in a row (3 n - 2 of them, summing to 1).

    Built from exact integer running sums: np.convolve on long boxes goes through a threaded
    BLAS dot product, which stalls on a loaded machine.
    """
    tri = np.minimum(np.arange(1, 2 * n), np.arange(2 * n - 1, 0, -1)).astype(float)
    run = np.concatenate([[0.0], np.cumsum(tri)])
    k = np.arange(3 * n - 2)
    return (run[np.minimum(k + 1, 2 * n - 1)] - run[np.clip(k - n + 1, 0, 2 * n - 1)]) / float(n) ** 3


def _box3(q, n_box):
    """Three box averages of n_box samples in a row.

    A sliding sum: the cost does not grow with the width, and its rounding error changes by one
    rounding per step, so even the third difference of the result stays clean.  (A running sum
    over the whole path would not: its rounding grows with the total.)
    """
    rows = np.ascontiguousarray(q.T)            # one contiguous row per joint: 10x faster
    for _ in range(3):
        rows = uniform_filter1d(rows, n_box, axis=1, mode="nearest")
    return rows.T


def path_at(path: SmoothPath, u: np.ndarray) -> np.ndarray:
    """The smooth path at arbitrary u, by cubic Hermite interpolation on its grid."""
    h = path.h
    x = np.clip(np.asarray(u, dtype=float), 0.0, path.u[-1]) / h
    k = np.minimum(np.floor(x).astype(int), len(path.u) - 2)
    tau = (x - k)[:, None]
    q, qn = path.q[k], path.q[k + 1]
    m, mn = path.dq[k] * h, path.dq[k + 1] * h
    return _hermite_value(q, qn, m, mn, tau)


# --------------------------------------------------------------------------- the cubic between samples


def _hermite_value(q0, q1, m0, m1, tau):
    t2, t3 = tau * tau, tau * tau * tau
    return ((2 * t3 - 3 * t2 + 1) * q0 + (t3 - 2 * t2 + tau) * m0
            + (-2 * t3 + 3 * t2) * q1 + (t3 - t2) * m1)


def hermite(t_knots: np.ndarray, q: np.ndarray, qd: np.ndarray, t):
    """Position, velocity and acceleration of the cubic matching q and qd at the knots.

    Before the first knot and after the last the arm holds still at the end configuration.
    """
    t = np.atleast_1d(np.asarray(t, dtype=float))
    k = np.clip(np.searchsorted(t_knots, t, side="right") - 1, 0, len(t_knots) - 2)
    dt = (t_knots[k + 1] - t_knots[k])[:, None]
    tau = ((t - t_knots[k])[:, None]) / dt
    inside = ((t >= t_knots[0]) & (t <= t_knots[-1]))[:, None]
    tau = np.clip(tau, 0.0, 1.0)
    q0, q1 = q[k], q[k + 1]
    m0, m1 = qd[k] * dt, qd[k + 1] * dt
    t2 = tau * tau
    pos = _hermite_value(q0, q1, m0, m1, tau)
    vel = ((6 * t2 - 6 * tau) * q0 + (3 * t2 - 4 * tau + 1) * m0
           + (-6 * t2 + 6 * tau) * q1 + (3 * t2 - 2 * tau) * m1) / dt
    acc = ((12 * tau - 6) * q0 + (6 * tau - 4) * m0
           + (-12 * tau + 6) * q1 + (6 * tau - 2) * m1) / (dt * dt)
    vel = np.where(inside, vel, 0.0)
    acc = np.where(inside, acc, 0.0)
    return pos, vel, acc


def rest_to_rest_velocities(t_knots: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Knot velocities that make the piecewise cubic twice differentiable, at rest at both ends.

    This is the clamped cubic spline through the knots.  With these velocities the acceleration
    is continuous across every knot, so jerk is finite everywhere and a finite-difference
    measurement does not grow with the sampling rate.
    """
    qd = CubicSpline(t_knots, q, axis=0, bc_type="clamped")(t_knots, 1)
    qd[0] = 0.0
    qd[-1] = 0.0
    return qd
