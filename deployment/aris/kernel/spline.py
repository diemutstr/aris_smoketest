"""Curves for the timing step: smoothing a joint polyline, and the cubic that a Trajectory means.

A path made of straight pieces has corners, and at a corner the acceleration is an impulse
however slowly the corner is flown (lesson L44).  So before any timing, the polyline is replaced
by a smooth path that stays within a given distance of it:

    each point of the path is replaced by a weighted average of its neighbours along the path,
    over a window of fixed width.

On a straight piece the average of a straight line is the line itself, so straight pieces are
kept exactly; only the corners are rounded.  The weights are three box averages in a row, which
makes the result smooth up to its third derivative (acceleration and jerk stay finite).  The
window width is the largest one whose result stays within the deviation budget, found by
shrinking until it does.  The two ends are handled by mirroring the path through its end
points, which keeps both end points exactly where they were.

Deviation is measured pointwise at equal path parameter: the smooth path at parameter u against
the polyline at the same u, as a Euclidean distance in joint space (radians).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import correlate1d


# --------------------------------------------------------------------------- the smooth path


@dataclass(frozen=True)
class SmoothPath:
    """A smooth joint path sampled on a fine uniform grid of the path parameter u in [0, L]."""
    u: np.ndarray          # (M,) grid, u[0] = 0, u[-1] = L, spacing h
    q: np.ndarray          # (M, 7)
    dq: np.ndarray         # (M, 7) dq/du
    ddq: np.ndarray        # (M, 7) d2q/du2
    dddq: np.ndarray       # (M, 7) d3q/du3
    width: float           # the averaging window, in units of u
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
        path = _smooth_on_grid(u_knots, q_knots, h, n_box)
        err = float(np.max(np.linalg.norm(path.q - polyline_at(u_knots, q_knots, path.u),
                                          axis=1)))
        if err <= deviation:
            return SmoothPath(path.u, path.q, path.dq, path.ddq, path.dddq, width, err)
        width *= max(0.3, 0.95 * deviation / err)
    return "smoothing did not converge"


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


def _smooth_on_grid(u_knots, q_knots, h, n_box) -> SmoothPath:
    L = float(u_knots[-1])
    n = int(round(L / h))
    pad = 3 * (n_box // 2) + 4                    # reach of three boxes, plus the difference stencils
    i = np.arange(-pad, n + pad + 1)
    u = i * h
    q = polyline_at(u_knots, q_knots, np.abs(u))
    # Mirror through the end points (odd reflection): the average of a symmetric window at an
    # end point is then the end point itself.
    lo, hi = i < 0, i > n
    q[lo] = 2.0 * q_knots[0] - q[lo]
    q[hi] = 2.0 * q_knots[-1] - polyline_at(u_knots, q_knots, 2.0 * L - u[hi])
    kernel = np.ones(n_box)
    kernel = np.convolve(np.convolve(kernel, kernel), kernel)
    kernel /= kernel.sum()
    qs = correlate1d(q, kernel, axis=0, mode="nearest")
    d1 = np.gradient(qs, h, axis=0)
    d2 = np.gradient(d1, h, axis=0)
    d3 = np.gradient(d2, h, axis=0)
    keep = slice(pad, pad + n + 1)
    qk = qs[keep].copy()
    qk[0], qk[-1] = q_knots[0], q_knots[-1]
    return SmoothPath(u[keep] * 1.0, qk, d1[keep], d2[keep], d3[keep], h * n_box, 0.0)


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
