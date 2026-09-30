"""Curves for the timing step: the path through the input samples, and the cubic that a
Trajectory means.

Two readings of the input samples.

**Corners of a polyline** (`round_corners`; free-space paths).  A path made of straight pieces
has corners, and at a corner the acceleration is an impulse however slowly the corner is flown
(lesson L44).  So before any timing each corner is rounded:

    near a corner, each point is replaced by an average of its neighbours along the path
    (three box averages in a row, over a window of width w).

A straight piece averages to itself, so a corner's rounding only reaches 1.5 w either side of
it, and the rounded path is the polyline plus one small, exactly known bump per corner:

    q(u) = polyline(u) + sum over corners j of  dm_j * bump(u - u_j, w_j)

where dm_j is the change of direction (slope) at corner j.  The bump and its first three
derivatives are short polynomials, so the path can be evaluated exactly at any u without a grid,
and each corner gets its own window.  A lone corner moves the path by 0.203 w |dm| at the corner
and nowhere more; the window is the widest that keeps every point within the deviation budget
(and the pen tip within its budget, if one is given), found by narrowing only the corners that
break it.  Each window also stays 1.5 w clear of the path's ends, so the ends are exact.

**Samples of a smooth curve** (`smooth_curve`; drawing, lowers, lifts).  The base is the cubic
spline through the samples (twice differentiable), and there are no corners, except where the
drawn line itself has one: where the spline's pen strays from the pen polyline, the spline is
cut at the sample where the pen turns most, and that corner is rounded as above.  The spline
pieces meet a cut with zero second derivative, so the bump keeps the path twice differentiable.

Deviation is measured at equal path parameter u, as a Euclidean distance in joint space, from
the polyline (corners) or from the spline (smooth).
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.linalg import solve_banded

BUMP_AT_CORNER = 0.203125   # bump(0, w) / w: the deviation of a lone corner per unit turn and width
NARROW_TRIES = 40           # rounds of narrowing the corners that break a budget
DENSE_WIDTH = 4.0           # a window never spans more than this many sample spacings: wider
                            # averaging gains nothing on a densely sampled curve, and costs time
_C = np.array([1.0, -3.0, 3.0, -1.0])   # truncated-power weights of three box averages
TIP_PROBES = 3              # smooth curve: pen measured at this many points inside each piece
CUT_SHARE = 0.5             # smooth curve: of the pen budget, what the spline alone may use


# --------------------------------------------------------------------------- the rounded path


@dataclass(frozen=True)
class Rounded:
    """A polyline, or a spline (`coef`), with rounded corners.  Evaluate it with `evaluate`."""
    u_knots: np.ndarray    # (N,) path parameter of the samples, 0 .. L
    q_knots: np.ndarray    # (N, 7)
    slope: np.ndarray      # (N-1, 7) dq/du of each straight piece (chord slopes for a spline)
    u_c: np.ndarray        # (C,) where the corners are
    dm: np.ndarray         # (C, 7) change of slope at each corner
    w: np.ndarray          # (C,) window width at each corner
    deviation: float       # largest joint-space distance found from the base (polyline, spline)
    tip_deviation: float | None   # largest pen-tip distance found from the input's pen path
    coef: np.ndarray | None = None       # (4, N-1, 7) the spline's cubic pieces; None: polyline
    tip_knots: np.ndarray | None = None  # (N, 3) spline: the input's pen positions, if asked

    @property
    def smooth(self) -> bool:
        return self.coef is not None


def polyline_at(u_knots: np.ndarray, q_knots: np.ndarray, u: np.ndarray) -> np.ndarray:
    """The input polyline evaluated at parameters u (clamped to its ends)."""
    u = np.clip(np.asarray(u, dtype=float), u_knots[0], u_knots[-1])
    k = np.clip(np.searchsorted(u_knots, u, side="right") - 1, 0, len(u_knots) - 2)
    f = ((u - u_knots[k]) / (u_knots[k + 1] - u_knots[k]))[:, None]
    return q_knots[k] + f * (q_knots[k + 1] - q_knots[k])


def _bump(t, order):
    """The bump and its derivatives in units of the window: t = x / w + 1.5, 0 < t < 3.

    order 0: bump / w     1: d bump / dx     2: w d2/dx2     3: w^2 d3/dx3
    """
    d = np.maximum(t[:, None] - np.arange(4.0), 0.0)
    if order == 0:
        return (d ** 4) @ _C / 24.0 - np.maximum(t - 1.5, 0.0)
    if order == 1:
        return (d ** 3) @ _C / 6.0 - (t >= 1.5)
    if order == 2:
        return (d ** 2) @ _C / 2.0
    return d @ _C


def _pairs(u_c, w, u):
    """(point index, corner index, t) for every point of sorted `u` inside a corner's window."""
    lo = np.searchsorted(u, u_c - 1.5 * w, side="right")
    hi = np.searchsorted(u, u_c + 1.5 * w, side="left")
    count = np.maximum(hi - lo, 0)
    corner = np.repeat(np.arange(len(u_c)), count)
    start = np.repeat(np.cumsum(count) - count, count)
    point = np.arange(len(corner)) - start + lo[corner]
    return point, corner, (u[point] - u_c[corner]) / w[corner] + 1.5


def spline_at(u_knots, coef, u, order=0):
    """d^order q / du^order of the piecewise cubic `coef` at u (clamped to its pieces)."""
    k = np.clip(np.searchsorted(u_knots, u, side="right") - 1, 0, len(u_knots) - 2)
    x = (u - u_knots[k])[:, None]
    c0, c1, c2, c3 = coef[0][k], coef[1][k], coef[2][k], coef[3][k]
    if order == 0:
        return ((c0 * x + c1) * x + c2) * x + c3
    if order == 1:
        return (3.0 * c0 * x + 2.0 * c1) * x + c2
    return 6.0 * c0 * x + 2.0 * c1 if order == 2 else 6.0 * c0 + 0.0 * x


def base_at(r: Rounded, u: np.ndarray) -> np.ndarray:
    """The path before rounding (the polyline or the spline) at sorted points u."""
    u = np.asarray(u, dtype=float)
    return spline_at(r.u_knots, r.coef, u) if r.smooth else polyline_at(r.u_knots, r.q_knots, u)


def evaluate(r: Rounded, u: np.ndarray, orders=(0,)) -> list[np.ndarray]:
    """The rounded path (order 0) and its derivatives d^k q / du^k at sorted points u."""
    u = np.asarray(u, dtype=float)
    point, corner, t = _pairs(r.u_c, r.w, u)
    out = []
    for k in orders:
        if r.smooth:
            base = spline_at(r.u_knots, r.coef, u, k)
        elif k == 0:
            base = polyline_at(r.u_knots, r.q_knots, u)
        elif k == 1:
            seg = np.clip(np.searchsorted(r.u_knots, u, side="right") - 1, 0, len(r.slope) - 1)
            base = r.slope[seg].copy()
        else:
            base = np.zeros((len(u), r.q_knots.shape[1]))
        weight = _bump(t, k) * r.w[corner] ** (1 - k)
        m = base.shape[1]
        flat = (point[:, None] * m + np.arange(m)).ravel()
        base += np.bincount(flat, (weight[:, None] * r.dm[corner]).ravel(),
                            minlength=len(u) * m).reshape(len(u), m)
        out.append(base)
    return out


def round_corners(u_knots: np.ndarray, q_knots: np.ndarray, deviation: float,
                  tip_of=None, tip_budget: float | None = None) -> Rounded | str:
    """Round every corner of the polyline as widely as the budgets allow; a reason if it can't.

    u_knots strictly increasing from 0.  Budgets are checked at the corners and halfway between
    samples, where a rounded path is furthest from its polyline.
    """
    L = float(u_knots[-1])
    slope = np.diff(q_knots, axis=0) / np.diff(u_knots)[:, None]
    dm = np.diff(slope, axis=0)
    turn = np.linalg.norm(dm, axis=1)
    keep = turn > 1e-12 * max(1.0, float(np.abs(slope).max()))
    u_c, dm, turn = u_knots[1:-1][keep], dm[keep], turn[keep]
    spacing = 0.5 * (np.diff(u_knots)[:-1] + np.diff(u_knots)[1:])[keep]
    # Widest window for a lone corner; for a densely sampled curve, the window whose average
    # bends the curve by the budget; and never past the ends of the path.
    w = np.minimum.reduce([deviation / (BUMP_AT_CORNER * np.maximum(turn, 1e-300)),
                           0.85 * np.sqrt(8.0 * deviation * spacing / np.maximum(turn, 1e-300)),
                           DENSE_WIDTH * spacing,
                           np.minimum(u_c, L - u_c) / 1.5])
    # Probe at every sample, and halfway along every piece that is long next to the windows at
    # its ends (on a densely sampled curve the deviation varies too slowly to need both).
    w_end = np.full(len(u_knots), np.inf)
    w_end[1:-1][keep] = w
    long_piece = np.diff(u_knots) > 0.25 * np.minimum(w_end[:-1], w_end[1:])
    probe = np.unique(np.concatenate([u_knots, 0.5 * (u_knots[1:] + u_knots[:-1])[long_piece]]))
    lin = polyline_at(u_knots, q_knots, probe)
    tip_lin = tip_of(lin) if tip_budget is not None else None
    r = Rounded(u_knots, q_knots, slope, u_c, dm, w, 0.0, None)
    return _fit_windows(r, probe, lin, tip_lin, deviation, tip_of, tip_budget)


def _fit_windows(r: Rounded, probe, lin, tip_lin, deviation, tip_of, tip_budget):
    """Narrow the windows of `r` until every probe is within both budgets of its reference
    (`lin` in joint space, `tip_lin` for the pen)."""
    u_c, w = r.u_c, r.w
    over = np.zeros(len(probe))
    tip_over = np.zeros(len(probe))
    redo = np.arange(len(probe))              # the points whose rounded position may have moved
    for _ in range(NARROW_TRIES):
        r = replace(r, w=w)
        q = evaluate(r, probe[redo])[0]
        over[redo] = np.linalg.norm(q - lin[redo], axis=1) / deviation
        if tip_lin is not None:
            tip_over[redo] = np.linalg.norm(tip_of(q) - tip_lin[redo], axis=1) / tip_budget
        worst = np.maximum(over, tip_over)
        if worst.max() <= 1.0:
            tip_dev = float(tip_over.max()) * tip_budget if tip_lin is not None else None
            return replace(r, deviation=float(over.max()) * deviation, tip_deviation=tip_dev)
        narrowed = _narrow(u_c, w, probe, worst)
        moved = narrowed < w
        if not moved.any():                   # over budget where no window reaches
            break
        redo = np.unique(_pairs(u_c[moved], w[moved], probe)[0])
        w = narrowed
    return f"corners could not be rounded within the budget in {NARROW_TRIES} rounds"


def _narrow(u_c, w, probe, over):
    """Narrow every corner whose window covers a point over budget, by that point's excess."""
    bad = over > 1.0
    point, corner, _ = _pairs(u_c, w, probe[bad])
    factor = np.ones(len(w))
    np.minimum.at(factor, corner, 0.95 / over[bad][point])
    return w * factor


# --------------------------------------------------------------------------- samples of a smooth curve


def smooth_curve(u_knots: np.ndarray, q_knots: np.ndarray, deviation: float, tip_of=None,
                 tip_budget: float | None = None) -> Rounded | str:
    """The spline through the samples, cut and rounded where the drawn line has a corner (the
    spline's pen more than CUT_SHARE of the budget from the pen polyline); a reason if the
    budgets can't be met.  Without a pen budget there are no cuts."""
    f = np.arange(1, TIP_PROBES + 1) / (TIP_PROBES + 1)
    probe = np.concatenate([u_knots, (u_knots[:-1, None] + np.diff(u_knots)[:, None] * f).ravel()])
    order = np.argsort(probe, kind="stable")
    piece = np.concatenate([np.full(len(u_knots), -1),              # a sample is on the line
                            np.repeat(np.arange(len(u_knots) - 1), TIP_PROBES)])
    probe, piece = probe[order], piece[order]
    tips = tip_of(q_knots) if tip_budget is not None else None
    tip_lin = polyline_at(u_knots, tips, probe) if tips is not None else None
    cuts = np.zeros(len(u_knots), dtype=bool)
    turn = _pen_turn(tips) if tips is not None else None
    for _ in range(len(u_knots)):
        coef = _spline(u_knots, q_knots, cuts)
        if tips is None:
            break
        lin = spline_at(u_knots, coef, probe)
        off = np.linalg.norm(tip_of(lin) - tip_lin, axis=1) > CUT_SHARE * tip_budget
        bad = np.unique(piece[off & (piece >= 0)])
        if not len(bad):
            break
        if len(u_knots) < 3:
            return "the curve through the two samples leaves the pen polyline"
        # Cut each bad piece where the pen turns most, among its two samples and their outer
        # neighbours (a corner makes the spline ring on the pieces next to it too).
        near = np.clip(bad[:, None] + np.arange(-1, 3), 1, len(u_knots) - 2)
        score = np.where(cuts[near], -1.0, turn[near])
        pick = np.argmax(score, axis=1)
        new = near[np.arange(len(bad)), pick][score[np.arange(len(bad)), pick] >= 0.0]
        new = np.unique(new)
        if not len(new):
            return "the curve through the samples leaves the pen polyline between two corners"
        cuts[new] = True
    lin = spline_at(u_knots, coef, probe)
    k = np.flatnonzero(cuts)
    h = (u_knots[k] - u_knots[k - 1])[:, None]
    dm = coef[2][k] - ((3.0 * coef[0][k - 1] * h + 2.0 * coef[1][k - 1]) * h + coef[2][k - 1])
    u_c, L = u_knots[k], float(u_knots[-1])
    w = np.minimum(deviation / (BUMP_AT_CORNER * np.maximum(np.linalg.norm(dm, axis=1), 1e-300)),
                   np.minimum(u_c, L - u_c) / 1.5)
    slope = np.diff(q_knots, axis=0) / np.diff(u_knots)[:, None]
    r = Rounded(u_knots, q_knots, slope, u_c, dm, w, 0.0, None, coef, tips)
    return _fit_windows(r, probe, lin, tip_lin, deviation, tip_of, tip_budget)


def _pen_turn(tips):
    """How far the pen polyline turns at each sample, radians (0 at the ends)."""
    d = np.diff(tips, axis=0)
    d = d / np.maximum(np.linalg.norm(d, axis=1, keepdims=True), 1e-300)
    cos = np.clip(np.sum(d[1:] * d[:-1], axis=1), -1.0, 1.0)
    return np.concatenate([[0.0], np.arccos(cos), [0.0]])


def _spline(u_knots, q_knots, cuts):
    """The cubic spline through the samples, in pieces between the cuts.  Not-a-knot at the two
    ends of the path (no condition is invented there), zero second derivative at a cut."""
    edges = np.concatenate([[0], np.flatnonzero(cuts), [len(u_knots) - 1]])
    zero = np.zeros(q_knots.shape[1])
    parts = []
    for a, b in zip(edges[:-1], edges[1:]):
        bc = ("not-a-knot" if a == 0 else (2, zero), "not-a-knot" if b == len(u_knots) - 1
              else (2, zero))
        parts.append(CubicSpline(u_knots[a:b + 1], q_knots[a:b + 1], axis=0, bc_type=bc).c)
    return np.concatenate(parts, axis=1)


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

    This is the clamped cubic spline through the knots: continuity of acceleration at every
    inner knot is one tridiagonal equation per knot.  With these velocities the acceleration
    is continuous, so jerk is finite everywhere and a finite-difference measurement does not
    grow with the sampling rate.
    """
    h = np.diff(t_knots)
    qd = np.zeros_like(q)
    if len(h) < 2:
        return qd
    slope = np.diff(q, axis=0) / h[:, None]
    band = np.zeros((3, len(h) - 1))
    band[0, 1:] = h[:-2]                          # above the diagonal: h_{i-1} v_{i+1}
    band[1] = 2.0 * (h[:-1] + h[1:])
    band[2, :-1] = h[2:]                          # below the diagonal: h_{i+1} v_{i-1}
    rhs = 3.0 * (h[1:, None] * slope[:-1] + h[:-1, None] * slope[1:])
    qd[1:-1] = solve_banded((1, 1), band, rhs)
    return qd


def peaks(t_knots: np.ndarray, q: np.ndarray, qd: np.ndarray):
    """Exact extremes of the piecewise cubic, per joint: lowest and highest position, largest
    |velocity|, |acceleration| and |jerk|.  Any sampled finite difference is an average of the
    true derivative, so it can never read more than these."""
    h = np.diff(t_knots)[:, None]
    q0, q1, v0, v1 = q[:-1], q[1:], qd[:-1], qd[1:]
    c2 = (3 * (q1 - q0) / h - 2 * v0 - v1) / h          # q = q0 + v0 s + c2 s^2 + c3 s^3
    c3 = (v0 + v1 - 2 * (q1 - q0) / h) / (h * h)
    a0, a1 = 2 * c2, 2 * c2 + 6 * c3 * h
    s_turn = np.where(c3 != 0, -c2 / np.where(c3 != 0, 3 * c3, 1.0), -1.0)   # where |v| peaks
    inner = (s_turn > 0) & (s_turn < h)
    s_in = np.where(inner, s_turn, 0.0)
    v_in = np.where(inner, v0 + 2 * c2 * s_in + 3 * c3 * s_in ** 2, 0.0)
    vel = np.maximum.reduce([np.abs(v0), np.abs(v1), np.abs(v_in)]).max(axis=0)
    acc = np.maximum(np.abs(a0), np.abs(a1)).max(axis=0)
    jerk = np.abs(6 * c3).max(axis=0)
    lo, hi = np.minimum(q0, q1).min(axis=0), np.maximum(q0, q1).max(axis=0)
    # A position extreme between knots needs the velocity to change sign there.
    k, j = np.nonzero((v0 * v1 < 0) | (v0 * v_in < 0) | (v1 * v_in < 0))
    hk, a, b, c, d = h[k, 0], q0[k, j], v0[k, j], c2[k, j], c3[k, j]
    for sign in (-1.0, 1.0):                              # where the velocity is zero
        disc = np.maximum(c * c - 3 * d * b, 0.0)
        with np.errstate(divide="ignore", invalid="ignore"):
            s = np.where(d != 0, (-c + sign * np.sqrt(disc)) / (3 * d),
                         np.where(c != 0, -b / (2 * c), -1.0))
        ok = (s > 0) & (s < hk)
        s = np.where(ok, s, 0.0)
        x = a + b * s + c * s ** 2 + d * s ** 3
        np.minimum.at(lo, j, x)
        np.maximum.at(hi, j, x)
    # The trajectory holds still before and after: an end acceleration is a step a 1 kHz
    # third difference reads as |a| / 1 ms.
    jerk = np.maximum(jerk, np.maximum(np.abs(a0[0]), np.abs(a1[-1])) / 1e-3)
    return lo, hi, vel, acc, jerk
