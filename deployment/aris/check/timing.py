"""The motion as it will be flown: the cubic between samples, and what the driver will read.

A trajectory's samples mean: between two samples the joints follow the cubic that matches q
and qd at both ends.  Everything the checker says is about that curve, never about the samples
alone, so a motion handed over coarse or fine gets the same verdict.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def hermite(t_k, q_k, qd_k, t) -> tuple[np.ndarray, np.ndarray]:
    """Position and velocity (len(t), 7) of the flown curve; held still outside [t_0, t_end]."""
    t = np.asarray(t, float)
    i = np.clip(np.searchsorted(t_k, t, side="right") - 1, 0, len(t_k) - 2)
    h = (t_k[i + 1] - t_k[i])[:, None]
    u = np.clip((t - t_k[i])[:, None] / h, 0.0, 1.0)
    p0, p1, v0, v1 = q_k[i], q_k[i + 1], qd_k[i] * h, qd_k[i + 1] * h
    # Bernstein form of the cubic: control points p0, p0 + v0/3, p1 - v1/3, p1.
    c1, c2 = p0 + v0 / 3.0, p1 - v1 / 3.0
    w = 1.0 - u
    q = w ** 3 * p0 + 3 * w * w * u * c1 + 3 * w * u * u * c2 + u ** 3 * p1
    qd = 3 * (w * w * (c1 - p0) + 2 * w * u * (c2 - c1) + u * u * (p1 - c2)) / h
    inside = ((t >= t_k[0]) & (t <= t_k[-1]))[:, None]
    return q, np.where(inside, qd, 0.0)


def speed_bound(t_k, q_k, qd_k) -> np.ndarray:
    """(n-1, 7): the largest |qd| of each joint on each piece of the curve, exactly.

    On a piece the velocity is a quadratic in the piece's own time; its largest magnitude is
    at an end or at the vertex.
    """
    h = np.diff(t_k)[:, None]
    p0, p1, v0, v1 = q_k[:-1], q_k[1:], qd_k[:-1] * h, qd_k[1:] * h
    # velocity * h = a u^2 + b u + c on u in [0, 1]
    a = 3 * v0 + 3 * v1 + 6 * (p0 - p1)
    b = -4 * v0 - 2 * v1 + 6 * (p1 - p0)
    c = v0
    safe = np.where(np.abs(a) > 1e-300, a, 1.0)
    u = np.where(np.abs(a) > 1e-300, np.clip(-b / (2 * safe), 0.0, 1.0), 0.0)
    vertex = a * u * u + b * u + c
    return np.maximum(np.maximum(np.abs(c), np.abs(a + b + c)), np.abs(vertex)) / h


@dataclass(frozen=True)
class Travel:
    """Bound on how far each joint turns: `cum[i, j]` >= the total |motion| of joint j from the
    start to knot i; linear in time on each piece."""
    t: np.ndarray        # (n,)
    cum: np.ndarray      # (n, 7)

    def between(self, ta, tb) -> np.ndarray:
        """(M, 7) bound on the joint turning between times ta and tb."""
        f = lambda x: np.stack([np.interp(x, self.t, self.cum[:, j]) for j in range(7)], 1)
        return np.maximum(f(tb) - f(ta), 0.0)


def travel(t_k, q_k, qd_k) -> Travel:
    vmax = speed_bound(t_k, q_k, qd_k)
    return Travel(t_k, np.vstack([np.zeros(7), np.cumsum(vmax * np.diff(t_k)[:, None], 0)]))


def place(t_grid, moved, step: float) -> np.ndarray:
    """Times such that `moved` (a bound on distance travelled, nondecreasing, linear between
    the grid times) grows by at most `step` from one to the next; both ends included."""
    n = int(np.ceil((moved[-1] - moved[0]) / step))
    if n <= 1:
        return np.array([t_grid[0], t_grid[-1]])
    x = moved + 1e-15 * np.arange(len(moved))          # strictly increasing for interp
    return np.interp(np.linspace(x[0], x[-1], n + 1), x, t_grid)


@dataclass(frozen=True)
class Rates:
    """What the driver reads at one rate: largest |finite difference| per joint."""
    rate_hz: float
    t: np.ndarray        # sample times, with three samples of holding still at each end
    q: np.ndarray
    vel: np.ndarray      # (7,) rad/s
    acc: np.ndarray      # (7,) rad/s^2
    jerk: np.ndarray     # (7,) rad/s^3


def rates(t_k, q_k, qd_k, rate_hz: float, sub: int = 1) -> list[Rates]:
    """The driver's readings at rate_hz * s for s in 1..sub, from one sampling (each coarser
    grid is every s-th sample of the finest one, on the same clock)."""
    fine = rate_hz * sub
    n = int(np.ceil((t_k[-1] - t_k[0]) * rate_hz)) * sub
    k = np.arange(-3 * sub, n + 3 * sub + 1)
    t = t_k[0] + k / fine
    q = hermite(t_k, q_k, qd_k, t)[0]
    out = []
    for s in (sub, 1) if sub > 1 else (1,):
        dt = s / fine
        qs, ts = q[::s], t[::s]
        d1, d2, d3 = np.diff(qs, 1, 0), np.diff(qs, 2, 0), np.diff(qs, 3, 0)
        out.append(Rates(fine / s, ts, qs, np.abs(d1).max(0) / dt,
                         np.abs(d2).max(0) / dt ** 2, np.abs(d3).max(0) / dt ** 3))
    return out


def interval_bound(va, vb, T) -> np.ndarray:
    """Lower bound on a clearance over an interval, from its values at both ends (va, vb) and a
    bound T on how far anything moves in between: at fraction s it is at least va - T s and at
    least vb - T (1 - s); the worst s is where those two lines cross."""
    va, vb = np.minimum(va, 1e9), np.minimum(vb, 1e9)      # "nothing to hit" stays finite
    s = np.clip((va - vb + T) / np.where(T > 0, 2 * T, 1.0), 0.0, 1.0)
    return np.where(T > 0, np.maximum(va - T * s, vb - T * (1 - s)), np.minimum(va, vb))
