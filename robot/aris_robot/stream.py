"""From a timed joint trajectory to the reference stream of the impedance controller.  No ROS.

A trajectory means: between two knots, the cubic that matches q and qd at both.  The driver
samples that cubic every millisecond (the controller's rate) and sends the samples a little
ahead of time, in chunks.  The controller joins neighbouring samples with the same kind of
cubic, so what it tracks is the trajectory that was checked, to rounding.

Each sample also carries the pen force as a 3-vector in the base frame: the setpoint from
`force.profile` along minus the paper normal (the arm pushes the pen into the paper).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

RATE_HZ = 1000.0


def cubic(t_knots, q_knots, qd_knots, t) -> tuple[np.ndarray, np.ndarray]:
    """Position and velocity (len(t), 7) of the trajectory at times t; at rest outside it.

    Written out here in the power basis (the planner's copy uses the Hermite basis, the
    checker's the Bernstein one) so that a test comparing them compares two derivations.
    """
    tk = np.asarray(t_knots, float)
    qk, vk = np.asarray(q_knots, float), np.asarray(qd_knots, float)
    t = np.atleast_1d(np.asarray(t, float))
    i = np.clip(np.searchsorted(tk, t, side="right") - 1, 0, len(tk) - 2)
    h = (tk[i + 1] - tk[i])[:, None]
    x = np.clip(t - tk[i], 0.0, None)[:, None]
    x = np.minimum(x, h)
    q0, q1, v0, v1 = qk[i], qk[i + 1], vk[i], vk[i + 1]
    c2 = (3.0 * (q1 - q0) / h - 2.0 * v0 - v1) / h
    c3 = (2.0 * (q0 - q1) / h + v0 + v1) / (h * h)
    q = q0 + x * (v0 + x * (c2 + x * c3))
    qd = v0 + x * (2.0 * c2 + 3.0 * x * c3)
    inside = ((t >= tk[0]) & (t <= tk[-1]))[:, None]
    return q, np.where(inside, qd, 0.0)


def grid(t_knots, rate: float = RATE_HZ) -> np.ndarray:
    """Sample times from the first knot every 1/rate seconds, ending exactly on the last knot."""
    t0, t1 = float(t_knots[0]), float(t_knots[-1])
    n = int(np.floor((t1 - t0) * rate + 1e-9))
    t = t0 + np.arange(n + 1) / rate
    if t1 - t[-1] > 1e-9:
        t = np.append(t, t1)
    else:
        t[-1] = t1
    return t


@dataclass(frozen=True)
class Samples:
    """A whole motion as reference samples, times from the motion's start."""
    t: np.ndarray        # (n,)
    q: np.ndarray        # (n, 7)
    qd: np.ndarray       # (n, 7)
    f: np.ndarray        # (n, 3) N, base frame, the force the arm applies at the pen tip

    def __len__(self) -> int:
        return len(self.t)


def samples(traj, force_fn, normal_base, rate: float = RATE_HZ) -> Samples:
    """The trajectory sampled at `rate`; `force_fn(t)` (motion time) the pressing force in N."""
    t = grid(traj.t, rate)
    q, qd = cubic(traj.t, traj.q, traj.qd, t)
    rel = t - float(traj.t[0])
    press = np.asarray(force_fn(rel), float).reshape(-1)
    n = np.asarray(normal_base, float)
    f = -press[:, None] * (n / np.linalg.norm(n))[None, :]
    return Samples(rel, q, qd, f)


@dataclass(frozen=True)
class Chunk:
    """What one reference message carries: consecutive samples of one stream."""
    stream: int
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    f: np.ndarray
    last: bool           # the stream ends with this chunk: hold at its final sample


class Pacer:
    """Which samples to send now: everything due within `lead` seconds of the stream clock.

    The controller's clock starts when the first chunk arrives, so a driver that sends
    `lead` ahead of its own clock (started when it sent the first chunk) is always ahead.
    """

    def __init__(self, s: Samples, stream: int, lead: float = 0.1, max_chunk: int = 200):
        self.s, self.stream, self.lead, self.max_chunk = s, stream, lead, max_chunk
        self.sent = 0

    @property
    def done(self) -> bool:
        return self.sent >= len(self.s)

    def due(self, elapsed: float) -> list[Chunk]:
        """The chunks to send when the stream clock reads `elapsed` seconds."""
        end = int(np.searchsorted(self.s.t, elapsed + self.lead, side="right"))
        out = []
        while self.sent < end:
            k = min(end, self.sent + self.max_chunk)
            sl = slice(self.sent, k)
            out.append(Chunk(self.stream, self.s.t[sl], self.s.q[sl], self.s.qd[sl],
                             self.s.f[sl], k >= len(self.s)))
            self.sent = k
        return out
