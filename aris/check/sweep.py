"""Clearance along the whole flown motion, between the samples included.

1. Sample the flown curve so that no capsule point can move more than `step` between two
   samples.  The bound: on each piece of the curve each joint turns at most so far (exact, from
   the cubic), and a joint's turn moves a point at most by the point's distance from that
   joint's axis, which is measured where the arm actually is (plus what the joints further
   down the chain can change it by in between).
2. Measure every obstacle class at every sample.
3. On each interval, subtract a true bound on what can happen in between (`interval_bound`).
4. Where that bound binds, halve the interval and measure again, until it no longer binds.
The tightest class's answer lies at most `tol` under its true minimum, whatever sampling the
motion came in; every other class gets a true lower bound.

Exactness costs time, so each class is computed exactly only up to a threshold: a little above
its own smallest value, and at most `loose` above the tightest class.  A class further out than
that is reported as a lower bound ("at least"), which is still a true statement.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.check import timing
from aris.check.model import pose
from aris.check.scene import CLASSES, Scene, clearance, clearance_of

_PROBE = 33           # samples measured exactly first, to learn where exactness matters
_COARSE = 16          # first pass: steps this many times longer, to see where the arm is
_ROUNDS = 24          # halvings at most; each halves the travel bound of an interval


@dataclass(frozen=True)
class ClassResult:
    value: float          # lower bound on clearance beyond the demanded margin, whole motion
    exact: bool           # within tol of the true minimum (else: only a lower bound)
    where: str            # closest pair near that point
    t: float              # time of the closest sample


@dataclass(frozen=True)
class SweepResult:
    per_class: dict       # class -> ClassResult
    n_samples: int
    rounds: int
    q_min_margin: float   # smallest distance to a joint position limit over the samples, rad


def local_reach(delta, Da, Db, reach) -> np.ndarray:
    """(M,7,K): over an interval, the farthest capsule k gets from joint j's axis.

    delta (M,7) bounds each joint's turning over the interval; Da, Db (M,7,K) are the distances
    at its two ends.  Relative to joint j's axis a capsule moves only by the joints after j, so
    its distance from the axis changes by at most sum_{i>j} delta_i reach[i,k].
    """
    E = delta[:, :, None] * reach[None]                                      # (M,7,K)
    after = np.cumsum(E[:, ::-1], axis=1)[:, ::-1] - E                        # sum over i > j
    return np.minimum(np.minimum(Da, Db) + after, reach[None])


def _travel(delta, Da, Db, reach) -> np.ndarray:
    """(M,K): how far any point of capsule k can move over each interval."""
    return np.einsum("mj,mjk->mk", delta, local_reach(delta, Da, Db, reach))


def _class_travel(scene: Scene, T_k: np.ndarray) -> dict:
    """Per class: (M,) bound on how far anything that class measures can move, per interval."""
    m = scene.model
    move = ~m.is_fixed
    worst = lambda mask: T_k[:, mask].max(axis=1) if mask.any() else np.zeros(len(T_k))
    # Two capsules of the arm approach each other at most as fast as both move together;
    # the two fastest capsules bound every pair.
    two = -np.partition(-T_k, 1, axis=1)[:, :2].sum(axis=1) if T_k.shape[1] > 1 else T_k[:, 0]
    return dict(steel=worst(move), links=worst(move & ~m.is_pen & ~m.is_tool),
                tool=worst(move & m.is_tool), pen=worst(move & m.is_pen),
                walls=worst(move), parked=worst(move), self=two)


def _times(scene, tr, at, step):
    """Sample times with at most `step` of capsule travel between two of them."""
    reach = scene.model.travel
    tc = timing.place(tr.t, tr.cum @ reach.max(axis=1), _COARSE * step)
    _, _, Dc = pose(scene.model, at(tc), scene.T_table_base)
    W = local_reach(tr.between(tc[:-1], tc[1:]), Dc[:-1], Dc[1:], reach).max(axis=2)   # (Mc,7)
    grid = np.union1d(tr.t, tc)
    piece = np.clip(np.searchsorted(tc, 0.5 * (grid[1:] + grid[:-1])) - 1, 0, len(W) - 1)
    moved = np.concatenate([[0.0], np.cumsum(np.einsum(
        "mj,mj->m", tr.between(grid[:-1], grid[1:]), W[piece]))])
    return timing.place(grid, moved, step)


def sweep(scene: Scene, traj, step: float = 1e-3, tol: float = 2.5e-4,
          loose: float = 0.02) -> SweepResult:
    t_k, q_k, qd_k = traj.t, traj.q, traj.qd
    tr = timing.travel(t_k, q_k, qd_k)
    reach = scene.model.travel                                              # (7,K)
    at = lambda t: timing.hermite(t_k, q_k, qd_k, t)[0]

    probe = clearance(scene, at(np.linspace(t_k[0], t_k[-1], _PROBE))).value
    low = {c: float(np.min(probe[c])) for c in CLASSES}
    best = min(low.values())
    thr = {c: max(min(low[c], best + loose), 0.0) + 2 * step + tol for c in CLASSES}

    t = _times(scene, tr, at, step)
    Q = at(t)
    A, B, D = pose(scene.model, Q, scene.T_table_base)
    cl = clearance_of(scene, A, B, thr)
    val, pair, label = dict(cl.value), dict(cl.pair), cl.label
    qm = [_limit_margin(scene, Q)]
    Tc = _class_travel(scene, _travel(tr.between(t[:-1], t[1:]), D[:-1], D[1:], reach))
    lb = {c: timing.interval_bound(val[c][:-1], val[c][1:], Tc[c]) for c in CLASSES}
    rounds = 0
    while rounds < _ROUNDS:
        # An interval is split when its bound could hide a value more than `tol` below what
        # matters: the class's own smallest sample, but no lower than the tightest class
        # overall (or zero, if that one is already failing).  So only the tightest class, and
        # classes whose verdict could still change, are measured to `tol`; the others keep a
        # true lower bound that may be up to half a step below their own minimum.
        tightest = min(val[c].min() for c in CLASSES)
        bad = np.zeros(len(t) - 1, bool)
        for c in CLASSES:
            level = min(val[c].min(), max(tightest, 0.0), thr[c])
            bad |= (lb[c] < level - tol) & (Tc[c] > 1e-9)
        if not bad.any():
            break
        rounds += 1
        t, D = _split(scene, tr, at, thr, bad, t, D, val, pair, lb, Tc, qm)

    out = {}
    for c in CLASSES:
        k = int(np.argmin(lb[c])) if len(lb[c]) else 0
        v = float(min(val[c].min(), lb[c][k] if len(lb[c]) else np.inf))
        v = v if np.isfinite(val[c].min()) else np.inf          # nothing of this class here
        n = k if val[c][k] <= val[c][min(k + 1, len(t) - 1)] else k + 1
        p = int(pair[c][n])
        exact = bool(val[c].min() <= thr[c] and v >= val[c].min() - tol)
        out[c] = ClassResult(v, exact, label[c](p) if p >= 0 else "", float(t[n]))
    return SweepResult(out, len(t), rounds, float(min(qm)))


def _split(scene, tr, at, thr, bad, t, D, val, pair, lb, Tc, qm):
    """Halve the flagged intervals; measure the new samples; update the per-sample (val, pair)
    and per-interval (lb, Tc) tables in place of the old ones.  -> new t, D."""
    reach = scene.model.travel
    i = np.flatnonzero(bad)
    t_mid = 0.5 * (t[i] + t[i + 1])
    Q_mid = at(t_mid)
    qm.append(_limit_margin(scene, Q_mid))
    Am, Bm, Dm = pose(scene.model, Q_mid, scene.T_table_base)
    cm = clearance_of(scene, Am, Bm, thr)
    ta, tb = np.stack([t[i], t_mid], 1).ravel(), np.stack([t_mid, t[i + 1]], 1).ravel()
    Da = np.stack([D[i], Dm], 1).reshape(-1, *D.shape[1:])
    Db = np.stack([Dm, D[i + 1]], 1).reshape(-1, *D.shape[1:])
    Tn = _class_travel(scene, _travel(tr.between(ta, tb), Da, Db, reach))
    keep = np.repeat(np.arange(len(bad)), 1 + bad)              # interval order after the split
    halves = np.repeat(bad, 1 + bad)
    for c in CLASSES:
        va = np.stack([val[c][i], cm.value[c]], 1).ravel()
        vb = np.stack([cm.value[c], val[c][i + 1]], 1).ravel()
        lb_c, T_c = lb[c][keep], Tc[c][keep]
        lb_c[halves], T_c[halves] = timing.interval_bound(va, vb, Tn[c]), Tn[c]
        lb[c], Tc[c] = lb_c, T_c
        val[c] = np.insert(val[c], i + 1, cm.value[c])
        pair[c] = np.insert(pair[c], i + 1, cm.pair[c])
    return np.insert(t, i + 1, t_mid), np.insert(D, i + 1, Dm, axis=0)


def _limit_margin(scene: Scene, Q) -> float:
    m = scene.model
    return float(np.min(np.minimum(Q - m.q_min, m.q_max - Q)))
