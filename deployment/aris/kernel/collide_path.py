"""The collision check along joint paths, and from joint angles through the arm's tables.

Part of `aris.kernel.collide` (import everything from there).  `path_clearance` and its
relatives give a LOWER BOUND on the clearance along a whole piecewise-linear joint path,
including the motion between samples; `edges_clearance_q` does that for a batch of straight
edges at once, in compiled code when it is installed.
"""
from __future__ import annotations

from typing import Callable

import numpy as np

from aris.types import Body
from aris.kernel import collide_native as _cn
from aris.kernel import geometry as geo
from aris.kernel.collide import (_BIG, ClearanceDetail, _detail, capsule_clearance, clearance,
                                 clearance_detail, self_clearance)
from aris.kernel.collide_native import ArmTables, body_q, pack


def _interval_bound(ca, cb, delta):
    """Lower bound on an item's clearance anywhere on an interval.

    `ca`, `cb`: its clearance (or a lower bound of it) at the two ends; `delta`: how far any of
    its points can travel over the whole interval.  At fraction s it is at most delta*s from
    where it was at the start and delta*(1-s) from where it is at the end, and clearance is
    1-Lipschitz in that displacement, so clearance(s) >= max(ca - delta s, cb - delta (1 - s)).
    The minimum over s is where the two lines cross.
    """
    ca, cb = np.minimum(ca, _BIG), np.minimum(cb, _BIG)
    safe = np.where(delta > 0.0, delta, 1.0)
    s = np.where(delta > 0.0, np.clip((ca - cb + delta) / (2.0 * safe), 0.0, 1.0), 0.5)
    return np.maximum(ca - delta * s, cb - delta * (1.0 - s))


CAP = 0.25   # m: above this nobody needs the exact clearance, so no interval is refined for it


def _refine(values, q, R, tol, max_depth, max_evals, floor=None, cap=CAP) -> float:
    """The halving loop.  `values(Q)` -> (n, I) clearance of I items (capsules or capsule
    pairs); `R(I)` -> (J, I) bounds how far each item moves per radian of each joint.

    An interval is halved only while its bound is more than `tol` below both the smallest
    clearance seen so far and `cap`; so the answer is within `tol` of the true minimum when
    that is below `cap`, and at least `cap - tol` otherwise.  Nothing to hit anywhere: +inf at
    once.  With `floor`, stop as soon as the bound is proven at least `floor` (return it) or a
    sample reads below 0 (return that sample's clearance, which proves the path is not free).
    """
    c = values(q)
    m = float(np.min(c, initial=np.inf))
    if len(q) == 1 or m == np.inf or (floor is not None and m < 0.0):
        return m
    R = R(c.shape[1])
    qa, qb, ca, cb = q[:-1], q[1:], c[:-1], c[1:]
    done, evals = np.inf, 0
    for depth in range(max_depth + 1):
        delta = np.abs(qb - qa) @ R                                  # (intervals, I)
        lb = _interval_bound(ca, cb, delta).min(axis=1, initial=np.inf)
        now = min(done, float(np.min(lb, initial=np.inf)))
        if floor is not None and now >= floor:
            return now
        split = lb < min(m, cap) - tol
        if depth == max_depth or evals + split.sum() > max_evals:
            split[:] = False
        done = min(done, float(np.min(lb[~split], initial=np.inf)))
        if not split.any():
            break
        qa, qb, ca, cb = qa[split], qb[split], ca[split], cb[split]
        qm = 0.5 * (qa + qb)
        cm = values(qm)
        evals += len(qm)
        m = min(m, float(np.min(cm, initial=np.inf)))
        if floor is not None and m < 0.0:
            return m
        qa, qb = np.concatenate([qa, qm]), np.concatenate([qm, qb])
        ca, cb = np.concatenate([ca, cm]), np.concatenate([cm, cb])
    return min(done, m)


def _reach(reach, J, K):
    return np.ascontiguousarray(np.broadcast_to(np.asarray(reach, float).reshape(J, -1), (J, K)))


def _pair_reach(reach, J, pairs):
    """(J, P): a pair closes no faster than both its capsules move together."""
    R = np.asarray(reach, float).reshape(J, -1)
    i, j = (pairs[:, 0], pairs[:, 1]) if R.shape[1] > 1 else (np.zeros(len(pairs), int),) * 2
    return np.ascontiguousarray(R[:, i] + R[:, j])


def _pair_values(body: Body, pairs, margin) -> np.ndarray:
    """(N, P) clearance of each capsule pair of the arm against itself."""
    i, j = pairs[:, 0], pairs[:, 1]
    r = np.asarray(body.radius, float)
    p0, p1 = np.asarray(body.p0, float), np.asarray(body.p1, float)
    d = geo.segment_segment_distance(p0[:, i], p1[:, i], p0[:, j], p1[:, j])
    return d - r[i] - r[j] - margin


def path_clearance(body_of: Callable[[np.ndarray], Body], q: np.ndarray, obstacles,
                   reach: np.ndarray, drawing: bool = False, tol: float = 5e-4,
                   max_depth: int = 30, max_evals: int = 200_000, backend: str | None = None,
                   threads: int = 1, cap: float = CAP) -> float:
    """A lower bound on the clearance along the whole piecewise-linear joint path q (N, 7).

    `reach` (7,) or (7, K): for joint j (and capsule k), an upper bound on the distance from
    joint j's axis to any point on the capsule's axis, over every configuration; 0 for a
    capsule the joint does not move (`Arm.reach`).  Moving the joints by dq then moves any
    point of capsule k along a curve no longer than  sum_j reach[j, k] |dq_j|.  A capsule that
    does not move is charged nothing.

    Each interval between neighbouring samples is charged once, from both ends (never per
    sample and again per interval).  An interval whose bound is more than `tol` below the
    smallest clearance seen at any sample is halved, and both halves are looked at again,
    until no interval is.  The result then lies within `tol` below the true minimum, whatever
    the sampling it was handed; if `max_depth` or `max_evals` stop the halving first it is
    still a lower bound, only a looser one.  Clearance above `cap` is not refined: there the
    answer is only promised to be at least `cap - tol`.  No obstacles to check: +inf at once.
    """
    P = pack(obstacles)
    q = np.asarray(q, float).reshape(-1, 7)
    values = lambda Q: capsule_clearance(body_of(Q), P, drawing, True, backend, threads)
    if not len(P.names):
        return np.inf
    return _refine(values, q, lambda K: _reach(reach, 7, K), tol, max_depth, max_evals, cap=cap)


def path_self_clearance(body_of: Callable[[np.ndarray], Body], q: np.ndarray, pairs, margin,
                        reach: np.ndarray, tol: float = 5e-4, max_depth: int = 30,
                        max_evals: int = 200_000, cap: float = CAP) -> float:
    """As `path_clearance`, for the arm against itself over the capsule pairs (numpy).  A pair
    moves apart or together at most as fast as both its capsules together."""
    q = np.asarray(q, float).reshape(-1, 7)
    pairs = np.asarray(pairs, np.int64).reshape(-1, 2)
    if not len(pairs):
        return np.inf
    Rp = _pair_reach(reach, 7, pairs)
    return _refine(lambda Q: _pair_values(body_of(Q), pairs, margin), q, lambda _: Rp, tol,
                   max_depth, max_evals, cap=cap)


# --------------------------------------------------------------------------- from joint angles


def _Q(tables, Q):
    return np.ascontiguousarray(np.asarray(Q, float).reshape(-1, len(tables.dh)))


def clearance_q(tables: ArmTables, Q, obstacles, drawing: bool = False,
                backend: str | None = None, threads: int = 1) -> np.ndarray:
    """(N, J) joint angles -> (N,) clearance, as `clearance(body_q(tables, Q), ...)`."""
    Q, P = _Q(tables, Q), pack(obstacles)
    if _cn.native_on(backend):
        return _cn._native.clearance_q(tables.chain, tables.caps, P.scene, Q, drawing, threads)
    return clearance(body_q(tables, Q, "numpy"), P, drawing, backend="numpy")


def clearance_detail_q(tables: ArmTables, Q, obstacles, drawing: bool = False,
                       backend: str | None = None, threads: int = 1) -> ClearanceDetail:
    Q, P = _Q(tables, Q), pack(obstacles)
    if _cn.native_on(backend):
        val, arg = _cn._native.capsule_values_q(tables.chain, tables.caps, P.scene, Q, drawing,
                                                True, threads)
        return _detail(val, arg, tables.names, P)
    return clearance_detail(body_q(tables, Q, "numpy"), P, drawing, backend="numpy")


def self_clearance_q(tables: ArmTables, Q, pairs, margin: float, backend: str | None = None,
                     threads: int = 1) -> np.ndarray:
    Q = _Q(tables, Q)
    pairs = np.ascontiguousarray(np.asarray(pairs, np.int64).reshape(-1, 2))
    if _cn.native_on(backend) and len(pairs):
        return _cn._native.self_clearance_q(tables.chain, tables.radius, Q, pairs, float(margin),
                                            threads)
    return self_clearance(body_q(tables, Q, "numpy"), pairs, margin, backend="numpy")


def path_clearance_q(tables: ArmTables, reach, q, obstacles, drawing: bool = False,
                     tol: float = 5e-4, max_depth: int = 30, max_evals: int = 200_000,
                     backend: str | None = None, threads: int = 1, cap: float = CAP) -> float:
    """`path_clearance` with the body computed from the tables; the same method and answer."""
    q, P = _Q(tables, q), pack(obstacles)
    R = _reach(reach, len(tables.dh), len(tables.radius))
    if _cn.native_on(backend):
        return _cn._native.path_clearance_q(tables.chain, tables.caps, P.scene, R, q, drawing, tol,
                                            max_depth, max_evals, threads, cap)
    return path_clearance(lambda Q: body_q(tables, Q, "numpy"), q, P, R, drawing, tol,
                          max_depth, max_evals, backend="numpy", cap=cap)


def path_self_clearance_q(tables: ArmTables, reach, q, pairs, margin: float, tol: float = 5e-4,
                          max_depth: int = 30, max_evals: int = 200_000,
                          backend: str | None = None, cap: float = CAP) -> float:
    """`path_self_clearance` with the body computed from the tables."""
    q = _Q(tables, q)
    pairs = np.ascontiguousarray(np.asarray(pairs, np.int64).reshape(-1, 2))
    R = _reach(reach, len(tables.dh), len(tables.radius))
    if not len(pairs):
        return np.inf
    if _cn.native_on(backend):
        return _cn._native.path_self_q(tables.chain, tables.caps, R, q, pairs, float(margin), tol,
                                       max_depth, max_evals, cap)
    return path_self_clearance(lambda Q: body_q(tables, Q, "numpy"), q, pairs, margin, R, tol,
                               max_depth, max_evals, cap)


def edges_clearance_q(tables: ArmTables, reach, Qa, Qb, obstacles, drawing: bool = False,
                      tol: float = 5e-4, self_pairs=None, self_margin: float = 0.0,
                      floor: float | None = 0.0, threads: int = 0, max_depth: int = 30,
                      max_evals: int = 200_000, backend: str | None = None,
                      cap: float = CAP) -> np.ndarray:
    """(E,) one bound per straight joint-space edge Qa[e] -> Qb[e].

    With `floor=None` each is exactly `path_clearance_q` of the two-sample path, or, with
    `self_pairs`, the smaller of that and `path_self_clearance_q`.  With a `floor` an edge
    stops early once its bound is proven at least `floor` (the value is then a lower bound, at
    least `floor`) or a point on it is proven below 0 (the value is then that point's
    clearance, negative).  Only "at least floor" / "below 0" is promised then, which is all a
    planner asking "free or not" needs.  `threads` spreads the edges over threads (0 or 1: none).
    """
    Qa, Qb, P = _Q(tables, Qa), _Q(tables, Qb), pack(obstacles)
    if Qa.shape != Qb.shape:
        raise ValueError("Qa and Qb must have the same shape")
    pairs = np.zeros((0, 2), np.int64) if self_pairs is None else \
        np.ascontiguousarray(np.asarray(self_pairs, np.int64).reshape(-1, 2))
    J, K = len(tables.dh), len(tables.radius)
    R = _reach(reach, J, K)
    use_floor = floor is not None
    if _cn.native_on(backend):
        return _cn._native.edges_clearance_q(tables.chain, tables.caps, P.scene, R, Qa, Qb,
                                             drawing, pairs, float(self_margin), tol, max_depth,
                                             max_evals, use_floor, float(floor or 0.0),
                                             max(1, threads), cap)
    body_of = lambda Q: body_q(tables, Q, "numpy")
    obst = lambda Q: capsule_clearance(body_of(Q), P, drawing, True, "numpy")
    selfv = lambda Q: _pair_values(body_of(Q), pairs, self_margin)
    R_pairs = _pair_reach(R, J, pairs)
    out = np.empty(len(Qa))
    for e in range(len(Qa)):
        q = np.stack([Qa[e], Qb[e]])
        b = np.inf if not len(P.names) else \
            _refine(obst, q, lambda _: R, tol, max_depth, max_evals, floor, cap)
        if len(pairs) and not (use_floor and b < 0.0):
            b = min(b, _refine(selfv, q, lambda _: R_pairs, tol, max_depth, max_evals, floor, cap))
        out[e] = b
    return out
