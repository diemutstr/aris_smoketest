"""Collision check: how far the arm's collision body is from the obstacles, in batches.

Clearance of one capsule against one obstacle = distance between their surfaces minus the
margin that obstacle demands.  The clearance of a configuration is the smallest of these over
all capsule-obstacle pairs.  At least 0 means free.  Negative means the margin is violated;
a capsule that cuts into a box or another capsule reads distance 0 (so clearance is minus its
radii and the margin), a capsule through a plane reads the signed depth.  Capsules the body
marks as fixed (the base, inside the arm's own mount) are never checked against obstacles.

`path_clearance` also covers the motion between the samples of a joint path.

Two engines give the same answers: the compiled module `aris_collide_native` (built from
native/collide, used when it is importable) and plain numpy.  `backend()` says which is in use;
every call takes `backend="numpy"` or `"native"` to choose, for tests.  The `_q` calls go from
joint configurations straight to clearance, with the arm's chain given as a table
(`arm_tables(arm)`), so a planner's inner loop never passes through Python per configuration.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from aris.types import Body
from aris.kernel import collide_native as _cn
from aris.kernel import geometry as geo
from aris.kernel.collide_native import (ArmTables, Packed, arm_tables, backend,  # noqa: F401
                                        body_q, pack)

_CHUNK_PAIRS = 1 << 16       # capsule-obstacle pairs handled at once; bounds memory, fits cache
_BIG = 1e6                   # stands in for "nothing to hit" so the interval arithmetic stays finite


# --------------------------------------------------------------------------- per-pair clearances


def _box_block(p0, p1, r, P, keep=None):
    """Clearances against the boxes: (n, K, Mb) for every pair, or, with `keep` (an (n, K, Mb)
    mask), a flat array for the kept pairs only.  Same arithmetic per pair either way."""
    if keep is None:
        d = geo.segment_box_distance(p0[:, :, None], p1[:, :, None], P.box_R, P.box_c, P.box_h)
        return d - r[:, None] - P.box_m
    ni, ki, mi = np.nonzero(keep)
    d = geo.segment_box_distance(p0[ni, ki], p1[ni, ki], P.box_R[mi], P.box_c[mi], P.box_h[mi])
    return d - r[ki] - P.box_m[mi]


def _cap_block(p0, p1, r, P, keep=None):
    """As `_box_block`, against the capsule obstacles."""
    if keep is None:
        d = geo.segment_segment_distance(p0[:, :, None], p1[:, :, None], P.cap_a, P.cap_b)
        return d - r[:, None] - P.cap_rm
    ni, ki, mi = np.nonzero(keep)
    d = geo.segment_segment_distance(p0[ni, ki], p1[ni, ki], P.cap_a[mi], P.cap_b[mi])
    return d - r[ki] - P.cap_rm[mi]


def _plane_block(p0, p1, r, is_pen, P, drawing):
    """(n, K, Mp) clearances against the planes; +inf where a pair is not checked."""
    d = geo.segment_plane_distance(p0[:, :, None], p1[:, :, None], P.pl_n, P.pl_off)
    margin = np.where(is_pen[:, None], P.pl_pen_m, P.pl_m)
    val = d - r[:, None] - margin
    if drawing:
        val = np.where(is_pen[:, None] & P.pl_paper, np.inf, val)
    return val


def _midpoint_bounds(p0, p1, r, P):
    """Cheap bounds from each capsule's midpoint, for the boxes and the capsule obstacles.

    Every point of a segment is within half its length of the midpoint, and distance is
    1-Lipschitz, so  dist(midpoint) - half length <= true distance <= dist(midpoint).
    Returns the upper bounds (n, K, Mb) and (n, K, Mc), and the half lengths (n, K).
    """
    c = 0.5 * (p0 + p1)[:, :, None]
    half = 0.5 * np.sqrt(geo.dot(p1 - p0, p1 - p0))
    ub_b = geo.point_box_distance(c, P.box_R, P.box_c, P.box_h) - r[:, None] - P.box_m
    ub_c = geo.point_segment_distance(c, P.cap_a, P.cap_b) - r[:, None] - P.cap_rm
    return ub_b, ub_c, half


def _chunk_values(p0, p1, r, is_pen, P, drawing, prune):
    """(n, K, M) clearance of every pair, obstacles in `P.names` order.

    With `prune`, a box or capsule pair whose midpoint lower bound is already above the best
    upper bound of its configuration is not computed exactly; it keeps its lower bound.  That
    pair cannot be the configuration's minimum (its true value is at least the lower bound,
    which is above the minimum), so the per-configuration minimum and which pair attains it
    are identical with and without pruning, and every value stays a lower bound of the truth.
    """
    n = p0.shape[0]
    planes = _plane_block(p0, p1, r, is_pen, P, drawing)
    if not prune:
        return np.concatenate([_box_block(p0, p1, r, P), planes, _cap_block(p0, p1, r, P)], axis=2)
    ub_b, ub_c, half = _midpoint_bounds(p0, p1, r, P)
    best = np.min([x.reshape(n, -1).min(axis=1, initial=np.inf) for x in (ub_b, ub_c, planes)],
                  axis=0)[:, None, None]
    ob, oc = ub_b - half[:, :, None], ub_c - half[:, :, None]
    kb, kc = ob <= best, oc <= best
    ob[kb] = _box_block(p0, p1, r, P, kb)
    oc[kc] = _cap_block(p0, p1, r, P, kc)
    return np.concatenate([ob, planes, oc], axis=2)


def _caps(body: Body):
    K = np.shape(body.p0)[1]
    fixed = np.zeros(K, bool) if body.is_fixed is None else np.asarray(body.is_fixed, bool)
    return np.asarray(body.radius, float), np.asarray(body.is_pen, bool), fixed



def _per_capsule(body: Body, obstacles, drawing: bool, prune: bool, backend=None, threads=1):
    """(N, K) smallest clearance of each capsule over the obstacles, and (N, K) which one.

    Fixed capsules (`body.is_fixed`) are not checked: +inf, obstacle -1.
    """
    P = pack(obstacles)
    r, is_pen, fixed = _caps(body)
    if _cn.native_on(backend):
        val, arg = _cn._native.capsule_values(body.p0, body.p1, (r, is_pen, fixed), P.scene,
                                          drawing, prune, threads)
        return val, arg, P
    N, K = np.shape(body.p0)[:2]
    live = np.flatnonzero(~fixed)
    p0 = np.asarray(body.p0, float)[:, live]
    p1 = np.asarray(body.p1, float)[:, live]
    M, L = len(P.names), len(live)
    val = np.full((N, K), np.inf)
    arg = np.full((N, K), -1, np.int64)
    if M == 0 or N == 0 or L == 0:
        return val, arg, P
    step = max(1, _CHUNK_PAIRS // (L * M))
    for s in range(0, N, step):
        v = _chunk_values(p0[s:s + step], p1[s:s + step], r[live], is_pen[live], P, drawing, prune)
        a = np.argmin(v, axis=2)
        arg[s:s + step, live] = a
        val[s:s + step, live] = np.take_along_axis(v, a[:, :, None], axis=2)[:, :, 0]
    arg[~np.isfinite(val)] = -1
    return val, arg, P


# --------------------------------------------------------------------------- the calls


@dataclass(frozen=True)
class ClearanceDetail:
    """Per configuration: the clearance and the closest capsule-obstacle pair (-1: none)."""
    value: np.ndarray                  # (N,) metres beyond the demanded margin
    capsule: np.ndarray                # (N,) index into capsule_names
    obstacle: np.ndarray               # (N,) index into obstacle_names
    capsule_names: tuple[str, ...]
    obstacle_names: tuple[str, ...]    # boxes, then planes, then capsules


def capsule_clearance(body: Body, obstacles, drawing: bool = False, prune: bool = True,
                      backend: str | None = None, threads: int = 1) -> np.ndarray:
    """(N, K) per capsule.  Exact where a capsule holds its configuration's minimum; elsewhere
    a lower bound when `prune` is on (see `_chunk_values`)."""
    return _per_capsule(body, obstacles, drawing, prune, backend, threads)[0]


def clearance(body: Body, obstacles, drawing: bool = False, prune: bool = True,
              backend: str | None = None, threads: int = 1) -> np.ndarray:
    """(N,) smallest clearance over all capsule-obstacle pairs.  At least 0 means free."""
    return capsule_clearance(body, obstacles, drawing, prune, backend, threads).min(
        axis=1, initial=np.inf)


def _detail(val, arg, names, P) -> ClearanceDetail:
    N, K = val.shape
    k = np.argmin(val, axis=1) if K else np.zeros(N, np.int64)
    v = val[np.arange(N), k] if K else np.full(N, np.inf)
    m = arg[np.arange(N), k] if K else np.full(N, -1)
    k = np.where(np.isfinite(v), k, -1)
    return ClearanceDetail(v, k, m, tuple(names), P.names)


def clearance_detail(body: Body, obstacles, drawing: bool = False, prune: bool = True,
                     backend: str | None = None, threads: int = 1) -> ClearanceDetail:
    val, arg, P = _per_capsule(body, obstacles, drawing, prune, backend, threads)
    return _detail(val, arg, body.names, P)


def self_clearance(body: Body, pairs: np.ndarray, margin: float, backend: str | None = None,
                   threads: int = 1) -> np.ndarray:
    """(N,) smallest clearance between the capsule pairs (i, j) of the same arm."""
    pairs = np.asarray(pairs, np.int64).reshape(-1, 2)
    p0, p1 = np.asarray(body.p0, float), np.asarray(body.p1, float)
    r = np.asarray(body.radius, float)
    N = p0.shape[0]
    if len(pairs) == 0:
        return np.full(N, np.inf)
    if _cn.native_on(backend):
        return _cn._native.self_clearance(p0, p1, r, pairs, float(margin), threads)
    i, j = pairs[:, 0], pairs[:, 1]
    out = np.empty(N)
    step = max(1, _CHUNK_PAIRS // len(pairs))
    for s in range(0, N, step):
        a0, a1, b0, b1 = p0[s:s + step, i], p1[s:s + step, i], p0[s:s + step, j], p1[s:s + step, j]
        d = geo.segment_segment_distance(a0, a1, b0, b1)
        out[s:s + step] = (d - r[i] - r[j] - margin).min(axis=1)
    return out


# --------------------------------------------------------------------------- along a path


def _interval_bound(ca, cb, delta):
    """Lower bound on a capsule's clearance anywhere on an interval.

    `ca`, `cb`: its clearance (or a lower bound of it) at the two ends; `delta`: how far any of
    its points can travel over the whole interval.  At fraction s the capsule is at most
    delta*s from where it was at the start and delta*(1-s) from where it is at the end, and
    clearance is 1-Lipschitz in that displacement, so clearance(s) >= max(ca - delta s,
    cb - delta (1 - s)).  The minimum over s is where the two lines cross.
    """
    ca, cb = np.minimum(ca, _BIG), np.minimum(cb, _BIG)
    safe = np.where(delta > 0.0, delta, 1.0)
    s = np.where(delta > 0.0, np.clip((ca - cb + delta) / (2.0 * safe), 0.0, 1.0), 0.5)
    return np.maximum(ca - delta * s, cb - delta * (1.0 - s))


def path_clearance(body_of: Callable[[np.ndarray], Body], q: np.ndarray, obstacles,
                   reach: np.ndarray, drawing: bool = False, tol: float = 5e-4,
                   max_depth: int = 30, max_evals: int = 200_000, backend: str | None = None,
                   threads: int = 1) -> float:
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
    still a lower bound, only a looser one.
    """
    P = pack(obstacles)
    cc = lambda Q: capsule_clearance(body_of(Q), P, drawing, True, backend, threads)
    q = np.asarray(q, float).reshape(-1, 7)
    c = cc(q)                                                        # (N, K)
    K = c.shape[1]
    R = np.broadcast_to(np.asarray(reach, float).reshape(7, -1), (7, K))
    m = float(np.min(c, initial=np.inf))
    if len(q) == 1:
        return m
    qa, qb, ca, cb = q[:-1], q[1:], c[:-1], c[1:]
    done, evals = np.inf, 0
    for depth in range(max_depth + 1):
        delta = np.abs(qb - qa) @ R                                  # (I, K)
        lb = _interval_bound(ca, cb, delta).min(axis=1, initial=np.inf)
        split = lb < m - tol
        if depth == max_depth or evals + split.sum() > max_evals:
            split[:] = False
        done = min(done, float(np.min(lb[~split], initial=np.inf)))
        if not split.any():
            break
        qa, qb, ca, cb = qa[split], qb[split], ca[split], cb[split]
        qm = 0.5 * (qa + qb)
        cm = cc(qm)
        evals += len(qm)
        m = min(m, float(np.min(cm, initial=np.inf)))
        qa, qb = np.concatenate([qa, qm]), np.concatenate([qm, qb])
        ca, cb = np.concatenate([ca, cm]), np.concatenate([cm, cb])
    return min(done, m)


# --------------------------------------------------------------------------- from joint angles


def clearance_q(tables: ArmTables, Q, obstacles, drawing: bool = False,
                backend: str | None = None, threads: int = 1) -> np.ndarray:
    """(N, J) joint angles -> (N,) clearance, as `clearance(body_q(tables, Q), ...)`."""
    Q = np.ascontiguousarray(np.asarray(Q, float).reshape(-1, len(tables.dh)))
    P = pack(obstacles)
    if _cn.native_on(backend):
        return _cn._native.clearance_q(tables.chain, tables.caps, P.scene, Q, drawing, threads)
    return clearance(body_q(tables, Q, "numpy"), P, drawing, backend="numpy")


def clearance_detail_q(tables: ArmTables, Q, obstacles, drawing: bool = False,
                       backend: str | None = None, threads: int = 1) -> ClearanceDetail:
    Q = np.ascontiguousarray(np.asarray(Q, float).reshape(-1, len(tables.dh)))
    P = pack(obstacles)
    if _cn.native_on(backend):
        val, arg = _cn._native.capsule_values_q(tables.chain, tables.caps, P.scene, Q, drawing, True,
                                            threads)
        return _detail(val, arg, tables.names, P)
    return clearance_detail(body_q(tables, Q, "numpy"), P, drawing, backend="numpy")


def self_clearance_q(tables: ArmTables, Q, pairs, margin: float, backend: str | None = None,
                     threads: int = 1) -> np.ndarray:
    Q = np.ascontiguousarray(np.asarray(Q, float).reshape(-1, len(tables.dh)))
    pairs = np.ascontiguousarray(np.asarray(pairs, np.int64).reshape(-1, 2))
    if _cn.native_on(backend) and len(pairs):
        return _cn._native.self_clearance_q(tables.chain, tables.radius, Q, pairs, float(margin),
                                        threads)
    return self_clearance(body_q(tables, Q, "numpy"), pairs, margin, backend="numpy")


def path_clearance_q(tables: ArmTables, reach, q, obstacles, drawing: bool = False,
                     tol: float = 5e-4, max_depth: int = 30, max_evals: int = 200_000,
                     backend: str | None = None, threads: int = 1) -> float:
    """`path_clearance` with the body computed from the tables; the same method and answer."""
    q = np.ascontiguousarray(np.asarray(q, float).reshape(-1, len(tables.dh)))
    P = pack(obstacles)
    K = len(tables.radius)
    R = np.ascontiguousarray(np.broadcast_to(np.asarray(reach, float).reshape(len(tables.dh), -1),
                                             (len(tables.dh), K)))
    if _cn.native_on(backend):
        return _cn._native.path_clearance_q(tables.chain, tables.caps, P.scene, R, q, drawing, tol,
                                        max_depth, max_evals, threads)
    return path_clearance(lambda Q: body_q(tables, Q, "numpy"), q, P, R, drawing, tol,
                          max_depth, max_evals, backend="numpy")
