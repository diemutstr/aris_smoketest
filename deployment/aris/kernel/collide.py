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
from aris.kernel.collide_groups import body_groups
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


def _plane_block(p0, p1, r, is_pen, is_tool, P, drawing):
    """(n, K, Mp) clearances against the planes; +inf where a pair is not checked."""
    d = geo.segment_plane_distance(p0[:, :, None], p1[:, :, None], P.pl_n, P.pl_off)
    margin = np.where(is_pen[:, None], P.pl_pen_m, np.where(is_tool[:, None], P.tool_m, P.pl_m))
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


def _group_bounds(p0, p1, r, gid, P, G):
    """(n, NG, G) lower bound, per body group and obstacle group, on every pair between them:
    a sphere around the body group's capsules against the obstacle group's fat capsule."""
    start, members, a, b, R, marg = G
    NG = int(gid.max(initial=-1)) + 1
    out = np.full((p0.shape[0], NG, len(R)), np.inf)
    for g in range(NG):
        k = np.flatnonzero(gid == g)
        if not len(k):
            continue
        ends = np.concatenate([p0[:, k], p1[:, k]], axis=1)             # (n, 2m, 3)
        c = 0.5 * (ends.min(axis=1) + ends.max(axis=1))                  # (n, 3)
        rad = (np.linalg.norm(ends - c[:, None], axis=2) + np.r_[r[k], r[k]]).max(axis=1)
        d = geo.point_segment_distance(c[:, None], a, b)                # (n, G)
        out[:, g] = d - R - rad[:, None] - marg
    return out


def _chunk_values(p0, p1, r, is_pen, is_tool, P, drawing, mode, gid, exempt):
    """(n, K, M) clearance of every pair, obstacles in `P.names` order; +inf for a pair the
    work-saving skipped.  A pair is skipped only when a lower bound (its midpoint bound, or
    its two groups' bound) is above (best upper bound + span): its true value is then above
    (configuration minimum + span), so `_finish` gives the same answer as with no skipping.
    `exempt` (K, Mb) bool: pairs never checked (`Box.exempt`); they read +inf and do not
    count towards the best upper bound.
    """
    n = p0.shape[0]
    prune, groups, span = mode
    planes = _plane_block(p0, p1, r, is_pen, is_tool, P, drawing)
    if not prune:
        boxes = np.where(exempt, np.inf, _box_block(p0, p1, r, P))
        return np.concatenate([boxes, planes, _cap_block(p0, p1, r, P)], axis=2)
    ub_b, ub_c, half = _midpoint_bounds(p0, p1, r, P)
    ub_b = np.where(exempt, np.inf, ub_b)
    best = np.min([x.reshape(n, -1).min(axis=1, initial=np.inf) for x in (ub_b, ub_c, planes)],
                  axis=0)[:, None, None]
    thr = best + span
    skip_b, skip_c = (ub_b - half[:, :, None] > thr) | exempt, ub_c - half[:, :, None] > thr
    if groups:
        G = P.groups
        Mb, Mp = len(P.box_m), len(P.pl_m)
        grp = np.zeros(Mb + Mp + len(P.cap_rm), np.int64)
        grp[G[1]] = np.repeat(np.arange(len(G[0]) - 1), np.diff(G[0]))
        gl = _group_bounds(p0, p1, r, gid, P, G)[:, gid]                 # (n, K, G)
        skip_b |= gl[:, :, grp[:Mb]] > thr
        skip_c |= gl[:, :, grp[Mb + Mp:]] > thr
    ob = np.full(ub_b.shape, np.inf)
    oc = np.full(ub_c.shape, np.inf)
    ob[~skip_b] = _box_block(p0, p1, r, P, ~skip_b)
    oc[~skip_c] = _cap_block(p0, p1, r, P, ~skip_c)
    return np.concatenate([ob, planes, oc], axis=2)


def _finish(val, arg, live, span):
    """Clip each capsule at (configuration minimum + span); clipped capsules name no obstacle.
    This is what makes the answer independent of which pairs were skipped."""
    m = val[:, live].min(axis=1, initial=np.inf)[:, None]
    over = (val > m + span) & np.isfinite(m)
    over[:, ~live] = False
    val[over] = np.broadcast_to(m + span, val.shape)[over]
    arg[over] = -1


def _caps(body: Body):
    K = np.shape(body.p0)[1]
    fixed = np.zeros(K, bool) if body.is_fixed is None else np.asarray(body.is_fixed, bool)
    tool = np.zeros(K, bool) if getattr(body, "is_tool", None) is None else \
        np.asarray(body.is_tool, bool)
    return np.asarray(body.radius, float), np.asarray(body.is_pen, bool), fixed, tool


def _gid(names):
    start, members = body_groups(names)
    gid = np.empty(len(members), np.int64)
    gid[members] = np.repeat(np.arange(len(start) - 1), np.diff(start))
    return gid


def _per_capsule(body: Body, obstacles, drawing: bool, mode, backend=None, threads=1):
    """(N, K) value of each capsule and (N, K) its obstacle: min(true clearance, configuration
    minimum + span), obstacle -1 where clipped or none.  Fixed capsules: +inf, -1."""
    P = pack(obstacles)
    r, is_pen, fixed, tool = _caps(body)
    if _cn.native_on(backend):
        caps = (r, is_pen, fixed, tool) + body_groups(body.names) + \
            (_cn.exempt_mask(body.names, P),)
        val, arg, _ = _cn._native.capsule_values(body.p0, body.p1, caps, P.scene, drawing,
                                                 tuple(mode), threads)
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
    gid = np.unique(_gid(body.names)[live], return_inverse=True)[1]
    exempt = _cn.exempt_mask(body.names, P)[live].astype(bool)
    step = max(1, _CHUNK_PAIRS // (L * M))
    for s in range(0, N, step):
        v = _chunk_values(p0[s:s + step], p1[s:s + step], r[live], is_pen[live],
                          tool[live], P, drawing, mode, gid, exempt)
        a = np.argmin(v, axis=2)
        arg[s:s + step, live] = a
        val[s:s + step, live] = np.take_along_axis(v, a[:, :, None], axis=2)[:, :, 0]
    arg[~np.isfinite(val)] = -1
    _finish(val, arg, ~fixed, mode[2])
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
                      backend: str | None = None, threads: int = 1, span: float = np.inf,
                      groups: bool = True) -> np.ndarray:
    """(N, K) per capsule: min(its true clearance, configuration minimum + span).  With the
    default span every capsule is exact; a small span is faster and still exact for the
    closest capsule (the others are then lower bounds)."""
    return _per_capsule(body, obstacles, drawing, (prune, groups, span), backend, threads)[0]


def clearance(body: Body, obstacles, drawing: bool = False, prune: bool = True,
              backend: str | None = None, threads: int = 1, groups: bool = True) -> np.ndarray:
    """(N,) smallest clearance over all capsule-obstacle pairs.  At least 0 means free."""
    return capsule_clearance(body, obstacles, drawing, prune, backend, threads, 0.0,
                             groups).min(axis=1, initial=np.inf)


def _detail(val, arg, names, P) -> ClearanceDetail:
    N, K = val.shape
    # a capsule clipped at the configuration's minimum names no obstacle (arg -1) and may tie
    # with the closest one; the closest capsule is the first unclipped one with the minimum
    k = np.argmin(np.where(arg >= 0, val, np.inf), axis=1) if K else np.zeros(N, np.int64)
    v = val[np.arange(N), k] if K else np.full(N, np.inf)
    m = arg[np.arange(N), k] if K else np.full(N, -1)
    k = np.where(np.isfinite(v), k, -1)
    return ClearanceDetail(v, k, m, tuple(names), P.names)


def clearance_detail(body: Body, obstacles, drawing: bool = False, prune: bool = True,
                     backend: str | None = None, threads: int = 1,
                     groups: bool = True) -> ClearanceDetail:
    val, arg, P = _per_capsule(body, obstacles, drawing, (prune, groups, 0.0), backend, threads)
    return _detail(val, arg, body.names, P)


def self_clearance(body: Body, pairs: np.ndarray, margin: float, backend: str | None = None,
                   threads: int = 1, prune: bool = True, groups: bool = True) -> np.ndarray:
    """(N,) smallest clearance between the capsule pairs (i, j) of the same arm."""
    pairs = np.asarray(pairs, np.int64).reshape(-1, 2)
    p0, p1 = np.asarray(body.p0, float), np.asarray(body.p1, float)
    r = np.asarray(body.radius, float)
    N = p0.shape[0]
    if len(pairs) == 0:
        return np.full(N, np.inf)
    if _cn.native_on(backend):
        r_, pen, fixed, tool = _caps(body)
        caps = (r_, pen, fixed, tool) + body_groups(body.names)
        return _cn._native.self_values(p0, p1, caps, pairs, float(margin), (prune, groups, 0.0),
                                       False, threads)[0]
    i, j = pairs[:, 0], pairs[:, 1]
    out = np.empty(N)
    step = max(1, _CHUNK_PAIRS // len(pairs))
    for s in range(0, N, step):
        a0, a1, b0, b1 = p0[s:s + step, i], p1[s:s + step, i], p0[s:s + step, j], p1[s:s + step, j]
        d = geo.segment_segment_distance(a0, a1, b0, b1)
        out[s:s + step] = (d - r[i] - r[j] - margin).min(axis=1)
    return out


# The calls along a path and from joint angles live in collide_path.py; every public name is
# importable from here.  Imported last because collide_path uses the calls above.
from aris.kernel.collide_path import (CAP, SPAN, clearance_detail_q, clearance_q,  # noqa: E402,F401
                                      edges_clearance_q, exact_count_q, path_clearance, path_clearance_q,
                                      path_self_clearance, path_self_clearance_q,
                                      self_clearance_q)
