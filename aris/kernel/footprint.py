"""A neighbour's footprint over a phase: everywhere its body goes, as a distance field.

`footprint(arm, trajectories)` samples the motions so that no point of the body moves more than
half a cell between kept poses, lays sample points along every capsule's axis, and turns them
into a grid `dist` (types.Field) whose every value is a LOWER bound on the distance from that
cell's centre to the footprint (every capsule at every instant, grown by `pad`), negative inside.

How the bound is kept (h = half a cell diagonal, sigma <= cell/2 the axis sample spacing, ell
<= cell/2 the most any point moves between kept poses): capsules are sorted into radius
classes, the radius rounded up to a quarter cell.  For each class, the Euclidean distance
transform of the cells holding an axis sample gives e, the distance to the nearest such cell
centre.  Every axis sample is within h of its cell centre, every axis point within sigma/2 of
a sample, and every swept point within ell/2 of a kept pose, so
    dist = min over classes of (e - h - sigma/2 - ell/2 - r_class - pad)
is at most the distance to the nearest capsule.  Outside the footprint that IS the distance to
the footprint, so there dist is a lower bound, at most 2 h + sigma/2 + ell/2 + a quarter cell
(about 2.2 cells) below it.  Inside, dist is negative, no deeper than the depth into the
deepest single capsule; where capsules overlap, the depth into the union can be larger.  The
sign is right everywhere, which is what a clearance check needs.  Reading the field adds up to
h + cell/2 more (collide.py): a clearance against a field is at most 3 h + cell (72 mm at a
2 cm cell) below the exact one, typically a third of that.

`transform_field(field, T_ab)` re-expresses a field given in frame b in frame a on a new grid
of the same cell, each new value read through the same conservative lookup (up to h more).
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import distance_transform_edt

from aris.kernel import collide
from aris.kernel.geometry import field_lookup
from aris.kernel.retime import sample
from aris.types import Field

FINE_DT = 0.002          # s, the rate the motions are first read at
_ROUND = 1e-6            # m, taken off before storing as float32 so rounding cannot raise it


def _poses(trajectories, reach, cell):
    """(N, 7) poses, no capsule point moving more than cell/2 between neighbours (unless one
    step of FINE_DT already does), and the largest such move (ell)."""
    keep, ell = [], 0.0
    for tr in trajectories:
        t = np.append(np.arange(tr.t[0], tr.t[-1], FINE_DT), tr.t[-1])
        q, qd, _ = sample(tr, t)
        # per fine step, per joint: at least the chord, and the speed at either end times dt
        dq = np.maximum(np.abs(np.diff(q, axis=0)),
                        np.diff(t)[:, None] * np.maximum(np.abs(qd[:-1]), np.abs(qd[1:])))
        step = (dq @ reach).max(axis=1)                              # metres per fine step
        idx, acc = [0], 0.0
        for m, s in enumerate(step):
            if acc + s > 0.5 * cell and acc > 0.0:
                idx.append(m)
                ell, acc = max(ell, acc), 0.0
            acc += s
        idx.append(len(q) - 1)
        ell = max(ell, acc)
        keep.append(q[np.unique(idx)])
    return np.concatenate(keep), ell


def footprint(arm, trajectories, cell: float = 0.02, pad: float = 0.0, name: str = "footprint",
              margin: float = 0.0) -> Field:
    """The footprint of `arm` running `trajectories` (types.Trajectory), in its own base frame.

    Every capsule of the body counts, the fixed ones too: a neighbour sees the whole arm."""
    tables = collide.arm_tables(arm)
    Q, ell = _poses(trajectories, np.asarray(arm.reach, float), cell)
    body = collide.body_q(tables, Q)
    r = np.asarray(tables.radius, float)
    L = np.linalg.norm(tables.cap_b - tables.cap_a, axis=1)
    n = np.where(L > 0, np.ceil(L / (0.5 * cell)) + 1, 1).astype(int)
    sigma = np.where(n > 1, L / np.maximum(n - 1, 1), 0.0)
    rq = np.ceil(r / (0.25 * cell) - 1e-9) * (0.25 * cell)       # radius classes
    h = 0.5 * np.sqrt(3.0) * cell
    ends = np.concatenate([body.p0.reshape(-1, 3), body.p1.reshape(-1, 3)])
    grow = rq.max() + pad + 0.5 * ell + 2 * cell
    lo = ends.min(axis=0) - grow
    dims = np.ceil((ends.max(axis=0) + grow - lo) / cell).astype(int) + 1
    dist = np.full(tuple(dims), np.inf)
    for rc in np.unique(rq):
        skel = np.zeros(tuple(dims), bool)
        for k in np.flatnonzero(rq == rc):                   # axis samples, capsule by capsule
            u = (np.arange(n[k]) / max(n[k] - 1, 1))[None, :, None]
            pts = body.p0[:, k, None] + u * (body.p1[:, k] - body.p0[:, k])[:, None]
            idx = np.rint((pts.reshape(-1, 3) - lo) / cell).astype(int)
            skel[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        e = distance_transform_edt(~skel, sampling=cell)
        worst = 0.5 * sigma[rq == rc].max()
        np.minimum(dist, e - h - worst - 0.5 * ell - rc - pad, out=dist)
    return Field(name, lo, float(cell), (dist - _ROUND).astype(np.float32), float(margin))


def transform_field(field: Field, T_ab: np.ndarray, name: str | None = None) -> Field:
    """`field`, given in frame b, expressed in frame a (p_a = T_ab p_b), on a new grid of the
    same cell that holds the old one.  Each new value is the old field read conservatively at
    that point (collide's lookup), so it can only be lower than before: by up to half a cell
    diagonal, from reading between the old centres."""
    T = np.asarray(T_ab, float)
    R, t = T[:3, :3], T[:3, 3]
    c, n = float(field.cell), np.array(np.shape(field.dist))
    o = np.asarray(field.origin_base, float)
    corners = o + (np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
                   * (n - 1)) * c
    ca = corners @ R.T + t
    lo, hi = ca.min(axis=0) - c, ca.max(axis=0) + c
    dims = np.ceil((hi - lo) / c).astype(int) + 1
    out = np.empty(tuple(dims), np.float32)
    old = np.asarray(field.dist)
    for i in range(dims[0]):                            # one slab at a time bounds the memory
        g = np.stack(np.meshgrid([i], np.arange(dims[1]), np.arange(dims[2]), indexing="ij"),
                     axis=-1)[0]
        p_a = lo + g * c
        p_b = (p_a - t) @ R
        out[i] = (field_lookup(p_b, o, c, n, old) - _ROUND).astype(np.float32)
    return Field(name or field.name, lo, c, out, field.margin)
