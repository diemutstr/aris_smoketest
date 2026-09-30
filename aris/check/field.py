"""The checker's own reading of a distance field (`types.Field`: a neighbour's footprint).

A field stores, at each grid centre, a lower bound on the distance from that centre to the
footprint (negative inside).  Read conservatively:
  - at a point inside the grid's box: the value at the nearest centre minus the distance to it
    (the footprint is at least that far, by the triangle inequality);
  - at a point outside the box: move it onto the box, read there, and add the distance to the
    box back in quadrature.  The footprint lies inside the box and the box is convex, so any
    footprint point f satisfies |p - f|^2 >= |p - p_box|^2 + |p_box - f|^2.
A capsule is read at points along its axis no more than half a cell apart; any point of the
axis is then within a quarter cell of one of them, which is subtracted, with the radius.
The planners' kernel reads the 8 surrounding centres and samples a cell apart; this module
takes the nearest centre and samples half a cell apart.  Both are lower bounds.
"""
from __future__ import annotations

import numpy as np


def point_bound(origin, cell, dist, p) -> np.ndarray:
    """Lower bound on the distance from points p (M,3) to the footprint."""
    n = np.array(dist.shape)
    hi = origin + (n - 1) * cell
    pc = np.clip(p, origin, hi)
    e = np.linalg.norm(p - pc, axis=-1)
    idx = np.clip(np.rint((pc - origin) / cell).astype(int), 0, n - 1)
    centre = origin + idx * cell
    L = dist[idx[:, 0], idx[:, 1], idx[:, 2]].astype(float) - np.linalg.norm(pc - centre, axis=-1)
    return np.where(e > 0, np.sqrt(e * e + np.maximum(L, 0.0) ** 2), L)


def capsule_bound(origin, cell, dist, a, b, r) -> np.ndarray:
    """Lower bound on the gap between capsules (a, b, r), each (M,3)/(M,), and the footprint."""
    length = np.linalg.norm(b - a, axis=-1)
    n = np.maximum(np.ceil(length / (0.5 * cell)).astype(int), 1)          # pieces per capsule
    step = length / n                                                         # <= half a cell
    m = int(n.max()) + 1
    u = np.minimum(np.arange(m)[None, :] / n[:, None], 1.0)                  # (M,m) repeats end
    pts = a[:, None] + u[..., None] * (b - a)[:, None]
    v = point_bound(origin, cell, dist, pts.reshape(-1, 3)).reshape(len(a), m)
    return v.min(axis=1) - 0.5 * step - r
