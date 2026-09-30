"""Distances for the checker, written from a different derivation than the planner's kernel.

All functions broadcast over leading axes; points are (..., 3).

- segment to segment: the clamped parametrisation (clamp one parameter, solve the other,
  re-clamp), as in Ericson, Real-Time Collision Detection, 5.1.9.  The kernel instead takes the
  smallest of five candidates.
- segment to axis-aligned box: zero if the segment enters the box (slab clipping); otherwise
  the smallest of the two end points against the box and the segment against the box's twelve
  edges.  Why that is exact: if the closest segment point is not an end point, the direction
  to the box is square to the segment; if its closest box point is inside a face, the segment
  runs parallel to that face at a constant height, and following it one reaches either an end
  point (still above the face) or the face's rim (an edge) at that same height.  The kernel
  instead walks the piecewise-quadratic squared distance along the segment.
"""
from __future__ import annotations

import numpy as np

_EPS = 1e-18


def _dot(a, b):
    return np.einsum("...i,...i->...", a, b)


def segment_segment(a0, a1, b0, b1) -> np.ndarray:
    """Distance between segments [a0,a1] and [b0,b1]."""
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    c, b = _dot(d1, r), _dot(d1, d2)
    a_ok, e_ok = a > _EPS, e > _EPS
    a_s, e_s = np.where(a_ok, a, 1.0), np.where(e_ok, e, 1.0)
    denom = a * e - b * b
    s = np.where(denom > _EPS * np.maximum(a * e, _EPS),
                 np.clip((b * f - c * e) / np.where(denom > 0, denom, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / e_s
    # t outside [0,1]: clamp it and recompute s for the clamped t.
    s = np.where(t < 0.0, np.clip(-c / a_s, 0.0, 1.0),
                 np.where(t > 1.0, np.clip((b - c) / a_s, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    # degenerate segments (points)
    s = np.where(a_ok, s, 0.0)
    t = np.where(e_ok, np.where(a_ok, t, np.clip(f / e_s, 0.0, 1.0)), 0.0)
    s = np.where(a_ok & ~e_ok, np.clip(-c / a_s, 0.0, 1.0), s)
    diff = (a0 + s[..., None] * d1) - (b0 + t[..., None] * d2)
    return np.sqrt(_dot(diff, diff))


def point_box(p, lo, hi) -> np.ndarray:
    """Distance from points to axis-aligned boxes (0 inside)."""
    g = np.maximum(np.maximum(lo - p, p - hi), 0.0)
    return np.sqrt(_dot(g, g))


def segment_enters_box(a, b, lo, hi) -> np.ndarray:
    """Does the segment [a,b] touch the closed box [lo,hi]?  Slab clipping."""
    d = b - a
    moving = np.abs(d) > 1e-15
    safe = np.where(moving, d, 1.0)
    t1, t2 = (lo - a) / safe, (hi - a) / safe
    t_in = np.where(moving, np.minimum(t1, t2), -np.inf)
    t_out = np.where(moving, np.maximum(t1, t2), np.inf)
    still_inside = np.all(moving | ((a >= lo) & (a <= hi)), axis=-1)
    enter = np.maximum(np.max(t_in, axis=-1), 0.0)
    leave = np.minimum(np.min(t_out, axis=-1), 1.0)
    return still_inside & (enter <= leave)


_CORNER = np.array([[i >> 2 & 1, i >> 1 & 1, i & 1] for i in range(8)], float)
_EDGES = np.array([(i, i | 1 << k) for i in range(8) for k in range(3) if not i >> k & 1])


def segment_box(a, b, lo, hi) -> np.ndarray:
    """Distance from segments [a,b] to axis-aligned boxes [lo,hi] (0 when they touch)."""
    a, b, lo, hi = np.broadcast_arrays(a, b, lo, hi)
    corners = lo[..., None, :] + _CORNER * (hi - lo)[..., None, :]          # (...,8,3)
    e0, e1 = corners[..., _EDGES[:, 0], :], corners[..., _EDGES[:, 1], :]    # (...,12,3)
    d_edges = segment_segment(a[..., None, :], b[..., None, :], e0, e1).min(axis=-1)
    d = np.minimum(np.minimum(point_box(a, lo, hi), point_box(b, lo, hi)), d_edges)
    return np.where(segment_enters_box(a, b, lo, hi), 0.0, d)


def segment_plane_height(a, b, normal, offset) -> np.ndarray:
    """Lowest signed height of the segment above the plane normal . p = offset."""
    return np.minimum(_dot(a, normal), _dot(b, normal)) - offset
