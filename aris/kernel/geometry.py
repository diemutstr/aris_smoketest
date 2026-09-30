"""Exact distances between segments, points, boxes and planes.  Pure functions on arrays.

Every function takes arrays of pairs that broadcast against each other, shape (..., 3) per
point, (..., 3, 3) per rotation, (...) per scalar, and returns (...) distances.  A capsule is a segment with a radius; the radius is subtracted by the
caller, so everything here is about segments.

No iteration, no search: each distance is the minimum over a short, fixed list of candidate
points, and the list is known to contain the true closest point.  Dot products are written out
component by component (no matmul, no einsum) so that the result for one pair does not depend
on how many other pairs are in the same call; the pruning in `collide.py` relies on that.
"""
from __future__ import annotations

import numpy as np


def dot(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a[..., 0] * b[..., 0] + a[..., 1] * b[..., 1] + a[..., 2] * b[..., 2]


def _sum3(a: np.ndarray) -> np.ndarray:
    return a[..., 0] + a[..., 1] + a[..., 2]


def to_local(v: np.ndarray, R: np.ndarray) -> np.ndarray:
    """R^T v for each pair: a base-frame vector into the frame whose axes are R's columns."""
    return np.stack([v[..., 0] * R[..., 0, j] + v[..., 1] * R[..., 1, j] + v[..., 2] * R[..., 2, j]
                     for j in range(3)], axis=-1)


def _point_segment_d2(p, a, d, dd):
    """Squared distance from p to the segment a + t d, t in [0, 1]; dd = d . d."""
    t = np.clip(dot(p - a, d) / np.where(dd > 0.0, dd, 1.0), 0.0, 1.0)
    w = a + t[..., None] * d - p
    return dot(w, w)


def point_segment_distance(p, a, b):
    d = b - a
    return np.sqrt(_point_segment_d2(p, a, d, dot(d, d)))


def segment_segment_distance(p0, p1, q0, q1):
    """Distance between segments [p0, p1] and [q0, q1].

    The squared distance is a convex quadratic over the unit square of the two segment
    parameters.  Its minimum is either the unconstrained stationary point (inside the square)
    or on one of the four edges of the square, and each edge is an endpoint of one segment
    against the whole other segment.  So the five candidates below contain the minimum.  Every
    candidate is a real pair of points, one on each segment, so none can undercut the answer;
    for parallel or degenerate segments the stationary point is meaningless and the edges hold
    the minimum.
    """
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, b = dot(d1, d1), dot(d2, d2), dot(d1, d2)
    c, f = dot(d1, r), dot(d2, r)
    den = a * e - b * b
    ok = den > 0.0
    safe = np.where(ok, den, 1.0)
    s = np.where(ok, np.clip((b * f - c * e) / safe, 0.0, 1.0), 0.0)
    # the other parameter re-solved against this one, then this one again: the pair stays
    # consistent when the stationary point was clipped, and loses less to cancellation
    t = np.clip((b * s + f) / np.where(e > 0.0, e, 1.0), 0.0, 1.0)
    s = np.clip((b * t - c) / np.where(a > 0.0, a, 1.0), 0.0, 1.0)
    w = r + s[..., None] * d1 - t[..., None] * d2
    best = dot(w, w)
    best = np.minimum(best, _point_segment_d2(p0, q0, d2, e))
    best = np.minimum(best, _point_segment_d2(p1, q0, d2, e))
    best = np.minimum(best, _point_segment_d2(q0, p0, d1, a))
    best = np.minimum(best, _point_segment_d2(q1, p0, d1, a))
    return np.sqrt(best)


def segment_plane_distance(p0, p1, normal, offset):
    """Signed distance from the segment to the plane normal . p = offset (negative: through it).

    Linear along the segment, so the minimum is at an endpoint.
    """
    return np.minimum(dot(normal, p0), dot(normal, p1)) - offset


def point_box_distance(p, R, center, half):
    """Distance from p to a solid oriented box (0 inside)."""
    x = np.abs(to_local(p - center, R)) - half
    x = np.maximum(x, 0.0)
    return np.sqrt(dot(x, x))


def _slope(a, d, h, t):
    """Half the slope of the squared segment-box distance at parameter t (any shape (..., P))."""
    g = 0.0
    for i in range(3):
        x = a[i] + t * d[i]
        g = g + d[i] * (np.maximum(x - h[i], 0.0) + np.minimum(x + h[i], 0.0))
    return g


def segment_box_distance(p0, p1, R, center, half):
    """Distance from the segment [p0, p1] to a solid oriented box (0 if they touch).

    In the box frame the box is |x_i| <= h_i and the segment is x(t) = a + t d, t in [0, 1].
    The squared distance  f(t) = sum_i (x_i - clamp(x_i, -h_i, h_i))^2  is convex, and its
    slope  g(t) = 2 sum_i d_i (x_i - clamp(x_i, -h_i, h_i))  is piecewise linear and never
    decreasing; it bends only where a coordinate crosses a face, a_i + t d_i = +-h_i.  So:
    take the six crossings plus t = 0 and t = 1, evaluate g at all eight, and the minimum of f
    sits between the last of them with g <= 0 and the first with g > 0, where g is a straight
    line; its zero there is found by one linear interpolation.  If g > 0 already at t = 0 the
    minimum is at t = 0.  f is then evaluated at that one point.  No search, no sort.  A
    segment through the box has g = 0 on the stretch inside, and f = 0 there.
    """
    # Coordinates and candidates go first, pairs last: every reduction below is then
    # elementwise across 8 rows, which numpy does far faster than along a short last axis.
    a = np.ascontiguousarray(np.moveaxis(to_local(p0 - center, R), -1, 0))    # (3, ...)
    d = np.ascontiguousarray(np.moveaxis(to_local(p1 - p0, R), -1, 0))
    a, d = np.broadcast_arrays(a, d)
    h = np.moveaxis(np.asarray(half, float), -1, 0)
    shape = a.shape[1:]
    t = np.empty((8,) + shape)
    t[0], t[1] = 0.0, 1.0
    for i in range(3):
        moving = d[i] != 0.0
        inv = 1.0 / np.where(moving, d[i], 1.0)
        t[2 + i] = np.where(moving, np.minimum(np.maximum((h[i] - a[i]) * inv, 0.0), 1.0), 0.0)
        t[5 + i] = np.where(moving, np.minimum(np.maximum((-h[i] - a[i]) * inv, 0.0), 1.0), 0.0)
    g = _slope(a, d, h, t)
    below = g <= 0.0
    t_lo = np.where(below, t, -1.0).max(axis=0)
    t_hi = np.where(below, 2.0, t).min(axis=0)
    g_lo, g_hi = _slope(a, d, h, t_lo), _slope(a, d, h, t_hi)
    has_lo, has_hi = t_lo >= 0.0, t_hi <= 1.0
    both = has_lo & has_hi
    span = np.where(both, g_hi - g_lo, 1.0)
    ts = np.where(both, t_lo - np.where(both, g_lo, 0.0) * (t_hi - t_lo) / span,
                  np.where(has_lo, t_lo, 0.0))
    f = np.zeros(shape)
    for i in range(3):
        x = a[i] + ts * d[i]
        e = np.maximum(x - h[i], 0.0) + np.minimum(x + h[i], 0.0)
        f += e * e
    return np.sqrt(f)


def field_lookup(p, origin, cell, dims, dist):
    """A lower bound on the distance from points p (..., 3) to a field's footprint.

    The point is first moved to the nearest point q of the box of cell centres (the footprint
    lies inside that box); a point outside the box gains its distance to the box back, in
    quadrature.  Then, for each of the 8 centres c around q, dist[c] - |q - c| is a lower
    bound (distance changes no faster than the point moves); the largest of the 8 is kept.
    """
    dims = np.asarray(dims, np.int64)
    origin = np.asarray(origin, float)
    hi = origin + (dims - 1) * cell
    q = np.minimum(np.maximum(p, origin), hi)
    i = np.floor((q - origin) / cell).astype(np.int64)
    i = np.minimum(np.maximum(i, 0), np.maximum(dims - 2, 0))
    best = np.full(q.shape[:-1], -np.inf)
    for ax in (0, 1):
        ix = np.minimum(i[..., 0] + ax, dims[0] - 1)
        dx = q[..., 0] - (origin[0] + ix * cell)
        for ay in (0, 1):
            iy = np.minimum(i[..., 1] + ay, dims[1] - 1)
            dy = q[..., 1] - (origin[1] + iy * cell)
            for az in (0, 1):
                iz = np.minimum(i[..., 2] + az, dims[2] - 1)
                dz = q[..., 2] - (origin[2] + iz * cell)
                v = dist[ix, iy, iz].astype(float) - np.sqrt(dx * dx + dy * dy + dz * dz)
                best = np.maximum(best, v)
    # outside the box: every footprint point y is inside it, so |p - y|^2 >= |q - y|^2 + |p - q|^2
    e = p - q
    e2 = e[..., 0] * e[..., 0] + e[..., 1] * e[..., 1] + e[..., 2] * e[..., 2]
    pos = np.maximum(best, 0.0)
    return np.where(e2 > 0.0, np.sqrt(pos * pos + e2), best)


def segment_field_distance(p0, p1, origin, cell, dims, dist):
    """A lower bound on the distance from the segment [p0, p1] (..., 3) to a field's footprint.

    The segment is sampled at n = ceil(L / cell) + 1 evenly spaced points (spacing s <= cell);
    every point of it is within s/2 of a sample, so min over samples of `field_lookup` - s/2.
    """
    u = p1 - p0
    L = np.sqrt(dot(u, u))
    n = np.where(L > 0.0, np.ceil(L / cell) + 1, 1).astype(np.int64)
    j = np.arange(int(n.max(initial=1)))
    t = np.minimum(j, n[..., None] - 1) / np.maximum(n[..., None] - 1, 1)
    pts = p0[..., None, :] + t[..., None] * u[..., None, :]
    v = field_lookup(pts, origin, cell, dims, dist).min(axis=-1)
    s = np.where(n > 1, L / np.maximum(n - 1, 1), 0.0)
    return v - 0.5 * s
