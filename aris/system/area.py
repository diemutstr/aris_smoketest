"""The admissible drawing area: the largest rectangle with the given centre (the rig's
`drawing_area_centre_m`, by default the table centre) and sides along the table that lies
entirely inside what some arm can draw in some phase, kept `margin` (2 cm) away from the edge
of that.  A drawing must lie inside it; the drawing server scales drawings about its centre to
fit.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import binary_erosion

from aris.system.maps import DRAWABLE

MARGIN = 0.02            # m kept from the edge of what can be drawn


def admissible(maps: dict, margin: float = MARGIN, centre=(0.0, 0.0)) -> np.ndarray:
    """(2,) full width along x and y of the largest rectangle centred on `centre` (table
    frame) whose grid points all lie in the union of every map shrunk by `margin` (off the
    grid counts as not drawable)."""
    if not maps:                                # no mounted arm: nothing can be drawn
        return np.zeros(2)
    m0 = next(iter(maps.values()))
    union = np.zeros_like(m0.state, bool)
    for m in maps.values():
        union |= m.state == DRAWABLE
    step = float(m0.x[1] - m0.x[0])
    r = int(np.ceil(margin / step - 1e-9))
    o = np.arange(-r, r + 1)
    disc = np.hypot(*np.meshgrid(o, o, indexing="ij")) * step <= margin + 1e-9
    inner = binary_erosion(union, disc, border_value=0)
    dx, dy = np.abs(m0.x - centre[0]), np.abs(m0.y - centre[1])
    best, size = -1.0, np.zeros(2)
    for hx in np.unique(dx):                    # half widths at which a column joins
        rows_ok = inner[dx <= hx + 1e-9].all(axis=0)
        bad = dy[~rows_ok]
        limit = bad.min() if len(bad) else np.inf
        good = dy[dy < limit - 1e-9]
        if not len(good):
            break
        hy = float(good.max())
        if hx * hy > best:
            best, size = hx * hy, 2.0 * np.array([hx, hy])
    return size


def first_outside(lines, size: np.ndarray, centre=(0.0, 0.0), tol: float = 1e-9):
    """(line id, (x, y)) of the first point outside the area of `size` centred on `centre`,
    or None."""
    half, c = 0.5 * np.asarray(size, float), np.asarray(centre, float)
    for line in lines:
        p = np.asarray(line.points, float).reshape(-1, 3)[:, :2]
        bad = np.flatnonzero(np.any(np.abs(p - c) > half + tol, axis=1))
        if len(bad):
            return line.id, tuple(p[bad[0]])
    return None
