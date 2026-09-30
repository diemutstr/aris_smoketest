"""The admissible drawing area: the largest rectangle centred on the table, sides along the
table, that lies entirely inside what some arm can draw in some phase, kept `margin` (2 cm)
away from the edge of that.  A drawing must lie inside it; the drawing server scales drawings
to fit.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import binary_erosion

from aris.system.maps import DRAWABLE

MARGIN = 0.02            # m kept from the edge of what can be drawn


def admissible(maps: dict, margin: float = MARGIN) -> np.ndarray:
    """(2,) full width along x and y, from the union of every map (grid points; off the grid
    counts as not drawable)."""
    m0 = next(iter(maps.values()))
    union = np.zeros_like(m0.state, bool)
    for m in maps.values():
        union |= m.state == DRAWABLE
    step = float(m0.x[1] - m0.x[0])
    r = int(np.ceil(margin / step - 1e-9))
    o = np.arange(-r, r + 1)
    disc = np.hypot(*np.meshgrid(o, o, indexing="ij")) * step <= margin + 1e-9
    inner = binary_erosion(union, disc, border_value=0)
    cx, cy = int(np.argmin(np.abs(m0.x))), int(np.argmin(np.abs(m0.y)))
    best, size = -1.0, np.zeros(2)
    for ix in range(min(cx, len(m0.x) - 1 - cx) + 1):
        band = inner[cx - ix:cx + ix + 1].all(axis=0)
        if not band[cy]:
            break
        iy = 0
        while cy - iy - 1 >= 0 and cy + iy + 1 < len(band) and band[cy - iy - 1] \
                and band[cy + iy + 1]:
            iy += 1
        if ix * iy > best:
            best, size = ix * iy, 2 * step * np.array([ix, iy])
    return size


def first_outside(lines, size: np.ndarray, tol: float = 1e-9):
    """(line id, (x, y)) of the first point outside the area centred on the table, or None."""
    half = 0.5 * np.asarray(size, float)
    for line in lines:
        p = np.asarray(line.points, float).reshape(-1, 3)[:, :2]
        bad = np.flatnonzero(np.any(np.abs(p) > half + tol, axis=1))
        if len(bad):
            return line.id, tuple(p[bad[0]])
    return None
