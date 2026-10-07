"""SVG drawings: every shape's outline (paths, lines, polylines, polygons, rectangles, circles
and ellipses; cubic and quadratic Béziers and arcs flattened so no chord strays more than
FLAT_M from the curve, every transform applied) as lines of the drawing format, in the table
frame, millimetres.

Placement: the picture's bounding box scaled to `width_m` along the table's x, centred on `at`
(default: the drawing area's centre).  The SVG's x runs along the table's +x, its y (which
points down the page) along the table's −y, so the picture reads as on screen when seen with
the table's +y at the top.  Text is not drawn (convert it to paths first).  Parsing is by
`svgelements` (PyPI, pure Python).
"""
from __future__ import annotations

import numpy as np

from aris.types import Refusal

FLAT_M = 0.5e-3          # m: the most a flattened curve's chord strays from the curve
MAX_PIECES = 4096        # per curve segment


def _flatten(seg, tol_units: float) -> np.ndarray:
    """A curve segment's points from start to end (start left out), chords within tol."""
    n = 2
    while True:
        t = np.linspace(0.0, 1.0, n + 1)
        pts = np.array([[seg.point(x).x, seg.point(x).y] for x in t])
        mid = np.array([[seg.point(x).x, seg.point(x).y] for x in 0.5 * (t[:-1] + t[1:])])
        sag = np.linalg.norm(mid - 0.5 * (pts[:-1] + pts[1:]), axis=1).max()
        if sag <= tol_units or n >= MAX_PIECES:
            return pts[1:]
        n *= 2


def _polylines(shapes, tol_units: float) -> list[np.ndarray]:
    import svgelements as se
    out = []
    for shape in shapes:
        cur: list = []
        for seg in se.Path(shape).segments():
            if isinstance(seg, se.Move):
                if len(cur) >= 2:
                    out.append(np.array(cur))
                cur = [[seg.end.x, seg.end.y]]
            elif isinstance(seg, (se.Line, se.Close)):
                if seg.end is not None and cur:
                    cur.append([seg.end.x, seg.end.y])
            elif cur:
                cur.extend(_flatten(seg, tol_units).tolist())
        if len(cur) >= 2:
            out.append(np.array(cur))
    return [p for p in out if np.ptp(p, axis=0).max() > 0.0]


def to_drawing(path, width_m: float, at=(0.0, 0.0), flat_m: float = FLAT_M) -> dict | Refusal:
    """The SVG file at `path` as a drawing (the JSON format's dict): its picture `width_m`
    wide along the table's x, centred on `at` (table frame, metres)."""
    import svgelements as se
    if not width_m > 0.0:
        return Refusal("width", f"--width {width_m} m is not a width")
    try:
        svg = se.SVG.parse(str(path), reify=True)
    except Exception as e:                    # a file svgelements cannot read
        return Refusal("unreadable", f"{path}: {e}")
    shapes = [e for e in svg.elements() if isinstance(e, se.Shape)]
    if not shapes:
        return Refusal("no_lines", f"{path} has no shapes (text must be converted to paths)")
    boxes = [b for b in (se.Path(s).bbox() for s in shapes) if b is not None]
    if not boxes:
        return Refusal("no_lines", f"{path} has no shape with extent")
    b = np.asarray(boxes, float)                # the exact extent of the curves, for the scale
    lo, hi = b[:, :2].min(axis=0), b[:, 2:].max(axis=0)
    if not hi[0] > lo[0]:
        return Refusal("width", f"{path} has no width to scale (a vertical line only)")
    k = width_m / (hi[0] - lo[0])             # metres per SVG unit
    lines = _polylines(shapes, flat_m / k)
    mid = 0.5 * (lo + hi)
    at = np.asarray(at, float).reshape(2)
    out = []
    for i, p in enumerate(lines):
        xy = np.column_stack([(p[:, 0] - mid[0]) * k, -(p[:, 1] - mid[1]) * k]) + at
        out.append(dict(id=f"svg{i + 1}", points=np.round(xy * 1e3, 4).tolist()))
    return dict(units="mm", frame="table", lines=out,
                source=dict(svg=str(path), width_m=float(width_m),
                            at_m=[float(x) for x in at], flat_m=float(flat_m)))
