"""Reading a drawing, and fitting it to the drawing area.

The drawing file (JSON):

    {"units": "mm", "frame": "table",
     "lines": [{"id": "a", "points": [[x, y], ...], "intensity": 1.0}, ...]}

`units` is "mm" or "m"; `frame` must be "table" (origin at the table centre, on the paper);
`intensity` (0 to 1, how hard to press) is optional, default 1.  One pen.  The same content
is accepted as a `.npz` file (`lines` an object array of such dicts, or of (N, 2) arrays with
the ids in `ids`); a `.npz` is read only from a local file, never from the network, because
object arrays are pickles.

`fit` scales a drawing that does not lie inside the drawing area uniformly about the area's
centre (`Rig.drawing_area_centre_m`) until it does.  A drawing that fits is not touched; one that would have to shrink below
half its size is refused.
"""
from __future__ import annotations

import io
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from aris.types import Line, Refusal

UNITS = {"mm": 1e-3, "m": 1.0}
MIN_SCALE = 0.5


@dataclass(frozen=True)
class Fit:
    """What `fit` did.  Boxes are (x_lo, y_lo, x_hi, y_hi) in metres, table frame."""
    scale: float
    bbox_in: tuple
    bbox_out: tuple
    area: tuple                      # (width along x, width along y)
    centre: tuple = (0.0, 0.0)       # the drawing area's centre, table frame


def load(path) -> list[Line] | Refusal:
    """A drawing file (.json or .npz) -> table-frame Lines in metres, z = 0."""
    p = Path(path)
    try:
        data = p.read_bytes()
    except OSError as e:
        return Refusal("unreadable", f"{p}: {e}")
    return from_npz(data) if p.suffix.lower() == ".npz" else parse(data)


def parse(data: bytes) -> list[Line] | Refusal:
    """The JSON drawing format, as bytes."""
    try:
        d = json.loads(data)
    except (ValueError, UnicodeDecodeError) as e:
        return Refusal("not_json", str(e))
    return from_dict(d)


def from_npz(data: bytes) -> list[Line] | Refusal:
    try:
        with np.load(io.BytesIO(data), allow_pickle=True) as z:
            d = {k: z[k] for k in z.files}
    except (OSError, ValueError) as e:
        return Refusal("not_npz", str(e))
    if "lines" not in d:
        return Refusal("no_lines", "the .npz file has no 'lines'")
    ids = [str(x) for x in d["ids"]] if "ids" in d else None
    lines = []
    for k, x in enumerate(d["lines"]):
        if isinstance(x, dict):
            lines.append(dict(x, points=np.asarray(x.get("points")).tolist()))
        else:
            lines.append(dict(id=ids[k] if ids else str(k), points=np.asarray(x).tolist()))
    return from_dict(dict(units=str(d.get("units", "mm")), frame=str(d.get("frame", "table")),
                          lines=lines))


def from_dict(d) -> list[Line] | Refusal:
    if not isinstance(d, dict) or not isinstance(d.get("lines"), list):
        return Refusal("no_lines", "a drawing is an object with a list 'lines'")
    if d.get("units") not in UNITS:
        return Refusal("units", f"units must be one of {sorted(UNITS)}, not {d.get('units')!r}")
    if d.get("frame", "table") != "table":
        return Refusal("frame", f"only the table frame is accepted, not {d.get('frame')!r}")
    if not d["lines"]:
        return Refusal("no_lines", "the drawing has no lines")
    k_m, out, seen = UNITS[d["units"]], [], set()
    for k, ln in enumerate(d["lines"]):
        if not isinstance(ln, dict):
            return Refusal("bad_line", f"line {k} is not an object")
        lid = str(ln.get("id", k))
        try:
            pts = np.asarray(ln.get("points"), float)
            intensity = float(ln.get("intensity", 1.0))
        except (TypeError, ValueError) as e:
            return Refusal("bad_line", f"line {lid}: {e}")
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2 or not np.all(np.isfinite(pts)):
            return Refusal("bad_line", f"line {lid}: points must be at least two finite [x, y]")
        if not 0.0 <= intensity <= 1.0:
            return Refusal("bad_line", f"line {lid}: intensity {intensity} is not in 0..1")
        if lid in seen:
            return Refusal("bad_line", f"two lines have the id {lid!r}")
        seen.add(lid)
        out.append(Line(lid, np.column_stack([pts * k_m, np.zeros(len(pts))]), "table",
                        intensity))
    return out


def to_dict(lines) -> dict:
    """Lines -> the JSON drawing format (millimetres)."""
    return dict(units="mm", frame="table",
                lines=[dict(id=x.id, points=(np.asarray(x.points)[:, :2] * 1e3).tolist(),
                            intensity=float(x.intensity)) for x in lines])


def bbox(lines) -> tuple:
    p = np.concatenate([np.asarray(x.points, float)[:, :2] for x in lines])
    lo, hi = p.min(axis=0), p.max(axis=0)
    return (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))


def fit(lines, area, centre=(0.0, 0.0)) -> tuple[list[Line], Fit] | Refusal:
    """Scale about the drawing area's centre (`centre`, table frame) so every point lies inside
    the area (full widths along x and y around that centre).  A drawing inside is returned as
    it is (scale 1)."""
    half, c = 0.5 * np.asarray(area, float), np.asarray(centre, float).reshape(2)
    p = np.concatenate([np.asarray(x.points, float)[:, :2] for x in lines]) - c
    ext = np.abs(p).max(axis=0)
    ratio = [h / e for h, e in zip(half, ext) if e > 0.0]
    scale = min([1.0] + ratio)
    box = bbox(lines)
    area_t, centre_t = tuple(float(a) for a in area), tuple(float(x) for x in c)
    if scale >= 1.0:
        return list(lines), Fit(1.0, box, box, area_t, centre_t)
    if scale < MIN_SCALE:
        return Refusal("too_large", f"the drawing reaches {ext[0]:.3f} x {ext[1]:.3f} m from "
                       f"the centre of the drawing area ({c[0]:.3f}, {c[1]:.3f}); to fit the "
                       f"area {area[0]:.2f} x {area[1]:.2f} m it would shrink to {scale:.2f} "
                       f"of its size (the least allowed is {MIN_SCALE})")
    out = []
    for x in lines:
        q = np.asarray(x.points, float).copy()
        q[:, :2] = c + scale * (q[:, :2] - c)
        out.append(Line(x.id, q, "table", x.intensity))
    return out, Fit(float(scale), box, bbox(out), area_t, centre_t)


def stretch(line: Line, s0: float, s1: float, lid: str) -> Line:
    """The part of a polyline from arc length s0 to s1, as a new line `lid`."""
    p = np.asarray(line.points, float)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    at = lambda u: np.array([np.interp(u, s, p[:, k]) for k in range(p.shape[1])])
    inner = p[(s > s0) & (s < s1)]
    return Line(lid, np.vstack([at(s0), inner, at(s1)]), line.frame, line.intensity)
