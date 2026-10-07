"""The checker's own reading of the paper height map, `config/calibration/paper.json`.

The file stores a thin-plate spline and says how to evaluate it (its `evaluate` line):
  z(x, y) = paper_z + t(d) * (a0 + ax x + ay y + sum_i w_i phi(|(x, y) - c_i|)),
  phi(r) = r^2 ln r (0 at r = 0), d = distance from the hull polygon (0 inside),
  t = 1 inside the hull, 1 - d / taper_m up to taper_m outside, 0 beyond.
Written here from that line with numpy alone, apart from the code that fits and writes it.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PaperMap:
    paper_z: float
    centres: np.ndarray        # (N,2) table frame
    weights: np.ndarray        # (N,)
    affine: np.ndarray         # (3,) a0, ax, ay
    hull: np.ndarray           # (H,2), counter-clockwise
    taper: float               # m
    low: float                 # lowest measured paper height, table z
    about: str                 # for the verdict's note

    def z(self, x, y) -> np.ndarray:
        """Paper height at table (x, y), arrays of one shape."""
        x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
        r = np.hypot(x[..., None] - self.centres[:, 0], y[..., None] - self.centres[:, 1])
        with np.errstate(divide="ignore", invalid="ignore"):
            phi = np.where(r > 0.0, r * r * np.log(np.where(r > 0.0, r, 1.0)), 0.0)
        dz = self.affine[0] + self.affine[1] * x + self.affine[2] * y + phi @ self.weights
        t = np.clip(1.0 - _hull_distance(self.hull, x, y) / self.taper, 0.0, 1.0)
        return self.paper_z + t * dz


def _hull_distance(hull, x, y):
    """Distance from the convex polygon (counter-clockwise), 0 inside."""
    p = np.stack([x, y], axis=-1)[..., None, :]                         # (..., 1, 2)
    a, e = hull, np.roll(hull, -1, axis=0) - hull                       # (H,2)
    u = np.clip(np.sum((p - a) * e, -1) / np.sum(e * e, -1), 0.0, 1.0)  # (..., H)
    d = np.linalg.norm(p - (a + u[..., None] * e), axis=-1).min(-1)
    left = e[:, 0] * (p[..., 1] - a[:, 1]) - e[:, 1] * (p[..., 0] - a[:, 0])
    return np.where(np.all(left >= 0.0, axis=-1), 0.0, d)


def read_paper(path: Path, paper_z: float) -> PaperMap | None:
    """The map, or None for the flat paper (no file, or a file of kind "flat").  Raises
    ValueError on a file that does not describe a surface over this paper."""
    if not path.exists():
        return None
    d = json.loads(path.read_text())
    if d.get("kind") == "flat":
        return None
    if d.get("kind") != "thin_plate_spline":
        raise ValueError(f"{path}: unknown kind {d.get('kind')!r}")
    if abs(float(d["paper_z_m"]) - paper_z) > 1e-12:
        raise ValueError(f"{path}: made for paper_z {d['paper_z_m']}, rig.json says {paper_z}")
    c = np.asarray(d["centres_m"], float).reshape(-1, 2)
    w = np.asarray(d["weights"], float).reshape(-1)
    hull = np.asarray(d["hull_m"], float).reshape(-1, 2)
    if len(c) != len(w) or len(hull) < 3 or not np.all(np.isfinite(w)):
        raise ValueError(f"{path}: centres, weights and hull do not make a surface")
    pts = np.asarray(d.get("points_table_m", []), float).reshape(-1, 3)
    low = float(pts[:, 2].min()) if len(pts) else paper_z
    about = (f"paper height map {path.name} ({d.get('date', 'undated')}, {len(c)} touches, "
             f"{', '.join(d.get('slots', ())) or 'slots not given'})")
    return PaperMap(paper_z, c, w, np.asarray(d["affine"], float).reshape(3), hull,
                    float(d["taper_m"]), min(low, paper_z), about)
