"""The paper's height over the whole table, from every slot's plane-job touches (DESIGN.md
section 6, the height map).

Each plane job keeps its touches in the table frame (the `base` part's `height_map_table_m`:
x, y and the measured z, which is the paper's height there as that arm sees it).  This module
gathers them from every slot into one field and fits one smooth surface through them: a
thin-plate spline with a small smoothing term (one method, one setting, `SMOOTHING`).  Why a
thin-plate spline: it is the smoothest surface through scattered points (least bending), needs
no grid, works the same for 8 points under one arm and 150 under six, and is evaluated with
numpy alone from its centres and weights, so the checker can read the same file with its own
reader.  Away from the touches the surface fades back to the flat paper (z = paper_z): fully
inside the points' convex hull, linearly to nothing over `TAPER_M` outside it.

`build_surface(config_dir)` fits, `write_paper(surface, config_dir)` writes
`calibration/paper.json`, `surface(config_dir)` reads it back (the flat paper when there is
none).  See docs/modules/calib.md.
"""
from __future__ import annotations

import datetime
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SMOOTHING = 1e-4     # thin-plate smoothing (m^2, added to the kernel's diagonal): the fit may
                     # miss each touch by a little (0.01-0.02 mm at 5-10 cm spacing) instead of
                     # bending through it, and two touches at nearly one spot (overlapping
                     # grids) do not make the system singular; 1e-3 already flattens 30 cm
                     # bumps by 0.1 mm, 0 interpolates every touch exactly
TAPER_M = 0.10       # outside the touches' hull the surface fades to the flat paper over this
MIN_POINTS = 3       # fewer (or all on a line): no surface, the flat paper
EVALUATE = ("z(x, y) = paper_z + t(d) * (a0 + ax*x + ay*y + sum_i w_i * phi(|(x, y) - c_i|)), "
            "phi(r) = r^2 ln r (0 at r = 0); d = distance from the hull polygon (0 inside); "
            "t = 1 inside the hull, 1 - d/taper_m up to taper_m outside, 0 beyond")


def _phi(r):
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(r > 0.0, r * r * np.log(np.where(r > 0.0, r, 1.0)), 0.0)


def _hull(xy):
    """Convex hull of (N,2) points, counter-clockwise (monotone chain); [] if degenerate."""
    pts = sorted(set(map(tuple, np.round(xy, 9))))
    if len(pts) < 3:
        return np.zeros((0, 2))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    h = np.array(lower[:-1] + upper[:-1])
    return h if len(h) >= 3 else np.zeros((0, 2))


def _outside(hull, x, y):
    """Distance of each point from the hull polygon, 0 inside (vectorised)."""
    p = np.stack([x, y], axis=-1)
    a, b = hull, np.roll(hull, -1, axis=0)
    ab = b - a
    t = np.clip(np.einsum("...kj,kj->...k", p[..., None, :] - a, ab) / np.sum(ab * ab, 1), 0, 1)
    d = np.min(np.linalg.norm(p[..., None, :] - (a + t[..., None] * ab), axis=-1), axis=-1)
    inside = np.all((ab[:, 0] * (p[..., None, 1] - a[:, 1])
                     - ab[:, 1] * (p[..., None, 0] - a[:, 0])) >= 0.0, axis=-1)
    return np.where(inside, 0.0, d)


@dataclass(frozen=True)
class Surface:
    """The paper's height over the table.  With no centres it is the flat paper."""
    paper_z: float
    centres: np.ndarray        # (N,2) table frame, the touches' xy
    weights: np.ndarray        # (N,)
    affine: np.ndarray         # (3,) a0, ax, ay
    hull: np.ndarray           # (H,2) counter-clockwise, [] for the flat paper
    points: np.ndarray         # (N,3) the touches as measured, table frame
    residual_mm: float         # RMS of fit minus touch, mm (0 for the flat paper)
    date: str                  # the newest plane job used, "" for the flat paper
    slots: tuple = ()          # the slots whose touches went in
    smoothing: float = SMOOTHING
    taper_m: float = TAPER_M

    @property
    def n_points(self) -> int:
        return len(self.points)

    @property
    def range(self) -> tuple[float, float]:
        """(lowest, highest) measured paper height, metres (the flat paper: paper_z twice)."""
        if not len(self.points):
            return (self.paper_z, self.paper_z)
        return (float(self.points[:, 2].min()), float(self.points[:, 2].max()))

    def z(self, x, y) -> np.ndarray:
        """The paper height at table (x, y); arrays of any matching shape."""
        x, y = np.broadcast_arrays(np.asarray(x, float), np.asarray(y, float))
        if not len(self.centres):
            return np.full(x.shape, self.paper_z)
        r = np.hypot(x[..., None] - self.centres[:, 0], y[..., None] - self.centres[:, 1])
        dz = self.affine[0] + self.affine[1] * x + self.affine[2] * y + _phi(r) @ self.weights
        t = np.clip(1.0 - _outside(self.hull, x, y) / self.taper_m, 0.0, 1.0)
        return self.paper_z + t * dz


def flat(paper_z: float = 0.0) -> Surface:
    return Surface(paper_z, np.zeros((0, 2)), np.zeros(0), np.zeros(3), np.zeros((0, 2)),
                   np.zeros((0, 3)), 0.0, "")


def fit(points, paper_z: float = 0.0, date: str = "", slots=()) -> Surface:
    """The smoothed thin-plate spline through (N,3) table-frame touches; the flat paper when
    there are too few or they lie on a line."""
    P = np.asarray(points, float).reshape(-1, 3)
    hull = _hull(P[:, :2]) if len(P) >= MIN_POINTS else np.zeros((0, 2))
    if not len(hull):
        return flat(paper_z)
    xy, dz = P[:, :2], P[:, 2] - paper_z
    n = len(P)
    K = _phi(np.linalg.norm(xy[:, None] - xy[None], axis=-1)) + SMOOTHING * np.eye(n)
    A = np.column_stack([np.ones(n), xy])
    M = np.block([[K, A], [A.T, np.zeros((3, 3))]])
    sol = np.linalg.lstsq(M, np.r_[dz, np.zeros(3)], rcond=None)[0]
    s = Surface(paper_z, xy.copy(), sol[:n], sol[n:], hull, P.copy(), 0.0, date, tuple(slots))
    res = s.z(xy[:, 0], xy[:, 1]) - P[:, 2]
    return Surface(paper_z, xy.copy(), sol[:n], sol[n:], hull, P.copy(),
                   float(np.sqrt(np.mean(res ** 2)) * 1e3), date, tuple(slots))


def _touches(cal: dict):
    """-> ((N,3) table-frame touches, date) from one slot file's base part, or None.  A marks
    base part keeps the plane job under "plane": its touches were placed with the pose of that
    time and are moved into the pose the marks job found."""
    base = cal.get("base") or {}
    if not base.get("passed"):
        return None
    plane = base if base.get("method") == "plane" else base.get("plane")
    if not isinstance(plane, dict) or not plane.get("height_map_table_m"):
        return None
    P = np.asarray(plane["height_map_table_m"], float).reshape(-1, 3)
    if plane is not base:
        T_then = np.asarray(plane["T_table_base"], float)
        T_now = np.asarray(base["T_table_base"], float)
        p_base = (P - T_then[:3, 3]) @ T_then[:3, :3]
        P = p_base @ T_now[:3, :3].T + T_now[:3, 3]
    return P, str(plane.get("date", ""))


def build_surface(config_dir) -> Surface:
    """One surface through every slot's plane-job touches under `config_dir`."""
    config_dir = Path(config_dir)
    paper_z = float(json.loads((config_dir / "rig.json").read_text())["table"]
                    ["paper_surface_z_m"])
    pts, dates, slots = [], [], []
    for f in sorted((config_dir / "calibration").glob("*.json")):
        try:
            cal = json.loads(f.read_text())
        except ValueError:
            continue
        if not isinstance(cal, dict) or "slot" not in cal:
            continue
        got = _touches(cal)
        if got is not None:
            pts.append(got[0])
            dates.append(got[1])
            slots.append(cal["slot"])
    if not pts:
        return flat(paper_z)
    return fit(np.concatenate(pts), paper_z, max(dates), slots)


def paper_path(config_dir) -> Path:
    return Path(config_dir) / "calibration" / "paper.json"


def write_paper(s: Surface, config_dir, date: str | None = None) -> Path:
    """calibration/paper.json: everything needed to evaluate the surface with numpy alone."""
    from aris.calib.files import _write_json
    lst = lambda a, nd=9: np.round(np.asarray(a, float), nd).tolist()
    data = {
        "kind": "thin_plate_spline" if len(s.centres) else "flat",
        "date": s.date or date or datetime.date.today().isoformat(),
        "written": date or datetime.date.today().isoformat(),
        "paper_z_m": s.paper_z, "smoothing": s.smoothing, "taper_m": s.taper_m,
        "evaluate": EVALUATE,
        "centres_m": lst(s.centres), "weights": lst(s.weights, 12), "affine": lst(s.affine, 12),
        "hull_m": lst(s.hull),
        "points_table_m": lst(s.points), "slots": list(s.slots),
        "n_points": s.n_points, "residual_mm": round(s.residual_mm, 4),
        "range_mm": [round((v - s.paper_z) * 1e3, 4) for v in s.range],
    }
    return _write_json(paper_path(config_dir), data)


def surface(config_dir) -> Surface:
    """The surface in calibration/paper.json; the flat paper when there is none."""
    config_dir = Path(config_dir)
    p = paper_path(config_dir)
    if not p.exists():
        paper_z = float(json.loads((config_dir / "rig.json").read_text())["table"]
                        ["paper_surface_z_m"])
        return flat(paper_z)
    d = json.loads(p.read_text())
    arr = lambda k, shape: np.asarray(d.get(k, []), float).reshape(shape)
    return Surface(float(d["paper_z_m"]), arr("centres_m", (-1, 2)), arr("weights", (-1,)),
                   arr("affine", (3,)), arr("hull_m", (-1, 2)), arr("points_table_m", (-1, 3)),
                   float(d.get("residual_mm", 0.0)), str(d.get("date", "")),
                   tuple(d.get("slots", ())), float(d.get("smoothing", SMOOTHING)),
                   float(d.get("taper_m", TAPER_M)))
