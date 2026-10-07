"""The paper's height map as the server uses it: read it, rebuild it after a plane job,
describe it.

The map (`config/calibration/paper.json`) is the calib agent's: one height field for the table
built from every slot's plane-job touch points, read by `aris.calib.paper.surface(config_dir)`
-> Surface (`z(x, y)` in the table frame; the flat paper where there are no points) and rebuilt
by `build_surface(config_dir)` + `write_paper(surface, config_dir)`.  Heights are reported in
millimetres about the nominal paper (`paper_z`).  The system planner draws on `z(x, y)` less the
press; the checker reads the same file itself.  Until that module lands every function here
answers "no surface" and the planners keep the flat paper.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def _calib():
    try:
        from aris.calib import paper
    except ImportError:
        return None
    return paper


def load(config_dir):
    """The table's paper surface, or None: no map module yet, or a surface without measured
    points (the reader's flat-plane fallback, which the planners already have).  Read only
    through `aris.calib.paper.surface`; the server never opens the file itself."""
    mod = _calib()
    if mod is None:
        return None
    s = mod.surface(config_dir)
    return None if s is None or not len(_points(s)) else s


def rebuild(config_dir) -> str:
    """The map rebuilt from every slot's plane-job points: the file written, or why not."""
    mod = _calib()
    if mod is None:
        return "not rebuilt: no paper-surface module (aris.calib.paper)"
    return str(mod.write_paper(mod.build_surface(config_dir), config_dir))


def _points(surface) -> np.ndarray:
    p = getattr(surface, "points", None)
    return np.zeros((0, 3)) if p is None else np.asarray(p, float).reshape(-1, 3)


def describe(surface) -> dict:
    """For `aris rig`: whether there is a surface, its points, its height range, its date."""
    if surface is None:
        return dict(exists=False)
    p, z0 = _points(surface), float(getattr(surface, "paper_z", 0.0))
    return dict(exists=True, points=len(p),
                z_min_m=float(p[:, 2].min()) - z0 if len(p) else None,
                z_max_m=float(p[:, 2].max()) - z0 if len(p) else None,
                date=getattr(surface, "date", None))


def under(surface, lines) -> dict | None:
    """The surface's height range under a drawing (every point of its table-frame lines, about
    the nominal paper), for the drawing report; None without a surface."""
    if surface is None or not lines:
        return None
    xy = np.concatenate([np.asarray(x.points, float)[:, :2] for x in lines])
    z = np.asarray(surface.z(xy[:, 0], xy[:, 1]), float) - float(getattr(surface, "paper_z", 0.0))
    return dict(z_min_m=float(z.min()), z_max_m=float(z.max()),
                range_mm=1e3 * float(z.max() - z.min()))
