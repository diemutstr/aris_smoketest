"""Drawable maps: for one arm in one phase, the part of the canvas that arm can draw.

A grid over the canvas (`Settings.grid_step`, 2 cm).  A grid point is drawable when at least
one drawing configuration puts the pen tip there, square to the paper (no lean), with one of
`n_spin` hand spins and any elbow value and IK branch, and passes the local planner's own node
test: joint limits, singular value, the arm against the paper and against itself, then every
obstacle of that phase (steel, walls, parked arms).  The test is the local planner's graph
layer (`aris.local.lattice.Lattice`), one layer per grid point, so the map and the planner judge
a configuration the same way.

State per grid point: 0 out of reach, 1 reachable but blocked by an obstacle, 2 drawable.

Maps are built once per rig and kept in a directory the caller names, one file per (phase, arm),
with a digest of everything the map depends on in the file name (the arm, its pose, its tool,
the phase's obstacles, the gates, the grid).  No directory, no cache: built every time.
"""
from __future__ import annotations

import hashlib
import os
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from aris.local.gates import Judge
from aris.local.lattice import Lattice
from aris.local.settings import Settings as LocalSettings
from aris.system.settings import Settings
from aris.types import Gates, Obstacles, Phase, Slot

VERSION = 1
_CHUNK = 300                 # grid points per lattice; bounds memory
OUT, BLOCKED, DRAWABLE = 0, 1, 2


@dataclass(frozen=True)
class Map:
    phase: str
    arm_id: Slot
    x: np.ndarray            # (nx,) grid x, table frame
    y: np.ndarray            # (ny,) grid y
    state: np.ndarray        # (nx, ny) int8: OUT, BLOCKED or DRAWABLE
    cpu: float = 0.0         # s it took to build (0 when read from the cache)

    @property
    def share(self) -> float:
        """Share of the canvas grid that is drawable."""
        return float(np.mean(self.state == DRAWABLE))

    def contains(self, p_table: np.ndarray) -> np.ndarray:
        """(n,) bool: each point's nearest grid point is drawable (off the grid: no)."""
        p = np.asarray(p_table, float).reshape(-1, 3)
        step = self.x[1] - self.x[0]
        i = np.rint((p[:, 0] - self.x[0]) / step).astype(int)
        j = np.rint((p[:, 1] - self.y[0]) / step).astype(int)
        inside = (i >= 0) & (i < len(self.x)) & (j >= 0) & (j < len(self.y))
        out = np.zeros(len(p), bool)
        out[inside] = self.state[i[inside], j[inside]] == DRAWABLE
        return out


def grid(rig, step: float) -> tuple[np.ndarray, np.ndarray]:
    """Grid points over the canvas, edges included, centred on the canvas."""
    half = 0.5 * np.asarray(rig.canvas_size, float)
    n = np.floor(half / step + 1e-9).astype(int)
    return step * np.arange(-n[0], n[0] + 1), step * np.arange(-n[1], n[1] + 1)


def _obstacle_digest(h, obs: Obstacles) -> None:
    for b in obs.boxes:
        h.update(repr((b.name, b.exempt)).encode())
        h.update(np.concatenate([b.T_base_box.ravel(), b.half, [b.margin]]).tobytes())
    for p in obs.planes:
        h.update(repr((p.name, p.kind, p.pen_margin, p.tool_margin)).encode())
        h.update(np.concatenate([p.normal, [p.offset, p.margin]]).tobytes())
    for c in obs.capsules:
        h.update(c.name.encode())
        h.update(np.concatenate([c.p0, c.p1, [c.radius, c.margin]]).tobytes())


def digest(rig, phase: Phase, arm_id: Slot, gates: Gates, cfg: Settings, press: float = 0.0) -> str:
    """Everything the map of `arm_id` in `phase` depends on."""
    h = hashlib.blake2b(digest_size=12)
    arm = rig.arm(arm_id)
    tool = arm.tool
    h.update(repr((VERSION, phase.name, arm_id, gates, cfg.grid_step, cfg.n_spin, cfg.reach,
                   press, tool.pen_names)).encode())
    for a in (rig.T_table_base(arm_id), np.asarray(rig.canvas_size, float), tool.tip_hand,
              tool.pen_axis_hand, arm.limits.q_min, arm.limits.q_max):
        h.update(np.ascontiguousarray(a, float).tobytes())
    for c in tool.capsules_hand:
        h.update(np.concatenate([c.p0, c.p1, [c.radius]]).tobytes())
    _obstacle_digest(h, rig.obstacles_for(arm_id, phase))
    return h.hexdigest()


def build(rig, phase: Phase, arm_id: Slot, gates: Gates, cfg: Settings, obstacles=None,
          press: float = 0.0) -> Map:
    """The map of one arm in one phase, computed; against `obstacles` if given, else the
    phase's (`rig.obstacles_for`).  The pen tip is put
    on the drawing surface, `press` below the paper."""
    t0 = time.process_time()
    x, y = grid(rig, cfg.grid_step)
    X, Y = np.meshgrid(x, y, indexing="ij")
    p_table = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, rig.paper_z - press)])
    axis = rig.T_table_base(arm_id)[:2, 3]
    near = np.flatnonzero(np.linalg.norm(p_table[:, :2] - axis, axis=1) <= cfg.reach)
    T = rig.T_base_table(arm_id)
    p_base = p_table[near] @ T[:3, :3].T + T[:3, 3]
    arm = rig.arm(arm_id)
    obstacles = rig.obstacles_for(arm_id, phase) if obstacles is None else obstacles
    judge = Judge(arm, obstacles, gates)
    node_cfg = LocalSettings(n_spin=cfg.n_spin)
    state = np.zeros(X.size, np.int8)
    for a in range(0, len(near), _CHUNK):
        tips = p_base[a:a + _CHUNK]
        lat = Lattice(arm, judge, tips, np.arange(len(tips), dtype=float), 0.0, node_cfg)
        state[near[a:a + _CHUNK]] = [DRAWABLE if L.free.any() else BLOCKED if len(L.q) else OUT
                                     for L in lat.layers]
    state = state.reshape(X.shape)
    return Map(phase.name, arm_id, x, y, state, time.process_time() - t0)


def _build_job(job):
    rig, phase, arm_id, gates, cfg, press = job
    return build(rig, phase, arm_id, gates, cfg, press=press)


def load_or_build(rig, phases, gates: Gates, cfg: Settings, cache_dir=None,
                  workers: int = 1, press: float = 0.0) -> dict:
    """{(phase name, arm id): Map} for every active arm of every phase.  Read from `cache_dir`
    where present; the rest built (in `workers` processes) and, with `cache_dir`, saved."""
    keys = [(p, a) for p in phases for a in p.active]
    files = {}
    if cache_dir is not None:
        files = {(p.name, a): Path(cache_dir) / f"system_map_{digest(rig, p, a, gates, cfg, press)}.npz"
                 for p, a in keys}
    out, todo = {}, []
    for p, a in keys:
        f = files.get((p.name, a))
        if f is not None and f.exists():
            d = np.load(f)
            out[(p.name, a)] = Map(p.name, a, d["x"], d["y"], d["state"])
        else:
            todo.append((rig, p, a, gates, cfg, press))
    if workers > 1 and len(todo) > 1:
        with ProcessPoolExecutor(min(workers, len(todo)), mp_context=get_context("spawn")) as ex:
            built = list(ex.map(_build_job, todo))
    else:
        built = [_build_job(j) for j in todo]
    for m in built:
        out[(m.phase, m.arm_id)] = m
        f = files.get((m.phase, m.arm_id))
        if f is not None:
            f.parent.mkdir(parents=True, exist_ok=True)
            tmp = f.with_suffix(f".{os.getpid()}.tmp.npz")   # written whole, then renamed
            np.savez_compressed(tmp, x=m.x, y=m.y, state=m.state)
            os.replace(tmp, f)
    return {(p.name, a): out[(p.name, a)] for p, a in keys}


def coverage(maps: dict, phases) -> dict:
    """{phase name: {arm id: share, "union": share}}, and {"all": share} over every map.  A
    phase nobody moves in (an unmounted rig's leaders) is left out; no maps at all: 0.0."""
    rep, every = {}, None
    for p in phases:
        if not p.active:
            continue
        row, union = {}, None
        for a in p.active:
            d = maps[(p.name, a)].state == DRAWABLE
            row[a] = float(d.mean())
            union = d if union is None else union | d
        row["union"] = float(union.mean())
        rep[p.name] = row
        every = union if every is None else every | union
    rep["all"] = 0.0 if every is None else float(every.mean())
    return rep
