"""The rig: what the installation looks like, and the one place that turns table-frame things
into one arm's base frame.

Reads `config/rig.json` and, when present, `config/calibration/<arm_id>.json`.  Nothing else in
the package reads `config/`.  Everything handed out is in one arm's base frame (obstacles,
lines, planes) except `wall_between`, which is a table-frame thing by nature: it belongs to two
arms at once.  See docs/modules/rig.md.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from aris.kernel.arm import Arm, default_tool
from aris.types import Box, Capsule, Gates, Line, Obstacles, Phase, Plane, Wall


@dataclass(frozen=True)
class SteelBox:
    """One axis-aligned box of steel in the table frame."""
    name: str
    lo_table: np.ndarray               # (3,)
    hi_table: np.ndarray               # (3,)
    source: str


@dataclass(frozen=True)
class Mount:
    """Where one arm hangs.  `T_table_base` is the calibrated pose when a calibration applies."""
    arm_id: int
    axis_xy_table: np.ndarray          # (2,) nominal, from rig.json; walls use this
    T_table_base: np.ndarray           # (4, 4)
    strut_wide_side: str               # "-x" or "+x"
    park_q: np.ndarray                 # (7,)
    tip_hand: np.ndarray | None        # (3,) calibrated pen tip in the hand frame, or None
    calibration: str                   # "none", "applied: <file>" or "not applied: <why>"


def _rot_ok(R: np.ndarray) -> bool:
    return bool(np.allclose(R.T @ R, np.eye(3), atol=1e-6) and np.linalg.det(R) > 0.0)


def _read_calibration(path: Path, arm_id: int, T_nominal: np.ndarray):
    """-> (T_table_base, tip_hand or None, status).  A malformed file is a broken install: raise."""
    if not path.exists():
        return T_nominal, None, "none"
    cal = json.loads(path.read_text())
    if int(cal["arm_id"]) != arm_id:
        raise ValueError(f"{path}: arm_id {cal['arm_id']} in a file for arm {arm_id}")
    if not cal.get("passed", False):
        return T_nominal, None, f"not applied: {path.name} did not pass"
    T = np.asarray(cal["T_table_base"], float)
    if T.shape != (4, 4) or not _rot_ok(T[:3, :3]) or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError(f"{path}: T_table_base is not a rigid transform")
    tip = cal.get("tip_hand_m")
    tip = None if tip is None else np.asarray(tip, float).reshape(3)
    return T, tip, f"applied: {path.name} ({cal.get('date', 'undated')})"


def _hanger_boxes(arm_id: int, axis_xy: np.ndarray, side: str, h: dict) -> list[SteelBox]:
    """The two struts, the plate and the clamp of one arm, placed from its axis."""
    if side not in ("-x", "+x"):
        raise ValueError(f"arm {arm_id}: strut_wide_side must be '-x' or '+x', not {side!r}")
    s = -1.0 if side == "-x" else 1.0
    ax, ay = float(axis_xy[0]), float(axis_xy[1])
    wide, narrow, sx, sy = (h["wide_side_outer_face_to_axis_m"],
                            h["narrow_side_outer_face_to_axis_m"],
                            h["strut_size_x_m"], h["strut_size_y_m"])
    z0, z1 = h["strut_bottom_z_m"], h["strut_top_z_m"]

    def box(name, x_a, x_b, half_y, za, zb, src):
        return SteelBox(name, np.array([min(x_a, x_b), ay - half_y, za]),
                        np.array([max(x_a, x_b), ay + half_y, zb]), src)

    pc = ax + s * h["plate_centre_toward_wide_side_m"]
    return [
        box(f"strut{arm_id}_wide", ax + s * wide, ax + s * (wide - sx), sy / 2, z0, z1,
            h["strut_source"] + f" Wide (0.240) side assumed toward table {side}."),
        box(f"strut{arm_id}_narrow", ax - s * narrow, ax - s * (narrow - sx), sy / 2, z0, z1,
            h["strut_source"]),
        box(f"plate{arm_id}", pc - h["plate_size_x_m"] / 2, pc + h["plate_size_x_m"] / 2,
            h["plate_size_y_m"] / 2, h["plate_bottom_z_m"], h["plate_top_z_m"],
            h["plate_source"]),
        box(f"clamp{arm_id}", pc - h["clamp_size_x_m"] / 2, pc + h["clamp_size_x_m"] / 2,
            h["clamp_size_y_m"] / 2, h["clamp_bottom_z_m"], h["clamp_top_z_m"],
            h["clamp_source"]),
    ]


def _point_box_distance(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0)))


@dataclass(frozen=True)
class Rig:
    mounts: dict                       # arm_id -> Mount, in rig.json order
    steel: tuple[SteelBox, ...]        # every box, table frame, cage first then hangers
    clearance: dict                    # demanded, metres (rig.json "clearances")
    allowance: dict                    # planning allowance, metres (rig.json "planning_allowance")
    shoulder_below_base: float
    body_reach: float                  # from the shoulder, see rig.json "reach"
    rows: tuple[tuple[int, int], ...]
    leader_sets: dict                  # phase -> tuple of arm ids
    wall_pairs: dict                   # phase -> tuple of (arm, arm) with a wall between them
    canvas_size: np.ndarray            # (2,)
    table_size: np.ndarray             # (2,)
    paper_z: float

    # ------------------------------------------------------------------ loading

    @staticmethod
    def load(config_dir) -> "Rig":
        config_dir = Path(config_dir)
        cfg = json.loads((config_dir / "rig.json").read_text())
        mounts, hangers = {}, []
        for a in cfg["arms"]["list"]:
            aid = int(a["id"])
            T = np.eye(4)
            T[:3, :3] = np.asarray(a["R_table_base"], float)
            T[:3, 3] = [a["axis_xy_m"][0], a["axis_xy_m"][1], a["base_z_m"]]
            if not _rot_ok(T[:3, :3]):
                raise ValueError(f"arm {aid}: R_table_base is not a rotation")
            T, tip, status = _read_calibration(config_dir / "calibration" / f"{aid}.json", aid, T)
            axis = np.asarray(a["axis_xy_m"], float)
            mounts[aid] = Mount(aid, axis, T, a["strut_wide_side"],
                                np.asarray(a["park_q_rad"], float), tip, status)
            hangers += _hanger_boxes(aid, axis, a["strut_wide_side"], cfg["hanger"])
        cage = [SteelBox(b["name"], np.asarray(b["lo_m"], float), np.asarray(b["hi_m"], float),
                         b["source"]) for b in cfg["steel"]["boxes"]]
        rp = cfg["rows_and_phases"]
        num = lambda d: {k: float(v) for k, v in d.items() if k.endswith("_m")}
        return Rig(
            mounts=mounts, steel=tuple(cage + hangers),
            clearance=num(cfg["clearances"]), allowance=num(cfg["planning_allowance"]),
            shoulder_below_base=float(cfg["reach"]["shoulder_below_base_m"]),
            body_reach=float(cfg["reach"]["body_reach_from_shoulder_m"]),
            rows=tuple(tuple(int(i) for i in r) for r in rp["rows"]),
            leader_sets={int(k): tuple(int(i) for i in v) for k, v in rp["leaders"].items()},
            wall_pairs={int(k): tuple((int(a), int(b)) for a, b in v)
                        for k, v in rp["walls"].items()},
            canvas_size=np.array([cfg["canvas"]["size_x_m"], cfg["canvas"]["size_y_m"]]),
            table_size=np.array([cfg["table"]["size_x_m"], cfg["table"]["size_y_m"]]),
            paper_z=float(cfg["table"]["paper_surface_z_m"]),
        )

    # ------------------------------------------------------------------ arms and frames

    @property
    def arm_ids(self) -> tuple[int, ...]:
        return tuple(self.mounts)

    def _mount(self, arm_id: int) -> Mount:
        if arm_id not in self.mounts:
            raise KeyError(f"no arm {arm_id} on this rig; arms are {self.arm_ids}")
        return self.mounts[arm_id]

    def T_table_base(self, arm_id: int) -> np.ndarray:
        return self._mount(arm_id).T_table_base.copy()

    def T_base_table(self, arm_id: int) -> np.ndarray:
        T = self._mount(arm_id).T_table_base
        out = np.eye(4)
        out[:3, :3] = T[:3, :3].T
        out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
        return out

    def park_q(self, arm_id: int) -> np.ndarray:
        return self._mount(arm_id).park_q.copy()

    def calibration_status(self, arm_id: int) -> str:
        return self._mount(arm_id).calibration

    def arm(self, arm_id: int):
        """The arm model with this arm's tool (the calibrated tip when there is one)."""
        tool = default_tool()
        tip = self._mount(arm_id).tip_hand
        if tip is not None:
            tool = replace(tool, tip_hand=tip.copy())
        return Arm(tool)

    def to_base(self, arm_id: int, line: Line) -> Line:
        if line.frame != "table":
            raise ValueError(f"line {line.id} is in the {line.frame} frame, not the table frame")
        T = self.T_base_table(arm_id)
        p = np.asarray(line.points, float) @ T[:3, :3].T + T[:3, 3]
        return Line(line.id, p, "base", line.intensity)

    def to_table(self, arm_id: int, points_base) -> np.ndarray:
        T = self._mount(arm_id).T_table_base
        return np.asarray(points_base, float) @ T[:3, :3].T + T[:3, 3]

    def _plane_in_base(self, arm_id, name, n_table, point_table, margin, kind, pen_margin):
        T = self.T_base_table(arm_id)
        n = T[:3, :3] @ n_table
        p = T[:3, :3] @ point_table + T[:3, 3]
        return Plane(name, n, float(n @ p), margin, kind, pen_margin)

    def paper(self, arm_id: int, for_planning: bool = False) -> Plane:
        """The paper surface in this arm's base frame; free side is up, toward the arm."""
        c, a = self.clearance, self.allowance
        body = c["body_to_paper_m"] + (a["body_to_paper_m"] if for_planning else 0.0)
        pen = c["pen_lifted_to_paper_m"] + (a["pen_lifted_to_paper_m"] if for_planning else 0.0)
        return self._plane_in_base(arm_id, "paper", np.array([0.0, 0.0, 1.0]),
                                   np.array([0.0, 0.0, self.paper_z]), body, "paper", pen)

    def self_margin(self, for_planning: bool = False) -> float:
        return self.clearance["self_m"] + (self.allowance["self_m"] if for_planning else 0.0)

    def gates(self) -> Gates:
        """The planners' gates, with the self clearance taken from rig.json (demanded plus the
        planning allowance).  Joint-limit margin and sigma_min are not rig facts: defaults."""
        return Gates(self_margin=self.self_margin(for_planning=True))

    # ------------------------------------------------------------------ walls

    def wall_between(self, arm_a: int, arm_b: int) -> Wall:
        """Halfway between the two arms' nominal axes, square to the line joining them."""
        pa, pb = self._mount(arm_a).axis_xy_table, self._mount(arm_b).axis_xy_table
        d = pb - pa
        if np.linalg.norm(d) < 1e-9:
            raise ValueError(f"arms {arm_a} and {arm_b} share an axis")
        n = np.array([d[0], d[1], 0.0]) / np.linalg.norm(d)
        mid = 0.5 * (pa + pb)
        return Wall(f"wall_{arm_a}_{arm_b}", (arm_a, arm_b),
                    np.array([mid[0], mid[1], self.paper_z]), n)

    def wall_in_base(self, arm_id: int, wall: Wall, for_planning: bool = False) -> Plane:
        """The wall in this arm's base frame, free side = the side this arm's axis is on."""
        axis = self._mount(arm_id).axis_xy_table
        side = float(wall.normal_table[:2] @ (axis - wall.point_table[:2]))
        if abs(side) < 1e-9:
            raise ValueError(f"arm {arm_id} stands on {wall.name}")
        n = wall.normal_table * np.sign(side)
        margin = self.clearance["wall_m"] + (self.allowance["wall_m"] if for_planning else 0.0)
        return self._plane_in_base(arm_id, wall.name, n, wall.point_table, margin, "wall", None)

    # ------------------------------------------------------------------ obstacles

    def shoulder_table(self, arm_id: int) -> np.ndarray:
        T = self._mount(arm_id).T_table_base
        return T[:3, 3] + self.shoulder_below_base * T[:3, 2]

    def steel_for(self, arm_id: int, margin: float) -> tuple[list[SteelBox], list[SteelBox]]:
        """-> (boxes this arm must clear, boxes out of its reach).  A box is out of reach when
        it is further from the shoulder than the body reaches plus the clearance demanded."""
        s = self.shoulder_table(arm_id)
        near, far = [], []
        for b in self.steel:
            d = _point_box_distance(s, b.lo_table, b.hi_table)
            (far if d > self.body_reach + margin else near).append(b)
        return near, far

    def parked_capsules(self, arm_id: int, parked_id: int, for_planning: bool = True):
        """The body of `parked_id` standing at its park configuration, in `arm_id`'s base frame."""
        if parked_id == arm_id:
            raise ValueError(f"arm {arm_id} cannot be parked next to itself")
        body = self.arm(parked_id).body(self.park_q(parked_id)[None, :])
        T = self.T_base_table(arm_id) @ self._mount(parked_id).T_table_base
        R, t = T[:3, :3], T[:3, 3]
        m = self.clearance["arm_to_arm_m"] + (self.allowance["arm_to_arm_m"] if for_planning
                                              else 0.0)
        return tuple(Capsule(f"parked{parked_id}:{name}", R @ body.p0[0, k] + t,
                             R @ body.p1[0, k] + t, float(body.radius[k]), m)
                     for k, name in enumerate(body.names))

    def obstacles(self, arm_id: int, parked=(), walls=(), for_planning: bool = True) -> Obstacles:
        """Everything `arm_id` must stay clear of, in its base frame: the paper, the steel it
        can reach, the given walls, and the bodies of the given parked arms."""
        steel_m = self.clearance["steel_m"] + (self.allowance["steel_m"] if for_planning else 0.0)
        near, _ = self.steel_for(arm_id, steel_m)
        T = self.T_base_table(arm_id)
        boxes = []
        for b in near:
            Tb = T.copy()
            Tb[:3, 3] = T[:3, :3] @ (0.5 * (b.lo_table + b.hi_table)) + T[:3, 3]
            boxes.append(Box(b.name, Tb, 0.5 * (b.hi_table - b.lo_table), steel_m))
        planes = [self.paper(arm_id, for_planning)]
        planes += [self.wall_in_base(arm_id, w, for_planning) for w in walls]
        caps = [c for p in parked for c in self.parked_capsules(arm_id, p, for_planning)]
        return Obstacles(tuple(boxes), tuple(planes), tuple(caps))

    # ------------------------------------------------------------------ roles

    def leaders(self, phase: int) -> tuple[int, ...]:
        if phase not in self.leader_sets:
            raise ValueError(f"no phase {phase}; phases with fixed leaders are "
                             f"{tuple(self.leader_sets)}")
        return self.leader_sets[phase]

    def phase(self, n: int) -> Phase:
        """Phase n: the leaders move, the other three stand parked, walls between the leaders
        that can reach each other."""
        active = self.leaders(n)
        parked = tuple(a for a in self.arm_ids if a not in active)
        walls = tuple(self.wall_between(a, b) for a, b in self.wall_pairs[n])
        return Phase(f"phase {n}", active, parked, walls)

    def _parked_in_reach(self, arm_id: int, parked_id: int, margin: float) -> bool:
        """Can any part of `arm_id` come within `margin` of `parked_id` standing parked?"""
        caps = self.parked_capsules(arm_id, parked_id)
        s = np.array([0.0, 0.0, self.shoulder_below_base])      # shoulder, base frame
        for c in caps:
            d = c.p1 - c.p0
            t = np.clip((s - c.p0) @ d / max(d @ d, 1e-18), 0.0, 1.0)
            if np.linalg.norm(c.p0 + t * d - s) - c.radius <= self.body_reach + margin:
                return True
        return False

    def obstacles_for(self, arm_id: int, phase: Phase, for_planning: bool = True) -> Obstacles:
        """`obstacles` for an arm that moves in `phase`: its parked row partner, any other parked
        arm it can reach, and the walls it stands next to."""
        if arm_id not in phase.active:
            raise ValueError(f"arm {arm_id} does not move in {phase.name}")
        m = self.clearance["arm_to_arm_m"] + (self.allowance["arm_to_arm_m"] if for_planning
                                              else 0.0)
        partner = self.row_partner(arm_id)
        parked = tuple(p for p in phase.parked
                       if p == partner or self._parked_in_reach(arm_id, p, m))
        walls = tuple(w for w in phase.walls if arm_id in w.arms)
        return self.obstacles(arm_id, parked, walls, for_planning)

    def row_partner(self, arm_id: int) -> int:
        for a, b in self.rows:
            if arm_id in (a, b):
                return b if arm_id == a else a
        raise KeyError(f"arm {arm_id} is in no row")
