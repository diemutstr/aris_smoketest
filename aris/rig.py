"""The rig: what the installation looks like, and the one place that turns table-frame things
into one arm's base frame.

Reads `config/rig.json` and, when present, `config/calibration/<slot>.json`.  Nothing else in
the package reads `config/`; nothing here reads `site/` (which robot hangs in which slot is the
robot side's business).  Everything handed out is in one arm's base frame (obstacles, lines,
planes) except `wall_between`, which is a table-frame thing by nature: it belongs to two arms at
once.  An arm is named by its slot ("1L" .. "3R", `types.Slot`).  See docs/modules/rig.md.

Longer than 400 lines because it is the one reader of the rig's sources (slots, pens, the two
calibration parts, steel, phases) and the one place that builds geometry from them; splitting
the reader from the frame calls would give two files that only make sense together.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from aris.kernel.arm import Arm
from aris.kernel.tool import default_tool, with_tip
from aris.types import (Box, Capsule, DrawRules, Gates, Line, Obstacles, Phase, Plane, Slot,
                        Tool, Wall)

SLOT_PATTERN = re.compile(r"[123][LR]")    # row 1..3 along table y, L/R by table x
# aris/kernel/tool.py builds its pen capsule for graphite this far past the cap's outer face;
# a pen of another nominal length moves the tip along the pen axis by the difference.
MODEL_TIP_LENGTH = 0.020


def is_slot(name) -> bool:
    return isinstance(name, str) and SLOT_PATTERN.fullmatch(name) is not None


@dataclass(frozen=True)
class SteelBox:
    """One axis-aligned box of steel in the table frame."""
    name: str
    lo_table: np.ndarray               # (3,)
    hi_table: np.ndarray               # (3,)
    source: str
    owner: Slot | None = None          # the slot hanging from it, for hanger boxes


@dataclass(frozen=True)
class Execution:
    """What the executor checks before it runs a motion (rig.json "execution")."""
    start_tolerance: float             # rad, per joint: measured vs the motion's start


@dataclass(frozen=True)
class Mount:
    """Where one arm hangs.  `T_table_base` is the calibrated pose when the base part applies."""
    slot: Slot
    axis_xy_table: np.ndarray          # (2,) nominal, from rig.json; walls use this
    T_table_base: np.ndarray           # (4, 4)
    park_q: np.ndarray                 # (7,)
    tool: Tool                         # holder and pen: measured tip, or the pen's nominal length
    tip_hand: np.ndarray               # (3,) = tool.tip_hand, the tip every model of this arm uses
    calibration: dict                  # {"base": status, "pen": status}, see calibration_status
    T_nominal: np.ndarray | None = None  # (4, 4) the pose from rig.json alone, before calibration


def _rot_ok(R: np.ndarray) -> bool:
    return bool(np.allclose(R.T @ R, np.eye(3), atol=1e-6) and np.linalg.det(R) > 0.0)


def _why(part: dict) -> str:
    return part.get("why") or "no reason given"


def _read_calibration(path: Path, slot: Slot, T_nominal: np.ndarray, pen_name: str):
    """-> (T_table_base, measured tip_hand or None, {"base": status, "pen": status}).  The two
    parts are independent: each applies when it passed (the pen part also only for the pen that
    is in).  A malformed file is a broken install: raise."""
    status = {"base": "none", "pen": "none"}
    if not path.exists():
        return T_nominal, None, status
    cal = json.loads(path.read_text())
    if cal.get("slot") != slot:
        raise ValueError(f"{path}: slot {cal.get('slot')!r} in a file for slot {slot}")
    T, tip = T_nominal, None
    base, pen = cal.get("base"), cal.get("pen")
    if base is not None and not base.get("passed", False):
        status["base"] = f"base part not applied: it did not pass ({_why(base)})"
    elif base is not None:
        T = np.asarray(base["T_table_base"], float)
        if T.shape != (4, 4) or not _rot_ok(T[:3, :3]) or not np.allclose(T[3], [0, 0, 0, 1]):
            raise ValueError(f"{path}: base.T_table_base is not a rigid transform")
        status["base"] = (f"applied: {path.name} base ({base.get('date', 'undated')}, "
                          f"{base.get('method', 'method not given')})")
    if pen is not None and not pen.get("passed", False):
        status["pen"] = f"pen part not applied: it did not pass ({_why(pen)})"
    elif pen is not None and pen.get("pen") != pen_name:
        status["pen"] = (f"pen part not applied: it was measured for pen {pen.get('pen')!r}, "
                         f"the pen in is {pen_name!r}")
    elif pen is not None:
        tip = np.asarray(pen["tip_hand_m"], float)
        if tip.shape != (3,) or not np.all(np.isfinite(tip)):
            raise ValueError(f"{path}: pen.tip_hand_m is not three numbers")
        status["pen"] = f"applied: {path.name} pen {pen_name} ({pen.get('date', 'undated')})"
    return T, tip, status


def _pen_tool(pen: dict, tip_measured: np.ndarray | None) -> Tool:
    """The holder with this pen: the measured tip if there is one, else the tip moved along the
    pen axis by the pen's nominal length against the model's; the pen capsule's radius."""
    tool = default_tool()
    r = float(pen["capsule_radius_m"])
    tip = tip_measured
    if tip is None:
        tip = tool.tip_hand + (float(pen["tip_length_nominal_m"]) - MODEL_TIP_LENGTH) * \
            tool.pen_axis_hand
    pens = [c for c in tool.capsules_hand if c.name in tool.pen_names]
    if all(c.radius == r for c in pens) and np.array_equal(tip, tool.tip_hand):
        return tool                    # the model as built: no rounding from moving it
    caps = tuple(replace(c, radius=r) if c.name in tool.pen_names else c
                 for c in tool.capsules_hand)
    return with_tip(replace(tool, capsules_hand=caps), tip)


def _hanger_boxes(slot: Slot, axis_xy: np.ndarray, h: dict) -> list[SteelBox]:
    """The two struts, the plate and the clamp of one slot, placed from its axis."""
    ax, ay = float(axis_xy[0]), float(axis_xy[1])
    lo, hi, sx, sy = (h["axis_to_minus_x_outer_face_m"], h["axis_to_plus_x_outer_face_m"],
                      h["strut_size_x_m"], h["strut_size_y_m"])
    z0, z1 = h["strut_bottom_z_m"], h["strut_top_z_m"]

    def box(name, x_a, x_b, half_y, za, zb, src):
        return SteelBox(name, np.array([min(x_a, x_b), ay - half_y, za]),
                        np.array([max(x_a, x_b), ay + half_y, zb]), src, slot)

    pc = ax + h["plate_centre_offset_x_m"]
    return [
        box(f"strut{slot}_minus_x", ax - lo, ax - lo + sx, sy / 2, z0, z1, h["strut_source"]),
        box(f"strut{slot}_plus_x", ax + hi - sx, ax + hi, sy / 2, z0, z1, h["strut_source"]),
        box(f"plate{slot}", pc - h["plate_size_x_m"] / 2, pc + h["plate_size_x_m"] / 2,
            h["plate_size_y_m"] / 2, h["plate_bottom_z_m"], h["plate_top_z_m"],
            h["plate_source"]),
        box(f"clamp{slot}", pc - h["clamp_size_x_m"] / 2, pc + h["clamp_size_x_m"] / 2,
            h["clamp_size_y_m"] / 2, h["clamp_bottom_z_m"], h["clamp_top_z_m"],
            h["clamp_source"]),
    ]


def _check_slots(slots: list[dict]) -> None:
    """Slot names are well formed, unique, and say where the axis is: L at -x, R at +x, rows
    numbered in the order of y."""
    names = [s["slot"] for s in slots]
    bad = [n for n in names if not is_slot(n)]
    if bad or len(set(names)) != len(names):
        raise ValueError(f"rig.json: slot names {names}; each must be one of 1L .. 3R, once")
    for s in slots:
        if (s["axis_xy_m"][0] < 0.0) != (s["slot"][1] == "L"):
            raise ValueError(f"rig.json: slot {s['slot']} at x = {s['axis_xy_m'][0]}")
    by_y = [s["slot"][0] for s in sorted(slots, key=lambda s: s["axis_xy_m"][1])]
    if by_y != sorted(by_y):
        raise ValueError(f"rig.json: rows are not numbered along y: {by_y}")


def _pen(cfg: dict) -> tuple[str, dict]:
    """The pen that is in.  A pen without its own speed on the paper draws at
    drawing.draw_speed_m_per_s (the checker reads it the same way)."""
    pens = cfg["pens"]
    name = pens["current"]
    if name not in pens["table"]:
        raise ValueError(f"rig.json: pens.current {name!r} is not in pens.table")
    pen = pens["table"][name]
    need = ("press_m", "tip_length_nominal_m", "capsule_radius_m")
    if any(k not in pen for k in need):
        raise ValueError(f"rig.json: pen {name} lacks one of {need}")
    pen = {k: v for k, v in pen.items() if not k.endswith("note") and k != "source"}
    if "speed_m_per_s" not in pen:
        if "draw_speed_m_per_s" not in cfg["drawing"]:
            raise ValueError(f"rig.json: pen {name} has no speed_m_per_s and drawing has no "
                             "draw_speed_m_per_s")
        pen["speed_m_per_s"] = float(cfg["drawing"]["draw_speed_m_per_s"])
    return name, pen


def _marks(cfg: dict, config_dir: Path, slots: tuple):
    """-> (name -> (nominal xy, sharers), group -> slots, name -> marks.json entry).  A
    marks.json entry for a mark this rig does not have is left out: the calibration folder is
    shared with rigs of fewer arms."""
    m = cfg.get("marks", {})
    marks = {}
    for e in m.get("list", ()):
        share = tuple(e["shared_by"])
        if len(share) != 2 or not set(share) <= set(slots) or e["name"] in marks:
            raise ValueError(f"rig.json: mark {e['name']} shared by {share}")
        marks[e["name"]] = (np.asarray(e["xy_m"], float).reshape(2), share)
    groups = {k: tuple(v) for k, v in m.get("groups", {}).items()}
    if any(not set(v) <= set(slots) for v in groups.values()):
        raise ValueError(f"rig.json: a mark group names a slot not on the frame: {groups}")
    path = config_dir / "calibration" / "marks.json"
    solved = {}
    if path.exists():
        d = json.loads(path.read_text())
        d = d["marks"] if isinstance(d.get("marks"), dict) else d   # {"marks": {...}} or flat
        for name, e in d.items():
            if name not in marks or not isinstance(e, dict):
                continue
            if e.get("state") not in ("nominal", "solved"):
                raise ValueError(f"{path}: mark {name} state {e.get('state')!r}")
            solved[name] = dict(e, xy_m=np.asarray(e["xy_m"], float).reshape(2))
    return marks, groups, solved


def _point_box_distance(p: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0)))


@dataclass(frozen=True)
class Rig:
    mounts: dict                       # slot -> Mount, mounted slots only, in rig.json order
    slot_names: tuple[Slot, ...]       # every slot of the frame, mounted or not
    steel: tuple[SteelBox, ...]        # every box, table frame, cage first then hangers
    clearance: dict                    # demanded, metres (rig.json "clearances")
    allowance: dict                    # planning allowance, metres (rig.json "planning_allowance")
    gate_cfg: dict                     # rig.json "gates", as read
    drawing_cfg: dict                  # rig.json "drawing", as read
    execution_cfg: Execution
    pen_name: str                      # rig.json pens.current
    pen_cfg: dict                      # its pens.table entry, notes and source left out
    shoulder_below_base: float
    body_reach: float                  # from the shoulder, see rig.json "reach"
    own_hanger_exempt: tuple[str, ...] # body groups not checked against the arm's own hanger
    rows: tuple[tuple[Slot, Slot], ...]
    leader_sets: dict                  # phase -> tuple of slots
    wall_pairs: dict                   # phase -> tuple of (slot, slot) with a wall between them
    fences: tuple                      # (name, point_table, normal_table): planes every arm stays
                                       # on the normal's side of, in every phase (rig.json fences)
    canvas_size: np.ndarray            # (2,)
    drawing_area_m: np.ndarray | None  # (2,) x by y, around drawing_area_centre_m; written by
                                       # the system planner from its maps, None until it has
    drawing_area_centre_m: np.ndarray  # (2,) table frame, [0, 0] when rig.json has none
    table_size: np.ndarray             # (2,)
    paper_z: float
    marks: dict                        # name -> (nominal xy (2,), the two slots sharing it)
    mark_groups: dict                  # group name -> slots (`aris mark --group`)
    mark_files: dict                   # name -> its calibration/marks.json entry, when there

    # ------------------------------------------------------------------ loading

    @staticmethod
    def load(config_dir) -> "Rig":
        config_dir = Path(config_dir)
        cfg = json.loads((config_dir / "rig.json").read_text())
        pen_name, pen = _pen(cfg)
        _check_slots(cfg["slots"]["list"])
        mounts, hangers = {}, []
        for a in cfg["slots"]["list"]:
            slot = a["slot"]
            axis = np.asarray(a["axis_xy_m"], float)
            if not a.get("mounted", True):
                # The hanger is bolted to the frame whether or not an arm hangs from it today.
                hangers += _hanger_boxes(slot, axis, cfg["hanger"])
                continue
            T = np.eye(4)
            T[:3, :3] = np.asarray(a["R_table_base"], float)
            T[:3, 3] = [axis[0], axis[1], a["base_z_m"]]
            if not _rot_ok(T[:3, :3]):
                raise ValueError(f"slot {slot}: R_table_base is not a rotation")
            T_nominal = T.copy()
            T, tip, status = _read_calibration(config_dir / "calibration" / f"{slot}.json", slot,
                                               T, pen_name)
            # The arm is bolted to its plate: the hanger goes where the calibrated axis is.
            hangers += _hanger_boxes(slot, T[:2, 3], cfg["hanger"])
            tool = _pen_tool(pen, tip)
            mounts[slot] = Mount(slot, axis, T, np.asarray(a["park_q_rad"], float), tool,
                                 tool.tip_hand.copy(), status, T_nominal)
        if not mounts:
            raise ValueError("rig.json: no arm is mounted")
        cage = [SteelBox(b["name"], np.asarray(b["lo_m"], float), np.asarray(b["hi_m"], float),
                         b["source"]) for b in cfg["steel"]["boxes"]]
        # Rows, leaders and walls keep only the mounted slots: a row with one arm missing is no
        # row, a wall between a mounted and a missing arm is no wall.
        rp = cfg["rows_and_phases"]
        here = lambda ids: tuple(s for s in ids if s in mounts)
        fences = tuple((f["name"], np.asarray(f["point_m"], float),
                        np.asarray(f["normal"], float) / np.linalg.norm(f["normal"]))
                       for f in cfg.get("fences", {}).get("planes", ()))
        for name, pt, n in fences:          # a mounted arm must stand on the free side
            for slot, m in mounts.items():
                axis = np.array([m.axis_xy_table[0], m.axis_xy_table[1], pt[2]])
                if n @ (axis - pt) <= 0.0:
                    raise ValueError(f"rig.json: slot {slot} stands behind fence {name}")
        num = lambda d: {k: float(v) for k, v in d.items() if k.endswith("_m")}
        canvas = cfg["canvas"]
        return Rig(
            mounts=mounts, slot_names=tuple(a["slot"] for a in cfg["slots"]["list"]),
            steel=tuple(cage + hangers),
            clearance=num(cfg["clearances"]), allowance=num(cfg["planning_allowance"]),
            gate_cfg={k: float(v) for k, v in cfg["gates"].items() if not isinstance(v, str)},
            drawing_cfg={k: float(v) for k, v in cfg["drawing"].items()
                         if not isinstance(v, str)},
            execution_cfg=Execution(float(cfg["execution"]["start_tolerance_rad"])),
            pen_name=pen_name, pen_cfg=pen,
            shoulder_below_base=float(cfg["reach"]["shoulder_below_base_m"]),
            body_reach=float(cfg["reach"]["body_reach_from_shoulder_m"]),
            own_hanger_exempt=tuple(cfg["hanger"]["exempt_links"]),
            rows=tuple(here(r) for r in rp["rows"] if len(here(r)) == 2),
            leader_sets={int(k): here(v) for k, v in rp["leaders"].items()},
            wall_pairs={int(k): tuple((a, b) for a, b in v if len(here((a, b))) == 2)
                        for k, v in rp["walls"].items()},
            fences=fences,
            canvas_size=np.array([canvas["size_x_m"], canvas["size_y_m"]]),
            drawing_area_m=(None if "drawing_area_m" not in canvas
                            else np.asarray(canvas["drawing_area_m"], float).reshape(2)),
            drawing_area_centre_m=np.asarray(canvas.get("drawing_area_centre_m", [0.0, 0.0]),
                                             float).reshape(2),
            table_size=np.array([cfg["table"]["size_x_m"], cfg["table"]["size_y_m"]]),
            paper_z=float(cfg["table"]["paper_surface_z_m"]),
            **dict(zip(("marks", "mark_groups", "mark_files"),
                       _marks(cfg, config_dir, tuple(a["slot"] for a in cfg["slots"]["list"])))),
        )

    # ------------------------------------------------------------------ arms and frames

    @property
    def arm_ids(self) -> tuple[Slot, ...]:
        """The mounted slots, in rig.json order."""
        return tuple(self.mounts)

    def _mount(self, slot: Slot) -> Mount:
        if slot not in self.mounts:
            hint = "" if is_slot(slot) else " (arms are named by slot now, e.g. '2R')"
            raise KeyError(f"no arm in slot {slot!r}{hint}; mounted slots are {self.arm_ids}")
        return self.mounts[slot]

    def T_table_base(self, slot: Slot) -> np.ndarray:
        return self._mount(slot).T_table_base.copy()

    def T_base_table(self, slot: Slot) -> np.ndarray:
        T = self._mount(slot).T_table_base
        out = np.eye(4)
        out[:3, :3] = T[:3, :3].T
        out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
        return out

    def park_q(self, slot: Slot) -> np.ndarray:
        return self._mount(slot).park_q.copy()

    def calibration_status(self, slot: Slot) -> dict:
        """{"base": s, "pen": s}, each "none" (no such part), "applied: ..." or
        "<part> part not applied: <why>"."""
        return dict(self._mount(slot).calibration)

    def calibrated(self, slot: Slot) -> bool:
        """Both parts of the slot's calibration applied."""
        return all(s.startswith("applied") for s in self._mount(slot).calibration.values())

    def arm(self, slot: Slot):
        """The arm model with this slot's tool (the measured pen tip when the pen part applies,
        else the current pen's nominal length)."""
        return Arm(self._mount(slot).tool)

    def nominal_pose(self, slot: Slot) -> np.ndarray:
        """(4, 4) the slot's pose from rig.json alone, before any calibration: what the
        calibration solvers measure their refusals against."""
        return self._mount(slot).T_nominal.copy()

    def nominal_tool(self) -> Tool:
        """The holder with the current pen at its nominal length, before any calibration."""
        return _pen_tool(self.pen_cfg, None)

    def nominal_tip(self) -> np.ndarray:
        """(3,) the current pen's nominal tip in the hand frame."""
        return self.nominal_tool().tip_hand.copy()

    def to_base(self, slot: Slot, line: Line) -> Line:
        if line.frame != "table":
            raise ValueError(f"line {line.id} is in the {line.frame} frame, not the table frame")
        T = self.T_base_table(slot)
        p = np.asarray(line.points, float) @ T[:3, :3].T + T[:3, 3]
        return Line(line.id, p, "base", line.intensity)

    def to_table(self, slot: Slot, points_base) -> np.ndarray:
        T = self._mount(slot).T_table_base
        return np.asarray(points_base, float) @ T[:3, :3].T + T[:3, 3]

    def _plane_in_base(self, slot, name, n_table, point_table, margin, kind, pen_margin):
        T = self.T_base_table(slot)
        n = T[:3, :3] @ n_table
        p = T[:3, :3] @ point_table + T[:3, 3]
        return Plane(name, n, float(n @ p), margin, kind, pen_margin)

    def paper(self, slot: Slot, for_planning: bool = False) -> Plane:
        """The paper surface in this arm's base frame; free side is up, toward the arm.  Three
        margins: the links (`margin`), the lifted pen (`pen_margin`), the rest of the tool:
        gripper, blades, holder (`tool_margin`)."""
        c, a = self.clearance, self.allowance
        body = c["body_to_paper_m"] + (a["body_to_paper_m"] if for_planning else 0.0)
        pen = c["pen_lifted_to_paper_m"] + (a["pen_lifted_to_paper_m"] if for_planning else 0.0)
        tool = c["tool_to_paper_m"] + (a["tool_to_paper_m"] if for_planning else 0.0)
        plane = self._plane_in_base(slot, "paper", np.array([0.0, 0.0, 1.0]),
                                    np.array([0.0, 0.0, self.paper_z]), body, "paper", pen)
        return replace(plane, tool_margin=tool)

    def self_margin(self, for_planning: bool = False) -> float:
        return self.clearance["self_m"] + (self.allowance["self_m"] if for_planning else 0.0)

    def gates(self) -> Gates:
        """The gates a configuration must pass, all from rig.json.  The self margin is the
        demanded self clearance plus its planning allowance."""
        g = self.gate_cfg
        return Gates(limit_margin=g["limit_margin_rad"], sigma_min=g["sigma_min"],
                     self_margin=self.self_margin(for_planning=True))

    def execution(self) -> Execution:
        return self.execution_cfg

    def rules(self) -> DrawRules:
        """The drawing rules, all from rig.json, with `gates()` inside; the press and the speed
        on the paper are the current pen's.  The one source every planner uses."""
        d, p = self.drawing_cfg, self.pen_cfg
        return DrawRules(draw_speed=float(p["speed_m_per_s"]), press=float(p["press_m"]),
                         landing_speed=d.get("landing_speed_m_per_s", 0.010),
                         lean_max=float(np.deg2rad(self.gate_cfg["pen_lean_max_deg"])),
                         min_piece=d["min_piece_m"], speed_fraction=d["speed_fraction"],
                         gates=self.gates())

    AT_PARK_RAD = 1e-6                 # a configuration this close to the park counts as parked

    def at_park(self, slot: str, q) -> bool:
        """The arm stands at its park configuration (to the planning tolerance; the executor's
        start tolerance, rig.json `execution`, is a different, larger number)."""
        return float(np.max(np.abs(np.asarray(q, float) - self.park_q(slot)))) <= self.AT_PARK_RAD

    def pen(self) -> dict:
        """The pen that is in, as plain data: its pens.table entry plus "name".  The server
        copies it into every job header."""
        return dict(self.pen_cfg, name=self.pen_name)

    # ------------------------------------------------------------------ walls

    def wall_between(self, arm_a: Slot, arm_b: Slot) -> Wall:
        """Halfway between the two arms' nominal axes, square to the line joining them."""
        pa, pb = self._mount(arm_a).axis_xy_table, self._mount(arm_b).axis_xy_table
        d = pb - pa
        if np.linalg.norm(d) < 1e-9:
            raise ValueError(f"arms {arm_a} and {arm_b} share an axis")
        n = np.array([d[0], d[1], 0.0]) / np.linalg.norm(d)
        mid = 0.5 * (pa + pb)
        return Wall(f"wall_{arm_a}_{arm_b}", (arm_a, arm_b),
                    np.array([mid[0], mid[1], self.paper_z]), n)

    def wall_in_base(self, slot: Slot, wall: Wall, for_planning: bool = False) -> Plane:
        """The wall in this arm's base frame, free side = the side this arm's axis is on."""
        axis = self._mount(slot).axis_xy_table
        side = float(wall.normal_table[:2] @ (axis - wall.point_table[:2]))
        if abs(side) < 1e-9:
            raise ValueError(f"arm {slot} stands on {wall.name}")
        n = wall.normal_table * np.sign(side)
        margin = self.clearance["wall_m"] + (self.allowance["wall_m"] if for_planning else 0.0)
        return self._plane_in_base(slot, wall.name, n, wall.point_table, margin, "wall", None)

    # ------------------------------------------------------------------ obstacles

    def shoulder_table(self, slot: Slot) -> np.ndarray:
        T = self._mount(slot).T_table_base
        return T[:3, 3] + self.shoulder_below_base * T[:3, 2]

    def steel_for(self, slot: Slot, margin: float) -> tuple[list[SteelBox], list[SteelBox]]:
        """-> (boxes this arm must clear, boxes out of its reach).  A box is out of reach when
        it is further from the shoulder than the body reaches plus the clearance demanded."""
        s = self.shoulder_table(slot)
        near, far = [], []
        for b in self.steel:
            d = _point_box_distance(s, b.lo_table, b.hi_table)
            (far if d > self.body_reach + margin else near).append(b)
        return near, far

    def parked_capsules(self, slot: Slot, parked_id: Slot, for_planning: bool = True):
        """The body of `parked_id` standing at its park configuration, in `slot`'s base frame."""
        if parked_id == slot:
            raise ValueError(f"arm {slot} cannot be parked next to itself")
        body = self.arm(parked_id).body(self.park_q(parked_id)[None, :])
        T = self.T_base_table(slot) @ self._mount(parked_id).T_table_base
        R, t = T[:3, :3], T[:3, 3]
        m = self.clearance["arm_to_arm_m"] + (self.allowance["arm_to_arm_m"] if for_planning
                                              else 0.0)
        caps = [Capsule(f"parked{parked_id}:{name}", R @ body.p0[0, k] + t,
                        R @ body.p1[0, k] + t, float(body.radius[k]), m)
                for k, name in enumerate(body.names)]
        return tuple(caps)

    def obstacles(self, slot: Slot, parked=(), walls=(), for_planning: bool = True) -> Obstacles:
        """Everything `slot` must stay clear of, in its base frame: the paper, the steel it
        can reach, the given walls, and the bodies of the given parked arms."""
        steel_m = self.clearance["steel_m"] + (self.allowance["steel_m"] if for_planning else 0.0)
        near, _ = self.steel_for(slot, steel_m)
        T = self.T_base_table(slot)
        boxes = []
        for b in near:
            Tb = T.copy()
            Tb[:3, 3] = T[:3, :3] @ (0.5 * (b.lo_table + b.hi_table)) + T[:3, 3]
            exempt = self.own_hanger_exempt if b.owner == slot else ()
            boxes.append(Box(b.name, Tb, 0.5 * (b.hi_table - b.lo_table), steel_m, exempt))
        planes = [self.paper(slot, for_planning)]
        planes += [self.wall_in_base(slot, w, for_planning) for w in walls]
        # fences: walls that are always there, whatever the phase (e.g. toward the hangers of
        # arms that are present but not controlled)
        wall_m = self.clearance["wall_m"] + (self.allowance["wall_m"] if for_planning else 0.0)
        planes += [self._plane_in_base(slot, name, n, pt, wall_m, "wall", None)
                   for name, pt, n in self.fences]
        caps = [c for p in parked for c in self.parked_capsules(slot, p, for_planning)]
        return Obstacles(tuple(boxes), tuple(planes), tuple(caps))

    # ------------------------------------------------------------------ roles

    def leaders(self, phase: int) -> tuple[Slot, ...]:
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

    def _parked_in_reach(self, slot: Slot, parked_id: Slot, margin: float) -> bool:
        """Can any part of `slot` come within `margin` of `parked_id` standing parked?"""
        caps = self.parked_capsules(slot, parked_id)
        s = np.array([0.0, 0.0, self.shoulder_below_base])      # shoulder, base frame
        for c in caps:
            d = c.p1 - c.p0
            t = np.clip((s - c.p0) @ d / max(d @ d, 1e-18), 0.0, 1.0)
            if np.linalg.norm(c.p0 + t * d - s) - c.radius <= self.body_reach + margin:
                return True
        return False

    def obstacles_for(self, slot: Slot, phase: Phase, for_planning: bool = True) -> Obstacles:
        """`obstacles` for an arm that moves in `phase`: its parked row partner, any other parked
        arm it can reach, and the walls it stands next to."""
        if slot not in phase.active:
            raise ValueError(f"arm {slot} does not move in {phase.name}")
        m = self.clearance["arm_to_arm_m"] + (self.allowance["arm_to_arm_m"] if for_planning
                                              else 0.0)
        partner = self.row_partner(slot)
        parked = tuple(p for p in phase.parked
                       if p == partner or self._parked_in_reach(slot, p, m))
        walls = tuple(w for w in phase.walls if slot in w.arms)
        return self.obstacles(slot, parked, walls, for_planning)

    def row_partner(self, slot: Slot) -> Slot | None:
        """The other arm of this arm's row; None when that arm is not mounted."""
        for a, b in self.rows:
            if slot in (a, b):
                return b if slot == a else a
        return None

    # ------------------------------------------------------------------ calibration marks

    def marks_for(self, slots) -> tuple[str, ...]:
        """The marks both of whose sharers are in `slots`, in rig.json order."""
        return tuple(n for n, (_, share) in self.marks.items() if set(share) <= set(slots))

    def mark_state(self, name: str) -> str:
        """"solved" or "nominal" (also when calibration/marks.json does not name it)."""
        if name not in self.marks:
            raise KeyError(f"no mark {name!r}; marks are {tuple(self.marks)}")
        return self.mark_files.get(name, {}).get("state", "nominal")

    def mark_xy(self, name: str) -> np.ndarray:
        """(2,) table frame: the solved position when solved, else the nominal one."""
        if self.mark_state(name) == "solved":
            return self.mark_files[name]["xy_m"].copy()
        return self.marks[name][0].copy()
