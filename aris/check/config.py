"""The checker's own reader of `config/rig.json` and `config/calibration/<arm>.json`.

Written apart from `aris/rig.py` on purpose: the rig is data, the reading is code, and the
checker shares data with the planners but no code.  Everything here is in the table frame.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ArmMount:
    arm_id: int
    T_table_base: np.ndarray       # (4,4), the calibrated pose when a passed calibration exists
    park_q: np.ndarray             # (7,)
    tip_hand: np.ndarray | None    # calibrated pen tip in the hand frame, or None
    calibration: str


@dataclass(frozen=True)
class RigData:
    mounts: dict                   # arm_id -> ArmMount
    box_names: tuple               # steel, table frame, axis aligned
    box_owner: tuple               # per box: the arm hanging from it, or None
    own_exempt: tuple              # body names (e.g. "link1") not checked against the arm's own
                                   # hanger boxes (rig.json hanger.exempt_links)
    box_lo: np.ndarray             # (B,3)
    box_hi: np.ndarray             # (B,3)
    clearance: dict                # the demanded clearances, metres
    paper_z: float
    draw_speed: float              # m/s, rig.json drawing.draw_speed_m_per_s
    notes: tuple = ()              # anything the verdict should say about how rig.json was read


def read_rig(config_dir) -> RigData:
    """Raises ValueError on a file that does not describe a rig (a broken install)."""
    config_dir = Path(config_dir)
    cfg = json.loads((config_dir / "rig.json").read_text())
    mounts = {}
    names, lo, hi, owner = [], [], [], []
    for a in cfg["arms"]["list"]:
        aid = int(a["id"])
        T = np.eye(4)
        T[:3, :3] = np.asarray(a["R_table_base"], float)
        T[:3, 3] = [a["axis_xy_m"][0], a["axis_xy_m"][1], a["base_z_m"]]
        _need_rigid(T, f"arm {aid} in rig.json")
        T, tip, status = _calibration(config_dir / "calibration" / f"{aid}.json", aid, T)
        mounts[aid] = ArmMount(aid, T, np.asarray(a["park_q_rad"], float), tip, status)
        for n, l, h in _hanger(aid, a, cfg["hanger"]):
            names.append(n), lo.append(l), hi.append(h), owner.append(aid)
    for b in cfg["steel"]["boxes"]:
        names.append(b["name"]), lo.append(b["lo_m"]), hi.append(b["hi_m"]), owner.append(None)
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if np.any(hi < lo):
        raise ValueError("rig.json: a steel box has hi below lo")
    clearance = {k: float(v) for k, v in cfg["clearances"].items() if k.endswith("_m")}
    notes = []
    if "tool_to_paper_m" not in clearance:
        clearance["tool_to_paper_m"] = clearance["body_to_paper_m"]
        notes.append("rig.json has no clearances.tool_to_paper_m: the tool keeps "
                     "body_to_paper_m")
    return RigData(mounts, tuple(names), tuple(owner),
                   tuple(cfg["hanger"].get("exempt_links", ())), lo, hi, clearance,
                   float(cfg["table"]["paper_surface_z_m"]),
                   float(cfg["drawing"]["draw_speed_m_per_s"]), tuple(notes))


def _need_rigid(T, what):
    R = T[:3, :3]
    if not (np.allclose(R @ R.T, np.eye(3), atol=1e-6) and np.linalg.det(R) > 0
            and np.allclose(T[3], [0, 0, 0, 1])):
        raise ValueError(f"{what}: not a rigid transform")


def _calibration(path: Path, arm_id: int, T_nominal):
    if not path.exists():
        return T_nominal, None, "none"
    cal = json.loads(path.read_text())
    if int(cal["arm_id"]) != arm_id:
        raise ValueError(f"{path}: written for arm {cal['arm_id']}")
    if cal.get("passed") is not True:
        return T_nominal, None, f"not used: {path.name} did not pass"
    T = np.asarray(cal["T_table_base"], float).reshape(4, 4)
    _need_rigid(T, str(path))
    tip = cal.get("tip_hand_m")
    return T, None if tip is None else np.asarray(tip, float).reshape(3), f"used: {path.name}"


def _hanger(aid, arm, h):
    """The two struts, the plate and the clamp an arm hangs from, as table-frame boxes.

    From the words in rig.json: struts of section strut_size_x by strut_size_y, centred on the
    arm's row line; the outer face of one is `wide_side_outer_face_to_axis` from the axis on the
    side `strut_wide_side` names, the other's `narrow_side_outer_face_to_axis` on the other side;
    each strut extends from its outer face back toward the axis.  Plate and clamp are centred
    `plate_centre_toward_wide_side` from the axis toward the wide side, on the row line.
    """
    x, y = float(arm["axis_xy_m"][0]), float(arm["axis_xy_m"][1])
    side = {"-x": -1.0, "+x": 1.0}.get(arm["strut_wide_side"])
    if side is None:
        raise ValueError(f"arm {aid}: strut_wide_side {arm['strut_wide_side']!r}")
    out = []
    for tag, dist, sgn in (("wide", h["wide_side_outer_face_to_axis_m"], side),
                           ("narrow", h["narrow_side_outer_face_to_axis_m"], -side)):
        outer = x + sgn * dist
        inner = outer - sgn * h["strut_size_x_m"]
        out.append((f"strut{aid}_{tag}", [min(outer, inner), y - h["strut_size_y_m"] / 2,
                                          h["strut_bottom_z_m"]],
                    [max(outer, inner), y + h["strut_size_y_m"] / 2, h["strut_top_z_m"]]))
    cx = x + side * h["plate_centre_toward_wide_side_m"]
    for tag in ("plate", "clamp"):
        sx, sy = h[f"{tag}_size_x_m"] / 2, h[f"{tag}_size_y_m"] / 2
        out.append((f"{tag}{aid}", [cx - sx, y - sy, h[f"{tag}_bottom_z_m"]],
                    [cx + sx, y + sy, h[f"{tag}_top_z_m"]]))
    return out
