"""The checker's own reader of `config/rig.json` and `config/calibration/<slot>.json`.

Written apart from `aris/rig.py` on purpose: the rig is data, the reading is code, and the
checker shares data with the planners but no code.  Everything here is in the table frame.
Arms are named by their slot on the frame ("1L" .. "3R").
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Tolerances:
    """The checker's own numerical allowances (rig.json `checker`; these defaults when the block
    is missing).  None of them relaxes a limit of the rig; each says how finely it is judged."""
    step: float = 1e-3            # m, most any capsule point moves between two clearance samples
    tol: float = 2.5e-4           # m, how far under the true minimum a reported clearance may lie
    rate_tol: float = 0.05        # 1 kHz vs 4 kHz readings may differ this much: catches corners
    tip_height_tol: float = 5e-4  # m, tip off the drawing surface (draw; lower/lift/touch ends)
    line_tol: float = 2e-4        # m, tip off the planned line while drawing
    back_tol: float = 1e-5        # m, numerical allowance on "the pen never goes backwards"
    speed_tol: float = 0.03       # tip may run this share over the pen's speed (timing overshoot)
    stop_speed: float = 2.5e-4    # m/s, slower than this mid-line along the line is a stop


_TOL_KEYS = dict(step="step_m", tol="clearance_tol_m", rate_tol="rate_tol",
                 tip_height_tol="tip_height_tol_m", line_tol="line_tol_m",
                 back_tol="back_tol_m", speed_tol="speed_tol",
                 stop_speed="stop_speed_m_per_s")


def _tolerances(cfg) -> Tolerances:
    block = cfg.get("checker", {})
    return Tolerances(**{f: float(block[k]) for f, k in _TOL_KEYS.items() if k in block})


@dataclass(frozen=True)
class SlotMount:
    slot: str
    T_table_base: np.ndarray       # (4,4), the calibrated pose when the base part passed
    park_q: np.ndarray             # (7,)
    tip_hand: np.ndarray | None    # calibrated pen tip in the hand frame, or None (nominal)
    calibration: str               # what was used, in words
    notes: tuple = ()              # what fell back to nominal, and why


@dataclass(frozen=True)
class RigData:
    mounts: dict                   # slot -> SlotMount (mounted slots only)
    box_names: tuple               # steel, table frame, axis aligned
    box_owner: tuple               # per box: the slot hanging from it, or None
    own_exempt: tuple              # body names (e.g. "link1") not checked against the arm's own
                                   # hanger boxes (rig.json hanger.exempt_links)
    box_lo: np.ndarray             # (B,3)
    box_hi: np.ndarray             # (B,3)
    clearance: dict                # the demanded clearances, metres
    paper_z: float                 # the real paper: what links, tool and lifted pen clear
    draw_speed: float              # m/s on the paper, the current pen's speed_m_per_s
    press: float                   # m, the drawing surface lies this far below the paper
    pen: str                       # the pen that is in (rig.json pens.current)
    pen_length: float              # m, its nominal length (where no measured tip applies)
    pen_radius: float              # m, its capsule's radius
    notes: tuple = ()              # anything the verdict should say about how rig.json was read
    fences: tuple = ()             # (name, point, unit normal), table frame: planes every arm
                                   # stays on the normal's side of, in every phase (rig.json)
    tolerances: Tolerances = Tolerances()   # rig.json `checker`

    @property
    def surface_z(self) -> float:
        """Where the drawing's points lie: the paper less the press."""
        return self.paper_z - self.press


def read_rig(config_dir) -> RigData:
    """Raises ValueError (or KeyError) on a file that does not describe a rig (a broken
    install); the callers turn that into a failed verdict."""
    config_dir = Path(config_dir)
    cfg = json.loads((config_dir / "rig.json").read_text())
    pen_name, pen, notes = _current_pen(cfg)
    mounts = {}
    names, lo, hi, owner = [], [], [], []
    for a in cfg["slots"]["list"]:
        slot = str(a["slot"])
        T_nom = np.eye(4)
        T_nom[:3, :3] = np.asarray(a["R_table_base"], float)
        T_nom[:3, 3] = [a["axis_xy_m"][0], a["axis_xy_m"][1], a["base_z_m"]]
        _need_rigid(T_nom, f"slot {slot} in rig.json")
        T = T_nom
        if a.get("mounted", True):
            T, tip, used, why = _calibration(config_dir / "calibration" / f"{slot}.json",
                                             slot, T_nom, pen_name)
            mounts[slot] = SlotMount(slot, T, np.asarray(a["park_q_rad"], float), tip,
                                     used, why)
        # The hanger is on the frame whether or not an arm hangs from it; the arm is bolted to
        # its plate, so the hanger follows the calibrated axis across the table (shifted, not
        # turned: the boxes stay axis aligned, and a few mrad of turn moves a plate corner well
        # under a millimetre).  Its heights are the frame's, as rig.json gives them.
        # A slot without a hanger ("hanger": false, e.g. a floor arm or an empty slot) has no
        # steel; an arm hanging there always has one.
        if not a.get("mounted", True) and not a.get("hanger", True):
            continue
        shift = np.r_[T[:2, 3] - T_nom[:2, 3], 0.0]
        for n, l, h in _hanger(slot, a, cfg["hanger"]):
            names.append(n), lo.append(np.add(l, shift)), hi.append(np.add(h, shift))
            owner.append(slot)
    for b in cfg["steel"]["boxes"]:
        names.append(b["name"]), lo.append(b["lo_m"]), hi.append(b["hi_m"]), owner.append(None)
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    if np.any(hi < lo):
        raise ValueError("rig.json: a steel box has hi below lo")
    clearance = {k: float(v) for k, v in cfg["clearances"].items() if k.endswith("_m")}
    if "tool_to_paper_m" not in clearance:
        clearance["tool_to_paper_m"] = clearance["body_to_paper_m"]
        notes.append("rig.json has no clearances.tool_to_paper_m: the tool keeps "
                     "body_to_paper_m")
    speed = pen.get("speed_m_per_s")
    if speed is None:
        speed = cfg["drawing"]["draw_speed_m_per_s"]
        notes.append(f"pen {pen_name} has no speed_m_per_s: drawing speed from "
                     f"drawing.draw_speed_m_per_s ({float(speed) * 1e3:.1f} mm/s)")
    press = float(pen["press_m"])
    if not 0.0 <= press < 0.02:
        raise ValueError(f"rig.json: pen {pen_name} press_m {press} is not a press")
    fences = []
    for f in cfg.get("fences", {}).get("planes", ()):
        n = np.asarray(f["normal"], float)
        fences.append((str(f["name"]), np.asarray(f["point_m"], float), n / np.linalg.norm(n)))
    return RigData(mounts, tuple(names), tuple(owner),
                   tuple(cfg["hanger"].get("exempt_links", ())), lo, hi, clearance,
                   float(cfg["table"]["paper_surface_z_m"]), float(speed), press, pen_name,
                   float(pen["tip_length_nominal_m"]), float(pen["capsule_radius_m"]),
                   tuple(notes), tuple(fences), _tolerances(cfg))


def _current_pen(cfg):
    """(name, the pen's entry, notes): rig.json `pens.current` names the pen that is in,
    `pens.table` holds every pen by name."""
    pens = cfg["pens"]
    name = str(pens["current"])
    table = pens["table"]
    if name not in table:
        raise ValueError(f"rig.json: pens.current names {name!r}, which is not in the table")
    return name, table[name], []


def _need_rigid(T, what):
    R = T[:3, :3]
    if not (np.allclose(R @ R.T, np.eye(3), atol=1e-6) and np.linalg.det(R) > 0
            and np.allclose(T[3], [0, 0, 0, 1])):
        raise ValueError(f"{what}: not a rigid transform")


def _calibration(path: Path, slot: str, T_nominal, pen_name):
    """-> (T_table_base, tip_hand or None, what was used, notes).  Each part is applied on its
    own: `base` when it passed, `pen` when it passed and names the pen that is in."""
    if not path.exists():
        return T_nominal, None, "nominal", (f"{slot}: no calibration file, base and pen tip "
                                            f"nominal",)
    cal = json.loads(path.read_text())
    if str(cal.get("slot")) != slot:
        raise ValueError(f"{path}: written for slot {cal.get('slot')!r}")
    T, tip, used, notes = T_nominal, None, [], []
    base = cal.get("base") or {}
    if base.get("passed") is True:
        T = np.asarray(base["T_table_base"], float).reshape(4, 4)
        _need_rigid(T, f"{path} base")
        used.append(f"base {base.get('method', '')} {base.get('date', '')}".strip())
    else:
        notes.append(f"{slot}: base nominal ({_why(base, 'base')})")
    pen = cal.get("pen") or {}
    if pen.get("passed") is True and str(pen.get("pen")) == pen_name:
        tip = np.asarray(pen["tip_hand_m"], float).reshape(3)
        used.append(f"pen {pen_name} {pen.get('date', '')}".strip())
    elif pen.get("passed") is True:
        notes.append(f"{slot}: pen tip nominal (touched off with {pen.get('pen')}, "
                     f"the pen in is {pen_name})")
    else:
        notes.append(f"{slot}: pen tip nominal ({_why(pen, 'pen')})")
    return T, tip, "; ".join(used) or "nominal", tuple(notes)


def _why(part, name):
    if not part:
        return f"no {name} part"
    return "did not pass" + (f": {part['why']}" if part.get("why") else "")


def _hanger(slot, arm, h):
    """The two struts, the plate and the clamp a slot's arm hangs from, as table-frame boxes
    around its nominal axis (the caller shifts them to the calibrated one).

    From the words in rig.json: struts of section strut_size_x by strut_size_y, centred on the
    arm's row line; one has its outer face `axis_to_minus_x_outer_face` toward table -x of the
    axis, the other `axis_to_plus_x_outer_face` toward +x; each extends from its outer face
    back toward the axis.  Plate and clamp are centred `plate_centre_offset_x` toward table +x
    of the axis, on the row line.  The same for every slot (one clocking).
    """
    x, y = float(arm["axis_xy_m"][0]), float(arm["axis_xy_m"][1])
    out = []
    for tag, sign in (("minus_x", -1.0), ("plus_x", 1.0)):
        outer = x + sign * h[f"axis_to_{tag}_outer_face_m"]
        inner = outer - sign * h["strut_size_x_m"]
        out.append((f"strut{slot}_{tag}", [min(outer, inner), y - h["strut_size_y_m"] / 2,
                                           h["strut_bottom_z_m"]],
                    [max(outer, inner), y + h["strut_size_y_m"] / 2, h["strut_top_z_m"]]))
    cx = x + h["plate_centre_offset_x_m"]
    for tag in ("plate", "clamp"):
        sx, sy = h[f"{tag}_size_x_m"] / 2, h[f"{tag}_size_y_m"] / 2
        out.append((f"{tag}{slot}", [cx - sx, y - sy, h[f"{tag}_bottom_z_m"]],
                    [cx + sx, y + sy, h[f"{tag}_top_z_m"]]))
    return out
