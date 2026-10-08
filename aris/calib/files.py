"""The calibration files: `config/calibration/<slot>.json`, one per slot, in two parts.

`base` is written by the plane job (`write_base`), `pen` by the touch-off (`write_pen`); each
writer rewrites only its own part and keeps the other as it was.  `aris/rig.py` reads the files
for everything else; the server hands them out with `read` and `listing`.  Only rig.py and this
package name paths inside config/.  The layout is in docs/BUILD.md ("The calibration file").
"""
from __future__ import annotations

import datetime
import hashlib
import json
import os
from pathlib import Path

import numpy as np

from aris.calib.marks import MarkSolution, SlotFit, pen_from_pivot
from aris.calib.plane import PlaneCalibration
from aris.calib.pen import PenCalibration
from aris.types import Slot

PLANE_CONVENTION = (
    "rotation = R(v) @ the rotation the rig had, v = (roll, pitch, 0) a rotation vector in the "
    "table frame (the smallest turn, about a horizontal axis through the base origin); base "
    "table z moved by height_change; x, y and the turn about the vertical are unchanged. The "
    "height includes the length error of the pen named here against the tip named here.")


MARKS_CONVENTION = (
    "rotation = Rz(yaw) @ the rotation the rig had (a turn about the table's vertical through "
    "the base origin); x, y solved; z, roll and pitch as before (the plane job), z moved by the "
    "hand-z part of the tip change when the solver was given the tip the plane was measured "
    "with. Frame: the solved slot axes fitted rigidly onto the nominal mountings (or marks "
    "solved before held it); see `frame` in calibration/marks.json.")


def calibration_file(config_dir, slot: Slot) -> Path:
    """config/calibration/<slot>.json under `config_dir`."""
    return Path(config_dir) / "calibration" / f"{slot}.json"


def _mm(x):
    return None if x is None or not np.isfinite(x) else round(float(x) * 1e3, 4)


def _deg(x):
    return None if not np.isfinite(x) else round(float(np.rad2deg(x)), 5)


def _list(a, nd=9):
    return None if a is None else np.round(np.asarray(a, float), nd).tolist()


def base_part(r: PlaneCalibration, date: str | None = None) -> dict:
    """The `base` part for a plane result: what the rig reads plus what a person wants."""
    return {
        "passed": bool(r.passed), "date": date or datetime.date.today().isoformat(),
        "method": "plane", "why": r.why,
        "T_table_base": _list(r.T_table_base, 12),
        "T_table_base_before": _list(r.T_table_base_nominal, 12),
        "convention": PLANE_CONVENTION,
        "measured_with": {"pen": r.pen, "tip_hand_m": _list(r.tip_hand)},
        "roll_deg": _deg(r.roll), "pitch_deg": _deg(r.pitch), "tilt_deg": _deg(r.tilt),
        "height_change_mm": _mm(r.height_change),
        "residuals": {"n_points": int(r.n_points), "rms_mm": _mm(r.rms),
                      "max_mm": _mm(r.max_residual), "worst_point": int(r.worst_index)},
        "height_map_table_m": _list(r.points_table, 7),
    }


def pen_part(r: PenCalibration, date: str | None = None) -> dict:
    """The `pen` part for a touch-off result."""
    return {
        "passed": bool(r.passed), "date": date or datetime.date.today().isoformat(),
        "pen": r.pen, "why": r.why, "method": r.method,
        "tip_hand_m": _list(r.tip_hand),
        "reference_touch": (None if r.reference_xy_table is None else
                            {"xy_table_m": _list(r.reference_xy_table), "q": _list(r.q, 12)}),
        "tip_hand_nominal_m": _list(r.tip_hand_nominal),
        "tip_hand_before_m": _list(r.tip_hand_before),
        "correction_mm": _mm(r.correction), "change_mm": _mm(r.change),
        "height_before_mm": _mm(r.height_before),
        "touch_xy_table_m": _list(r.touch_xy_table),
        "from_reference_mm": _mm(r.from_reference),
        "against_base": r.base_status,
        **({} if r.detail is None else {r.method: r.detail}),
    }


def _write_json(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=1) + "\n")
    os.replace(tmp, path)              # a reader never sees half a file
    return path


def _write_part(config_dir, slot: Slot, name: str, make, robot: str | None = None) -> Path:
    """Read the slot's file (if any), replace one part with `make(old part or None)`, write it
    back whole.  A file that is not this slot's calibration file is a broken install: raise
    rather than lose the other part."""
    path = calibration_file(config_dir, slot)
    cal = {"slot": slot}
    if path.exists():
        cal = json.loads(path.read_text())
        if not isinstance(cal, dict) or cal.get("slot") != slot:
            raise ValueError(f"{path} is not the calibration file of slot {slot}")
    part = make(cal.get(name))
    if robot is not None:
        part["robot"] = str(robot)          # the server refuses a part measured on another robot
    cal[name] = part
    return _write_json(path, cal)


def marks_base_part(r: SlotFit, date: str | None = None, before: dict | None = None) -> dict:
    """The `base` part for one slot of a mark solution.  The plane job's numbers (where z, roll
    and pitch came from) are kept under "plane"."""
    plane = None
    if isinstance(before, dict):
        plane = before if before.get("method") == "plane" else before.get("plane")
    return {
        "passed": True, "date": date or datetime.date.today().isoformat(),
        "method": r.method, "why": "", "yaw_from": r.yaw_from,
        "T_table_base": _list(r.T_table_base, 12),
        "T_table_base_before": _list(r.T_before, 12),
        "convention": MARKS_CONVENTION,
        "measured_with": {"pen": r.pen, "tip_hand_m": _list(r.tip_hand)},
        "shift_from_nominal_mm": _mm(r.shift), "yaw_from_nominal_mrad": round(r.yaw * 1e3, 5),
        "residuals": {"n_points": int(r.n_touches), "rms_mm": _mm(r.rms)},
        "pivot": None if r.pivot is None else {
            "mark": r.pivot_mark, "spread_deg": _deg(r.pivot.spread),
            "residuals_mm": _list(r.pivot.residuals * 1e3, 4)},
        "plane": plane,
    }


def write_base(result, config_dir, date: str | None = None, robot: str | None = None) -> Path:
    """Write a plane result (`PlaneCalibration`) or one slot of a mark solution (`SlotFit`) as
    the `base` part of its slot's file; the `pen` part stays.  `robot` (e.g. "fr3-71"), when
    given, is written into the part."""
    if isinstance(result, SlotFit):
        return _write_part(config_dir, result.slot, "base",
                           lambda old: marks_base_part(result, date, old), robot)
    return _write_part(config_dir, result.slot, "base", lambda old: base_part(result, date),
                       robot)


def write_marks(solution: MarkSolution, config_dir, date: str | None = None) -> Path:
    """`calibration/marks.json`: every mark this solution solved, as {"marks": {name: {xy_m,
    state "solved", date, from_nominal_mm, residual_mm}}}; marks it took as known, and marks it
    did not touch, stay as they were.  Only a passing solution is written."""
    if not solution.passed:
        raise ValueError(f"a refused mark solution is not written: {solution.why}")
    path = Path(config_dir) / "calibration" / "marks.json"
    data = json.loads(path.read_text()) if path.exists() else {}
    marks = data.get("marks", {}) if isinstance(data.get("marks"), dict) else {}
    day = date or datetime.date.today().isoformat()
    for name, m in solution.marks.items():
        if m.state == "known":
            continue
        marks[name] = {"xy_m": _list(m.xy, 9), "state": "solved", "date": day,
                       "from_nominal_mm": _list(m.from_nominal * 1e3, 3),
                       "residual_mm": _mm(m.residual), "by": list(m.by),
                       **({"note": m.note} if m.note else {})}
    return _write_json(path, {"frame": solution.frame, "marks": marks})


def write_mark_solution(rig, solution: MarkSolution, config_dir, date: str | None = None,
                        robot: dict | None = None) -> list[Path]:
    """The mark job's one writer: every slot's base part (method "marks" or "meetings"), every
    slot's pen part when a pivot measured its tip, and marks.json when marks were solved.
    `robot`: {slot: robot} written into each slot's parts."""
    if not solution.passed:
        raise ValueError(f"a refused mark solution is not written: {solution.why}")
    robot = robot or {}
    out = []
    for slot, f in solution.slots.items():
        out.append(write_base(f, config_dir, date, robot.get(slot)))
        if f.pivot is not None:
            out.append(write_pen(pen_from_pivot(rig, slot, f.pivot, f.pivot_mark, f.pivot_q),
                                 config_dir, date, robot.get(slot)))
    if solution.marks:
        out.append(write_marks(solution, config_dir, date))
    return out


def write_pen(result: PenCalibration, config_dir, date: str | None = None,
              robot: str | None = None) -> Path:
    """Write the touch-off result as the `pen` part of its slot's file; the `base` part stays.
    `robot`, when given, is written into the part."""
    return _write_part(config_dir, result.slot, "pen", lambda old: pen_part(result, date),
                       robot)


def _summary(part) -> dict | None:
    if not isinstance(part, dict):
        return None
    out = {k: part.get(k) for k in ("passed", "date", "why")}
    if "pen" in part:
        out["pen"] = part["pen"]
    if "method" in part:
        out["method"] = part["method"]
    return out


def listing(config_dir) -> list[dict]:
    """Every calibration file under `config_dir`: slot, file name, size, a digest of its bytes,
    and each part's passed/date/why (None for a missing part)."""
    out = []
    for p in sorted((Path(config_dir) / "calibration").glob("*.json")):
        data = p.read_bytes()
        try:
            cal = json.loads(data)
        except ValueError:
            cal = None
        cal = cal if isinstance(cal, dict) else {}
        out.append(dict(file=p.name, slot=cal.get("slot"), bytes=len(data),
                        digest=hashlib.blake2b(data, digest_size=12).hexdigest(),
                        base=_summary(cal.get("base")), pen=_summary(cal.get("pen"))))
    return out


def read(config_dir, slot: Slot) -> dict | None:
    """One slot's calibration file as written, or None."""
    p = calibration_file(config_dir, slot)
    return json.loads(p.read_text()) if p.exists() else None


def base_tips(config_dir, slots) -> dict:
    """{slot: the tip its base part's height was measured with} for `marks.solve_marks`, from
    each passing base part's `measured_with`; slots without one are left out (z then kept)."""
    out = {}
    for s in slots:
        base = (read(config_dir, s) or {}).get("base") or {}
        tip = (base.get("measured_with") or {}).get("tip_hand_m")
        if base.get("passed") and tip is not None:
            out[s] = np.asarray(tip, float).reshape(3)
    return out
