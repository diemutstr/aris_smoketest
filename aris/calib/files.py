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

from aris.calib.plane import PlaneCalibration
from aris.calib.pen import PenCalibration
from aris.types import Slot

PLANE_CONVENTION = (
    "rotation = R(v) @ the rotation the rig had, v = (roll, pitch, 0) a rotation vector in the "
    "table frame (the smallest turn, about a horizontal axis through the base origin); base "
    "table z moved by height_change; x, y and the turn about the vertical are unchanged. The "
    "height includes the length error of the pen named here against the tip named here.")


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
        "pen": r.pen, "why": r.why,
        "tip_hand_m": _list(r.tip_hand),
        "reference_touch": {"xy_table_m": _list(r.reference_xy_table),
                            "q": _list(r.q, 12)},
        "tip_hand_nominal_m": _list(r.tip_hand_nominal),
        "tip_hand_before_m": _list(r.tip_hand_before),
        "correction_mm": _mm(r.correction), "change_mm": _mm(r.change),
        "height_before_mm": _mm(r.height_before),
        "touch_xy_table_m": _list(r.touch_xy_table),
        "from_reference_mm": _mm(r.from_reference),
        "against_base": r.base_status,
    }


def _write_part(config_dir, slot: Slot, name: str, part: dict) -> Path:
    """Read the slot's file (if any), replace one part, write it back whole.  A file that is not
    a calibration file for this slot is a broken install: raise rather than lose the other part."""
    path = calibration_file(config_dir, slot)
    cal = {"slot": slot}
    if path.exists():
        cal = json.loads(path.read_text())
        if not isinstance(cal, dict) or cal.get("slot") != slot:
            raise ValueError(f"{path} is not the calibration file of slot {slot}")
    cal[name] = part
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(cal, indent=1) + "\n")
    os.replace(tmp, path)              # a reader never sees half a file
    return path


def write_base(result: PlaneCalibration, config_dir, date: str | None = None) -> Path:
    """Write the plane result as the `base` part of its slot's file; the `pen` part stays."""
    return _write_part(config_dir, result.slot, "base", base_part(result, date))


def write_pen(result: PenCalibration, config_dir, date: str | None = None) -> Path:
    """Write the touch-off result as the `pen` part of its slot's file; the `base` part stays."""
    return _write_part(config_dir, result.slot, "pen", pen_part(result, date))


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
