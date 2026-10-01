"""Where the calibration files live, for the one writer (the drawing server's calibrate job) and
the one place that hands them out (the server's `GET /calibration`).  `aris/rig.py` reads them
for everything else.  (Added by the server's agent, 2026-10-01: only rig.py and this package may
name paths inside config/.)"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


def calibration_file(config_dir, arm_id: int) -> Path:
    """config/calibration/<arm_id>.json under `config_dir`."""
    return Path(config_dir) / "calibration" / f"{int(arm_id)}.json"


def write(result, config_dir, date: str | None = None) -> Path:
    """The plane calibration `result` as the arm's calibration file under `config_dir`."""
    from aris.calib.plane import write_calibration
    return write_calibration(result, calibration_file(config_dir, result.arm_id), date)


def listing(config_dir) -> list[dict]:
    """Every calibration file under `config_dir`: arm, file name, size, a digest of its bytes."""
    out = []
    for p in sorted((Path(config_dir) / "calibration").glob("*.json")):
        data = p.read_bytes()
        try:
            arm = int(json.loads(data).get("arm_id"))
        except (ValueError, TypeError, AttributeError):
            arm = None
        out.append(dict(file=p.name, arm=arm, bytes=len(data),
                        digest=hashlib.blake2b(data, digest_size=12).hexdigest()))
    return out


def read(config_dir, arm_id: int) -> dict | None:
    """One arm's calibration file as written, or None."""
    p = calibration_file(config_dir, arm_id)
    return json.loads(p.read_text()) if p.exists() else None
