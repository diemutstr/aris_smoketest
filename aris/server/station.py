"""The station: what the drawing server works with.  The rig, its calibration state, the arm
drivers, and the knobs of planning and checking, all as arguments; nothing global.

`open_station` refuses (returns a Refusal) when the rig does not load, when an arm has no
passing calibration and the server was not started with `uncalibrated=True`, or when the
driver asked for is not built.  Uncalibrated, every job report says so.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from aris.execute.queue import digest
from aris.rig import Rig
from aris.system import area as area_mod
from aris.system import maps as maps_mod
from aris.system import phases as all_phases
from aris.system.settings import Settings
from aris.types import Refusal

DRIVERS = ("sim",)


@dataclass
class Station:
    rig: Rig
    config_dir: Path
    drivers: dict                      # arm id -> Driver
    driver_kind: str
    speed: float
    uncalibrated: bool
    cache_dir: Path | None
    jobs_dir: Path
    workers: int                       # processes for the system planner
    check_workers: int                 # processes for the checker
    settings: Settings = field(default_factory=Settings)
    drawing_area: tuple = ()           # (x, y) full widths, m, centred on the table

    @property
    def rules(self):
        return self.rig.rules()

    def calibration(self) -> dict:
        return {a: self.rig.calibration_status(a) for a in self.rig.arm_ids}

    def digests(self) -> dict:
        calib = {a: (m.T_table_base, m.tip_hand, m.calibration)
                 for a, m in self.rig.mounts.items()}
        return dict(rig_digest=digest(self.rig), calibration_digest=digest(calib))

    def assumptions(self) -> dict:
        """What every job runs on, printed once by every command."""
        cal = self.calibration()
        return dict(self.digests(), calibration=cal, uncalibrated=self.uncalibrated,
                    driver=self.driver_kind, speed=self.speed,
                    note=("UNCALIBRATED: every arm runs on its nominal pose and pen"
                          if self.uncalibrated else "calibrated"))


def default_workers() -> int:
    return max(1, min(8, (os.cpu_count() or 2) // 2))


def open_station(config_dir, driver: str = "sim", speed: float = 1.0,
                 uncalibrated: bool = False, cache_dir="out/cache", jobs_dir="out/jobs",
                 workers: int | None = None, check_workers: int | None = None,
                 settings: Settings | None = None, drivers: dict | None = None,
                 with_arms: bool = True, with_area: bool = True) -> Station | Refusal:
    """`drivers`: arm id -> Driver to use instead of starting them (tests).  `with_arms`
    False: no drivers at all (plan and check only).  `with_area` False: the drawing area is
    not worked out (it needs the drawable maps; checking does not)."""
    config_dir = Path(config_dir)
    try:
        rig = Rig.load(config_dir)
    except (OSError, ValueError, KeyError) as e:
        return Refusal("rig", f"the rig in {config_dir} does not load: {e}")
    missing = {a: s for a, s in ((a, rig.calibration_status(a)) for a in rig.arm_ids)
               if not s.startswith("applied")}
    if missing and not uncalibrated:
        return Refusal("uncalibrated", "no passing calibration for arms "
                       + ", ".join(f"{a} ({s})" for a, s in missing.items())
                       + "; start with --uncalibrated to run on the nominal poses")
    if driver not in DRIVERS:
        return Refusal("driver", f"driver {driver!r} is not built; built: {DRIVERS}")
    if not speed > 0:
        return Refusal("speed", "the speed must be positive")
    if drivers is None and with_arms:
        from aris.execute.drivers.sim import SimArm
        drivers = {a: SimArm(a, rig.park_q(a), speed=speed) for a in rig.arm_ids}
    cfg = settings or Settings()
    w = workers or default_workers()
    kind = driver if drivers or with_arms else "none (plan and check only)"
    st = Station(rig, config_dir, dict(drivers or {}), kind, float(speed), bool(missing),
                 None if cache_dir is None else Path(cache_dir), Path(jobs_dir), w,
                 check_workers or w, cfg)
    if st.cache_dir is not None:
        st.cache_dir.mkdir(parents=True, exist_ok=True)
    if with_area:
        st.drawing_area = drawing_area(st)
    return st


def drawing_area(st: Station) -> tuple:
    """The area the system planner accepts, from its own drawable maps (read from the cache,
    or built): the same number it refuses a drawing against.  rig.json carries the same
    rectangle (`canvas.drawing_area_m`), but `rig.py` does not read it yet."""
    ph = all_phases(st.rig)
    maps = maps_mod.load_or_build(st.rig, ph, st.rules.gates, st.settings, st.cache_dir,
                                  st.workers)
    return tuple(float(x) for x in np.asarray(area_mod.admissible(maps)))
