"""The station: what the drawing server works with.  The rig, its calibration state, the arm
drivers, and the knobs of planning and checking, all as arguments; nothing global.

`open_station` refuses (returns a Refusal) when the rig does not load, when an arm has no
passing calibration and the server was not started with `uncalibrated=True`, or when the
driver asked for is not built.  Uncalibrated, every job report says so.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from aris.execute.queue import digest
from aris.rig import Rig
from aris.server import paper as paper_mod
from aris.system import area as area_mod
from aris.system import maps as maps_mod
from aris.system import phases as all_phases
from aris.system.settings import Settings
from aris.types import Refusal

# "sim": simulated arms in this process.  "robot": the arms are on the operator PC, whose
# runner copies the queues and posts the events back; the server runs no executors.
DRIVERS = ("sim", "robot")


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
    settings: Settings = field(default_factory=Settings)
    drawing_area: tuple = ()           # (x, y) full widths, m, centred on the table: rig.json's
    maps_area: tuple = ()              # the same worked out from the drawable maps
    # --driver robot: where the operator PC last said each arm stands (Positions)
    positions: object = None
    calib_settings: object = None      # calibrate.CalibSettings; None: the defaults
    operator: object = None            # the operator channel (operator.py)
    # the paper's height map (aris.calib.paper.Surface), None: the flat paper (server/paper.py)
    surface: object = None
    # why a drawing cannot be planned on this rig ("" when it can): no drawing area, or one the
    # maps do not cover.  Only drawing jobs are refused; park, calibrate and marks still run.
    area_problem: str = ""

    @property
    def drawing_centre(self) -> tuple:
        """The drawing area's centre in the table frame (`Rig.drawing_area_centre_m`)."""
        c = getattr(self.rig, "drawing_area_centre_m", None)
        return (0.0, 0.0) if c is None else tuple(float(x) for x in np.asarray(c).reshape(2))

    def pen(self) -> dict:
        """The pen that is in, as the rig has it (`rig.pen()`: its table entry and its name)."""
        return dict(self.rig.pen()) if hasattr(self.rig, "pen") else {}

    def reload(self) -> None:
        """Read the rig again (after a calibration file was written)."""
        self.rig = Rig.load(self.config_dir)
        self.surface = paper_mod.load(self.config_dir)
        self.uncalibrated = any(not self.rig.calibrated(a) for a in self.rig.arm_ids)

    @property
    def remote(self) -> bool:
        """The arms are on the operator PC: this server only plans, checks and queues."""
        return self.driver_kind == "robot"

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
                    pen=self.pen().get("name"),
                    note=("UNCALIBRATED: every arm runs on its nominal pose and pen"
                          if self.uncalibrated else "calibrated"))


STALE_S = 60.0           # s: a joint reading older than this is no reading


class Positions:
    """The newest joints the operator PC reported per arm, with when (its clock), when the
    server heard it, and the robot it named.  Only `--driver robot` fills it.

    A reading that is no reading — `"q": null` (the arm's stack is down), all-zero joints (what
    a stack without joint states once sent), or one older than STALE_S — is kept as "unknown";
    nothing is ever planned or judged from it (`known`)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._q: dict = {}

    def note(self, arm: str, q, at: float, job: str, robot=None) -> None:
        if q is not None:
            q = np.asarray(q, float).reshape(-1)
            if q.shape != (7,) or not np.all(np.isfinite(q)) or not np.any(q):
                q = None                                 # malformed or all zeros: no reading
        with self._lock:
            old = self._q.get(arm)
            if old is None or at >= old["reported_at"]:
                self._q[arm] = dict(q=q, reported_at=float(at), received_at=time.time(),
                                    job=job, source="operator PC",
                                    robot=robot if robot is not None else
                                    (old or {}).get("robot"))

    def from_row(self, row: dict, job: str) -> None:
        """Every joint position an event row carries: `q` of its arm (null: no reading),
        `where` of every arm."""
        at = float(row.get("time", time.time()))
        robot = row.get("robot")
        if "arm" in row and "q" in row and (row["q"] is None or (
                isinstance(row["q"], list) and len(row["q"]) == 7)):
            self.note(str(row["arm"]), row["q"], at, job, robot)
        where = row.get("where")
        if isinstance(where, dict):
            for a, q in where.items():
                if q is None or (isinstance(q, list) and len(q) == 7):
                    self.note(str(a), q, at, job)

    def all(self) -> dict:
        with self._lock:
            return {a: dict(v) for a, v in self._q.items()}

    def known(self, arm: str, now: float | None = None):
        """-> (joints, "") when there is a fresh real reading; (None, why) when there is not
        (never reported: why is "")."""
        with self._lock:
            p = self._q.get(arm)
        if p is None:
            return None, ""
        now = time.time() if now is None else now
        if p["q"] is None:
            return None, f"no joint states for {arm} (FCI off?)"
        if now - p["received_at"] > STALE_S:
            return None, (f"no joint states for {arm} (FCI off?): the last reading is "
                          f"{now - p['received_at']:.0f} s old")
        return p["q"].copy(), ""


def fake_paper(rig, a: str, spec=None) -> tuple:
    """The simulated arms' paper in arm a's base frame: (normal toward the arm, offset), the
    nominal paper moved by dz and tilted by roll (about table x) and pitch (about table y)
    about the table centre.  The simulated arms hang where the rig says at the start."""
    dz, roll, pitch = (0.0, 0.0, 0.0) if spec is None else (float(x) for x in spec)
    r, p = np.deg2rad(roll), np.deg2rad(pitch)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    n_t = Ry @ Rx @ np.array([0.0, 0.0, 1.0])
    p_t = np.array([0.0, 0.0, rig.paper_z + dz])
    T = rig.T_base_table(a)
    n_b = T[:3, :3] @ n_t
    return n_b, float(n_b @ (T[:3, :3] @ p_t + T[:3, 3]))


def default_workers() -> int:
    return max(1, min(8, (os.cpu_count() or 2) // 2))


def open_station(config_dir, driver: str = "sim", speed: float = 1.0,
                 uncalibrated: bool = False, cache_dir="out/cache", jobs_dir="out/jobs",
                 workers: int | None = None,
                 settings: Settings | None = None, drivers: dict | None = None,
                 with_arms: bool = True, with_area: bool = True,
                 sim_paper=None, sim_truth=None,
                 sim_base_error=None, sim_buttons=None,
                 sim_mark_error=None) -> Station | Refusal:
    """`drivers`: arm id -> Driver to use instead of starting them (tests).  `with_arms`
    False: no drivers at all (plan and check only).  `sim_paper`: (dz m, roll deg, pitch deg),
    the simulated arms' paper against the nominal one (default: the nominal paper).
    `with_area` False: the drawing area is not worked out (it needs the drawable maps;
    checking does not).  The mark job's simulated person (simtruth.py) works in a "true" world:
    `sim_truth` another config directory, or `sim_base_error` (mm, mrad) every base moved and
    `sim_mark_error` (cm) every mark;
    `sim_buttons` {slot: [button, ...]} scripts the pilot buttons."""
    config_dir = Path(config_dir)
    try:
        rig = Rig.load(config_dir)
    except (OSError, ValueError, KeyError) as e:
        return Refusal("rig", f"the rig in {config_dir} does not load: {e}")
    missing = {a: rig.calibration_status(a) for a in rig.arm_ids if not rig.calibrated(a)}
    if missing and not uncalibrated:
        return Refusal("uncalibrated", "no passing calibration for arms "
                       + ", ".join(f"{a} ({s})" for a, s in missing.items())
                       + "; start with --uncalibrated to run on the nominal poses")
    if driver not in DRIVERS:
        return Refusal("driver", f"driver {driver!r} is not built; built: {DRIVERS}")
    if not speed > 0:
        return Refusal("speed", "the speed must be positive")
    if drivers is None and with_arms and driver == "sim":
        from aris.execute.drivers.sim import SimArm
        from aris.server.simtruth import Person, Truth
        truth = Truth(rig, None if sim_truth is None else Rig.load(sim_truth), sim_base_error,
                      sim_mark_error)
        drivers = {a: SimArm(a, rig.park_q(a), speed=speed,
                             paper=fake_paper(rig, a, sim_paper), tip_of=rig.arm(a).tip,
                             person=Person(truth, a, (sim_buttons or {}).get(a, ())))
                   for a in rig.arm_ids}
    cfg = settings or Settings()
    w = workers or default_workers()
    kind = driver if drivers or with_arms else "none (plan and check only)"
    if driver == "robot":
        drivers = {}
    st = Station(rig, config_dir, dict(drivers or {}), kind, float(speed), bool(missing),
                 None if cache_dir is None else Path(cache_dir), Path(jobs_dir), w,
                 cfg)
    if st.cache_dir is not None:
        st.cache_dir.mkdir(parents=True, exist_ok=True)
    st.positions = Positions()
    from aris.server.calibrate import CalibSettings
    from aris.server.operator import Channel
    st.calib_settings = CalibSettings()
    st.operator = Channel()
    if with_area:
        st.maps_area = drawing_area(st)
        fa = file_area(rig)
        if fa is None:
            st.area_problem = "rig.json has no canvas.drawing_area_m"
        else:
            st.area_problem = area_mismatch(st.maps_area, fa, cfg.grid_step, st.drawing_centre)
            st.drawing_area = fa
    st.surface = paper_mod.load(config_dir)
    return st


def file_area(rig) -> tuple | None:
    """rig.json's `canvas.drawing_area_m`, as the rig read it (`Rig.drawing_area_m`): the area
    the system planner refuses a drawing against, and so the one the server fits to."""
    a = getattr(rig, "drawing_area_m", None)
    return None if a is None else tuple(float(x) for x in np.asarray(a).reshape(2))


def area_mismatch(maps_area, rig_area, cell: float, centre=(0.0, 0.0)) -> str:
    """Why the rig file's drawing area cannot be drawn, or "".  It may be smaller than the
    area the drawable maps give (a conservative choice), never larger by more than one grid
    cell (the system planner checks the same)."""
    if rig_area is None:
        return ""
    gap = max(b - a for a, b in zip(maps_area, rig_area))
    if gap <= cell + 1e-9:
        return ""
    if min(maps_area) <= 0.0:
        return (f"the drawable maps are empty about the centre ({centre[0]:+.3f}, "
                f"{centre[1]:+.3f}) m: no mounted arm can draw a rectangle around it; write a "
                "drawing area and centre for these arms (tools/mounted_rig.py computes them)")
    return (f"rig.json's canvas.drawing_area_m {rig_area[0]:.3f} x {rig_area[1]:.3f} m is larger "
            f"than the area the drawable maps give, {maps_area[0]:.3f} x {maps_area[1]:.3f} m, "
            f"by {100 * gap:.1f} cm (more than one grid cell, {100 * cell:.0f} cm): the rig file "
            "is stale; write the maps' area into it")


def drawing_area(st: Station) -> tuple:
    """The area the drawable maps give (read from the cache, or built)."""
    ph = all_phases(st.rig)
    maps = maps_mod.load_or_build(st.rig, ph, st.rules.gates, st.settings, st.cache_dir,
                                  st.workers, press=st.rules.press)
    return tuple(float(x) for x in np.asarray(area_mod.admissible(maps, centre=st.drawing_centre)))
