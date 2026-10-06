"""Run one job of the drawing server on the arms of this operator PC (serve's "run" command).

1. Reads the job header from the server and refuses when this PC's rig or calibration is not
   the one the job was planned against (the same commit must run on both machines).
2. Keeps a local copy of the job directory: the phase list and, as each phase appears, the
   queue of every arm that moves in it, byte for byte (`remote.Remote.follow`).
3. Runs the job with the package's own `Coordinator` and `Executor`s on that copy, with a real
   driver for every mounted arm.  A phase that needs an arm that is not mounted fails the job
   before that phase starts (the arms already done stay where they ended).
4. Posts every line of the local event log to the server as it is written, and stops the arms
   when the server says the job was stopped.

Where the arms stand is reported, because the server plans (parks, the next drawing) from it:
the first row, "runner started", carries `where` (arm id -> 7 joints) for every mounted arm,
read before anything moves; every row about an arm carries `q`, read from that arm when the
row is written; the last row, "runner finished", carries `where` again.  A stale position
cannot move an arm: the executor refuses a motion that does not start within the rig's start
tolerance (0.005 rad per joint) of the arm's joints, writes a "failed" row saying by how much,
and the arm holds.

The header's `tracking` says how the arms follow (DESIGN 4c): "position" (mode A, the default:
every motion through the trajectory controller, the press geometric, the collision thresholds
raised for the job and restored after) or "impedance" (mode B: lower, draw and lift under the
impedance controller with the pen force).  The "runner started" row says which, with the pen
(its `press_m` is for the record: the plan already runs that far below the paper) and, per
slot, the robot the site table names and whether it was verified.

How hard the pen presses (band, levels, cap, ramps, servo) is the job header's `pen` block,
copied by the server from its rig.json, so both machines use the same numbers; a header
without it (an older job) runs with this PC's rig file.  The "runner started" row says which.

Nothing here depends on what a job draws: a park job (one arm per phase, free motions) runs
the same way.
"""
from __future__ import annotations

import json
import threading

import numpy as np
from pathlib import Path

from aris.execute import Coordinator, EventLog, Job
from aris.execute.queue import Queue, digest
from aris.types import Refusal

from aris_robot.remote import EventForwarder, Remote

def reading(driver) -> tuple:
    """(q as 7 floats, None) when the arm has a reading; (None, why) when it has none (its
    stack down, FCI off): a position is never reported as zeros."""
    s = driver.state()
    q = np.asarray(s.q, float)
    if q.shape == (7,) and np.all(np.isfinite(q)):
        return [float(x) for x in q], None
    why = next((f for f in s.flags if f.startswith("no joint states")), "no joint states")
    return None, why


def where_of(drivers: dict) -> dict:
    """`where` for a row: slot -> 7 joints or None, and `where_missing`: slot -> why."""
    where, missing = {}, {}
    for a, d in drivers.items():
        where[str(a)], why = reading(d)
        if why:
            missing[str(a)] = why
    return dict(where=where, where_missing=missing) if missing else dict(where=where)


class ArmLog(EventLog):
    """The job's event log, with where the arm stands on every row about an arm (`q`, or
    `"q": null` with the `reason`), and every `where` read fresh (never zeros)."""

    def __init__(self, path, drivers: dict):
        super().__init__(path)
        self.drivers = drivers

    def write(self, event: str, **fields) -> dict:
        a = fields.get("arm")
        q = fields.get("q")
        if a in self.drivers and (q is None or not np.all(np.isfinite(np.asarray(q, float)))):
            fields["q"], why = reading(self.drivers[a])
            if why:
                fields["reason"] = why
        if "where" in fields:
            fields.update(where_of(self.drivers))
        return super().write(event, **fields)

    def where(self) -> dict:
        return where_of(self.drivers)["where"]


def check_header(header: dict, rig) -> str:
    """Why this PC may not run the job ("" if it may)."""
    if "rig_digest" in header and header["rig_digest"] != digest(rig):
        return "the rig on this PC differs from the one the job was planned for (update config/)"
    if "calibration_digest" in header:
        calib = {a: (m.T_table_base, m.tip_hand, m.calibration) for a, m in rig.mounts.items()}
        if header["calibration_digest"] != digest(calib):
            return "the calibration on this PC differs from the job's (copy config/calibration/)"
    return ""


def _queue_done(path: Path) -> bool:
    return Queue(path).end() is not None


def _phases_done(path: Path) -> bool:
    text = path.read_text()
    return any(json.loads(x).get("end") for x in text[:text.rfind("\n") + 1].splitlines() if x)


class Mirror:
    """The local copy of a job: the phase list, then each moving arm's queue as its phase
    appears (the mounted arms' queues only)."""

    def __init__(self, remote: Remote, job_id: str, job: Job, mounted):
        self.remote, self.id, self.job, self.mounted = remote, job_id, job, set(mounted)
        self.stop = threading.Event()
        self.threads: list[threading.Thread] = []

    def _spawn(self, fn, *args) -> None:
        t = threading.Thread(target=fn, args=args, daemon=True)
        t.start()
        self.threads.append(t)

    def start(self) -> "Mirror":
        self._spawn(self.remote.follow, ("jobs", self.id, "phases"),
                    self.job.dir / "phases.jsonl", _phases_done, self.stop)
        self._spawn(self._phases)
        return self

    def _phases(self) -> None:
        for phase in self.job.watch_phases(0.05, self.stop):
            for a in phase.active:
                if a in self.mounted:
                    self._spawn(self.remote.follow, ("jobs", self.id, "queues", phase.name, a),
                                self.job.queue(phase.name, a).path, _queue_done, self.stop)

    def close(self) -> None:
        self.stop.set()


TRACKING = ("position", "impedance")


def _job_settings(header: dict, rig, drivers: dict, robots: dict | None):
    """The job's pen rules and tracking mode, given to every driver; or a Refusal.  `robots`:
    slot -> {"robot", "identity"} (site.identity); a slot whose robot is not the one the site
    table names is refused."""
    tracking = header.get("tracking", "position")
    if tracking not in TRACKING:
        return Refusal("bad_header", f"tracking {tracking!r}: position or impedance")
    bad = {a: r["identity"] for a, r in (robots or {}).items()
           if a in drivers and str(r.get("identity", "")).startswith("mismatch")}
    if bad:
        return Refusal("wrong_robot", "; ".join(f"slot {a}: {w}" for a, w in bad.items()))
    pen, pen_from = (header["pen"], "job header") if header.get("pen") else (rig.pen(), "rig file")
    for drv in drivers.values():
        if hasattr(drv, "set_job"):
            drv.set_job(pen, tracking)
        elif hasattr(drv, "set_pen"):
            drv.set_pen(pen)
    tol = (header.get("execution") or {}).get("start_tolerance_rad")
    tol = float(tol) if tol is not None else float(rig.execution().start_tolerance)
    for drv in drivers.values():
        if hasattr(drv, "start_tol"):
            drv.start_tol = tol
    return dict(pen=pen, pen_from=pen_from, tracking=tracking, robots=robots or {},
                start_tolerance=tol)


def _collision(drivers: dict, which: str) -> str:
    """The site's collision thresholds `which` on every driver that has them; why not, or ""."""
    whys = []
    for a, drv in drivers.items():
        if hasattr(drv, "set_collision"):
            why = drv.set_collision(which)
            if why:
                whys.append(f"slot {a}: {why}")
    return "; ".join(whys)


def run_job(remote: Remote, job_id: str, rig, config_dir, work_dir, drivers: dict,
            poll: float = 0.01, robots: dict | None = None, around_phase=None):
    """Runs the job to its end on `drivers` (slot -> Driver, the mounted arms).  Returns the
    coordinator's JobRun, or a Refusal when it cannot start.  In position tracking (mode A)
    the collision thresholds are the site's "job" ones while it runs, "normal" after.
    `around_phase(phase)`: a context manager entered before the phase runs and left after it
    (serve's calibration hand-over); its `__enter__` returns why the phase cannot run, or ""."""
    header = remote.header(job_id)
    if isinstance(header, Refusal):
        return header
    why = check_header(header, rig)
    if why:
        return Refusal("wrong_rig", why)
    d = Path(work_dir) / job_id
    if (d / "events.jsonl").exists():
        return Refusal("already_run", f"{d} holds a run of this job; a job runs once")
    settings = _job_settings(header, rig, drivers, robots)
    if isinstance(settings, Refusal):
        return settings
    # mode A raises the collision thresholds through the stacks; a mark job has its arm's stack
    # down (the calibration driver flies it), so it does not
    thresholds = settings["tracking"] == "position" and header.get("kind") != "mark"
    if thresholds:
        why = _collision(drivers, "job")
        if why:
            _collision(drivers, "normal")
            return Refusal("collision_thresholds", why)
    try:
        return _run(remote, job_id, header, d, rig, config_dir, drivers, poll, settings,
                    around_phase)
    finally:
        if thresholds:
            _collision(drivers, "normal")


def _run(remote, job_id, header, d, rig, config_dir, drivers, poll, settings, around_phase):
    d.mkdir(parents=True, exist_ok=True)
    (d / "job.json").write_text(json.dumps(header, indent=1, sort_keys=True))
    job = Job(d)
    coord = Coordinator(job, drivers, config_dir, settings["start_tolerance"], poll=poll)
    coord.log = log = ArmLog(job.log_path, drivers)   # the executors write through it too
    log.write("runner started", job=job_id, where=None, **settings)
    refused: list[str] = []
    halt = threading.Event()

    def stop() -> None:
        halt.set()
        coord.stop()

    def phases():
        """The job's phases as they appear; the first one that needs an arm that is not
        mounted ends the job, before it starts."""
        for phase in job.watch_phases(poll, halt):
            missing = [a for a in phase.active if a not in drivers]
            if missing:
                refused.append(f"{phase.name} needs arms {missing}, which are not mounted "
                               f"(site.json)")
                coord.log.write("refused", phase=phase.name, why=refused[-1])
                stop()
                return
            if around_phase is None:
                yield phase
                continue
            with around_phase(phase) as why:
                if why:
                    refused.append(f"{phase.name}: {why}")
                    coord.log.write("refused", phase=phase.name, why=refused[-1])
                    stop()
                    return
                yield phase                 # the coordinator runs it; we resume after it

    mirror = Mirror(remote, job_id, job, drivers).start()
    events = EventForwarder(remote, job_id, job.log_path, stop).start()
    source = phases()
    try:
        result = coord.run(source)
        if refused:
            result.status, result.why = "failed", refused[0]
        log.write("runner finished", job=job_id, status=result.status, why=result.why,
                  where=None)
    finally:
        source.close()                      # leaves a phase's hand-over if the job broke off
        mirror.close()
        events.close()
    return result
