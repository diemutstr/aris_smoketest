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

The arms follow every motion under joint position control (one tracking mode, 2026-10-07);
a header asking for another (`tracking` other than "position") was planned for something
this PC cannot fly, and is refused.  The collision thresholds are raised for the job and
restored after.  The "runner started" row carries the job's pen (its
`press_m` is for the record: the plan already runs that far below the paper; a header
without a pen falls back to this PC's rig file) and, per slot, the robot the site table names
and whether it was verified.

Nothing here depends on what a job draws: a park job (one arm per phase, free motions) runs
the same way.
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np
from pathlib import Path

from aris.execute import Coordinator, EventLog, Job
from aris.version import code_version, describe, same
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
    why = next((f for f in s.flags if f.startswith(("no joint states", "stale joint states"))),
               "no joint states")
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


GRIP_PARAMS = {"width_m": "width", "speed_m_per_s": "speed", "force_n": "force",
               "epsilon_inner_m": "epsilon_inner", "epsilon_outer_m": "epsilon_outer"}


def run_grip(remote: Remote, job_id: str, header: dict, drivers: dict, progress=None):
    """A grip job: `{"kind": "grip", "slot": "2L", "verb": "home"|"open"|"close", "params":
    {"width_m", "speed_m_per_s", "force_n", "epsilon_inner_m", "epsilon_outer_m"} (any of
    them; the site's for the rest)}`, no phases.  Posts on the job "grip started" (width),
    then "grip done" (verb, width_before_m, width_after_m, grasped) or "failed" (why), then
    "job done" / "job failed" and "runner finished"."""
    from aris.execute.coordinator import JobRun
    progress = progress or (lambda step: None)
    slot, verb = str(header.get("slot")), str(header.get("verb"))
    drv = drivers.get(slot)
    if drv is None or not hasattr(drv, "gripper"):
        return Refusal("no_gripper", f"slot {slot} is not mounted on this PC")
    params = {GRIP_PARAMS[k]: float(v) for k, v in (header.get("params") or {}).items()
              if k in GRIP_PARAMS}
    progress("started")
    seq = [0]

    def post(event, **f):
        remote.post_events(job_id, [dict(seq=seq[0], time=time.time(), event=event, **f)])
        seq[0] += 1

    post("grip started", arm=slot, verb=verb, width_m=drv.gripper.port.width())
    r = drv.gripper.run(verb, **params)
    if r.done:
        grasped = True if verb == "close" else (False if verb == "open" else None)
        post("grip done", arm=slot, verb=verb, width_before_m=r.width_before,
             width_after_m=r.width_after, grasped=grasped, nothing_to_do=r.noop, why=r.why)
        post("job done", why="")
    else:
        post("failed", arm=slot, verb=verb, why=r.why, width_before_m=r.width_before,
             width_after_m=r.width_after)
        post("job failed", why=r.why)
    status = "done" if r.done else "failed"
    post("runner finished", status=status, why=r.why)
    return JobRun(status=status, why=r.why)


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


def _job_settings(header: dict, rig, drivers: dict, robots: dict | None):
    """What the job's header says, checked and given to the drivers; or a Refusal.  `robots`:
    slot -> {"robot", "identity"} (site.identity); a slot whose robot is not the one the site
    table names is refused."""
    tracking = header.get("tracking", "position")
    if tracking != "position":
        return Refusal("bad_header", f"tracking {tracking!r}: this PC flies joint position "
                                     f"control only")
    bad = {a: r["identity"] for a, r in (robots or {}).items()
           if a in drivers and str(r.get("identity", "")).startswith("mismatch")}
    if bad:
        return Refusal("wrong_robot", "; ".join(f"slot {a}: {w}" for a, w in bad.items()))
    pen, pen_from = (header["pen"], "job header") if header.get("pen") else (rig.pen(), "rig file")
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
            poll: float = 0.01, robots: dict | None = None, progress=None,
            code: dict | None = None):
    """Runs the job to its end on `drivers` (slot -> Driver, the mounted arms).  Returns the
    coordinator's JobRun, or a Refusal when it cannot start.  For every job the collision
    thresholds are the site's "job" ones while it runs, "normal" after.
    `progress(step)`: told each step before the job starts, and "started" once it has (serve
    watches it: a job never hangs silently before it starts).  `code`: this PC's code version
    (default: read now); a job planned by other code is refused before anything moves."""
    progress = progress or (lambda step: None)
    progress("job header")
    header = remote.header(job_id)
    if isinstance(header, Refusal):
        return header
    mine = code if code is not None else code_version()
    if not same(header.get("code"), mine):
        return Refusal("wrong_code", f"the job was planned by {describe(header.get('code'))}, "
                                     f"this PC runs {describe(mine)}: update both machines to "
                                     f"the same commit")
    if header.get("kind") == "mark":            # its touches are hand-guided
        return Refusal("no_hand_guiding", "hand-guiding is not available on this rig")
    if header.get("kind") == "grip":            # no plan, no motion, no thresholds
        return run_grip(remote, job_id, header, drivers, progress)
    why = check_header(header, rig)
    if why:
        return Refusal("wrong_rig", why)
    d = Path(work_dir) / job_id
    if (d / "events.jsonl").exists():
        return Refusal("already_run", f"{d} holds a run of this job; a job runs once")
    progress("job settings")
    settings = _job_settings(header, rig, drivers, robots)
    if isinstance(settings, Refusal):
        return settings
    progress("collision thresholds")
    threshold_note = _collision(drivers, "job")
    if threshold_note:      # the arms keep their normal (lower) thresholds: noted, not refused
        settings["collision_thresholds_not_set"] = threshold_note
    try:
        return _run(remote, job_id, header, d, rig, config_dir, drivers, poll, settings,
                    progress)
    finally:
        _collision(drivers, "normal")


def _run(remote, job_id, header, d, rig, config_dir, drivers, poll, settings, progress):
    progress("work folder")
    d.mkdir(parents=True, exist_ok=True)
    (d / "job.json").write_text(json.dumps(header, indent=1, sort_keys=True))
    job = Job(d)
    coord = Coordinator(job, drivers, config_dir, settings["start_tolerance"], poll=poll)
    coord.log = log = ArmLog(job.log_path, drivers)   # the executors write through it too
    log.write("runner started", job=job_id, where=None, **settings)
    progress("started")
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
            yield phase

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
