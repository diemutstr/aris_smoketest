"""Run one job of the drawing server on the arms of this operator PC.

    aris-robot run --site robot/site.json --job <id>

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

Nothing here depends on what a job draws: a park job (one arm per phase, free motions) runs
the same way.

The executor sends lower and lift motions to `move` with the trajectory only.  The pen force
needs to know them, so `KindRouter` finds each trajectory in the arm's local queues and sends
lower and lift to the driver's `draw` (until the executor does that itself).
"""
from __future__ import annotations

import hashlib
import json
import threading
from pathlib import Path

from aris.execute import Coordinator, EventLog, Job
from aris.execute.queue import Cursor, End, Queue, digest
from aris.types import Refusal

from aris_robot.remote import EventForwarder, Remote

PEN_MOTIONS = ("lower", "draw", "lift")


def _key(traj) -> bytes:
    return hashlib.blake2b(traj.t.tobytes() + traj.q.tobytes(), digest_size=16).digest()


class KindRouter:
    """A driver that sends every pen motion (lower, draw, lift) to `draw`."""

    def __init__(self, driver, job_dir, arm_id: int):
        self.driver, self.dir, self.arm_id = driver, Path(job_dir), arm_id
        self._cursors: dict[Path, Cursor] = {}
        self._kinds: dict[bytes, object] = {}
        self._lock = threading.Lock()

    def _lookup(self, traj):
        with self._lock:
            for p in self.dir.glob(f"*__arm{self.arm_id}.queue"):
                cur = self._cursors.setdefault(p, Cursor(p))
                for item in cur.poll():
                    if not isinstance(item, End):
                        self._kinds[_key(item.motion.traj)] = item.motion
            return self._kinds.get(_key(traj))

    def move(self, traj):
        m = self._lookup(traj)
        if m is not None and m.kind in PEN_MOTIONS:
            return self.driver.draw(m)
        return self.driver.move(traj)

    def draw(self, motion):
        return self.driver.draw(motion)

    def touch(self, motion):
        return self.driver.touch(motion)

    def state(self):
        return self.driver.state()

    def hold(self):
        return self.driver.hold()

    def stop(self):
        return self.driver.stop()

    def recover(self):
        return self.driver.recover()


class ArmLog(EventLog):
    """The job's event log, with where the arm stands on every row about an arm."""

    def __init__(self, path, drivers: dict):
        super().__init__(path)
        self.drivers = drivers

    def write(self, event: str, **fields) -> dict:
        a = fields.get("arm")
        if a in self.drivers and fields.get("q") is None:
            fields["q"] = self.drivers[a].state().q
        return super().write(event, **fields)

    def where(self) -> dict:
        return {str(a): d.state().q for a, d in self.drivers.items()}


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


def run_job(remote: Remote, job_id: str, rig, config_dir, work_dir, drivers: dict,
            poll: float = 0.01):
    """Runs the job to its end on `drivers` (arm id -> Driver, the mounted arms).  Returns the
    coordinator's JobRun, or a Refusal when it cannot start."""
    header = remote.header(job_id)
    if isinstance(header, Refusal):
        return header
    why = check_header(header, rig)
    if why:
        return Refusal("wrong_rig", why)
    d = Path(work_dir) / job_id
    if (d / "events.jsonl").exists():
        return Refusal("already_run", f"{d} holds a run of this job; a job runs once")
    d.mkdir(parents=True, exist_ok=True)
    (d / "job.json").write_text(json.dumps(header, indent=1, sort_keys=True))
    job = Job(d)

    routed = {a: KindRouter(drv, d, a) for a, drv in drivers.items()}
    coord = Coordinator(job, routed, config_dir, rig, poll=poll)
    coord.log = log = ArmLog(job.log_path, drivers)   # the executors write through it too
    log.write("runner started", job=job_id, where=log.where())
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
    try:
        result = coord.run(phases())
        if refused:
            result.status, result.why = "failed", refused[0]
        log.write("runner finished", job=job_id, status=result.status, why=result.why,
                  where=log.where())
    finally:
        mirror.close()
        events.close()
    return result
