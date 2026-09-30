"""The job's state, and the store of jobs.  One job at a time.

A drawing job goes received -> fitted -> planning -> drawing -> done | stopped | failed; a
park job goes received -> planning -> moving -> done | stopped | failed.  Every transition is
a "job state" event in the job's log (`events.jsonl` in its directory).  Planning goes on
while the arms draw: "drawing" starts when the first motion is queued.

What the arms have done (motions done, the current motion) is read from the event log that the
executors write; what the planner and the checker have done is kept here by the job's thread.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from aris.execute.log import EventLog
from aris.types import Refusal

FINAL = ("done", "stopped", "failed")


@dataclass
class JobRecord:
    id: str
    kind: str                          # "draw" or "park"
    dir: Path
    name: str = ""                     # the drawing's file name
    t_received: float = 0.0            # time.time()
    state: str = "received"
    why: str = ""
    lines: list = field(default_factory=list)            # the fitted drawing
    fit: object = None                                   # drawing.Fit
    queued: dict = field(default_factory=dict)           # (phase, arm) -> motions queued
    refused: list = field(default_factory=list)          # (phase, arm, index, why)
    system: object = None              # the system planner's Report, filled while it plans
    report: dict | None = None
    stop: threading.Event = field(default_factory=threading.Event)
    coordinator: object = None
    thread: threading.Thread | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    @property
    def log(self) -> EventLog:
        return EventLog(self.dir / "events.jsonl")

    @property
    def finished(self) -> bool:
        return self.state in FINAL

    def set_state(self, state: str, why: str = "", **fields) -> None:
        with self.lock:
            self.state, self.why = state, why
        self.log.write("job state", job=self.id, state=state, why=why, **fields)

    def count_queued(self, phase: str, arm: int) -> int:
        with self.lock:
            n = self.queued.get((phase, arm), 0) + 1
            self.queued[(phase, arm)] = n
            return n


class JobStore:
    """Every job of this server, and which one is running."""

    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.jobs: dict[str, JobRecord] = {}
        self._lock = threading.Lock()

    def running(self) -> JobRecord | None:
        with self._lock:
            return next((j for j in self.jobs.values() if not j.finished), None)

    def admit(self, kind: str, name: str = "") -> JobRecord | Refusal:
        """A new job, unless one is running.  Its directory is not made here (the queue's
        `Job.create` makes it)."""
        with self._lock:
            busy = next((j for j in self.jobs.values() if not j.finished), None)
            if busy is not None:
                return Refusal("busy", f"job {busy.id} is {busy.state}; one job at a time")
            n = len(list(self.root.iterdir())) + 1
            while True:
                jid = f"{time.strftime('%Y%m%d-%H%M%S')}-{n:03d}-{kind}"
                if not (self.root / jid).exists() and jid not in self.jobs:
                    break
                n += 1
            rec = JobRecord(jid, kind, self.root / jid, name, time.time())
            self.jobs[jid] = rec
            return rec

    def get(self, jid: str) -> JobRecord | None:
        with self._lock:
            return self.jobs.get(jid)

    def listing(self) -> list[dict]:
        with self._lock:
            jobs = list(self.jobs.values())
        return [dict(id=j.id, kind=j.kind, state=j.state, why=j.why, name=j.name,
                     received=j.t_received) for j in sorted(jobs, key=lambda j: j.t_received)]

    def from_disk(self, jid: str) -> dict | None:
        """The report of a job of an earlier server run, if it finished."""
        p = self.root / jid / "report.json"
        if "/" in jid or ".." in jid or not p.exists():
            return None
        return json.loads(p.read_text())


def arm_progress(events: list[dict]) -> tuple[dict, float | None]:
    """From the event log: per (phase, arm) motions done and the current motion; and the time
    the first motion started (None before)."""
    rows, first = {}, None
    for e in events:
        if "arm" not in e or "phase" not in e:
            continue
        r = rows.setdefault((e["phase"], e["arm"]), dict(done=0, current=None, status=""))
        ev = e["event"]
        if ev == "motion started":
            r["current"] = dict(index=e["index"], kind=e["kind"], duration=e["duration"])
            first = e["time"] if first is None else first
        elif ev == "motion done":
            r["done"] += 1
            r["current"] = None
        elif ev in ("started", "holding", "finished", "failed", "stopped"):
            r["status"] = ev if ev != "failed" else f"failed: {e.get('why', '')}"
            if ev in ("finished", "failed", "stopped"):
                r["current"] = None
    return rows, first


def handed_back(system_report) -> dict:
    """Per (phase, arm): what the arm planner could not draw so far, by reason (m)."""
    out = {}
    if system_report is None:
        return out
    for _ in range(3):              # the planner thread may be adding to it right now
        try:
            for pr in list(system_report.phases):
                for a, r in list(pr.arms.items()) + list(pr.idle.items()):
                    if r.handed_back:
                        out[(pr.name, a)] = dict(r.handed_back)
            return out
        except RuntimeError:
            out = {}
    return out


def view(rec: JobRecord) -> dict:
    """What GET /jobs/{id} answers: the state, the fitted drawing, per phase and arm what is
    queued, done and running and what was handed back, and the report once finished."""
    rows, first = arm_progress(rec.log.read() if rec.dir.exists() else [])
    with rec.lock:
        queued, refused = dict(rec.queued), list(rec.refused)
    back = handed_back(rec.system)
    keys = sorted(set(rows) | set(queued) | set(back), key=lambda k: (str(k[0]), k[1]))
    arms = []
    for k in keys:
        r = rows.get(k, dict(done=0, current=None, status=""))
        arms.append(dict(phase=k[0], arm=k[1], queued=queued.get(k, 0), done=r["done"],
                         current=r["current"], status=r["status"],
                         handed_back_m=back.get(k, {}),
                         refused=[x[3] for x in refused if (x[0], x[1]) == k]))
    fit = rec.fit
    out = dict(id=rec.id, kind=rec.kind, name=rec.name, state=rec.state, why=rec.why,
               received=rec.t_received, elapsed_s=time.time() - rec.t_received,
               first_motion_s=None if first is None else first - rec.t_received,
               arms=arms, report=rec.report)
    if fit is not None:
        out["drawing"] = dict(lines=len(rec.lines), scale=fit.scale, bbox_in=fit.bbox_in,
                              bbox=fit.bbox_out, area=fit.area)
    return out
