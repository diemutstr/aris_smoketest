"""From the system planner's stream, through the independent checker, into the job's queues.

The planner runs in its own thread and hands over (phase, arm, Motion) as they come.  Each
motion is sent to the checker at once, in a pool of processes (one check takes 0.1 to 0.6 s),
from where that arm's previous motion ends.  The verdicts come back in any order; the motions
are queued strictly in the order the planner produced them.  A phase is written to the job's
phase list when its first motion arrives, and its queues are closed when the next phase
begins or the planner is done, so the arms can start while the planner still works.

A motion the checker refuses is not queued; the rest of that arm's motions in the phase are
dropped and its queue is closed as cut short (the coordinator then ends the job after the
phase).  What they would have drawn is left over as "failed_check".

The phase the arms run in (the one written to the job) lists every arm that may move in it:
in a leader phase, the leaders and their row partners (the followers, which draw against
their leader's footprint).  A follower with nothing to do gets an empty queue and holds.  Each
motion is checked in the phase its planner planned it in: a leader's or a fill arm's own
phase; a follower's view of the leader phase with its leader's footprint.  That context is
saved next to the queue (`<queue>.check.npz`) whenever it is not simply the named phase, so
`aris check` can check the queue again.
"""
from __future__ import annotations

import io
import json
import queue as queue_mod
import threading
import time
from collections import deque
from concurrent.futures import Future, ProcessPoolExecutor
from dataclasses import dataclass, field
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from aris.check import check
from aris.system import Report as SystemReport
from aris.system import phase_named
from aris.system import plan as system_plan
from aris.system.phases import follower_phase, is_fill
from aris.types import Field, Phase, Refusal, Wall


@dataclass
class Outcome:
    """What planning and checking did."""
    leftovers: list = field(default_factory=list)    # the planner's Leftover list
    refusal: Refusal | None = None                   # the planner refused the drawing
    error: str = ""                                  # the planner raised (a bug)
    checked: int = 0
    passed: int = 0
    tightest: float = float("inf")                   # m beyond the demanded clearance
    tightest_at: str = ""
    cut_pieces: list = field(default_factory=list)   # (Piece, why): refused or dropped draws
    stopped: bool = False
    planning_s: float = 0.0                          # wall, until the planner was done
    motions: int = 0                                 # handed over by the planner


# --------------------------------------------------------------------------- phases


def execution_phase(rig, name: str) -> Phase:
    """The phase as the arms run it: in a leader phase the followers may move too."""
    ph = phase_named(rig, name)
    if is_fill(ph):
        return ph
    partners = tuple(rig.row_partner(a) for a in ph.active if rig.row_partner(a) in ph.parked)
    return Phase(ph.name, ph.active + partners,
                 tuple(a for a in ph.parked if a not in partners), ph.walls)


def check_context(rig, name: str, arm: int, system_report) -> tuple[Phase, tuple]:
    """The Phase and footprints one arm's motion is checked in."""
    ph = phase_named(rig, name)
    if arm in ph.active:
        return ph, ()
    fld = (system_report.fields if system_report is not None else {}).get((name, arm))
    return follower_phase(rig, ph, arm), (() if fld is None else (fld,))


def phase_json(p: Phase) -> dict:
    return dict(name=p.name, active=list(p.active), parked=list(p.parked),
                walls=[dict(name=w.name, arms=list(w.arms), point=w.point_table.tolist(),
                            normal=w.normal_table.tolist()) for w in p.walls])


def phase_from(d: dict) -> Phase:
    return Phase(d["name"], tuple(d["active"]), tuple(d["parked"]),
                 tuple(Wall(w["name"], tuple(w["arms"]), np.array(w["point"]),
                            np.array(w["normal"])) for w in d["walls"]))


def save_context(path, phase: Phase, fields) -> None:
    arrays = {}
    meta = dict(phase=phase_json(phase), fields=[])
    for k, f in enumerate(fields):
        meta["fields"].append(dict(name=f.name, cell=f.cell, margin=f.margin))
        arrays[f"origin{k}"], arrays[f"dist{k}"] = f.origin_base, f.dist
    buf = io.BytesIO()
    np.savez(buf, meta=np.array(json.dumps(meta)), **arrays)
    Path(path).write_bytes(buf.getvalue())


def load_context(path) -> tuple[Phase, tuple]:
    with np.load(path, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"]))
        fields = tuple(Field(f["name"], z[f"origin{k}"], f["cell"], z[f"dist{k}"], f["margin"])
                       for k, f in enumerate(meta["fields"]))
    return phase_from(meta["phase"]), fields


def context_path(queue) -> Path:
    return queue.path.with_suffix(".check.npz")


# --------------------------------------------------------------------------- checking


def check_one(args):
    """One motion through the checker; runs in a worker process."""
    config_dir, arm, motion, phase, q_before, fields = args
    return check(config_dir, arm, motion, phase, q_before, fields=fields)


def check_pool(workers: int):
    """A pool of checker processes, or None for checking in this thread."""
    if workers <= 1:
        return None
    return ProcessPoolExecutor(workers, mp_context=get_context("spawn"))


def submit(pool, args) -> Future:
    if pool is not None:
        return pool.submit(check_one, args)
    f = Future()
    f.set_result(check_one(args))
    return f


def note_verdict(out: Outcome, v, where: str) -> None:
    out.checked += 1
    out.passed += bool(v.passed)
    mc = float(v.min_clearance)
    if np.isfinite(mc) and mc < out.tightest:
        out.tightest, out.tightest_at = mc, f"{where}: {v.min_clearance_at}"


# --------------------------------------------------------------------------- the stream


def _pump(st, lines, arm_configs, rep, stop, box) -> None:
    """The planner, in its own thread: every item goes into `box`, then ("end", value)."""
    try:
        gen = system_plan(st.rig, lines, st.rules, arm_configs, st.cache_dir, st.workers,
                          st.settings, rep)
        while True:
            if stop.is_set():
                gen.close()
                box.put(("end", None))
                return
            try:
                item = next(gen)
            except StopIteration as e:
                box.put(("end", e.value))
                return
            box.put(("motion", item))
    except Exception as e:                          # a bug; reported, never swallowed
        box.put(("error", f"{type(e).__name__}: {e}"))


class _Queuer:
    """Verdicts in, motions queued in the planner's order."""

    def __init__(self, job, rec, out: Outcome, on_first):
        self.job, self.rec, self.out, self.on_first = job, rec, out, on_first
        self.pending, self.queues, self.cut, self.index = deque(), {}, {}, {}
        self.first = False

    def add(self, name, arm, motion, fut) -> None:
        self.pending.append((name, arm, motion, fut))

    def drain(self, wait: bool) -> None:
        while self.pending and (wait or self.pending[0][3].done()):
            name, arm, motion, fut = self.pending.popleft()
            self._queue(name, arm, motion, fut.result())

    def _queue(self, name, arm, motion, v) -> None:
        key = (name, arm)
        i = self.index.get(key, 0)
        self.index[key] = i + 1
        note_verdict(self.out, v, f"{name}, arm {arm}, motion {i}")
        if key in self.cut:
            if motion.piece is not None:
                self.out.cut_pieces.append((motion.piece, f"{name}, arm {arm}: {self.cut[key]}"))
            return
        q = self.queues.get(key)
        if q is None:
            q = self.queues[key] = self.job.queue(name, arm)
        res = q.append(motion, v)
        if isinstance(res, Refusal):
            why = f"motion {i} refused ({res.reason}: {res.detail})"
            self.cut[key] = why
            with self.rec.lock:
                self.rec.refused.append((name, arm, i, why))
            self.rec.log.write("motion refused", phase=name, arm=arm, index=i, why=why,
                               failed=list(v.failed))
            if motion.piece is not None:
                self.out.cut_pieces.append((motion.piece, f"{name}, arm {arm}: {why}"))
            return
        self.rec.count_queued(name, arm)
        if not self.first:
            self.first = True
            self.on_first()

    def close_phase(self, phase: Phase | None, note: str = "") -> None:
        if phase is None:
            return
        for a in phase.active:
            q = self.queues.get((phase.name, a)) or self.job.queue(phase.name, a)
            why = self.cut.get((phase.name, a), note)
            q.close(complete=not why, note=why)


def plan_into(st, job, lines, arm_configs, rec, on_first=lambda: None) -> Outcome:
    """Plans `lines`, checks every motion and queues it in `job`.  Returns when the planner
    is done or `rec.stop` is set; the job's phase list is ended either way."""
    out, rep = Outcome(), SystemReport()
    rec.system = rep
    box: queue_mod.Queue = queue_mod.Queue()
    stop = threading.Event()
    t0 = time.perf_counter()
    pump = threading.Thread(target=_pump, args=(st, lines, arm_configs, rep, stop, box),
                            daemon=True, name=f"planner {rec.id}")
    pump.start()
    pool = check_pool(st.check_workers)
    qr = _Queuer(job, rec, out, on_first)
    where = {a: np.asarray(q, float) for a, q in (arm_configs or {}).items()}
    phase, saved = None, set()
    try:
        while True:
            if rec.stop.is_set():
                out.stopped = True
                break
            try:
                kind, x = box.get(timeout=0.05)
            except queue_mod.Empty:
                qr.drain(wait=False)
                continue
            if kind == "error":
                out.error = x
                break
            if kind == "end":
                if isinstance(x, Refusal):
                    out.refusal = x
                elif x is not None:
                    out.leftovers = list(x)
                break
            name, arm, motion = x
            out.motions += 1
            if phase is None or name != phase.name:
                qr.drain(wait=True)
                qr.close_phase(phase)
                phase = execution_phase(st.rig, name)
                job.add_phase(phase)
            q_before = where.get(arm, st.rig.park_q(arm))
            ph, fields = check_context(st.rig, name, arm, rep)
            if (name, arm) not in saved and arm not in phase_named(st.rig, name).active:
                save_context(context_path(job.queue(name, arm)), ph, fields)
                saved.add((name, arm))
            qr.add(name, arm, motion, submit(pool, (st.config_dir, arm, motion, ph, q_before,
                                                    fields)))
            where[arm] = motion.q_end
            qr.drain(wait=False)
        out.planning_s = time.perf_counter() - t0
        if out.stopped:
            qr.pending.clear()
        else:
            qr.drain(wait=True)
        note = "stopped" if out.stopped else ("planner failed" if out.error else "")
        qr.close_phase(phase, note)
    finally:
        stop.set()
        if pool is not None:
            pool.shutdown(wait=not out.stopped, cancel_futures=True)
        job.end_phases("stopped" if out.stopped else "the planner is done")
    return out
