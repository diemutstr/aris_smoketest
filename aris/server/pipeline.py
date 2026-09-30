"""From the system planner's stream into the job's queues.

The system planner checks every motion itself, inside each arm planner's loop, with the
`verify` the server hands it (verify.py): a piece is drawn only if every motion of its group
passes, else it is left over as "failed_check" and the arm plans on.  So every motion that
arrives here carries the checker's word (`Motion.checked`) and is queued at once, in the order
it came (the queue refuses anything unchecked; that would be a bug, and ends the job).

The planner runs in its own thread.  A phase is written to the job's phase list when its first
motion arrives, and its queues are closed when the next phase begins or the planner is done,
so the arms can start while the planner still works.  The phase the arms run in lists every
arm that may move in it (`aris.system.execution_phase`: in a leader phase, the followers too;
a follower with nothing to do gets an empty queue and holds).  A follower is checked in its
own view of the phase with its leader's footprint (`aris.system.check_view`); that view is
saved next to its queue (`<queue>.check.npz`) so `aris check` can check the queue again.
"""
from __future__ import annotations

import io
import json
import queue as queue_mod
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from aris.server.verify import CheckVerify
from aris.system import Report as SystemReport
from aris.system import check_view, execution_phase, phase_named
from aris.system import plan as system_plan
from aris.types import Field, Phase, Refusal, Wall


@dataclass
class Outcome:
    """What planning did."""
    leftovers: list = field(default_factory=list)    # the planner's Leftover list
    refusal: Refusal | None = None                   # the planner refused the drawing
    error: str = ""                                  # the planner raised, or a queue refused
    stopped: bool = False
    planning_s: float = 0.0                          # wall, until the planner was done
    motions: int = 0                                 # handed over by the planner


# --------------------------------------------------------------------------- phases


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


# --------------------------------------------------------------------------- the stream


def _pump(st, lines, arm_configs, rep, stop, box, verify) -> None:
    """The planner, in its own thread: every item goes into `box`, then ("end", value)."""
    try:
        gen = system_plan(st.rig, lines, st.rules, arm_configs, st.cache_dir, st.workers,
                          st.settings, rep, verify)
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


def _close(job, queues: dict, phase: Phase | None, note: str = "") -> None:
    """End markers for every queue of the phase (the open Queue objects, so a long queue is
    not read back to find its end)."""
    if phase is not None:
        for a in phase.active:
            q = queues.get((phase.name, a)) or job.queue(phase.name, a)
            q.close(complete=not note, note=note)


def plan_into(st, job, lines, arm_configs, rec, on_first=lambda: None) -> Outcome:
    """Plans `lines` (checked inside the planner) and queues every motion in `job`.  Returns
    when the planner is done or `rec.stop` is set; the job's phase list is ended either way."""
    out, rep = Outcome(), SystemReport()
    rec.system = rep
    box: queue_mod.Queue = queue_mod.Queue()
    stop = threading.Event()
    t0 = time.perf_counter()
    verify = CheckVerify(Path(st.config_dir), job.dir / "refused")
    pump = threading.Thread(target=_pump, args=(st, lines, arm_configs, rep, stop, box, verify),
                            daemon=True, name=f"planner {rec.id}")
    pump.start()
    phase, named, saved, queues = None, None, set(), {}
    try:
        while True:
            if rec.stop.is_set():
                out.stopped = True
                break
            try:
                kind, x = box.get(timeout=0.05)
            except queue_mod.Empty:
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
                _close(job, queues, phase)
                named = phase_named(st.rig, name)
                phase = execution_phase(st.rig, named)
                job.add_phase(phase)
            q = queues.get((name, arm)) or queues.setdefault((name, arm), job.queue(name, arm))
            if (name, arm) not in saved and arm not in named.active:
                save_context(context_path(q), *check_view(st.rig, named, arm, rep))
                saved.add((name, arm))
            res = q.append(motion)
            if isinstance(res, Refusal):
                out.error = f"{name}, arm {arm}: the queue refused a motion ({res.reason}: " \
                    f"{res.detail})"
                rec.log.write("motion refused", phase=name, arm=arm, why=out.error)
                break
            rec.count_queued(name, arm)
            if out.motions == 1:
                on_first()
        out.planning_s = time.perf_counter() - t0
        _close(job, queues, phase, "stopped" if out.stopped else
               ("the planner failed" if out.error else ""))
    finally:
        stop.set()
        job.end_phases("stopped" if out.stopped else "the planner is done")
    return out
