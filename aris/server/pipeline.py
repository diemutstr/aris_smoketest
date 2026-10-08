"""From the system planner's stream into the job's queues.

The system planner checks every motion itself, inside each arm planner's loop, with the
`verify` the server hands it (verify.py): a piece is drawn only if every motion of its group
passes, else it is left over as "failed_check" and the arm plans on.  So every motion that
arrives here carries the checker's word (`Motion.checked`) and is queued at once, in the order
it came (the queue refuses anything unchecked; that would be a bug, and ends the job).

The planner runs in its own thread.  A phase is written to the job's phase list when its first
motion arrives, and its queues are closed when the next phase begins or the planner is done,
so the arms can start while the planner still works.
"""
from __future__ import annotations

import queue as queue_mod
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

from aris.server.verify import CheckVerify
from aris.system import Report as SystemReport
from aris.system import phase_named
from aris.system import plan as system_plan
from aris.types import Phase, Refusal


@dataclass
class Outcome:
    """What planning did."""
    leftovers: list = field(default_factory=list)    # the planner's Leftover list
    refusal: Refusal | None = None                   # the planner refused the drawing
    error: str = ""                                  # the planner raised, or a queue refused
    stopped: bool = False
    planning_s: float = 0.0                          # wall, until the planner was done
    motions: int = 0                                 # handed over by the planner


# --------------------------------------------------------------------------- the stream


def job_rules(st, rec, slot=None):
    """The drawing rules of this job (of the arm in `slot`: its pen's press and speed), with
    the drawing surface raised `air_mm` above the paper for an air run (a negative press: every
    draw flown that high, no contact)."""
    air = float(getattr(rec, "air_mm", 0.0) or 0.0)
    r = st.rules if slot is None else st.rules_for(slot)
    return r if air <= 0.0 else replace(r, press=-air * 1e-3)


def rules_by_slot(st, rec) -> dict | None:
    """{slot: rules} when the arms carry different pens (the rig keeps pens per slot), for the
    system planner's per-arm planners; None when one rule set holds for all."""
    per = {a: job_rules(st, rec, a) for a in st.rig.arm_ids}
    return per if len({repr(r) for r in per.values()}) > 1 else None


def surface_z(st, rec) -> float | None:
    """The drawing surface's table height the checker holds the pen to on an air run; None:
    the checker's own (the paper less the pen's press)."""
    air = float(getattr(rec, "air_mm", 0.0) or 0.0)
    return None if air <= 0.0 else st.rig.paper_z + air * 1e-3


def _pump(st, lines, arm_configs, rep, stop, box, verify, rules, by_slot=None) -> None:
    """The planner, in its own thread: every item goes into `box`, then ("end", value)."""
    try:
        kw = {} if st.surface is None else dict(surface=st.surface)
        if by_slot is not None:
            kw["rules_by_slot"] = by_slot
        gen = system_plan(st.rig, lines, rules, arm_configs, st.cache_dir, st.workers,
                          st.settings, rep, verify, **kw)
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
    verify = CheckVerify(Path(st.config_dir), job.dir / "refused", surface_z(st, rec))
    pump = threading.Thread(target=_pump, args=(st, lines, arm_configs, rep, stop, box, verify,
                                                job_rules(st, rec), rules_by_slot(st, rec)),
                            daemon=True, name=f"planner {rec.id}")
    pump.start()
    phase, queues = None, {}
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
                phase = phase_named(st.rig, name)
                job.add_phase(phase)
            q = queues.get((name, arm)) or queues.setdefault((name, arm), job.queue(name, arm))
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
