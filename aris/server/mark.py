"""The mark job (`aris mark`): steps 2 and 3 of the calibration (DESIGN.md section 6), by
hand-guiding, as `docs/figures/mark_protocol.png` draws it.

One arm at a time, in rig order, the others standing (parked, or as their joints where they
stand).  For each arm, its marks (the group's marks this slot shares, rig.json order): at its
first mark six hand orientations (the pivot: pen upright and five tilts of 30 degrees, 72
degrees apart; turning about the vertical alone leaves the pen length free), at
every other mark one.  The whole sequence is planned and checked up front and queued at once:
a free move to each hover (the pen 30 mm above the mark), the `guide` there, and home.  At a
guide the driver hands the arm to the person, who seats the pen on the mark and presses a
pilot button; the executor writes the "registered" row (joints, button, mark); the driver
then lifts the pen 3 cm and returns to the hover itself, so a guide ends where it began.
  ✓ check   the touch counts;
  ✗ cross   never reaches the server: the driver waits for the next button;
  ○ circle  the touch is marked skipped for the solver; the rest runs as planned.
A guide that fails (the hand-over itself) fails the job; the arm holds where it is.  When the
arm's phase has run, the solver looks at this arm's touches (the pivot's residuals): a touch
it names as bad gets one small extra phase (to that hover, the guide, home).  At the end the
joint solve over every arm of the group, with the marks solved before as known; when it
passes, each slot's `base` (method "marks") and `pen` parts and calibration/marks.json are
written and the rig reloads; when it fails, the report names the slot or touch and nothing is
written.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np

from aris.execute import Coordinator
from aris.server.mark_plan import Task, choose_pivot, plan_arm, tasks_for
from aris.types import Refusal

__all__ = ["submit_mark", "group_slots", "solver_input", "Task", "tasks_for"]

POLL = 0.05                      # s between looks at the event log


@dataclass
class Book:
    """What the job has gathered."""
    touches: list = field(default_factory=list)    # {slot, mark, orientation, q, index,
                                                   #  skipped, button}
    notes: list = field(default_factory=list)
    buttons: dict = field(default_factory=dict)    # button -> count
    redone: list = field(default_factory=list)


def run_phase(st, rec, job, slot, plan) -> list | str:
    """Queue an arm's planned phase whole, wait until the arm has run it, and gather its
    touches from the registered rows.  -> the touches, or why the job ends."""
    phase, standing, items, at, _ = plan
    job.add_phase(phase, standing)
    q = job.queue(phase.name, slot)
    for m, v in items:
        res = q.append(m, v)
        if isinstance(res, Refusal):
            q.close(complete=False, note=res.detail)
            return f"the queue refused a {m.kind}: {res.reason}: {res.detail}"
        rec.count_queued(phase.name, slot)
    q.close()
    if rec.state != "moving":
        rec.set_state("moving")
    while True:
        rows = [r for r in rec.log.read() if r.get("arm") == slot and r.get("phase") == phase.name]
        end = next((r for r in rows if r.get("event") in ("finished", "failed", "stopped")), None)
        if end is not None:
            break
        if rec.stop.is_set():
            return "stop requested"
        time.sleep(POLL)
    if end["event"] != "finished":
        return f"{phase.name}: {end.get('why')}"
    touches = []
    for r in rows:
        if r.get("event") == "registered" and r.get("index") in at:
            t = at[r["index"]]
            touches.append(dict(slot=slot, mark=t.mark, orientation=t.orientation,
                                q=list(map(float, r["q"])), index=r["index"],
                                skipped=str(r.get("button")) == "circle",
                                button=str(r.get("button"))))
    return touches


def run_arm(st, rec, job, slot, now, slots, book) -> str:
    """One arm: its phase, its own solve, at most one extra phase for a touch the solver names
    as bad.  -> "" or why the job ends."""
    plan = plan_arm(st, slot, now, choose_pivot(st, slot, now, slots), f"mark {slot}")
    if isinstance(plan, str):
        return plan
    book.notes += plan[4]
    touches = run_phase(st, rec, job, slot, plan)
    if isinstance(touches, str):
        return touches
    bad = _bad_touch(st.rig, slot, [t for t in touches if not t["skipped"]])
    if bad is not None:
        kept = [t for t in touches if not t["skipped"]]
        t = kept[bad]
        book.redone.append(f"{slot}: {t['mark']} orientation {t['orientation']}")
        again = plan_arm(st, slot, {**now, slot: st.rig.park_q(slot)},
                         [Task(t["mark"], t["orientation"])], f"mark {slot} again")
        if isinstance(again, str):
            return again
        more = run_phase(st, rec, job, slot, again)
        if isinstance(more, str):
            return more
        touches = [x for x in touches if x is not t] + more
    for t in touches:
        book.buttons[t["button"]] = book.buttons.get(t["button"], 0) + 1
        if t["skipped"]:
            book.notes.append(f"{slot}: {t['mark']} orientation {t['orientation']} skipped")
    book.touches += touches
    return ""


def _bad_touch(rig, slot, touches) -> int | None:
    """The pivot touch the solver names as off the common point (the pen slipped), as an index
    into `touches`, or None."""
    from aris.calib.marks import pivot
    piv = [k for k, t in enumerate(touches) if t["mark"] == touches[0]["mark"]] if touches \
        else []
    if len(piv) < 3:
        return None
    pv = pivot(rig, slot, np.array([touches[k]["q"] for k in piv]))
    if pv.passed or not len(pv.residuals) or "off the common point" not in pv.why:
        return None
    return piv[int(np.argmax(pv.residuals))]


def solver_input(touches) -> dict:
    """{slot: {mark: (K,7)}} for `aris.calib.solve_marks`, skipped touches left out, each
    slot's first mark (its pivot) first."""
    out = {}
    for t in touches:
        if not t["skipped"]:
            out.setdefault(t["slot"], {}).setdefault(t["mark"], []).append(t["q"])
    return {s: {m: np.asarray(q, float) for m, q in by.items()} for s, by in out.items()}


# --------------------------------------------------------------------------- the job


def group_slots(rig, slots=(), group: str | None = None):
    """The slots of a mark job: the ones named, or a group's (default "all", or every
    controlled slot when the rig has no such group), in rig order."""
    if slots:
        bad = [s for s in slots if s not in rig.arm_ids]
        if bad:
            return Refusal("no_slot", f"slots {bad} are not controlled on this rig")
        return tuple(s for s in rig.arm_ids if s in slots)
    name = group or "all"
    if name in rig.mark_groups:
        return tuple(s for s in rig.arm_ids if s in rig.mark_groups[name])
    if group is None:
        return tuple(rig.arm_ids)
    return Refusal("no_group", f"no mark group {group!r}; groups are {tuple(rig.mark_groups)}")


def needs_solved_marks(rig, slots) -> str:
    """Why the job cannot start: a slot touching no mark at all, or one that touches marks
    shared with slots outside the job (a single slot) while none of them has a solved place
    yet ("" when it can).  Checked before anything moves; the solver never gets an empty set."""
    from aris.server.mark_plan import slot_marks
    for s in slots:
        marks = slot_marks(rig, s, slots)
        if not marks:
            return f"{s} shares no calibration mark with a controlled slot"
        outside = [m for m in marks if not set(rig.marks[m][1]) <= set(slots)]
        if outside and not any(rig.mark_state(m) == "solved" for m in marks):
            partners = sorted({p for m in outside for p in rig.marks[m][1]} | set(slots))
            return (f"{s} alone needs marks solved by an earlier run; run `aris mark` "
                    f"({', '.join(partners)}) first")
    return ""


def submit_mark(st, store, slots=(), group: str | None = None):
    """Admit and start the mark job (refused while another job runs, with the robot when an
    arm never reported where it stands, and for slots or a group the rig does not have)."""
    from aris.server import runner
    chosen = group_slots(st.rig, tuple(slots), group)
    if isinstance(chosen, Refusal):
        return chosen
    why = needs_solved_marks(st.rig, chosen)
    if why:
        return Refusal("needs_solved_marks", why)
    return runner.start(st, store, "mark", "mark " + " ".join(chosen), _work(chosen),
                        lambda rec: dict(slots=list(chosen), group=group),
                        need_positions=True)


def _work(slots):
    def work(st, rec, job):
        from aris.server import runner
        where = runner.where_now(st, need_all=True)
        if isinstance(where, Refusal):
            return runner.fail_job(st, rec, job, where.detail)
        before = {s: (st.rig.T_table_base(s), np.asarray(st.rig.arm(s).tool.tip_hand))
                  for s in slots}
        result, ct = [], None
        if not st.remote:
            coord = rec.coordinator = Coordinator(job, st.drivers, st.config_dir, st.rig)
            ct = threading.Thread(target=lambda: result.append(coord.run()), daemon=True)
            ct.start()
        rec.set_state("planning")
        book, now, why = Book(), {a: np.asarray(q, float) for a, q in where.items()}, ""
        for s in slots:
            why = run_arm(st, rec, job, s, now, slots, book)
            if why:
                break
            now[s] = st.rig.park_q(s)
        job.end_phases(why or "every arm touched its marks")
        if ct is None:
            run = runner.wait_robot(rec)
        else:
            if rec.stop.is_set():
                rec.coordinator.stop()
            ct.join()
            run = result[0]
        from aris.server.markreport import finish
        finish(st, rec, job, slots, book, run, why, before)
    return work
