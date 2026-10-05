"""The mark job (`aris mark`): steps 2 and 3 of the calibration (DESIGN.md section 6), by
hand-guiding, as `docs/figures/mark_protocol.png` draws it.

One arm at a time, in rig order, the others standing (parked, or as their joints where they
stand).  For each arm, its marks (the group's marks this slot shares, rig.json order): at its
first mark four hand orientations (the pivot: pen upright and three leans of 20 degrees), at
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

from aris.check import check
from aris.execute import Coordinator
from aris.execute.queue import verdict_numbers
from aris.free import plan as free_plan
from aris.sequencer.guard import Guard
from aris.server.steps import Scene, lift_pens, pen_down
from aris.types import Motion, Piece, Refusal, Trajectory

HOVER = 0.030                    # m, the pen above the mark at the hover
LEAN = np.deg2rad(20.0)
LEANS = ((0.0, 0.0), (LEAN, 0.0), (-LEAN / 2, LEAN * np.sqrt(3) / 2),
         (-LEAN / 2, -LEAN * np.sqrt(3) / 2))
SPINS = np.arange(24) * np.deg2rad(15.0)
Q7_TRIES = np.deg2rad([0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0])
POLL = 0.05                      # s between looks at the event log
WAYPOINT_UP = 0.08               # m: turning the pen near the paper goes by way of this high


@dataclass(frozen=True)
class Task:
    mark: str
    orientation: int             # 0 upright; 1..3 the pivot's leans

    @property
    def lean(self):
        return LEANS[self.orientation]


def tasks_for(rig, slot, slots) -> list[Task]:
    """This slot's touches: four orientations at its first mark, one at each other mark."""
    marks = [m for m in rig.marks_for(slots) if slot in rig.marks[m][1]]
    out = []
    for k, m in enumerate(marks):
        out += [Task(m, o) for o in (range(4) if k == 0 else (0,))]
    return out


@dataclass
class ArmPlan:
    """What one arm needs to plan its motions: the scene as it stands."""
    st: object
    slot: str
    obs: object
    standing: dict
    phase: object

    def __post_init__(self):
        rig = self.st.rig
        self.arm = rig.arm(self.slot)
        self.rules = self.st.rules
        self.guard = Guard(self.arm, self.obs, self.rules.gates)
        self.paper = rig.paper(self.slot, for_planning=True)

    def _tip_base(self, mark, height):
        rig = self.st.rig
        xy = rig.mark_xy(mark)
        T = rig.T_base_table(self.slot)
        return T[:3, :3] @ np.array([xy[0], xy[1], rig.paper_z + height]) + T[:3, 3]

    def hover(self, task: Task, q_near) -> np.ndarray | None:
        """A hover for this mark and orientation: over the hand spins (15 degrees apart) and
        arm shapes whose hover and seated poses pass the gates, the one nearest `q_near` (the
        arm keeps its shape from touch to touch, so the moves between them stay short)."""
        arm, g, gates = self.arm, self.guard, self.rules.gates
        up, seat = self._tip_base(task.mark, HOVER), self._tip_base(task.mark, 0.0)
        lean = np.array([task.lean])
        q7s = self.st.rig.park_q(self.slot)[6] + Q7_TRIES
        best = None
        for spin in SPINS:
            Th = arm.hand_pose(up[None], self.paper.normal, np.array([spin]), lean)
            Ts = arm.hand_pose(seat[None], self.paper.normal, np.array([spin]), lean)
            Qh, okh = arm.ik(np.repeat(Th, len(q7s), axis=0), q7s)
            Qs, oks = arm.ik(np.repeat(Ts, len(q7s), axis=0), q7s)
            for i in range(len(q7s)):
                for b in np.flatnonzero(okh[i] & oks[i]):
                    h, s = Qh[i, b], Qs[i, b]
                    if (arm.limit_margin(np.stack([h, s])).min() < gates.limit_margin
                            or arm.sigma_min(np.stack([h, s])).min() < gates.sigma_min
                            or g.hold(h, touching=False) is not None
                            or g.hold(s, touching=True) is not None):
                        continue
                    d = float(np.linalg.norm(h - q_near))
                    if best is None or d < best[0]:
                        best = (d, h)
        return None if best is None else best[1]

    def go_to(self, q_from, task: Task) -> list | str:
        """[free move to the hover (with its verdict), guide] or why not."""
        h = self.hover(task, q_from)
        if h is None:
            return f"no hand pose seats the pen on mark {task.mark} (orientation " \
                   f"{task.orientation}) inside the gates"
        legs = self._way(q_from, h)
        if isinstance(legs, str):
            return f"no way to the hover of {task.mark}: {legs}"
        out, q = [], q_from
        for m in legs:
            v = check(self.st.config_dir, self.slot, m, self.phase, q, standing=self.standing)
            if not v.passed:
                return f"the way to the hover of {task.mark} fails the checker: " + \
                    ", ".join(v.failed)
            out.append((m, v))
            q = m.q_end
        # the guide stands at the hover, which the checker held as the last move's end
        g = Motion("guide", Trajectory(np.zeros(1), q[None], np.zeros((1, 7))),
                   Piece(task.mark, 0.0, 0.0), self._tip_base(task.mark, 0.0)[None],
                   checked=dict(verdict_numbers(out[-1][1]), tightest="the hover, held at "
                                "the free move's end"))
        return out + [(g, None)]

    def _up(self, q, rise: float):
        """The same hand pose `rise` higher along the paper normal, the nearest answer."""
        arm = self.arm
        n = np.asarray(self.paper.normal, float) / np.linalg.norm(self.paper.normal)
        T = arm.fk(np.asarray(q, float)[None])[0]
        T[:3, 3] += rise * n
        q7s = q[6] + np.linspace(-0.3, 0.3, 7)
        Q, ok = arm.ik(np.repeat(T[None], len(q7s), axis=0), q7s)
        flat, good = Q.reshape(-1, 7), ok.reshape(-1)
        order = np.argsort(np.where(good, np.linalg.norm(np.nan_to_num(flat - q, nan=1e9),
                                                         axis=1), np.inf))
        for k in order:
            if not good[k]:
                break
            if self.guard.hold(flat[k], touching=False) is None:
                return flat[k]
        return None

    def _way(self, q_from, h) -> list | str:
        """Free moves from q_from to h: straight there, or, close to the paper where turning
        the pen is cramped, up WAYPOINT_UP first, across, and down to the hover."""
        m = free_plan(self.arm, q_from, h, self.obs, self.rules, seed_extra=b"mark")
        if not isinstance(m, Refusal):
            return [m]
        a, b = self._up(q_from, WAYPOINT_UP), self._up(h, WAYPOINT_UP)
        if a is None or b is None:
            return f"{m.reason}: {m.detail}"
        legs, q = [], q_from
        for goal in (a, b, h):
            leg = free_plan(self.arm, q, goal, self.obs, self.rules, seed_extra=b"mark via")
            if isinstance(leg, Refusal):
                return f"{leg.reason}: {leg.detail} (also by way of {WAYPOINT_UP * 1e3:.0f} mm up)"
            legs.append(leg)
            q = leg.q_end
        return legs

    def home(self, q_from) -> list | str:
        """[(free move, verdict), ...] back to the park, or why not."""
        legs = self._way(q_from, self.st.rig.park_q(self.slot))
        if isinstance(legs, str):
            return f"no way home: {legs}"
        out, q = [], q_from
        for m in legs:
            v = check(self.st.config_dir, self.slot, m, self.phase, q, standing=self.standing)
            if not v.passed:
                return "the way home fails the checker: " + ", ".join(v.failed)
            out.append((m, v))
            q = m.q_end
        return out


@dataclass
class Book:
    """What the job has gathered."""
    touches: list = field(default_factory=list)    # {slot, mark, orientation, q, index,
                                                   #  skipped, button}
    notes: list = field(default_factory=list)
    buttons: dict = field(default_factory=dict)    # button -> count
    redone: list = field(default_factory=list)


def plan_arm(st, slot, now, tasks, name) -> tuple | str:
    """One arm's whole sequence, planned and checked up front: (a lift if its pen is at the
    paper,) for each task a free move to its hover and the guide, then home.  A task with no
    hover inside the gates or no way there is dropped and noted.
    -> (phase, standing, [(motion, verdict)], {queue index: Task}, notes) or why not."""
    rig = st.rig
    obs, standing, phase = Scene(rig).of(slot, now, (slot,), (), name)
    ap, items, at, notes = ArmPlan(st, slot, obs, standing, phase), [], {}, []
    cur = np.asarray(now[slot], float)
    if pen_down(rig, slot, cur):                     # a pen at the paper rises first
        up = lift_pens(st, Scene(rig), now, [slot])[0]
        if up.why:
            return up.why
        items += list(zip(up.motions, up.verdicts))
        cur = up.motions[-1].q_end
    for t in tasks:
        go = ap.go_to(cur, t)
        if isinstance(go, str):
            notes.append(f"{slot}: {go}")
            continue
        items += go
        at[len(items) - 1] = t
        cur = go[-1][0].q_end
    home = ap.home(cur)
    if isinstance(home, str):
        return home
    return phase, standing, items + home, at, notes


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


def run_arm(st, rec, job, slot, now, slots, book, solve) -> str:
    """One arm: its phase, its own solve, at most one extra phase for a touch the solver names
    as bad.  -> "" or why the job ends."""
    plan = plan_arm(st, slot, now, tasks_for(st.rig, slot, slots), f"mark {slot}")
    if isinstance(plan, str):
        return plan
    book.notes += plan[4]
    touches = run_phase(st, rec, job, slot, plan)
    if isinstance(touches, str):
        return touches
    bad = _bad_touch(solve, st.rig, [t for t in touches if not t["skipped"]])
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


def _bad_touch(solve, rig, touches) -> int | None:
    """The touch the solver names as bad among this arm's own (the pivot), or None."""
    if len(touches) < 2:
        return None
    try:
        sol = solve(rig, touches, {})
    except Exception:                                # the arm's own solve is advice only
        return None
    bad = getattr(sol, "bad_touch", None)
    return None if bad is None else int(bad)


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


def submit_mark(st, store, slots=(), group: str | None = None):
    """Admit and start the mark job (refused while another job runs, with the robot when an
    arm never reported where it stands, and for slots or a group the rig does not have)."""
    from aris.server import runner
    chosen = group_slots(st.rig, tuple(slots), group)
    if isinstance(chosen, Refusal):
        return chosen
    return runner.start(st, store, "mark", "mark " + " ".join(chosen), _work(chosen),
                        lambda rec: dict(slots=list(chosen), group=group),
                        need_positions=True)


def _work(slots):
    def work(st, rec, job):
        from aris.calib import solve_marks
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
            why = run_arm(st, rec, job, s, now, slots, book, solve_marks)
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
        finish(st, rec, job, slots, book, run, why, before, solve_marks)
    return work
