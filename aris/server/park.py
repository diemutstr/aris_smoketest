"""Park all arms: every mounted arm to its park configuration from where it stands, checked,
queued and run.

1. Pens at the paper first.  An arm stopped in the middle of drawing has its pen on the paper,
   and every phase-end check fails while any pen is down.  So every such arm first raises its
   pen straight up (the sequencer's lift-off rule and turns, `LIFT_EXTRA` above the pen's
   clearance), one arm per phase ("lift pens 1L", ...), in rig order, the others standing.
2. Then one arm at a time, in rig order, each in its own phase ("park 1L", ...): a free motion
   to its park.  No two arms move at once and no walls are needed.  An arm already at its park
   is left alone.

While an arm moves, every other arm stands still: those at their parks are parked arms as in
any phase; one that is not parked stands where it is, and is given to the free-space planner
as its body at that configuration and to the checker as its joints (`standing`; the checker
builds the body itself).  An arm the planners or the checker refuse
stays where it is, and the arms after it are planned around it there.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.free import plan as free_plan
from aris.server.jobs import arm_progress
from aris.server.steps import (LIFT_EXTRA, Scene, Step, checked_step, lift_pens,
                               pen_down, steps_work)
from aris.types import Refusal

__all__ = ["submit_park", "plan_park", "Step", "pen_down", "LIFT_EXTRA"]


def plan_park(st, where: dict) -> list[Step]:
    """Steps in the order they run: the pens-down arms' lifts (see steps.lift_pens), then one park phase
    per arm that is not at its park.  `where`: arm id -> where it stands now."""
    rig = st.rig
    from aris.server.retreat import retreats
    now = {a: np.asarray(q, float) for a, q in where.items()}
    scene = Scene(rig)
    steps, now = retreats(st, now)                 # arms within the clearance move apart first
    stuck = {s.arm for s in steps if s.why}        # arms that stay where they are
    down = [a for a in rig.arm_ids if not rig.at_park(a, now[a]) and pen_down(rig, a, now[a])
            and a not in stuck]
    if down:
        lifts = lift_pens(st, scene, now, down)    # one phase per pen, in rig order
        for s in lifts:
            if s.why:
                stuck.add(s.arm)                   # its pen stays down; it does not park
            else:
                now[s.arm] = s.motions[-1].q_end
        steps += lifts
    for a in rig.arm_ids:
        if rig.at_park(a, now[a]):
            steps.append(Step(a, why="already at its park"))
            continue
        if a in stuck:
            continue                               # its pen could not rise; it stays
        obs, standing, phase = scene.of(a, now, (a,), (), f"park {a}")
        m = free_plan(rig.arm(a), now[a], rig.park_q(a), obs, st.rules_for(a),
                      seed_extra=b"park")
        if isinstance(m, Refusal):
            steps.append(Step(a, phase, why=f"free-space planner: {m.reason}: {m.detail}"))
            continue
        s = checked_step(st, a, [m], phase, now[a], standing)
        steps.append(s)
        if not s.why:
            now[a] = rig.park_q(a)
    return steps


# --------------------------------------------------------------------------- the job


@dataclass(frozen=True)
class ParkPlan:
    steps: list
    why: str = ""


def submit_park(st, store):
    """Admit and start the park job (refused while another job runs, and with the robot when
    an arm never reported where it stands)."""
    from aris.server import runner
    plan = lambda st_, where: ParkPlan(plan_park(st_, where))
    return runner.start(st, store, "park", "park all arms", steps_work(plan, _report),
                        need_positions=True)


def _end(st, rec, steps, run) -> tuple[str, str]:
    if rec.stop.is_set():
        return "stopped", "stop requested"
    tol = st.rig.execution().start_tolerance if st.remote else 1e-6
    left = [f"arm {a}" for a, q in run.where.items()
            if float(np.max(np.abs(np.asarray(q) - st.rig.park_q(a)))) > tol]
    if run.status != "done":
        return "failed", run.why
    if left:
        return "failed", "not parked: " + ", ".join(left) + "; " + "; ".join(
            f"arm {s.arm}: {s.why}" for s in steps
            if s.why and s.why != "already at its park")
    return "done", ""


def _arms(st, steps, run) -> dict:
    """Per arm: parked, already at its park, or why not; its motions and how long they take."""
    out = {}
    for a in st.rig.arm_ids:
        mine = [s for s in steps if s.arm == a]
        why = next((s.why for s in mine if s.why), "")
        ms = [m for s in mine if not s.why for m in s.motions]
        out[str(a)] = dict(result=why or ("parked" if ms else "already at its park"),
                           motions=[m.kind for m in ms],
                           motion_s=sum(float(m.traj.t[-1] - m.traj.t[0]) for m in ms),
                           at_park=a in run.where and bool(st.rig.at_park(a, run.where[a])))
    return out


def _report(st, rec, plan, run, planning_s):
    steps = plan.steps
    state, why = _end(st, rec, steps, run)
    checked = [v for s in steps for v in s.verdicts]
    rows, first = arm_progress(rec.log.read())
    rep = dict(state=state, why=why, kind="park", arms=_arms(st, steps, run),
               checker=dict(checked=len(checked), passed=sum(bool(v.passed) for v in checked),
                            tightest_clearance_m=min((float(v.min_clearance) for v in checked),
                                                     default=None)),
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               planning_s=planning_s,
               first_motion_s=None if first is None else first - rec.t_received,
               assumptions=st.assumptions())
    return rep, state, why
