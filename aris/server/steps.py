"""Jobs that are a short list of planned motions (park, calibrate): the arms where they stand,
lifting pens at the paper, checking a step, queueing the steps, running them.

`queue_steps` writes the steps' motions into the job, phase by phase in the order the steps
come, each with its checker verdict (or the one the motion carries); `run_queued` runs the job:
on the simulated arms with the coordinator here, or, with `--driver robot`, by waiting for the
operator PC's runner to report the end.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

from aris.check import check
from aris.execute import Coordinator
from aris.sequencer.guard import Guard
from aris.kernel.retime import retime_detailed
from aris.sequencer.lift import LANDING_DT, landing_cap, lift, rise_path
from aris.sequencer.tour import TourOptions
from aris.system.phases import cannot_touch
from aris.types import Capsule, JointPath, Motion, Phase, Refusal

LIFT_EXTRA = 0.005       # m above the pen clearance: how far a pen at the paper rises first
PEN_DOWN = 0.005         # m: a tip closer to the paper than the pen's clearance plus this
                         # rises before it moves on


@dataclass(frozen=True)
class Step:
    """One arm's motions in one phase, with the checker's verdicts, or why there are none."""
    arm: str
    phase: Phase | None = None
    motions: tuple = ()
    verdicts: tuple = ()
    standing: dict = field(default_factory=dict)   # {slot: joints} standing off their parks
    why: str = ""            # "" when every motion passed; else why the arm stays


def pen_down(rig, a, q) -> bool:
    """The pen is within its clearance of the paper (rig.json `pen_lifted_to_paper_m` and its
    planning allowance), give or take PEN_DOWN: it must rise before a free motion."""
    paper = rig.paper(a, for_planning=True)
    clear = paper.pen_margin if paper.pen_margin is not None else paper.margin
    tip = rig.to_table(a, rig.arm(a).tip(np.asarray(q, float)[None])[0])
    return float(tip[2] - rig.paper_z) < clear + PEN_DOWN


def standing_capsules(rig, a, b, q_b, margin) -> tuple:
    """Arm b's body at q_b, in a's base frame (the free-space planner's view)."""
    body = rig.arm(b).body(np.asarray(q_b, float)[None, :])
    T = rig.T_base_table(a) @ rig.T_table_base(b)
    R, t = T[:3, :3], T[:3, 3]
    return tuple(Capsule(f"arm{b}:{n}", R @ body.p0[0, k] + t, R @ body.p1[0, k] + t,
                         float(body.radius[k]), margin) for k, n in enumerate(body.names))


class Scene:
    """The arms where they stand, as one moving arm must see them: what arm a must keep clear
    of while the arms in `moving` move and the rest stand (parked arms at their parks; the
    others where they stand, as their bodies for the planners and as `standing` joints for the
    checker, which builds their bodies itself)."""

    def __init__(self, rig):
        self.rig = rig
        self.margin = rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]

    def of(self, a, now, moving, walls, name):
        """-> (obstacles for the planners, standing {slot: joints} for the checker, Phase)."""
        rig = self.rig
        still = [b for b in rig.arm_ids if b not in moving]
        standing = {b: np.asarray(now[b], float) for b in still if not rig.at_park(b, now[b])}
        near = [b for b in still if not cannot_touch(rig, a, b)]
        obs = rig.obstacles(a, tuple(b for b in near if b not in standing),
                            tuple(w for w in walls if a in w.arms))
        obs = replace(obs, capsules=obs.capsules + tuple(
            c for b in near if b in standing
            for c in standing_capsules(rig, a, b, now[b], self.margin)))
        return obs, standing, Phase(name, tuple(moving), tuple(still), tuple(walls))


def checked_step(st, a, motions, phase, q, standing) -> Step:
    """One arm's motions through the checker, one after another from q: a Step, with why
    when any fails."""
    verdicts = []
    for m in motions:
        verdicts.append(check(st.config_dir, a, m, phase, q, standing=standing))
        q = m.q_end
    bad = [f"{m.kind}: " + ", ".join(v.failed) for m, v in zip(motions, verdicts)
           if not v.passed]
    return Step(a, phase, tuple(motions), tuple(verdicts), dict(standing),
                "checker: " + "; ".join(bad) if bad else "")


RECOVERY_LIMIT_MARGIN = 0.075   # rad: a pen stopped near a joint limit may rise this close


def lift_pens(st, scene, now, down) -> list[Step]:
    """The arms in `down` (pens at the paper) raise their pens straight up (the sequencer's
    lift-off rule, LIFT_EXTRA above the pen's clearance), ONE ARM PER PHASE, always: "lift pens
    <slot>" in rig order, every other arm standing (the phase-end check accepts a standing pen
    at the paper that a later phase raises).  A pen stopped between the surface and its
    clearance is first set down onto the surface.  One checked Step per arm; each later lift
    sees the earlier ones risen, or standing where they were if they could not.  A stop can
    leave an arm closer to a joint limit than planning would choose; if the planning gates
    refuse the rise, it is tried once more with the joint-limit margin halved (the checker
    still holds every motion to the arm's real limits)."""
    rig = st.rig
    now = {b: np.asarray(q, float) for b, q in now.items()}
    steps = []
    for a in (b for b in rig.arm_ids if b in down):
        step = _lift_one(st, scene.of(a, now, (a,), (), f"lift pens {a}"), now[a], a)
        steps.append(step)
        if not step.why:
            now[a] = step.motions[-1].q_end
    return steps


def _lift_one(st, scene_of, q, a) -> Step:
    obs, standing, phase = scene_of
    rig, rules, got = st.rig, st.rules_for(a), None
    for gates in (rules.gates, replace(rules.gates, limit_margin=RECOVERY_LIMIT_MARGIN)):
        got = _rise_from(rig, a, obs, q, replace(rules, gates=gates))
        if not isinstance(got, str):
            break
    if isinstance(got, str):
        return Step(a, phase, why=got)
    return checked_step(st, a, got, phase, q, standing)


def _rise_from(rig, a, obs, q, rules) -> list | str:
    """[set-down (if needed), lift-off] for one arm, or why not."""
    arm, opt = rig.arm(a), TourOptions()
    guard, paper = Guard(arm, obs, rules.gates), rig.paper(a, for_planning=True)
    setdown = to_surface(arm, guard, paper, q, rules, opt)
    if isinstance(setdown, str):
        return f"the pen is near the paper and cannot be set down to rise from it: {setdown}"
    q_low = q if setdown is None else setdown.q_end
    up = lift(arm, guard, paper, q_low, rules, LIFT_EXTRA, opt.lift_step, opt.lift_jump,
              opt.lift_turns)
    if isinstance(up, str):
        return f"the pen is at the paper and cannot rise: {up}"
    return ([] if setdown is None else [setdown]) + [up.up]


SURFACE_TOL = 0.0005     # m: a tip this close to the drawing surface is on it


def to_surface(arm, guard, paper, q, rules, opt):
    """A pen stopped between the drawing surface and its clearance (a stop in the middle of a
    set-down or a lift-off) is first set down onto the surface, straight along the paper
    normal, the last 5 mm before the surface at the landing speed (as `lift._lower`), so that the lift-off starts where a lift-off starts (the
    checker holds it to that).  -> the "lower" Motion, None when the pen is on the surface
    already, or why not."""
    q = np.asarray(q, float)
    nn = float(np.linalg.norm(paper.normal))
    n = np.asarray(paper.normal, float) / nn
    tip = arm.tip(q[None])[0]
    above_paper = float(np.asarray(paper.normal, float) @ tip - paper.offset) / nn
    drop = above_paper + float(getattr(rules, "press", 0.0))       # down to the surface
    if drop <= SURFACE_TOL:
        return None
    T = arm.fk(q[None])[0]
    T[:3, 3] -= drop * n
    Q, ok = arm.ik(T[None], np.array([q[6]]))
    if not ok[0].any():
        return "no arm configuration puts the pen on the drawing surface below it"
    d = np.where(ok[0], np.linalg.norm(np.nan_to_num(Q[0] - q, nan=1e9), axis=1), np.inf)
    q_surface = Q[0, int(np.argmin(d))]
    path = rise_path(arm, guard, q_surface, paper, above_paper, opt.lift_step, opt.lift_jump,
                     rules.gates)
    if isinstance(path, str):
        return path
    if float(np.max(np.abs(path[-1] - q))) > 1e-6:
        return "the way down does not start where the arm stands"
    path = path[::-1].copy()
    path[0] = q                                     # exactly where the arm stands
    tips = arm.tip(path)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))])
    res = retime_detailed(JointPath(path), arm.limits, rules, s=s, smooth=True, tip_of=arm.tip,
                          speed_cap=landing_cap(s[-1], rules.landing_speed), knot_dt=LANDING_DT)
    if isinstance(res, Refusal):
        return f"the set-down cannot be timed: {res.reason} {res.detail}"
    if guard.flown(res.traj, touching=True) < 0.0:
        return "the set-down comes too close"
    return Motion("lower", res.traj)


def steps_work(plan, report):
    """The work of a job that is a short list of planned steps (park, calibrate, touch-off),
    for `runner.start`: where every arm stands, `plan(st, where)` (-> an object with `.steps`
    and `.why`; a why fails the job, with `plan.failed()` extra report fields if it has them),
    the steps queued and run, and `report(st, rec, plan, run, planning_s) -> (report, state,
    why)`."""
    def work(st, rec, job) -> None:
        import time
        from aris.server import runner
        where = runner.where_now(st, need_all=True)
        if isinstance(where, Refusal):
            return runner.fail_job(st, rec, job, where.detail)
        rec.set_state("planning")
        t0 = time.perf_counter()
        p = plan(st, where)
        planning_s = time.perf_counter() - t0
        if p.why:
            extra = p.failed() if hasattr(p, "failed") else {}
            return runner.fail_job(st, rec, job, p.why, **extra)
        moving = queue_steps(job, rec, p.steps, f"{rec.kind} planned")
        if moving:
            rec.set_state("moving")
        run = run_queued(st, rec, job, moving, where)
        rep, state, why = report(st, rec, p, run, planning_s)
        runner.finish_job(rec, job.dir, rep, state, why)
    return work


def add_steps(job, rec, steps) -> bool:
    """The steps' phases and queues added to the job (the phase list left open for more).
    -> whether anything moves.  Steps with a `why` are not queued."""
    moving = [s for s in steps if s.motions and not s.why]
    for name in dict.fromkeys(s.phase.name for s in moving):        # phases in order
        these = [s for s in moving if s.phase.name == name]
        job.add_phase(these[0].phase, these[0].standing)     # the same for every arm in it
        for arm in dict.fromkeys(s.arm for s in these):
            q = job.queue(name, arm)
            for s in (s for s in these if s.arm == arm):
                for m, v in zip(s.motions, s.verdicts):
                    q.append(m, v)
                    rec.count_queued(name, arm)
            q.close()
    return bool(moving)


def queue_steps(job, rec, steps, note: str) -> bool:
    """`add_steps`, then the phase list ended.  -> whether anything moves."""
    moving = add_steps(job, rec, steps)
    job.end_phases(note)
    return moving


def run_queued(st, rec, job, moving: bool, where: dict):
    """The coordinator's JobRun for the queued job (the robot's, read from its rows)."""
    from aris.server.runner import robot_run, wait_robot
    if st.remote:                        # the operator PC runs it and reports
        run = wait_robot(rec) if moving else robot_run(rec)
        if not moving:
            run.status, run.why = "done", ""
        run.where = {**where, **run.where}
        return run
    coord = Coordinator(job, st.drivers, st.config_dir, st.rig)
    rec.coordinator = coord
    if rec.stop.is_set():
        coord.stop()
    return coord.run()
