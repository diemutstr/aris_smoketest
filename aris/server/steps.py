"""Jobs that are a short list of planned motions (park, calibrate): the arms where they stand,
lifting pens at the paper, checking a step, queueing the steps, running them.

`queue_steps` writes the steps' motions into the job, phase by phase in the order the steps
come, each with its checker verdict (or the one the motion carries); `run_queued` runs the job:
on the simulated arms with the coordinator here, or, with `--driver robot`, by waiting for the
operator PC's runner to report the end.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.check import check
from aris.execute import Coordinator
from aris.kernel.footprint import footprint, transform_field
from aris.sequencer.guard import Guard
from aris.sequencer.lift import lift
from aris.sequencer.tour import TourOptions
from aris.system.phases import cannot_touch
from aris.system.planner import AT_PARK
from aris.types import Capsule, Phase, Trajectory



CELL = 0.01              # m, the footprint grid of an arm standing where it is
LIFT_EXTRA = 0.005       # m above the pen clearance: how far a pen at the paper rises first
PEN_DOWN = 0.005         # m: a tip closer to the paper than the pen's clearance plus this
                         # rises before it moves on


@dataclass(frozen=True)
class Step:
    """One arm's motions in one phase, with the checker's verdicts, or why there are none."""
    arm: int
    phase: Phase | None = None
    motions: tuple = ()
    verdicts: tuple = ()
    fields: tuple = ()
    why: str = ""            # "" when every motion passed; else why the arm stays


def at_park(rig, a, q) -> bool:
    """The arm stands at its park (to the system planner's tolerance)."""
    return float(np.max(np.abs(np.asarray(q, float) - rig.park_q(a)))) <= AT_PARK


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
    of while the arms in `moving` move and the rest stand (parked arms at their parks, the
    others as their bodies for the planners and their footprints for the checker).  Each
    standing arm's footprint is made once and kept."""

    def __init__(self, rig):
        self.rig = rig
        self.margin = rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]
        self.prints = {}           # (arm, q bytes) -> its footprint standing there, own frame

    def _print(self, b, q):
        key = (b, np.asarray(q, float).tobytes())
        if key not in self.prints:
            stand = Trajectory(np.array([0.0, 1.0]), np.stack([q, q]), np.zeros((2, 7)))
            self.prints[key] = footprint(self.rig.arm(b), [stand], cell=CELL, name=f"arm{b}",
                                         margin=self.margin)
        return self.prints[key]

    def of(self, a, now, moving, walls, name):
        """-> (obstacles for the planners, fields for the checker, Phase)."""
        rig = self.rig
        near = [b for b in rig.arm_ids if b not in moving and not cannot_touch(rig, a, b)]
        off = [b for b in near if not at_park(rig, b, now[b])]
        # arms that cannot touch a are listed as parked wherever they stand: the checker then
        # measures a against them at their parks, which a cannot reach either
        parked = tuple(b for b in rig.arm_ids if b not in moving and b not in off)
        fields = tuple(transform_field(self._print(b, now[b]),
                                       rig.T_base_table(a) @ rig.T_table_base(b)) for b in off)
        mine = tuple(w for w in walls if a in w.arms)
        obs = rig.obstacles(a, tuple(b for b in near if b not in off), mine)
        obs = replace(obs, capsules=obs.capsules + tuple(
            c for b in off for c in standing_capsules(rig, a, b, now[b], self.margin)))
        return obs, fields, Phase(name, tuple(moving), parked, tuple(walls))


def checked_step(st, a, motions, phase, q, fields) -> Step:
    """One arm's motions through the checker, one after another from q: a Step, with why
    when any fails."""
    verdicts = []
    for m in motions:
        verdicts.append(check(st.config_dir, a, m, phase, q, fields=fields))
        q = m.q_end
    bad = [f"{m.kind}: " + ", ".join(v.failed) for m, v in zip(motions, verdicts)
           if not v.passed]
    return Step(a, phase, tuple(motions), tuple(verdicts), fields,
                "checker: " + "; ".join(bad) if bad else "")


def lift_walls(rig, down) -> tuple | str:
    """The walls the pens-down arms rise behind, or why they cannot rise together."""
    if len(down) == 1 or all(cannot_touch(rig, a, b) for a in down for b in down if a < b):
        return ()
    for n in (1, 2):
        ph = rig.phase(n)
        if set(down) <= set(ph.active):
            return tuple(w for w in ph.walls if set(w.arms) <= set(down))
    return f"pens down on arms {sorted(down)}, which never draw at the same time"


def lift_pens(st, scene, now, down) -> list[Step]:
    """The arms in `down` (pens at the paper) raise their pens straight up together, in one
    phase "lift pens" behind their walls (the sequencer's lift-off rule, LIFT_EXTRA above the
    pen's clearance): one checked Step per arm."""
    rig, rules = st.rig, st.rules
    walls = lift_walls(rig, down)
    if isinstance(walls, str):
        return [Step(a, why=walls) for a in down]
    steps = []
    for a in down:
        obs, fields, phase = scene.of(a, now, tuple(down), walls, "lift pens")
        arm = rig.arm(a)
        opt = TourOptions()
        up = lift(arm, Guard(arm, obs, rules.gates), rig.paper(a, for_planning=True), now[a],
                  rules, LIFT_EXTRA, opt.lift_step, opt.lift_jump, opt.lift_turns)
        if isinstance(up, str):
            steps.append(Step(a, phase, why=f"the pen is at the paper and cannot rise: {up}"))
            continue
        steps.append(checked_step(st, a, [up.up], phase, now[a], fields))
    return steps


def queue_steps(job, rec, steps, note: str) -> bool:
    """-> whether anything moves.  Steps with a `why` are not queued."""
    from aris.server.pipeline import context_path, save_context
    moving = [s for s in steps if s.motions and not s.why]
    for name in dict.fromkeys(s.phase.name for s in moving):        # phases in order
        these = [s for s in moving if s.phase.name == name]
        job.add_phase(these[0].phase)
        for arm in dict.fromkeys(s.arm for s in these):
            q = job.queue(name, arm)
            for s in (s for s in these if s.arm == arm):
                for m, v in zip(s.motions, s.verdicts):
                    q.append(m, v)
                    rec.count_queued(name, arm)
                if s.fields:
                    save_context(context_path(q), s.phase, s.fields)
            q.close()
    job.end_phases(note)
    return bool(moving)


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
