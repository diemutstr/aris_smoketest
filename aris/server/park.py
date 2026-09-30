"""Park all arms: every mounted arm to its park configuration from where it stands, checked,
queued and run.

1. Pens at the paper first.  An arm stopped in the middle of drawing has its pen on the paper,
   and every phase-end check fails while any pen is down.  So every such arm first raises its
   pen straight up (the sequencer's lift-off rule and turns, `LIFT_EXTRA` above the pen's
   clearance), all together in one
   phase, "lift pens", behind the walls they were drawing behind (the arms that were drawing
   at a stop are one phase's arms, or arms that cannot touch each other).
2. Then one arm at a time, in rig order, each in its own phase ("park 13", ...): a free motion
   to its park.  No two arms move at once and no walls are needed.  An arm already at its park
   is left alone.

While an arm moves, every other arm stands still: those at their parks are parked arms as in
any phase; one that is not parked stands where it is, and is given to the free-space planner
as its body at that configuration and to the checker as its footprint (a distance field; the
checker knows parked arms only at their parks).  An arm the planners or the checker refuse
stays where it is, and the arms after it are planned around it there.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.check import check
from aris.free import plan as free_plan
from aris.kernel.footprint import footprint, transform_field
from aris.sequencer.guard import Guard
from aris.sequencer.lift import lift
from aris.sequencer.tour import TourOptions
from aris.system.phases import cannot_touch
from aris.system.planner import AT_PARK
from aris.types import Capsule, Phase, Refusal, Trajectory

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
    return float(np.max(np.abs(np.asarray(q, float) - rig.park_q(a)))) <= AT_PARK


def pen_down(rig, a, q) -> bool:
    """The pen is within its clearance of the paper (rig.json `pen_lifted_to_paper_m` and its
    planning allowance), give or take PEN_DOWN: it must rise before a free motion."""
    paper = rig.paper(a, for_planning=True)
    clear = paper.pen_margin if paper.pen_margin is not None else paper.margin
    tip = rig.to_table(a, rig.arm(a).tip(np.asarray(q, float)[None])[0])
    return float(tip[2] - rig.paper_z) < clear + PEN_DOWN


def _standing_capsules(rig, a, b, q_b, margin) -> tuple:
    """Arm b's body at q_b, in a's base frame (the free-space planner's view)."""
    body = rig.arm(b).body(np.asarray(q_b, float)[None, :])
    T = rig.T_base_table(a) @ rig.T_table_base(b)
    R, t = T[:3, :3], T[:3, 3]
    return tuple(Capsule(f"arm{b}:{n}", R @ body.p0[0, k] + t, R @ body.p1[0, k] + t,
                         float(body.radius[k]), margin) for k, n in enumerate(body.names))


class _Scene:
    """What arm a must keep clear of while the arms in `moving` move and the rest stand."""

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
            c for b in off for c in _standing_capsules(rig, a, b, now[b], self.margin)))
        return obs, fields, Phase(name, tuple(moving), parked, tuple(walls))


def _checked(st, a, motions, phase, q, fields) -> Step:
    verdicts = []
    for m in motions:
        verdicts.append(check(st.config_dir, a, m, phase, q, fields=fields))
        q = m.q_end
    bad = [f"{m.kind}: " + ", ".join(v.failed) for m, v in zip(motions, verdicts)
           if not v.passed]
    return Step(a, phase, tuple(motions), tuple(verdicts), fields,
                "checker: " + "; ".join(bad) if bad else "")


def _lift_walls(rig, down) -> tuple | str:
    """The walls the pens-down arms rise behind, or why they cannot rise together."""
    if len(down) == 1 or all(cannot_touch(rig, a, b) for a in down for b in down if a < b):
        return ()
    for n in (1, 2):
        ph = rig.phase(n)
        if set(down) <= set(ph.active):
            return tuple(w for w in ph.walls if set(w.arms) <= set(down))
    return f"pens down on arms {sorted(down)}, which never draw at the same time"


def _lift_pens(st, scene, now, down) -> list[Step]:
    rig, rules = st.rig, st.rules
    walls = _lift_walls(rig, down)
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
        steps.append(_checked(st, a, [up.up], phase, now[a], fields))
    return steps


def plan_park(st, where: dict) -> list[Step]:
    """Steps in the order they run: the pens-down arms' lifts (one phase), then one park phase
    per arm that is not at its park.  `where`: arm id -> where it stands now."""
    rig, rules = st.rig, st.rules
    now = {a: np.asarray(q, float) for a, q in where.items()}
    scene, steps = _Scene(rig), []
    down = [a for a in rig.arm_ids if not at_park(rig, a, now[a]) and pen_down(rig, a, now[a])]
    stuck = set()                                  # pens that stay down
    if down:
        lifts = _lift_pens(st, scene, now, down)
        if all(not s.why for s in lifts):          # all rise, or none: one phase end check
            for s in lifts:
                now[s.arm] = s.motions[-1].q_end
        else:
            lifts = [s if s.why else replace(s, motions=(), why="another pen cannot rise")
                     for s in lifts]
            stuck = set(down)
        steps += lifts
    for a in rig.arm_ids:
        if at_park(rig, a, now[a]):
            steps.append(Step(a, why="already at its park"))
            continue
        if a in stuck:
            continue                               # its pen could not rise; it stays
        obs, fields, phase = scene.of(a, now, (a,), (), f"park {a}")
        m = free_plan(rig.arm(a), now[a], rig.park_q(a), obs, rules, seed_extra=b"park")
        if isinstance(m, Refusal):
            steps.append(Step(a, phase, why=f"free-space planner: {m.reason}: {m.detail}"))
            continue
        s = _checked(st, a, [m], phase, now[a], fields)
        steps.append(s)
        if not s.why:
            now[a] = rig.park_q(a)
    return steps
