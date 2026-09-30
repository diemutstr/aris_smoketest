"""Park all arms: every mounted arm to its park configuration by a free motion from where it
stands, checked, queued and run.

One arm at a time, in rig order, each in its own phase ("park 13", ...), so that no two arms
ever move at once and no walls are needed.  An arm already at its park is left alone.  While
arm a moves, every other arm stands still: those already at their park are parked arms as in
any phase; one that is not yet parked stands where it is, and is given to the free-space
planner as its body at that configuration and to the checker as its footprint (a distance
field, since the checker only knows parked arms at their parks).  An arm the planner or the
checker refuses stays where it is, and the arms after it are planned around it there.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.free import plan as free_plan
from aris.kernel.footprint import footprint, transform_field
from aris.server.pipeline import guarded_check
from aris.system.phases import cannot_touch
from aris.system.planner import AT_PARK
from aris.types import Capsule, Motion, Phase, Refusal, Trajectory

CELL = 0.01              # m, the footprint grid of an arm standing where it is


@dataclass(frozen=True)
class Step:
    arm: int
    phase: Phase | None = None
    motion: Motion | None = None
    verdict: object = None
    fields: tuple = ()
    why: str = ""            # "" when the motion passed; else why the arm stays


def at_park(rig, a, q) -> bool:
    return float(np.max(np.abs(np.asarray(q, float) - rig.park_q(a)))) <= AT_PARK


def _standing_capsules(rig, a, b, q_b, margin) -> tuple:
    """Arm b's body at q_b, in a's base frame (the free-space planner's view)."""
    body = rig.arm(b).body(np.asarray(q_b, float)[None, :])
    T = rig.T_base_table(a) @ rig.T_table_base(b)
    R, t = T[:3, :3], T[:3, 3]
    return tuple(Capsule(f"arm{b}:{n}", R @ body.p0[0, k] + t, R @ body.p1[0, k] + t,
                         float(body.radius[k]), margin) for k, n in enumerate(body.names))


def plan_park(st, where: dict) -> list[Step]:
    """One Step per arm, in rig order.  `where`: arm id -> where it stands now."""
    rig, rules = st.rig, st.rules
    now = {a: np.asarray(q, float) for a, q in where.items()}
    margin = rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]
    prints = {}                    # arm -> its footprint standing where it is, own frame
    steps = []
    for a in rig.arm_ids:
        if at_park(rig, a, now[a]):
            steps.append(Step(a, why="already at its park"))
            continue
        near = [b for b in rig.arm_ids if b != a and not cannot_touch(rig, a, b)]
        off = [b for b in near if not at_park(rig, b, now[b])]
        # arms that cannot touch a are listed as parked wherever they stand: the checker then
        # measures a against them at their parks, which a cannot reach either
        parked = tuple(b for b in rig.arm_ids if b != a and b not in off)
        fields = []
        for b in off:
            if b not in prints:
                stand = Trajectory(np.array([0.0, 1.0]), np.stack([now[b], now[b]]),
                                   np.zeros((2, 7)))
                prints[b] = footprint(rig.arm(b), [stand], cell=CELL, name=f"arm{b}",
                                      margin=margin)
            fields.append(transform_field(prints[b],
                                          rig.T_base_table(a) @ rig.T_table_base(b)))
        obs = rig.obstacles(a, tuple(b for b in near if b not in off), ())
        obs = replace(obs, capsules=obs.capsules + tuple(
            c for b in off for c in _standing_capsules(rig, a, b, now[b], margin)))
        phase = Phase(f"park {a}", (a,), parked, ())
        m = free_plan(rig.arm(a), now[a], rig.park_q(a), obs, rules, seed_extra=b"park")
        if isinstance(m, Refusal):
            steps.append(Step(a, phase, why=f"free-space planner: {m.reason}: {m.detail}"))
            continue
        v = guarded_check(st.config_dir, a, m, phase, now[a], tuple(fields))
        if isinstance(v, str):
            steps.append(Step(a, phase, m, why=v))
            continue
        if not v.passed:
            steps.append(Step(a, phase, m, v, tuple(fields),
                              why="checker: " + ", ".join(v.failed)))
            continue
        steps.append(Step(a, phase, m, v, tuple(fields)))
        now[a] = rig.park_q(a)
        prints.pop(a, None)
    return steps
