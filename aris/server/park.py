"""Park all arms: every mounted arm to its park configuration from where it stands, checked,
queued and run.

1. Pens at the paper first.  An arm stopped in the middle of drawing has its pen on the paper,
   and every phase-end check fails while any pen is down.  So every such arm first raises its
   pen straight up (the sequencer's lift-off rule and turns, `LIFT_EXTRA` above the pen's
   clearance), all together in one
   phase, "lift pens", behind the walls they were drawing behind (the arms that were drawing
   at a stop are one phase's arms, or arms that cannot touch each other).
2. Then one arm at a time, in rig order, each in its own phase ("park 1L", ...): a free motion
   to its park.  No two arms move at once and no walls are needed.  An arm already at its park
   is left alone.

While an arm moves, every other arm stands still: those at their parks are parked arms as in
any phase; one that is not parked stands where it is, and is given to the free-space planner
as its body at that configuration and to the checker as its footprint (a distance field; the
checker knows parked arms only at their parks).  An arm the planners or the checker refuse
stays where it is, and the arms after it are planned around it there.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.free import plan as free_plan
from aris.server.steps import (LIFT_EXTRA, Scene, Step, at_park, checked_step, lift_pens,
                               pen_down)
from aris.types import Refusal

__all__ = ["plan_park", "Step", "at_park", "pen_down", "LIFT_EXTRA"]


def plan_park(st, where: dict) -> list[Step]:
    """Steps in the order they run: the pens-down arms' lifts (one phase), then one park phase
    per arm that is not at its park.  `where`: arm id -> where it stands now."""
    rig, rules = st.rig, st.rules
    now = {a: np.asarray(q, float) for a, q in where.items()}
    scene, steps = Scene(rig), []
    down = [a for a in rig.arm_ids if not at_park(rig, a, now[a]) and pen_down(rig, a, now[a])]
    stuck = set()                                  # pens that stay down
    if down:
        lifts = lift_pens(st, scene, now, down)
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
        s = checked_step(st, a, [m], phase, now[a], fields)
        steps.append(s)
        if not s.why:
            now[a] = rig.park_q(a)
    return steps
