"""The drawing motion of one alternative plan of the local planner, in one direction.

The local planner hands a joint path with the pen on the line, and only the duration of its
timing.  Here the whole path is timed in one call (the timing step handles the corners of the
line and keeps the pen at the draw speed), the pen kept within 0.1 mm of the planned pen path,
and the result is checked as flown.  One piece, one drawing motion.
"""
from __future__ import annotations

import numpy as np

from aris.kernel.retime import retime_detailed
from aris.local import reverse_plan
from aris.sequencer.guard import Guard
from aris.types import DrawPlan, DrawRules, JointPath, Motion, Refusal


def oriented(plan: DrawPlan, backwards: bool) -> DrawPlan:
    return reverse_plan(plan) if backwards else plan


def draw_motions(arm, guard: Guard, plan: DrawPlan, rules: DrawRules, deviation: float,
                 intensity: float = 1.0) -> list[Motion] | str:
    """[the timed drawing motion of `plan` (as oriented)], or why it cannot be flown."""
    along = np.abs(plan.s - plan.s[0])          # arc length in the direction of drawing
    res = retime_detailed(JointPath(plan.q), arm.limits, rules, s=along, tip_of=arm.tip,
                          deviation=deviation)
    if isinstance(res, Refusal):
        return f"cannot be timed: {res.reason} {res.detail}"
    flown = guard.flown(res.traj, touching=True)
    if flown < 0.0:
        return f"the timed drawing comes {-flown * 1e3:.2f} mm too close"
    # The planned pen positions at the timed samples: the check measures the pen against these.
    tip = np.column_stack([np.interp(res.s, along, plan.tip_base[:, i]) for i in range(3)])
    return [Motion("draw", res.traj, piece=plan.piece, tip_base=tip, intensity=intensity)]
