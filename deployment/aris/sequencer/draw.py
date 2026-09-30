"""The drawing motions of one alternative plan of the local planner, in one direction.

The local planner hands a joint path with the pen on the line, and only the duration of its
timing.  Here the whole path is timed in one call (the timing step handles the corners of the
line), the pen kept within 0.1 mm of the planned pen path, and the result is checked as flown.

Where the timed pen comes (almost) to a stop anyway (a line that doubles back), and at every
corner sharper than `cut_angle`, the drawing is cut into two motions, each starting and ending
at rest with the pen on the paper: a drawing motion never stops in its middle (the checker's
rule "never stops").  At a sharp corner the pen rounds the corner within 0.1 mm and slows
down a lot anyway; there the checker's reading of the speed along the line, which follows the
nearest point of the line, falls back for an instant, so such a corner is made a stop.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.kernel.retime import retime_detailed, sample
from aris.local import reverse_plan
from aris.sequencer.guard import Guard
from aris.types import DrawPlan, DrawRules, JointPath, Motion, Refusal

SLOW = 0.10           # of the draw speed: a slower pen in the middle of a motion is a stop
GOING = 0.25          # of the draw speed: the pen has got going (as the checker reads it)
OVER = 0.01           # of the draw speed: the most the flown pen may go faster than asked
RATE = 1000.0         # Hz, the pen speed is read like the driver reads the joints


def oriented(plan: DrawPlan, backwards: bool) -> DrawPlan:
    return reverse_plan(plan) if backwards else plan


def draw_motions(arm, guard: Guard, plan: DrawPlan, rules: DrawRules, deviation: float,
                 cut_angle: float, intensity: float = 1.0) -> list[Motion] | str:
    """The timed drawing motions of `plan` (as oriented), in order, or why it cannot be flown."""
    along = np.abs(plan.s - plan.s[0])          # arc length in the direction of drawing
    res = _timed(arm, plan.q, along, rules, deviation)
    if isinstance(res, str):
        return res
    cuts = sorted(set(_stops(res, along, rules.draw_speed))
                  | set(corners(plan.tip_base, cut_angle).tolist()))
    bounds = [0, *cuts, len(plan.q) - 1]
    parts = [res] if not cuts else [_timed(arm, plan.q[a:b + 1], along[a:b + 1], rules,
                                           deviation) for a, b in zip(bounds[:-1], bounds[1:])]
    out = []
    for part in parts:
        if isinstance(part, str):
            return part
        flown = guard.flown(part.traj, touching=True)
        if flown < 0.0:
            return f"the timed drawing comes {-flown * 1e3:.2f} mm too close"
        # The planned pen positions at the timed samples: the check measures the pen against
        # these.
        tip = np.column_stack([np.interp(part.s, along, plan.tip_base[:, i]) for i in range(3)])
        out.append(Motion("draw", part.traj, piece=plan.piece, tip_base=tip,
                          intensity=intensity))
    return out


def corners(tip: np.ndarray, angle: float) -> np.ndarray:
    """Sample indices (never the ends) where the pen path turns by more than `angle`."""
    seg = np.diff(tip, axis=0)
    seg = seg / np.maximum(np.linalg.norm(seg, axis=1, keepdims=True), 1e-12)
    turn = np.arccos(np.clip(np.sum(seg[1:] * seg[:-1], axis=1), -1.0, 1.0))
    return np.flatnonzero(turn > angle) + 1


def pen_speed(arm, traj) -> float:
    """The fastest pen tip speed on the flown curve, read at `RATE`."""
    t = np.linspace(0.0, traj.t[-1], max(2, int(np.ceil(traj.t[-1] * RATE)) + 1))
    tip = arm.tip(sample(traj, t)[0])
    return float(np.max(np.linalg.norm(np.diff(tip, axis=0), axis=1) / np.diff(t)))


def _timed(arm, q, along, rules, deviation):
    """Timed, with the pen no faster than the draw speed (+ OVER) as flown.  The timing step
    holds the speed along the path; between its samples the flown cubic can run a few per cent
    faster where the pen speeds up or slows down, so then the draw speed asked is lowered by
    that much and the path is timed again."""
    asked = rules
    for _ in range(3):
        res = retime_detailed(JointPath(q), arm.limits, asked, s=along - along[0],
                              tip_of=arm.tip, deviation=deviation)
        if isinstance(res, Refusal):
            return f"cannot be timed: {res.reason} {res.detail}"
        fastest = pen_speed(arm, res.traj)
        if fastest <= rules.draw_speed * (1.0 + OVER):
            break
        asked = replace(asked, draw_speed=asked.draw_speed * rules.draw_speed / fastest)
    else:
        return f"the pen runs at {fastest * 1e3:.2f} mm/s, faster than the draw speed"
    if along[0] != 0.0:
        res = replace(res, s=res.s + along[0])
    return res


def _stops(res, along, draw_speed) -> list[int]:
    """Plan sample indices where the timed pen nearly stops between getting going and the
    final stop: the slowest point of every such dip, as the nearest sample of the plan."""
    t, s = res.traj.t, res.s
    v = np.diff(s) / np.diff(t)                               # pen speed along the line
    going = np.flatnonzero(v >= GOING * draw_speed)
    if len(going) < 2:
        return []
    slow = np.zeros(len(v), bool)
    slow[going[0]:going[-1]] = v[going[0]:going[-1]] < SLOW * draw_speed
    cuts = []
    edges = np.flatnonzero(np.diff(np.concatenate([[0], slow.astype(int), [0]])))
    for a, b in zip(edges[::2], edges[1::2]):
        k = a + int(np.argmin(v[a:b]))
        i = int(np.argmin(np.abs(along - 0.5 * (s[k] + s[k + 1]))))
        if 0 < i < len(along) - 1 and (not cuts or i > cuts[-1]):
            cuts.append(i)
    return cuts
