"""The lift-off at an end of a piece, and the set-down (the same motion flown backwards).

Two rules (Pete and the orchestrator, 2026-09-30), nothing else:
1. A piece starts and ends with the pen rising straight up along the paper normal until it is
   the pen clearance plus `extra` (2 mm) above the higher of the real paper and the drawing
   surface (from a surface below the paper, the press, the rise is that much longer; from one
   above it, an air run, it is the same as from the paper), following at every `step` (2 mm) the nearest IK answer that
   passes the gates (at most `max_jump`, 0.15 rad, from the last), with the hand's spin about
   the normal and joint 7 changing evenly over the rise by the smallest amounts, each within
   0.5 rad, that let every sample pass; the timed motion is checked as flown.
   (Evenly, not step by step: turns chosen per step zigzag, and the timing then slows at every
   corner; a lift took up to 3.2 s instead of 0.3.)
2. If rule 1 fails at an end, the piece is shortened at that end by `cut_step` (1 cm) and rule 1
   is tried again, up to `max_cut` (5 cm); the part cut off is a leftover.
The pen clearance is the paper's lifted-pen margin (rig.json `pen_lifted_to_paper_m`).
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.kernel.retime import retime_detailed
from aris.sequencer.guard import Guard
from aris.types import DrawPlan, DrawRules, JointPath, Motion, Piece, Plane, Refusal, Trajectory

SAME = 1e-6           # rad: the IK answer that reproduces the drawing configuration itself


@dataclass(frozen=True)
class Lift:
    """A lift-off from a drawing configuration: `up` goes from it to `q_up`."""
    q_draw: np.ndarray
    q_up: np.ndarray
    up: Motion                        # kind "lift": q_draw -> q_up
    down: Motion                      # kind "lower": q_up -> q_draw, the same path backwards,
                                      # timed to land at `rules.landing_speed`


def lift_height(paper: Plane, extra: float) -> float:
    """How high above the real paper a lift-off ends: the pen clearance plus `extra`."""
    pen = paper.pen_margin if paper.pen_margin is not None else paper.margin
    return float(pen) + extra


def rise_path(arm, guard: Guard, q: np.ndarray, paper: Plane, height: float, step: float,
              max_jump: float, gates, spin: float = 0.0, turn7: float = 0.0):
    """Rule 1's joint path for one choice of turns: from q (first row, exactly) the pen straight
    up until its tip is `height` above the real paper (the plane `paper`; a drawing surface
    below it, the press, makes the rise that much longer), the hand turning by `spin` about the normal through the tip and joint 7 by
    `turn7`, both evenly over the rise; at every `step` the IK answer nearest the last one that
    passes the gates (joint-limit margin, singular value, clearance with the pen exempt from
    the paper), at most `max_jump` from it.  -> path or why not."""
    n = np.asarray(paper.normal, float)
    nn = np.linalg.norm(n)
    n = n / nn
    T0 = arm.fk(q[None])[0]
    tip0 = arm.tip(q[None])[0]
    # Above the higher of the real paper and the drawing surface (where the pen starts): a
    # surface below the paper (the press) makes the rise that much longer; one above it (an air
    # run) keeps the same geometry relative to the surface as a real run has to the paper.
    h0 = (float(np.asarray(paper.normal, float) @ tip0) - paper.offset) / nn
    height = height - min(h0, 0.0)
    if height <= 0.0:
        return "the pen is already above the lift height"
    k = int(np.ceil(height / step)) + 1
    rise = np.linspace(0.0, 1.0, k)
    T = np.repeat(T0[None], k, axis=0)
    T[:, :3, :3] = _about(n, spin * rise) @ T0[:3, :3]
    T[:, :3, 3] = tip0 + height * rise[:, None] * n - T[:, :3, :3] @ arm.tool.tip_hand
    Q, ok = arm.ik(T, q[6] + turn7 * rise)
    if np.min(np.where(ok[0], np.linalg.norm(np.nan_to_num(Q[0] - q, nan=1e9), axis=1),
                       np.inf)) > SAME:
        return "the IK does not reproduce the drawing configuration"
    out = [q]
    for i in range(1, k):
        dist = np.where(ok[i], np.linalg.norm(np.nan_to_num(Q[i] - out[-1], nan=1e9), axis=1),
                        np.inf)
        pick = None
        for b in np.argsort(dist, kind="stable"):
            if dist[b] > max_jump:
                break
            c = Q[i, b][None]
            if (arm.limit_margin(c)[0] >= gates.limit_margin
                    and arm.sigma_min(c)[0] >= gates.sigma_min
                    and guard.hold(c[0], touching=True) is None):
                pick = b
                break
        if pick is None:
            return f"no answer within the gates {i * height / (k - 1) * 1e3:.0f} mm up"
        out.append(Q[i, pick])
    return np.array(out)


def _about(n: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """(k,3,3) rotations by `angles` about the unit axis n (Rodrigues)."""
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    a = np.asarray(angles, float)[:, None, None]
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)


def lift(arm, guard: Guard, paper: Plane, q_draw: np.ndarray, rules: DrawRules, extra: float,
         step: float, max_jump: float, turns=((0.0, 0.0),)) -> Lift | str:
    """Rule 1 from `q_draw`: the lift-off, timed and checked; or why there is none (the reason
    of the first choice of turns, the held shape).  `turns`: (spin, joint 7) over the whole
    rise, tried in this order, smallest first."""
    first = None
    for spin, turn7 in turns:
        got = _lift(arm, guard, paper, np.asarray(q_draw, float), rules, extra, step, max_jump,
                    spin, turn7)
        if not isinstance(got, str):
            return got
        first = first or got
    return first


def _lift(arm, guard, paper, q_draw, rules, extra, step, max_jump, spin, turn7) -> Lift | str:
    path = rise_path(arm, guard, q_draw, paper, lift_height(paper, extra), step, max_jump,
                     rules.gates, spin, turn7)
    if isinstance(path, str):
        return path
    g = rules.gates
    margin = arm.limit_margin(path)
    if margin.min() < g.limit_margin:
        return f"comes {margin.min():.3f} rad from a joint limit (gate {g.limit_margin})"
    sig = arm.sigma_min(path)
    if sig.min() < g.sigma_min:
        return f"singular value {sig.min():.3f} (gate {g.sigma_min})"
    why = guard.hold(path[-1], touching=False)
    if why is not None:
        return f"cannot hold the lifted pen: {why}"
    res = retime_detailed(JointPath(path), arm.limits, rules, smooth=True)  # IK samples of a straight rise
    if isinstance(res, Refusal):
        return f"cannot be timed: {res.reason} {res.detail}"
    flown = guard.flown(res.traj, touching=True)
    if flown < 0.0:
        return f"the lift comes {-flown * 1e3:.2f} mm too close"
    down = _lower(arm, guard, path[::-1], rules)
    if isinstance(down, str):
        return down
    return Lift(q_draw=path[0], q_up=res.traj.q[-1], up=Motion("lift", res.traj),
                down=Motion("lower", down))


def _lower(arm, guard, path, rules):
    """The set-down along the lift's path backwards, timed so that the pen never goes faster
    than `rules.landing_speed`: the pen's path length along the descent is the arc length, the
    landing speed its cap (a fast landing against the controller's soft spring spikes the
    force).  -> Trajectory or why not."""
    tip = arm.tip(path)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tip, axis=0), axis=1))])
    res = retime_detailed(JointPath(path), arm.limits,
                          replace(rules, draw_speed=rules.landing_speed), s=s, smooth=True,
                          tip_of=arm.tip)
    if isinstance(res, Refusal):
        return f"the set-down cannot be timed: {res.reason} {res.detail}"
    flown = guard.flown(res.traj, touching=True)
    if flown < 0.0:
        return f"the set-down comes {-flown * 1e3:.2f} mm too close"
    return res.traj


def end_lift(arm, guard, paper: Plane, plan: DrawPlan, end: int, rules: DrawRules, opt):
    """Rule 2 around rule 1 at one end (0: the plan's first sample, 1: its last).
    -> (Lift, metres cut at that end) or why not (rule 1's reason with nothing cut)."""
    first = None
    for cut in np.arange(0.0, opt.max_cut + 1e-9, opt.cut_step):
        p = plan if cut == 0.0 else trim(plan, end, float(cut))
        if p is None or abs(p.s[-1] - p.s[0]) < rules.min_piece:
            break
        got = lift(arm, guard, paper, p.q[-1 if end else 0], rules, opt.lift_extra,
                   opt.lift_step, opt.lift_jump, opt.lift_turns)
        if not isinstance(got, str):
            return got, float(cut)
        first = first or got
    return first


def trim(plan: DrawPlan, end: int, cut: float) -> DrawPlan | None:
    """The plan without its first (end 0) or last (end 1) `cut` metres of line, cut at a
    sample; None if nothing is left."""
    s = np.asarray(plan.s, float)
    along = np.abs(s - s[0]) if end == 0 else np.abs(s - s[-1])
    idx = np.flatnonzero(along >= cut - 1e-12)
    if len(idx) < 2:
        return None
    sl = slice(idx[0], None) if end == 0 else slice(None, idx[-1] + 1)
    lo, hi = float(min(s[sl][0], s[sl][-1])), float(max(s[sl][0], s[sl][-1]))
    return DrawPlan(piece=Piece(plan.piece.line_id, lo, hi), q=plan.q[sl], s=plan.s[sl],
                    tip_base=plan.tip_base[sl], score=plan.score,
                    joint_travel=float(np.abs(np.diff(plan.q[sl], axis=0)).sum()),
                    draw_time=plan.draw_time,
                    spin=None if plan.spin is None else plan.spin[sl],
                    lean=None if plan.lean is None else plan.lean[sl])


def reverse(traj: Trajectory) -> Trajectory:
    """The same timed motion flown backwards (the cubic between two samples is symmetric)."""
    return Trajectory(t=traj.t[-1] - traj.t[::-1], q=traj.q[::-1].copy(), qd=-traj.qd[::-1])
