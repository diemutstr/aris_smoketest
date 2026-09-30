"""The escape ladder: how the pen gets off the paper at one end of a piece.

The lift only has to bring the pen into free space, where the free-space planner takes over:
the pen `required` metres off the paper (its lifted-pen margin plus a little) and the arm clear.
`rules.lift_height` (25 mm) is the preferred height when the arm can have it.  Rungs, in order,
until one gives a lift-off the free-space planner accepts as a start:
  a  straight up to the preferred height, the arm's shape held
  b  straight up as far as the gates allow, if that is at least the required height
  c  back along the line just drawn, the pen at the required height above it, up to `back`
  d  straight up with the arm's shape allowed to change within the gates: joint 7 and the
     hand's turn about the normal, and the pen drifting toward the base axis
  e  the piece shortened at that end by up to `cut` (in steps), and a to d again
The set-down at the start of a piece is the same, mirrored: the lift-off from the piece's first
configuration, flown backwards.  What a cut removes is handed back as a leftover.
"""
from __future__ import annotations

import numpy as np

from aris.sequencer.lift import Lift, along_path, finish, rise_path, straight_limit
from aris.types import DrawPlan, Piece, Plane


def required_height(paper: Plane, extra: float) -> float:
    pen = paper.pen_margin if paper.pen_margin is not None else paper.margin
    return float(pen) + extra


def escape(arm, guard, paper: Plane, Q_in: np.ndarray, rules, opt) -> Lift | str:
    """Rungs a to d from the configuration Q_in[0], the piece's drawing configurations
    following inward.  -> the first Lift that works, or the first rung's reason."""
    q = Q_in[0]
    need = required_height(paper, opt.lift_extra)
    top = rules.lift_height
    step, jump = opt.lift_step, opt.lift_jump
    first = None

    def attempt(path, how):
        nonlocal first
        got = path if isinstance(path, str) else finish(arm, guard, path, rules, how)
        if isinstance(got, str):
            first = first or f"{how}: {got}"
            return None
        return got

    got = attempt(rise_path(arm, q, paper, top, step, jump), "a straight up")
    if got:
        return got
    h, _ = straight_limit(arm, guard, paper, q, rules, top=top)
    for height in (np.floor((h - 1e-3) * 2e3) / 2e3, need):     # 1 mm below the limit
        if need <= height < top:
            got = attempt(rise_path(arm, q, paper, height, step, jump), "b straight up, lower")
            if got:
                return got
    along = along_path(arm, Q_in, paper, need, opt.lift_back, jump)
    if isinstance(along, str):
        first = first or f"c back along the line: {along}"
    else:
        path, d = along
        for stop in np.arange(need, opt.lift_back + 1e-9, opt.cut_step):
            j = int(np.searchsorted(d, stop))
            if j < len(path):
                got = attempt(path[:j + 1], "c back along the line")
                if got:
                    return got
    for inward in (0.0, *opt.lift_inward):
        for turn7, spin in opt.lift_turns:
            if inward == 0.0 and turn7 == 0.0 and spin == 0.0:
                continue                                       # that is rung a/b
            for height in (top, need):
                got = attempt(rise_path(arm, q, paper, height, step, jump, turn7, spin, inward),
                              "d shape allowed to change")
                if got:
                    return got
    return first or "no lift-off"


def trim(plan: DrawPlan, end: int, cut: float) -> DrawPlan | None:
    """The plan without its first (end 0) or last (end 1) `cut` metres of line, cut at a
    sample; None if nothing is left."""
    s = np.asarray(plan.s, float)
    along = np.abs(s - s[0]) if end == 0 else np.abs(s - s[-1])
    keep = along >= cut - 1e-12
    idx = np.flatnonzero(keep)
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


def end_lift(arm, guard, paper: Plane, plan: DrawPlan, end: int, rules, opt):
    """Rung e around a to d: -> (Lift, metres cut at that end) or why not."""
    first = None
    for cut in np.arange(0.0, opt.lift_cut + 1e-9, opt.cut_step):
        p = plan if cut == 0.0 else trim(plan, end, cut)
        if p is None or abs(p.s[-1] - p.s[0]) < rules.min_piece:
            break
        Q_in = p.q if end == 0 else p.q[::-1]
        got = escape(arm, guard, paper, Q_in, rules, opt)
        if not isinstance(got, str):
            if cut > 0.0:
                got = Lift(got.q_draw, got.q_up, got.up, got.down, f"e cut {cut * 1e3:.0f} mm, "
                           + got.how)
            return got, float(cut)
        first = first or got
    return first
