"""Drag-only pens: which way a piece may be drawn.

A pen that leans in its holder (the lateral holder: about 23 degrees) skids when a stroke pushes
its tip forward, toward the side the tip leans to, and draws when it is pulled.  With
`rules.drag_only` a piece is drawn in the direction in which the pen is pulled: at every step
the drawing direction's component along the pen's lean (the pen axis laid into the paper plane,
pointing from the hand toward the tip) is at most zero, i.e. the tip trails.

A line whose pull changes side (a corner of a V, or a curve whose tangent crosses the lean's
perpendicular) is split there (`split_bunch`), each part drawn in its own pulled direction with
its own set-down and lift-off.  A part shorter than `rules.min_piece` joins its neighbour and is
drawn in that neighbour's direction, pushed.
"""
from __future__ import annotations

import numpy as np

from aris.types import Bunch, DrawPlan, Piece

TOL = 1e-9            # m of tip travel along the lean per step still counted as "not pushed"


def _along(arm, plan: DrawPlan, normal: np.ndarray):
    """-> (each step's travel along the lean at mid-step / the lean's length, step lengths)."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    step = np.diff(plan.tip_base, axis=0)
    lean = arm.pen_axis(plan.q)
    lean = lean - (lean @ n)[:, None] * n
    lean = 0.5 * (lean[:-1] + lean[1:])
    along = np.einsum("ij,ij->i", step, lean) / np.maximum(np.linalg.norm(lean, axis=1), 1e-15)
    return along, np.linalg.norm(step, axis=1)


def runs(arm, plan: DrawPlan, normal: np.ndarray, min_piece: float) -> list[tuple]:
    """The plan cut where the pull changes side: [(first sample, last sample, direction 0 as
    the plan runs / 1 backwards, metres pushed when drawn that way)], in order along the plan.
    A step across the lean goes with the run before it (the first ones with the run after).
    A run shorter than `min_piece` joins its longer neighbour, in that neighbour's direction."""
    along, ln = _along(arm, plan, normal)
    if len(ln) == 0:
        return [(0, len(plan.q) - 1, 0, 0.0)]
    lab = np.where(along <= -TOL, 0, np.where(along >= TOL, 1, -1))
    known = np.flatnonzero(lab >= 0)
    if len(known) == 0:
        return [(0, len(plan.q) - 1, 0, 0.0)]
    lab[:known[0]] = lab[known[0]]
    for i in range(known[0] + 1, len(lab)):
        if lab[i] < 0:
            lab[i] = lab[i - 1]
    # runs of steps [a, b) with one direction
    cuts = [0] + [i for i in range(1, len(lab)) if lab[i] != lab[i - 1]] + [len(lab)]
    rr = [[a, b, int(lab[a])] for a, b in zip(cuts[:-1], cuts[1:])]

    def length(r):
        return float(ln[r[0]:r[1]].sum())

    while len(rr) > 1:
        k = min(range(len(rr)), key=lambda i: (length(rr[i]), i))
        if length(rr[k]) >= min_piece:
            break
        nb = [i for i in (k - 1, k + 1) if 0 <= i < len(rr)]
        j = max(nb, key=lambda i: (length(rr[i]), -i))
        rr[k][2] = rr[j][2]
        merged = [rr[0]]                      # neighbours with one direction become one run
        for r in rr[1:]:
            if r[2] == merged[-1][2]:
                merged[-1] = [merged[-1][0], r[1], r[2]]
            else:
                merged.append(r)
        rr = merged
    out = []
    for a, b, d in rr:
        sl = slice(a, b)
        pushed = ln[sl][(along[sl] > TOL) if d == 0 else (along[sl] < -TOL)].sum()
        out.append((a, b, d, float(pushed)))
    return out


def sub_plan(plan: DrawPlan, i0: int, i1: int, line_id: str) -> DrawPlan:
    """Samples i0..i1 (both kept) of the plan, as a plan of its own."""
    sl = slice(i0, i1 + 1)
    s = plan.s[sl]
    return DrawPlan(piece=Piece(line_id, float(min(s[0], s[-1])), float(max(s[0], s[-1]))),
                    q=plan.q[sl], s=s, tip_base=plan.tip_base[sl], score=plan.score,
                    joint_travel=float(np.abs(np.diff(plan.q[sl], axis=0)).sum()),
                    draw_time=0.0,
                    spin=None if plan.spin is None else plan.spin[sl],
                    lean=None if plan.lean is None else plan.lean[sl])


SNAP = 0.0015         # m, how far an alternative's sample may lie from a cut and still be cut there


def split_bunch(arm, bunch: Bunch, normal: np.ndarray, min_piece: float) -> list[Bunch]:
    """The piece cut where the pull changes side, each part a bunch of its own.  The cuts are
    those of the alternative pushed least once cut (then the fewest parts, then the first);
    every alternative is cut at the same places (at its sample nearest each cut; one with no
    sample within SNAP of a cut is dropped).  A piece pulled one way throughout is returned
    as it is."""
    cut = [runs(arm, p, normal, min_piece) for p in bunch.plans]
    ref = min(range(len(cut)), key=lambda i: (sum(r[3] for r in cut[i]), len(cut[i]), i))
    if len(cut[ref]) <= 1:
        return [bunch]
    rp = bunch.plans[ref]
    at = sorted(float(rp.s[r[1]]) for r in cut[ref][:-1])     # arc length of each cut
    bounds = [bunch.piece.s0] + at + [bunch.piece.s1]
    lid = bunch.piece.line_id
    parts: list[list[DrawPlan]] = [[] for _ in range(len(at) + 1)]
    for p in bunch.plans:
        s = np.asarray(p.s, float)
        idx = [int(np.argmin(np.abs(s - a))) for a in at]
        if any(abs(s[i] - a) > SNAP for i, a in zip(idx, at)):
            continue
        ends = [0] + idx + [len(s) - 1]
        if s[-1] < s[0]:                       # a plan running against the line's arc length
            ends = [0] + idx[::-1] + [len(s) - 1]
        if any(b <= a for a, b in zip(ends[:-1], ends[1:])):
            continue
        sub = [sub_plan(p, a, b, lid) for a, b in zip(ends[:-1], ends[1:])]
        sub.sort(key=lambda x: x.piece.s0)
        for k, x in enumerate(sub):
            parts[k].append(x)
    return [Bunch(Piece(lid, lo, hi), tuple(ps))
            for lo, hi, ps in zip(bounds[:-1], bounds[1:], parts)]


def pulled_shares(arm, plan: DrawPlan, normal: np.ndarray) -> tuple[float, float]:
    """(share of the piece's length on which the pen is pulled when drawn as the plan runs,
    the same drawn backwards).  A step across the lean counts for both."""
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    step = np.diff(plan.tip_base, axis=0)                               # (N-1, 3) m
    lean = arm.pen_axis(plan.q)
    lean = lean - (lean @ n)[:, None] * n
    lean = 0.5 * (lean[:-1] + lean[1:])                                 # at mid-step
    along = np.einsum("ij,ij->i", step, lean)
    ln = np.linalg.norm(step, axis=1)
    total = max(float(ln.sum()), 1e-15)
    fwd = float(ln[along <= TOL * np.linalg.norm(lean, axis=1)].sum()) / total
    back = float(ln[-along <= TOL * np.linalg.norm(lean, axis=1)].sum()) / total
    return fwd, back
