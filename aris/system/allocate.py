"""Allocation: which arm, in which phase, draws each stretch still to draw.

The rule.  The candidates are every (phase, arm) still to come, phases in order, arms in the
phase's order.  A stretch goes to the first candidate whose drawable map holds all of it (every
point, `Settings.sample_step` apart, judged against the map shrunk by `erode` grid steps).

A stretch no single map holds is cut.  From its start, take the candidate whose map holds the
longest run of it from here; the next stretch starts where that run ends, reaching back
`rules.min_piece` over the join, so a join is drawn twice rather than not at all.  (A run that
starts one sample later still counts as starting here: the sample spacing is well inside the
erosion.)  Where no map holds the line, the part up to the next point some map holds is left
over.  Stretches shorter than `rules.min_piece` are left over as "too_short".
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.system.settings import Settings
from aris.system.stretch import Stretch
from aris.types import DrawRules, Leftover


def run_ends(inside: np.ndarray) -> np.ndarray:
    """(C, n) -> (C, n): for each sample inside a run, the index of the run's last sample;
    -1 outside."""
    C, n = inside.shape
    end = np.full((C, n), -1)
    last = np.full(C, -1)
    for i in range(n - 1, -1, -1):
        last = np.where(inside[:, i], np.where(last >= 0, last, i), -1)
        end[:, i] = last
    return end


def cover(inside: np.ndarray, u: np.ndarray, overlap: float) -> list[tuple]:
    """Cut arc lengths `u` (n,) into [(a, b, candidate or None)], candidates' runs longest first,
    None where no candidate holds the line.  `inside` (C, n): candidate c holds sample i."""
    n = len(u)
    end = run_ends(inside)
    out, cur, joined = [], 0, False
    while cur < n - 1:
        reach = np.maximum(end[:, cur], end[:, cur + 1])
        best = int(np.argmax(reach))                 # the first candidate on a tie
        if reach[best] > cur:
            a = max(u[0], u[cur] - overlap) if joined else u[cur]
            out.append((a, u[reach[best]], best))
            cur, joined = int(reach[best]), True
            continue
        later = np.flatnonzero(inside[:, cur + 1:].any(axis=0))
        nxt = n - 1 if not len(later) else cur + 1 + int(later[0])
        if out and out[-1][2] is None:
            out[-1] = (out[-1][0], u[nxt], None)
        else:
            out.append((u[cur], u[nxt], None))
        cur, joined = nxt, False
    return out


def _left(st: Stretch, reason_if_new: str, detail_if_new: str) -> Leftover:
    if st.reason:
        return Leftover(st.piece, st.reason, (st.detail + "; " if st.detail else "")
                        + "no later phase can draw it")
    return Leftover(st.piece, reason_if_new, detail_if_new)


def allocate(pool, candidates, rules: DrawRules, cfg: Settings):
    """-> (stretches, each with a target (phase index, arm id), leftovers).

    `candidates`: [(phase index, arm id, Map)] in order of preference.  Stretches that already
    have a target keep it."""
    out, left = [], []
    for st in pool:
        if st.target is not None:
            out.append(st)
            continue
        if st.length < rules.min_piece:
            left.append(_left(st, "too_short", "shorter than the shortest piece worth drawing"))
            continue
        if not candidates:
            left.append(_left(st, "unreachable", "outside every drawable map"))
            continue
        u, p = st.samples(cfg.sample_step)
        inside = np.array([m.contains(p) for _, _, m in candidates])
        whole = np.flatnonzero(inside.all(axis=1))
        if len(whole):
            out.append(replace(st, target=candidates[whole[0]][:2]))
            continue
        for a, b, c in cover(inside, u, rules.min_piece):
            part = st.sub(a, b)
            part = replace(part, reason=st.reason, detail=st.detail) if c is None else part
            if c is None:
                left.append(_left(part, "unreachable", "outside every drawable map"))
            elif part.length < rules.min_piece:
                left.append(Leftover(part.piece, "too_short",
                                     "a cut piece shorter than the shortest piece worth drawing"))
            else:
                out.append(replace(part, target=candidates[c][:2]))
    return out, left
