"""Allocation: which arm, in which phase, draws each stretch still to draw.

The candidates are every (phase, arm) still to come, phases in order, arms in the phase's
order; each is a leader phase (1, 2) or a fill phase.  A map "holds" a point when the point
is inside it (`Settings.sample_step` apart, judged against the map shrunk by `erode` grid
steps).

The rule, leader phases first:
1. A stretch one leader-phase map holds whole goes to the first such candidate.
2. Otherwise a fill phase may take part of it only if it is worth it: the leader phases
   between them hold less than `leader_share` (80 %) of it, or some fill map holds a run at
   least `fill_factor` (1.5) times the longest run any leader map holds.  If so, a fill map
   that holds all of it takes it whole.
3. Otherwise it is cut.  From its start, take the leader candidate holding the longest run
   from here (a fill candidate instead only where it is allowed and its run is at least
   `fill_factor` times as long, or where no leader map holds the line at all); the next piece
   starts where that run ends, reaching back `rules.min_piece` over the join, so a join is
   drawn twice rather than not at all.  (A run starting one sample later counts as starting
   here: the sample spacing is well inside the erosion.)
Where no map holds the line, the part up to the next point some map holds is left over.
Stretches shorter than `rules.min_piece` are left over as "too_short".
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


def longest_run(inside: np.ndarray, u: np.ndarray) -> float:
    """The longest stretch (m) any one row of `inside` (C, n) holds without a break."""
    if not len(inside):
        return 0.0
    end = run_ends(inside)
    start = inside & ~np.concatenate([np.zeros((len(inside), 1), bool), inside[:, :-1]], axis=1)
    c, i = np.nonzero(start)
    return float(np.max(u[end[c, i]] - u[i], initial=0.0))


def _pick(reach, u, cur, leader, fill_ok, factor):
    """The candidate to take from sample `cur`, or None."""
    gain = np.where(reach > cur, u[np.maximum(reach, 0)] - u[cur], 0.0)
    gl = np.where(leader, gain, 0.0)
    gf = np.where(~leader, gain, 0.0)
    bl, bf = int(np.argmax(gl)), int(np.argmax(gf))       # the first candidate on a tie
    if gl[bl] > 0.0 and not (fill_ok and gf[bf] >= factor * gl[bl]):
        return bl
    return bf if gf[bf] > 0.0 else None


def cover(inside: np.ndarray, u: np.ndarray, overlap: float, leader=None, fill_ok=True,
          factor: float = 1.0) -> list[tuple]:
    """Cut arc lengths `u` (n,) into [(a, b, candidate or None)], None where no candidate holds
    the line.  `inside` (C, n): candidate c holds sample i; `leader` (C,) bool (default: all)."""
    n = len(u)
    leader = np.ones(len(inside), bool) if leader is None else np.asarray(leader, bool)
    end = run_ends(inside)
    out, cur, joined = [], 0, False
    while cur < n - 1:
        reach = np.maximum(end[:, cur], end[:, cur + 1])
        best = _pick(reach, u, cur, leader, fill_ok, factor)
        if best is not None:
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


def _held_share(inside: np.ndarray, u: np.ndarray) -> float:
    """Share of the length where both ends of a sample step are held by some row."""
    if not len(inside) or u[-1] <= u[0]:
        return 0.0
    held = inside.any(axis=0)
    return float(np.sum(np.diff(u)[held[:-1] & held[1:]]) / (u[-1] - u[0]))


def allocate(pool, candidates, rules: DrawRules, cfg: Settings):
    """-> (stretches, each with a target (phase index, arm id), leftovers, cuts made).

    `candidates`: [(phase index, arm id, Map, is a leader phase)] in order of preference.
    Stretches that already have a target keep it."""
    out, left, cuts = [], [], 0
    leader = np.array([c[3] for c in candidates], bool)
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
        inside = np.array([m.contains(p) for _, _, m, _ in candidates])
        whole = inside.all(axis=1)
        first = np.flatnonzero(whole & leader)
        if len(first):
            out.append(replace(st, target=candidates[first[0]][:2]))
            continue
        fill_ok = (_held_share(inside[leader], u) < cfg.leader_share
                   or longest_run(inside[~leader], u)
                   >= cfg.fill_factor * longest_run(inside[leader], u))
        first = np.flatnonzero(whole & ~leader)
        if fill_ok and len(first):
            out.append(replace(st, target=candidates[first[0]][:2]))
            continue
        parts = cover(inside, u, rules.min_piece, leader, fill_ok, cfg.fill_factor)
        cuts += sum(1 for x, y in zip(parts, parts[1:]) if x[2] is not None and y[2] is not None)
        for a, b, c in parts:
            part = st.sub(a, b)
            if c is None:
                left.append(_left(replace(part, reason=st.reason, detail=st.detail),
                                  "unreachable", "outside every drawable map"))
            elif part.length < rules.min_piece:
                left.append(Leftover(part.piece, "too_short",
                                     "a cut piece shorter than the shortest piece worth drawing"))
            else:
                out.append(replace(part, target=candidates[c][:2]))
    return out, left, cuts
