"""Allocation: which arm, in which phase, draws each stretch still to draw.

The candidates are the (phase, arm) pairs still to come, in the fixed phase order.  A map holds
a stretch when it holds every point of it (`Settings.sample_step` apart).  The laws:

1. A stretch goes to the first candidate whose map holds all of it.
2. A stretch no map holds whole is cut into the longest stretches some map holds, from its
   start (the earlier phase on a tie), with one join of `rules.min_piece` between neighbouring
   stretches.
3. A stretch goes to a fill phase only when the leader phases hold less than
   `Settings.leader_share` (80 %) of it.
4. Parts of one line given to the same arm in the same phase are one stretch.
5. What no map holds is left over as "unreachable"; a stretch shorter than `rules.min_piece`
   as "too_short".

The arm planner is the judge: what it hands back returns to the pool for the phases after.
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


def cover(inside: np.ndarray, u: np.ndarray, join: float) -> list[tuple]:
    """Law 2 on sampled arc lengths `u` (n,): [(a, b, candidate or None)], each run the longest
    from where the last one ended (the first candidate on a tie), reaching back `join` over
    the join; None where no candidate holds the line.  `inside` (C, n)."""
    n = len(u)
    end = run_ends(inside)
    out, cur, joined = [], 0, False
    while cur < n - 1:
        best = int(np.argmax(end[:, cur]))
        if end[best, cur] > cur:
            a = max(u[0], u[cur] - join) if joined else u[cur]
            out.append((a, u[end[best, cur]], best))
            cur, joined = int(end[best, cur]), True
            continue
        later = np.flatnonzero(inside[:, cur + 1:].any(axis=0))
        nxt = n - 1 if not len(later) else cur + 1 + int(later[0])
        # a stretch between two samples counts as held only if one map holds both ends
        nxt = max(nxt, cur + 1)
        if out and out[-1][2] is None:
            out[-1] = (out[-1][0], u[nxt], None)
        else:
            out.append((u[cur], u[nxt], None))
        cur, joined = nxt, False
    return out


def held_share(inside: np.ndarray, u: np.ndarray) -> float:
    """Share of the length (sample steps with both ends held by some row)."""
    if not len(inside) or u[-1] <= u[0]:
        return 0.0
    held = inside.any(axis=0)
    return float(np.sum(np.diff(u)[held[:-1] & held[1:]]) / (u[-1] - u[0]))


def _place(st: Stretch, a, b, candidates, leader, rules, cfg, fill_ok=None):
    """Laws 1 to 3 and 5 on the part a..b of `st`.  -> ([(a, b, candidate)], [(a, b)] held by
    no map)."""
    u, p = st.sub(a, b).samples(cfg.sample_step)
    inside = np.array([m.contains(p) for _, _, m, _ in candidates])
    if fill_ok is None:
        fill_ok = held_share(inside[leader], u) < cfg.leader_share
    allowed = np.ones(len(leader), bool) if fill_ok else leader
    whole = np.flatnonzero(inside.all(axis=1) & allowed)
    if len(whole):
        return [(a, b, int(whole[0]))], []
    parts, gaps = [], []
    for x, y, c in cover(inside & allowed[:, None], u, rules.min_piece):
        if c is not None:
            parts.append((x, y, c))
        elif fill_ok:
            gaps.append((x, y))
        else:                  # held by no leader: a stretch of its own, joined at both ends
            got, left = _place(st, max(a, x - rules.min_piece), min(b, y + rules.min_piece),
                               candidates, leader, rules, cfg, fill_ok=True)
            parts += got
            gaps += [(max(g0, x), min(g1, y)) for g0, g1 in left if min(g1, y) > max(g0, x)]
    return parts, gaps


def _merge(parts: list) -> list:
    """Law 4: neighbouring parts for the same candidate become one."""
    out = []
    for x, y, c in sorted(parts):
        if out and out[-1][2] == c and x <= out[-1][1] + 1e-9:
            out[-1] = (out[-1][0], max(out[-1][1], y), c)
        else:
            out.append((x, y, c))
    return out


def _left(st: Stretch, reason_if_new: str, detail_if_new: str) -> Leftover:
    if st.reason:
        return Leftover(st.piece, st.reason, (st.detail + "; " if st.detail else "")
                        + "no later phase can draw it")
    return Leftover(st.piece, reason_if_new, detail_if_new)


def allocate(pool, candidates, rules: DrawRules, cfg: Settings):
    """-> (stretches, each with a target (phase index, arm id), leftovers, joins made).

    `candidates`: [(phase index, arm id, Map, is a leader phase)] in phase order.  Stretches
    that already have a target keep it."""
    out, left, joins = [], [], 0
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
        parts, gaps = _place(st, st.s0, st.s1, candidates, leader, rules, cfg)
        parts = _merge(parts)
        joins += sum(1 for p, q in zip(parts, parts[1:]) if q[0] < p[1] + 1e-9)
        out += [replace(st.sub(x, y), target=candidates[c][:2]) for x, y, c in parts]
        left += [_left(replace(st.sub(x, y), reason=st.reason, detail=st.detail),
                       "unreachable", "outside every drawable map") for x, y in gaps]
    return out, left, joins
