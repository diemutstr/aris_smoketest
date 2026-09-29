"""Shorten a free joint path.  Every replacement piece is checked before it is accepted, so the
result is free wherever the input was, and never longer.

Two moves:
- drop: remove every waypoint whose two neighbours see each other (one batch per round);
- cut corners: pick pairs of points part-way along the path, try the straight move between them
  (a batch of candidates at once) and splice in the free one that saves the most.
"""
from __future__ import annotations

import numpy as np


def length(q: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))


def drop(checker, q: np.ndarray) -> np.ndarray:
    """Remove waypoints whose two neighbours see each other.  Each round checks every
    waypoint's bypass in one batch and removes a set of non-neighbouring ones (so every new
    piece is one that was checked); rounds repeat until nothing more can go."""
    failed = set()                     # bypasses already found blocked: not asked again
    key = lambda a, b: a.tobytes() + b.tobytes()
    while len(q) > 2:
        k = np.arange(1, len(q) - 1)
        ask = np.array([key(q[i - 1], q[i + 1]) not in failed for i in k])
        ok = np.zeros(len(k), bool)
        if ask.any():
            ok[ask] = checker.edges(q[k[ask] - 1], q[k[ask] + 1])
        failed.update(key(q[i - 1], q[i + 1]) for i in k[ask & ~ok])
        remove, last = [], -2
        for kk, good in zip(k, ok):
            if good and kk - last > 1:
                remove.append(kk)
                last = kk
        if not remove:
            break
        q = np.delete(q, remove, axis=0)
    return q


def _point_at(q, cum, s):
    k = int(np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(q) - 2))
    f = (s - cum[k]) / max(cum[k + 1] - cum[k], 1e-15)
    return k, q[k] + f * (q[k + 1] - q[k])


def cut_corners(checker, q: np.ndarray, rng: np.random.Generator, rounds: int,
                tries: int = 16) -> np.ndarray:
    for _ in range(rounds):
        if len(q) < 3:
            break
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
        s = np.sort(rng.uniform(0.0, cum[-1], (tries, 2)), axis=1)
        cand = [(_point_at(q, cum, a), _point_at(q, cum, b)) for a, b in s]
        cand = [(ka, pa, kb, pb) for (ka, pa), (kb, pb) in cand if kb > ka]   # spans a corner
        if not cand:
            continue
        pa = np.array([c[1] for c in cand])
        pb = np.array([c[3] for c in cand])
        ok = checker.edges(pa, pb)
        best, gain = None, 1e-9
        for c, good in zip(cand, ok):
            if not good:
                continue
            ka, a, kb, b = c
            old = length(np.concatenate([a[None], q[ka + 1:kb + 1], b[None]]))
            if old - np.linalg.norm(b - a) > gain:
                best, gain = c, old - np.linalg.norm(b - a)
        if best is not None:
            ka, a, kb, b = best
            q = dedupe(np.concatenate([q[:ka + 1], a[None], b[None], q[kb + 1:]]))
    return q


def dedupe(q: np.ndarray, tol: float = 1e-9) -> np.ndarray:
    """Drop interior waypoints that sit on a neighbour; the two ends are kept exactly."""
    keep = [0]
    for i in range(1, len(q) - 1):
        if (np.linalg.norm(q[i] - q[keep[-1]]) > tol and np.linalg.norm(q[-1] - q[i]) > tol):
            keep.append(i)
    return q[keep + [len(q) - 1]]


def shorten(checker, q: np.ndarray, rng: np.random.Generator, rounds: int = 3,
            head: int = 0, tail: int = 0) -> np.ndarray:
    """Shorten q.  The first `head` and last `tail` pieces are lifts off the paper: close to
    the paper every check is expensive, so a lift is kept whole or skipped whole, never
    trimmed sample by sample."""
    n = len(q)
    mid = q[head:n - tail]
    mid = drop(checker, cut_corners(checker, drop(checker, mid), rng, rounds))
    out = np.concatenate([q[:head], mid, q[n - tail:]])
    if (head or tail) and len(mid) > 2 - (not head) - (not tail):
        # skip a lift whole: from the low end straight to the first waypoint past its top
        a, b = [], []
        if head:
            a.append(q[0]), b.append(mid[1])
        if tail:
            a.append(mid[-2]), b.append(q[-1])
        ok = list(checker.edges(np.array(a), np.array(b)))
        pre = np.concatenate([q[:1], mid[1:]]) if head and ok.pop(0) else out[:head + len(mid)]
        out = (np.concatenate([pre[:-1], q[-1:]]) if tail and ok.pop(0)
               else np.concatenate([pre, q[n - tail:]]))
    return dedupe(out)
