"""The search: one sweep over the layers of the graph (dynamic programming).

Along the way, the pen is in one of two states at each layer: down on some node, or up
("gap": this stretch is left undrawn).  Going from layer k to layer k+1 the pen either draws
the step along an edge, or leaves the step undrawn at `gap_cost`.  Inside a layer it may lift
from one node and come down on any other at `lift_cost` (a lift edge: the line is cut into two
pieces there), or come down from the gap state on any node for free.  The objective is the
summed joint motion plus those costs; the gates are already in the graph (a node is usable or
not).  Every step is array operations over a whole layer.

`best_route` finds the cheapest way along the whole line.  `piece_routes` sweeps one stretch
with no lifts and no gaps and returns, for every node of its last layer, the cheapest route
there and where it started: the raw material of the alternatives.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.local.lattice import Lattice

EDGE, LIFT, FROM_GAP, START = 0, 1, 2, 3


@dataclass(frozen=True)
class Run:
    """Pen down from layer k0 to layer k1 (k1 > k0), on nodes[i] at layer k0 + i."""
    k0: int
    k1: int
    nodes: np.ndarray


@dataclass(frozen=True)
class Route:
    runs: tuple[Run, ...]            # in order along the line; consecutive runs meet at a lift
    gaps: tuple[tuple[int, int], ...]  # (k0, k1): layer steps left undrawn
    cost: float


def _relax(lat: Lattice, k: int, cost: np.ndarray):
    """Best cost at every node of layer k+1 reached along an edge from layer k.
    -> cost (n1,), predecessor (n1,), -1 where there is none."""
    pred, travel = lat.edges(k)
    if not len(cost):
        n1 = len(pred)
        return np.full(n1, np.inf), np.full(n1, -1)
    c = np.where(pred >= 0, cost[np.maximum(pred, 0)], np.inf) + travel
    j = np.argmin(c, axis=1)
    rows = np.arange(len(c))
    best = c[rows, j]
    back = np.where(np.isfinite(best), pred[rows, j], -1)
    best = np.where(lat.usable(k + 1), best, np.inf)
    return best, back


def best_route(lat: Lattice, lift_cost: float, gap_cost: float) -> Route:
    """The cheapest way along the whole line."""
    n = lat.n_layers
    cost = np.where(lat.usable(0), 0.0, np.inf)
    gap = 0.0
    how, back, lift_src, gap_src = [np.full(len(cost), START)], [None], [-1], [-1]
    for k in range(n - 1):
        node_min = float(cost.min(initial=np.inf))
        # the gap state at k+1: the step k -> k+1 undrawn, coming from the gap or a node at k
        g_src = -1 if gap <= node_min else int(np.argmin(cost))
        gap = min(gap, node_min) + gap_cost
        new, bk = _relax(lat, k, cost)
        h = np.full(len(new), EDGE)
        # inside layer k+1: lift from the best node, or come down from the gap
        src = int(np.argmin(new)) if len(new) else -1
        lifted = (float(new.min(initial=np.inf)) + lift_cost)
        usable = lat.usable(k + 1)
        for alt, code in ((lifted, LIFT), (gap, FROM_GAP)):
            better = usable & (alt < new)
            new = np.where(better, alt, new)
            h = np.where(better, code, h)
        cost = new
        how.append(h)
        back.append(bk)
        lift_src.append(src)
        gap_src.append(g_src)
    end_node = float(cost.min(initial=np.inf))
    total = min(end_node, gap)
    return _unwind(lat, how, back, lift_src, gap_src, cost, gap, end_node < gap, total)


def _unwind(lat, how, back, lift_src, gap_src, cost, gap, end_on_node, total) -> Route:
    runs, gaps = [], []
    k = lat.n_layers - 1
    node = int(np.argmin(cost)) if end_on_node else -1
    trail = []                       # nodes of the run being unwound, last layer first
    while True:
        if node >= 0:
            trail.append(node)
            h = how[k][node]
            if h == EDGE:
                node, k = int(back[k][node]), k - 1
                continue
            _close(runs, k, trail)
            trail = []
            if h == LIFT:
                node = lift_src[k]
            elif h == FROM_GAP:
                node = -1
            else:                    # START
                break
        else:
            if k == 0:
                break
            gaps.append((k - 1, k))
            node, k = gap_src[k], k - 1
    runs.reverse()
    return Route(tuple(runs), tuple(_merge(sorted(gaps))), float(total))


def _close(runs, k0, trail):
    if len(trail) > 1:              # a run of one layer draws nothing
        runs.append(Run(k0, k0 + len(trail) - 1, np.array(trail[::-1], int)))


def _merge(steps):
    out = []
    for a, b in steps:
        if out and out[-1][1] == a:
            out[-1] = (out[-1][0], b)
        else:
            out.append((a, b))
    return out


@dataclass(frozen=True)
class Sweep:
    """Cheapest pen-down routes over layers k0..k1, no lifts."""
    k0: int
    k1: int
    cost: np.ndarray                 # (n,) at every node of layer k1
    origin: np.ndarray               # (n,) the node of layer k0 each route starts on
    back: tuple[np.ndarray, ...]     # back[i]: predecessors of layer k0 + 1 + i

    def route(self, end: int) -> np.ndarray:
        """Node numbers from layer k0 to k1 of the route ending on node `end`."""
        out = [end]
        for b in reversed(self.back):
            out.append(int(b[out[-1]]))
        return np.array(out[::-1], int)


def piece_routes(lat: Lattice, k0: int, k1: int) -> Sweep:
    cost = np.where(lat.usable(k0), 0.0, np.inf)
    origin = np.arange(len(cost))
    backs = []
    for k in range(k0, k1):
        cost, bk = _relax(lat, k, cost)
        origin = np.where(bk >= 0, origin[np.maximum(bk, 0)], -1)
        backs.append(bk)
    return Sweep(k0, k1, cost, origin, tuple(backs))
