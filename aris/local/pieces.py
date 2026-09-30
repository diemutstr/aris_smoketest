"""From a route to pieces and leftovers, and from a piece's routes to its alternatives.

Every layer step of a line is either drawn by exactly one piece or left over with a reason, so
pieces and leftovers cover the line exactly once, by construction: their ends are the same
layer positions.
"""
from __future__ import annotations

import numpy as np

from aris.local.gates import Judge
from aris.local.lattice import Lattice
from aris.local.search import Sweep
from aris.types import Leftover, Piece

NO_NODE = ("unreachable", "no arm configuration inside the gates")
NO_STEP = ("unreachable", "no continuous arm motion within the joint-step cap")
REPAIRED = ("unreachable", "the exact path fails here between search nodes")


def _step_reason(lat: Lattice, judge: Judge, k: int) -> tuple[str, str]:
    """Why the step from layer k to k+1 cannot be drawn."""
    A, B = lat.layers[k], lat.layers[k + 1]
    if len(A.q) == 0 or len(B.q) == 0:
        return NO_NODE
    if not A.free.any() or not B.free.any():
        return ("blocked", _blocker(lat, judge, k))
    pred, _ = lat.edges(k)
    if not (pred >= 0).any():
        return NO_STEP
    free_pred = np.where(pred >= 0, A.free[np.maximum(pred, 0)], False) & B.free[:, None]
    if not free_pred.any():
        return ("blocked", _blocker(lat, judge, k))
    return REPAIRED


def _blocker(lat: Lattice, judge: Judge, k: int) -> str:
    """'blocked by <what>': the obstacle closest to most of the step's blocked nodes.  A parked
    arm is named as a whole."""
    Q = np.concatenate([lat.layers[j].q[~lat.layers[j].free] for j in (k, k + 1)])
    name = judge.blocker(Q[:: max(1, len(Q) // 200)])
    if name.startswith("parked"):
        name = "parked arm " + name[len("parked"):].split(":")[0]
    return f"blocked by {name}" if name else "blocked by the obstacles"


def gap_leftovers(lat: Lattice, judge: Judge, line_id: str, gaps, gap_from: dict,
                  gap_to: dict) -> list[Leftover]:
    """One leftover per stretch of undrawn steps with the same reason.  A gap starting at layer
    k really starts at gap_from[k] if a piece was extended past k; it ends at gap_to[k]."""
    out = []
    for k0, k1 in gaps:
        first = len(out)
        reasons = [_step_reason(lat, judge, k) for k in range(k0, k1)]
        a = k0
        for k in range(k0, k1):
            if k + 1 == k1 or reasons[k + 1 - k0] != reasons[k - k0]:
                out.append(Leftover(Piece(line_id, float(lat.s[a]), float(lat.s[k + 1])),
                                    *reasons[k - k0]))
                a = k + 1
        x = out[first]
        out[first] = Leftover(Piece(line_id, gap_from.get(k0, x.piece.s0), x.piece.s1),
                              x.reason, x.detail)
        x = out[-1]
        out[-1] = Leftover(Piece(line_id, x.piece.s0, gap_to.get(k1, x.piece.s1)), x.reason,
                           x.detail)
    return [x for x in out if x.piece.s1 > x.piece.s0]


def families(lat: Lattice, sweep: Sweep, sectors: int) -> list[int]:
    """End nodes of the best route of each family, cheapest first.

    A family is the IK slot plus the quarter-turn of the spin at the start and at the end:
    two routes of different families start or end in a really different arm shape.
    """
    ends = np.flatnonzero(np.isfinite(sweep.cost))
    if not len(ends):
        return []
    A, B = lat.layers[sweep.k0], lat.layers[sweep.k1]
    n_spin = len(lat.spins)
    start = sweep.origin[ends]
    key = np.stack([B.slot[ends], A.spin[start] * sectors // n_spin,
                    B.spin[ends] * sectors // n_spin], axis=1)
    order = np.lexsort((ends, sweep.cost[ends]))           # cheapest first, ties by node number
    seen, best = set(), []
    for i in order:
        kk = tuple(int(x) for x in key[i])
        if kk not in seen:
            seen.add(kk)
            best.append(int(ends[i]))
    return best
