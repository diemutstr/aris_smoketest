"""The free-space planner's view of "free": obstacles, the arm against itself, joint limits.

One `Checker` per question.  Edges (straight joint-space moves) go to the collision kernel in
batches, `collide.edges_clearance_q`, which bounds the clearance all along each edge in compiled
code and stops as soon as the edge is proven free or a point on it is found not free.

The search may demand more than "free": `need_obst` and `need_self` metres beyond every margin,
so that the path still reads free once timing has rounded its corners.  That is asked of the
kernel by adding them to the margins, so "free" keeps meaning "at least 0".

The cap on the search is a count of edges checked (`n_edges`), not a time.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.kernel import collide
from aris.types import Gates, Obstacles


def _shift(P: collide.Packed, d: float) -> collide.Packed:
    """The same obstacles, each demanding d metres more clearance."""
    if d == 0.0:
        return P
    return replace(P, box_m=P.box_m + d, pl_m=P.pl_m + d, pl_pen_m=P.pl_pen_m + d,
                   pl_tool_m=P.tool_m + d, cap_rm=P.cap_rm + d)


class Checker:
    """Free = inside the joint limits by the gate's margin, clear of every obstacle by its own
    margin, clear of itself by the gate's self margin."""

    def __init__(self, arm, obstacles: Obstacles, gates: Gates, backend: str | None = None):
        self.tables = collide.arm_tables(arm)
        self.packed = collide.pack(obstacles)
        self.reach = np.asarray(arm.reach, float)                      # (7, K)
        self.pairs = np.asarray(arm.self_pairs, np.int64)
        self.self_margin = float(gates.self_margin)
        self.q_lo = arm.limits.q_min + gates.limit_margin
        self.q_hi = arm.limits.q_max - gates.limit_margin
        self.backend = backend
        self.n_edges = 0
        self.proven = set()          # edges shown free with the need to spare, both directions
        self.set_need(0.0, 0.0)

    def set_need(self, obst: float, own: float, tol: float = 5e-4) -> None:
        """Demand `obst` and `own` metres to spare.  `tol` is the kernel's refinement tolerance:
        it must be below the room the ends leave, or no edge out of a tight end is ever proven
        free."""
        self.need_obst, self.need_self, self.tol = obst, own, tol
        self._search_scene = _shift(self.packed, obst)

    def inside(self, Q) -> np.ndarray:
        Q = np.asarray(Q, float).reshape(-1, 7)
        return np.all((Q >= self.q_lo) & (Q <= self.q_hi), axis=1)

    def edges(self, QA, QB) -> np.ndarray:
        """(E,7), (E,7) -> (E,) bool: is the straight move from QA[e] to QB[e] free, with
        `need_obst` and `need_self` to spare all along it?"""
        QA = np.asarray(QA, float).reshape(-1, 7)
        QB = np.asarray(QB, float).reshape(-1, 7)
        ok = self.inside(QA) & self.inside(QB)          # the limit box is convex: ends suffice
        if ok.any():
            self.n_edges += int(ok.sum())
            v = collide.edges_clearance_q(
                self.tables, self.reach, QA[ok], QB[ok], self._search_scene,
                self_pairs=self.pairs, self_margin=self.self_margin + self.need_self,
                tol=self.tol, floor=0.0, backend=self.backend)
            ok[ok] = v >= 0.0
            for a, b in zip(QA[ok], QB[ok]):
                self.proven.add(a.tobytes() + b.tobytes())
                self.proven.add(b.tobytes() + a.tobytes())
        return ok

    def was_proven(self, qa, qb) -> bool:
        """Was the edge qa-qb shown free (with `need_obst`, `need_self` to spare) by `edges`?"""
        return qa.tobytes() + qb.tobytes() in self.proven

    def edge_margins(self, QA, QB, charge=None, sag=None) -> np.ndarray:
        """(E,): lower bound on the real clearance (obstacles and the arm against itself,
        nothing to spare) along each straight edge, minus a charge per edge: `charge` (E,) in
        metres, or `sag` (E, 7), how far a flown curve can stray from the edge per joint, which
        is turned into what it can do to a capsule or a pair of them."""
        if sag is not None:
            move = sag @ self.reach                                     # (E, K)
            i, j = self.pairs[:, 0], self.pairs[:, 1]
            charge = np.maximum(move.max(axis=1), (move[:, i] + move[:, j]).max(axis=1))
        charge = np.zeros(len(QA)) if charge is None else np.asarray(charge, float)
        v = collide.edges_clearance_q(self.tables, self.reach, QA, QB, self.packed,
                                      self_pairs=self.pairs, self_margin=self.self_margin,
                                      tol=min(self.tol, 1e-4), floor=float(charge.max()),
                                      backend=self.backend)
        return v - charge

    def ends(self, Q):
        """(obstacle detail, self clearance) at a few configurations, for the refusal reasons."""
        det = collide.clearance_detail_q(self.tables, Q, self.packed, backend=self.backend)
        own = collide.self_clearance_q(self.tables, Q, self.pairs, self.self_margin,
                                       backend=self.backend)
        return det, own
