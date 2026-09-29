"""The free-space planner's view of "free": obstacles, the arm against itself, joint limits.

One `Checker` per question.  It answers in batches, because one call into the collision kernel
costs far more than one configuration inside a call.

Motion between two configurations is a straight line in joint space.  `bound.py` says how far
the capsules can move along it, which turns the clearances measured at both ends into a lower
bound everywhere between.  Where that bound does not prove an edge free, the edge is halved and
the middle is measured, until it is proved free, a measured point is not free, or the halving
cap is hit (then the edge counts as blocked: never optimistic).

The arm against itself is bounded with the zero-order bound and one refinement: the distance
between two of the arm's capsules depends only on the joints between them (turning an earlier
joint turns both rigidly), so only those joints are charged.

The search may demand more than "free": `need_obst` and `need_self` metres beyond every margin,
so that the path still reads free once timing has rounded its corners.
"""
from __future__ import annotations

import numpy as np

from aris.free.bound import accel_bound, interval_bound, n_moving, self_pair_weights, speeds
from aris.kernel import collide
from aris.types import Gates, Obstacles


class Checker:
    """Free = inside the joint limits by the gate's margin, clear of every obstacle by its own
    margin, clear of itself by the gate's self margin.  Counts every configuration it measures."""

    def __init__(self, arm, obstacles: Obstacles, gates: Gates, max_depth: int = 14,
                 backend: str | None = None):
        self.arm = arm
        self.tables = collide.arm_tables(arm)
        self.packed = collide.pack(obstacles)
        self.reach = np.asarray(arm.reach, float)                      # (7, K)
        self.pairs = np.asarray(arm.self_pairs, np.int64)
        self.self_w = self_pair_weights(self.reach, self.pairs)        # (P, 7)
        self.n_moving = n_moving(self.reach)                           # (K,)
        self.self_margin = float(gates.self_margin)
        self.q_lo = arm.limits.q_min + gates.limit_margin
        self.q_hi = arm.limits.q_max - gates.limit_margin
        self.max_depth = max_depth
        self.backend = backend
        self.n_checked = 0
        self.need_obst = 0.0
        self.need_self = 0.0

    # ------------------------------------------------------------ configurations

    def measure(self, Q, U=None):
        """(N,7) -> per-capsule obstacle clearance (N,K), self clearance (N,), and, given joint
        rates U (N,7), each capsule's largest point speed (N,K)."""
        Q = np.asarray(Q, float).reshape(-1, 7)
        self.n_checked += len(Q)
        body = collide.body_q(self.tables, Q, self.backend)
        cap = collide.capsule_clearance(body, self.packed, backend=self.backend)
        own = collide.self_clearance(body, self.pairs, self.self_margin, backend=self.backend)
        if U is None:
            return cap, own
        v = speeds(self.arm.link_frames(Q), body.p0, body.p1, self.n_moving, U)
        return cap, own, v

    def inside(self, Q) -> np.ndarray:
        Q = np.asarray(Q, float).reshape(-1, 7)
        return np.all((Q >= self.q_lo) & (Q <= self.q_hi), axis=1)

    # ------------------------------------------------------------ straight joint-space edges

    def edges(self, QA, QB) -> np.ndarray:
        """(E,7), (E,7) -> (E,) bool: is the straight move from QA[e] to QB[e] free, with
        `need_obst` and `need_self` to spare all along it?"""
        QA = np.asarray(QA, float).reshape(-1, 7)
        QB = np.asarray(QB, float).reshape(-1, 7)
        ok = self.inside(QA) & self.inside(QB)          # the limit box is convex: ends suffice
        if not ok.any():
            return ok
        e = np.flatnonzero(ok)
        n = len(e)
        U = QB[e] - QA[e]                                # every piece of an edge points this way
        cap, own, v = self.measure(np.concatenate([QA[e], QB[e]]), np.concatenate([U, U]))
        no, ns = self.need_obst, self.need_self
        ends_ok = ((cap[:n].min(1) >= no) & (cap[n:].min(1) >= no)
                   & (own[:n] >= ns) & (own[n:] >= ns))
        ok[e[~ends_ok]] = False
        U_abs = np.abs(U)
        Z = U_abs @ self.reach                           # (n,K) zero-order travel of the edge
        A = accel_bound(U_abs, self.reach)               # (n,K)
        Zs = (U_abs @ self.self_w.T).max(axis=1)         # (n,) relative travel, self pairs
        # an interval: its edge (0..n-1), both ends' configurations, clearances and speeds
        iv = [x[ends_ok] for x in (np.arange(n), QA[e], QB[e], cap[:n], cap[n:], own[:n],
                                   own[n:], v[:n], v[n:])]
        for depth in range(self.max_depth + 1):
            iv = [x[ok[e[iv[0]]]] for x in iv]
            g, qa, qb, ca, cb, sa, sb, va, vb = iv
            if not len(g):
                break
            h = 0.5 ** depth                             # the pieces' share of their edge
            da = np.minimum(Z[g] * h, va * h + A[g] * (0.5 * h * h))
            db = np.minimum(Z[g] * h, vb * h + A[g] * (0.5 * h * h))
            lb = interval_bound(ca, cb, da, db).min(axis=1)
            lbs = interval_bound(sa, sb, Zs[g] * h, Zs[g] * h)
            open_ = (lb < no) | (lbs < ns)
            if not open_.any():
                break
            if depth == self.max_depth:            # not proved free: blocked
                ok[e[g[open_]]] = False
                break
            g, qa, qb, ca, cb, sa, sb, va, vb = (x[open_] for x in iv)
            qm = 0.5 * (qa + qb)
            cm, sm, vm = self.measure(qm, U[g])
            ok[e[g[(cm.min(axis=1) < no) | (sm < ns)]]] = False
            iv = [np.concatenate(x) for x in ((g, g), (qa, qm), (qm, qb), (ca, cm), (cm, cb),
                                              (sa, sm), (sm, sb), (va, vm), (vm, vb))]
        return ok

    # ------------------------------------------------------------ margins of a whole path

    def path_margins(self, q, sag=None, tol: float = 5e-4) -> tuple[float, float]:
        """Lower bounds (obstacles, self) along the piecewise-straight path q (N,7).

        Obstacles: `collide.path_clearance_q`, the kernel's own verdict.  Self: the same method
        here (halve where the bound is more than `tol` below the smallest sample).  `sag`
        (N-1, 7), if given, is how far the flown curve can stray from each straight piece per
        joint; the worst it can do to any capsule is charged on top (see flown.py).
        """
        q = np.asarray(q, float).reshape(-1, 7)
        self.n_checked += len(q)
        obst = collide.path_clearance_q(self.tables, self.reach, q, self.packed, tol=tol,
                                        backend=self.backend)
        own = self._self_path(q, tol)
        if sag is not None:
            obst -= float(np.max(sag @ self.reach, initial=0.0))
            own -= float(np.max(sag @ self.self_w.T, initial=0.0))
        return obst, own

    def _self_path(self, q, tol):
        clear = lambda Q: collide.self_clearance_q(self.tables, Q, self.pairs, self.self_margin,
                                                   backend=self.backend)
        s = clear(q)
        m = float(s.min())
        if len(q) == 1:
            return m
        qa, qb, sa, sb = q[:-1], q[1:], s[:-1], s[1:]
        done = np.inf
        for depth in range(self.max_depth + 1):
            d = (np.abs(qb - qa) @ self.self_w.T).max(axis=1)
            lb = interval_bound(sa, sb, d, d)
            split = lb < m - tol
            if depth == self.max_depth:
                split[:] = False
            done = min(done, float(np.min(lb[~split], initial=np.inf)))
            if not split.any():
                break
            qa, qb, sa, sb = qa[split], qb[split], sa[split], sb[split]
            qm = 0.5 * (qa + qb)
            self.n_checked += len(qm)
            sm = clear(qm)
            m = min(m, float(sm.min()))
            qa, qb = np.concatenate([qa, qm]), np.concatenate([qm, qb])
            sa, sb = np.concatenate([sa, sm]), np.concatenate([sm, sb])
        return min(done, m)
