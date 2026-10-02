"""The graph: one layer per step along the line, one node per usable arm configuration.

A node is one choice of spin (hand turned about the paper normal, counted from the direction
pointing away from the base axis), lean (pen tilted off its nominal direction, a 2-vector),
elbow value (joint 7) and IK slot (the solver's answer number, which is the branch).  With the pen tip fixed on the line, that choice is one joint
configuration: `arm.hand_pose`, then `arm.ik`.

The graph answers two questions and nothing else:
  usable(k)   which nodes of layer k pass every gate (and have not been banned)
  edges(k)    which nodes of layer k each node of layer k+1 may come from, and at what cost
A search (search.py) uses those answers; it never looks inside.

A node is stored when it passes the gates that do not depend on obstacles other than the
paper ("reachable"); `free` says whether it also clears every other obstacle.  Keeping both
lets a leftover say whether a stretch is unreachable or blocked.

Layers start narrow (zero lean only) and are widened to every lean on request (`widen`).
Nodes are solved with the IK, or looked up in the kinematic table (table.py) when given one.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.local.gates import Judge
from aris.local.settings import Settings

N_SLOT = 8
_CHUNK = 20_000          # poses per IK batch; bounds memory


@dataclass
class Layer:
    """The stored nodes of one layer.  All arrays have one entry per node."""
    spin: np.ndarray         # (n,) spin index
    q7: np.ndarray           # (n,) elbow-value index
    lean: np.ndarray         # (n,) lean index
    slot: np.ndarray         # (n,) IK slot
    q: np.ndarray            # (n, 7)
    free: np.ndarray         # (n,) bool: clears every obstacle
    banned: np.ndarray       # (n,) bool: taken out after an exact path failed near it
    index: np.ndarray        # (n_lean, n_spin, n_q7, 8) node number, -1 where there is none
    answers: int = 0         # IK answers found in this layer (inside the joint limits)


def lean_set(lean_max: float, rings: int, dirs: int) -> np.ndarray:
    """(L, 2) leans: zero first, then `rings` magnitudes up to lean_max, `dirs` directions each."""
    out = [np.zeros(2)]
    if lean_max > 0.0 and rings > 0:
        for r in range(1, rings + 1):
            a = lean_max * r / rings
            for d in range(dirs):
                t = 2.0 * np.pi * d / dirs
                out.append(a * np.array([np.cos(t), np.sin(t)]))
    return np.array(out)


def lean_neighbours(leans: np.ndarray, rings: int) -> np.ndarray:
    """(L, M) for each lean the leans it may change to in one step (itself first), -1 padded.
    Neighbours differ by at most one ring step."""
    if len(leans) == 1:
        return np.zeros((1, 1), int)
    step = np.linalg.norm(leans[-1]) / rings
    d = np.linalg.norm(leans[:, None] - leans[None], axis=-1)
    nb = [[i] + [j for j in range(len(leans)) if j != i and d[i, j] <= step * (1 + 1e-6)]
          for i in range(len(leans))]
    width = max(len(n) for n in nb)
    return np.array([n + [-1] * (width - len(n)) for n in nb])


class Lattice:
    """The layered graph of one line for one arm."""

    def __init__(self, arm, judge: Judge, tips: np.ndarray, s: np.ndarray, lean_max: float,
                 settings: Settings, table=None):
        self.arm, self.judge, self.cfg, self.table = arm, judge, settings, table
        self.tips, self.s = tips, s
        # Spins are counted from the direction pointing away from the base axis at each layer
        # (the arm is symmetric about that axis, so this is the kinematic table's grid too);
        # the absolute spin of spin i at layer k is spins[i] - theta[k].
        self.spins = np.arange(settings.n_spin) * (2.0 * np.pi / settings.n_spin)
        self.theta = np.arctan2(tips[:, 1], tips[:, 0])
        q7_top = arm.limits.q_max[6] - judge.gates.limit_margin
        q7_low = arm.limits.q_min[6] + judge.gates.limit_margin
        k = np.arange(np.ceil(q7_low / settings.q7_step - 1e-9),
                      np.floor(q7_top / settings.q7_step + 1e-9) + 1)
        self.q7s = k * settings.q7_step
        self.leans = lean_set(lean_max, settings.lean_rings, settings.lean_dirs)
        self.lean_nb = lean_neighbours(self.leans, settings.lean_rings)
        self.opened = np.zeros((len(s), len(self.leans)), bool)
        self.layers: list[Layer] = [self._empty() for _ in s]
        self._edges: dict[int, tuple[np.ndarray, np.ndarray]] = {}
        self.ik_poses = 0
        self.lookups = 0
        self.checked = 0              # nodes checked against the obstacles
        self._add(np.arange(len(s)), [0])

    # ------------------------------------------------------------------ questions

    @property
    def n_layers(self) -> int:
        return len(self.s)

    def usable(self, k: int) -> np.ndarray:
        L = self.layers[k]
        return L.free & ~L.banned

    def edges(self, k: int) -> tuple[np.ndarray, np.ndarray]:
        """Edges from layer k to layer k+1.  -> pred (n1, P) node numbers in layer k (-1: none),
        cost (n1, P) summed joint motion (inf where there is no edge).

        A node may come from a neighbour: same slot; same or next absolute spin (a circle), elbow value
        and lean; and only if no joint moves more than `jump`.  Computed over all stored
        nodes, whether free or not; the search masks the ones it may not use.
        """
        if k not in self._edges:
            self._edges[k] = self._make_edges(k)
        return self._edges[k]

    def params(self, k: int, nodes: np.ndarray):
        """Spin (rad), elbow value (rad), lean (2,), slot of nodes of layer k."""
        L = self.layers[k]
        return self.spins[L.spin[nodes]] - self.theta[k], self.q7s[L.q7[nodes]], \
            self.leans[L.lean[nodes]], \
            L.slot[nodes]

    # ------------------------------------------------------------------ changes

    def widen(self, layers) -> int:
        """Open every lean on these layers.  -> how many layers changed."""
        layers = np.asarray(sorted(set(int(k) for k in layers)), int)
        todo = [k for k in layers if not self.opened[k].all()]
        if todo:
            self._add(np.asarray(todo), list(range(1, len(self.leans))))
        return len(todo)

    def ban(self, k: int, nodes) -> None:
        self.layers[k].banned[np.asarray(nodes, int)] = True

    # ------------------------------------------------------------------ building

    def _empty(self) -> Layer:
        z = np.zeros(0, int)
        return Layer(z, z, z, z, np.zeros((0, 7)), np.zeros(0, bool), np.zeros(0, bool),
                     np.full((len(self.leans), len(self.spins), len(self.q7s), N_SLOT), -1,
                             np.int32))

    def _add(self, layers: np.ndarray, leans: list[int]) -> None:
        """Every (layer, lean, spin, elbow value, slot) inside the gates, with whether it is free
        of the obstacles, stored in its layer.  From the kinematic table when there is one."""
        found = self._looked_up(layers, leans) if self.table is not None else \
            self._solved(layers, leans)
        K, Si, Li, Qi, b, Q, answers = found
        free = self.judge.obstacle_clear(Q) >= 0.0 if len(Q) else np.zeros(0, bool)
        self.checked += len(Q)
        for i, k in enumerate(layers):
            sel = K == k
            L = self.layers[k]
            n0 = len(L.q)
            L.spin = np.concatenate([L.spin, Si[sel]])
            L.q7 = np.concatenate([L.q7, Qi[sel]])
            L.lean = np.concatenate([L.lean, Li[sel]])
            L.slot = np.concatenate([L.slot, b[sel]])
            L.q = np.concatenate([L.q, Q[sel]])
            L.free = np.concatenate([L.free, free[sel]])
            L.banned = np.concatenate([L.banned, np.zeros(int(sel.sum()), bool)])
            L.index[L.lean[n0:], L.spin[n0:], L.q7[n0:], L.slot[n0:]] = np.arange(n0, len(L.q))
            L.answers += int(answers[i])
            self.opened[k, leans] = True
            self._edges.pop(k - 1, None)
            self._edges.pop(k, None)

    def _looked_up(self, layers, leans):
        from aris.local.table import lookup
        parts, answers = [], []
        for k in layers:
            got = lookup(self.table, self.arm, self.tips[k], leans, self.judge.gates)
            self.lookups += len(self.spins) * len(leans) * len(self.q7s)
            if got is None:
                answers.append(0)
                continue
            si, li, qi, b, q, n = got
            parts.append((np.full(len(si), k), si, li, qi, b, q))
            answers.append(n)
        if not parts:
            z = np.zeros(0, int)
            return z, z, z, z, z, np.zeros((0, 7)), np.array(answers)
        cat = [np.concatenate([p[i] for p in parts]) for i in range(6)]
        return (*cat, np.array(answers))

    def _solved(self, layers, leans):
        ns, nq = len(self.spins), len(self.q7s)
        K, Li, Si, Qi = np.meshgrid(layers, leans, np.arange(ns), np.arange(nq), indexing="ij")
        K, Li, Si, Qi = K.ravel(), Li.ravel(), Si.ravel(), Qi.ravel()
        found = []
        for a in range(0, len(K), _CHUNK):
            sl = slice(a, a + _CHUNK)
            T = self.arm.hand_pose(self.tips[K[sl]], self.judge.normal,
                                   self.spins[Si[sl]] - self.theta[K[sl]],
                                   self.leans[Li[sl]])
            Q, ok = self.arm.ik(T, self.q7s[Qi[sl]])
            self.ik_poses += len(T)
            m, b = np.nonzero(ok)
            found.append((m + a, b, Q[m, b]))
        m = np.concatenate([f[0] for f in found])
        b = np.concatenate([f[1] for f in found])
        Q = np.concatenate([f[2] for f in found]).reshape(-1, 7)
        answers = np.bincount(np.searchsorted(layers, K[m]), minlength=len(layers))
        keep = self.judge.reachable(Q)
        m, b, Q = m[keep], b[keep], Q[keep]
        return K[m], Si[m], Li[m], Qi[m], b, Q, answers

    def _make_edges(self, k: int):
        A, B = self.layers[k], self.layers[k + 1]
        n1 = len(B.q)
        ns, nq = len(self.spins), len(self.q7s)
        if n1 == 0 or len(A.q) == 0:
            return np.full((n1, 1), -1, int), np.full((n1, 1), np.inf)
        # the same absolute spin sits `shift` grid steps further round at layer k than at k+1
        turn = np.mod(self.theta[k] - self.theta[k + 1] + np.pi, 2 * np.pi) - np.pi
        shift = int(np.round(turn / (self.spins[1] - self.spins[0]))) if ns > 1 else 0
        nb = self.lean_nb[B.lean]                                       # (n1, M)
        off = np.array([(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1)])
        sp = (B.spin[:, None, None] + shift + off[None, None, :, 0]) % ns    # (n1, 1, 9)
        q7 = B.q7[:, None, None] + off[None, None, :, 1]
        ln = nb[:, :, None]                                              # (n1, M, 1)
        ok = (q7 >= 0) & (q7 < nq) & (ln >= 0)
        pred = A.index[np.maximum(ln, 0), sp, np.clip(q7, 0, nq - 1), B.slot[:, None, None]]
        pred = np.where(ok, pred, -1).reshape(n1, -1)
        dq = np.abs(B.q[:, None, :] - A.q[np.maximum(pred, 0)])
        allowed = (pred >= 0) & (dq.max(axis=2) <= self.cfg.jump)
        cost = np.where(allowed, dq.sum(axis=2), np.inf)
        return np.where(allowed, pred, -1), cost
