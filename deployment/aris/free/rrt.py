"""Bidirectional RRT (RRT-Connect): one tree from each end, each growing toward the other.

Each round draws random configurations inside the joint limits (a small batch of them), grows
one tree one step toward each, then lets the other tree run straight at every new node for as
long as the way is free.  The trees swap roles every round.  When a run reaches its node, the
two trees meet and the path is read off both.

A round's steps are checked in one batch and its runs in another (every step of every run at
once, then each run keeps its free prefix): one call into the collision check costs far more
than one configuration inside it.  The search stops after a fixed number of configurations
checked (not after a time), so the same question gets the same answer on any machine.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Tree:
    q: np.ndarray            # (cap, 7)
    parent: np.ndarray       # (cap,) int, -1 at the root
    n: int = 1

    @staticmethod
    def rooted(q0: np.ndarray, cap: int = 1024) -> "Tree":
        t = Tree(np.empty((cap, 7)), np.full(cap, -1, np.int64))
        t.q[0] = q0
        return t

    def add(self, qs: np.ndarray, parent: int) -> int:
        """Append a chain of nodes, each the child of the one before; -> index of the last."""
        need = self.n + len(qs)
        if need > len(self.q):
            cap = max(need, 2 * len(self.q))
            self.q = np.concatenate([self.q, np.empty((cap - len(self.q), 7))])
            self.parent = np.concatenate([self.parent, np.full(cap - len(self.parent), -1)])
        for q in qs:
            self.q[self.n] = q
            self.parent[self.n] = parent
            parent = self.n
            self.n += 1
        return parent

    def nearest(self, q: np.ndarray) -> int:
        return int(np.argmin(np.sum((self.q[:self.n] - q) ** 2, axis=1)))

    def branch(self, i: int) -> np.ndarray:
        """Configurations from node i back to the root."""
        out = []
        while i >= 0:
            out.append(self.q[i])
            i = int(self.parent[i])
        return np.array(out)


def _steps(qa: np.ndarray, qb: np.ndarray, step: float) -> np.ndarray:
    """Points from qa (excluded) to qb (included), at most `step` apart."""
    k = max(1, int(np.ceil(np.linalg.norm(qb - qa) / step)))
    return qa + np.linspace(0.0, 1.0, k + 1)[1:, None] * (qb - qa)


@dataclass(frozen=True)
class SearchResult:
    path: np.ndarray | None  # (M, 7) from q_a to q_b, or None
    rounds: int
    nodes: int


def connect(checker, q_a: np.ndarray, q_b: np.ndarray, rng: np.random.Generator,
            max_checks: int, step: float = 1.0, batch: int = 8) -> SearchResult:
    """A free joint path from q_a to q_b, or none within `max_checks` configurations checked.

    Each round grows one tree toward `batch` random targets at once and then lets the other
    tree run at every new node, all edges of a round in two batched checks."""
    lo, hi = checker.q_lo, checker.q_hi
    trees = [Tree.rooted(q_a), Tree.rooted(q_b)]
    start = checker.n_checked
    rounds = 0
    while checker.n_checked - start < max_checks:
        rounds += 1
        grow, other = trees[(rounds - 1) % 2], trees[rounds % 2]
        targets = rng.uniform(lo, hi, (batch, 7))
        near = [grow.nearest(t) for t in targets]
        q_near = grow.q[near]
        d = targets - q_near
        dist = np.linalg.norm(d, axis=1, keepdims=True)
        q_new = np.where(dist <= step, targets, q_near + d * (step / np.maximum(dist, 1e-12)))
        good = checker.edges(q_near, q_new)
        new = [(grow.add(q[None], i), q) for i, q, g in zip(near, q_new, good) if g]
        if not new:
            continue
        runs = []                                    # (index in other, chain to the new node)
        for _, q in new:
            j = other.nearest(q)
            runs.append((j, _steps(other.q[j], q, step)))
        ok = checker.edges(np.concatenate([np.concatenate([other.q[j][None], c[:-1]])
                                           for j, c in runs]),
                           np.concatenate([c for _, c in runs]))
        at = 0
        for (i_new, _), (j, chain) in zip(new, runs):
            part = ok[at:at + len(chain)]
            at += len(chain)
            free = len(chain) if part.all() else int(np.argmin(part))
            if free:
                j = other.add(chain[:free], j)
            if free == len(chain):                   # the trees meet at this new node
                a_side, b_side = (grow, other) if grow is trees[0] else (other, grow)
                ia, ib = (i_new, j) if grow is trees[0] else (j, i_new)
                path = np.concatenate([a_side.branch(ia)[::-1], b_side.branch(ib)[1:]])
                return SearchResult(path, rounds, trees[0].n + trees[1].n)
    return SearchResult(None, rounds, trees[0].n + trees[1].n)
