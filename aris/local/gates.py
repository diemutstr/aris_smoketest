"""The gates a drawing configuration must pass, and the clearance along a drawing path.

One object per (arm, obstacles, gates), built once and asked many times.  The tests come in
the order the planner asks them: joint limits and singular value (cheap, on every node), then
the arm against the paper and against itself, then the other obstacles (the expensive test,
only on what survived).  A node that fails only the last test is "blocked";
one that fails an earlier test is "unreachable" for this arm.

The paper while drawing: the kernel's rule (`drawing=True`).  The pen is on the paper on
purpose and not checked against it; the tool (hand, finger blades, holder) keeps the paper's
`tool_margin`; the arm's links keep its `margin`.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from aris.kernel import collide as C
from aris.types import Gates, Obstacles, Plane

class Judge:
    """Gates and clearances for one arm among one set of obstacles, while drawing."""

    def __init__(self, arm, obstacles: Obstacles, gates: Gates):
        papers = [p for p in obstacles.planes if p.kind == "paper"]
        if len(papers) != 1:
            raise ValueError(f"the obstacles must hold exactly one plane of kind 'paper', "
                             f"not {len(papers)}")
        self.arm, self.gates = arm, gates
        self.paper: Plane = papers[0]
        self.normal = np.asarray(self.paper.normal, float) / np.linalg.norm(self.paper.normal)
        # everything but the paper
        self.rest = C.pack(replace(obstacles, planes=tuple(p for p in obstacles.planes
                                                           if p.kind != "paper")))
        self.tables = C.arm_tables(arm)
        self._paper_only = C.pack(Obstacles(planes=(self.paper,)))
        self.reach = reach = np.asarray(arm.reach, float)
        # How far any capsule point moves per radian of joint motion (2-norm): the flown path is
        # within the timing step's deviation of this path, so clearance can drop by this much.
        self.lipschitz = float(np.max(np.linalg.norm(reach, axis=0)))

    # ------------------------------------------------------------------ per configuration

    def kinematic(self, Q) -> np.ndarray:
        """(N,) joint-limit margin and singular value both inside the gates."""
        Q = np.asarray(Q, float).reshape(-1, 7)
        ok = self.arm.limit_margin(Q) >= self.gates.limit_margin
        idx = np.flatnonzero(ok)
        if len(idx):
            ok[idx] = self.arm.sigma_min(Q[idx]) >= self.gates.sigma_min
        return ok

    def self_clear(self, Q) -> np.ndarray:
        """(N,) clearance of the arm against itself, beyond the self margin."""
        return C.self_clearance_q(self.tables, Q, self.arm.self_pairs, self.gates.self_margin)

    def paper_clear(self, Q) -> np.ndarray:
        """(N,) clearance against the paper while drawing (the kernel's rule)."""
        return C.clearance_q(self.tables, Q, self._paper_only, drawing=True)

    def obstacle_clear(self, Q) -> np.ndarray:
        """(N,) clearance against every obstacle but the paper."""
        return C.clearance_q(self.tables, Q, self.rest, drawing=True)

    def reachable(self, Q) -> np.ndarray:
        """(N,) passes every gate that does not depend on obstacles other than the paper."""
        Q = np.asarray(Q, float).reshape(-1, 7)
        ok = self.kinematic(Q)
        for test in (self.paper_clear, self.self_clear):      # cheaper first
            idx = np.flatnonzero(ok)
            if len(idx):
                ok[idx] = test(Q[idx]) >= 0.0
        return ok

    def blocker(self, Q) -> str:
        """The obstacle that most often is the closest one for these configurations."""
        if not len(Q):
            return ""
        d = C.clearance_detail_q(self.tables, Q, self.rest, drawing=True)
        names = [d.obstacle_names[m] for m in d.obstacle[d.value < 0.0] if m >= 0]
        if not names:
            return ""
        u, n = np.unique(names, return_counts=True)
        return str(u[np.argmax(n)])

    # ------------------------------------------------------------------ along a path

    def path_clear(self, q) -> float:
        """A lower bound on the clearance along the joint path q (N,7), between samples too,
        against every obstacle and the paper (the kernel's drawing rule)."""
        q = np.asarray(q, float).reshape(-1, 7)
        rest = C.path_clearance_q(self.tables, self.reach, q, self.rest, drawing=True) \
            if len(self.rest.names) else np.inf        # nothing but the paper: nothing to halve for
        return float(min(rest, C.path_clearance_q(self.tables, self.reach, q, self._paper_only,
                                                  drawing=True)))
