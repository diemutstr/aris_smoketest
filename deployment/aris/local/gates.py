"""The gates a drawing configuration must pass, and the clearance along a drawing path.

One object per (arm, obstacles, gates), built once and asked many times.  The tests come in
the order the planner asks them: joint limits and singular value (cheap, on every node), then
the arm against the paper and against itself, then the other obstacles (the expensive test,
only on what survived).  A node that fails only the last test is "blocked";
one that fails an earlier test is "unreachable" for this arm.

The paper while drawing.  The pen is on the paper on purpose, so it is not checked against it
(the kernel's `drawing=True`).  Everything bolted to the hand (hand, finger blades, holder,
pencil tail) is rigid with the pen, so while the pen is on the paper its height above the paper
depends on the lean alone: at zero lean the holder's cap capsule is 0.03 mm above the paper.
So while drawing the hand end must only not go into the paper (`hand_paper_margin`), and it is
judged at the samples (a lean that tips it into the paper is refused there); the arm's links
keep the paper's own margin, checked along the whole path like every other obstacle.
"""
from __future__ import annotations

import numpy as np

from aris.kernel import collide as C
from aris.types import Body, Gates, Obstacles, Plane

HAND_FRAME = 9      # frame index of the hand in the arm's chain (kernel/fr3.py)


def _subset(body: Body, idx: np.ndarray) -> Body:
    fixed = None if body.is_fixed is None else np.asarray(body.is_fixed)[idx]
    return Body(body.p0[:, idx], body.p1[:, idx], np.asarray(body.radius)[idx],
                tuple(body.names[i] for i in idx), np.asarray(body.is_pen)[idx], fixed)


class Judge:
    """Gates and clearances for one arm among one set of obstacles, while drawing."""

    def __init__(self, arm, obstacles: Obstacles, gates: Gates, hand_paper_margin: float):
        papers = [p for p in obstacles.planes if p.kind == "paper"]
        if len(papers) != 1:
            raise ValueError(f"the obstacles must hold exactly one plane of kind 'paper', "
                             f"not {len(papers)}")
        self.arm, self.gates = arm, gates
        self.paper: Plane = papers[0]
        self.normal = np.asarray(self.paper.normal, float) / np.linalg.norm(self.paper.normal)
        self.rest = C.pack(Obstacles(obstacles.boxes,
                                     tuple(p for p in obstacles.planes if p.kind != "paper"),
                                     obstacles.capsules))
        self.tables = C.arm_tables(arm)
        hand = np.asarray(self.tables.cap_frame) == HAND_FRAME
        self.hand_idx = np.flatnonzero(hand)
        self.link_idx = np.flatnonzero(~hand & ~np.asarray(self.tables.is_fixed))
        self._paper_only = C.pack(Obstacles(planes=(self.paper,)))
        # The contract rule (types.Body.is_tool, Plane.tool_margin, kernel.collide) replaces the
        # rule of this module as soon as the arm marks its tool capsules.
        self.kernel_rule = (arm.body(np.zeros((1, 7))).is_tool is not None
                            and self.paper.tool_margin is not None)
        self._hand_bonus = np.zeros(len(self.tables.radius))
        self._hand_bonus[self.hand_idx] = self.paper.margin - hand_paper_margin
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
        """(N,) clearance against the paper, with the drawing rule (pen free, hand end close)."""
        if self.kernel_rule:
            return C.clearance_q(self.tables, Q, self._paper_only, drawing=True)
        body = C.body_q(self.tables, Q)
        per = C.capsule_clearance(body, self._paper_only, drawing=True)
        return (per + self._hand_bonus).min(axis=1, initial=np.inf)

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
        """A lower bound on the clearance along the joint path q (N,7), between samples too:
        the whole body against the other obstacles, the links against the paper.  (The hand
        end against the paper depends on the lean only; `paper_clear` judges it per sample.)"""
        q = np.asarray(q, float).reshape(-1, 7)
        rest = C.path_clearance_q(self.tables, self.reach, q, self.rest, drawing=True) \
            if len(self.rest.names) else np.inf        # nothing but the paper: nothing to halve for
        if self.kernel_rule:
            return float(min(rest, C.path_clearance_q(self.tables, self.reach, q,
                                                      self._paper_only, drawing=True)))
        links = C.path_clearance(lambda Q: _subset(C.body_q(self.tables, Q), self.link_idx),
                                 q, Obstacles(planes=(self.paper,)), self.reach[:, self.link_idx],
                                 drawing=True)
        return float(min(rest, links))
