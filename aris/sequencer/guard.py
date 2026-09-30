"""What the sequencer checks itself, with the kernel: can the arm hold a configuration, and is a
motion the sequencer made (a lift-off, a set-down, a drawing) free as it will be flown.

The free-space planner checks its own motions as flown; the ones made here are checked here.
"Touching" means the pen is on (or within a lift of) the paper on purpose: the pen is then not
judged against the paper (the kernel's `drawing=True`); everything else is.
"""
from __future__ import annotations

import numpy as np

from aris.kernel import collide
from aris.kernel.retime import sample
from aris.types import Gates, Obstacles, Trajectory


def piece_accel(traj: Trajectory) -> np.ndarray:
    """(N-1, 7): the largest |q''| on each cubic piece (q'' is linear on a piece: an end)."""
    h = np.diff(traj.t)[:, None]
    dq = np.diff(traj.q, axis=0)
    v0, v1 = traj.qd[:-1], traj.qd[1:]
    a0 = (6.0 * dq - h * (4.0 * v0 + 2.0 * v1)) / h ** 2
    a1 = (-6.0 * dq + h * (2.0 * v0 + 4.0 * v1)) / h ** 2
    return np.maximum(np.abs(a0), np.abs(a1))


SUB = 4               # the flown curve is checked this many times finer than it is sampled


class Guard:
    """Holding and flown-clearance checks for one arm among one set of obstacles."""

    def __init__(self, arm, obstacles: Obstacles, gates: Gates):
        self.arm, self.gates = arm, gates
        self.tables = collide.arm_tables(arm)
        self.packed = collide.pack(obstacles)
        self.reach = np.asarray(arm.reach, float)                 # (7, K) m per rad
        self.pairs = np.asarray(arm.self_pairs, np.int64)

    def hold(self, q, touching: bool) -> str | None:
        """None if the arm may stand at q for as long as it likes; else why not."""
        q = np.asarray(q, float).reshape(1, 7)
        margin = float(self.arm.limit_margin(q)[0])
        if margin < self.gates.limit_margin:
            return f"{margin:.4f} rad from a joint limit (gate {self.gates.limit_margin})"
        det = collide.clearance_detail_q(self.tables, q, self.packed, drawing=touching)
        if det.value[0] < 0.0:
            return (f"{det.capsule_names[det.capsule[0]]} {-det.value[0] * 1e3:.2f} mm inside "
                    f"the margin of {det.obstacle_names[det.obstacle[0]]}")
        own = float(collide.self_clearance_q(self.tables, q, self.pairs,
                                             self.gates.self_margin)[0])
        if own < 0.0:
            return f"{-own * 1e3:.2f} mm inside its own self margin"
        return None

    def flown(self, traj: Trajectory, touching: bool, sub: int = SUB) -> float:
        """A lower bound on the clearance of the timed trajectory as flown, obstacles and the arm
        against itself, metres beyond the margins.

        The flown curve is the cubic between the trajectory's samples; it is sampled `sub`
        times finer (the samples plus sub - 1 points of the same cubic between each two).  The
        kernel bounds the clearance along the straight joint moves between those points; the
        cubic strays from each such chord by at most max|q''| dt^2 / 8 per joint (q'' of the
        piece it lies on, dt the finer gap), and that is charged, turned into metres by how far
        each joint can move each capsule.  Both the kernel's between-point bound and the charge
        shrink with the gap, so a finer sampling reads closer to the truth, never above it."""
        if len(traj.t) < 2:
            return float("inf")
        u = np.arange(sub) / sub                                             # (sub,)
        h = np.diff(traj.t)
        t = np.concatenate([(traj.t[:-1, None] + h[:, None] * u[None]).ravel(), traj.t[-1:]])
        q = sample(traj, t)[0]
        q[::sub] = traj.q                                   # the samples themselves, exactly
        sag = piece_accel(traj) * (h / sub)[:, None] ** 2 / 8.0             # (N-1, 7) rad
        move = sag @ self.reach                                              # (N-1, K) m
        obst = collide.path_clearance_q(self.tables, self.reach, q, self.packed,
                                        drawing=touching)
        own = collide.path_self_clearance_q(self.tables, self.reach, q, self.pairs,
                                            self.gates.self_margin)
        return float(min(obst - move.max(), own - 2.0 * move.max()))
