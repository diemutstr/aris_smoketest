"""Timed test trajectories near an arm's park, made by the planner's own timing."""
import numpy as np

from aris.kernel.retime import retime
from aris.types import JointPath


def random_trajectory(rig, arm_id, seed, n=5, spread=0.2):
    rng = np.random.default_rng(seed)
    p = rig.park_q(arm_id)
    qs = [p] + [p + rng.normal(0.0, spread, 7) for _ in range(n - 1)]
    return retime(JointPath(np.array(qs)), rig.arm(arm_id).limits, rig.rules())
