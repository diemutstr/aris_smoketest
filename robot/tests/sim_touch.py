"""A simulated arm under position control with a fake paper, and touch motions made the way
the calibration planner makes them (a hover 60 mm up, straight down to the nominal paper and
back, timed slowly)."""
import numpy as np

from aris.types import Motion
from aris_robot.touch import FakePaper, Kinematics, manual_touch

from aris_robot.simarm import SimPosition as SimPositionArm  # noqa: F401  (the tests' name)


def hover_q(rig, arm_id, xy=(0.45, 0.1), height=0.06):
    kin = Kinematics.of(rig, arm_id)
    arm, n = kin.arm, kin.normal
    paper = rig.paper(arm_id)
    p = np.array([xy[0], xy[1], 0.0])
    p = p + (paper.offset - p @ n) * n + height * n          # height above the nominal paper
    T = arm.hand_pose(p[None], n, 0.3, np.zeros(2))
    for q7 in (-1.0, 0.0, 1.0):
        Q, ok = arm.ik(T, q7)
        if ok[0].any():
            return Q[0][ok[0]][0]
    raise AssertionError("no hover")


def touch_motion(rig, arm_id, q_hover, height=0.06, extra=0.02, speed=0.005):
    return manual_touch(Kinematics.of(rig, arm_id), q_hover, height, extra, speed)
