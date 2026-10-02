"""A simulated arm under position control with a fake paper, and touch motions made the way
the calibration planner makes them (a hover 60 mm up, straight down to the nominal paper and
back, timed slowly)."""
import dataclasses

import numpy as np

from aris.kernel.retime import retime
from aris.types import JointPath, Refusal
from aris_robot.touch import FakePaper, Kinematics, straight_on  # noqa: F401
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


def manual_touch(kin: Kinematics, q_hover, depth: float, extra: float, speed: float = 0.005):
    """A touch motion as the calibration planner makes it: from the hover straight down
    `depth`, timed at `speed`, and back up the same way.  -> Motion, or a Refusal."""
    from aris.types import Motion
    down = straight_on(kin.arm, kin.arm.limits, kin.rules, q_hover, kin.down, depth, speed)
    if isinstance(down, Refusal):
        return down
    path = np.concatenate([down.q, down.q[-2::-1]])
    tips = kin.tip(path)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))])
    traj = retime(JointPath(path), kin.arm.limits, dataclasses.replace(kin.rules, draw_speed=speed),
                  s=s)
    if isinstance(traj, Refusal):
        return traj
    return Motion("touch", traj, tip_base=kin.tip(traj.q), extra_depth=extra)


def touch_motion(rig, arm_id, q_hover, height=0.06, extra=0.02, speed=0.005):
    return manual_touch(Kinematics.of(rig, arm_id), q_hover, height, extra, speed)
