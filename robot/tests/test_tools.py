"""The operator tools' moves: small, straight, refused when not small."""
from pathlib import Path

import numpy as np
import pytest

from aris.kernel.retime import check
from aris.rig import Rig
from aris.types import Refusal
from aris_robot import tools

CONFIG = Path(__file__).resolve().parents[2] / "config"


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


def test_park_only_from_close_by(rig):
    p = rig.park_q(31)
    near = p + np.array([0.03, -0.02, 0, 0, 0.01, 0, 0])
    traj = tools.park_move(rig, 31, near)
    assert np.array_equal(traj.q[0], near) and np.allclose(traj.q[-1], p, atol=1e-12)
    assert check(traj, rig.arm(31).limits).inside
    far = tools.park_move(rig, 31, p + 0.2)
    assert isinstance(far, Refusal) and far.reason == "too_far" and "park job" in far.detail
    assert tools.park_move(rig, 31, p).reason == "at_park"


def test_jog_is_small_and_inside_the_limits(rig):
    p = rig.park_q(31)
    q = tools.jog_target(rig, 31, p, 7, 0.05)
    assert np.allclose(q - p, [0, 0, 0, 0, 0, 0, 0.05])
    assert tools.jog_target(rig, 31, p, 7, 0.2).reason == "too_far"
    assert tools.jog_target(rig, 31, p, 0, 0.05).reason == "bad_joint"
    edge = p.copy()
    edge[6] = rig.arm(31).limits.q_max[6] - 0.06
    assert tools.jog_target(rig, 31, edge, 7, 0.05).reason == "limit"


def test_touch_descends_straight_down_keeping_the_hand(rig):
    arm, n = rig.arm(31), rig.paper(31).normal
    T = arm.hand_pose(np.array([[0.45, 0.1, 0.97 - 0.04]]), n, 0.3, np.zeros(2))
    q0 = next(Q[0][ok[0]][0] for Q, ok in (arm.ik(T, q7) for q7 in (-1.0, 0.0, 1.0))
              if ok[0].any())
    traj = tools.descent(rig, 31, q0, 0.03)
    tips = arm.tip(traj.q)
    move = tips - tips[0]
    assert np.allclose(move - np.outer(move @ -n, -n), 0.0, atol=2e-5)     # straight along -n
    assert (tips[-1] - tips[0]) @ -n == pytest.approx(0.03, abs=1e-6)
    R = arm.fk(traj.q)[:, :3, :3]
    assert np.allclose(R, R[0], atol=1e-4)                                  # hand keeps its turn
    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(traj.t)
    assert speed.max() <= tools.TOUCH_SPEED * 1.05
    assert tools.descent(rig, 31, q0, 0.1).reason == "too_far"
