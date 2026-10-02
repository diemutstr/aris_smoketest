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
    p = rig.park_q("2L")
    near = p + np.array([0.03, -0.02, 0, 0, 0.01, 0, 0])
    traj = tools.park_move(rig, "2L", near)
    assert np.array_equal(traj.q[0], near) and np.allclose(traj.q[-1], p, atol=1e-12)
    assert check(traj, rig.arm("2L").limits).inside
    far = tools.park_move(rig, "2L", p + 0.2)
    assert isinstance(far, Refusal) and far.reason == "too_far" and "park job" in far.detail
    assert tools.park_move(rig, "2L", p).reason == "at_park"


def test_jog_is_small_and_inside_the_limits(rig):
    p = rig.park_q("2L")
    q = tools.jog_target(rig, "2L", p, 7, 0.05)
    assert np.allclose(q - p, [0, 0, 0, 0, 0, 0, 0.05])
    assert tools.jog_target(rig, "2L", p, 7, 0.2).reason == "too_far"
    assert tools.jog_target(rig, "2L", p, 0, 0.05).reason == "bad_joint"
    edge = p.copy()
    edge[6] = rig.arm("2L").limits.q_max[6] - 0.06
    assert tools.jog_target(rig, "2L", edge, 7, 0.05).reason == "limit"


def test_the_hand_touch_goes_straight_down_and_back(rig):
    from aris_robot.touch import Kinematics, manual_touch
    from sim_touch import hover_q
    kin = Kinematics.of(rig, "2L")
    q0 = hover_q(rig, "2L", height=0.04)
    m = manual_touch(kin, q0, 0.03, 0.01)
    tips = kin.tip(m.traj.q)
    depth = (tips - tips[0]) @ kin.down
    assert m.kind == "touch" and m.extra_depth == 0.01
    assert depth.max() == pytest.approx(0.03, abs=1e-5)
    assert np.allclose(m.q_start, q0) and np.allclose(m.q_end, q0, atol=1e-12)
    lateral = (tips - tips[0]) - np.outer(depth, kin.down)
    assert np.abs(lateral).max() < 2e-5
    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / np.diff(m.traj.t)
    assert speed.max() <= 0.005 * 1.05
