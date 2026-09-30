"""The reference stream: the trajectory sampled at 1 kHz is the planner's trajectory."""
from pathlib import Path

import numpy as np
import pytest

from aris.kernel.retime import sample
from aris.rig import Rig
from aris_robot.force import profile
from aris_robot.stream import Pacer, cubic, grid, samples
from motions import random_trajectory

CONFIG = Path(__file__).resolve().parents[2] / "config"


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


@pytest.mark.parametrize("arm_id,seed", [(31, 0), (71, 1), (13, 2), (97, 3)])
def test_sampling_at_1khz_matches_the_planners_sample(rig, arm_id, seed):
    traj = random_trajectory(rig, arm_id, seed)
    t = grid(traj.t)
    q, qd = cubic(traj.t, traj.q, traj.qd, t)
    q_ref, qd_ref, _ = sample(traj, t)
    assert np.abs(q - q_ref).max() <= 1e-12
    assert np.abs(qd - qd_ref).max() <= 1e-12
    # and between the millisecond samples too, and outside the trajectory (at rest)
    t_off = np.concatenate([[traj.t[0] - 1.0], t[:-1] + 0.00037, [traj.t[-1] + 1.0]])
    q, qd = cubic(traj.t, traj.q, traj.qd, t_off)
    q_ref, qd_ref, _ = sample(traj, t_off)
    assert np.abs(q - q_ref).max() <= 1e-12 and np.abs(qd - qd_ref).max() <= 1e-12


def test_grid_is_every_millisecond_and_ends_on_the_last_knot(rig):
    traj = random_trajectory(rig, 31, 5)
    t = grid(traj.t)
    assert t[0] == traj.t[0] and t[-1] == traj.t[-1]
    assert np.all(np.diff(t) <= 0.001 + 1e-12) and np.all(np.diff(t) > 0)
    assert np.allclose(np.diff(t[:-1]), 0.001)


def test_samples_carry_the_force_into_the_paper(rig):
    traj = random_trajectory(rig, 31, 6)
    n = rig.paper(31).normal
    fn = profile("draw", traj.t, None, 1.0, _settings())
    s = samples(traj, fn, n)
    assert s.t[0] == 0.0 and len(s) == len(grid(traj.t))
    assert np.allclose(s.f, -np.outer(fn(s.t), n))              # along minus the normal
    assert np.all(s.f[:, 2] >= 0.0)                             # base z points down here
    assert np.array_equal(s.q[-1], traj.q[-1])
    assert np.allclose(s.n, n / np.linalg.norm(n))              # pen down: the normal rides along
    assert np.all(samples(traj, fn, n, pen_down=False).n == 0.0)


def test_pacer_sends_everything_once_in_order_ahead_of_time(rig):
    traj = random_trajectory(rig, 31, 7)
    s = samples(traj, lambda t: np.zeros_like(t), rig.paper(31).normal)
    p = Pacer(s, stream=42, lead=0.1, max_chunk=50)
    sent, elapsed = [], 0.0
    while not p.done:
        for c in p.due(elapsed):
            assert c.stream == 42
            assert c.t[-1] <= elapsed + 0.1 + 1e-12          # never more than the lead ahead
            sent.append(c)
        elapsed += 0.013
    t = np.concatenate([c.t for c in sent])
    assert np.array_equal(t, s.t)
    assert [c.last for c in sent] == [False] * (len(sent) - 1) + [True]
    assert max(len(c.t) for c in sent) <= 50


def _settings():
    from aris_robot.force import ForceSettings
    return ForceSettings()
