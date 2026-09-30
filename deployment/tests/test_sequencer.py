"""Tests of the sequencer (aris/sequencer): the parts, and the tour on its own.

Quick set: `../.venv/bin/python -m pytest tests/test_sequencer.py -q`.  The end-to-end tests
(local planner and sequencer together) are in tests/test_arm_planner.py.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import local_cases as lc  # noqa: E402
from test_arm_planner import RIG, assert_tour, two_lines  # noqa: E402

from aris import local  # noqa: E402
from aris.kernel.retime import sample  # noqa: E402
from aris.sequencer import TourOptions, TourReport, price, tour, tour_all  # noqa: E402
from aris.sequencer.draw import corners, draw_motions  # noqa: E402
from aris.sequencer.guard import Guard  # noqa: E402
from aris.sequencer.lift import lift, reverse  # noqa: E402


@pytest.fixture(scope="module")
def problem():
    arm, obs, rules, _ = lc.problem(RIG, 31)
    bunches, left = local.plan(arm, two_lines(), obs, rules)
    assert len(bunches) == 2 and not left
    return arm, obs, rules, bunches


def test_price_is_the_slowest_joint_at_the_allowed_speed(problem):
    arm, _, rules, _ = problem
    q0 = np.zeros(7)
    Q = np.zeros((2, 7))
    Q[0, 3] = 0.5
    Q[1, 0], Q[1, 6] = 0.1, 0.3
    got = price(q0, Q, rules, arm.limits.qd_max)
    v = rules.speed_fraction * arm.limits.qd_max
    assert np.allclose(got, [0.5 / v[3], max(0.1 / v[0], 0.3 / v[6])])


def test_lift_off_goes_straight_up_and_the_set_down_is_the_same_backwards(problem):
    arm, obs, rules, bunches = problem
    guard = Guard(arm, obs, rules.gates)
    paper = [p for p in obs.planes if p.kind == "paper"][0]
    q = bunches[0].plans[0].q_start
    up = lift(arm, guard, paper, q, rules, 0.002, 0.05, TourOptions().lift_turns)
    assert not isinstance(up, str), up
    assert np.array_equal(up.up.q_start, q) and np.array_equal(up.down.q_end, q)
    assert np.max(np.abs(up.down.q_start - up.q_up)) == 0.0
    n = paper.normal / np.linalg.norm(paper.normal)
    t = np.linspace(0, up.up.traj.t[-1], 200)
    tip = arm.tip(sample(up.up.traj, t)[0])
    rise = (tip - tip[0]) @ n
    side = np.linalg.norm((tip - tip[0]) - rise[:, None] * n, axis=1)
    assert abs(rise[-1] - rules.lift_height) < 1e-6 and side.max() < 2e-4
    assert np.all(np.diff(rise) > -1e-6)                         # it only rises
    back = sample(up.down.traj, up.down.traj.t[-1] - t)[0]
    assert np.allclose(back, sample(up.up.traj, t)[0], atol=1e-12)
    assert guard.hold(up.q_up, touching=False) is None


def test_reverse_of_a_trajectory():
    from aris.types import Trajectory
    tr = Trajectory(np.array([0.0, 0.5, 2.0]), np.arange(21.0).reshape(3, 7),
                    np.vstack([np.zeros(7), np.ones(7), np.zeros(7)]))
    r = reverse(tr)
    assert np.allclose(r.t, [0.0, 1.5, 2.0]) and np.array_equal(r.q[0], tr.q[-1])
    assert np.allclose(r.qd[1], -1.0)


def test_drawing_is_cut_at_sharp_corners_only(problem):
    tip = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [1.9, 0.05, 0], [1.9, 1, 0]], float)
    assert corners(tip, np.deg2rad(120)).tolist() == [2]
    arm, obs, rules, bunches = problem
    plan = bunches[0].plans[0]
    ms = draw_motions(arm, Guard(arm, obs, rules.gates), plan, rules, 1.5e-4, np.deg2rad(120))
    assert len(ms) == 1 and ms[0].kind == "draw"                  # a straight line: one motion
    assert np.max(np.linalg.norm(arm.tip(ms[0].traj.q) - ms[0].tip_base, axis=1)) < 1.2e-4


def test_empty_drawing_is_only_the_move_to_q_end(problem):
    arm, obs, rules, _ = problem
    q0 = RIG.park_q(31)
    q1 = q0.copy()
    q1[0] -= 0.3
    ms, left, rep = tour_all(arm, [], q0, obs, rules, q_end=q1)
    assert len(ms) == 1 and ms[0].kind == "free" and left == []
    assert_tour(arm, obs, rules, ms, q0, q1)
    assert rep.pieces == 0 and rep.draw_time == 0.0 and rep.penup_share == 1.0
    ms, left, _ = tour_all(arm, [], q0, obs, rules)              # back where it is: nothing
    assert ms == [] and left == []


def test_a_piece_nothing_can_fly_to_is_a_leftover_with_the_reason(problem):
    arm, obs, rules, bunches = problem
    q_bad = RIG.park_q(31).copy()
    q_bad[3] = arm.limits.q_max[3] - 0.01                        # inside the gate's margin
    rep = TourReport()
    gen = tour(arm, bunches, q_bad, obs, rules, q_end=RIG.park_q(31), report=rep)
    motions = []
    try:
        while True:
            motions.append(next(gen))
    except StopIteration as stop:
        left = stop.value
    assert motions == []
    assert sorted(x.piece.line_id for x in left) == ["a", "b"]
    assert all(x.reason == "no_free_path" and "outside_limits" in x.detail for x in left)
    assert rep.end_refusal.startswith("outside_limits")
    assert rep.refusals == {"outside_limits": rep.free_calls}


def test_a_piece_under_a_box_is_a_leftover_and_the_rest_is_drawn(problem):
    arm, obs, rules, bunches = problem
    from aris.types import Box, Obstacles
    # A flat box 20 to 40 mm above line b, known to the sequencer only: no alternative of b
    # can be lifted off (nor drawn); line a is drawn as before.
    x, y = RIG.T_table_base(31)[:2, 3]
    T = np.eye(4)
    T[:3, 3] = [x + 0.10, y + 0.325, 0.030]
    box = Box("lid", RIG.T_base_table(31) @ T, np.array([0.05, 0.12, 0.010]), 0.0)
    lid = Obstacles(obs.boxes + (box,), obs.planes, obs.capsules)
    ms, left, rep = tour_all(arm, bunches, RIG.park_q(31), lid, rules)
    assert rep.pieces == 1 and len(left) == 1
    assert left[0].piece.line_id == "b" and left[0].reason in ("no_free_path", "unreachable")
    assert left[0].detail
    assert_tour(arm, lid, rules, ms, RIG.park_q(31), RIG.park_q(31))
