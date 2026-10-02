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
from aris.sequencer import TourOptions, TourReport, price, tour  # noqa: E402
from arm_cases import drain  # noqa: E402
from aris.sequencer.draw import draw_motions  # noqa: E402
from aris.sequencer.guard import Guard  # noqa: E402
from aris.sequencer.lift import end_lift, lift, lift_height, trim
from aris.sequencer.lift import reverse  # noqa: E402


def tour_all(arm, bunches, q_start, obstacles, rules, q_end=None, **kw):
    """`tour` collected: -> (motions, leftovers, report)."""
    rep = TourReport()
    ms, left = drain(tour(arm, bunches, q_start, obstacles, rules, q_end, report=rep, **kw))
    return ms, left, rep


@pytest.fixture(scope="module")
def problem():
    arm, obs, rules, _ = lc.problem(RIG, "2L")
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


def test_lift_off_goes_straight_up_and_the_set_down_lands_slowly(problem):
    arm, obs, rules, bunches = problem
    guard = Guard(arm, obs, rules.gates)
    paper = [p for p in obs.planes if p.kind == "paper"][0]
    opt = TourOptions()
    ends = [q for b in bunches for p in b.plans for q in (p.q[0], p.q[-1])]
    ups = [lift(arm, guard, paper, q, rules, opt.lift_extra, opt.lift_step, opt.lift_jump)
           for q in ends]
    k = next(i for i, u in enumerate(ups) if not isinstance(u, str))
    q, up = ends[k], ups[k]
    assert np.array_equal(up.up.q_start, q) and np.array_equal(up.down.q_end, q)
    assert up.up.kind == "lift" and up.down.kind == "lower"
    assert np.max(np.abs(up.down.q_start - up.q_up)) == 0.0
    n = paper.normal / np.linalg.norm(paper.normal)
    t = np.linspace(0, up.up.traj.t[-1], 200)
    tip = arm.tip(sample(up.up.traj, t)[0])
    rise = (tip - tip[0]) @ n
    side = np.linalg.norm((tip - tip[0]) - rise[:, None] * n, axis=1)
    assert abs(rise[-1] - lift_height(paper, opt.lift_extra)) < 1e-6 and side.max() < 2e-4
    assert np.all(np.diff(rise) > -1e-6)                         # it only rises
    assert guard.hold(up.q_up, touching=False) is None
    # the lift keeps its fast timing (the joint limits alone): well under the landing-speed time
    assert up.up.traj.t[-1] < 0.5 * lift_height(paper, opt.lift_extra) / rules.landing_speed
    # the set-down: the same path down, the pen never faster than the landing speed
    td = np.linspace(0, up.down.traj.t[-1], int(up.down.traj.t[-1] * 4000) + 2)
    tip_d = arm.tip(sample(up.down.traj, td)[0])
    speed = np.linalg.norm(np.diff(tip_d, axis=0), axis=1) / np.diff(td)
    assert speed.max() <= rules.landing_speed * 1.005, speed.max()
    fall = (tip_d - tip_d[-1]) @ n
    off = np.linalg.norm((tip_d - tip_d[-1]) - fall[:, None] * n, axis=1)
    assert np.all(np.diff(fall) < 1e-6) and off.max() < 3e-4      # straight down, only down
    assert up.down.traj.t[-1] > lift_height(paper, opt.lift_extra) / rules.landing_speed


def test_rule_2_shortens_the_piece_until_rule_1_works(problem):
    from dataclasses import replace
    arm, obs, rules, bunches = problem
    guard = Guard(arm, obs, rules.gates)
    paper = [p for p in obs.planes if p.kind == "paper"][0]
    opt = TourOptions()
    plan = next(p for b in bunches for p in b.plans
                if not isinstance(lift(arm, guard, paper, p.q[-1], rules, opt.lift_extra,
                                       opt.lift_step, opt.lift_jump), str))
    got, cut = end_lift(arm, guard, paper, plan, 1, rules, opt)
    assert cut == 0.0 and np.array_equal(got.q_draw, plan.q[-1])
    # a jump cap nothing can meet: rule 1 fails everywhere, rule 2 gives up after 5 cm
    assert isinstance(end_lift(arm, guard, paper, plan, 1, rules, replace(opt, lift_jump=0.0)),
                      str)
    # trimming an end by 20 mm leaves the rest of the piece, at a sample
    t = trim(plan, 1, 0.02)
    L = plan.piece.s1 - plan.piece.s0
    assert abs((t.piece.s1 - t.piece.s0) - (L - 0.02)) < 0.0021 and t.piece.s0 == plan.piece.s0
    assert np.array_equal(t.q[0], plan.q[0]) and len(t.q) < len(plan.q)


def test_reverse_of_a_trajectory():
    from aris.types import Trajectory
    tr = Trajectory(np.array([0.0, 0.5, 2.0]), np.arange(21.0).reshape(3, 7),
                    np.vstack([np.zeros(7), np.ones(7), np.zeros(7)]))
    r = reverse(tr)
    assert np.allclose(r.t, [0.0, 1.5, 2.0]) and np.array_equal(r.q[0], tr.q[-1])
    assert np.allclose(r.qd[1], -1.0)


def test_one_piece_is_one_drawing_motion(problem):
    arm, obs, rules, bunches = problem
    plan = bunches[0].plans[0]
    ms = draw_motions(arm, Guard(arm, obs, rules.gates), plan, rules, 1.5e-4)
    assert len(ms) == 1 and ms[0].kind == "draw"                  # a straight line: one motion
    assert np.max(np.linalg.norm(arm.tip(ms[0].traj.q) - ms[0].tip_base, axis=1)) < 1.2e-4


def test_empty_drawing_is_only_the_move_to_q_end(problem):
    arm, obs, rules, _ = problem
    q0 = RIG.park_q("2L")
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
    q_bad = RIG.park_q("2L").copy()
    q_bad[3] = arm.limits.q_max[3] - 0.01                        # inside the gate's margin
    rep = TourReport()
    gen = tour(arm, bunches, q_bad, obs, rules, q_end=RIG.park_q("2L"), report=rep)
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
    x, y = RIG.T_table_base("2L")[:2, 3]
    T = np.eye(4)
    T[:3, 3] = [x + 0.10, y + 0.325, 0.030]
    box = Box("lid", RIG.T_base_table("2L") @ T, np.array([0.05, 0.12, 0.010]), 0.0)
    lid = Obstacles(obs.boxes + (box,), obs.planes, obs.capsules)
    ms, left, rep = tour_all(arm, bunches, RIG.park_q("2L"), lid, rules)
    assert rep.pieces == 1 and len(left) == 1
    assert left[0].piece.line_id == "b" and left[0].reason in ("no_free_path", "unreachable")
    assert left[0].detail
    assert_tour(arm, lid, rules, ms, RIG.park_q("2L"), RIG.park_q("2L"))


# --------------------------------------------------------------------------- the checker in the loop


class _Refuse:
    """A fake independent checker: refuses the first motion of `kind` it sees, passes the rest.
    Records every call as (kind, q_before)."""

    def __init__(self, kind):
        self.kind, self.calls, self.done = kind, [], False

    def __call__(self, motion, q_before):
        self.calls.append((motion.kind, np.array(q_before)))
        if motion.kind == self.kind and not self.done:
            self.done = True
            return {"passed": False, "tightest": f"fake refusal of a {motion.kind} motion"}
        return {"passed": True, "tightest": "fake: fine"}


@pytest.mark.parametrize("kind", ["free", "lower", "draw", "lift"])
def test_a_refused_motion_leaves_its_piece_over_and_the_tour_goes_on(problem, kind):
    arm, obs, rules, bunches = problem
    park = RIG.park_q("2L")
    plain, _, _ = tour_all(arm, bunches, park, obs, rules)
    fake = _Refuse(kind)
    ms, left, rep = tour_all(arm, bunches, park, obs, rules, verify=fake)
    failed = [x for x in left if x.reason == "failed_check"]
    assert len(failed) == 1 and rep.failed_check == 1
    role = {"free": "move", "lower": "lower", "draw": "drawing", "lift": "lift"}[kind]
    assert f"the {role} (" in failed[0].detail and "fake refusal" in failed[0].detail
    lost = failed[0].piece.line_id
    assert all(m.piece.line_id != lost for m in ms if m.kind == "draw")
    assert {m.piece.line_id for m in plain if m.kind == "draw"} - {lost} == \
        {m.piece.line_id for m in ms if m.kind == "draw"}
    # every motion handed on passed and carries the word; they join up from the park to the park
    assert all(m.checked is not None and m.checked["passed"] for m in ms)
    assert_tour(arm, obs, rules, ms, park, park)
    # the refused group was checked from where the arm stood: the park (it was the first piece)
    first_of_group = next(i for i, c in enumerate(fake.calls) if c[0] == "free")
    assert np.array_equal(fake.calls[first_of_group][1], park)
    # after the refusal, the next group starts again from the park
    assert np.max(np.abs(ms[0].q_start - park)) <= 1e-9


def test_without_verify_nothing_changes(problem):
    arm, obs, rules, bunches = problem
    park = RIG.park_q("2L")
    a, la, _ = tour_all(arm, bunches, park, obs, rules)
    b, lb, rep = tour_all(arm, bunches, park, obs, rules,
                          verify=lambda m, q: {"passed": True, "tightest": ""})
    assert len(a) == len(b) and la == lb and rep.verified == len(b)
    for x, y in zip(a, b):
        assert x.checked is None and y.checked == {"passed": True, "tightest": ""}
        assert np.array_equal(x.traj.q, y.traj.q) and np.array_equal(x.traj.t, y.traj.t)


def test_a_refused_move_home_is_the_end_refusal(problem):
    arm, obs, rules, bunches = problem
    park = RIG.park_q("2L")
    n = len(tour_all(arm, bunches, park, obs, rules)[0])
    calls = []

    def last_refused(m, q):
        calls.append(m)
        return {"passed": len(calls) < n, "tightest": "fake: home refused"}

    ms, left, rep = tour_all(arm, bunches, park, obs, rules, verify=last_refused)
    assert len(ms) == n - 1 and rep.end_refusal.startswith("failed_check: the move to q_end")
    assert not [x for x in left if x.reason == "failed_check"]
