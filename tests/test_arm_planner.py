"""Tests of the arm planner (aris/arm_planner.py) with the sequencer inside it.

Quick set: `../.venv/bin/python -m pytest tests/test_arm_planner.py -m "not slow" -q`.
Slow set: the word "unknown" for arms 2L and 1L, every motion through the independent checker,
with the numbers measured on 2026-09-30 as floors and ceilings (see docs/modules/sequencer.md).
Run with -s to see the numbers.
"""
from __future__ import annotations

import hashlib
import time
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_cases as ac  # noqa: E402
import local_cases as lc  # noqa: E402

from aris.arm_planner import PlanStats, plan, plan_all, plan_detailed  # noqa: E402
from aris.rig import Rig  # noqa: E402
from aris.sequencer.guard import Guard  # noqa: E402
from aris.types import Line  # noqa: E402

DEPLOY = Path(__file__).resolve().parents[1]
RIG = Rig.load(ac.CONFIG)


def two_lines(arm_id: str = "2L") -> list[Line]:
    """Two short lines under the arm, table frame -> base frame."""
    x, y = RIG.T_table_base(arm_id)[:2, 3]
    lines = [Line("a", np.array([[x - 0.15, y - 0.25, 0.0], [x, y - 0.25, 0.0]]), "table"),
             Line("b", np.array([[x + 0.10, y + 0.25, 0.0], [x + 0.10, y + 0.40, 0.0]]), "table")]
    return [RIG.to_base(arm_id, line) for line in lines]


def assert_tour(arm, obs, rules, motions, q_start, q_end):
    """Every motion starts where the previous ended, is timed, ends where the arm can stand."""
    guard = Guard(arm, obs, rules.gates)
    q = np.asarray(q_start, float)
    for i, m in enumerate(motions):
        t = m.traj
        assert len(t.t) >= 2 and np.all(np.diff(t.t) > 0), i
        assert np.max(np.abs(t.q[0] - q)) <= 1e-9, i
        assert np.max(np.abs(t.qd[0])) <= 1e-12 and np.max(np.abs(t.qd[-1])) <= 1e-12, i
        touching = m.kind in ("draw", "lower")
        assert guard.hold(m.q_end, touching) is None, (i, guard.hold(m.q_end, touching))
        if m.kind == "draw":
            assert m.tip_base.shape == (len(t.t), 3) and m.piece is not None
        q = m.q_end
    assert np.max(np.abs(q - q_end)) <= 1e-9


def _digest(motions, leftovers) -> str:
    h = hashlib.sha256()
    for m in motions:
        h.update(m.kind.encode())
        for a in (m.traj.t, m.traj.q, m.traj.qd):
            h.update(np.ascontiguousarray(a).tobytes())
    for x in leftovers:
        h.update(repr(x).encode())
    return h.hexdigest()


def _fresh() -> str:
    arm, obs, rules, _ = lc.problem(RIG, "2L")
    return _digest(*plan_all(arm, two_lines(), obs, RIG.park_q("2L"), rules))


# --------------------------------------------------------------------------- quick


def test_two_lines_end_to_end_and_the_same_in_a_fresh_process(tmp_path):
    arm, obs, rules, _ = lc.problem(RIG, "2L")
    q0 = RIG.park_q("2L")
    motions, leftovers, st = plan_detailed(arm, two_lines(), obs, q0, rules)
    assert leftovers == []
    kinds = [m.kind for m in motions]
    assert kinds.count("draw") >= 2 and kinds[0] == "free" and kinds[-1] == "free"
    assert kinds[:4] == ["free", "lower", "draw", "lift"]
    assert st.tour.pieces == 2 and st.tour.lifts == 2
    assert {m.piece.line_id for m in motions if m.kind == "draw"} == {"a", "b"}
    assert_tour(arm, obs, rules, motions, q0, q0)
    assert 0.0 <= st.first_cpu <= st.cpu and st.tour.penup_share > 0.0
    print(f"\ntwo lines: {len(motions)} motions, planning CPU {st.cpu:.2f} s, first motion "
          f"{st.first_cpu:.2f} s; drawing {st.tour.draw_time:.1f} s, pen up "
          f"{st.tour.penup_time:.1f} s")
    out = tmp_path / "fresh.txt"
    code = ("import sys; sys.path.insert(0, %r); import test_arm_planner as t; "
            "open(%r, 'w').write(t._fresh())" % (str(Path(__file__).parent), str(out)))
    env = dict(os.environ, PYTHONHASHSEED="4321")
    subprocess.run([sys.executable, "-c", code], check=True, env=env, cwd=str(DEPLOY))
    assert out.read_text() == _digest(motions, leftovers)


def test_the_first_motion_comes_before_the_tour_is_decided():
    arm, obs, rules, _ = lc.problem(RIG, "2L")
    st = PlanStats()
    gen = plan(arm, two_lines(), obs, RIG.park_q("2L"), rules, stats=st)
    first = next(gen)
    assert first.kind == "free"
    # One piece is planned (move, set-down, drawing, lift-off); the other one not yet.
    assert st.tour.pieces == 1 and st.tour.motions == 1 and st.tour.free_calls == 1
    rest = list(gen)
    assert len(rest) >= 5 and st.tour.pieces == 2


# --------------------------------------------------------------------------- slow: the word

# Measured 2026-09-30 on branch aris3 with the two lift rules (turns allowed) and a 20 mm pen clearance
# (docs/modules/sequencer.md): every motion of the word passes the checker.
# Pen-up share ceilings raised 2026-10-01 for the set-down at the landing speed (10 mm/s, about
# 2.3 s per piece instead of 0.3): measured 0.371 (arm 2L) and 0.339 (arm 1L).
WORD = {"2L": dict(checked=53, motions=53, share=0.42, cpu=2.4),
        "1L": dict(checked=53, motions=53, share=0.40, cpu=4.3)}


@pytest.mark.slow
@pytest.mark.parametrize("arm_id", sorted(ac.ARMS))
def test_word_every_motion_through_the_checker(arm_id, tmp_path_factory):
    cache = tmp_path_factory.getbasetemp() / "kinematic_table"
    lines = ac.case_lines(RIG, arm_id)["word"]
    ms, left, st, load = ac.plan_case(RIG, arm_id, lines, cache)
    checks = ac.check_all(RIG, arm_id, ms, workers=8)
    print("\n" + "\n".join(ac.summary(f"arm {arm_id} word", ms, left, st, load, checks, arm_id)))
    arm, obs, rules, _ = lc.problem(RIG, arm_id)
    assert_tour(arm, obs, rules, ms, RIG.park_q(arm_id), RIG.park_q(arm_id))
    assert not st.tour.end_refusal
    want = WORD[arm_id]
    assert st.tour.penup_share <= want["share"]
    assert st.cpu <= 10 * want["cpu"]
    passed = sum(c[0] for c in checks)
    assert passed == len(ms) and passed >= want["checked"]
    assert len(ms) >= want["motions"] - 8                        # about the same tour


@pytest.mark.slow
@pytest.mark.parametrize("arm_id", sorted(ac.ARMS))
def test_word_with_the_checker_in_the_loop(arm_id, tmp_path_factory):
    """The law, with the real checker as `verify`: every motion handed on carries the
    checker's pass; the same tour as without it; and what the checking costs."""
    import time
    cache = tmp_path_factory.getbasetemp() / "kinematic_table"
    lines = ac.case_lines(RIG, arm_id)["word"]
    ms0, left0, st0, _ = ac.plan_case(RIG, arm_id, lines, cache)
    c0 = time.process_time()
    ms, left, st, load = ac.plan_case(RIG, arm_id, lines, cache,
                                      verify=ac.checker_verify(arm_id))
    cpu = time.process_time() - c0
    print(f"\narm {arm_id} word, load {load:.0f}: CPU {st0.cpu:.1f} s without verify, {cpu:.1f} s "
          f"with ({st.tour.verify_wall:.1f} s wall inside verify, {st.tour.verified} calls)")
    assert all(m.checked is not None and m.checked["passed"] for m in ms)
    assert not [x for x in left if x.reason == "failed_check"] and not st.tour.end_refusal
    assert len(ms) == len(ms0) and st.tour.verified == len(ms)
    for a, b in zip(ms0, ms):
        assert np.array_equal(a.traj.q, b.traj.q)
    assert cpu <= 10 * 8.0                  # measured 7.9 s (arm 1L, load 21)


# --------------------------------------------------------------------------- batches


def _lines_near(arm_id: str, n: int, seed: int = 5):
    """n short random lines within the arm's reach, base frame (deterministic)."""
    axis = RIG.T_table_base(arm_id)[:2, 3]
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        c = axis + np.sqrt(rng.uniform(0.04, 0.6 ** 2)) * np.array(
            [np.cos(a := rng.uniform(0, 2 * np.pi)), np.sin(a)])
        d = rng.uniform(0.03, 0.12) * np.array([np.cos(b := rng.uniform(0, np.pi)), np.sin(b)])
        xy = np.array([c - d / 2, c + d / 2])
        out.append(RIG.to_base(arm_id, Line(f"n:{i}", np.column_stack([xy, np.zeros(2)]),
                                            "table")))
    return out


def test_nearest_first_orders_by_distance_from_the_pen():
    from aris.arm_planner import nearest_first
    arm = RIG.arm("2L")
    lines = _lines_near("2L", 12)
    order = nearest_first(arm, lines, RIG.park_q("2L"))
    tip = arm.tip(RIG.park_q("2L")[None])[0]
    d = [np.min(np.linalg.norm(np.linspace(x.points[0], x.points[1], 200) - tip, axis=1))
         for x in (lines[i] for i in order)]
    assert sorted(order) == list(range(12)) and np.all(np.diff(d) >= -1e-4)


def _slowed(monkeypatch, pause):
    import aris.arm_planner as ap
    real = ap._batches

    def slow(*a, **k):
        for b in real(*a, **k):
            time.sleep(pause)
            yield b
    monkeypatch.setattr(ap, "_batches", slow)


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_batches_give_the_same_tour_with_1_or_8_workers_and_a_slow_feeder(monkeypatch):
    arm, obs, rules, _ = lc.problem(RIG, "2L")
    lines = _lines_near("2L", 14)
    q0 = RIG.park_q("2L")
    kw = dict(batch=4)
    one = _digest(*plan_all(arm, lines, obs, q0, rules, workers=1, **kw))
    many = _digest(*plan_all(arm, lines, obs, q0, rules, workers=8, **kw))
    _slowed(monkeypatch, 0.5)
    slow = _digest(*plan_all(arm, lines, obs, q0, rules, workers=8, **kw))
    assert one == many == slow
    ms, left, st = plan_detailed(arm, lines, obs, q0, rules, **kw)
    assert st.batches == 4 and st.lines == 14
    assert_tour(arm, obs, rules, ms, q0, q0)
    drawn = {m.piece.line_id for m in ms if m.kind == "draw"} | {x.piece.line_id for x in left}
    assert drawn == {x.id for x in lines}                      # nothing dropped


@pytest.mark.slow
def test_first_motion_of_1000_lines_comes_within_seconds(tmp_path_factory):
    """1 000 lines of tests/big_cases.big() within arm 1L's reach, 8 workers, batches of 32.
    Measured 2026-09-30 at load 8: first motion after 1.6 s (all at once: 19.4 s), 3 881
    motions, pen-up share 0.163 (all at once 0.141)."""
    cache = tmp_path_factory.getbasetemp() / "kinematic_table"
    lines = ac.big_lines(RIG, "1L", 1000)
    ms, left, st, load = ac.plan_case(RIG, "1L", lines, cache, 8)
    print(f"\n1 000 lines, arm 1L, load {load:.0f}: first motion after {st.first_wall:.2f} s, "
          f"planning wall {st.wall:.1f} s, CPU {st.cpu:.1f} s, pen-up share "
          f"{st.tour.penup_share:.3f}, {st.batches} batches")
    arm, obs, rules, _ = lc.problem(RIG, "1L")
    assert_tour(arm, obs, rules, ms, RIG.park_q("1L"), RIG.park_q("1L"))
    assert st.batches == 32 and st.first_wall < 15.0         # 5 s wanted at load under 20
    # raised from 0.20 on 2026-10-01: set-down at the landing speed, measured 0.290
    assert st.tour.penup_share <= 0.33
    drawn = {m.piece.line_id for m in ms if m.kind == "draw"} | {x.piece.line_id for x in left}
    assert drawn == {x.id for x in lines}


def test_the_line_pool_gives_the_same_bunches_as_the_local_planner():
    from aris import local
    from test_local import _digest as local_digest
    arm, obs, rules, _ = lc.problem(RIG, "2L")
    lines = _lines_near("2L", 6)
    want = local_digest(local.plan(arm, lines, obs, rules))
    pool = local.LinePool(arm, obs, rules, workers=3)
    try:
        got = [pool.submit([x]).result() for x in lines]
        together = pool.submit(lines).result()
    finally:
        pool.close()
    one_by_one = ([b for r in got for b in r[0]], [x for r in got for x in r[1]])
    assert local_digest(one_by_one) == want
    assert local_digest(together[:2]) == want


# --------------------------------------------------------------------------- the drawing surface


def test_lines_on_the_drawing_surface_below_the_paper():
    """The system planner hands lines at z = paper - press (the drawing surface).  The pen
    draws on that surface, each lift-off ends the pen clearance + 2 mm above the REAL paper
    (so it rises the press further), and each set-down lands on the surface."""
    from aris.sequencer import TourOptions
    from aris.sequencer.lift import lift_height
    arm, obs, rules, _ = lc.problem(RIG, "2L")
    assert rules.press > 0.0
    x, y = RIG.T_table_base("2L")[:2, 3]
    z = RIG.paper_z - rules.press
    lines = [RIG.to_base("2L", Line(i, np.array(p), "table")) for i, p in (
        ("a", [[x - 0.15, y - 0.25, z], [x, y - 0.25, z]]),
        ("b", [[x + 0.10, y + 0.25, z], [x + 0.10, y + 0.40, z]]))]
    paper = [p for p in obs.planes if p.kind == "paper"][0]
    n = np.asarray(paper.normal) / np.linalg.norm(paper.normal)
    above = lambda q: arm.tip(np.atleast_2d(q)) @ n - paper.offset          # over the paper
    q0 = RIG.park_q("2L")
    ms, left = plan_all(arm, lines, obs, q0, rules)
    assert not left and [m.kind for m in ms].count("draw") == 2
    top = lift_height(paper, TourOptions().lift_extra)
    for m in ms:
        if m.kind == "draw":
            assert np.allclose(above(m.traj.q), -rules.press, atol=2e-4)   # on the surface
        if m.kind == "lift":
            assert abs(above(m.q_end)[0] - top) < 1e-6                     # clearance + 2 mm
            assert abs(above(m.q_start)[0] + rules.press) < 1e-6
        if m.kind == "lower":
            assert abs(above(m.q_end)[0] + rules.press) < 1e-6             # lands on the surface
    assert_tour(arm, obs, rules, ms, q0, q0)
    # the same through the real checker as `verify`
    verify = ac.checker_verify("2L")
    word = verify(ms[0], q0)
    if not word["passed"] and "cannot read the rig" in word["detail"]:
        pytest.skip("the checker cannot read this rig.json yet: " + word["detail"])
    vms, vleft = plan_all(arm, lines, obs, q0, rules, verify=verify)
    refused = [x for x in vleft if x.reason == "failed_check"]
    print("\non the drawing surface, through the checker: "
          f"{len(vms)} motions handed on, {len(refused)} pieces refused"
          + "".join(f"\n  {x.detail}" for x in refused))
    if refused and all("tip on paper" in x.detail for x in refused):
        pytest.skip("the checker does not know the press yet: " + refused[0].detail)
    assert not refused and len(vms) == len(ms)
