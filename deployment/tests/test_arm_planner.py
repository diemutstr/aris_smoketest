"""Tests of the arm planner (aris/arm_planner.py) with the sequencer inside it.

Quick set: `../.venv/bin/python -m pytest tests/test_arm_planner.py -m "not slow" -q`.
Slow set: the word "unknown" for arms 31 and 13, every motion through the independent checker,
with the numbers measured on 2026-09-30 as floors and ceilings (see docs/modules/sequencer.md).
Run with -s to see the numbers.
"""
from __future__ import annotations

import hashlib
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


def two_lines(arm_id: int = 31) -> list[Line]:
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
        touching = m.kind == "draw" or (i + 1 < len(motions) and motions[i + 1].kind == "draw")
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
    arm, obs, rules, _ = lc.problem(RIG, 31)
    return _digest(*plan_all(arm, two_lines(), obs, RIG.park_q(31), rules))


# --------------------------------------------------------------------------- quick


def test_two_lines_end_to_end_and_the_same_in_a_fresh_process(tmp_path):
    arm, obs, rules, _ = lc.problem(RIG, 31)
    q0 = RIG.park_q(31)
    motions, leftovers, st = plan_detailed(arm, two_lines(), obs, q0, rules)
    assert leftovers == []
    kinds = [m.kind for m in motions]
    assert kinds.count("draw") >= 2 and kinds[0] == "free" and kinds[-1] == "free"
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
    arm, obs, rules, _ = lc.problem(RIG, 31)
    st = PlanStats()
    gen = plan(arm, two_lines(), obs, RIG.park_q(31), rules, stats=st)
    first = next(gen)
    assert first.kind == "free"
    # One piece is planned (move, set-down, drawing, lift-off); the other one not yet.
    assert st.tour.pieces == 1 and st.tour.motions == 1 and st.tour.free_calls == 1
    rest = list(gen)
    assert len(rest) >= 5 and st.tour.pieces == 2


# --------------------------------------------------------------------------- slow: the word

# Measured 2026-09-30 at machine load 6-11 (docs/modules/sequencer.md).  `checked`: motions the
# checker passes outright.  It was 0 of 58 and 0 of 52: every motion failed only on the two known
# disagreements between the checker and the planners (link 1 against its own struts; the pen
# on the paper at the drawing end of a set-down or lift-off; see `arm_cases.known_disagreement`).
# Raise the floors when the checker follows commit f371483 and the set-down question is settled.
WORD = {31: dict(checked=0, motions=58, share=0.30, cpu=2.5),
        13: dict(checked=0, motions=52, share=0.25, cpu=2.1)}


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
    explained = sum(c[0] or ac.known_disagreement(arm_id, r, c) for r, c in
                    zip(ac.roles(ms), checks))
    assert explained == len(ms)
    assert passed >= want["checked"]
    assert len(ms) >= want["motions"] - 8                        # about the same tour
