"""Tests of the system planner (aris/system).  Quick set under a minute; the slow test runs the
word across the middle arms end to end through the independent checker.  The acceptance cases
and the numbers of docs/modules/system.md are in tests/system_cases.py."""
from __future__ import annotations

import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aris.rig import Rig  # noqa: E402
from aris.system import NoDropViolation, account, phases, plan_all, plan_detailed  # noqa: E402
from aris.system import maps as mp  # noqa: E402
from aris.system.allocate import allocate, cover  # noqa: E402
from aris.system.phases import (cannot_touch, fill_groups, follower_phase,  # noqa: E402
                                 horizontal_reach, is_fill)
from aris.system.settings import Settings  # noqa: E402
from aris.system.stretch import of_line  # noqa: E402
from aris.types import Leftover, Line, Piece, Refusal  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / "config"
COARSE = Settings(grid_step=0.05)          # quick tests: a 5 cm grid


def _line(lid, *xy) -> Line:
    xy = np.asarray(xy, float)
    return Line(lid, np.column_stack([xy, np.zeros(len(xy))]), "table")


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


@pytest.fixture(scope="module")
def rules(rig):
    return rig.rules()


@pytest.fixture(scope="module")
def all_maps(rig, rules):
    return mp.load_or_build(rig, phases(rig), rules.gates, COARSE, None, workers=6)


# --------------------------------------------------------------------------- maps


def test_maps_of_one_phase_build_and_cache(rig, rules, tmp_path):
    ph = [rig.phase(1)]
    built = mp.load_or_build(rig, ph, rules.gates, COARSE, tmp_path)
    assert len(list(tmp_path.glob("system_map_*.npz"))) == 3
    again = mp.load_or_build(rig, ph, rules.gates, COARSE, tmp_path)
    for k, m in built.items():
        assert m.cpu > 0.0 and again[k].cpu == 0.0            # the second came from the files
        assert np.array_equal(m.state, again[k].state)
        assert 0.15 < m.share < 0.35
    # a different grid is a different map, in a different file
    mp.load_or_build(rig, ph, rules.gates, Settings(grid_step=0.1), tmp_path)
    assert len(list(tmp_path.glob("system_map_*.npz"))) == 6
    m13 = built[("phase 1", 13)]
    under = np.array([[-0.40, -1.0, 0.0]])
    across = np.array([[0.30, -0.45, 0.0]])                   # 71's side of the wall 13-71
    assert m13.contains(under)[0] and not m13.contains(across)[0]
    assert built[("phase 1", 71)].contains(across)[0]


def test_maps_cover_the_canvas(rig, all_maps):
    cov = mp.coverage(all_maps, phases(rig))
    assert cov["phase 1"]["union"] > 0.65 and cov["phase 2"]["union"] > 0.65
    assert cov["all"] > 0.97


# --------------------------------------------------------------------------- allocation


def _cands(rig, all_maps, k0=0):
    ph = phases(rig)
    return [(j, a, all_maps[(ph[j].name, a)], not is_fill(ph[j]))
            for j in range(k0, len(ph)) for a in ph[j].active]


def test_allocation_of_hand_made_lines(rig, rules, all_maps):
    inside = _line("inside", (-0.45, -1.0), (-0.30, -0.95))         # arm 13's region, phase 1
    wall = _line("wall", (-0.55, -0.55), (-0.55, -0.15))            # crosses the wall 13-71
    out = _line("out", (-0.88, -0.62), (-0.88, -0.58))              # beyond every arm's reach
    across = _line("across", (-0.75, -1.2), (0.75, -1.2))           # longer than any one reach
    pool = [of_line(x) for x in (inside, wall, out, across)]
    got, left, cuts = allocate(pool, _cands(rig, all_maps), rules, COARSE)
    assert cuts == 1
    by = {}
    for s in got:
        by.setdefault(s.line_id, []).append(s)
    assert [s.target for s in by["inside"]] == [(0, 13)]
    assert [s.target for s in by["wall"]] == [(1, 31)]              # whole in phase 2
    assert "out" not in by and [x.reason for x in left] == ["unreachable"]
    assert left[0].piece == Piece("out", 0.0, pytest.approx(0.04))
    parts = sorted(by["across"], key=lambda s: s.s0)
    assert len(parts) == 2 and {s.target[1] for s in parts} == {13, 17}
    assert parts[0].s0 == 0.0 and parts[-1].s1 == pytest.approx(1.5)
    assert parts[0].s1 - parts[1].s0 == pytest.approx(rules.min_piece)   # the join, twice


def test_cover_cuts_longest_first_with_overlap_and_gaps():
    u = np.arange(11) * 0.01
    inside = np.array([[1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                       [0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1]], bool)
    got = cover(inside, u, 0.005)
    assert got[0] == (0.0, pytest.approx(0.03), 0)
    assert got[1] == (pytest.approx(0.025), pytest.approx(0.06), 1)
    assert got[2] == (pytest.approx(0.06), pytest.approx(0.09), None)
    assert got[3] == (pytest.approx(0.09), pytest.approx(0.10), 2)


def test_leader_phases_come_first(rig, rules, all_maps):
    # arm 31 alone could draw it whole, but the leaders hold all of it in two long runs
    both = _line("both", (-0.58, 0.40), (-0.24, 0.69))
    got, left, _ = allocate([of_line(both)], _cands(rig, all_maps), rules, COARSE)
    by = {}
    for s in got:
        by.setdefault(s.line_id, []).append(s.target)
    assert sorted(by["both"]) == [(0, 2), (1, 31)] and not left


def _toy_map(name, arm, x_lo, x_hi, hole=None):
    """A map drawable for x_lo <= x <= x_hi (grid 2 cm), except the grid column at `hole`."""
    x = y = np.arange(16) * 0.02
    state = np.zeros((16, 16), np.int8)
    state[(x >= x_lo - 1e-9) & (x <= x_hi + 1e-9), :] = mp.DRAWABLE
    if hole is not None:
        state[np.isclose(x, hole), :] = mp.OUT
    return mp.Map(name, arm, x, y, state)


def test_the_laws_on_toy_maps(rules):
    cfg = Settings()
    line = of_line(_line("l", (0.0, 0.1), (0.2, 0.1)))
    fill = (2, 31, _toy_map("fill 31", 31, 0.0, 0.3), False)
    # law 3: the leaders hold all of it, in two stretches: cut, joined, the fill unused
    two = [(0, 13, _toy_map("phase 1", 13, 0.0, 0.12), True),
           (1, 17, _toy_map("phase 2", 17, 0.10, 0.3), True), fill]
    got, left, joins = allocate([line], two, rules, cfg)
    assert [s.target for s in got] == [(0, 13), (1, 17)]
    assert got[0].s1 - got[1].s0 == pytest.approx(rules.min_piece)       # one join
    assert got[0].s1 == pytest.approx(0.13, abs=0.005)
    assert joins == 1 and not left
    # law 3: the leaders hold under 80 % of it: the fill phase takes it whole
    few = [(0, 13, _toy_map("phase 1", 13, 0.0, 0.08), True), fill]
    got, left, _ = allocate([line], few, rules, cfg)
    assert [(s.target, s.s0, s.s1) for s in got] == [((2, 31), 0.0, 0.2)]
    # laws 2 and 3: a hole in the leader's map goes to the fill, with a join at both ends
    holed = [(0, 13, _toy_map("phase 1", 13, 0.0, 0.3, hole=0.10), True), fill]
    got, left, joins = allocate([line], holed, rules, cfg)
    assert [s.target for s in got] == [(0, 13), (2, 31), (0, 13)] and joins == 2 and not left
    assert got[1].s0 < got[0].s1 and got[2].s0 < got[1].s1
    # law 5: nothing holds it
    got, left, _ = allocate([line], [(0, 13, _toy_map("phase 1", 13, 0.25, 0.3), True)], rules,
                            cfg)
    assert not got and [x.reason for x in left] == ["unreachable"]


def test_parts_for_the_same_arm_that_meet_are_one():
    from aris.system.allocate import _merge
    assert _merge([(0.0, 0.1, 0), (0.09, 0.2, 0), (0.19, 0.3, 1)]) == [(0.0, 0.2, 0),
                                                                        (0.19, 0.3, 1)]


def test_a_follower_sees_its_leaders_footprint(rig, rules):
    from aris.kernel.geometry import field_lookup
    from aris.system import followers
    ph = rig.phase(1)
    assert followers.pairs(rig, ph) == [(13, 17), (71, 31), (2, 97)]
    t = time.process_time()
    f, field, obs, m = followers.setup((rig, ph, 13, 17, [], rules.gates, COARSE))
    print(f"follower set-up (leader standing, 5 cm map): CPU {time.process_time() - t:.1f} s")
    assert f == 17 and obs.fields == (field,) and field.cell == followers.CELL
    # the leader's body at its park, in the follower's frame, is inside the footprint
    body = rig.arm(13).body(rig.park_q(13)[None])
    p13 = rig.to_table(13, 0.5 * (body.p0[0] + body.p1[0]))
    T = rig.T_base_table(17)
    p17 = p13 @ T[:3, :3].T + T[:3, 3]
    d = field_lookup(p17, np.asarray(field.origin_base), field.cell,
                     np.array(field.dist.shape), np.asarray(field.dist))
    assert np.all(d < 0.0)
    # standing, the leader takes only a little of the follower's area: under its own park
    alone = mp.build(rig, ph, 17, rules.gates, COARSE,
                     obstacles=rig.obstacles(17, (13,), follower_phase(rig, ph, 17).walls))
    assert 0.9 * alone.share <= m.share <= alone.share


def test_fill_groups_cannot_touch(rig):
    # the two ends of a column are 2.42 m apart; each arm's body stays within 1.15 m of its axis
    assert fill_groups(rig) == [(13, 2), (17, 97), (31,), (71,)]
    gap = 2.4204267 - 2 * horizontal_reach(rig, 13)
    assert gap >= rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]
    assert cannot_touch(rig, 13, 2) and cannot_touch(rig, 17, 97)
    assert not cannot_touch(rig, 13, 71) and not cannot_touch(rig, 31, 71)
    # the bound holds on sampled configurations (it is a bound, so the sample stays inside)
    arm = rig.arm(13)
    Q = np.random.default_rng(0).uniform(arm.limits.q_min, arm.limits.q_max, (5000, 7))
    b = arm.body(Q)
    far = np.maximum(np.hypot(b.p0[..., 0], b.p0[..., 1]), np.hypot(b.p1[..., 0], b.p1[..., 1]))
    assert np.max(far + b.radius) <= horizontal_reach(rig, 13)


# --------------------------------------------------------------------------- the account


def test_account_raises_on_a_drop_or_a_double():
    lines = [_line("a", (0, 0), (0.1, 0))]
    left = [Leftover(Piece("a", 0.0, 0.04), "blocked"), Leftover(Piece("a", 0.04, 0.1), "unreachable")]
    acc = account(lines, [], left, 0.01)
    assert acc.drawn == 0.0 and acc.left == pytest.approx(0.1)
    with pytest.raises(NoDropViolation, match="not covered"):
        account(lines, [], left[:1], 0.01)
    with pytest.raises(NoDropViolation, match="overlap"):
        account(lines, [], left + [Leftover(Piece("a", 0.02, 0.08), "blocked")], 0.01)
    with pytest.raises(NoDropViolation, match="not in the drawing"):
        account(lines, [], left + [Leftover(Piece("b", 0.0, 0.01), "blocked")], 0.01)


# --------------------------------------------------------------------------- end to end


def _small():
    return [_line("under13", (-0.45, -1.0), (-0.30, -0.95)),
            _line("under71", (0.30, 0.25), (0.45, 0.20))]


def _joined(rig, tagged):
    by = {}
    for ph, a, m in tagged:
        by.setdefault(a, []).append((ph, m))
    for a, ms in by.items():
        q = rig.park_q(a)
        for ph, m in ms:
            assert np.max(np.abs(m.q_start - q)) < 1e-9, f"arm {a}: a motion does not join"
            q = m.q_end
        assert np.max(np.abs(q - rig.park_q(a))) < 1e-9, f"arm {a} does not end at its park"
    return by


def test_small_drawing_end_to_end_on_two_arms(rig, rules):
    lines = _small()
    t = time.process_time()
    tagged, left, rep = plan_detailed(rig, lines, rules, settings=COARSE, workers=2)
    print(f"small drawing: CPU {time.process_time() - t:.1f} s (this process), "
          f"{rep.cpu:.1f} s in all, first motion after {rep.first_wall:.1f} s")
    by = _joined(rig, tagged)
    assert set(by) == {13, 71}
    assert {ph for ph, _, _ in tagged} == {"phase 1"}
    drawn = {m.piece.line_id: (a, m.piece) for _, a, m in tagged if m.kind == "draw"}
    assert drawn["under13"][0] == 13 and drawn["under71"][0] == 71
    assert drawn["under13"][1].s1 == pytest.approx(np.hypot(0.15, 0.05))
    acc = account(lines, tagged, left, rules.min_piece)
    assert acc.drawn == pytest.approx(2 * np.hypot(0.15, 0.05))
    assert acc.left_by_reason == {}
    assert [p.name for p in rep.phases] == ["phase 1"]
    assert all(r.first_phase_wall >= 0 for r in rep.phases[0].arms.values())
    # the same plan from one process
    tagged1, left1 = plan_all(rig, lines, rules, settings=COARSE, workers=1)
    key = lambda xs: sorted((a, m.kind, round(float(m.traj.t[-1]), 9),
                             m.piece.line_id if m.piece else "") for _, a, m in xs)
    assert key(tagged1) == key(tagged) and left1 == left


def test_a_drawing_outside_the_drawing_area_is_refused(rig, rules):
    lines = _small()[:1] + [_line("wide", (0.0, 0.0), (0.95, 0.0))]
    tagged, out, rep = plan_detailed(rig, lines, rules, settings=COARSE)
    assert tagged == [] and isinstance(out, Refusal) and out.reason == "outside_drawing_area"
    assert "line wide" in out.detail and "(0.9500, 0.0000)" in out.detail
    assert 1.0 < rep.drawing_area[0] < 1.8 and 3.0 < rep.drawing_area[1] < 3.63


def test_the_drawing_area_comes_from_the_rig_and_must_match_the_maps(rig, rules):
    lines = _small()[:1]
    stale = replace(rig, drawing_area_m=rig.drawing_area_m - 0.2)
    _, out, _ = plan_detailed(stale, lines, rules, settings=COARSE)
    assert isinstance(out, Refusal) and out.reason == "stale_drawing_area"
    _, out, _ = plan_detailed(replace(rig, drawing_area_m=None), lines, rules, settings=COARSE)
    assert isinstance(out, Refusal) and out.reason == "no_drawing_area"


def test_an_arm_away_from_its_park_must_move_first(rig, rules):
    q = rig.park_q(17) + 0.1
    with pytest.raises(ValueError, match="first phase"):
        plan_all(rig, _small()[:1], rules, arm_configs={17: q}, settings=COARSE)


# --------------------------------------------------------------------------- slow


@pytest.mark.slow
def test_word_across_the_middle_arms_through_the_checker(rig, tmp_path_factory):
    import system_cases as sc
    cache = tmp_path_factory.mktemp("system_cache")
    t = time.process_time()
    res = sc.run_case(rig, "word", sc.word(), cache, workers=4, check_workers=8)
    # the drawing area in rig.json is what the maps at rig.json's gates give
    assert np.allclose(res["rep"].drawing_area, sc.AREA, atol=1e-9)
    cpu = time.process_time() - t
    print("\n".join(sc.summary(res)), f"\n  (this process CPU {cpu:.1f} s)")
    n = sc.numbers(res)
    assert n["drawn"] / n["length"] >= 0.99                 # measured 1.000
    assert n["passed"] == n["checked"] > 40                 # measured 53 of 53
    assert n["phase_ends_passed"] == n["phase_ends"] >= 1
    assert res["rep"].drawing_time < 200.0                  # measured 97 s
    plan_cpu = res["rep"].cpu - res["rep"].map_cpu
    assert plan_cpu < 100.0                                 # measured 8 to 16 s (maps apart)
    # the word goes to arm 71 in phase 1; nothing is left for the fill phases
    assert {ph for ph, _, _ in res["tagged"]} == {"phase 1"}
