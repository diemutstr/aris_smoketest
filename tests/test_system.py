"""Tests of the system planner (aris/system).  Quick set under a minute; the slow test runs the
word across the middle arms end to end through the independent checker.  The acceptance cases
and the numbers of docs/modules/system.md are in tests/system_cases.py."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

from aris.rig import Rig  # noqa: E402
from aris.system import NoDropViolation, account, phases, plan_all, plan_detailed  # noqa: E402
from aris.system import maps as mp  # noqa: E402
from aris.system.allocate import allocate, cover  # noqa: E402
from aris.system.phases import cannot_touch, fill_groups, horizontal_reach, is_fill  # noqa: E402
from aris.system.settings import Settings  # noqa: E402
from aris.system.stretch import of_line  # noqa: E402
from aris.types import Leftover, Line, Piece  # noqa: E402

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
    m13 = built[("phase 1", "1R")]
    under = np.array([[-0.40, -1.0, 0.0]])
    across = np.array([[0.30, -0.45, 0.0]])                   # 2L's side of the wall 1R-2L
    assert m13.contains(under)[0] and not m13.contains(across)[0]
    assert built[("phase 1", "2L")].contains(across)[0]


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
    inside = _line("inside", (-0.45, -1.0), (-0.30, -0.95))         # arm 1R's region, phase 1
    wall = _line("wall", (-0.55, -0.55), (-0.55, -0.15))            # crosses the wall 1R-2L
    out = _line("out", (-0.88, -0.62), (-0.88, -0.58))              # beyond every arm's reach
    across = _line("across", (-0.75, -1.2), (0.75, -1.2))           # longer than any one reach
    pool = [of_line(x) for x in (inside, wall, out, across)]
    got, left, cuts = allocate(pool, _cands(rig, all_maps), rules, COARSE)
    assert cuts == 1
    by = {}
    for s in got:
        by.setdefault(s.line_id, []).append(s)
    assert [s.target for s in by["inside"]] == [(0, "1R")]
    assert [s.target for s in by["wall"]] == [(1, "2R")]              # whole in phase 2
    assert "out" not in by and [x.reason for x in left] == ["unreachable"]
    assert left[0].piece == Piece("out", 0.0, pytest.approx(0.04))
    parts = sorted(by["across"], key=lambda s: s.s0)
    assert len(parts) == 2 and {s.target[1] for s in parts} == {"1R", "1L"}
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
    # arm 2R alone could draw it whole, but the leaders hold all of it in two long runs
    both = _line("both", (-0.58, 0.40), (-0.24, 0.69))
    got, left, _ = allocate([of_line(both)], _cands(rig, all_maps), rules, COARSE)
    by = {}
    for s in got:
        by.setdefault(s.line_id, []).append(s.target)
    assert sorted(by["both"]) == [(0, "3R"), (1, "2R")] and not left


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
    fill = (2, "2R", _toy_map("fill 2R", "2R", 0.0, 0.3), False)
    # law 3: the leaders hold all of it, in two stretches: cut, joined, the fill unused
    two = [(0, "1R", _toy_map("phase 1", "1R", 0.0, 0.12), True),
           (1, "1L", _toy_map("phase 2", "1L", 0.10, 0.3), True), fill]
    got, left, joins = allocate([line], two, rules, cfg)
    assert [s.target for s in got] == [(0, "1R"), (1, "1L")]
    assert got[0].s1 - got[1].s0 == pytest.approx(rules.min_piece)       # one join
    assert got[0].s1 == pytest.approx(0.13, abs=0.005)
    assert joins == 1 and not left
    # law 3: the leaders hold under 80 % of it: the fill phase takes it whole
    few = [(0, "1R", _toy_map("phase 1", "1R", 0.0, 0.08), True), fill]
    got, left, _ = allocate([line], few, rules, cfg)
    assert [(s.target, s.s0, s.s1) for s in got] == [((2, "2R"), 0.0, 0.2)]
    # laws 2 and 3: a hole in the leader's map goes to the fill, with a join at both ends
    holed = [(0, "1R", _toy_map("phase 1", "1R", 0.0, 0.3, hole=0.10), True), fill]
    got, left, joins = allocate([line], holed, rules, cfg)
    assert [s.target for s in got] == [(0, "1R"), (2, "2R"), (0, "1R")] and joins == 2 and not left
    assert got[1].s0 < got[0].s1 and got[2].s0 < got[1].s1
    # law 5: nothing holds it
    got, left, _ = allocate([line], [(0, "1R", _toy_map("phase 1", "1R", 0.25, 0.3), True)], rules,
                            cfg)
    assert not got and [x.reason for x in left] == ["unreachable"]


def test_the_area_is_centred_where_the_rig_says():
    from aris.system import area
    x = np.arange(-10, 11) * 0.02
    y = np.arange(-20, 21) * 0.02
    state = np.full((21, 41), mp.DRAWABLE, np.int8)
    state[:, 30:] = mp.OUT                             # nothing drawable above y = 0.18
    maps = {("phase 1", "1R"): mp.Map("phase 1", "1R", x, y, state)}
    # about the table centre the top edge limits it; about y = -0.1 it is the bottom edge
    assert np.allclose(area.admissible(maps, margin=0.0), [0.4, 0.36])
    assert np.allclose(area.admissible(maps, margin=0.0, centre=(0.0, -0.1)), [0.4, 0.56])
    line = _line("l", (0.0, -0.35), (0.0, 0.15))
    assert area.first_outside([line], (0.4, 0.56), centre=(0.0, -0.1)) is None
    assert area.first_outside([line], (0.4, 0.56))[0] == "l"


def test_parts_for_the_same_arm_that_meet_are_one():
    from aris.system.allocate import _merge
    assert _merge([(0.0, 0.1, 0), (0.09, 0.2, 0), (0.19, 0.3, 1)]) == [(0.0, 0.2, 0),
                                                                        (0.19, 0.3, 1)]


def test_fill_groups_cannot_touch(rig):
    # the two ends of a column are 2.42 m apart; each arm's body stays within 1.15 m of its axis
    assert fill_groups(rig) == [("1L", "3L"), ("1R", "3R"), ("2L",), ("2R",)]
    gap = 2.4204267 - 2 * horizontal_reach(rig, "1R")
    assert gap >= rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]
    assert cannot_touch(rig, "1R", "3R") and cannot_touch(rig, "1L", "3L")
    assert not cannot_touch(rig, "1R", "2L") and not cannot_touch(rig, "2R", "2L")
    # the bound holds on sampled configurations (it is a bound, so the sample stays inside)
    arm = rig.arm("1R")
    Q = np.random.default_rng(0).uniform(arm.limits.q_min, arm.limits.q_max, (5000, 7))
    b = arm.body(Q)
    far = np.maximum(np.hypot(b.p0[..., 0], b.p0[..., 1]), np.hypot(b.p1[..., 0], b.p1[..., 1]))
    assert np.max(far + b.radius) <= horizontal_reach(rig, "1R")


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


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_small_drawing_end_to_end_on_two_arms(rig, rules):
    lines = _small()
    t = time.process_time()
    tagged, left, rep = plan_detailed(rig, lines, rules, settings=COARSE, workers=2)
    print(f"small drawing: CPU {time.process_time() - t:.1f} s (this process), "
          f"{rep.cpu:.1f} s in all, first motion after {rep.first_wall:.1f} s")
    by = _joined(rig, tagged)
    assert set(by) == {"1R", "2L"}
    assert {ph for ph, _, _ in tagged} == {"phase 1"}
    drawn = {m.piece.line_id: (a, m.piece) for _, a, m in tagged if m.kind == "draw"}
    assert drawn["under13"][0] == "1R" and drawn["under71"][0] == "2L"
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


def test_ink_no_arm_reaches_is_left_over_not_refused(rig, rules):
    lines = [_line("out", (-0.88, -0.62), (-0.88, -0.58))]          # beyond every arm's reach
    tagged, left, rep = plan_detailed(rig, lines, rules, settings=COARSE, workers=6)
    assert tagged == [] and [(x.piece.line_id, x.reason) for x in left] == [("out", "unreachable")]
    assert 1.0 < rep.drawing_area[0] < 1.8 and 3.0 < rep.drawing_area[1] < 3.63


def _mounted(tmp_path, slots):
    """config/rig.json with only `slots` mounted (tools/mounted_rig.py's derivation)."""
    import importlib.util
    import json
    spec = importlib.util.spec_from_file_location("mounted_rig", Path(__file__).resolve()
                                                  .parents[1] / "tools" / "mounted_rig.py")
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    cfg = tool.derive(json.loads((CONFIG / "rig.json").read_text()), list(slots), None)
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    return Rig.load(tmp_path)


def test_a_one_arm_rig_plans(tmp_path, rules):
    one = _mounted(tmp_path / "one", ["1L"]) if (tmp_path / "one").mkdir() is None else None
    assert one.arm_ids == ("1L",) and not phases(one)[0].active      # phase 1: nobody moves
    lines = [_line("far", (0.5, 1.5), (0.6, 1.5))]                   # beyond 1L's reach
    tagged, left, rep = plan_detailed(one, lines, rules, settings=COARSE)
    assert tagged == [] and [x.reason for x in left] == ["unreachable"]
    assert set(rep.coverage) == {"phase 2", "fill 1L", "all"} and 0.1 < rep.coverage["all"] < 0.4
    # about the table centre (out of 1L's reach) no rectangle fits; about 1L's axis one does
    assert np.all(rep.drawing_area == 0.0)
    from aris.system import area, maps as mp
    m = mp.load_or_build(one, phases(one), rules.gates, COARSE, press=rules.press)
    assert np.all(area.admissible(m, centre=one.T_table_base("1L")[:2, 3]) > 0.2)
    # no arm mounted: the rig itself refuses to load
    (tmp_path / "none").mkdir()
    with pytest.raises(ValueError, match="no arm is mounted"):
        _mounted(tmp_path / "none", [])


FAKE_CLEARANCE = {"1R": 0.023, "1L": 0.027, "2R": 0.041, "2L": 0.081, "3R": 0.012, "3L": 0.107}


class RefuseLine:
    """A fake independent checker: refuses every motion that draws `line_id` for `arm_id`,
    passes everything else.  Picklable (a module-level class)."""

    def __init__(self, arm_id, line_id):
        self.arm_id, self.line_id = arm_id, line_id
        self.seen = []

    def __call__(self, arm_id, phase, motion, q_before):
        self.seen.append((arm_id, phase.name))
        bad = (arm_id == self.arm_id and motion.piece is not None
               and motion.piece.line_id.split("#")[0] == self.line_id)
        return dict(passed=not bad, tightest="fake", min_clearance=FAKE_CLEARANCE[arm_id])


@pytest.mark.slow  # 5 to 17 s: over the quick set's budget (orchestrator, 2026-10-01)
def test_a_refused_line_flows_on_and_is_left_over_as_failed_check(rig, rules):
    import pickle
    from functools import partial
    verify = RefuseLine("1R", "under13")
    pickle.loads(pickle.dumps(partial(verify, "1R", rig.phase(1))))    # crosses processes
    lines = _small()
    tagged, left, rep = plan_detailed(rig, lines, rules, settings=COARSE, workers=4, verify=verify)
    assert tagged and all(m.checked is not None and m.checked["passed"] for _, _, m in tagged)
    by = {(m.piece.line_id, a) for _, a, m in tagged if m.kind == "draw"}
    # arm 1R never draws it; arm 1L (phase 2, then filling with 3L) draws what its maps hold
    assert by == {("under71", "2L"), ("under13", "1L")}
    # the start of the line only arm 1R holds: offered to it again in fill 1R+3R, refused again
    assert [(x.piece.line_id, x.reason) for x in left] == [("under13", "failed_check")]
    assert left[0].piece.s0 == 0.0 and 0.0 < left[0].piece.s1 < 0.05
    assert left[0].detail.startswith("fill 1R+3R, arm 1R")
    p1 = next(p for p in rep.phases if p.name == "phase 1")
    assert p1.idle["1R"].handed_back == {"failed_check": pytest.approx(0.1581, abs=1e-3)}
    assert rep.tightest == pytest.approx(0.027) and "arm 1L" in rep.tightest_at
    account(lines, tagged, left, rules.min_piece)
    assert sum(r.checked for p in rep.phases for r in p.arms.values()) == len(tagged)


def test_an_arm_away_from_its_park_must_move_first(rig, rules):
    q = rig.park_q("1L") + 0.1
    with pytest.raises(ValueError, match="first phase"):
        plan_all(rig, _small()[:1], rules, arm_configs={"1L": q}, settings=COARSE)


# --------------------------------------------------------------------------- slow


class Bumps:
    """A measured paper that is not flat: +-2 mm, 15 cm waves along x and y."""

    def __init__(self, paper_z):
        self.paper_z = paper_z

    def z(self, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        return self.paper_z + 0.002 * np.sin(2 * np.pi * x / 0.15) * np.cos(2 * np.pi * y / 0.15)


@pytest.mark.slow  # 5 to 10 s
def test_the_pen_follows_a_measured_paper(rig, rules):
    surface = Bumps(rig.paper_z)
    lines = [_line("bumpy", (-0.50, -1.05), (-0.25, -0.90))]
    tagged, left, rep = plan_detailed(rig, lines, rules, settings=COARSE, surface=surface)
    acc = account(lines, tagged, left, rules.min_piece)
    assert acc.drawn == pytest.approx(acc.length, abs=1e-6)       # in the drawing's arc length
    draws = [(a, m) for _, a, m in tagged if m.kind == "draw"]
    tips = np.concatenate([rig.to_table(a, m.tip_base) for a, m in draws])
    want = surface.z(tips[:, 0], tips[:, 1]) - rules.press
    print(f"tip height off the bumpy surface: at most {np.abs(tips[:, 2] - want).max() * 1e3:.3f} "
          f"mm; surface spans {np.ptp(want) * 1e3:.2f} mm along the line")
    assert np.ptp(want) > 0.002                                     # the line crosses the bumps
    assert np.abs(tips[:, 2] - want).max() < 0.2e-3                 # and the tip follows them


@pytest.mark.slow  # 10 to 20 s
def test_each_arm_draws_with_its_own_pen(rig, rules):
    from dataclasses import replace
    from aris.sequencer.drag import pulled_shares
    from aris.types import DrawPlan
    pens = {"1R": replace(rules, press=0.0016),
            "2L": replace(rules, press=0.0025, drag_only=True)}
    lines = _small()
    tagged, left, rep = plan_detailed(rig, lines, rules, settings=COARSE, workers=2,
                                      rules_by_slot=pens)
    account(lines, tagged, left, rules.min_piece)
    draws = [(a, m) for _, a, m in tagged if m.kind == "draw"]
    assert {a for a, _ in draws} == {"1R", "2L"}
    for a, m in draws:
        tip = rig.to_table(a, m.tip_base)
        assert np.allclose(tip[:, 2], rig.paper_z - pens[a].press, atol=1e-5), a
        if pens[a].drag_only:                     # the drag-only pen is pulled, never pushed
            plan = DrawPlan(m.piece, m.traj.q, np.zeros(len(m.traj.q)), m.tip_base, 0.0, 0.0)
            pulled, _ = pulled_shares(rig.arm(a), plan, rig.paper(a).normal)
            assert pulled > 0.99, pulled


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
    assert res["rep"].drawing_time < 500.0                  # measured 223 s (2026-10-07)
    plan_cpu = res["rep"].cpu - res["rep"].map_cpu
    assert plan_cpu < 100.0                                 # measured 8 to 16 s (maps apart)
    # the word goes to arm 2L in phase 1; nothing is left for the fill phases
    assert {ph for ph, _, _ in res["tagged"]} == {"phase 1"}
