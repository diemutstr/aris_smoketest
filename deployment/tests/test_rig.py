"""Tests for aris/rig.py.  Reference numbers from tests/oracle/make_rig_reference.py (old code).

Run with -s to see the tables (steel list, box counts, park clearances).
"""
import json
import shutil
from pathlib import Path

import numpy as np
import pytest

from aris.rig import Rig
from aris.types import Line

DEPLOY = Path(__file__).resolve().parents[1]
CONFIG = DEPLOY / "config"
REF = np.load(DEPLOY / "tests" / "data" / "rig_reference.npz")
SHIFT = REF["shift"]                      # canvas corner -> table centre, (0.9017, 1.81532, 0)


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


# --------------------------------------------------------------------------- 1. base poses


def test_T_table_base_matches_old(rig):
    worst = 0.0
    for aid, T_old in zip(REF["arm_ids"], REF["T_canvas_base"]):
        T_old = T_old.copy()
        T_old[:3, 3] -= SHIFT
        err = np.abs(rig.T_table_base(int(aid)) - T_old).max()
        worst = max(worst, err)
    print(f"\nT_table_base vs old T_world_base(0.970), worst entry over six arms: {worst:.2e}")
    assert worst <= 1e-12
    assert rig.arm_ids == (13, 17, 31, 71, 2, 97)


def test_T_base_table_is_the_inverse(rig):
    for aid in rig.arm_ids:
        np.testing.assert_allclose(rig.T_base_table(aid) @ rig.T_table_base(aid), np.eye(4),
                                   atol=1e-15)


# --------------------------------------------------------------------------- 2. paper


def test_paper_round_trip_and_plane(rig):
    rng = np.random.default_rng(2)
    half = rig.canvas_size / 2
    p_table = np.column_stack([rng.uniform(-half[0], half[0], 500),
                               rng.uniform(-half[1], half[1], 500), np.zeros(500)])
    worst_rt = worst_plane = 0.0
    for aid in rig.arm_ids:
        line = rig.to_base(aid, Line("l", p_table, "table"))
        assert line.frame == "base"
        back = rig.to_table(aid, line.points)
        worst_rt = max(worst_rt, np.abs(back - p_table).max())
        pl = rig.paper(aid)
        worst_plane = max(worst_plane, np.abs(line.points @ pl.normal - pl.offset).max())
        # the arm's base origin is on the free side, 0.970 above the paper
        assert pl.kind == "paper" and pl.margin == 0.020 and pl.pen_margin == 0.003
        assert abs((0.0 - pl.offset) - 0.970) < 1e-12
    print(f"\nround trip table->base->table worst {worst_rt:.2e}; paper points off the "
          f"plane worst {worst_plane:.2e}")
    assert worst_rt <= 1e-12 and worst_plane <= 1e-12


def test_to_base_refuses_a_base_frame_line(rig):
    with pytest.raises(ValueError):
        rig.to_base(13, Line("l", np.zeros((2, 3)), "base"))


# --------------------------------------------------------------------------- 3. walls


@pytest.mark.parametrize("a,b,y_sign", [(13, 71, -1), (71, 2, +1), (17, 31, -1), (31, 97, +1)])
def test_walls(rig, a, b, y_sign):
    w = rig.wall_between(a, b)
    # passes through (0, +-0.6051); exactly +-L/6 of the 3.63064 canvas
    np.testing.assert_allclose(w.point_table[:2], [0.0, y_sign * 0.6051], atol=1e-4)
    np.testing.assert_allclose(w.point_table[:2], [0.0, y_sign * 3.63064 / 6], atol=1e-12)
    for aid in (a, b):
        ax = rig.T_table_base(aid)[:3, 3]
        d = abs(w.normal_table @ (ax - w.point_table))
        assert abs(d - 0.6776) < 1e-4
    direction = np.array([-w.normal_table[1], w.normal_table[0]])   # along the wall
    ang = np.degrees(np.arccos(abs(direction[0])))
    assert abs(ang - 26.75) < 0.01
    for aid in (a, b):
        pl = rig.wall_in_base(aid, w)
        assert pl.kind == "wall" and pl.margin == 0.025
        assert 0.0 - pl.offset > 0.6776 - 1e-4        # base origin: free side, 0.6776 in
        # vertical: the normal, seen from the table, has no z
        np.testing.assert_allclose(rig.T_table_base(aid)[:3, :3] @ pl.normal @ [0, 0, 1], 0.0,
                                   atol=1e-15)
    print(f"\nwall {a}-{b}: through ({w.point_table[0]:+.4f}, {w.point_table[1]:+.7f}), "
          f"{abs(w.normal_table @ (rig.T_table_base(a)[:3, 3] - w.point_table)):.5f} from "
          f"both axes, {ang:.3f} deg from x")


def test_phase_walls_cross_on_the_centre_line(rig):
    w1, w2 = rig.wall_between(13, 71), rig.wall_between(17, 31)
    np.testing.assert_allclose(w1.point_table, w2.point_table, atol=1e-15)
    assert abs(w1.normal_table @ w2.normal_table) < 0.61      # not parallel: they cross


def test_wall_planning_margin(rig):
    pl = rig.wall_in_base(13, rig.wall_between(13, 71), for_planning=True)
    assert pl.margin == 0.025 + rig.allowance["wall_m"]


# --------------------------------------------------------------------------- 4. steel


def _old_boxes_table(aid):
    k = REF["old_box_owner"] == aid
    return [(str(n), lo - SHIFT, hi - SHIFT)
            for n, lo, hi in zip(REF["old_box_name"][k], REF["old_box_lo"][k],
                                 REF["old_box_hi"][k])]


def test_steel_list(rig):
    print("\nsteel, table frame (m): name, size x y z, centre x y z, source")
    for b in rig.steel:
        size, c = b.hi_table - b.lo_table, 0.5 * (b.hi_table + b.lo_table)
        print(f"  {b.name:15s} {size[0]:.4f} {size[1]:.4f} {size[2]:.4f}   "
              f"{c[0]:+.4f} {c[1]:+.4f} {c[2]:+.4f}   {b.source[:70]}")
        assert np.all(size > 0) and b.source
    assert len(rig.steel) == 13 + 6 * 4


def test_struts_follow_petes_tape(rig):
    for aid in rig.arm_ids:
        ax = rig.T_table_base(aid)[0, 3]
        wide = next(b for b in rig.steel if b.name == f"strut{aid}_wide")
        narrow = next(b for b in rig.steel if b.name == f"strut{aid}_narrow")
        assert abs(wide.lo_table[0] - (ax - 0.240)) < 1e-12       # wide side toward -x
        assert abs(narrow.hi_table[0] - (ax + 0.156)) < 1e-12
        assert abs(narrow.hi_table[0] - wide.lo_table[0] - 0.396) < 1e-12
        plate = next(b for b in rig.steel if b.name == f"plate{aid}")
        assert plate.lo_table[0] > wide.hi_table[0] and plate.hi_table[0] < narrow.lo_table[0]


def test_strut_side_setting_mirrors(tmp_path):
    cfg = json.loads((CONFIG / "rig.json").read_text())
    for a in cfg["arms"]["list"]:
        a["strut_wide_side"] = "+x"
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    r = Rig.load(tmp_path)
    ax = r.T_table_base(31)[0, 3]
    wide = next(b for b in r.steel if b.name == "strut31_wide")
    assert abs(wide.hi_table[0] - (ax + 0.240)) < 1e-12


def test_steel_per_arm_and_against_old(rig):
    """Every old box is accounted for: a seam bar (identical), a neighbour's plate (moved and
    thinner, see rig.md), a neighbour's boom (replaced by the struts), or a neighbour's body
    column (an arm, not steel: it arrives as a parked arm's capsules or behind a wall)."""
    new_by_name = {b.name: b for b in rig.steel}
    print("\narm  boxes(for planning)  boxes(checker)  out of reach")
    for aid in rig.arm_ids:
        near, far = rig.steel_for(aid, rig.clearance["steel_m"] + rig.allowance["steel_m"])
        obs = rig.obstacles(aid)
        obs_chk = rig.obstacles(aid, for_planning=False)
        assert len(obs.boxes) == len(near)
        print(f"{aid:3d}  {len(obs.boxes):3d}  {len(obs_chk.boxes):3d}   "
              f"{', '.join(b.name for b in far)}")
        kinds = {}
        for name, lo, hi in _old_boxes_table(aid):
            if name.startswith("seam_bar"):
                np.testing.assert_allclose(new_by_name[name].lo_table, lo, atol=1e-9)
                np.testing.assert_allclose(new_by_name[name].hi_table, hi, atol=1e-9)
                kinds["seam"] = kinds.get("seam", 0) + 1
            elif name.startswith("mount:") and name.endswith("_plate"):
                other = int(name[6:].split("_")[0])
                p = new_by_name[f"plate{other}"]
                assert abs(p.lo_table[2] - lo[2]) < 1e-9                # same underside
                assert abs((hi[0] - lo[0]) - (p.hi_table[0] - p.lo_table[0])) < 0.001
                kinds["plate"] = kinds.get("plate", 0) + 1
            elif name.startswith("mount:") and name.endswith("_boom"):
                other = int(name[6:].split("_")[0])
                assert f"strut{other}_wide" in new_by_name
                kinds["boom"] = kinds.get("boom", 0) + 1
            elif name.startswith("body:"):
                kinds["column"] = kinds.get("column", 0) + 1
            else:
                raise AssertionError(f"old box {name} not accounted for")
        assert kinds == {"seam": 2, "plate": 5, "boom": 5, "column": 20}


def test_reach_bound_covers_the_old_body(rig):
    print(f"\nreach from shoulder: config {rig.body_reach} m, old capsules sampled "
          f"{float(REF['reach_sampled']):.4f} m")
    assert rig.body_reach >= float(REF["reach_sampled"])


def test_reach_bound_covers_the_kernel_body(rig):
    arm = rig.arm(31)
    rng = np.random.default_rng(5)
    lim = arm.limits
    Q = rng.uniform(lim.q_min, lim.q_max, size=(50_000, 7))
    body = arm.body(Q)
    s = np.array([0.0, 0.0, rig.shoulder_below_base])
    d = np.maximum(np.linalg.norm(body.p0 - s, axis=2), np.linalg.norm(body.p1 - s, axis=2))
    worst = float((d + body.radius).max())
    print(f"\nreach from shoulder, kernel body, 50k random configurations: {worst:.4f}")
    assert worst <= rig.body_reach


# --------------------------------------------------------------------------- 5. parked arms


def test_park_clearances_old_model(rig):
    print("\narm  paper(capsules)  pen tip z  self   steel(old set)  steel(new set, binding box)"
          "  own struts alone")
    for i, aid in enumerate(REF["arm_ids"]):
        np.testing.assert_allclose(REF["park_q"][i], rig.park_q(int(aid)), atol=0)
        print(f"{aid:3d}   {REF['park_caps_paper'][i]:.4f}   {REF['park_tip_z'][i]:.4f}   "
              f"{REF['park_self'][i]:.4f}  {REF['park_old_steel'][i]:.4f}   "
              f"{REF['park_new_steel'][i]:.4f} ({REF['park_new_steel_box'][i]})  "
              f"{REF['park_own_struts'][i]:.4f}")
    assert np.all(REF["park_caps_paper"] >= 0.020)
    assert np.all(REF["park_tip_z"] >= 0.003)
    assert np.all(REF["park_self"] >= 0.020)
    assert np.all(REF["park_new_steel"] >= 0.050)
    assert np.all(REF["park_own_struts"] >= 0.050)


def test_park_clearances_kernel(rig):
    from aris.kernel.collide import clearance_detail, self_clearance
    print("\narm  worst clearance beyond the demanded margin (kernel), against what; self")
    for aid in rig.arm_ids:
        arm = rig.arm(aid)
        body = arm.body(rig.park_q(aid)[None, :])
        det = clearance_detail(body, rig.obstacles(aid, for_planning=False), prune=False)
        sc = float(self_clearance(body, arm.self_pairs, rig.self_margin())[0])
        print(f"{aid:3d}  {det.value[0]:+.4f}  {body.names[det.capsule[0]]} vs "
              f"{det.obstacle_names[det.obstacle[0]]};  self {sc:+.4f}")
        assert det.value[0] >= 0.0 and sc >= 0.0


def _split_own(obs, aid):
    """-> (the arm's own struts, plate and clamp; everything else), as two Obstacles."""
    from aris.types import Obstacles
    own_names = {f"strut{aid}_wide", f"strut{aid}_narrow", f"plate{aid}", f"clamp{aid}"}
    own = tuple(b for b in obs.boxes if b.name in own_names)
    rest = tuple(b for b in obs.boxes if b.name not in own_names)
    assert len(own) == 4
    return Obstacles(boxes=own), Obstacles(rest, obs.planes, obs.capsules)


def test_own_hardware(rig):
    """The arm's own struts, plate and clamp are obstacles for it (its fixed base capsules are
    not checked).  Parks keep the demanded clearance to them; count the random configurations
    that are otherwise fine and still come too close."""
    from aris.kernel.collide import clearance, self_clearance
    rng = np.random.default_rng(11)
    print("\narm  park vs own hardware (beyond 0.050)  random configs too close / otherwise fine")
    for aid in rig.arm_ids:
        arm = rig.arm(aid)
        own, rest = _split_own(rig.obstacles(aid, for_planning=False), aid)
        park = clearance(arm.body(rig.park_q(aid)[None, :]), own, prune=False)[0]
        lim = arm.limits
        body = arm.body(rng.uniform(lim.q_min, lim.q_max, size=(10_000, 7)))
        ok = self_clearance(body, arm.self_pairs, rig.self_margin()) >= 0.0
        ok &= clearance(body, rest) >= 0.0
        hit = clearance(body, own) < 0.0
        print(f"{aid:3d}  {park:+.4f}   {int((hit & ok).sum())} of {int(ok.sum())}")
        assert park >= 0.0


def test_link1_against_own_struts_over_q1(rig):
    """Link 1 turns with q1 alone, 35 mm under the struts' bottom ends.  It keeps the demanded
    clearance at every q1, but not the planning allowance on part of the q1 range."""
    from aris.kernel.collide import capsule_clearance
    arm = rig.arm(31)
    own, _ = _split_own(rig.obstacles(31, for_planning=False), 31)
    q = np.zeros((721, 7))
    q[:, 0] = np.linspace(arm.limits.q_min[0], arm.limits.q_max[0], 721)
    body = arm.body(q)
    k = [i for i, n in enumerate(body.names) if n.startswith("link1")]
    v = capsule_clearance(body, own, prune=False)[:, k].min(axis=1)
    short = float((v < rig.allowance["steel_m"]).mean())
    print(f"\nlink1 vs own struts over q1, beyond 0.050: {v.min():+.4f} .. {v.max():+.4f}; "
          f"short of the {rig.allowance['steel_m']} planning allowance on {100 * short:.0f} % "
          "of the q1 range")
    assert v.min() >= 0.0
    # room under the strut ends, surface to steel, at every q1
    print(f"link1 room under its own struts: {0.050 + v.min():.4f} .. {0.050 + v.max():.4f} m")


def test_gates(rig):
    g = rig.gates()
    assert abs(g.self_margin - 0.023) < 1e-15


def test_parked_arm_capsules(rig):
    obs = rig.obstacles(13, parked=(17,), walls=(rig.wall_between(13, 71),))
    m = 0.050 + rig.allowance["arm_to_arm_m"]
    assert len(obs.capsules) > 0 and all(c.margin == m for c in obs.capsules)
    assert [p.kind for p in obs.planes] == ["paper", "wall"]
    # the parked capsules sit where arm 17's own body is, seen from arm 13
    body = rig.arm(17).body(rig.park_q(17)[None, :])
    p_table = rig.to_table(17, body.p0[0])
    T = rig.T_base_table(13)
    np.testing.assert_allclose(np.array([c.p0 for c in obs.capsules]),
                               p_table @ T[:3, :3].T + T[:3, 3], atol=1e-12)


# --------------------------------------------------------------------------- 6. calibration


def _calibrated_config(tmp_path, passed=True):
    shutil.copy(CONFIG / "rig.json", tmp_path / "rig.json")
    (tmp_path / "calibration").mkdir()
    base = Rig.load(CONFIG).T_table_base(31)
    a = np.radians(0.5)
    tilt = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    T = base.copy()
    T[:3, :3] = tilt @ base[:3, :3]
    T[:3, 3] += [0.003, -0.002, 0.001]
    cal = {"arm_id": 31, "date": "2026-10-01", "passed": passed, "T_table_base": T.tolist(),
           "tip_hand_m": [0.086, 0.0, 0.150], "source": "made up for tests/test_rig.py"}
    (tmp_path / "calibration" / "31.json").write_text(json.dumps(cal))
    return T


def test_calibration_changes_arm_31_only(tmp_path, rig):
    T = _calibrated_config(tmp_path)
    cal = Rig.load(tmp_path)
    assert cal.calibration_status(31).startswith("applied")
    np.testing.assert_allclose(cal.T_table_base(31), T, atol=0)
    assert not np.allclose(cal.T_table_base(31), rig.T_table_base(31))
    p0, p1 = rig.paper(31), cal.paper(31)
    assert not np.allclose(p0.normal, p1.normal) and abs(p0.offset - p1.offset) > 1e-4
    np.testing.assert_allclose(cal.mounts[31].tip_hand, [0.086, 0.0, 0.150])
    for aid in rig.arm_ids:
        if aid == 31:
            continue
        assert cal.calibration_status(aid) == "none"
        np.testing.assert_array_equal(cal.T_table_base(aid), rig.T_table_base(aid))
        a, b = rig.paper(aid), cal.paper(aid)
        np.testing.assert_array_equal(a.normal, b.normal)
        assert a.offset == b.offset
        oa, ob = rig.obstacles(aid), cal.obstacles(aid)
        assert [x.name for x in oa.boxes] == [x.name for x in ob.boxes]
        for x, y in zip(oa.boxes, ob.boxes):
            np.testing.assert_array_equal(x.T_base_box, y.T_base_box)
    for a, b in [(13, 71), (71, 2), (17, 31), (31, 97)]:
        wa, wb = rig.wall_between(a, b), cal.wall_between(a, b)
        np.testing.assert_array_equal(wa.point_table, wb.point_table)
        np.testing.assert_array_equal(wa.normal_table, wb.normal_table)
    for x, y in zip(rig.steel, cal.steel):
        np.testing.assert_array_equal(x.lo_table, y.lo_table)
    # the paper as arm 31 now sees it: its base origin is 0.970 + 1 mm above the paper
    assert abs(-p1.offset - (0.970 + 0.001)) < 1e-12


def test_calibration_that_did_not_pass_is_not_applied(tmp_path, rig):
    _calibrated_config(tmp_path, passed=False)
    cal = Rig.load(tmp_path)
    assert cal.calibration_status(31).startswith("not applied")
    np.testing.assert_array_equal(cal.T_table_base(31), rig.T_table_base(31))


def test_calibration_that_is_not_rigid_is_refused(tmp_path):
    _calibrated_config(tmp_path)
    p = tmp_path / "calibration" / "31.json"
    cal = json.loads(p.read_text())
    cal["T_table_base"][0][0] *= 1.1
    p.write_text(json.dumps(cal))
    with pytest.raises(ValueError):
        Rig.load(tmp_path)


# --------------------------------------------------------------------------- roles


def test_leaders_and_rows(rig):
    assert rig.leaders(1) == (13, 71, 2) and rig.leaders(2) == (17, 31, 97)
    pairs = {13: 17, 17: 13, 31: 71, 71: 31, 2: 97, 97: 2}
    assert all(rig.row_partner(a) == b for a, b in pairs.items())
    with pytest.raises(ValueError):
        rig.leaders(3)


def test_config_only_read_by_rig():
    pkg = DEPLOY / "aris"
    offenders = [p for p in pkg.rglob("*.py")
                 if p.name != "rig.py" and "rig.json" in p.read_text()]
    assert offenders == []


# --------------------------------------------------------------------------- phases


def test_phases(rig):
    for n, active, pairs in [(1, (13, 71, 2), {(13, 71), (71, 2)}),
                             (2, (17, 31, 97), {(17, 31), (31, 97)})]:
        ph = rig.phase(n)
        assert ph.active == active
        assert set(ph.parked) == set(rig.arm_ids) - set(active)
        assert {w.arms for w in ph.walls} == pairs
    print("\narm  phase  parked arms it sees  walls")
    for n in (1, 2):
        ph = rig.phase(n)
        for aid in ph.active:
            obs = rig.obstacles_for(aid, ph)
            seen = sorted({int(c.name[6:].split(":")[0]) for c in obs.capsules})
            walls = [p.name for p in obs.planes if p.kind == "wall"]
            print(f"{aid:3d}  {n}  {seen}  {walls}")
            assert rig.row_partner(aid) in seen
            assert all(aid in map(int, w.split("_")[1:]) for w in walls)
            assert len(walls) == (2 if aid in (71, 31) else 1)
    with pytest.raises(ValueError):
        rig.obstacles_for(17, rig.phase(1))


def test_obstacles_for_matches_obstacles(rig):
    ph = rig.phase(1)
    a = rig.obstacles_for(71, ph, for_planning=False)
    seen = tuple(p for p in ph.parked
                 if any(c.name.startswith(f"parked{p}:") for c in a.capsules))
    assert 31 in seen
    b = rig.obstacles(71, parked=seen, walls=tuple(ph.walls), for_planning=False)
    assert [c.name for c in a.capsules] == [c.name for c in b.capsules]
    assert [p.name for p in a.planes] == [p.name for p in b.planes]
