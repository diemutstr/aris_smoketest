"""Tests for aris/rig.py.  Reference numbers from tests/oracle/make_rig_reference.py (old code).

Run with -s to see the tables (steel list, box counts, park clearances).
"""
import json
import re
import shutil
from pathlib import Path

import numpy as np
import pytest

from aris.kernel.tool import default_tool
from aris.rig import Rig
from aris.types import Line

DEPLOY = Path(__file__).resolve().parents[1]
CONFIG = DEPLOY / "config"
REF = np.load(DEPLOY / "tests" / "data" / "rig_reference.npz")
TIP_SHIFT = np.array([0.001, 0.0, -0.002])  # made-up calibrated pen tip, hand frame
SHIFT = REF["shift"]                      # canvas corner -> table centre, (0.9017, 1.81532, 0)
# the reference file names arms by their old robot ids; the rig names them by slot
OLD = {13: "1L", 17: "1R", 31: "2L", 71: "2R", 2: "3L", 97: "3R"}
SLOTS = ("1L", "1R", "2L", "2R", "3L", "3R")


@pytest.fixture(scope="module")
def rig():
    return Rig.load(CONFIG)


# --------------------------------------------------------------------------- 1. base poses


def test_T_table_base_matches_old(rig):
    worst = 0.0
    for aid, T_old in zip(REF["arm_ids"], REF["T_canvas_base"]):
        T_old = T_old.copy()
        T_old[:3, 3] -= SHIFT
        err = np.abs(rig.T_table_base(OLD[int(aid)]) - T_old).max()
        worst = max(worst, err)
    print(f"\nT_table_base vs old T_world_base(0.970), worst entry over six arms: {worst:.2e}")
    assert worst <= 1e-12
    assert rig.arm_ids == SLOTS and rig.slot_names == SLOTS


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
        assert pl.kind == "paper" and pl.margin == 0.020 and pl.pen_margin == 0.020
        assert pl.tool_margin == rig.clearance["tool_to_paper_m"]
        assert abs((0.0 - pl.offset) - 0.970) < 1e-12
    print(f"\nround trip table->base->table worst {worst_rt:.2e}; paper points off the "
          f"plane worst {worst_plane:.2e}")
    assert worst_rt <= 1e-12 and worst_plane <= 1e-12


def test_paper_margins_for_planning(rig):
    c, a = rig.clearance, rig.allowance
    pl = rig.paper("2L", for_planning=True)
    assert pl.margin == c["body_to_paper_m"] + a["body_to_paper_m"]
    assert pl.pen_margin == c["pen_lifted_to_paper_m"] + a["pen_lifted_to_paper_m"]
    assert pl.tool_margin == c["tool_to_paper_m"] + a["tool_to_paper_m"]


def test_walls_have_one_margin(rig):
    pl = rig.wall_in_base("1L", rig.wall_between("1L", "2R"))
    assert pl.pen_margin is None and pl.tool_margin is None


def test_to_base_refuses_a_base_frame_line(rig):
    with pytest.raises(ValueError):
        rig.to_base("1L", Line("l", np.zeros((2, 3)), "base"))


# --------------------------------------------------------------------------- 3. walls


@pytest.mark.parametrize("a,b,y_sign", [("1L", "2R", -1), ("2R", "3L", +1), ("1R", "2L", -1),
                                        ("2L", "3R", +1)])
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
        assert pl.kind == "wall" and pl.margin == 0.040
        assert 0.0 - pl.offset > 0.6776 - 1e-4        # base origin: free side, 0.6776 in
        # vertical: the normal, seen from the table, has no z
        np.testing.assert_allclose(rig.T_table_base(aid)[:3, :3] @ pl.normal @ [0, 0, 1], 0.0,
                                   atol=1e-15)
    print(f"\nwall {a}-{b}: through ({w.point_table[0]:+.4f}, {w.point_table[1]:+.7f}), "
          f"{abs(w.normal_table @ (rig.T_table_base(a)[:3, 3] - w.point_table)):.5f} from "
          f"both axes, {ang:.3f} deg from x")


def test_phase_walls_cross_on_the_centre_line(rig):
    w1, w2 = rig.wall_between("1L", "2R"), rig.wall_between("1R", "2L")
    np.testing.assert_allclose(w1.point_table, w2.point_table, atol=1e-15)
    assert abs(w1.normal_table @ w2.normal_table) < 0.61      # not parallel: they cross


def test_wall_planning_margin(rig):
    pl = rig.wall_in_base("1L", rig.wall_between("1L", "2R"), for_planning=True)
    assert pl.margin == 0.040 + rig.allowance["wall_m"]


# --------------------------------------------------------------------------- 4. steel


def _old_boxes_table(slot):
    k = REF["old_box_owner"] == {v: o for o, v in OLD.items()}[slot]
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


# the strut face schedule of docs/drawings/plan_centre_datum.pdf, sheet 3, panel D (mm)
SCHEDULE = {"left": (-305.00, (-476.75, -400.55), (-159.15, -82.95), (-392.76, -166.94)),
            "right": (305.00, (133.25, 209.45), (450.85, 527.05), (217.24, 443.06))}


def test_hanger_follows_the_drawing(rig):
    for aid in rig.arm_ids:
        ax = rig.T_table_base(aid)[0, 3]
        axis_mm, s1, s2, plate = SCHEDULE["left" if ax < 0 else "right"]
        assert abs(ax * 1000 - axis_mm) < 1e-9
        for name, (a, b) in ((f"strut{aid}_minus_x", s1), (f"strut{aid}_plus_x", s2),
                             (f"plate{aid}", plate)):
            box = next(x for x in rig.steel if x.name == name)
            # plate edges are printed to 0.01 mm: the 225.82 plate is centred on axis + 25.15
            np.testing.assert_allclose([box.lo_table[0] * 1000, box.hi_table[0] * 1000], [a, b],
                                       atol=0.006)
            if name.startswith("strut"):
                assert abs(box.hi_table[1] - box.lo_table[1] - 0.1524) < 1e-12


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
        print(f"{aid:>3s}  {len(obs.boxes):3d}  {len(obs_chk.boxes):3d}   "
              f"{', '.join(b.name for b in far)}")
        kinds = {}
        for name, lo, hi in _old_boxes_table(aid):
            if name.startswith("seam_bar"):
                np.testing.assert_allclose(new_by_name[name].lo_table, lo, atol=1e-9)
                np.testing.assert_allclose(new_by_name[name].hi_table, hi, atol=1e-9)
                kinds["seam"] = kinds.get("seam", 0) + 1
            elif name.startswith("mount:") and name.endswith("_plate"):
                other = OLD[int(name[6:].split("_")[0])]
                p = new_by_name[f"plate{other}"]
                assert abs(p.lo_table[2] - lo[2]) < 1e-9                # same underside
                assert abs((hi[0] - lo[0]) - (p.hi_table[0] - p.lo_table[0])) < 0.001
                kinds["plate"] = kinds.get("plate", 0) + 1
            elif name.startswith("mount:") and name.endswith("_boom"):
                other = OLD[int(name[6:].split("_")[0])]
                assert f"strut{other}_minus_x" in new_by_name
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
    arm = rig.arm("2L")
    rng = np.random.default_rng(5)
    lim = arm.limits
    Q = rng.uniform(lim.q_min, lim.q_max, size=(20_000, 7))
    body = arm.body(Q)
    s = np.array([0.0, 0.0, rig.shoulder_below_base])
    d = np.maximum(np.linalg.norm(body.p0 - s, axis=2), np.linalg.norm(body.p1 - s, axis=2))
    worst = float((d + body.radius).max())
    print(f"\nreach from shoulder, kernel body, 20k random configurations: {worst:.4f}")
    assert worst <= rig.body_reach


# --------------------------------------------------------------------------- 5. parked arms


def test_park_clearances_old_model(rig):
    print("\narm  paper(capsules)  pen tip z  self   steel(old set)  steel(new set, binding box)"
          "  own struts alone")
    for i, aid in enumerate(REF["arm_ids"]):
        np.testing.assert_allclose(REF["park_q"][i], rig.park_q(OLD[int(aid)]), atol=0)
        print(f"{OLD[int(aid)]:>3s}   {REF['park_caps_paper'][i]:.4f}   {REF['park_tip_z'][i]:.4f}   "
              f"{REF['park_self'][i]:.4f}  {REF['park_old_steel'][i]:.4f}   "
              f"{REF['park_new_steel'][i]:.4f} ({REF['park_new_steel_box'][i]})  "
              f"{REF['park_own_struts'][i]:.4f}")
    assert np.all(REF["park_caps_paper"] >= 0.020)
    assert np.all(REF["park_tip_z"] >= rig.clearance["pen_lifted_to_paper_m"])
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
        print(f"{aid:>3s}  {det.value[0]:+.4f}  {body.names[det.capsule[0]]} vs "
              f"{det.obstacle_names[det.obstacle[0]]};  self {sc:+.4f}")
        assert det.value[0] >= 0.0 and sc >= 0.0


def _split_own(obs, aid):
    """-> (the arm's own struts, plate and clamp; everything else), as two Obstacles."""
    from aris.types import Obstacles
    own_names = {f"strut{aid}_minus_x", f"strut{aid}_plus_x", f"plate{aid}", f"clamp{aid}"}
    own = tuple(b for b in obs.boxes if b.name in own_names)
    rest = tuple(b for b in obs.boxes if b.name not in own_names)
    assert len(own) == 4
    return Obstacles(boxes=own), Obstacles(rest, obs.planes, obs.capsules)


@pytest.mark.slow
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
        print(f"{aid:>3s}  {park:+.4f}   {int((hit & ok).sum())} of {int(ok.sum())}")
        assert park >= 0.0


def _link1_sweep(rig, aid, n=721):
    """Worst clearance of link 1 (all its capsules) over the whole q1 range, per obstacle class,
    beyond the demanded margin: link1_to_own_mount against the arm's own hanger steel, the usual
    margins against everything else.  Link 1 turns about the base axis only, so this is all of it."""
    from dataclasses import replace
    from aris.kernel.collide import capsule_clearance
    from aris.types import Obstacles
    arm = rig.arm(aid)
    q = np.zeros((n, 7))
    q[:, 0] = np.linspace(arm.limits.q_min[0], arm.limits.q_max[0], n)
    body = replace(arm.body(q), is_fixed=None)          # the kernel skips link 1: measure it
    k = [i for i, name in enumerate(body.names) if name.startswith("link1")]
    others = tuple(a for a in rig.arm_ids if a != aid)
    walls = tuple({w.name: w for ph in (rig.phase(1), rig.phase(2)) for w in ph.walls}.values())
    obs = rig.obstacles(aid, parked=others, walls=walls, for_planning=False)
    own, _ = _split_own(obs, aid)
    m_own = rig.clearance["link1_to_own_mount_m"]
    own = Obstacles(tuple(replace(b, margin=m_own, exempt=()) for b in own.boxes))
    own_names = {b.name for b in own.boxes}
    classes = {
        "own hanger": own,
        "other steel": Obstacles(tuple(b for b in obs.boxes if b.name not in own_names)),
        "walls": Obstacles(planes=tuple(p for p in obs.planes if p.kind == "wall")),
        "parked arms": Obstacles(capsules=obs.capsules),
        "paper": Obstacles(planes=tuple(p for p in obs.planes if p.kind == "paper")),
    }
    return {c: float(capsule_clearance(body, o, prune=False)[:, k].min())
            for c, o in classes.items()}


def test_link1_against_everything(rig):
    """Link 1 is not checked by the collision check (it only turns about the base axis), so the
    rig checks it once here against all it can ever meet: 0.020 to the arm's own hanger steel
    (same plate), the demanded clearances to everything else."""
    print("\nlink 1 over all of q1, worst clearance beyond the demanded margin")
    print("arm   own hanger  other steel   walls   parked arms   paper")
    for aid in rig.arm_ids:
        w = _link1_sweep(rig, aid)
        print(f"{aid:>3s}  " + "  ".join(f"{w[c]:+.4f}" for c in w))
        assert all(v >= 0.0 for v in w.values())


def test_gates(rig):
    g = rig.gates()
    assert abs(g.self_margin - 0.023) < 1e-15
    assert g.limit_margin == 0.15 and g.sigma_min == 0.04


def test_execution(rig, tmp_path):
    assert rig.execution().start_tolerance == 0.005
    cfg = json.loads((CONFIG / "rig.json").read_text())
    cfg["execution"]["start_tolerance_rad"] = 0.01
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    assert Rig.load(tmp_path).execution().start_tolerance == 0.01


def test_drawing_area(rig, tmp_path):
    cfg = json.loads((CONFIG / "rig.json").read_text())
    np.testing.assert_array_equal(rig.drawing_area_m, cfg["canvas"]["drawing_area_m"])
    assert rig.drawing_area_m.shape == (2,) and np.all(rig.drawing_area_m <= rig.canvas_size)
    np.testing.assert_array_equal(rig.drawing_area_centre_m, [0.0, 0.0])
    cfg["canvas"]["drawing_area_centre_m"] = [0.1, -0.2]
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    np.testing.assert_array_equal(Rig.load(tmp_path).drawing_area_centre_m, [0.1, -0.2])
    del cfg["canvas"]["drawing_area_m"], cfg["canvas"]["drawing_area_centre_m"]
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    r = Rig.load(tmp_path)
    assert r.drawing_area_m is None
    np.testing.assert_array_equal(r.drawing_area_centre_m, [0.0, 0.0])


def test_rules(rig):
    r = rig.rules()
    # press and speed on the paper are the current pen's (graphite_4h, 2026-10-01)
    assert r.draw_speed == 0.015 and r.press == 0.0035
    assert r.speed_fraction == 0.30 and abs(r.lean_max - np.deg2rad(15.0)) < 1e-15
    assert r.gates == rig.gates()


def test_rules_follow_the_config(tmp_path):
    cfg = json.loads((CONFIG / "rig.json").read_text())
    cfg["gates"]["sigma_min"] = 0.05
    cfg["gates"]["pen_lean_max_deg"] = 10.0
    cfg["pens"]["table"]["graphite_4h"]["speed_m_per_s"] = 0.03
    cfg["pens"]["table"]["graphite_4h"]["press_m"] = 0.002
    cfg["clearances"]["self_m"] = 0.030
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    r = Rig.load(tmp_path).rules()
    assert r.gates.sigma_min == 0.05 and r.draw_speed == 0.03 and r.press == 0.002
    assert abs(r.lean_max - np.deg2rad(10.0)) < 1e-15 and abs(r.gates.self_margin - 0.033) < 1e-15


# --------------------------------------------------------------------------- pens


def test_pen_is_the_current_entry_with_its_name(rig):
    cfg = json.loads((CONFIG / "rig.json").read_text())
    entry = cfg["pens"]["table"][cfg["pens"]["current"]]
    pen = rig.pen()
    assert pen["name"] == "graphite_4h" == cfg["pens"]["current"]
    assert {k: v for k, v in pen.items() if k != "name"} == {
        k: v for k, v in entry.items() if not k.endswith("note") and k != "source"}
    assert pen["press_m"] == 0.0035 and pen["force_band_n"] == [0.7, 1.0]
    assert json.loads(json.dumps(pen)) == pen                 # travels in a job header as is
    # the nominal pen is the tool model as built: nothing moved
    for slot in rig.arm_ids:
        np.testing.assert_array_equal(rig.arm(slot).tool.tip_hand, default_tool().tip_hand)


def test_a_longer_and_thicker_pen_moves_the_tip_and_the_capsule(tmp_path, rig):
    cfg = json.loads((CONFIG / "rig.json").read_text())
    cfg["pens"]["table"]["fat"] = dict(cfg["pens"]["table"]["graphite_4h"],
                                       tip_length_nominal_m=0.025, capsule_radius_m=0.004)
    cfg["pens"]["current"] = "fat"
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    r = Rig.load(tmp_path)
    tool0, tool = default_tool(), r.arm("2L").tool
    np.testing.assert_allclose(tool.tip_hand, tool0.tip_hand + 0.005 * tool0.pen_axis_hand,
                               atol=1e-15)
    pen = next(c for c in tool.capsules_hand if c.name == "pen")
    assert pen.radius == 0.004
    np.testing.assert_allclose(pen.p1 + pen.radius * tool.pen_axis_hand, tool.tip_hand,
                               atol=1e-15)
    assert r.pen()["name"] == "fat"
    cfg["pens"]["current"] = "nonesuch"
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    with pytest.raises(ValueError):
        Rig.load(tmp_path)


# --------------------------------------------------------------------------- slots


def test_slot_names_are_checked(tmp_path, rig):
    from aris.rig import is_slot
    assert all(is_slot(s) for s in rig.slot_names)
    assert not any(is_slot(s) for s in ("4L", "2l", "2", 31, "2RR", ""))
    with pytest.raises(KeyError, match="slot"):
        rig.T_table_base(31)                                   # an old robot id
    cfg = json.loads((CONFIG / "rig.json").read_text())
    for change in ({"slot": "2X"}, {"slot": "2R"}, {"axis_xy_m": [0.305, 0.0]}):
        bad = json.loads(json.dumps(cfg))
        bad["slots"]["list"][2].update(change)                # slot 2L
        (tmp_path / "rig.json").write_text(json.dumps(bad))
        with pytest.raises(ValueError):
            Rig.load(tmp_path)
    bad = json.loads(json.dumps(cfg))
    bad["slots"]["list"][0]["axis_xy_m"] = [-0.305, 1.5]     # 1L beyond row 3
    (tmp_path / "rig.json").write_text(json.dumps(bad))
    with pytest.raises(ValueError):
        Rig.load(tmp_path)


def test_own_hanger_exempts_link1_for_its_own_arm_only(rig):
    for aid in rig.arm_ids:
        obs = rig.obstacles(aid, for_planning=False)
        own = {f"strut{aid}_minus_x", f"strut{aid}_plus_x", f"plate{aid}", f"clamp{aid}"}
        for b in obs.boxes:
            assert b.exempt == (("link1",) if b.name in own else ())



def test_parked_arm_capsules(rig):
    obs = rig.obstacles("1L", parked=("1R",), walls=(rig.wall_between("1L", "2R"),))
    m = 0.050 + rig.allowance["arm_to_arm_m"]
    assert len(obs.capsules) > 0 and all(c.margin == m for c in obs.capsules)
    assert [p.kind for p in obs.planes] == ["paper", "wall"]
    # the parked capsules sit where 1R's own body is, seen from 1L
    body = rig.arm("1R").body(rig.park_q("1R")[None, :])
    p_table = rig.to_table("1R", body.p0[0])
    T = rig.T_base_table("1L")
    body_caps = [c for c in obs.capsules if c.name.startswith("parked1R:")]
    np.testing.assert_allclose(np.array([c.p0 for c in body_caps]),
                               p_table @ T[:3, :3].T + T[:3, 3], atol=1e-12)
    assert len(body_caps) == len(obs.capsules)


# --------------------------------------------------------------------------- 6. calibration


def _moved(T, dxyz, tilt_deg=0.0):
    a = np.radians(tilt_deg)
    tilt = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    out = T.copy()
    out[:3, :3] = tilt @ T[:3, :3]
    out[:3, 3] += dxyz
    return out


def _write_calibration(tmp_path, slot="2L", base=None, pen=None, base_passed=True,
                       pen_passed=True, pen_name="graphite_4h"):
    """A made-up two-part calibration file (BUILD.md); a part is left out when None."""
    shutil.copy(CONFIG / "rig.json", tmp_path / "rig.json")
    (tmp_path / "calibration").mkdir(exist_ok=True)
    cal = {"slot": slot}
    if base is not None:
        cal["base"] = {"passed": base_passed, "date": "2026-10-02", "method": "plane",
                       "T_table_base": base.tolist(), "residuals": {},
                       "why": "" if base_passed else "residual too large"}
    if pen is not None:
        cal["pen"] = {"passed": pen_passed, "date": "2026-10-02", "pen": pen_name,
                      "tip_hand_m": list(pen), "why": "" if pen_passed else "no contact",
                      "reference_touch": {"xy_table_m": [0.0, 0.3], "q": [0.0] * 7}}
    (tmp_path / "calibration" / f"{slot}.json").write_text(json.dumps(cal))


def _boxes_of(r, slot):
    return {b.name: b for b in r.steel if b.owner == slot}


def test_base_part_alone_changes_its_slot_only(tmp_path, rig):
    T = _moved(rig.T_table_base("2L"), [0.003, -0.002, 0.001], tilt_deg=0.5)
    _write_calibration(tmp_path, base=T)
    cal = Rig.load(tmp_path)
    st = cal.calibration_status("2L")
    assert st["base"].startswith("applied: 2L.json base") and st["pen"] == "none"
    assert not cal.calibrated("2L")
    np.testing.assert_allclose(cal.T_table_base("2L"), T, atol=0)
    p0, p1 = rig.paper("2L"), cal.paper("2L")
    assert not np.allclose(p0.normal, p1.normal) and abs(p0.offset - p1.offset) > 1e-4
    np.testing.assert_array_equal(cal.mounts["2L"].tip_hand, default_tool().tip_hand)
    for slot in rig.arm_ids:
        if slot == "2L":
            continue
        assert cal.calibration_status(slot) == {"base": "none", "pen": "none"}
        np.testing.assert_array_equal(cal.T_table_base(slot), rig.T_table_base(slot))
        a, b = rig.paper(slot), cal.paper(slot)
        np.testing.assert_array_equal(a.normal, b.normal)
        assert a.offset == b.offset
        oa, ob = rig.obstacles(slot), cal.obstacles(slot)
        assert [x.name for x in oa.boxes] == [x.name for x in ob.boxes]
        for x, y in zip(oa.boxes, ob.boxes):
            if y.name.endswith("2L") or "2L_" in y.name:
                continue                                # 2L's hanger moved with 2L
            np.testing.assert_array_equal(x.T_base_box, y.T_base_box)
    for a, b in [("1L", "2R"), ("2R", "3L"), ("1R", "2L"), ("2L", "3R")]:
        wa, wb = rig.wall_between(a, b), cal.wall_between(a, b)    # walls stay nominal
        np.testing.assert_array_equal(wa.point_table, wb.point_table)
        np.testing.assert_array_equal(wa.normal_table, wb.normal_table)
    for x, y in zip(rig.steel, cal.steel):
        if y.owner != "2L":
            np.testing.assert_array_equal(x.lo_table, y.lo_table)
    # the paper as 2L now sees it: its base origin is 0.970 + 1 mm above the paper
    assert abs(-p1.offset - (0.970 + 0.001)) < 1e-12


def test_hanger_follows_the_calibrated_axis(tmp_path, rig):
    off = np.array([0.020, -0.020, 0.0])
    _write_calibration(tmp_path, base=_moved(rig.T_table_base("2L"), off))
    cal = Rig.load(tmp_path)
    before, after = _boxes_of(rig, "2L"), _boxes_of(cal, "2L")
    assert sorted(before) == sorted(after) and len(after) == 4
    for name in before:
        np.testing.assert_allclose(after[name].lo_table - before[name].lo_table, off, atol=1e-12)
        np.testing.assert_allclose(after[name].hi_table - before[name].hi_table, off, atol=1e-12)
    for slot in rig.slot_names:
        if slot != "2L":
            for name, b in _boxes_of(rig, slot).items():
                np.testing.assert_array_equal(_boxes_of(cal, slot)[name].lo_table, b.lo_table)
    # seen from the arm, its own hanger has not moved, and link 1 still clears it
    T0, T1 = rig.T_base_table("2L"), cal.T_base_table("2L")
    for name in before:
        c0 = T0[:3, :3] @ (0.5 * (before[name].lo_table + before[name].hi_table)) + T0[:3, 3]
        c1 = T1[:3, :3] @ (0.5 * (after[name].lo_table + after[name].hi_table)) + T1[:3, 3]
        np.testing.assert_allclose(c0, c1, atol=1e-12)
    w = _link1_sweep(cal, "2L")
    print("\n2L moved 20 mm in x and y, link 1 over all of q1: " +
          ", ".join(f"{c} {v:+.4f}" for c, v in w.items()))
    assert all(v >= 0.0 for v in w.values())


def test_pen_part_for_another_pen_is_not_applied(tmp_path, rig):
    tip = default_tool().tip_hand + TIP_SHIFT
    _write_calibration(tmp_path, pen=tip, pen_name="gel_06")
    cal = Rig.load(tmp_path)
    st = cal.calibration_status("2L")
    assert st["base"] == "none"
    assert st["pen"].startswith("pen part not applied") and "gel_06" in st["pen"] \
        and "graphite_4h" in st["pen"]
    np.testing.assert_array_equal(cal.arm("2L").tool.tip_hand, default_tool().tip_hand)
    np.testing.assert_array_equal(cal.T_table_base("2L"), rig.T_table_base("2L"))


def test_both_parts_applied(tmp_path, rig):
    T = _moved(rig.T_table_base("2L"), [0.003, -0.002, 0.001], tilt_deg=0.5)
    _write_calibration(tmp_path, base=T, pen=default_tool().tip_hand + TIP_SHIFT)
    cal = Rig.load(tmp_path)
    st = cal.calibration_status("2L")
    assert st["base"].startswith("applied") and st["pen"].startswith("applied: 2L.json pen")
    assert cal.calibrated("2L") and not cal.calibrated("2R")
    np.testing.assert_allclose(cal.T_table_base("2L"), T, atol=0)
    q = rig.park_q("2L")[None, :]
    arm0, arm1 = rig.arm("2L"), cal.arm("2L")
    np.testing.assert_allclose(arm1.tool.tip_hand, arm0.tool.tip_hand + TIP_SHIFT, atol=1e-15)
    np.testing.assert_allclose(cal.mounts["2L"].tip_hand, arm1.tool.tip_hand, atol=0)
    tip, axis = arm1.tip(q)[0], arm1.pen_axis(q)[0]
    body = arm1.body(q)
    k = body.names.index("pen")
    end = body.p1[0, k] + body.radius[k] * axis        # where the pen capsule's surface ends
    print(f"\ncalibrated tip: pen capsule surface ends {np.linalg.norm(end - tip):.1e} m "
          f"from the tip; nominal tip moved {np.linalg.norm(tip - arm0.tip(q)[0]) * 1e3:.2f} mm")
    np.testing.assert_allclose(end, tip, atol=1e-12)


def test_parts_that_did_not_pass_are_not_applied(tmp_path, rig):
    T = _moved(rig.T_table_base("2L"), [0.003, 0.0, 0.0])
    tip = default_tool().tip_hand + TIP_SHIFT
    _write_calibration(tmp_path, base=T, pen=tip, base_passed=False)
    cal = Rig.load(tmp_path)
    st = cal.calibration_status("2L")
    assert st["base"] == "base part not applied: it did not pass (residual too large)"
    assert st["pen"].startswith("applied")                     # the parts are independent
    np.testing.assert_array_equal(cal.T_table_base("2L"), rig.T_table_base("2L"))
    _write_calibration(tmp_path, base=T, pen=tip, pen_passed=False)
    st = Rig.load(tmp_path).calibration_status("2L")
    assert st["base"].startswith("applied")
    assert st["pen"] == "pen part not applied: it did not pass (no contact)"


def test_malformed_calibration_is_refused(tmp_path, rig):
    T = rig.T_table_base("2L")
    T[0, 0] *= 1.1
    _write_calibration(tmp_path, base=T)
    with pytest.raises(ValueError):
        Rig.load(tmp_path)
    _write_calibration(tmp_path, base=rig.T_table_base("2L"))
    p = tmp_path / "calibration" / "2L.json"
    cal = json.loads(p.read_text())
    cal["slot"] = "2R"                                          # a file in the wrong place
    p.write_text(json.dumps(cal))
    with pytest.raises(ValueError):
        Rig.load(tmp_path)


# --------------------------------------------------------------------------- roles


def test_leaders_and_rows(rig):
    assert rig.leaders(1) == ("1L", "2R", "3L") and rig.leaders(2) == ("1R", "2L", "3R")
    pairs = {"1L": "1R", "1R": "1L", "2L": "2R", "2R": "2L", "3L": "3R", "3R": "3L"}
    assert all(rig.row_partner(a) == b for a, b in pairs.items())
    with pytest.raises(ValueError):
        rig.leaders(3)


# a file name used as a path: "rig.json" as a whole string, or the calibration folder joined on
PATH_IN_CONFIG = r"""(["'])rig\.json\1|/\s*(["'])calibration\2|["']calibration/"""


def test_config_only_read_by_rig():
    """Only rig.py reads config/.  Exempt: the independent checker, which has its own reader on
    purpose, and the calibration job, which writes config/calibration/."""
    pkg = DEPLOY / "aris"
    exempt = (pkg / "check", pkg / "calib")
    offenders = [p for p in pkg.rglob("*.py")
                 if p != pkg / "rig.py" and not any(e in p.parents for e in exempt)
                 and re.search(PATH_IN_CONFIG, p.read_text())]
    assert offenders == []




# --------------------------------------------------------------------------- phases


def test_phases(rig):
    for n, active, pairs in [(1, ("1L", "2R", "3L"), {("1L", "2R"), ("2R", "3L")}),
                             (2, ("1R", "2L", "3R"), {("1R", "2L"), ("2L", "3R")})]:
        ph = rig.phase(n)
        assert ph.active == active
        assert set(ph.parked) == set(rig.arm_ids) - set(active)
        assert {w.arms for w in ph.walls} == pairs
    print("\nslot  phase  parked arms it sees  walls")
    for n in (1, 2):
        ph = rig.phase(n)
        for slot in ph.active:
            obs = rig.obstacles_for(slot, ph)
            seen = sorted({c.name[6:].split(":")[0] for c in obs.capsules})
            walls = [p.name for p in obs.planes if p.kind == "wall"]
            print(f"{slot:>3s}  {n}  {seen}  {walls}")
            assert rig.row_partner(slot) in seen
            assert all(slot in w.split("_")[1:] for w in walls)
            assert len(walls) == (2 if slot in ("2R", "2L") else 1)
    with pytest.raises(ValueError):
        rig.obstacles_for("1R", rig.phase(1))


def test_obstacles_for_matches_obstacles(rig):
    ph = rig.phase(1)
    a = rig.obstacles_for("2R", ph, for_planning=False)
    seen = tuple(p for p in ph.parked
                 if any(c.name.startswith(f"parked{p}:") for c in a.capsules))
    assert "2L" in seen
    b = rig.obstacles("2R", parked=seen, walls=tuple(ph.walls), for_planning=False)
    assert [c.name for c in a.capsules] == [c.name for c in b.capsules]
    assert [p.name for p in a.planes] == [p.name for p in b.planes]


def test_the_two_live_arms_2L_and_2R():
    """config/two_arms is config/rig.json with only 2L and 2R mounted (tools/mounted_rig.py):
    every hanger stays, the middle row stays a row, walls go (the row partners never move
    together), fences toward rows 1 and 3."""
    import subprocess
    import sys
    two = Rig.load(CONFIG / "two_arms")
    six = Rig.load(CONFIG)
    assert two.arm_ids == ("2L", "2R") and two.slot_names == six.slot_names
    assert two.rows == (("2L", "2R"),) and two.leader_sets == {1: ("2R",), 2: ("2L",)}
    assert two.wall_pairs == {1: (), 2: ()}
    assert len(two.steel) == len(six.steel)                     # the empty hangers stay
    assert two.row_partner("2R") == "2L" and two.row_partner("1L") is None
    assert tuple(two.drawing_area_m) == (1.72, 0.9)
    assert tuple(two.drawing_area_centre_m) == (0.0, 0.0)
    assert json.loads((CONFIG / "two_arms" / "rig.json").read_text())["about"]["mounted"] == \
        ["2L", "2R"]
    # the derived file is in step with its source
    out = subprocess.run([sys.executable, str(DEPLOY / "tools" / "mounted_rig.py"),
                          "--check", str(CONFIG / "two_arms")], capture_output=True)
    assert out.returncode == 0, out.stdout.decode() + out.stderr.decode()


def test_fences_hold_in_every_phase_for_the_planner_and_the_checker():
    """config/two_arms fences off rows 1 and 3, whose arms hang there switched off: planes at
    y = -0.605 and +0.605 m that every controlled arm's body stays behind, in every phase and
    job, at the wall clearance."""
    from aris.check.config import read_rig
    from aris.check.scene import build_scene, clearance
    from aris.kernel.collide import clearance as kernel_clearance
    two, six = Rig.load(CONFIG / "two_arms"), Rig.load(CONFIG)
    fences = ["fence_row_-1p210", "fence_row_+1p210"]
    assert len(six.fences) == 0 and [f[0] for f in two.fences] == fences
    for ph in (two.phase(1), two.phase(2)):
        for a in ph.active:
            obs = two.obstacles_for(a, ph)
            names = [p.name for p in obs.planes]
            assert names == ["paper"] + fences
            assert all(p.margin == 0.040 + two.allowance["wall_m"] for p in obs.planes[1:])
    # a configuration of 2R reaching past y = -0.605 m toward row 1 is refused on both sides
    arm, T = two.arm("2R"), two.T_table_base("2R")
    rng = np.random.default_rng(3)
    Q = rng.uniform(arm.limits.q_min, arm.limits.q_max, size=(2000, 7))
    body = arm.body(Q)
    ys = np.minimum(body.p0 @ T[:3, :3].T, body.p1 @ T[:3, :3].T)[..., 1] + T[1, 3]
    far = np.flatnonzero(ys.min(axis=1) < -0.65)
    assert len(far) > 10, "no configuration of 2R reaches past the fence"
    reach = Q[far[0]]
    planner = kernel_clearance(arm.body(reach[None]), two.obstacles_for("2R", two.phase(1)))[0]
    assert planner < 0.0
    c = kernel_clearance(arm.body(Q[far]), two.obstacles_for("2R", two.phase(1)))
    assert np.all(c < 0.0)
    rig_c = read_rig(CONFIG / "two_arms")
    scene = build_scene(rig_c, "2R", (), ("2L",), drawing=False)
    chk = clearance(scene, reach[None])
    assert chk.value["walls"][0] < 0.0 and scene.plane_names == tuple(fences)


def test_a_pen_without_a_speed_draws_at_the_drawing_speed(tmp_path):
    """The same fallback as the checker's reader (aris/check/config.py)."""
    cfg = json.loads((CONFIG / "rig.json").read_text())
    del cfg["pens"]["table"]["graphite_4h"]["speed_m_per_s"]
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    with pytest.raises(ValueError):
        Rig.load(tmp_path)
    cfg["drawing"]["draw_speed_m_per_s"] = 0.010
    (tmp_path / "rig.json").write_text(json.dumps(cfg))
    r = Rig.load(tmp_path)
    assert r.rules().draw_speed == 0.010 and r.pen()["speed_m_per_s"] == 0.010


def test_nominal_tool_is_the_uncalibrated_tool(tmp_path, rig):
    np.testing.assert_array_equal(rig.nominal_tip(), default_tool().tip_hand)
    _write_calibration(tmp_path, pen=default_tool().tip_hand + TIP_SHIFT)
    cal = Rig.load(tmp_path)
    np.testing.assert_array_equal(cal.nominal_tip(), rig.arm("2R").tool.tip_hand)
    assert not np.array_equal(cal.nominal_tip(), cal.arm("2L").tool.tip_hand)
    assert cal.pen_name == cal.pen()["name"] == "graphite_4h"


# --------------------------------------------------------------------------- calibration marks


def test_the_ten_marks(rig):
    assert len(rig.marks) == 10
    xy, share = rig.marks["S12R"]
    np.testing.assert_array_equal(xy, [0.30, -0.605])
    assert share == ("1R", "2R")
    assert rig.marks_for(("2L", "2R")) == ("A", "B")
    assert rig.marks_for(rig.mark_groups["rows12"]) == ("A", "B", "R1a", "R1b", "S12L", "S12R")
    assert set(rig.marks_for(rig.mark_groups["all"])) == set(rig.marks)
    every = [s for _, (_, share) in rig.marks.items() for s in share]
    assert all(every.count(s) >= 3 for s in rig.slot_names)       # every arm: two spots or more
    assert rig.mark_state("A") == "nominal"
    np.testing.assert_array_equal(rig.mark_xy("A"), [0.0, -0.40])


def test_a_solved_mark_moves(tmp_path, rig):
    shutil.copy(CONFIG / "rig.json", tmp_path / "rig.json")
    (tmp_path / "calibration").mkdir()
    solved = {"marks": {"A": {"xy_m": [0.003, -0.40], "state": "solved", "date": "2026-10-05",
                              "residual_mm": 0.4},
                        "B": {"xy_m": [0.0, 0.40], "state": "nominal"},
                        "Z9": {"xy_m": [0.0, 0.0], "state": "solved"}}}   # not on this rig
    (tmp_path / "calibration" / "marks.json").write_text(json.dumps(solved))
    r = Rig.load(tmp_path)
    assert r.mark_state("A") == "solved" and r.mark_state("B") == "nominal"
    np.testing.assert_allclose(r.mark_xy("A") - rig.mark_xy("A"), [0.003, 0.0], atol=1e-15)
    np.testing.assert_array_equal(r.mark_xy("B"), [0.0, 0.40])
    np.testing.assert_array_equal(r.marks["A"][0], [0.0, -0.40])      # nominal unchanged


def test_two_arms_keep_their_marks_and_groups():
    import subprocess
    import sys
    two = Rig.load(CONFIG / "two_arms")
    assert tuple(two.marks) == ("A", "B") and two.mark_groups == {"row2": ("2L", "2R")}
    out = subprocess.run([sys.executable, str(DEPLOY / "tools" / "mounted_rig.py"),
                          "--check", str(CONFIG / "two_arms")], capture_output=True)
    assert out.returncode == 0, out.stdout.decode() + out.stderr.decode()
