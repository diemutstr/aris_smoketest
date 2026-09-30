"""Tests for aris/check, the independent checker.

Run with -s to see the measured numbers.  The tests may import the planners' kernel and rig to
compare answers; the checker itself may not (the first test enforces that).
"""
import ast
import time
import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from aris.check import check, check_phase_end
from aris.check import geometry as cgeo
from aris.check.config import read_rig
from aris.check.model import capsules, frames, load_model
from aris.check.scene import CLASSES, build_scene, clearance
from aris.kernel import collide, fr3 as kfr3
from aris.kernel.arm import default_tool
from aris.kernel.retime import retime, retime_detailed, sample
from aris.rig import Rig
from aris.types import (DrawRules, JointPath, Line, Motion, Obstacles, Phase, Piece,
                        Trajectory)

DEPLOY = Path(__file__).resolve().parents[1]
CONFIG = DEPLOY / "config"
REPO = DEPLOY   # the package is at the repository root
RIG = Rig.load(CONFIG)
MINE = read_rig(CONFIG)
MINE_MODEL = load_model()
KB = "numpy"          # the kernel's numpy engine: the reference, whatever is compiled
N_AGREE = 10_000


def phase_of(arm_id) -> Phase:
    return RIG.phase(1) if arm_id in RIG.leaders(1) else RIG.phase(2)


# =========================================================================== helpers


def kernel_classes(arm_id, phase, Q, drawing=False) -> dict:
    """The planners' answer per obstacle class (gap minus the demanded clearance).  Against the
    paper the kernel's raw gaps are taken per capsule and the demanded clearances of rig.json
    subtracted here, so the comparison does not depend on how the kernel splits them."""
    arm = RIG.arm(arm_id)
    body = arm.body(Q)
    walls = [w for w in phase.walls if arm_id in w.arms]
    obs = RIG.obstacles(arm_id, parked=phase.parked, walls=walls, for_planning=False)
    raw = replace(obs.planes[0], margin=0.0, pen_margin=0.0, tool_margin=0.0)
    per = collide.capsule_clearance(body, Obstacles(planes=(raw,)), drawing, prune=False,
                                     backend=KB)
    tool = MINE_MODEL.is_tool if body.is_tool is None else body.is_tool
    links = ~body.is_pen & ~tool
    c = MINE.clearance
    return dict(
        steel=collide.clearance(body, Obstacles(boxes=obs.boxes), prune=False, backend=KB),
        links=per[:, links].min(1) - c["body_to_paper_m"],
        tool=per[:, tool].min(1) - c["tool_to_paper_m"],
        pen=(per[:, body.is_pen].min(1) - c["pen_lifted_to_paper_m"]) if not drawing
        else np.full(len(Q), np.inf),
        walls=collide.clearance(body, Obstacles(planes=obs.planes[1:]), prune=False, backend=KB),
        parked=collide.clearance(body, Obstacles(capsules=obs.capsules), prune=False, backend=KB),
        fields=np.full(len(Q), np.inf),
        self=collide.self_clearance(body, arm.self_pairs, RIG.clearance["self_m"],
                                     backend=KB))


def my_classes(arm_id, phase, Q, drawing=False, chunk=500, fields=()) -> dict:
    scene = build_scene(MINE, arm_id, phase.walls, phase.parked, drawing, fields=fields)
    parts = [clearance(scene, Q[s:s + chunk]).value for s in range(0, len(Q), chunk)]
    return {c: np.concatenate([p[c] for p in parts]) for c in CLASSES}


def free_motion(arm_id, path, speed_fraction=0.3) -> Motion:
    arm = RIG.arm(arm_id)
    traj = retime(JointPath(np.asarray(path, float)), arm.limits,
                  DrawRules(speed_fraction=speed_fraction))
    assert isinstance(traj, Trajectory), traj
    return Motion("free", traj)


def resample(traj: Trajectory, hz: float) -> Trajectory:
    t = np.arange(traj.t[0], traj.t[-1], 1.0 / hz)
    t = np.append(t[t < traj.t[-1] - 0.25 / hz], traj.t[-1])
    q, qd, _ = sample(traj, t)
    return Trajectory(t, q, qd)


def ik_path(arm_id, pts_table, lean=(0.0, 0.0)):
    """Joint path putting the tip on the table-frame points: one fixed spin, joint 7 and IK
    branch, chosen for the best clearance to everything but the paper."""
    arm = RIG.arm(arm_id)
    tip_b = RIG.to_base(arm_id, Line("x", np.asarray(pts_table, float), "table")).points
    n = RIG.paper(arm_id).normal
    M = len(tip_b)
    ph = phase_of(arm_id)
    best, best_q = -np.inf, None
    for spin in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        T = arm.hand_pose(tip_b, n, np.full(M, spin), np.tile(lean, (M, 1)))
        for q7 in np.linspace(-2.2, 2.2, 9):
            Q, valid = arm.ik(T, np.full(M, q7))
            for b in range(Q.shape[1]):
                if not valid[:, b].all():
                    continue
                qb = Q[:, b]
                if np.abs(np.diff(qb, axis=0)).max() > 0.05:
                    continue
                k = kernel_classes(arm_id, ph, qb[::10], drawing=True)
                score = min(k[c].min() for c in ("steel", "walls", "parked", "self"))
                if score > best:
                    best, best_q = score, qb
    assert best_q is not None, "no continuous IK path"
    return best_q, tip_b


def draw_motion(arm_id, pts_table, split=None, lean=(0.0, 0.0)):
    """A drawing motion along the table-frame points.  With `split` (a sample index), the line
    is drawn as two motions that meet there, each starting and ending at rest."""
    arm = RIG.arm(arm_id)
    Q, tip_b = ik_path(arm_id, pts_table, lean)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(tip_b, axis=0), axis=1))])
    parts = [slice(0, len(Q))] if split is None else [slice(0, split + 1), slice(split, len(Q))]
    out = []
    for sl in parts:
        res = retime_detailed(JointPath(Q[sl]), arm.limits, DrawRules(), s=s[sl],
                              tip_of=arm.tip)
        assert not hasattr(res, "reason"), res
        tip_base = np.stack([np.interp(res.s, s, tip_b[:, j]) for j in range(3)], 1)
        out.append(Motion("draw", res.traj, Piece("line", float(s[sl][0]), float(s[sl][-1])),
                          tip_base))
    return out[0] if split is None else out


def line_table(a, b, n=120, z=0.0):
    u = np.linspace(0, 1, n)[:, None]
    p = (1 - u) * np.array([*a, z]) + u * np.array([*b, z])
    return p


def find_through(arm_id, phase, target, seed, n=6000, half=0.5):
    """A straight joint move whose middle hits `target` while both ends are clear of all."""
    rng = np.random.default_rng(seed)
    lim = RIG.arm(arm_id).limits
    Q = rng.uniform(lim.q_min + 0.2, lim.q_max - 0.2, size=(n, 7))
    k = kernel_classes(arm_id, phase, Q)
    others = [c for c in CLASSES if c != target]
    mid_ok = (k[target] < -0.01) & np.all([k[c] > 0.005 for c in others], axis=0)
    for mid in Q[mid_ok][:40]:
        d = rng.normal(size=(200, 7))
        d *= half / np.linalg.norm(d, axis=1, keepdims=True)
        A, B = mid + d, mid - d
        inside = np.all((A > lim.q_min) & (A < lim.q_max) & (B > lim.q_min) & (B < lim.q_max), 1)
        A, B = A[inside], B[inside]
        if not len(A):
            continue
        ka, kb = kernel_classes(arm_id, phase, A), kernel_classes(arm_id, phase, B)
        ok = np.all([np.minimum(ka[c], kb[c]) > 0.01 for c in CLASSES], axis=0)
        if ok.any():
            i = int(np.argmax(ok))
            return np.array([A[i], mid, B[i]])
    pytest.fail(f"no test motion through {target} found for arm {arm_id}")


# =========================================================================== independence


def test_checker_imports_nothing_from_the_planners():
    bad = []
    for f in sorted((DEPLOY / "aris" / "check").glob("*.py")):
        for node in ast.walk(ast.parse(f.read_text())):
            names = []
            if isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            for n in names:
                if n.split(".")[0] == "aris" and not (n == "aris.types" or
                                                      n.startswith("aris.check")):
                    bad.append(f"{f.name}: {n}")
                if n.split(".")[0] in ("aris_sixarm", "pydrake"):
                    bad.append(f"{f.name}: {n}")
    assert not bad, bad


def test_capsule_table_matches_the_planners():
    """The checker keeps its own copy of the capsules (aris/check/fr3.json).  This fails, and
    lists every difference, when that copy and aris/kernel/fr3.py + tool.py drift apart."""
    mine = load_model()
    tool = default_tool()
    rows = [(n, f, a, b, r) for n, f, a, b, r in kfr3.LINK_CAPSULES + kfr3.HAND_CAPSULES]
    rows += [(c.name, 9, c.p0, c.p1, c.radius) for c in tool.capsules_hand]
    diffs = []
    if tuple(r[0] for r in rows) != mine.names:
        diffs.append(f"names: planner {[r[0] for r in rows]} checker {list(mine.names)}")
    else:
        for k, (n, f, a, b, r) in enumerate(rows):
            if (8 if f == 9 else f) != mine.cap_frame[k]:
                diffs.append(f"{n}: frame {f} vs {mine.cap_frame[k]}")
            for what, x, y in (("a", a, mine.cap_a[k]), ("b", b, mine.cap_b[k])):
                if np.abs(np.asarray(x, float) - y).max() > 1e-12:
                    diffs.append(f"{n}: end {what} {x} vs {y}")
            if r != mine.radius[k]:
                diffs.append(f"{n}: radius {r} vs {mine.radius[k]}")
    for what, x, y in (("tip", tool.tip_hand, mine.tip_hand),
                       ("q_min", kfr3.Q_MIN, mine.q_min), ("q_max", kfr3.Q_MAX, mine.q_max),
                       ("qd_max", kfr3.QD_MAX, mine.qd_max),
                       ("qdd_max", kfr3.QDD_MAX, mine.qdd_max),
                       ("qddd_max", kfr3.QDDD_MAX, mine.qddd_max)):
        if np.abs(np.asarray(x) - y).max() > 1e-15:
            diffs.append(f"{what}: {x} vs {y}")
    if tuple(tool.pen_names) != tuple(np.array(mine.names)[mine.is_pen]):
        diffs.append("pen capsules differ")
    arm = RIG.arm(13)
    if {tuple(p) for p in arm.self_pairs} != {tuple(p) for p in mine.self_pairs}:
        diffs.append("self-collision pairs differ")
    kb = arm.body(np.zeros((1, 7)))
    if not np.array_equal(kb.is_fixed, mine.is_fixed):
        diffs.append("fixed capsules differ")
    if kb.is_tool is not None and not np.array_equal(kb.is_tool, mine.is_tool):
        diffs.append(f"tool capsules differ: planner {np.array(kb.names)[kb.is_tool]} "
                     f"checker {np.array(mine.names)[mine.is_tool]}")
    assert not diffs, "\n".join(diffs)


def _drake_fr3():
    """An FR3 plant from the URDF in assets (arm 13's chain of the installation), meshes left out."""
    from pydrake.multibody.parsing import Parser
    from pydrake.multibody.plant import MultibodyPlant
    root = ET.parse(REPO / "assets" / "system_model" / "installation.urdf").getroot()
    robot = ET.Element("robot", name="fr3")
    keep = [f"arm13_panda_link{i}" for i in range(9)] + ["arm13_panda_hand"]
    for el in root:
        if el.tag == "link" and el.get("name") in keep:
            link = ET.SubElement(robot, "link", name=el.get("name"))
            if el.find("inertial") is not None:
                link.append(el.find("inertial"))
        if el.tag == "joint" and el.find("parent").get("link") in keep \
                and el.find("child").get("link") in keep:
            robot.append(el)
    plant = MultibodyPlant(0.0)
    Parser(plant).AddModelsFromString(ET.tostring(robot, encoding="unicode"), "urdf")
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("arm13_panda_link0"))
    plant.Finalize()
    return plant


def test_kinematics_match_drake():
    plant = _drake_fr3()
    ctx = plant.CreateDefaultContext()
    names = [f"arm13_panda_link{i}" for i in range(8)] + ["arm13_panda_hand"]
    bodies = [plant.GetBodyByName(n) for n in names]
    m = load_model()
    rng = np.random.default_rng(5)
    Q = rng.uniform(m.q_min, m.q_max, size=(1000, 7))
    R, p = frames(m, Q, np.eye(4))
    worst = 0.0
    for n, q in enumerate(Q):
        plant.SetPositions(ctx, q)
        for f, b in enumerate(bodies):
            X = plant.EvalBodyPoseInWorld(ctx, b)
            worst = max(worst, np.abs(X.translation() - p[n, f]).max(),
                        np.abs(X.rotation().matrix() - R[n, f]).max())
    print(f"\nchecker kinematics vs Drake (FR3 URDF from assets), 1000 configurations, "
          f"9 frames: worst {worst:.2e}")
    assert worst < 1e-9


@pytest.mark.slow
def test_distances_match_brute_force():
    """The checker's geometry against dense sampling, on random and touching cases."""
    rng = np.random.default_rng(3)
    P = 600
    a0, a1, b0, b1 = (rng.normal(size=(P, 3)) * 0.2 for _ in range(4))
    b1[:100] = b0[:100] + (a1[:100] - a0[:100]) * 0.7              # parallel
    b1[100:150] = b0[100:150]                                         # point
    u = np.linspace(0, 1, 401)
    A = a0[:, None] + u[None, :, None] * (a1 - a0)[:, None]
    B = b0[:, None] + u[None, :, None] * (b1 - b0)[:, None]
    brute = np.linalg.norm(A[:, :, None] - B[:, None, :], axis=-1).min((1, 2))
    mine = cgeo.segment_segment(a0, a1, b0, b1)
    assert np.all(mine <= brute + 1e-12)
    assert np.all(brute - mine < 2e-3 * np.linalg.norm(a1 - a0, axis=1).clip(0.05) + 1e-9)
    lo = rng.normal(size=(P, 3)) * 0.1
    hi = lo + rng.uniform(0.01, 0.3, size=(P, 3))
    pts = A.reshape(-1, 3)
    g = np.maximum(np.maximum(np.repeat(lo, 401, 0) - pts, pts - np.repeat(hi, 401, 0)), 0)
    brute_box = np.linalg.norm(g, axis=1).reshape(P, 401).min(1)
    mine_box = cgeo.segment_box(a0, a1, lo, hi)
    assert np.all(mine_box <= brute_box + 1e-12)
    assert np.all(brute_box - mine_box < 2e-3)


# =========================================================================== agreement


@pytest.mark.slow
@pytest.mark.parametrize("arm_id", [13, 17, 31, 71, 2, 97])
def test_agrees_with_the_planners(arm_id):
    """Capsule end points to 1e-9 m, clearance to every obstacle class to 1e-6 m."""
    rng = np.random.default_rng(100 + arm_id)
    m = load_model()
    Q = rng.uniform(m.q_min, m.q_max, size=(N_AGREE, 7))
    ph = phase_of(arm_id)
    T = RIG.T_table_base(arm_id)
    kb = RIG.arm(arm_id).body(Q)
    A, B = capsules(m, Q, MINE.mounts[arm_id].T_table_base)
    to_table = lambda p: p @ T[:3, :3].T + T[:3, 3]
    ends = max(np.abs(A - to_table(kb.p0)).max(), np.abs(B - to_table(kb.p1)).max())
    k, me = kernel_classes(arm_id, ph, Q), my_classes(arm_id, ph, Q)
    worst = {}
    for c in CLASSES:
        both = np.isfinite(k[c]) | np.isfinite(me[c])
        assert np.array_equal(np.isfinite(k[c]), np.isfinite(me[c])), c
        worst[c] = float(np.abs(k[c][both] - me[c][both]).max()) if both.any() else 0.0
    print(f"\narm {arm_id} ({ph.name}), {N_AGREE} random configurations: capsule ends "
          f"{ends:.1e} m; " + ", ".join(f"{c} {v:.1e}" for c, v in worst.items()))
    assert ends < 1e-9
    assert max(worst.values()) < 1e-6, worst


def test_link1_exempt_from_its_own_hanger_only():
    """rig.json hanger.exempt_links: an arm's link 1 is not checked against its own struts,
    plate and clamp (a rig test settles that once); it is against every other box."""
    from dataclasses import replace as dc_replace
    scene = build_scene(MINE, 71, (), (), False)
    q = MINE.mounts[71].park_q[None]
    strict = clearance(dc_replace(scene, own_exempt=()), q)
    rule = clearance(scene, q)
    print(f"\narm 71 at park, steel: without the exemption {strict.value['steel'][0] * 1e3:.2f} mm "
          f"({strict.closest('steel', 0)}), with it {rule.value['steel'][0] * 1e3:.2f} mm "
          f"({rule.closest('steel', 0)})")
    assert MINE.own_exempt == ("link1",)
    assert strict.closest("steel", 0).startswith("link1") and "71" in strict.closest("steel", 0)
    assert not (rule.closest("steel", 0).startswith("link1") and "71" in rule.closest("steel", 0))
    other = dc_replace(scene, box_own=np.zeros_like(scene.box_own))       # nobody's own hanger
    assert clearance(other, q).value["steel"][0] == strict.value["steel"][0]


# =========================================================================== faults


def _fails(v, name):
    print(f"\n{v}")
    assert not v.passed
    assert name in v.failed, f"expected {name} to fail, failed: {v.failed}"


@pytest.mark.slow
def test_fault_path_through_a_strut():
    ph = phase_of(31)
    path = find_through(31, ph, "steel", seed=1)
    v = check(CONFIG, 31, free_motion(31, path), ph, path[0])
    _fails(v, "clearance steel")


@pytest.mark.slow
def test_fault_path_crosses_a_wall():
    ph = phase_of(31)
    path = find_through(31, ph, "walls", seed=2)
    _fails(check(CONFIG, 31, free_motion(31, path), ph, path[0]), "clearance walls")


@pytest.mark.slow
def test_fault_path_touches_a_parked_neighbour():
    ph = phase_of(31)
    path = find_through(31, ph, "parked", seed=3)
    _fails(check(CONFIG, 31, free_motion(31, path), ph, path[0]), "clearance parked arms")


@pytest.mark.slow
def test_fault_self_collision():
    ph = phase_of(13)
    path = find_through(13, ph, "self", seed=5)
    _fails(check(CONFIG, 13, free_motion(13, path), ph, path[0]), "clearance self")


def test_fault_pen_dips_into_the_paper_when_lifted():
    u = np.linspace(0, 1, 121)
    xy = (1 - u)[:, None] * np.array([-0.60, 0.10]) + u[:, None] * np.array([-0.55, 0.30])
    z = 0.030 - 0.031 * (1 - np.abs(2 * u - 1))                    # down to -1 mm, back up
    Q, _ = ik_path(31, np.column_stack([xy, z]))
    v = check(CONFIG, 31, free_motion(31, Q), phase_of(31), Q[0])
    _fails(v, "clearance paper (pen)")
    assert -0.0017 < v.get("clearance paper (pen)").value < -0.0009


def test_fault_corner_reads_differently_at_4_khz():
    """A joint-space corner flown through at speed, handed over at 48 Hz with the velocity of
    each leg: the acceleration jumps, so the finite differences grow with the rate (L44)."""
    q0 = RIG.park_q(31)
    a, b = np.zeros(7), np.zeros(7)
    a[6], b[5] = 0.15, 0.15
    t = np.arange(0, 4.0 + 1e-9, 1 / 48)
    u = 0.5 - 0.5 * np.cos(np.pi * t / 4.0)                        # 0..1, at rest at both ends
    ud = 0.5 * np.pi / 4.0 * np.sin(np.pi * t / 4.0)
    q = q0 + np.where(u[:, None] < 0.5, 2 * u[:, None] * a, a + (2 * u[:, None] - 1) * b)
    qd = np.where(u[:, None] < 0.5, 2 * ud[:, None] * a, 2 * ud[:, None] * b)
    v = check(CONFIG, 31, Motion("free", Trajectory(t, q, qd)), phase_of(31), q0)
    _fails(v, "1 kHz vs 4 kHz")


def _joint7_swing(peak):
    """Joint 7 from -2.4 to about +2.5 rad at a peak speed of `peak` rad/s, smooth."""
    ramp = peak * np.pi / (2 * 9.0)                                # acceleration 9 rad/s^2
    t = np.linspace(0, 2 * ramp, 4001)
    v = np.where(t < ramp, 0.5 * peak * (1 - np.cos(np.pi * t / ramp)),
                 0.5 * peak * (1 - np.cos(np.pi * (2 * ramp - t) / ramp)))
    x = np.concatenate([[0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(t))])
    q = np.tile(RIG.park_q(13), (len(t), 1))
    qd = np.zeros_like(q)
    q[:, 6], qd[:, 6] = -2.4 + x, v
    return Trajectory(t, q, qd)


def test_fault_velocity_one_percent_over():
    lim = RIG.arm(13).limits.qd_max[6]
    over = check(CONFIG, 13, Motion("free", _joint7_swing(1.01 * lim)), phase_of(13))
    _fails(over, "velocity at 1 kHz")
    assert 1.005 < over.get("velocity at 1 kHz").value < 1.011
    under = check(CONFIG, 13, Motion("free", _joint7_swing(0.99 * lim)), phase_of(13))
    assert under.get("velocity at 1 kHz").passed


@pytest.fixture(scope="module")
def good_draw():
    return draw_motion(31, line_table((-0.62, 0.05), (-0.50, 0.30)))


PAPER_ROWS = ("clearance paper (links)", "clearance paper (tool)", "clearance paper (pen)")


@pytest.mark.parametrize("lean", [(0.0, 0.0), (0.0, 0.26)],
                         ids=["upright", "leaned 15 deg, worst direction"])
def test_a_straight_line_draws(lean):
    """A straight line drawn on the paper passes everything, hand square and leaned 15 deg."""
    m = draw_motion(31, line_table((-0.62, 0.05), (-0.50, 0.30)), lean=lean)
    v = check(CONFIG, 31, m, phase_of(31), m.q_start)
    print(f"\n{v}")
    assert v.passed, v.failed
    assert "clearance paper (pen)" not in [x.name for x in v.measurements]
    for name in PAPER_ROWS[:2]:
        print(f"{name}: {v.get(name).value * 1e3:.2f} mm (limit {v.get(name).limit * 1e3:.1f})")


def test_fault_drawing_leaves_the_line(good_draw):
    tb = good_draw.tip_base
    d = tb[-1] - tb[0]
    side = np.cross(d, RIG.paper(31).normal)
    side = side / np.linalg.norm(side)
    moved = Motion("draw", good_draw.traj, good_draw.piece, tb + 0.0005 * side)
    v = check(CONFIG, 31, moved, phase_of(31), good_draw.q_start)
    _fails(v, "tip on line")
    assert 0.00045 < v.get("tip on line").value < 0.00055


def test_fault_drawing_stops_halfway():
    a, b = draw_motion(31, line_table((-0.62, 0.05), (-0.50, 0.30)), split=60)
    assert np.abs(a.q_end - b.q_start).max() < 1e-9
    t = np.concatenate([a.traj.t, a.traj.t[-1] + b.traj.t[1:]])
    traj = Trajectory(t, np.vstack([a.traj.q, b.traj.q[1:]]), np.vstack([a.traj.qd,
                                                                           b.traj.qd[1:]]))
    both = Motion("draw", traj, None, np.vstack([a.tip_base, b.tip_base[1:]]))
    _fails(check(CONFIG, 31, both, phase_of(31), a.q_start), "never stops")


def _nearest_progress(m):
    """The old reading: arc length of the nearest point of the planned line (for comparison)."""
    from aris.check.drawing import _project
    tr = m.traj
    t = np.arange(tr.t[0], tr.t[-1], 1e-3)
    x = RIG.arm(31).tip(sample(tr, t)[0])
    P = m.tip_base
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s_at = np.concatenate([[0.0], np.cumsum(seg)])
    i = np.clip(np.searchsorted(tr.t, t, side="right") - 1, 0, len(P) - 2)
    idx = np.clip(i[:, None] + np.arange(-2, 4), 0, len(P) - 2)
    dist, u = _project(x, P[idx], P[idx + 1])
    b = np.argmin(dist, axis=1)
    r = np.arange(len(t))
    return s_at[idx[r, b]] + u[r, b] * seg[idx[r, b]]


def test_drawing_round_a_sharp_corner():
    """A V turning 150 degrees: progress along the line never stops or goes back.  (How far
    the nearest point of the line falls back there is printed, for comparison.)"""
    a, v, b = np.array([-0.62, 0.05]), np.array([-0.52, 0.15]), None
    turn = np.deg2rad(150.0)
    d0 = (v - a) / np.linalg.norm(v - a)
    c, s_ = np.cos(turn), np.sin(turn)
    d1 = np.array([c * d0[0] - s_ * d0[1], s_ * d0[0] + c * d0[1]])
    b = v + 0.12 * d1
    pts = np.vstack([line_table(a, v, 80)[:-1], line_table(v, b, 80)])
    m = draw_motion(31, pts)
    v_ = check(CONFIG, 31, m, phase_of(31), m.q_start)
    near = _nearest_progress(m)
    dip = float(np.max(np.maximum.accumulate(near) - near))
    print(f"\n{v_}\nnearest point on the line falls back by {dip * 1e3:.3f} mm at the corner")
    for name in ("never stops", "never backwards", "tip on line", "tip speed"):
        assert v_.get(name).passed, name


def _config_with_draw_speed(tmp_path, speed):
    """A copy of config/ whose rig.json draws at `speed`."""
    import json
    import shutil
    dst = tmp_path / "config"
    shutil.copytree(CONFIG, dst)
    cfg = json.loads((dst / "rig.json").read_text())
    cfg["drawing"]["draw_speed_m_per_s"] = speed
    (dst / "rig.json").write_text(json.dumps(cfg))
    return dst


def test_draw_speed_comes_from_rig_json(good_draw, tmp_path):
    """The checker takes the drawing speed from rig.json, like the planners: the same motion
    (drawn at 20 mm/s) passes at 20 mm/s and fails 'tip speed' when rig.json says 10 mm/s."""
    assert read_rig(CONFIG).draw_speed == RIG.rules().draw_speed      # same number as planners
    ok = check(CONFIG, 31, good_draw, phase_of(31), good_draw.q_start)
    slow = check(_config_with_draw_speed(tmp_path, 0.010), 31, good_draw, phase_of(31),
                 good_draw.q_start)
    print(f"\ntip speed limit {ok.get('tip speed').limit * 1e3:.1f} mm/s, then "
          f"{slow.get('tip speed').limit * 1e3:.1f} mm/s")
    assert ok.get("tip speed").passed and abs(ok.get("tip speed").limit - 0.0206) < 1e-12
    _fails(slow, "tip speed")
    assert abs(slow.get("tip speed").limit - 0.0103) < 1e-12


def test_sharp_corner_at_80_mm_per_s_slows_but_does_not_stop(tmp_path):
    """The sequencer's word at 80 mm/s (tests/data/arm_stop_31_word_0.npz): at a corner that
    turns 130 + 18 degrees the pen's real speed (not a reading of the line) drops to about
    1 mm/s and picks up again; it never stops.  A real stop reads ~1e-7 m/s."""
    d = np.load(DEPLOY / "tests" / "data" / "arm_stop_31_word_0.npz")
    m = Motion(str(d["kind"]), Trajectory(d["t"], d["q"], d["qd"]),
               Piece(str(d["line_id"]), float(d["s0"]), float(d["s1"])), d["tip_base"])
    v = check(_config_with_draw_speed(tmp_path, 0.08), int(d["arm_id"]), m,
              phase_of(int(d["arm_id"])), d["q_before"])
    print(f"\n{v}")
    stop = v.get("never stops")
    assert 5e-4 < stop.value < 2e-3 and stop.passed
    t = np.arange(d["t"][0], d["t"][-1], 1e-4)
    tips = RIG.arm(31).tip(sample(m.traj, t)[0])
    speed = np.linalg.norm(np.diff(tips, axis=0), axis=1) / 1e-4
    k = int(np.argmin(np.abs(t[:-1] - 2.1516)))
    print(f"real tip speed at the corner (10 kHz): {speed[k - 10:k + 10].min() * 1e3:.2f} mm/s")
    assert speed[k - 10:k + 10].min() < 1.5e-3


def test_drawing_speed_allowance(good_draw):
    """The pen may run up to 3 % over the drawing speed."""
    tr = good_draw.traj
    for factor, ok in ((1.015, True), (1.04, False)):
        fast = Trajectory(tr.t / factor, tr.q, tr.qd * factor)
        v = check(CONFIG, 31, Motion("draw", fast, good_draw.piece, good_draw.tip_base),
                  phase_of(31), good_draw.q_start)
        print(f"\n{factor}: tip speed {v.get('tip speed').value * 1e3:.3f} mm/s, "
              f"limit {v.get('tip speed').limit * 1e3:.3f}")
        assert v.get("tip speed").passed == ok


def _down(depth, n=60):
    """Tip straight down from 10 mm above the paper to `depth` (negative: into it)."""
    z = np.linspace(0.010, depth, n)
    Q, _ = ik_path(31, np.column_stack([np.full(n, -0.56), np.full(n, 0.17), z]))
    return free_motion(31, Q).traj


def test_lower_and_lift():
    ph = phase_of(31)
    tr = _down(0.0)
    lower = check(CONFIG, 31, Motion("lower", tr), ph, tr.q[0])
    print(f"\n{lower}")
    assert lower.passed, lower.failed
    depth = lower.get("pen depth (lower, lift)")
    assert -0.0016 < depth.value < 0.0 and depth.limit == -0.002
    back = Trajectory(tr.t[-1] - tr.t[::-1], tr.q[::-1], -tr.qd[::-1])
    assert check(CONFIG, 31, Motion("lift", back), ph, back.q[0]).passed
    _fails(check(CONFIG, 31, Motion("free", tr), ph, tr.q[0]), "clearance paper (pen)")
    deep = _down(-0.003)
    _fails(check(CONFIG, 31, Motion("lower", deep), ph, deep.q[0]), "pen depth (lower, lift)")


@pytest.fixture(scope="module")
def good_free():
    q0 = RIG.park_q(31)
    return free_motion(31, [q0, q0 + np.array([-0.4, -0.1, 0.1, 0.3, 0.2, -0.2, 0.5])])


def test_fault_empty_and_still_motions(good_free):
    one = Trajectory(good_free.traj.t[:1], good_free.traj.q[:1], good_free.traj.qd[:1])
    _fails(check(CONFIG, 31, Motion("free", one), phase_of(31)), "well formed")
    q = np.tile(good_free.q_start, (2, 1))
    still = Trajectory(np.array([0.0, 1.0]), q, np.zeros((2, 7)))
    _fails(check(CONFIG, 31, Motion("free", still), phase_of(31)), "moves")


def test_fault_start_does_not_match(good_free):
    q_before = good_free.q_start.copy()
    q_before[2] += 1e-4
    _fails(check(CONFIG, 31, good_free, phase_of(31), q_before), "starts at q_before")


def test_fault_end_not_at_rest(good_free):
    tr = good_free.traj
    n = len(tr.t) // 2
    cut = Motion("free", Trajectory(tr.t[:n], tr.q[:n], tr.qd[:n]))
    _fails(check(CONFIG, 31, cut, phase_of(31), tr.q[0]), "at rest at end")


def test_fault_arm_not_active_in_the_phase(good_free):
    _fails(check(CONFIG, 31, good_free, RIG.phase(1), good_free.q_start), "well formed")


# =========================================================================== good motions


GOOD = {
    31: np.array([-0.4, -0.1, 0.1, 0.3, 0.2, -0.2, 0.5]),
    13: np.array([0.3, 0.1, -0.2, 0.2, -0.3, 0.2, -0.6]),
    97: np.array([-0.3, 0.1, 0.2, -0.2, 0.2, -0.1, 0.8]),
}


@pytest.mark.parametrize("arm_id", sorted(GOOD))
def test_good_free_motion_passes(arm_id):
    q0 = RIG.park_q(arm_id)
    m = free_motion(arm_id, [q0, q0 + GOOD[arm_id], q0])
    v = check(CONFIG, arm_id, m, phase_of(arm_id), q0)
    print(f"\n{v}")
    assert v.passed


# =========================================================================== sampling rate


def test_same_verdict_at_any_sampling(good_free):
    q0 = good_free.q_start
    out = {}
    for name, tr in (("retimed", good_free.traj), ("100 Hz", resample(good_free.traj, 100)),
                     ("1 kHz", resample(good_free.traj, 1000)),
                     ("4 kHz", resample(good_free.traj, 4000))):
        v = check(CONFIG, 31, Motion("free", tr), phase_of(31), q0)
        out[name] = v
        print(f"\n{name:8} ({len(tr.t)} samples): {'PASS' if v.passed else v.failed}, "
              f"smallest clearance beyond demanded {v.min_clearance * 1e3:.3f} mm "
              f"({v.min_clearance_at})")
    ref = out["retimed"]
    for v in out.values():
        assert v.passed == ref.passed
        assert abs(v.min_clearance - ref.min_clearance) < 5e-4


@pytest.mark.slow
def test_clearance_is_a_true_bound(good_free):
    """The reported clearance never exceeds the truth measured by the planners' kernel on a
    20 kHz sampling of the same flown curve, and lies within the tolerance (0.25 mm) of it."""
    tr = good_free.traj
    v = check(CONFIG, 31, good_free, phase_of(31), good_free.q_start)
    t = np.linspace(tr.t[0], tr.t[-1], int(tr.t[-1] * 20000) + 1)
    k = kernel_classes(31, phase_of(31), sample(tr, t)[0])
    truth = min(float(k[c].min()) for c in CLASSES)
    print(f"\nreported {v.min_clearance * 1e3:.4f} mm, kernel at 20 kHz {truth * 1e3:.4f} mm")
    assert v.min_clearance <= truth + 1e-9
    assert v.min_clearance >= truth - 2.5e-4


@pytest.mark.slow
def test_same_verdict_with_twice_the_samples():
    """A motion that fails, handed over again with a sample inserted between every two."""
    ph = phase_of(31)
    path = find_through(31, ph, "steel", seed=1)
    m = free_motion(31, path)
    tr = m.traj
    t2 = np.sort(np.concatenate([tr.t, 0.5 * (tr.t[1:] + tr.t[:-1])]))
    q2, qd2, _ = sample(tr, t2)
    a = check(CONFIG, 31, m, ph, path[0])
    b = check(CONFIG, 31, Motion("free", Trajectory(t2, q2, qd2)), ph, path[0])
    print(f"\n{len(tr.t)} samples: {a.min_clearance * 1e3:.3f} mm; {len(t2)} samples: "
          f"{b.min_clearance * 1e3:.3f} mm")
    assert a.passed == b.passed and a.failed == b.failed
    assert abs(a.min_clearance - b.min_clearance) < 5e-4


# =========================================================================== the old plan


@pytest.mark.slow
def test_old_plan_l44():
    """The old hover schedule of arm 71: kinematics agree with the old code on every frame; the
    old checker passed it, this one refuses it for its acceleration (lesson L44)."""
    ref = np.load(DEPLOY / "tests" / "data" / "check_old_plan.npz")
    m = load_model()
    tip_table = np.asarray(ref["tip_world"]) - ref["shift"]
    R, p = frames(m, ref["q"], MINE.mounts[71].T_table_base)
    mine = p[:, 8] + R[:, 8] @ m.tip_hand
    err = np.abs(mine - tip_table).max()
    t, q = ref["t"], ref["q"]
    qd = np.gradient(q, t, axis=0)
    qd[0] = qd[-1] = 0.0
    v = check(CONFIG, 71, Motion("free", Trajectory(t, q, qd)),
              Phase("old hover run", (71,), (), ()), q[0])
    print(f"\npen tip vs old code over {len(q)} frames: {err:.1e} m")
    print(f"old reading at 48 Hz: {ref['old_acc_48'].max():.1f} rad/s^2, linear to 1 kHz "
          f"{ref['old_acc_1k'].max():.0f}; this checker: "
          f"{v.get('acceleration at 1 kHz').detail}")
    print(v)
    assert err < 1e-9
    assert not v.passed and "acceleration at 1 kHz" in v.failed


# =========================================================================== speed


def _timed(arm_id, m, q0, reps=3):
    """Best of `reps`, in CPU time of this process (the machine is shared)."""
    best = np.inf
    for _ in range(reps):
        t0 = time.process_time()
        v = check(CONFIG, arm_id, m, phase_of(arm_id), q0)
        best = min(best, time.process_time() - t0)
    return best, v


@pytest.mark.slow
def test_speed():
    """Measured 2026-09-29 (CPU time, one core): 0.13 s for the 7 s free motion and 0.53 s
    for the 60 s drawing motion.  The ceilings are about ten times that."""
    q0 = RIG.park_q(31)
    d = GOOD[31]
    free = free_motion(31, [q0, q0 + d, q0 - 0.3 * d, q0 + d, q0], speed_fraction=0.12)
    a = np.linspace(0, 3.2 * np.pi, 960)                           # 1.6 turns, 1.2 m of line
    circle = np.column_stack([-0.55 + 0.12 * np.cos(a), 0.20 + 0.12 * np.sin(a), 0 * a])
    draw = draw_motion(31, circle)
    rows = []
    for name, m in (("free", free), ("draw", draw)):
        sec, v = _timed(31, m, m.q_start)
        rows.append(sec)
        print(f"\n{name}: {m.traj.t[-1]:.1f} s motion, {len(m.traj.t)} samples handed over, "
              f"checked in {sec * 1e3:.0f} ms CPU ({v.min_clearance_at}); "
              f"{'PASS' if v.passed else v.failed}")
    assert rows[0] < 1.5 and rows[1] < 6.0


# =========================================================================== phase end


def _at_park(phase, moved):
    """The active arms of the phase at park, except those in `moved`."""
    return {a: moved.get(a, RIG.park_q(a)) for a in RIG.phase(phase).active}


def test_phase_end_everyone_at_park_passes():
    ph = RIG.phase(1)
    v = check_phase_end(CONFIG, ph, {a: RIG.park_q(a) for a in ph.active})
    print(f"\n{v}")
    assert v.passed
    assert len([m for m in v.measurements if m.name.startswith("arms ")]) == 15


def test_phase_end_pairs_agree_with_the_planners():
    """Arm 71 at random configurations, 31 at park: the pair clearance equals the kernel's
    clearance of 71's whole body (base included, which a moving arm's check leaves out)
    against parked 31."""
    rng = np.random.default_rng(9)
    Q = rng.uniform(MINE_MODEL.q_min, MINE_MODEL.q_max, size=(40, 7))
    body = RIG.arm(71).body(Q)
    body = replace(body, is_fixed=np.zeros_like(body.is_fixed))
    parked = RIG.obstacles(71, parked=(31,), for_planning=False).capsules
    k = collide.clearance(body, Obstacles(capsules=parked), prune=False, backend=KB)
    worst = 0.0
    for q, kv in zip(Q, k):
        v = check_phase_end(CONFIG, RIG.phase(1), _at_park(1, {71: q}))
        m = v.get("arms 31 and 71")
        worst = max(worst, abs((m.value - m.limit) - kv))
    print(f"\npair clearance vs kernel, 40 configurations: {worst:.1e} m")
    assert worst < 1e-9


def test_phase_end_catches_a_touching_pair_and_a_missing_arm():
    ph = Phase("pair", (71,), (31,), ())
    rng = np.random.default_rng(10)
    Q = rng.uniform(MINE_MODEL.q_min, MINE_MODEL.q_max, size=(3000, 7))
    q = Q[np.argmax(kernel_classes(71, ph, Q)["parked"] < -0.01)]
    v = check_phase_end(CONFIG, RIG.phase(1), _at_park(1, {71: q}))
    _fails(v, "arms 31 and 71")
    _fails(check_phase_end(CONFIG, RIG.phase(1), {13: RIG.park_q(13)}), "well formed")


# =========================================================================== footprints


def _park_footprint_in_31():
    """Arm 71 standing at park, as a footprint (a distance field of 2 cm cells) in arm 31's
    base frame, as the system planner would hand it to arm 31."""
    from aris.kernel.footprint import footprint, transform_field
    q = RIG.park_q(71)
    still = Trajectory(np.array([0.0, 1.0]), np.stack([q, q]), np.zeros((2, 7)))
    f71 = footprint(RIG.arm(71), [still], cell=0.02, name="footprint71")
    return transform_field(f71, RIG.T_base_table(31) @ RIG.T_table_base(71), "footprint71")


def test_field_readings_never_exceed_the_exact_distance():
    """Arm 31 against arm 71 at park, 3000 random configurations: the checker's reading and the
    kernel's reading of the footprint are both lower bounds on the exact distance to 71's
    capsules."""
    f = _park_footprint_in_31()
    rng = np.random.default_rng(12)
    Q = rng.uniform(MINE_MODEL.q_min, MINE_MODEL.q_max, size=(3000, 7))
    m = MINE.clearance["arm_to_arm_m"]
    mine = my_classes(31, Phase("f", (31,), (), ()), Q, fields=(f,))["fields"] + m
    exact = my_classes(31, Phase("p", (31,), (71,), ()), Q)["parked"] + m
    kern = collide.clearance(RIG.arm(31).body(Q), Obstacles(fields=(replace(f, margin=0.0),)),
                             prune=False, backend=KB)
    near = exact < 0.10
    print(f"\n{near.sum()} of {len(Q)} within 10 cm; exact minus reading, median / largest: "
          f"checker {np.median((exact - mine)[near]) * 1e3:.1f} / "
          f"{(exact - mine)[near].max() * 1e3:.1f} mm, kernel "
          f"{np.median((exact - kern)[near]) * 1e3:.1f} / {(exact - kern)[near].max() * 1e3:.1f} mm")
    assert np.all(mine <= exact + 1e-9)
    assert np.all(kern <= exact + 1e-9)
    assert near.sum() > 50


@pytest.mark.slow
def test_fault_motion_through_a_footprint():
    """The motion that touches parked arm 71, checked with 71 given only as a footprint."""
    path = find_through(31, Phase("pair", (31,), (71,), ()), "parked", seed=3)
    f = _park_footprint_in_31()
    v = check(CONFIG, 31, free_motion(31, path), Phase("f", (31,), (), ()), path[0],
              fields=(f,))
    _fails(v, "clearance footprints")
    assert v.get("clearance footprints").value < 0


def test_everything_far_away_is_pruned_without_error(good_free):
    """Fields, parked arms and boxes all present but far beyond the threshold: every class
    prunes to nothing and still gives a true (large) answer.  (A far footprint once made the
    field class stack an empty list.)"""
    from dataclasses import replace as dc_replace
    far = replace(_park_footprint_in_31(), origin_base=np.array([20.0, 20.0, 20.0]))
    v = check(CONFIG, 31, good_free, phase_of(31), good_free.q_start, fields=(far,))
    assert v.passed, v.failed
    assert v.get("clearance footprints").value > 1.0
    scene = build_scene(MINE, 31, (), (71,), False, fields=(far,))
    shift = np.array([30.0, 0.0, 0.0])
    scene = dc_replace(scene, box_lo=scene.box_lo[:1] + 40.0, box_hi=scene.box_hi[:1] + 40.0,
                       box_own=scene.box_own[:1], box_names=scene.box_names[:1],
                       other_a=scene.other_a + shift, other_b=scene.other_b + shift)
    Q = good_free.traj.q
    thr = {c: 0.01 for c in CLASSES}
    cl = clearance(scene, Q, thr)
    for c in ("steel", "parked", "fields", "self"):
        assert np.all(cl.value[c] > 0.01), c
    exact = clearance(scene, Q)
    for c in ("steel", "parked", "fields"):
        assert np.all(cl.value[c] <= exact.value[c] + 1e-12), c
