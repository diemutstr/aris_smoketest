"""Tests for aris/check, the independent checker.

Run with -s to see the measured numbers.  The tests may import the planners' kernel and rig to
compare answers; the checker itself may not (the first test enforces that).
"""
import ast
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from aris.check import check
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
REPO = DEPLOY.parent
RIG = Rig.load(CONFIG)
MINE = read_rig(CONFIG)
N_AGREE = 10_000


def phase_of(arm_id) -> Phase:
    return RIG.phase(1) if arm_id in RIG.leaders(1) else RIG.phase(2)


# =========================================================================== helpers


def kernel_classes(arm_id, phase, Q, drawing=False) -> dict:
    """The planners' answer per obstacle class (gap minus the demanded clearance)."""
    arm = RIG.arm(arm_id)
    body = arm.body(Q)
    walls = [w for w in phase.walls if arm_id in w.arms]
    obs = RIG.obstacles(arm_id, parked=phase.parked, walls=walls, for_planning=False)
    per = collide.capsule_clearance(body, Obstacles(planes=obs.planes[:1]), drawing, prune=False)
    return dict(
        steel=collide.clearance(body, Obstacles(boxes=obs.boxes), prune=False),
        paper=per[:, ~body.is_pen].min(1),
        pen=per[:, body.is_pen].min(1) if not drawing else np.full(len(Q), np.inf),
        walls=collide.clearance(body, Obstacles(planes=obs.planes[1:]), prune=False),
        parked=collide.clearance(body, Obstacles(capsules=obs.capsules), prune=False),
        self=collide.self_clearance(body, arm.self_pairs, RIG.clearance["self_m"]))


def my_classes(arm_id, phase, Q, drawing=False, chunk=500) -> dict:
    scene = build_scene(MINE, arm_id, phase.walls, phase.parked, drawing)
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


def draw_motion(arm_id, pts_table, split=None):
    """A drawing motion along the table-frame points.  With `split` (a sample index), the line
    is drawn as two motions that meet there, each starting and ending at rest."""
    arm = RIG.arm(arm_id)
    Q, tip_b = ik_path(arm_id, pts_table)
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
    if not np.array_equal(arm.body(np.zeros((1, 7))).is_fixed, mine.is_fixed):
        diffs.append("fixed capsules differ")
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


# =========================================================================== faults


def _fails(v, name):
    print(f"\n{v}")
    assert not v.passed
    assert name in v.failed, f"expected {name} to fail, failed: {v.failed}"


def test_fault_path_through_a_strut():
    ph = phase_of(31)
    path = find_through(31, ph, "steel", seed=1)
    v = check(CONFIG, 31, free_motion(31, path), ph, path[0])
    _fails(v, "clearance steel")


def test_fault_path_crosses_a_wall():
    ph = phase_of(31)
    path = find_through(31, ph, "walls", seed=2)
    _fails(check(CONFIG, 31, free_motion(31, path), ph, path[0]), "clearance walls")


def test_fault_path_touches_a_parked_neighbour():
    ph = phase_of(31)
    path = find_through(31, ph, "parked", seed=3)
    _fails(check(CONFIG, 31, free_motion(31, path), ph, path[0]), "clearance parked arms")


def test_fault_self_collision():
    ph = phase_of(13)
    path = find_through(13, ph, "self", seed=4)
    _fails(check(CONFIG, 13, free_motion(13, path), ph, path[0]), "clearance self")


def test_fault_pen_dips_into_the_paper_when_lifted():
    u = np.linspace(0, 1, 121)
    xy = (1 - u)[:, None] * np.array([-0.60, 0.10]) + u[:, None] * np.array([-0.55, 0.30])
    z = 0.030 - 0.031 * (1 - np.abs(2 * u - 1))                    # down to -1 mm, back up
    Q, _ = ik_path(31, np.column_stack([xy, z]))
    v = check(CONFIG, 31, free_motion(31, Q), phase_of(31), Q[0])
    _fails(v, "pen above paper")
    assert -0.0017 < v.get("pen above paper").value < -0.0009


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


def test_a_straight_line_draws(good_draw):
    """Every pen measurement passes.  The body against the paper does not: see the page."""
    v = check(CONFIG, 31, good_draw, phase_of(31), good_draw.q_start)
    print(f"\n{v}")
    for name in ("tip on paper", "tip on line", "never backwards", "never stops", "tip speed",
                 "velocity at 1 kHz", "acceleration at 1 kHz", "jerk at 1 kHz",
                 "1 kHz vs 4 kHz", "clearance steel", "clearance walls",
                 "clearance parked arms", "clearance self"):
        assert v.get(name).passed, name


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
