"""The arm model against the old code (tests/data/arm_reference.npz, written by
tests/oracle/make_arm_reference.py) and against the FR3 meshes.

    cd deployment && ../.venv/bin/python -m pytest tests/test_kernel_arm.py -q -s
(-s prints the measured numbers.)
"""
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from aris.kernel import fr3
from aris.kernel.arm import Arm, default_tool

DATA = Path(__file__).parent / "data" / "arm_reference.npz"
TOL = 1e-9


@pytest.fixture(scope="module")
def arm():
    return Arm(default_tool())


@pytest.fixture(scope="module")
def ref():
    return np.load(DATA)


def random_q(arm, n, seed):
    L = arm.limits
    return L.q_min + np.random.default_rng(seed).random((n, 7)) * (L.q_max - L.q_min)


def pt_seg(P, a, b):
    """Distance from points P (..., 3) to segments a-b (..., 3)."""
    ab = b - a
    t = np.clip(np.sum((P - a) * ab, -1) / np.maximum(np.sum(ab * ab, -1), 1e-18), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[..., None] * ab), axis=-1)


def seg_seg(p0, p1, q0, q1):
    """Distance between segments, broadcasting (the old selfcoll formula, restated)."""
    d1, d2, r = p1 - p0, q1 - q0, p0 - q0
    a, e, b = np.sum(d1 * d1, -1), np.sum(d2 * d2, -1), np.sum(d1 * d2, -1)
    c, f = np.sum(d1 * r, -1), np.sum(d2 * r, -1)
    den = a * e - b * b
    inside = den > 1e-12
    s = np.where(inside, (b * f - c * e) / np.where(inside, den, 1.0), -1.0)
    t = np.where(inside, (a * f - b * c) / np.where(inside, den, 1.0), -1.0)
    good = inside & (s >= 0) & (s <= 1) & (t >= 0) & (t <= 1)
    w = r + s[..., None] * d1 - t[..., None] * d2
    mid = np.where(good, np.linalg.norm(w, axis=-1), np.inf)
    edge = np.minimum(np.minimum(pt_seg(p0, q0, q1), pt_seg(p1, q0, q1)),
                      np.minimum(pt_seg(q0, p0, p1), pt_seg(q1, p0, p1)))
    return np.minimum(mid, edge)


# ------------------------------------------------------------------ 1. forward kinematics

def test_fk_matches_old(arm, ref):
    q = ref["fk_q"]
    e_hand = np.abs(arm.fk(q) - ref["fk_hand"]).max()
    e_tip = np.abs(arm.tip(q) - ref["fk_tip"]).max()
    e_links = np.abs(arm.link_frames(q[:1000]) - ref["fk_links"]).max()
    T = arm.fk(q[:1000])
    tcp = T[:, :3, 3] + fr3.D_HAND_TCP * T[:, :3, 2]
    e_tcp = np.abs(tcp - ref["fk_tcp"][:, :3, 3]).max()
    print(f"\nFK vs old on {len(q)} configs: hand {e_hand:.1e}, tip {e_tip:.1e}, "
          f"all link frames {e_links:.1e}, TCP {e_tcp:.1e}")
    assert max(e_hand, e_tip, e_links, e_tcp) < TOL


def test_pen_axis(arm):
    q = random_q(arm, 200, 1)
    ax = arm.pen_axis(q)
    assert np.allclose(np.linalg.norm(ax, axis=1), 1.0)
    # the pen leans 23 deg off the hand's approach axis
    cosang = np.sum(ax * arm.fk(q)[:, :3, 2], axis=1)
    assert np.allclose(cosang, np.cos(np.deg2rad(23.0)))
    # and runs through the tip: the tip moves along the axis when the pen is longer
    tip = arm.tip(q)
    grip = arm.fk(q)[:, :3, :3] @ np.array([0.0665, 0.0, 0.1034]) + arm.fk(q)[:, :3, 3]
    along = np.cross(tip - grip, ax)
    assert np.abs(along).max() < 1e-6


# ------------------------------------------------------------------ 2. inverse kinematics

def _check_solutions(arm, T, Q, valid):
    q = Q[valid]
    Tf = arm.fk(q)
    Tr = np.broadcast_to(T[:, None], Q.shape[:2] + (4, 4))[valid]
    e_pos = np.linalg.norm(Tf[:, :3, 3] - Tr[:, :3, 3], axis=1)
    e_rot = np.linalg.norm(Tf[:, :3, :3] - Tr[:, :3, :3], axis=(1, 2))
    assert np.all(e_pos <= TOL) and np.all(e_rot <= TOL)
    assert np.all(arm.limit_margin(q) >= 0.0)
    assert np.all(np.isnan(Q[~valid]))
    return (e_pos.max() if len(q) else 0.0), (e_rot.max() if len(q) else 0.0)


def test_ik_reachable_matches_old(arm, ref):
    T, q = ref["ik_hand"], ref["ik_q"]
    Q, valid = arm.ik(T, q[:, 6])
    assert Q.shape == (len(T), 4, 7) and valid.shape == (len(T), 4)
    ep, er = _check_solutions(arm, T, Q, valid)
    assert np.array_equal(valid.sum(1), ref["ik_count"])
    # the same solutions, in the same solver order (the old code packed them to the front)
    old = ref["ik_Q"]
    for k in range(len(old)):
        n = valid[k].sum()
        assert np.allclose(Q[k][valid[k]], old[k][:n], atol=1e-12)
    found = (np.nan_to_num(np.abs(Q - q[:, None]), nan=9.0).max(-1) < 1e-6).any(1)
    print(f"\nIK on {len(T)} reachable poses: {valid.sum()} solutions (old: "
          f"{ref['ik_count'].sum()}), worst pose error {ep:.1e} m / {er:.1e}; "
          f"{(valid.sum(1) == 0).mean():.1%} poses with none; the configuration the pose came "
          f"from is among the answers for {found.mean():.1%}")


def test_ik_random_poses_matches_old(arm, ref):
    T = ref["ikr_hand"]
    Q, valid = arm.ik(T, ref["ikr_q7"])
    _check_solutions(arm, T, Q, valid)
    assert np.array_equal(valid.sum(1), ref["ikr_count"])


def test_ik_rejects_clamped_answers(arm):
    """The solver clamps at the workspace edge; such answers must come back invalid."""
    q = random_q(arm, 3000, 7)
    T = arm.fk(q)
    T[:, :3, 3] *= 1.6                      # push the hands outward, many beyond reach
    Q, valid = arm.ik(T, q[:, 6])
    _check_solutions(arm, T, Q, valid)


def test_ik_empty(arm):
    Q, valid = arm.ik(np.zeros((0, 4, 4)), np.zeros(0))
    assert Q.shape == (0, 4, 7) and valid.shape == (0, 4)


# ------------------------------------------------------------------ 3. hand_pose

def test_hand_pose_matches_old_convention(arm, ref):
    T = arm.hand_pose(ref["hp_tip"], np.array([0.0, 0.0, 1.0]), ref["hp_phi"], ref["hp_lean"])
    assert np.abs(T - ref["hp_hand"]).max() < 1e-12


def test_hand_pose_ik_tip_roundtrip(arm):
    """Inverted arm: paper below the base, normal -z in the base frame."""
    rng = np.random.default_rng(3)
    m = 4000
    r = rng.uniform(0.15, 0.65, m)
    a = rng.uniform(-np.pi, np.pi, m)
    tip = np.column_stack([r * np.cos(a), r * np.sin(a), rng.uniform(0.45, 0.95, m)])
    spin = rng.uniform(-np.pi, np.pi, m)
    ang = np.deg2rad(15.0) * np.sqrt(rng.random(m))
    d = rng.uniform(-np.pi, np.pi, m)
    lean = np.column_stack([ang * np.cos(d), ang * np.sin(d)])
    normal = np.array([0.0, 0.0, -1.0])
    T = arm.hand_pose(tip, normal, spin, lean)
    # the hand's approach axis is |lean| off straight into the paper
    assert np.allclose(np.sum(T[:, :3, 2] * -normal, axis=1), np.cos(ang))
    q7s = np.linspace(-2.8, 2.8, 8)
    Q, valid = arm.ik(np.repeat(T, len(q7s), 0), np.tile(q7s, m))
    tip_rep = np.repeat(tip, len(q7s), 0)
    err = np.linalg.norm(arm.tip(Q[valid]) - np.broadcast_to(tip_rep[:, None], Q.shape[:2] + (3,))
                         [valid], axis=1)
    solved = valid.reshape(m, -1).any(1).mean()
    print(f"\nhand_pose -> ik -> tip: {valid.sum()} solutions, worst tip error {err.max():.1e} m, "
          f"{solved:.1%} of tips solved at some q7")
    assert valid.sum() > 1000 and err.max() < TOL


def test_hand_pose_other_normals(arm):
    """Any normal: tip lands, approach axis is against the normal at zero lean."""
    rng = np.random.default_rng(4)
    for n in (np.array([1.0, 0, 0]), np.array([0.3, -0.2, 0.9]), np.array([0, 1.0, 0])):
        tip = rng.uniform(-0.5, 0.5, (50, 3))
        T = arm.hand_pose(tip, n, rng.uniform(-3, 3, 50), np.zeros((50, 2)))
        nn = n / np.linalg.norm(n)
        assert np.allclose(T[:, :3, 2], -nn)
        assert np.allclose(np.einsum("nij,nkj->nik", T[:, :3, :3], T[:, :3, :3]), np.eye(3))
        got = T[:, :3, 3] + T[:, :3, :3] @ arm.tool.tip_hand
        assert np.abs(got - tip).max() < 1e-12


# ------------------------------------------------------------------ 4. sigma_min, limit margin

def test_sigma_and_margin_match_old(arm, ref):
    q = ref["fk_q"]
    e_s = np.abs(arm.sigma_min(q) - ref["fk_sigma"]).max()
    e_m = np.abs(arm.limit_margin(q) - ref["fk_margin"]).max()
    print(f"\nsigma_min vs old {e_s:.1e}, limit_margin vs old {e_m:.1e}")
    assert e_s < 1e-12 and e_m == 0.0


def test_jacobian_by_differences(arm):
    q = random_q(arm, 50, 5)
    J = arm.tip_jacobian(q)
    h = 1e-6
    for j in range(7):
        dq = np.zeros(7)
        dq[j] = h
        fd = (arm.tip(q + dq) - arm.tip(q - dq)) / (2 * h)
        assert np.abs(fd - J[:, :, j]).max() < 1e-8


def test_limits(arm):
    L = arm.limits
    assert np.all(L.q_min < L.q_max)
    assert np.allclose(L.qdd_max, 10.0) and np.allclose(L.qddd_max, 5000.0)
    assert np.allclose(L.qd_max, [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


# ------------------------------------------------------------------ 5. body containment

@pytest.fixture(scope="module")
def meshes():
    pytest.importorskip("trimesh")
    sys.path.insert(0, str(Path(__file__).parent / "oracle"))
    from arm_meshes import load_bodies
    return load_bodies()


def test_body_contains_meshes(arm, meshes):
    q = random_q(arm, 5, 11)
    F = arm.link_frames(q)
    body = arm.body(q)
    report = []
    for part, (frame, V) in meshes.items():
        ks = [k for k, n in enumerate(body.names) if n == part or n.startswith(part + ".")]
        assert ks, part
        worst = -np.inf
        for i in range(len(q)):
            P = V @ F[i, frame, :3, :3].T + F[i, frame, :3, 3]
            d = np.min([pt_seg(P, body.p0[i, k], body.p1[i, k]) - body.radius[k] for k in ks],
                       axis=0)
            worst = max(worst, d.max())
        report.append(f"{part} {1e3 * worst:+.2f}")
        assert worst <= 1e-9, f"{part}: a vertex is {1e3 * worst:.2f} mm outside its capsules"
    print("\nworst vertex vs its capsules (mm, <= 0 inside): " + ", ".join(report))


def test_body_shape(arm):
    q = random_q(arm, 3, 2)
    b = arm.body(q)
    K = len(b.names)
    assert b.p0.shape == (3, K, 3) and b.p1.shape == (3, K, 3) and b.radius.shape == (K,)
    assert b.names[b.is_pen.nonzero()[0][0]] == "pen" and b.is_pen.sum() == 1
    # the pen capsule's surface reaches exactly to the tip
    k = b.names.index("pen")
    d = np.linalg.norm(arm.tip(q) - b.p1[:, k], axis=1)
    assert np.allclose(d, b.radius[k])


def test_fixed_capsules_do_not_move(arm):
    q = random_q(arm, 500, 13)
    b = arm.body(q)
    assert b.is_fixed.shape == (len(b.names),)
    assert [n for n, f in zip(b.names, b.is_fixed) if f] == [f"link0.{k}" for k in range(7)]
    for P in (b.p0, b.p1):
        assert np.array_equal(P[:, b.is_fixed], np.broadcast_to(P[:1, b.is_fixed],
                                                                P[:, b.is_fixed].shape))
    # and every other capsule does move
    moved = np.abs(b.p0 - b.p0[:1]).max(axis=(0, 2)) + np.abs(b.p1 - b.p1[:1]).max(axis=(0, 2))
    assert np.all(moved[~b.is_fixed] > 1e-3)


def test_self_pairs_can_separate(arm):
    """Every watched pair clears the margin somewhere: none is overlapping by construction."""
    q = random_q(arm, 20000, 9)
    b = arm.body(q)
    I, J = arm.self_pairs.T
    d = seg_seg(b.p0[:, I], b.p1[:, I], b.p0[:, J], b.p1[:, J]) - b.radius[I] - b.radius[J]
    assert np.all(d.max(0) > arm.self_margin)
    free = (d.min(1) >= arm.self_margin).mean()
    print(f"\n{len(I)} self pairs; {free:.1%} of random configurations clear the "
          f"{1e3 * arm.self_margin:.0f} mm self margin")


# ------------------------------------------------------------------ 6. speed

def test_speed(arm):
    q = random_q(arm, 10_000, 12)
    T = arm.fk(q)
    out = []
    for name, f in (("fk", lambda: arm.fk(q)), ("tip", lambda: arm.tip(q)),
                    ("body", lambda: arm.body(q)), ("ik", lambda: arm.ik(T, q[:, 6])),
                    ("sigma_min", lambda: arm.sigma_min(q))):
        f()
        t0 = time.perf_counter()
        f()
        dt = time.perf_counter() - t0
        out.append(f"{name} {len(q) / dt / 1e3:.0f}k/s")
        assert dt < 5.0
    print("\nbatch 10 000: " + ", ".join(out))
