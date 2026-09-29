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
from aris.types import Gates

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

IK_TOL = 1e-7


def _old_ik(arm, T_hand, q7):
    """The OLD vendored solver, used the way the old code used it (FR3 limits re-applied, every
    answer checked by forward kinematics to 1e-9).  For comparison only; the package never
    calls it.  -> Q (M,4,7) with NaN, valid (M,4)."""
    ik = pytest.importorskip("franka_analytical_ik")._franka_ik
    T = np.asarray(T_hand, float).copy()
    T[:, :3, 3] += fr3.D_HAND_TCP * T[:, :3, 2]
    flat = np.ascontiguousarray(T.transpose(0, 2, 1)).reshape(-1, 16)
    raw = ik.solve_batch(flat, np.ascontiguousarray(q7, dtype=float), np.zeros(7), 1e-9, 1e-9)
    ok = np.all(np.isfinite(raw), -1) & np.all((raw >= fr3.Q_MIN) & (raw <= fr3.Q_MAX), -1)
    return np.where(ok[..., None], raw, np.nan), ok


def _found(Q, q, tol=1e-6):
    """Is configuration q[i] among the answers Q[i]?  -> (M,) bool."""
    return (np.nan_to_num(np.abs(Q - q[:, None]), nan=9.0).max(-1) < tol).any(1)


def _check_solutions(arm, T, Q, valid):
    """Every valid answer is inside the limits and reproduces the pose. -> pose errors."""
    q = Q[valid]
    Tf = arm.fk(q)
    Tr = np.broadcast_to(T[:, None], Q.shape[:2] + (4, 4))[valid]
    e = np.maximum(np.linalg.norm(Tf[:, :3, 3] - Tr[:, :3, 3], axis=1),
                   np.linalg.norm(Tf[:, :3, :3] - Tr[:, :3, :3], axis=(1, 2)))
    assert np.all(e <= IK_TOL)
    assert np.all((q >= arm.limits.q_min) & (q <= arm.limits.q_max))
    assert np.all(np.isnan(Q[~valid]))
    return e


def _drawing_configs(arm, n_draw, seed):
    """Configurations that hold the pen on a paper 0.97 m below the base: tip within 3 cm of
    the plane, hand within 15 deg of vertical, 0.15 rad from every limit (rejection sampling)."""
    rng = np.random.default_rng(seed)
    L, keep = arm.limits, []
    while sum(len(k) for k in keep) < n_draw:
        q = L.q_min + rng.random((500_000, 7)) * (L.q_max - L.q_min)
        T = arm.fk(q)
        tip = T[:, :3, 3] + T[:, :3, :3] @ arm.tool.tip_hand
        ok = (np.abs(tip[:, 2] - 0.97) < 0.03) & (T[:, 2, 2] > np.cos(np.deg2rad(15.0)))
        keep.append(q[ok & (arm.limit_margin(q) >= 0.15)])
    return np.concatenate(keep)[:n_draw]


def test_ik_round_trip(arm):
    q = random_q(arm, 100_000, 21)
    T = arm.fk(q)
    Q, valid, flags = arm.ik(T, q[:, 6], with_flags=True)
    assert Q.shape == (len(q), 8, 7) and valid.shape == (len(q), 8)
    e = _check_solutions(arm, T, Q, valid)
    found = _found(Q, q)
    Qo, vo = _old_ik(arm, T, q[:, 6])
    print(f"\nround trip, 100 000 random configurations: new {found.mean():.4%}, old "
          f"{_found(Qo, q).mean():.2%}; {valid.sum() / len(q):.2f} answers per pose (old "
          f"{vo.sum() / len(q):.2f}); pose error worst {e.max():.1e}, "
          f"{int(((e > 1e-9) & (e <= 1e-7)).sum())} answers between 1e-9 and 1e-7; "
          f"{int((flags != 0).sum())} flagged")
    assert found.mean() >= 0.999


def test_ik_round_trip_drawing(arm):
    q = _drawing_configs(arm, 2000, 22)
    T = arm.fk(q)
    Q, valid = arm.ik(T, q[:, 6])
    _check_solutions(arm, T, Q, valid)
    found = _found(Q, q)
    Qo, vo = _old_ik(arm, T, q[:, 6])
    print(f"\nround trip, {len(q)} drawing configurations: new {found.mean():.2%}, old "
          f"{_found(Qo, q).mean():.2%}; poses with no answer: new "
          f"{(valid.sum(1) == 0).mean():.2%}, old {(vo.sum(1) == 0).mean():.2%}")
    assert found.mean() >= 0.999


def test_ik_shoulder_singular_is_flagged(arm):
    q = random_q(arm, 200, 23)
    q[:, 1] = 0.0
    T = arm.fk(q)
    Q, valid, flags = arm.ik(T, q[:, 6], with_flags=True)
    _check_solutions(arm, T, Q, valid)
    hit = (valid & (flags == 1)).any(1)
    assert hit.mean() > 0.9           # the family q1 + q3 = const is found and marked
    # a pose merely near it is solved normally; so close to the singularity q1 and q3 are
    # ill-conditioned (only their sum is sharp), so they come back to ~1e-5 rad at q2 = 1e-8
    for q2, tol in ((1e-8, 1e-4), (1e-5, 1e-6)):
        q[:, 1] = q2
        Q, valid, flags = arm.ik(arm.fk(q), q[:, 6], with_flags=True)
        assert _found(Q, q, tol).all() and not flags.any()


def test_ik_superset_of_old(arm, ref):
    worst = 0.0
    for T, q7, count in ((ref["ik_hand"], ref["ik_q"][:, 6], ref["ik_count"]),
                         (ref["ikr_hand"], ref["ikr_q7"], ref["ikr_count"])):
        Qo, vo = _old_ik(arm, T, q7)
        assert np.array_equal(vo.sum(1), count)        # the helper is the old code
        Q, valid = arm.ik(T, q7)
        _check_solutions(arm, T, Q, valid)
        m, b = np.nonzero(vo)
        d = np.nan_to_num(np.abs(Q[m] - Qo[m, b][:, None]), nan=9.0).max(-1).min(-1)
        assert d.max() < 1e-5, f"{(d >= 1e-5).sum()} old answers missing"
        worst = max(worst, d.max())
    print(f"\nsuperset: every old answer is among the new ones, to {worst:.1e} rad at worst "
          f"(the old solver's own precision near singularities)")


def test_ik_rejects_unreachable(arm):
    q = random_q(arm, 3000, 7)
    T = arm.fk(q)
    T[:, :3, 3] *= 1.6                      # push the hands outward, many beyond reach
    Q, valid = arm.ik(T, q[:, 6])
    _check_solutions(arm, T, Q, valid)


def test_ik_limits_are_arguments(arm):
    """Narrower limits only remove answers; the solver has none of its own."""
    import aris_fr3_ik
    q = random_q(arm, 2000, 24)
    T = arm.fk(q)
    lo, hi = arm.limits.q_min + 0.3, arm.limits.q_max - 0.3
    Q, _ = aris_fr3_ik.solve(T, np.ascontiguousarray(q[:, 6]), lo, hi)
    ok = np.isfinite(Q).all(-1)
    assert np.all((Q[ok] >= lo) & (Q[ok] <= hi))
    inside = np.all((q >= lo) & (q <= hi), 1)
    assert _found(Q[inside], q[inside]).mean() >= 0.999


def test_ik_empty(arm):
    Q, valid = arm.ik(np.zeros((0, 4, 4)), np.zeros(0))
    assert Q.shape == (0, 8, 7) and valid.shape == (0, 8)


def test_paper_coverage_old_vs_new(arm):
    """Tip positions on the paper 0.97 m below the base, out to 0.95 m, 2 cm grid, 8 spins,
    no lean, q7 on the old 16-value grid: how many have an answer passing the gates."""
    g = np.arange(-0.95, 0.95 + 1e-9, 0.02)
    X, Y = np.meshgrid(g, g)
    keep = np.hypot(X, Y) <= 0.95
    tips = np.column_stack([X[keep], Y[keep], np.full(keep.sum(), 0.97)])
    spins = np.linspace(-np.pi, np.pi, 8, endpoint=False)
    q7s = np.linspace(arm.limits.q_min[6] + 0.05, arm.limits.q_max[6] - 0.05, 16)
    n_t, n_s, n_q = len(tips), len(spins), len(q7s)
    T = arm.hand_pose(np.repeat(tips, n_s, 0), np.array([0.0, 0.0, -1.0]),
                      np.tile(spins, n_t), np.zeros((n_t * n_s, 2)))
    T = np.repeat(T, n_q, 0)
    q7 = np.tile(q7s, n_t * n_s)
    covered, spun, n_good, tip_ok = {}, {}, {}, {}
    for name, solver in (("old", lambda: _old_ik(arm, T, q7)), ("new", lambda: arm.ik(T, q7))):
        Q, valid = solver()
        m, b = np.nonzero(valid)
        q = Q[m, b]
        good = (arm.limit_margin(q) >= 0.15) & (arm.sigma_min(q) >= 0.08)
        hit = np.zeros(len(T), bool)
        hit[m[good]] = True
        h = hit.reshape(n_t, n_s, n_q)
        covered[name] = int(h.any((1, 2)).sum())
        spun[name] = int(h.any(2).sum())
        n_good[name] = int(good.sum())
        tip_ok[name] = h.any((1, 2))
    gain = covered["new"] / covered["old"] - 1.0
    r_new = np.hypot(*tips[tip_ok["new"] & ~tip_ok["old"], :2].T)
    print(f"\npaper coverage (2 cm grid, {n_t} tips out to 0.95 m): tips reachable inside the "
          f"gates old {covered['old']}, new {covered['new']} (+{gain:.1%}; the new ones lie at "
          f"{r_new.min():.2f}-{r_new.max():.2f} m); (tip, spin) pairs old {spun['old']}, new "
          f"{spun['new']} (+{spun['new'] / spun['old'] - 1:.1%}); gated answers old "
          f"{n_good['old']}, new {n_good['new']} (+{n_good['new'] / n_good['old'] - 1:.1%})")
    assert covered["new"] >= covered["old"] and not (tip_ok["old"] & ~tip_ok["new"]).any()


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
    margin = Gates().self_margin
    q = random_q(arm, 20000, 9)
    b = arm.body(q)
    I, J = arm.self_pairs.T
    d = seg_seg(b.p0[:, I], b.p1[:, I], b.p0[:, J], b.p1[:, J]) - b.radius[I] - b.radius[J]
    assert np.all(d.max(0) > margin)
    free = (d.min(1) >= margin).mean()
    print(f"\n{len(I)} self pairs; {free:.1%} of random configurations clear the "
          f"{1e3 * margin:.0f} mm self margin")


# ------------------------------------------------------------------ 6. speed

def _new_raw(arm, T, q7):
    import aris_fr3_ik
    return aris_fr3_ik.solve(T, np.ascontiguousarray(q7), arm.limits.q_min, arm.limits.q_max)


def _old_ik_checked(arm, T, q7):
    """The old solver plus the same forward-kinematics check the new path pays."""
    Q, ok = _old_ik(arm, T, q7)
    m, b = np.nonzero(ok)
    arm.fk(Q[m, b])
    return Q, ok


def test_reach_bounds_motion(arm):
    """Straight joint moves: no capsule endpoint moves further than sum_j |dq_j| reach[j, k]."""
    assert arm.reach.shape == (7, len(arm.body(np.zeros((1, 7))).names))
    b0 = arm.body(np.zeros((1, 7)))
    assert np.all(arm.reach[:, b0.is_fixed] == 0.0)
    ratios = []
    for seed, scale in ((25, 1.0), (26, 0.01)):   # large moves, and small ones (tight regime)
        rng = np.random.default_rng(seed)
        qa = random_q(arm, 10_000, seed)
        qb = np.clip(qa + scale * rng.uniform(-1, 1, qa.shape) * (arm.limits.q_max -
                                                                  arm.limits.q_min),
                     arm.limits.q_min, arm.limits.q_max)
        A, B = arm.body(qa), arm.body(qb)
        bound = np.abs(qb - qa) @ arm.reach                       # (N, K)
        move = np.maximum(np.linalg.norm(B.p0 - A.p0, axis=-1),
                          np.linalg.norm(B.p1 - A.p1, axis=-1))
        live = ~A.is_fixed
        assert np.all(move[:, live] <= bound[:, live] + 1e-12)
        ratios.append((move[:, live] / bound[:, live]).ravel())
    print(f"\nreach bound, endpoint travel / bound: large moves median {np.median(ratios[0]):.2f}"
          f" worst {ratios[0].max():.2f}; small moves median {np.median(ratios[1]):.2f} worst "
          f"{ratios[1].max():.2f}")


def test_speed(arm):
    q = random_q(arm, 10_000, 12)
    T = arm.fk(q)
    out = []
    for name, f in (("fk", lambda: arm.fk(q)), ("tip", lambda: arm.tip(q)),
                    ("body", lambda: arm.body(q)), ("sigma_min", lambda: arm.sigma_min(q)),
                    ("ik new (checked)", lambda: arm.ik(T, q[:, 6])),
                    ("ik old (checked)", lambda: _old_ik_checked(arm, T, q[:, 6])),
                    ("new solver alone", lambda: _new_raw(arm, T, q[:, 6])),
                    ("old solver alone", lambda: _old_ik(arm, T, q[:, 6]))):
        f()
        t0 = time.perf_counter()
        f()
        dt = time.perf_counter() - t0
        out.append(f"{name} {len(q) / dt / 1e3:.0f}k/s")
        assert dt < 10.0
    print("\nbatch 10 000: " + ", ".join(out))
