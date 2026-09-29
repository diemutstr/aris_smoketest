"""The compiled collision check against the numpy one: same numbers, same closest pairs.

Skipped when aris_collide_native is not installed.  Run with -s for the speed report.
"""
import time

import numpy as np
import pytest

from aris.kernel import collide
from aris.types import Body, Box, Obstacles
from test_kernel_collide import (_cage_scene, _chain_body, _random_body, _resample, _scene,
                                 _seg_box_cases, _seg_seg_cases, _toy_scene)

pytest.importorskip("aris_collide_native")
TOL = 1e-12


def _same(a, b):
    """Two ClearanceDetails agree: values to TOL, closest pair identical."""
    assert np.abs(a.value - b.value).max(initial=0.0) <= TOL
    assert np.array_equal(a.capsule, b.capsule) and np.array_equal(a.obstacle, b.obstacle)


def test_backend_is_native():
    assert collide.backend() == "native"


@pytest.mark.parametrize("which", ["crowded", "rig-like"])
def test_scenes_agree(which):
    rng = np.random.default_rng(20)
    obs, body = ((_scene(rng), _random_body(rng, 1000)) if which == "crowded"
                 else (_cage_scene(rng), _chain_body(rng, 1000)))
    for drawing in (False, True):
        for prune in (False, True):
            n = collide.clearance_detail(body, obs, drawing, prune, backend="native")
            p = collide.clearance_detail(body, obs, drawing, prune, backend="numpy")
            _same(n, p)
        full_n = collide.capsule_clearance(body, obs, drawing, prune=False, backend="native")
        full_p = collide.capsule_clearance(body, obs, drawing, prune=False, backend="numpy")
        assert np.abs(full_n - full_p).max() <= TOL


def test_segment_box_cases_agree():
    """The brute-force cases of the numpy tests, one box at a time, no pruning."""
    rng = np.random.default_rng(21)
    p0, p1, R, c, h = _seg_box_cases(rng, 10_000)
    worst = 0.0
    for s in range(0, 10_000, 50):                    # 50 segments against the same box
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R[s], c[s]
        obs = Obstacles(boxes=(Box("b", T, h[s], 0.0),))
        a, b = p0[s:s + 50], p1[s:s + 50]
        body = Body(a[:, None], b[:, None], np.zeros(1), ("seg",), np.zeros(1, bool))
        d_n = collide.capsule_clearance(body, obs, prune=False, backend="native")
        d_p = collide.capsule_clearance(body, obs, prune=False, backend="numpy")
        worst = max(worst, float(np.abs(d_n - d_p).max()))
    print(f"\nsegment-box native vs numpy: {worst:.1e} m")
    assert worst <= TOL


def test_segment_segment_cases_agree():
    rng = np.random.default_rng(22)
    p0, p1, q0, q1 = _seg_seg_cases(rng, 10_000)
    body = Body(np.stack([p0, q0], 1), np.stack([p1, q1], 1), np.zeros(2), ("p", "q"),
                np.zeros(2, bool))
    pairs = np.array([[0, 1]])
    d_n = collide.self_clearance(body, pairs, 0.0, backend="native")
    d_p = collide.self_clearance(body, pairs, 0.0, backend="numpy")
    print(f"\nsegment-segment native vs numpy: {np.abs(d_n - d_p).max():.1e} m")
    assert np.abs(d_n - d_p).max() <= TOL


# --------------------------------------------------------------------------- from joint angles


def _arm():
    return pytest.importorskip("aris.kernel.arm").Arm(
        pytest.importorskip("aris.kernel.arm").default_tool())


def _q(arm, n, seed):
    return np.random.default_rng(seed).uniform(arm.limits.q_min, arm.limits.q_max, (n, 7))


def test_kinematics_match_arm_body():
    arm = _arm()
    T = collide.arm_tables(arm)
    Q = _q(arm, 3000, 23)
    ref = arm.body(Q)
    for backend in ("native", "numpy"):
        b = collide.body_q(T, Q, backend)
        assert np.abs(b.p0 - ref.p0).max() <= TOL and np.abs(b.p1 - ref.p1).max() <= TOL
        assert np.array_equal(b.is_fixed, ref.is_fixed) and np.array_equal(b.is_pen, ref.is_pen)


def _pair_value(body, packed, k, m):
    """Clearance of capsule k against obstacle m alone, by the numpy code."""
    one = Body(body.p0[:, k:k + 1], body.p1[:, k:k + 1], body.radius[k:k + 1], (body.names[k],),
               body.is_pen[k:k + 1])
    mask = np.zeros(len(packed.names), bool)
    mask[m] = True
    Mb, Mp = len(packed.box_m), len(packed.pl_m)
    sel = lambda arr, lo, hi: arr[mask[lo:hi]]
    P = collide.Packed(sel(packed.box_R, 0, Mb), sel(packed.box_c, 0, Mb), sel(packed.box_h, 0, Mb),
                       sel(packed.box_m, 0, Mb), sel(packed.pl_n, Mb, Mb + Mp),
                       sel(packed.pl_off, Mb, Mb + Mp), sel(packed.pl_m, Mb, Mb + Mp),
                       sel(packed.pl_pen_m, Mb, Mb + Mp), sel(packed.pl_paper, Mb, Mb + Mp),
                       sel(packed.cap_a, Mb + Mp, None), sel(packed.cap_b, Mb + Mp, None),
                       sel(packed.cap_rm, Mb + Mp, None), (packed.names[m],))
    return float(collide.clearance(one, P, backend="numpy", prune=False)[0])


def test_q_calls_agree_and_threads_change_nothing():
    arm = _arm()
    T = collide.arm_tables(arm)
    rng = np.random.default_rng(24)
    obs = collide.pack(_cage_scene(rng))
    Q = _q(arm, 1500, 25)
    ref = collide.clearance(arm.body(Q), obs, backend="numpy")
    one = collide.clearance_q(T, Q, obs, backend="native", threads=1)
    assert np.abs(one - ref).max() <= TOL
    for th in (2, 8, 32):
        assert np.array_equal(collide.clearance_q(T, Q, obs, threads=th), one)
    # From the same capsule ends the closest pair is identical; from the tables the ends differ
    # from Arm.body's by 1e-16, which can only swap two pairs that tie to within TOL (two
    # neighbouring capsules of the parked arm meeting at a shared end).
    _same(collide.clearance_detail(arm.body(Q), obs, backend="native", threads=8),
          collide.clearance_detail(arm.body(Q), obs, backend="numpy"))
    dq = collide.clearance_detail_q(T, Q, obs, backend="native", threads=8)
    dp = collide.clearance_detail(arm.body(Q), obs, backend="numpy")
    assert np.abs(dq.value - dp.value).max() <= TOL
    swap = np.flatnonzero((dq.capsule != dp.capsule) | (dq.obstacle != dp.obstacle))
    assert len(swap) < 0.01 * len(Q)
    for i in swap:                                      # the native pair is as close, to TOL
        assert abs(_pair_value(arm.body(Q[i:i + 1]), obs, dq.capsule[i], dq.obstacle[i])
                   - dp.value[i]) <= TOL
    pairs = arm.self_pairs
    s_ref = collide.self_clearance(arm.body(Q), pairs, 0.023, backend="numpy")
    for th in (1, 8):
        assert np.abs(collide.self_clearance_q(T, Q, pairs, 0.023, threads=th) - s_ref).max() <= TOL


def test_path_clearance_q_agrees():
    arm = _arm()
    T = collide.arm_tables(arm)
    rng = np.random.default_rng(26)
    home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8])
    obs = collide.pack(_toy_scene())
    worst = 0.0
    for _ in range(8):
        q = np.clip(home + np.cumsum(rng.normal(size=(5, 7)) * 0.3, axis=0),
                    arm.limits.q_min, arm.limits.q_max)
        for per in (1, 8):
            qq = _resample(q, per)
            ref = collide.path_clearance(arm.body, qq, obs, arm.reach, backend="numpy")
            nat = collide.path_clearance_q(T, arm.reach, qq, obs, backend="native")
            assert nat == collide.path_clearance_q(T, arm.reach, qq, obs, threads=8)
            worst = max(worst, abs(nat - ref))
    print(f"\npath_clearance native vs numpy: {worst:.1e} m")
    assert worst <= 1e-9


# --------------------------------------------------------------------------- speed


def _rate(f, n, reps, clock=time.process_time):
    """Items per second, best of three.  CPU time of the process for one thread (the machine is
    shared); wall-clock time for the threaded rows, where CPU time would add up the threads."""
    f()
    best = np.inf
    for _ in range(3):
        t = clock()
        for _ in range(reps):
            f()
        best = min(best, (clock() - t) / reps)
    return n / best


def _speed_rows(label, body_of, T, obs, K_live, rng, q_min, q_max):
    P = collide.pack(obs)
    pairs = K_live * len(P.names)
    print(f"\n{label}: {K_live} capsules checked x {len(P.names)} obstacles = {pairs} pairs")
    for N in (1, 100, 10_000):
        Q = rng.uniform(q_min, q_max, (N, 7))
        reps = max(1, 2000 // N)
        r_np = _rate(lambda: collide.clearance(body_of(Q), P, backend="numpy"), N, max(1, reps // 10))
        r_nat = _rate(lambda: collide.clearance_q(T, Q, P, backend="native"), N, reps)
        print(f"  batch {N:6d}: numpy {r_np:9.0f}/s  native {r_nat:9.0f}/s "
              f"({r_nat * pairs / 1e6:.0f} M pairs/s)")
    Q = rng.uniform(q_min, q_max, (10_000, 7))
    for th in (8, 32):
        r = _rate(lambda: collide.clearance_q(T, Q, P, threads=th), 10_000, 3, time.perf_counter)
        print(f"  batch  10000, {th:2d} threads: native {r:9.0f}/s (wall clock)")
    return r_nat


@pytest.mark.slow
def test_speed_report_arm31():
    rig_mod = pytest.importorskip("aris.rig")
    from pathlib import Path
    rig = rig_mod.Rig.load(Path(__file__).parents[1] / "config")
    arm = rig.arm(31)
    T = collide.arm_tables(arm)
    obs = rig.obstacles(31, parked=(71,), walls=(rig.wall_between(31, 17), rig.wall_between(31, 97)))
    rng = np.random.default_rng(27)
    r = _speed_rows("arm 31, phase 2 (walls to 17 and 97, arm 71 parked)", arm.body, T, obs,
                    int((~T.is_fixed).sum()), rng, arm.limits.q_min, arm.limits.q_max)
    assert r > 2_000                                    # ten times below the 22 000/s measured (62-capsule arm)
    # one path of 50 samples through the free space
    q_free = rng.uniform(arm.limits.q_min, arm.limits.q_max, (4000, 7))
    q_free = q_free[collide.clearance_q(T, q_free, obs) > 0.0][:2]
    q = q_free[0] + np.linspace(0, 1, 50)[:, None] * (q_free[1] - q_free[0])
    P = collide.pack(obs)
    t_np = 1 / _rate(lambda: collide.path_clearance(arm.body, q, P, arm.reach, backend="numpy"), 1, 3)
    t_nat = 1 / _rate(lambda: collide.path_clearance_q(T, arm.reach, q, P), 1, 20)
    b = collide.path_clearance_q(T, arm.reach, q, P)
    print(f"  path_clearance, 50 samples (bound {b * 1e3:.1f} mm): numpy {t_np * 1e3:.2f} ms, "
          f"native {t_nat * 1e3:.3f} ms")


@pytest.mark.slow
def test_speed_report_synthetic12():
    """The 12-capsule case of the first report: native gets the same chain from a table."""
    rng = np.random.default_rng(28)
    obs = _cage_scene(rng)
    # a made-up 7-joint chain whose 12 capsules hang off its frames, as ArmTables
    arm = _arm()
    base = collide.arm_tables(arm)
    k = np.flatnonzero(~base.is_fixed)[:12]
    T = collide.ArmTables(base.dh, base.ex_parent, base.ex_R, base.ex_t, base.cap_frame[k],
                          base.cap_a[k], base.cap_b[k], np.full(12, 0.05), base.is_pen[k],
                          np.zeros(12, bool), tuple(base.names[i] for i in k))
    body_of = lambda Q: collide.body_q(T, Q, "numpy")
    r = _speed_rows("12 capsules (first report's case)", body_of, T, obs, 12, rng,
                    arm.limits.q_min, arm.limits.q_max)
    assert r > 15_000                                   # ten times below the 155 000/s measured


# --------------------------------------------------------------------------- batches of edges


def _arm31():
    rig_mod = pytest.importorskip("aris.rig")
    from pathlib import Path
    rig = rig_mod.Rig.load(Path(__file__).parents[1] / "config")
    arm = rig.arm(31)
    obs = rig.obstacles(31, parked=(71,), walls=(rig.wall_between(31, 17), rig.wall_between(31, 97)))
    return arm, collide.arm_tables(arm), collide.pack(obs)


def _edges(arm, T, P, n, length, seed):
    """n straight edges of the given joint-space length, starting in free space."""
    rng = np.random.default_rng(seed)
    Q = rng.uniform(arm.limits.q_min, arm.limits.q_max, (20 * n, 7))
    Qa = Q[collide.clearance_q(T, Q, P) > 0.0][:n]
    d = rng.normal(size=Qa.shape)
    Qb = np.clip(Qa + length * d / np.linalg.norm(d, axis=1, keepdims=True),
                 arm.limits.q_min, arm.limits.q_max)
    return Qa, Qb


def test_edges_equal_path_clearance():
    arm, T, P = _arm31()
    Qa, Qb = _edges(arm, T, P, 20, 0.3, 30)
    pairs = arm.self_pairs
    for backend in ("native", "numpy"):
        e = collide.edges_clearance_q(T, arm.reach, Qa, Qb, P, floor=None, backend=backend)
        es = collide.edges_clearance_q(T, arm.reach, Qa, Qb, P, floor=None, self_pairs=pairs,
                                       self_margin=0.023, backend=backend)
        for i in range(len(Qa)):
            q = np.stack([Qa[i], Qb[i]])
            ref = collide.path_clearance_q(T, arm.reach, q, P, backend="numpy")
            ref_s = collide.path_self_clearance_q(T, arm.reach, q, pairs, 0.023, backend="numpy")
            assert abs(e[i] - ref) <= TOL
            assert abs(es[i] - min(ref, ref_s)) <= TOL
    assert collide.path_self_clearance_q(T, arm.reach, np.stack([Qa[0], Qb[0]]), pairs, 0.023) == \
        pytest.approx(collide.path_self_clearance_q(T, arm.reach, np.stack([Qa[0], Qb[0]]), pairs,
                                                    0.023, backend="numpy"), abs=TOL)


def test_edges_with_floor_get_the_sign_right_and_threads_change_nothing():
    arm, T, P = _arm31()
    tol = 5e-4
    Qa, Qb = _edges(arm, T, P, 150, 0.5, 31)
    fast = collide.edges_clearance_q(T, arm.reach, Qa, Qb, P, tol=tol, floor=0.0)
    s = np.linspace(0, 1, 800)[None, :, None]
    dense = Qa[:, None] + s * (Qb - Qa)[:, None]
    truth = collide.clearance_q(T, dense.reshape(-1, 7), P, threads=8).reshape(len(Qa), -1).min(1)
    clear = np.abs(truth) > tol + 1e-4                 # dense sampling decides these
    assert np.all((fast[clear] >= 0) == (truth[clear] >= 0))
    assert np.all(fast[fast >= 0] <= truth[fast >= 0] + 1e-12)   # a "free" answer is a bound
    print(f"\nedges with floor 0: {np.sum(fast >= 0)} free, {np.sum(fast < 0)} not, "
          f"{np.sum(~clear)} too close to call by sampling")
    for th in (2, 8, 32):
        assert np.array_equal(collide.edges_clearance_q(T, arm.reach, Qa, Qb, P, tol=tol,
                                                        floor=0.0, threads=th), fast)
    nump = collide.edges_clearance_q(T, arm.reach, Qa, Qb, P, tol=tol, floor=0.0, backend="numpy")
    assert np.abs(nump - fast).max() <= TOL


@pytest.mark.slow
def test_edges_speed_report():
    arm, T, P = _arm31()
    pairs = arm.self_pairs
    print("\nedges on arm 31 (phase 2), one thread, CPU time")
    worst = 0.0
    for length in (0.05, 0.3, 1.0):
        Qa, Qb = _edges(arm, T, P, 400, length, 32)
        rows = []
        for label, call in [
            ("edges, floor 0", lambda: collide.edges_clearance_q(T, arm.reach, Qa, Qb, P)),
            ("edges, floor 0, self", lambda: collide.edges_clearance_q(
                T, arm.reach, Qa, Qb, P, self_pairs=pairs, self_margin=0.023)),
            ("edges, floor None", lambda: collide.edges_clearance_q(T, arm.reach, Qa, Qb, P,
                                                                    floor=None)),
            ("path_clearance_q loop", lambda: [collide.path_clearance_q(
                T, arm.reach, np.stack([a, b]), P) for a, b in zip(Qa, Qb)])]:
            t = time.process_time()
            call()
            dt = time.process_time() - t
            worst = max(worst, dt / len(Qa))
            rows.append(f"{label} {len(Qa) / dt:8.0f}/s")
        print(f"  {length:4.2f} rad: " + "  ".join(rows))
    assert worst < 10 * 3e-3                            # generous: ten times the slowest seen


def test_q_calls_with_nothing_to_hit():
    arm = _arm()
    T = collide.arm_tables(arm)
    Qa, Qb = _q(arm, 5, 33), _q(arm, 5, 34)
    for backend in ("native", "numpy"):
        assert collide.path_clearance_q(T, arm.reach, np.vstack([Qa, Qb]), Obstacles(),
                                        backend=backend) == np.inf
        assert np.all(collide.edges_clearance_q(T, arm.reach, Qa, Qb, Obstacles(), floor=None,
                                                backend=backend) == np.inf)
