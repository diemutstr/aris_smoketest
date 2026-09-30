"""Tests for aris.kernel.footprint and distance fields in aris.kernel.collide.

Run with -s to see the measured numbers.
"""
from pathlib import Path
import time

import numpy as np
import pytest

from aris.kernel import collide, geometry as geo
from aris.kernel.footprint import footprint, transform_field
from aris.types import Body, Obstacles, Trajectory

DATA = Path(__file__).parent / "data"
TOL = 1e-12
CELL = 0.02
H = 0.5 * np.sqrt(3.0) * CELL
# what the field may give away against the exact distance (footprint.py): building it
# 2h + sigma/2 + a quarter cell for the radius class, reading it h + the axis sampling cell/2
LOSS = 3 * H + CELL


def _arm():
    mod = pytest.importorskip("aris.kernel.arm")
    return mod.Arm(mod.default_tool())


def _still(q):
    """A trajectory that holds q."""
    q = np.asarray(q, float)
    return Trajectory(np.array([0.0, 0.1]), np.stack([q, q]), np.zeros((2, 7)))


Q0 = np.array([0.3, -0.4, 0.2, -2.0, 0.1, 1.9, 0.7])


def _probes(rng, body, n=3000):
    """Random capsules around the arm (most outside it, some through it)."""
    ends = np.concatenate([body.p0[0], body.p1[0]])
    lo, hi = ends.min(axis=0) - 0.25, ends.max(axis=0) + 0.25
    a = rng.uniform(lo, hi, (n, 3))
    b = a + rng.normal(size=(n, 3)) * 0.08
    r = rng.uniform(0.005, 0.06, n)
    return a, b, r


def _exact(body, a, b, r):
    """Exact clearance of each probe capsule to the arm body (all capsules, fixed too)."""
    K = body.p0.shape[1]
    d = geo.segment_segment_distance(a[:, None], b[:, None], body.p0[0][None], body.p1[0][None])
    return (d - r[:, None] - np.asarray(body.radius)[None]).min(axis=1)


def _probe_body(a, b, r):
    n = len(a)
    return [Body(a[i:i + 1, None], b[i:i + 1, None], r[i:i + 1], ("probe",), np.zeros(1, bool))
            for i in range(n)]


def _field_clearance(a, b, r, field, backend):
    """Clearance of each probe against the field alone (one call, probes as N configurations
    of a one-capsule body is not possible with different radii, so group by radius)."""
    out = np.empty(len(a))
    obs = collide.pack(Obstacles(fields=(field,)))
    for rr in np.unique(r):
        i = np.flatnonzero(r == rr)
        body = Body(a[i, None], b[i, None], np.array([rr]), ("probe",), np.zeros(1, bool))
        out[i] = collide.clearance(body, obs, backend=backend)
    return out


def test_static_field_is_a_lower_bound_and_tight():
    arm = _arm()
    body = arm.body(Q0[None])
    for pad in (0.0, 0.01):
        t = time.process_time()
        f = footprint(arm, [_still(Q0)], cell=CELL, pad=pad)
        dt = time.process_time() - t
        rng = np.random.default_rng(1)
        a, b, r = _probes(rng, body)
        r = np.round(r, 2)                              # a few radii, so few calls
        exact = _exact(body, a, b, r) - pad
        got = _field_clearance(a, b, r, f, None)
        assert np.all(got <= exact + 1e-9)                              # never above the truth
        # the loss bound holds within the grid's reach; further out the field only promises
        # a lower bound (the grid ends a few cells beyond the body)
        out = (exact > 0) & (exact < 0.04)
        loss = exact[out] - got[out]
        print(f"\nstatic pose, pad {pad}: grid {f.dist.shape}, built in {dt:.2f} s; loss within 4 cm "
              f"median {np.median(loss) * 1e3:.1f} mm, max {loss.max() * 1e3:.1f} mm "
              f"(bound {LOSS * 1e3:.1f} mm; one cell diagonal is {2 * H * 1e3:.1f} mm)")
        assert loss.max() <= LOSS + 1e-9
        assert np.all(got[exact < -0.03] < 0)                            # deep inside reads inside


def test_engines_agree_on_fields():
    if collide.backend() != "native":
        pytest.skip("compiled module not installed")
    arm = _arm()
    f = footprint(arm, [_still(Q0)], cell=CELL)
    rng = np.random.default_rng(2)
    a, b, r = _probes(rng, arm.body(Q0[None]), 1500)
    r = np.round(r, 2)
    n = _field_clearance(a, b, r, f, "native")
    p = _field_clearance(a, b, r, f, "numpy")
    assert np.abs(n - p).max() <= TOL
    # the real arm against a field and some boxes: every mode and engine the same
    from test_kernel_collide import _cage_scene
    cage = _cage_scene(np.random.default_rng(3))
    shift = np.eye(4)
    shift[:3, 3] = [0.45, 0.3, 0.0]
    g = transform_field(f, shift, "neighbour")
    P = collide.pack(Obstacles(cage.boxes, cage.planes, cage.capsules, (g,)))
    T = collide.arm_tables(arm)
    Q = np.random.default_rng(4).uniform(arm.limits.q_min, arm.limits.q_max, (400, 7))
    ref = collide.clearance_detail(arm.body(Q), P, prune=False, backend="numpy")
    assert np.any(ref.obstacle == len(P.names) - 1)                    # the field matters
    for backend in ("native", "numpy"):
        for groups in (True, False):
            d = collide.clearance_detail_q(T, Q, P, backend=backend, groups=groups)
            assert np.abs(d.value - ref.value).max() <= TOL
            assert np.array_equal(d.capsule, ref.capsule) and np.array_equal(d.obstacle, ref.obstacle)
    qa, qb = Q[:10], Q[10:20]
    e = [collide.edges_clearance_q(T, arm.reach, qa, qb, P, floor=None, backend=be)
         for be in ("native", "numpy")]
    assert np.abs(e[0] - e[1]).max() <= TOL


def test_path_through_the_footprint_reads_negative():
    arm = _arm()
    body = arm.body(Q0[None])
    f = footprint(arm, [_still(Q0)], cell=CELL)
    c = 0.5 * (body.p0[0, 20] + body.p1[0, 20])                        # a point on link 3
    start, end = c + np.array([-0.6, 0.0, 0.0]), c + np.array([0.6, 0.0, 0.0])

    def probe_of(Q):                                                    # joint 0 = position
        s = np.asarray(Q)[:, 0:1]
        p = start + s * (end - start)
        return Body(p[:, None], (p + [0, 0, 0.05])[:, None], np.array([0.02]), ("probe",),
                    np.zeros(1, bool))
    q = np.zeros((2, 7))
    q[1, 0] = 1.0
    reach = np.zeros((7, 1))
    reach[0, 0] = np.linalg.norm(end - start)
    obs = Obstacles(fields=(f,))
    assert collide.clearance(probe_of(q), obs).min() > 0                # both ends are clear
    for backend in ("numpy", "native"):
        if backend == "native" and collide.backend() != "native":
            continue
        assert collide.path_clearance(probe_of, q, obs, reach, backend=backend) < 0


def test_transform_field_is_conservative():
    arm = _arm()
    body = arm.body(Q0[None])
    f = footprint(arm, [_still(Q0)], cell=CELL)
    ang = 0.7
    T = np.eye(4)
    T[:3, :3] = [[np.cos(ang), -np.sin(ang), 0], [np.sin(ang), np.cos(ang), 0], [0, 0, 1]]
    T[:3, 3] = [0.3, -1.1, 0.05]
    g = transform_field(f, T)
    rng = np.random.default_rng(5)
    ends = np.concatenate([body.p0[0], body.p1[0]])
    p_b = rng.uniform(ends.min(axis=0) - 0.3, ends.max(axis=0) + 0.3, (20000, 3))
    p_a = p_b @ T[:3, :3].T + T[:3, 3]
    exact = (geo.point_segment_distance(p_b[:, None], body.p0[0][None], body.p1[0][None])
             - np.asarray(body.radius)[None]).min(axis=1)
    old = geo.field_lookup(p_b, f.origin_base, f.cell, f.dist.shape, f.dist)
    new = geo.field_lookup(p_a, g.origin_base, g.cell, g.dist.shape, g.dist)
    assert np.all(new <= exact + 1e-9) and np.all(old <= exact + 1e-9)
    out = (exact > 0) & (exact < 0.04)
    print(f"\ntransform_field: grid {f.dist.shape} -> {g.dist.shape}; extra loss against the "
          f"field before, median {np.median(old[out] - new[out]) * 1e3:.1f} mm, max "
          f"{np.max(old[out] - new[out]) * 1e3:.1f} mm (bound {H * 1e3:.1f} + {H * 1e3:.1f} mm)")
    assert np.all(new[out] >= old[out] - 2 * H - 1e-9)


def _spiral_71():
    z = np.load(DATA / "collide_footprint_spiral71.npz")
    o = z["offsets"]
    return [Trajectory(z["t"][a:b], z["q"][a:b], z["qd"][a:b]) for a, b in zip(o[:-1], o[1:])]


@pytest.mark.slow
def test_spiral_phase_speed_report():
    rig_mod = pytest.importorskip("aris.rig")
    rig = rig_mod.Rig.load(Path(__file__).parents[1] / "config")
    leader, follower = rig.arm(71), rig.arm(31)
    trs = _spiral_71()
    t = time.process_time()
    f71 = footprint(leader, trs, cell=CELL, name="leader71")
    t_build = time.process_time() - t
    T_31_71 = rig.T_base_table(31) @ rig.T_table_base(71)
    t = time.process_time()
    f = transform_field(f71, T_31_71)
    t_tf = time.process_time() - t
    base = rig.obstacles(31, walls=(rig.wall_between(31, 17), rig.wall_between(31, 97)))
    P = collide.pack(Obstacles(base.boxes, base.planes, base.capsules, (f,)))
    T = collide.arm_tables(follower)
    rng = np.random.default_rng(6)
    print(f"\nleader 71, spiral phase 1: {len(trs)} motions, "
          f"{sum(x.t[-1] - x.t[0] for x in trs):.0f} s of motion; field {f71.dist.shape} "
          f"({f71.dist.nbytes / 1e6:.1f} MB) built in {t_build:.1f} s CPU; in arm 31's frame "
          f"{f.dist.shape} ({f.dist.nbytes / 1e6:.1f} MB) in {t_tf:.1f} s")
    for backend in ("native", "numpy"):
        if backend == "native" and collide.backend() != "native":
            continue
        for N in (1, 100, 10_000) if backend == "native" else (100, 1000):
            Q = rng.uniform(follower.limits.q_min, follower.limits.q_max, (N, 7))
            reps = max(1, 2000 // N) if backend == "native" else 1
            collide.clearance_q(T, Q, P, backend=backend)
            t = time.process_time()
            for _ in range(reps):
                c = collide.clearance_q(T, Q, P, backend=backend)
            rate = N * reps / (time.process_time() - t)
            print(f"  {backend}, batch {N:6d}: {rate:9.0f} configurations/s")
        if backend == "native":
            assert rate > 5_000                      # ten times below the slowest seen
