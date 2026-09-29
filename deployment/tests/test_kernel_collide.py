"""Tests for aris.kernel.collide and aris.kernel.geometry.

Run with -s to see the measured numbers (errors, speed).
"""
from pathlib import Path
import time

import numpy as np
import pytest

from aris.kernel import collide, collide_native, geometry as geo
from aris.types import Body, Box, Capsule, Obstacles, Plane

DATA = Path(__file__).parent / "data"
N_PAIRS = 10_000


def _rot(rng, n):
    Q, Rr = np.linalg.qr(rng.normal(size=(n, 3, 3)))
    return Q * np.sign(np.linalg.det(Q))[:, None, None]


# --------------------------------------------------------------------------- brute force
# Independent of geometry.py: sample the parameters densely, keep the best, zoom in around it,
# repeat.  The squared distances involved are convex in the sampled parameters, so zooming
# around the best sample cannot lose the minimum.


def _zoom_1d(fun, P, rounds=7, n=201):
    lo, hi = np.zeros(P), np.ones(P)
    for _ in range(rounds):
        t = lo[:, None] + (hi - lo)[:, None] * np.linspace(0, 1, n)[None]
        v = fun(t)
        k = v.argmin(1)
        w = (hi - lo) / (n - 1)
        tk = t[np.arange(P), k]
        lo, hi = np.maximum(tk - 2 * w, 0), np.minimum(tk + 2 * w, 1)
    return v.min(1)


def _brute_seg_seg(p0, p1, q0, q1, chunk=2000):
    """Sample the first segment densely; the nearest point of the second is found by clamping
    the projection (elementary, written out here, not taken from geometry.py)."""
    out = []
    for s0 in range(0, len(p0), chunk):
        a, b, c, d = (x[s0:s0 + chunk] for x in (p0, p1, q0, q1))
        e = d - c
        ee = np.maximum(np.einsum("pi,pi->p", e, e), 1e-300)

        def f(s):
            pts = a[:, None] + s[..., None] * (b - a)[:, None]
            t = np.clip(np.einsum("pni,pi->pn", pts - c[:, None], e) / ee[:, None], 0, 1)
            return np.linalg.norm(pts - c[:, None] - t[..., None] * e[:, None], axis=-1)
        out.append(_zoom_1d(f, len(a)))
    return np.concatenate(out)


def _brute_seg_box(p0, p1, R, c, h, chunk=2000):
    out = []
    for s0 in range(0, len(p0), chunk):
        a, b, RR, cc, hh = (x[s0:s0 + chunk] for x in (p0, p1, R, c, h))

        def f(t):
            pts = a[:, None] + t[..., None] * (b - a)[:, None]
            loc = np.einsum("pnj,pji->pni", pts - cc[:, None], RR)       # R^T (p - c)
            near = np.clip(loc, -hh[:, None], hh[:, None])               # closest box point
            return np.linalg.norm(loc - near, axis=-1)
        out.append(_zoom_1d(f, len(a)))
    return np.concatenate(out)


# --------------------------------------------------------------------------- random cases


def _seg_seg_cases(rng, n):
    p0 = rng.normal(size=(n, 3)) * 0.3
    p1 = p0 + rng.normal(size=(n, 3)) * 0.3
    q0 = rng.normal(size=(n, 3)) * 0.3
    q1 = q0 + rng.normal(size=(n, 3)) * 0.3
    j = n // 6
    off = rng.normal(size=(j, 3)) * 0.05                                     # parallel
    q0[:j] = p0[:j] + off
    q1[:j] = p1[:j] + off + rng.uniform(-0.5, 0.5, (j, 1)) * (p1[:j] - p0[:j])
    sl = slice(j, 2 * j)                                                     # crossing
    m = p0[sl] + rng.uniform(0, 1, (j, 1)) * (p1[sl] - p0[sl])
    u = rng.uniform(0, 1, (j, 1))
    q0[sl], q1[sl] = m - u * (q1[sl] - q0[sl]), m + (1 - u) * (q1[sl] - q0[sl])
    sl = slice(2 * j, 3 * j)                                                 # touching at an end
    q0[sl] = p0[sl] + rng.uniform(0, 1, (j, 1)) * (p1[sl] - p0[sl])
    sl = slice(3 * j, 4 * j)                                                 # nearly parallel
    q1[sl] = q0[sl] + (p1[sl] - p0[sl]) * (1 + 1e-7 * rng.normal(size=(j, 1)))
    sl = slice(4 * j, 5 * j)                                                 # one is a point
    q1[sl] = q0[sl]
    return p0, p1, q0, q1


def _seg_box_cases(rng, n):
    R = _rot(rng, n)
    c = rng.normal(size=(n, 3)) * 0.2
    h = rng.uniform(0.01, 0.3, (n, 3))
    loc0 = rng.uniform(-0.6, 0.6, (n, 3))
    loc1 = loc0 + rng.normal(size=(n, 3)) * 0.4
    j = n // 5
    loc1[:j] = -loc0[:j] * rng.uniform(0.5, 2, (j, 1))                      # through the centre
    sl = slice(j, 2 * j)                                                     # touching a face
    loc0[sl, 2] = h[sl, 2]
    loc0[sl, :2] = rng.uniform(-1, 1, (j, 2)) * h[sl, :2]
    sl = slice(2 * j, 3 * j)                                                 # parallel to a face
    loc0[sl, 2] = h[sl, 2] + rng.uniform(0, 0.05, j)
    loc1[sl, 2] = loc0[sl, 2]
    sl = slice(3 * j, 4 * j)                                                 # a point
    loc1[sl] = loc0[sl]
    world = lambda x: np.einsum("pij,pj->pi", R, x) + c
    return world(loc0), world(loc1), R, c, h


# --------------------------------------------------------------------------- 1. brute force


@pytest.mark.slow
def test_segment_segment_vs_brute_force():
    rng = np.random.default_rng(1)
    p0, p1, q0, q1 = _seg_seg_cases(rng, N_PAIRS)
    d = geo.segment_segment_distance(p0, p1, q0, q1)
    b = _brute_seg_seg(p0, p1, q0, q1)
    err = d - b
    print(f"\nseg-seg vs brute force: max |err| {np.abs(err).max():.2e} m, "
          f"exact below brute by at most {-err.min():.2e}, above by at most {err.max():.2e}")
    assert np.abs(err).max() < 1e-6


@pytest.mark.slow
def test_segment_box_vs_brute_force():
    rng = np.random.default_rng(2)
    p0, p1, R, c, h = _seg_box_cases(rng, N_PAIRS)
    d = geo.segment_box_distance(p0, p1, R, c, h)
    b = _brute_seg_box(p0, p1, R, c, h)
    err = d - b
    print(f"\nseg-box vs brute force: max |err| {np.abs(err).max():.2e} m; "
          f"{(d == 0).sum()} of {N_PAIRS} pairs touch or cross")
    assert np.abs(err).max() < 1e-6
    assert np.all(d[: N_PAIRS // 5] == 0.0)          # through the centre: exactly zero


@pytest.mark.slow
def test_point_and_plane_vs_brute_force():
    rng = np.random.default_rng(3)
    n = N_PAIRS
    p, a, b = (rng.normal(size=(n, 3)) for _ in range(3))
    b[: n // 10] = a[: n // 10]
    ps = geo.point_segment_distance(p, a, b)
    brute = _zoom_1d(lambda t: np.linalg.norm(a[:, None] + t[..., None] * (b - a)[:, None] - p[:, None],
                                              axis=-1), n)
    assert np.abs(ps - brute).max() < 1e-9
    nrm = rng.normal(size=(n, 3))
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    off = rng.normal(size=n) * 0.2
    sp = geo.segment_plane_distance(a, b, nrm, off)
    brute = _zoom_1d(lambda t: np.einsum("pni,pi->pn", a[:, None] + t[..., None] * (b - a)[:, None], nrm)
                     - off[:, None], n)
    assert np.abs(sp - brute).max() < 1e-12


def test_distances_vs_brute_force_quick():
    """A 500-pair slice of the brute-force checks above, for the quick set."""
    rng = np.random.default_rng(17)
    p0, p1, q0, q1 = _seg_seg_cases(rng, 500)
    assert np.abs(geo.segment_segment_distance(p0, p1, q0, q1) - _brute_seg_seg(p0, p1, q0, q1)).max() < 1e-6
    p0, p1, R, c, h = _seg_box_cases(rng, 500)
    assert np.abs(geo.segment_box_distance(p0, p1, R, c, h) - _brute_seg_box(p0, p1, R, c, h)).max() < 1e-6


# --------------------------------------------------------------------------- 2. old code


@pytest.mark.slow
def test_against_old_segment_box_clearance():
    z = np.load(DATA / "collide_old_segbox.npz")
    n = len(z["A"])
    R = np.broadcast_to(np.eye(3), (n, 3, 3))
    d = geo.segment_box_distance(z["A"], z["B"], R, 0.5 * (z["lo"] + z["hi"]),
                                 0.5 * (z["hi"] - z["lo"]))
    err = d - z["old"]
    brute = _brute_seg_box(z["A"], z["B"], np.array(R), 0.5 * (z["lo"] + z["hi"]),
                           0.5 * (z["hi"] - z["lo"]))
    print(f"\nold segment_box_clearance: max |new-old| {np.abs(err).max():.2e} m "
          f"(new lower by up to {-err.min():.2e}); max |new-brute| {np.abs(d - brute).max():.2e}, "
          f"max |old-brute| {np.abs(z['old'] - brute).max():.2e}")
    assert np.abs(err).max() < 1e-8
    # the old ternary search only ever overshoots: every candidate it evaluates is a real point
    assert err.max() <= 1e-15


@pytest.mark.slow
def test_against_old_seg_seg_dist():
    z = np.load(DATA / "collide_old_segseg.npz")
    d = geo.segment_segment_distance(z["p0"], z["p1"], z["q0"], z["q1"])
    err = d - z["old"]
    brute = _brute_seg_seg(z["p0"], z["p1"], z["q0"], z["q1"])
    print(f"\nold seg_seg_dist: max |new-old| {np.abs(err).max():.2e} m; "
          f"max |new-brute| {np.abs(d - brute).max():.2e}, max |old-brute| {np.abs(z['old'] - brute).max():.2e}")
    assert np.abs(err).max() < 1e-9


# --------------------------------------------------------------------------- scenes


def _scene(rng, n_box=32, n_cap=14, spread=1.2):
    boxes = []
    for i in range(n_box):
        T = np.eye(4)
        T[:3, :3] = _rot(rng, 1)[0]
        T[:3, 3] = rng.uniform(-spread, spread, 3)
        boxes.append(Box(f"box{i}", T, rng.uniform(0.02, 0.25, 3), 0.03))
    planes = (Plane("paper", np.array([0, 0, 1.0]), -0.9, 0.02, "paper", pen_margin=0.003),
              Plane("wall_x", np.array([1.0, 0, 0]), -1.3, 0.05, "wall"),
              Plane("wall_y", np.array([0, -1.0, 0]), -1.3, 0.05, "wall"))
    caps = tuple(Capsule(f"cap{i}", rng.uniform(-spread, spread, 3), rng.uniform(-spread, spread, 3),
                         0.06, 0.05) for i in range(n_cap))
    return Obstacles(tuple(boxes), planes, caps)


def _random_body(rng, N, K=12, spread=0.8):
    p0 = rng.uniform(-spread, spread, (N, K, 3))
    p1 = p0 + rng.normal(size=(N, K, 3)) * 0.12
    is_pen = np.zeros(K, bool)
    is_pen[-1] = True
    return Body(p0, p1, rng.uniform(0.02, 0.08, K), tuple(f"k{k}" for k in range(K)), is_pen)


# --------------------------------------------------------------------------- contract


def test_clearance_matches_pairwise_definition():
    """clearance == min over pairs of (surface distance - margin), written out by hand."""
    rng = np.random.default_rng(4)
    obs = _scene(rng, 5, 3)
    body = _random_body(rng, 50, K=4)
    N, K = body.p0.shape[:2]
    want = np.full(N, np.inf)
    for k in range(K):
        a, b, r = body.p0[:, k], body.p1[:, k], body.radius[k]
        for bx in obs.boxes:
            R, c = np.broadcast_to(bx.T_base_box[:3, :3], (N, 3, 3)), np.broadcast_to(bx.T_base_box[:3, 3], (N, 3))
            want = np.minimum(want, geo.segment_box_distance(a, b, R, c, np.broadcast_to(bx.half, (N, 3))) - r - bx.margin)
        for pl in obs.planes:
            m = pl.pen_margin if body.is_pen[k] and pl.pen_margin is not None else pl.margin
            want = np.minimum(want, np.minimum(a @ pl.normal, b @ pl.normal) - pl.offset - r - m)
        for cp in obs.capsules:
            d = geo.segment_segment_distance(a, b, np.broadcast_to(cp.p0, (N, 3)), np.broadcast_to(cp.p1, (N, 3)))
            want = np.minimum(want, d - r - cp.radius - cp.margin)
    got = collide.clearance(body, obs)
    assert np.allclose(got, want, atol=1e-12)
    det = collide.clearance_detail(body, obs)
    assert np.array_equal(det.value, got)
    assert set(det.obstacle_names) >= {"paper", "box0", "cap0"}


def test_capsule_through_box_and_plane():
    T = np.eye(4)
    obs = Obstacles(boxes=(Box("b", T, np.array([0.1, 0.1, 0.1]), 0.02),))
    body = Body(np.array([[[-1.0, 0, 0]]]), np.array([[[1.0, 0, 0]]]), np.array([0.05]), ("link",),
                np.array([False]))
    assert collide.clearance(body, obs)[0] == pytest.approx(-0.07)
    plane = Obstacles(planes=(Plane("paper", np.array([0, 0, 1.0]), 0.0, 0.02, "paper", 0.001),))
    pen = Body(np.array([[[0, 0, -0.004]]]), np.array([[[0, 0, 0.1]]]), np.array([0.005]), ("pen",),
               np.array([True]))
    assert collide.clearance(pen, plane)[0] == pytest.approx(-0.004 - 0.005 - 0.001)
    assert collide.clearance(pen, plane, drawing=True)[0] == np.inf
    link = Body(pen.p0, pen.p1, pen.radius, ("link",), np.array([False]))
    assert collide.clearance(link, plane, drawing=True)[0] == pytest.approx(-0.004 - 0.005 - 0.02)
    det = collide.clearance_detail(pen, plane, drawing=True)
    assert det.capsule[0] == -1 and det.obstacle[0] == -1


def test_fixed_capsules_are_left_out():
    rng = np.random.default_rng(11)
    obs = _cage_scene(rng)
    body = _chain_body(rng, 400)
    mount = Box("mount", np.eye(4), np.array([0.2, 0.2, 0.05]), 0.02)     # buries capsule 0
    obs = Obstacles(obs.boxes + (mount,), obs.planes, obs.capsules)
    fixed = np.zeros(12, bool)
    fixed[0] = True
    with_fixed = Body(body.p0, body.p1, body.radius, body.names, body.is_pen, fixed)
    rest = Body(body.p0[:, 1:], body.p1[:, 1:], body.radius[1:], body.names[1:], body.is_pen[1:])
    got = collide.clearance_detail(with_fixed, obs)
    want = collide.clearance_detail(rest, obs)
    assert np.all(collide.clearance(body, obs) < 0)              # the buried capsule, unmarked
    assert np.array_equal(got.value, want.value)
    assert np.array_equal(got.capsule, want.capsule + 1) and np.array_equal(got.obstacle, want.obstacle)
    assert np.all(collide.capsule_clearance(with_fixed, obs)[:, 0] == np.inf)
    none = Body(body.p0, body.p1, body.radius, body.names, body.is_pen, None)
    all_false = Body(body.p0, body.p1, body.radius, body.names, body.is_pen, np.zeros(12, bool))
    assert np.array_equal(collide.clearance(none, obs), collide.clearance(all_false, obs))
    # along a path too: the toy arm with its first link fixed inside a box
    T = np.eye(4)
    T[:3, 3] = [0, 0, 0.15]
    toy_obs = Obstacles(_toy_scene().boxes + (Box("mount", T, np.array([0.1, 0.1, 0.2]), 0.02),),
                        _toy_scene().planes, _toy_scene().capsules)

    def fixed_body(Q):
        b = _toy_body(Q)
        f = np.zeros(8, bool)
        f[0] = True
        return Body(b.p0, b.p1, b.radius, b.names, b.is_pen, f)
    q = _paths()[0]
    b = collide.path_clearance(fixed_body, q, toy_obs, _toy_reach())
    truth = collide.clearance(fixed_body(_resample(q, 2000)), toy_obs).min()
    assert truth - 5e-4 - 1e-9 <= b <= truth


def test_self_clearance():
    rng = np.random.default_rng(5)
    body = _random_body(rng, 200, K=6)
    pairs = np.array([[0, 2], [1, 4], [3, 5], [0, 5]])
    got = collide.self_clearance(body, pairs, 0.023)
    want = np.min([geo.segment_segment_distance(body.p0[:, i], body.p1[:, i], body.p0[:, j], body.p1[:, j])
                   - body.radius[i] - body.radius[j] - 0.023 for i, j in pairs], axis=0)
    assert np.array_equal(got, want)


# --------------------------------------------------------------------------- 4. pruning


@pytest.mark.parametrize("spread", [0.4, 0.8, 1.5])
def test_pruning_changes_nothing(spread):
    rng = np.random.default_rng(6)
    obs = _scene(rng)
    body = _random_body(rng, 3000, spread=spread)
    for drawing in (False, True):
        a = collide.clearance_detail(body, obs, drawing=drawing, prune=True)
        b = collide.clearance_detail(body, obs, drawing=drawing, prune=False)
        assert np.array_equal(a.value, b.value)
        assert np.array_equal(a.capsule, b.capsule)
        assert np.array_equal(a.obstacle, b.obstacle)
        pc = collide.capsule_clearance(body, obs, drawing, prune=True)
        full = collide.capsule_clearance(body, obs, drawing, prune=False)
        assert np.all(pc <= full)                      # pruned entries are lower bounds


# --------------------------------------------------------------------------- 3. along a path
# A made-up 7-joint arm: one capsule per link between successive joint origins, plus a pen.
# `reach[j, k]` = sum of the link lengths from joint j to the far end of capsule k, which bounds
# the distance of any point of capsule k from joint j's axis.

_AXES = np.array([[0, 0, 1], [0, 1, 0], [0, 0, 1], [0, -1, 0], [0, 0, 1], [0, -1, 0], [0, 0, 1.0]])
_LINKS = np.array([[0, 0, 0.33], [0, 0, 0.0], [0.08, 0, 0.31], [-0.08, 0, 0], [0, 0, 0.38], [0.09, 0, 0],
                   [0, 0, 0.1]])
_PEN = np.array([0.086, 0, 0.12])


def _axis_rot(axis, th):
    th = np.asarray(th)[..., None, None]
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _toy_body(Q):
    Q = np.atleast_2d(Q)
    N = len(Q)
    R, p = np.broadcast_to(np.eye(3), (N, 3, 3)), np.zeros((N, 3))
    pts = []
    for j in range(7):
        R = R @ _axis_rot(_AXES[j], Q[:, j])
        q = p + np.einsum("nij,j->ni", R, _LINKS[j])
        pts.append((p, q))
        p = q
    pts.append((p, p + np.einsum("nij,j->ni", R, _PEN)))
    p0 = np.stack([a for a, _ in pts], 1)
    p1 = np.stack([b for _, b in pts], 1)
    is_pen = np.zeros(8, bool)
    is_pen[-1] = True
    return Body(p0, p1, np.full(8, 0.05), tuple(f"l{k}" for k in range(8)), is_pen)


def _toy_reach():
    lens = np.r_[np.linalg.norm(_LINKS, axis=1), np.linalg.norm(_PEN)]
    R = np.zeros((7, 8))
    for j in range(7):
        for k in range(j, 8):
            R[j, k] = lens[j:k + 1].sum()
    return R


def _toy_scene():
    T = np.eye(4)
    T[:3, 3] = [0.45, 0.2, 0.55]
    T2 = np.eye(4)
    T2[:3, 3] = [-0.3, -0.45, 0.7]
    return Obstacles(boxes=(Box("post", T, np.array([0.05, 0.05, 0.3]), 0.03),
                            Box("block", T2, np.array([0.1, 0.1, 0.1]), 0.03)),
                     planes=(Plane("paper", np.array([0, 0, 1.0]), -0.3, 0.02, "paper", 0.003),),
                     capsules=(Capsule("strut", np.array([0.5, -0.5, 0.9]), np.array([-0.5, 0.5, 1.1]),
                                       0.03, 0.05),))


def _resample(q, per):
    s = np.linspace(0, 1, per + 1)[:-1]
    out = [q[i] + s[:, None] * (q[i + 1] - q[i]) for i in range(len(q) - 1)]
    return np.vstack(out + [q[-1:]])


def _paths():
    rng = np.random.default_rng(7)
    base = np.array([0.0, 0.4, 0.0, -1.8, 0.0, 2.0, 0.6])
    return [base + np.cumsum(rng.normal(size=(5, 7)) * 0.35, axis=0) for _ in range(6)]


def test_path_clearance_independent_of_sampling():
    obs, reach, tol = _toy_scene(), _toy_reach(), 5e-4
    spreads = []
    for q in _paths():
        dense = _resample(q, 4000)
        truth = collide.clearance(_toy_body(dense), obs).min()
        bounds = [collide.path_clearance(_toy_body, _resample(q, per), obs, reach, tol=tol)
                  for per in (1, 3, 10, 40)]
        spreads.append(max(bounds) - min(bounds))
        assert max(bounds) - min(bounds) <= tol
        assert max(bounds) <= truth + 1e-12            # a lower bound, never above the truth
        assert truth - min(bounds) <= tol + 1e-5       # and within tol of it
    print(f"\npath_clearance: largest spread over 4 densities {max(spreads) * 1e3:.3f} mm (tol 0.5 mm)")


def test_still_arm_is_charged_nothing():
    obs, reach = _toy_scene(), _toy_reach()
    q = np.tile(_paths()[0][2], (5, 1))
    assert collide.path_clearance(_toy_body, q, obs, reach) == collide.clearance(_toy_body(q[:1]), obs)[0]


def test_moving_only_the_last_joint_charges_only_what_it_moves():
    obs, reach = _toy_scene(), _toy_reach()
    q = np.tile(_paths()[0][1], (2, 1))
    q[1, 6] += 0.5
    delta = np.abs(q[1] - q[0]) @ reach
    assert np.all(delta[:6] == 0.0) and delta[6] > 0 and delta[7] > 0
    truth = collide.clearance(_toy_body(_resample(q, 2000)), obs).min()
    b = collide.path_clearance(_toy_body, q, obs, reach)
    assert truth - 5e-4 - 1e-9 <= b <= truth


# --------------------------------------------------------------------------- 5. speed


def _cage_scene(rng):
    """Rig-like: struts on a shell 1.3-1.8 m out, the paper, two walls, a parked arm nearby."""
    boxes = []
    for i in range(32):
        T = np.eye(4)
        T[:3, :3] = _rot(rng, 1)[0]
        u = rng.normal(size=3)
        T[:3, 3] = u / np.linalg.norm(u) * rng.uniform(1.3, 1.8)
        boxes.append(Box(f"strut{i}", T, np.array([0.03, 0.03, rng.uniform(0.2, 0.6)]), 0.03))
    planes = (Plane("paper", np.array([0, 0, 1.0]), -0.95, 0.02, "paper", 0.003),
              Plane("wall_a", np.array([1.0, 0, 0]), -1.1, 0.05, "wall"),
              Plane("wall_b", np.array([-1.0, 0, 0]), -1.1, 0.05, "wall"))
    pts = np.array([0, 0.61, 0]) + np.cumsum(np.r_[[np.zeros(3)], rng.normal(size=(14, 3)) * 0.08], 0)
    caps = tuple(Capsule(f"parked{i}", pts[i], pts[i + 1], 0.06, 0.05) for i in range(14))
    return Obstacles(tuple(boxes), planes, caps)


def _chain_body(rng, N, K=12):
    """An arm-like chain of K capsules from the origin, hanging roughly downwards."""
    steps = rng.normal(size=(N, K, 3))
    steps *= 0.09 / np.linalg.norm(steps, axis=2, keepdims=True)
    steps[:, :, 2] -= 0.03
    pts = np.cumsum(np.concatenate([np.zeros((N, 1, 3)), steps], 1), 1)
    is_pen = np.zeros(K, bool)
    is_pen[-1] = True
    return Body(pts[:, :-1], pts[:, 1:], np.full(K, 0.05), tuple(f"k{k}" for k in range(K)), is_pen)


def test_pruning_changes_nothing_rig_like():
    rng = np.random.default_rng(10)
    obs, body = _cage_scene(rng), _chain_body(rng, 3000)
    for drawing in (False, True):
        a = collide.clearance_detail(body, obs, drawing=drawing, prune=True)
        b = collide.clearance_detail(body, obs, drawing=drawing, prune=False)
        assert np.array_equal(a.value, b.value) and np.array_equal(a.obstacle, b.obstacle)
        assert np.array_equal(a.capsule, b.capsule)


@pytest.mark.slow
def test_speed_report():
    rng = np.random.default_rng(8)
    scenes = {"rig-like (98 % free)": (_cage_scene(rng), lambda n: _chain_body(rng, n)),
              "crowded (all in contact)": (_scene(rng), lambda n: _random_body(rng, n))}
    rows = []
    for label, (obs, make) in scenes.items():
        pairs = 12 * (len(obs.boxes) + len(obs.planes) + len(obs.capsules))
        for prune in (False, True):
            for N in (1, 100, 10_000):
                body = make(N)
                reps = max(1, 2000 // N)
                collide.clearance(body, obs, prune=prune, backend="numpy")
                t = time.process_time()
                for _ in range(reps):
                    collide.clearance(body, obs, prune=prune, backend="numpy")
                dt = (time.process_time() - t) / reps
                rows.append((label, prune, N, N / dt, N * pairs / dt))
    print(f"\nspeed, numpy, one core, 12 capsules vs 32 boxes + 3 planes + 14 capsules = {pairs} pairs per configuration")
    for label, prune, N, cps, pps in rows:
        print(f"  {label:26} prune={prune!s:5} batch {N:6d}: {cps:9.0f} configurations/s, "
              f"{pps / 1e6:6.2f} M pairs/s")
    assert all(r[3] > 1000 for r in rows)            # ten times below the slowest seen (2 200/s)


# --------------------------------------------------------------------------- the real arm
# Only if kernel/arm.py is installed.  `Arm.reach` bounds how far each capsule moves per joint.


def _real_arm():
    arm_mod = pytest.importorskip("aris.kernel.arm")
    return arm_mod.Arm(arm_mod.default_tool())


def _real_scene(arm, q_paths):
    tips = arm.tip(np.vstack(q_paths))
    z0 = tips[:, 2].min() - 0.03
    T = np.eye(4)
    T[:3, 3] = [0.35, 0.35, 0.3]
    T2 = np.eye(4)
    T2[:3, :3] = _rot(np.random.default_rng(12), 1)[0]
    T2[:3, 3] = [-0.1, -0.5, 0.6]
    return Obstacles(boxes=(Box("strut", T, np.array([0.03, 0.03, 0.5]), 0.03),
                            Box("block", T2, np.array([0.1, 0.15, 0.1]), 0.03)),
                     planes=(Plane("paper", np.array([0, 0, 1.0]), z0, 0.02, "paper", 0.003),),
                     capsules=(Capsule("parked", np.array([0.2, -0.6, 0.2]), np.array([0.5, -0.4, 0.8]),
                                       0.06, 0.05),))


def test_real_arm_path_clearance():
    arm = _real_arm()
    reach = arm.reach
    rng = np.random.default_rng(13)
    home = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8])
    paths = []
    for _ in range(4):
        q = home + np.cumsum(rng.normal(size=(4, 7)) * 0.3, axis=0)
        paths.append(np.clip(q, arm.limits.q_min, arm.limits.q_max))
    obs, tol = _real_scene(arm, paths), 5e-4
    for q in paths:
        truth = collide.clearance(arm.body(_resample(q, 3000)), obs).min()
        bounds = [collide.path_clearance(arm.body, _resample(q, per), obs, reach, tol=tol)
                  for per in (1, 4, 16, 64)]
        assert max(bounds) - min(bounds) <= tol
        assert max(bounds) <= truth + 1e-12
        assert truth - min(bounds) <= tol + 1e-5


@pytest.mark.slow
def test_real_arm_speed_report():
    arm = _real_arm()
    rng = np.random.default_rng(14)
    obs = _cage_scene(rng)
    live = int((~arm.body(np.zeros((1, 7))).is_fixed).sum())
    pairs = live * (len(obs.boxes) + len(obs.planes) + len(obs.capsules))
    print(f"\nnumpy, real FR3 body ({live} capsules checked) vs 32 boxes + 3 planes + 14 capsules = {pairs} pairs")
    for N in (1, 100, 10_000):
        Q = rng.uniform(arm.limits.q_min, arm.limits.q_max, (N, 7))
        body = arm.body(Q)
        reps = max(1, 1000 // N)
        collide.clearance(body, obs, backend="numpy")
        t = time.process_time()
        for _ in range(reps):
            collide.clearance(body, obs, backend="numpy")
        dt = (time.process_time() - t) / reps
        print(f"  batch {N:6d}: {N / dt:9.0f} configurations/s, {N * pairs / dt / 1e6:6.2f} M pairs/s")
        assert N / dt > 200                             # ten times below the slowest seen


def test_without_the_compiled_module(monkeypatch):
    """With aris_collide_native missing, everything runs on numpy; asking for native refuses."""
    monkeypatch.setattr(collide_native, "_native", None)
    assert collide.backend() == "numpy"
    rng = np.random.default_rng(15)
    obs, body = _cage_scene(rng), _chain_body(rng, 50)
    assert collide.clearance(body, obs).shape == (50,)
    with pytest.raises(ValueError):
        collide.clearance(body, obs, backend="native")
    arm = _real_arm()
    T = collide.arm_tables(arm)
    Q = np.tile(np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8]), (3, 1))
    assert np.array_equal(collide.clearance_q(T, Q, obs), collide.clearance(arm.body(Q), obs))


# --------------------------------------------------------------------------- tool margin


@pytest.mark.parametrize("backend", ["numpy", "native"])
def test_tool_margin_against_planes(backend):
    if backend == "native" and collide.backend() != "native":
        pytest.skip("compiled module not installed")
    r = 0.01
    cap = dict(p0=np.array([[[0.0, 0, 0.003 + r]]]), p1=np.array([[[0.1, 0, 0.003 + r]]]),
               radius=np.array([r]), names=("holder",), is_pen=np.array([False]))
    tool = Body(**cap, is_tool=np.array([True]))
    plain = Body(**cap)
    paper = lambda tm: Obstacles(planes=(Plane("paper", np.array([0, 0, 1.0]), 0.0, 0.020, "paper",
                                               pen_margin=0.001, tool_margin=tm),))
    for drawing in (False, True):                      # drawing never takes the tool out
        assert collide.clearance(tool, paper(0.002), drawing, backend=backend)[0] == pytest.approx(0.001)
        assert collide.clearance(tool, paper(None), drawing, backend=backend)[0] == pytest.approx(-0.017)
        assert collide.clearance(plain, paper(0.002), drawing, backend=backend)[0] == pytest.approx(-0.017)
    # a box does not care about tool_margin
    box = Obstacles(boxes=(Box("b", np.eye(4), np.array([0.05, 0.05, 0.05]), 0.02),))
    assert np.array_equal(collide.clearance(tool, box, backend=backend),
                          collide.clearance(plain, box, backend=backend))


def test_is_tool_none_changes_nothing():
    rng = np.random.default_rng(16)
    obs, body = _cage_scene(rng), _chain_body(rng, 500)
    none = collide.clearance_detail(body, obs)
    zeros = collide.clearance_detail(Body(body.p0, body.p1, body.radius, body.names, body.is_pen,
                                          None, np.zeros(12, bool)), obs)
    assert np.array_equal(none.value, zeros.value) and np.array_equal(none.obstacle, zeros.obstacle)


# --------------------------------------------------------------------------- nothing to decide


class _Counting:
    """body_of that counts how many configurations it was asked for."""
    def __init__(self, body_of):
        self.body_of, self.n = body_of, 0

    def __call__(self, Q):
        self.n += len(np.atleast_2d(Q))
        return self.body_of(Q)


def test_path_with_nothing_to_hit_returns_inf_at_once():
    q = _resample(_paths()[0], 3)
    reach = _toy_reach()
    for obs, drawing in [(Obstacles(), False),
                         (Obstacles(planes=(Plane("paper", np.array([0, 0, 1.0]), -0.3, 0.02,
                                                  "paper"),)), True)]:
        def pen_only(Q):
            b = _toy_body(Q)
            fixed = ~b.is_pen                           # only the pen is checked ...
            return Body(b.p0, b.p1, b.radius, b.names, b.is_pen, fixed)
        for backend in ("numpy", "native"):
            if backend == "native" and collide.backend() != "native":
                continue
            body_of = _Counting(pen_only)
            got = collide.path_clearance(body_of, q, obs, reach, drawing=drawing, backend=backend)
            assert got == np.inf                        # ... and drawing takes it off the paper
            assert body_of.n <= len(q)
    body_of = _Counting(_toy_body)
    assert collide.path_self_clearance(body_of, q, np.zeros((0, 2)), 0.02, reach) == np.inf
    assert body_of.n == 0


def test_path_far_from_everything_costs_one_evaluation_per_sample():
    far = Obstacles(planes=(Plane("floor", np.array([0, 0, 1.0]), -3.0, 0.02),))
    reach = _toy_reach()
    for q in _paths():
        q = _resample(q, 4)
        body_of = _Counting(_toy_body)
        b = collide.path_clearance(body_of, q, far, reach, backend="numpy")
        truth = collide.clearance(_toy_body(q), far).min()
        assert collide.CAP - 5e-4 <= b <= truth and body_of.n <= len(q)
    # the arm against itself, two capsules that never come near each other: base and pen
    q = np.array([0.0, 0.0, 0.0, -0.2, 0.0, 0.3, 0.0]) + np.linspace(0, 0.2, 9)[:, None]
    body_of = _Counting(_toy_body)
    b = collide.path_self_clearance(body_of, q, np.array([[0, 7]]), 0.02, reach)
    assert b >= collide.CAP - 5e-4 and body_of.n <= len(q)
