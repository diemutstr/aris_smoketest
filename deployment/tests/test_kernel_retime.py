"""Tests of the timing step.  Run with -s to see the measured numbers."""
from __future__ import annotations

import functools
import time

import numpy as np
import pytest

from aris.kernel.retime import RetimeResult, check, retime, retime_detailed, sample
from aris.types import DrawRules, JointPath, Limits, Refusal

# FR3 position limits, as in aris_sixarm/frames.py (FR3_MIN, FR3_MAX)
Q_MIN = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159])
Q_MAX = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
LIMITS = Limits(Q_MIN, Q_MAX, np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]),
                np.full(7, 10.0), np.full(7, 5000.0))
RULES = DrawRules()
MID = 0.5 * (Q_MIN + Q_MAX)
ACCEL_FRACTION = JERK_FRACTION = 0.9          # retime's defaults
SPEED_CEILING_S = 2.5    # CPU s: ten times the 242 ms measured on a quiet run


# --------------------------------------------------------------------------- test paths


def corner_90():
    a, b, c = MID.copy(), MID.copy(), MID.copy()
    b[0] += 0.6
    c[0] += 0.6
    c[1] += 0.6
    return np.array([a, b, c])


def near_reversal():
    """Out along joint 2, back almost the way it came (175 degree turn), with a wrist move."""
    a = MID.copy()
    b = a + np.array([0.0, 0.0, 0.8, 0, 0.3, 0, 0])
    d = b - a
    side = np.zeros(7)
    side[3] = np.linalg.norm(d) * np.tan(np.deg2rad(5.0))
    return np.array([a, b, a + side])


def random_zigzag(seed, n=9):
    rng = np.random.default_rng(seed)
    return MID + rng.uniform(-0.6, 0.6, (n, 7))


def to_the_limit():
    """Two corners sitting exactly on joint limits."""
    a = MID.copy()
    b = MID.copy()
    b[0] = Q_MAX[0]
    c = b.copy()
    c[3] = Q_MIN[3]
    return np.array([a, b, c, MID])


def dense_wander(seed, n=2000):
    """A dense, jittery path: small steps whose direction drifts, with occasional kinks."""
    rng = np.random.default_rng(seed)
    d = np.cumsum(rng.normal(0, 0.05, (n - 1, 7)), axis=0)
    d = 0.002 * d / np.linalg.norm(d, axis=1, keepdims=True)
    return MID + np.concatenate([np.zeros((1, 7)), np.cumsum(d, axis=0)])


FREE_CASES = {
    "corner_90": corner_90(),
    "near_reversal": near_reversal(),
    "zigzag_1": random_zigzag(1),
    "zigzag_2": random_zigzag(2),
    "zigzag_3": random_zigzag(3),
    "to_the_limit": to_the_limit(),
}

# Drawing: a synthetic arm whose joints are a fixed linear map of the pen position on the paper,
# so the pen position can be read back exactly from the joints.
_rng = np.random.default_rng(7)
MAP = _rng.normal(size=(7, 2))
MAP *= 1.5 / np.linalg.norm(MAP, axis=0)       # 1.5 rad per metre along each paper axis


def pen(q):
    return np.linalg.lstsq(MAP, (q - MID).T, rcond=None)[0].T


def draw_case(p):
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    return MID + p @ MAP.T, s


def circle():
    th = np.linspace(0, 2 * np.pi, 1500)
    return 0.1 * np.c_[np.cos(th), np.sin(th)]


def square():
    return np.array([[0, 0], [0.1, 0], [0.1, 0.1], [0, 0.1], [0, 0.0]])


def zigzag_line():
    x = np.linspace(0, 0.12, 13)
    return np.c_[x, 0.01 * (np.arange(13) % 2)]


DRAW_CASES = {"circle": circle(), "square": square(), "zigzag": zigzag_line()}


@functools.cache                   # each case is timed once per test session
def _free(name):
    r = retime_detailed(JointPath(FREE_CASES[name]), LIMITS, RULES)
    assert isinstance(r, RetimeResult), r
    return r


@functools.cache
def _draw(name):
    q, s = draw_case(DRAW_CASES[name])
    r = retime_detailed(JointPath(q), LIMITS, RULES, s=s)
    assert isinstance(r, RetimeResult), r
    return r, q, s


# --------------------------------------------------------------------------- 1. inside the limits at 1 kHz


@pytest.mark.parametrize("name", list(FREE_CASES))
def test_free_inside_limits_at_1khz(name):
    r = _free(name)
    rep = check(r.traj, LIMITS, 1000.0)
    v = np.max(rep.qd_peak / (RULES.speed_fraction * LIMITS.qd_max))
    a, j = rep.qdd_ratio.max(), rep.qddd_ratio.max()
    print(f"\n{name:14s} T={r.traj.t[-1]:6.3f}s  vel/(0.3 limit)={v:.4f}  acc/limit={a:.4f}  "
          f"jerk/limit={j:.4f}  deviation={r.deviation * 1e3:.4f} mrad  stretch={r.stretch:.4f}")
    assert v <= 1.0
    assert a <= ACCEL_FRACTION
    assert j <= JERK_FRACTION
    assert rep.inside


@pytest.mark.parametrize("name", list(DRAW_CASES))
def test_draw_inside_limits_at_1khz(name):
    r, _, _ = _draw(name)
    rep = check(r.traj, LIMITS, 1000.0)
    assert np.max(rep.qd_peak / (RULES.speed_fraction * LIMITS.qd_max)) <= 1.0
    assert rep.qdd_ratio.max() <= ACCEL_FRACTION and rep.qddd_ratio.max() <= JERK_FRACTION


# --------------------------------------------------------------------------- 2. rate independence


@pytest.mark.parametrize("name", ["corner_90", "near_reversal", "zigzag_1"])
def test_measurement_does_not_depend_on_rate(name):
    r = _free(name)
    reps = {hz: check(r.traj, LIMITS, hz) for hz in (100.0, 1000.0, 4000.0)}
    rows = {hz: np.array([rep.qd_peak.max(), rep.qdd_peak.max(), rep.qddd_peak.max()])
            for hz, rep in reps.items()}
    print(f"\n{name}: peaks (vel, acc, jerk) " +
          "  ".join(f"{int(hz)} Hz {np.round(v, 3)}" for hz, v in rows.items()))
    fine, finest, coarse = rows[1000.0], rows[4000.0], rows[100.0]
    assert np.all(np.abs(fine - finest) <= 0.02 * finest)          # converged at 1 kHz
    assert np.all(np.abs(coarse - finest) <= 0.10 * finest)        # and even at 100 Hz
    assert np.all(coarse <= finest * 1.001)                        # a coarse look never reads more


# --------------------------------------------------------------------------- 3. drawing speed


@pytest.mark.parametrize("name", list(DRAW_CASES))
def test_draw_speed(name):
    r, q_in, s_in = _draw(name)
    t = np.arange(0.0, r.traj.t[-1], 1e-3)
    p = pen(sample(r.traj, t)[0])
    speed = np.linalg.norm(np.diff(p, axis=0), axis=1) / 1e-3
    s_t = np.interp(t, r.traj.t, r.s)
    # "where no limit binds": away from the two ends and from the corners of the drawn line
    corners = s_in[1:-1] if name != "circle" else np.array([])
    far = (s_t[1:] > 0.01) & (s_t[1:] < s_in[-1] - 0.01)
    for c in corners:
        far &= np.abs(s_t[1:] - c) > 0.005
    err = np.max(np.abs(speed[far] / RULES.draw_speed - 1.0))
    ideal = s_in[-1] / RULES.draw_speed
    print(f"\n{name:8s} T={r.traj.t[-1]:.3f}s (line/draw speed {ideal:.3f}s)  "
          f"speed error on the free stretches {err * 100:.3f} %  slowest interior pen speed "
          f"{speed[5:-5].min() * 1e3:.4f} mm/s  deviation {r.deviation * 1e3:.4f} mrad")
    assert err <= 0.02
    assert np.all(speed[1:-1] > 0.0)
    assert np.all(np.diff(r.s) > 0.0)
    assert r.s[0] == s_in[0] and abs(r.s[-1] - s_in[-1]) < 1e-12


def test_draw_slows_where_a_joint_limit_binds():
    """A stretch where the joints move 150 rad per metre: the velocity limit, not the draw speed."""
    p = np.c_[np.linspace(0, 0.1, 201), np.zeros(201)]
    q, s = draw_case(p)
    gain = np.where((s > 0.045) & (s < 0.055), 100.0, 1.0)
    q = MID + np.cumsum(np.r_[np.zeros((1, 7)), np.diff(q, axis=0) * gain[1:, None]], axis=0)
    r = retime_detailed(JointPath(q), LIMITS, RULES, s=s)
    assert isinstance(r, RetimeResult)
    sd = np.diff(r.s) / np.diff(r.traj.t)
    mid = (r.s[1:] > 0.045) & (r.s[1:] < 0.055)
    print(f"\nlimit-bound stretch: pen speed {sd[mid].max() * 1e3:.2f} mm/s there, "
          f"{np.median(sd) * 1e3:.2f} mm/s median")
    assert sd[mid].max() < 0.9 * RULES.draw_speed
    assert np.all(sd > 0.0)
    assert check(r.traj, LIMITS).qd_ratio.max() <= RULES.speed_fraction * 1.0005


# --------------------------------------------------------------------------- 4. deviation


def test_deviation_within_budget():
    budget = 1.5e-4
    for name in FREE_CASES:
        dev = _free(name).deviation
        print(f"\n{name:14s} deviation {dev * 1e3:.4f} mrad")
        assert dev <= budget
    for name, p in DRAW_CASES.items():
        r, q, s = _draw(name)
        t = np.arange(0.0, r.traj.t[-1], 1e-3)
        s_t = np.interp(t, r.traj.t, r.s)
        want = np.c_[np.interp(s_t, s, p[:, 0]), np.interp(s_t, s, p[:, 1])]
        pen_err = np.max(np.linalg.norm(pen(sample(r.traj, t)[0]) - want, axis=1))
        print(f"\n{name:14s} deviation {r.deviation * 1e3:.4f} mrad, pen {pen_err * 1e3:.4f} mm "
              f"(synthetic arm, 1.5 rad/m)")
        assert r.deviation <= budget
    tight = retime_detailed(JointPath(corner_90()), LIMITS, RULES, deviation=2e-5)
    assert isinstance(tight, RetimeResult) and tight.deviation <= 2e-5


# --------------------------------------------------------------------------- 5. ends


@pytest.mark.parametrize("name", list(FREE_CASES))
def test_ends_at_rest_and_exact(name):
    q_in = FREE_CASES[name]
    traj = _free(name).traj
    assert np.all(traj.qd[0] == 0.0) and np.all(traj.qd[-1] == 0.0)
    assert np.max(np.abs(traj.q[0] - q_in[0])) <= 1e-12
    assert np.max(np.abs(traj.q[-1] - q_in[-1])) <= 1e-12
    q, qd, qdd = sample(traj, np.array([traj.t[0], traj.t[-1]]))
    assert np.max(np.abs(qd)) == 0.0 and np.max(np.abs(qdd)) < 1e-2


# --------------------------------------------------------------------------- 6. determinism


def test_bit_identical():
    q, s = draw_case(square())
    for args in ((JointPath(near_reversal()), LIMITS, RULES), (JointPath(q), LIMITS, RULES, s)):
        a, b = retime(*args), retime(*args)
        assert a.t.tobytes() == b.t.tobytes()
        assert a.q.tobytes() == b.q.tobytes()
        assert a.qd.tobytes() == b.qd.tobytes()


# --------------------------------------------------------------------------- 7. speed


def test_speed_2000_samples():
    wander = dense_wander(11)
    q, s = draw_case(np.c_[0.1 * np.cos(np.linspace(0, 6, 2000)),
                           0.05 * np.sin(np.linspace(0, 17, 2000))])
    for label, args in (("free, 2000 samples", (JointPath(wander), LIMITS, RULES)),
                        ("draw, 2000 samples", (JointPath(q), LIMITS, RULES, s))):
        retime(*args)
        t0 = time.process_time()          # CPU time: the machine is shared and often loaded
        r = retime_detailed(*args)
        cpu = time.process_time() - t0
        print(f"\n{label}: {cpu * 1e3:.0f} ms CPU, trajectory {r.traj.t[-1]:.2f} s, "
              f"{len(r.traj.t)} samples, deviation {r.deviation * 1e3:.4f} mrad")
        assert cpu < SPEED_CEILING_S
        assert check(r.traj, LIMITS).inside


# --------------------------------------------------------------------------- 8. refusals


def test_refusals():
    one = retime(JointPath(MID[None, :]), LIMITS, RULES)
    same = retime(JointPath(np.repeat(MID[None, :], 5, axis=0)), LIMITS, RULES)
    bad = corner_90()
    bad[1, 3] = Q_MAX[3] + 0.01
    out = retime(JointPath(bad), LIMITS, RULES)
    nan = corner_90()
    nan[1, 0] = np.nan
    back = retime(JointPath(corner_90()), LIMITS, RULES, s=np.array([0.0, 0.02, 0.01]))
    stall = retime(JointPath(corner_90()), LIMITS, RULES, s=np.array([0.0, 0.0, 0.01]))
    for r, reason in ((one, "too_few_samples"), (same, "no_motion"), (out, "outside_limits"),
                      (retime(JointPath(nan), LIMITS, RULES), "not_finite"),
                      (back, "bad_arc_length"), (stall, "bad_arc_length")):
        assert isinstance(r, Refusal) and r.reason == reason, r
        assert r.detail


def test_repeated_samples_inside_a_path_are_fine():
    q = corner_90()
    q = np.array([q[0], q[1], q[1], q[1], q[2]])
    traj = retime(JointPath(q), LIMITS, RULES)
    assert not isinstance(traj, Refusal)
    assert check(traj, LIMITS).inside
