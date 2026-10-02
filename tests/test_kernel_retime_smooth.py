"""The timing step with `smooth=True`: input samples that lie on a smooth curve (drawing
motions, lowers, lifts).  Run with -s for the numbers.

The fixed case is the plan of line "big:8859" of tests/data/server_big.json for arm 13 (phase 1
obstacles), as the local planner returned it on 2026-09-30 (tests/data/retime_smooth_arc.npz):
a 10.6 cm polyline of 12 vertices, 9.7 mm apart, turning 10.7 degrees at each, sampled by the
planner every 0.25 to 2 mm (109 samples).  With the corner model it takes 7.6 s and the pen
slows to 1.7 mm/s mid-line.
"""
from __future__ import annotations

import hashlib
import time
from pathlib import Path

import numpy as np
import pytest

from aris.kernel.retime import RetimeResult, check, retime, retime_detailed, sample
from aris.kernel.spline import _pen_turn
from aris.rig import Rig
from aris.types import DrawRules, JointPath, Limits, Refusal

ROOT = Path(__file__).resolve().parents[1]
ARC = np.load(ROOT / "tests" / "data" / "retime_smooth_arc.npz")
LIMITS = Limits(ARC["q_min"], ARC["q_max"], ARC["qd_max"], ARC["qdd_max"], ARC["qddd_max"])
RULES = DrawRules(draw_speed=float(ARC["draw_speed"]), speed_fraction=float(ARC["speed_fraction"]))
ARM = Rig.load(ROOT / "config").arm("1L")
ALONG = np.abs(ARC["s"] - ARC["s"][0])
# The corner model's answer on the arc, recorded with the code as it was before `smooth`
# existed: smooth=False must keep giving exactly this.
CORNER_DIGEST = "a18ffa128330358b056cefd05c784ef201b2573e"


def _time(q, s, smooth, tip_of=ARM.tip, limits=LIMITS, **kw):
    r = retime_detailed(JointPath(q), limits, RULES, s=s, tip_of=tip_of, smooth=smooth, **kw)
    assert isinstance(r, RetimeResult), r
    return r


def pen_along(r, along, tip_base, tip_of=ARM.tip):
    """(t, speed along the line) at 1 kHz, measured as aris/check/drawing.py does: the planned
    pen positions at the timed samples; the flown tip's position along the chord between the
    two timed samples around it; that fraction of the arc length between them."""
    P = np.column_stack([np.interp(r.s, along, tip_base[:, i]) for i in range(3)])
    s_at = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    t = np.arange(0.0, r.traj.t[-1], 1e-3)
    t = np.append(t[t < r.traj.t[-1] - 1e-9], r.traj.t[-1])
    x = tip_of(sample(r.traj, t)[0])
    knots = tip_of(r.traj.q)
    i = np.clip(np.searchsorted(r.traj.t, t, side="right") - 1, 0, len(P) - 2)
    a, d = knots[i], knots[i + 1] - knots[i]
    dd = np.einsum("mi,mi->m", d, d)
    u = np.clip(np.einsum("mi,mi->m", x - a, d) / np.where(dd > 0, dd, 1.0), 0.0, 1.0)
    s = s_at[i] + u * (s_at[i + 1] - s_at[i])
    return t[1:], np.diff(s) / np.diff(t), x


def slowest(v, share=0.25):
    """The checker's slowest mid-line speed: between the pen first reaching `share` of its
    fastest speed and last being there."""
    going = np.flatnonzero(v >= share * v.max())
    return float(v[going[0]:going[-1] + 1].min())


def off_line(x, tip_base):
    """Largest distance of flown tips x from the planned pen polyline."""
    a, b = tip_base[:-1], tip_base[1:]
    d = b - a
    u = np.clip(np.einsum("mki,ki->mk", x[:, None] - a, d) / np.einsum("ki,ki->k", d, d), 0, 1)
    return float(np.min(np.linalg.norm(a + u[..., None] * d - x[:, None], axis=-1), axis=1).max())


def tip_speed(r, tip_of=ARM.tip):
    t = np.arange(0.0, r.traj.t[-1], 1e-3)
    return np.linalg.norm(np.diff(tip_of(sample(r.traj, t)[0]), axis=0), axis=1) / 1e-3


# --------------------------------------------------------------------------- 1. the arc


def test_arc():
    """Acceptance 1.  The pen keeps above 15 mm/s along the line wherever it has once reached
    15 mm/s, except at one spot where joint 5's acceleration binds (s = 48.3 mm, where a
    10.7 degree vertex of the line meets joints moving 65 rad per metre of line)."""
    corner = _time(ARC["q"], ALONG, False)
    smooth = _time(ARC["q"], ALONG, True)
    rows = {}
    for name, r in (("corners", corner), ("smooth", smooth)):
        t, v, x = pen_along(r, ALONG, ARC["tip_base"])
        above = np.flatnonzero(v >= 0.015)
        mid = v[above[0]:above[-1] + 1]
        k = above[0] + int(np.argmin(mid))
        s_k = float(np.interp(t[k], r.traj.t, r.s))
        rows[name] = (r, t, v, x, mid, s_k)
        print(f"\n{name:8s} T={r.traj.t[-1]:.3f} s  slowest after 5 mm/s (checker) "
              f"{slowest(v) * 1e3:.2f} mm/s  slowest once at 15 mm/s {mid.min() * 1e3:.2f} mm/s "
              f"at s={s_k * 1e3:.1f} mm  off line {off_line(x, ARC['tip_base']) * 1e3:.4f} mm  "
              f"pen budget {r.tip_deviation * 1e3:.4f} mm  fastest pen "
              f"{tip_speed(r).max() / RULES.draw_speed * 100 - 100:+.2f} %")
    r, t, v, x, mid, s_k = rows["smooth"]
    assert r.traj.t[-1] <= 5.6
    assert slowest(v) >= 0.0049
    s_t = np.interp(t, r.traj.t, r.s)
    above = np.flatnonzero(v >= 0.015)
    window = np.zeros(len(v), dtype=bool)
    window[above[0]:above[-1] + 1] = True
    binds = np.abs(s_t - 0.0483) < 0.001
    assert v[window & ~binds].min() >= 0.015
    assert v[window & binds].min() >= 0.0145
    # There, joint 5 is at over half its acceleration limit (the speed choice plans the bend
    # with 0.7 of the 0.9 target, 0.63 of the limit): it is the acceleration that binds.
    _, _, qdd = sample(r.traj, t[binds])
    frac = np.abs(qdd).max(axis=0) / LIMITS.qdd_max
    print(f"at s=48.3 mm: acceleration / limit per joint {np.round(frac, 3)}")
    assert int(np.argmax(frac)) == 4 and frac[4] >= 0.5
    assert off_line(x, ARC["tip_base"]) <= 1e-4 and r.tip_deviation <= 1e-4
    for hz in (1000.0, 4000.0):
        assert check(r.traj, LIMITS, hz).inside
    assert tip_speed(r).max() <= 1.005 * RULES.draw_speed


def test_corner_model_is_unchanged():
    r = _time(ARC["q"], ALONG, False)
    m = hashlib.sha1()
    for a in (r.traj.t, r.traj.q, r.traj.qd, r.s):
        m.update(a.tobytes())
    assert m.hexdigest() == CORNER_DIGEST


def test_same_measurement_at_every_rate():
    r = _time(ARC["q"], ALONG, True)
    rows = {hz: check(r.traj, LIMITS, hz) for hz in (100.0, 1000.0, 4000.0)}
    peak = {hz: np.array([x.qd_peak.max(), x.qdd_peak.max(), x.qddd_peak.max()])
            for hz, x in rows.items()}
    print("\narc, smooth: peaks (vel, acc, jerk) " +
          "  ".join(f"{int(hz)} Hz {np.round(p, 3)}" for hz, p in peak.items()))
    assert np.all(np.abs(peak[1000.0] - peak[4000.0]) <= 0.02 * peak[4000.0])
    assert np.all(peak[100.0] <= peak[4000.0] * 1.001)
    exact = np.array([r.report.qd_peak.max(), r.report.qdd_peak.max(), r.report.qddd_peak.max()])
    assert np.all(peak[4000.0] <= exact * (1 + 1e-6))


def test_bit_identical_and_both_sweeps_agree():
    import aris.kernel.speed as sp
    a, b = _time(ARC["q"], ALONG, True), _time(ARC["q"], ALONG, True)
    assert a.traj.t.tobytes() == b.traj.t.tobytes() and a.traj.q.tobytes() == b.traj.q.tobytes()
    if sp._native_sweeps is None:
        pytest.skip("aris_retime_native is not installed")
    saved, sp._native_sweeps = sp._native_sweeps, None
    try:
        c = _time(ARC["q"], ALONG, True)
    finally:
        sp._native_sweeps = saved
    assert np.max(np.abs(a.traj.t - c.traj.t)) <= 1e-12
    assert np.max(np.abs(a.traj.q - c.traj.q)) <= 1e-12


# --------------------------------------------------------------------------- lines with corners


def _turns(angles, leg=0.015, step=0.002):
    p, heading = [np.zeros(2)], 0.0
    for a in list(angles) + [None]:
        for _ in range(int(round(leg / step))):
            p.append(p[-1] + step * np.array([np.cos(heading), np.sin(heading)]))
        if a is not None:
            heading += np.deg2rad(a)
    return np.array(p)


@pytest.mark.parametrize("angles", [(30, -90, 150), (10.7, -10.7, 10.7, 45)])
def test_corners_of_the_line(angles):
    """A drawn line with real corners, laid on the paper under arm 13 and solved exactly: the
    spline is cut at the corners, which are rounded within the pen budget; the pen slows there
    and does not stop."""
    p2 = _turns(angles)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p2, axis=0), axis=1))])
    q = _solve(p2)
    tips = ARM.tip(q)
    r = _time(q, s, True)
    t = np.arange(0.0, r.traj.t[-1], 1e-3)
    v = tip_speed(r)
    s_t = np.interp(t[1:], r.traj.t, r.s)
    print(f"\npen budget {r.tip_deviation * 1e3:.3f} mm, off line "
          f"{off_line(ARM.tip(sample(r.traj, t)[0]), tips) * 1e3:.3f} mm, T={r.traj.t[-1]:.2f} s")
    for k, a in enumerate(angles, start=1):
        near = np.abs(s_t - 0.015 * k) < 0.004
        print(f"corner of {abs(a):5.1f} deg: slowest pen {v[near].min() * 1e3:6.2f} mm/s")
        assert v[near].min() > 1e-4
    assert r.tip_deviation <= 1e-4
    assert v.max() <= 1.005 * RULES.draw_speed
    assert check(r.traj, LIMITS).inside and check(r.traj, LIMITS, 4000.0).inside


def _solve(p2):
    """Joint path for pen points p2 (in the paper plane, from the arc's last pen point), nearest
    IK answer each step, hand orientation and joint 7 of the arc's last sample."""
    T0 = ARM.fk(ARC["q"][-1:])[0]
    y = np.cross(_normal(), ARC["tip_base"][-1] - ARC["tip_base"][0])
    y /= np.linalg.norm(y)
    x = np.cross(y, _normal())
    q = [ARC["q"][-1]]
    for p in p2:
        T = T0.copy()
        T[:3, 3] += p[0] * x + p[1] * y
        Q, ok = ARM.ik(T[None], q[0][6:7])
        Q = Q[0][ok[0]]
        q.append(Q[np.argmin(np.linalg.norm(Q - q[-1], axis=1))])
    return np.array(q[1:])


# --------------------------------------------------------------------------- 3. a lift


def _normal():
    """The paper normal in arm 13's base frame, toward the arm, from the arc's pen points."""
    P = ARC["tip_base"]
    n = np.cross(P[40] - P[0], P[-1] - P[0])
    n /= np.linalg.norm(n)
    return -n if n @ P[0] > 0 else n


def lift_path(q_draw, rise=0.002, n=10):
    """The pen raised along the paper normal in `rise` steps, same hand orientation and joint 7,
    nearest IK answer each step (the lift of the sequencer, in short)."""
    normal = _normal()
    T0 = ARM.fk(q_draw[None])[0]
    q = [q_draw]
    for k in range(1, n + 1):
        T = T0.copy()
        T[:3, 3] += rise * k * normal
        Q, ok = ARM.ik(T[None], q_draw[6:7])
        Q = Q[0][ok[0]]
        q.append(Q[np.argmin(np.linalg.norm(Q - q[-1], axis=1))])
    return np.array(q)


@pytest.mark.parametrize("at", [-1, 45, 53])
def test_lift(at):
    """Acceptance 3: 11 samples 2 mm apart along the paper normal, timed without s."""
    q = lift_path(ARC["q"][at])
    out = {sm: _time(q, None, sm, tip_of=None) for sm in (False, True)}
    for sm, r in out.items():
        print(f"\nlift from sample {at}, smooth={sm}: T={r.traj.t[-1]:.3f} s, peak joint speed "
              f"{np.max(r.report.qd_peak / (RULES.speed_fraction * LIMITS.qd_max)):.3f} of the "
              f"target, acceleration {r.report.qdd_ratio.max():.3f} of the limit, deviation "
              f"{r.deviation * 1e3:.4f} mrad", end="")
        assert check(r.traj, LIMITS).inside and check(r.traj, LIMITS, 4000.0).inside
    assert out[True].traj.t[-1] <= out[False].traj.t[-1] * 1.001
    assert np.max(np.abs(out[True].traj.q[-1] - q[-1])) <= 1e-12


# --------------------------------------------------------------------------- the contract


def test_refusals_are_the_same():
    q = ARC["q"]
    bad = q.copy()
    bad[5, 3] = LIMITS.q_max[3] + 0.01
    cases = [((JointPath(q[:1]), LIMITS, RULES), {}, "too_few_samples"),
             ((JointPath(np.repeat(q[:1], 4, axis=0)), LIMITS, RULES), {}, "no_motion"),
             ((JointPath(bad), LIMITS, RULES), {}, "outside_limits"),
             ((JointPath(q[:3]), LIMITS, RULES, np.array([0.0, 0.02, 0.01])), {}, "bad_arc_length"),
             ((JointPath(q), LIMITS, RULES), {"tip_budget_m": 1e-4}, "bad_rules"),
             ((JointPath(q), LIMITS, RULES, ALONG), {"tip_of": ARM.tip, "tip_budget_m": 1e-9},
              "cannot_smooth")]
    for args, kw, reason in cases:
        for sm in (False, True):
            r = retime(*args, smooth=sm, **kw)
            assert isinstance(r, Refusal) and r.reason == reason, (sm, r)


def test_ends_at_rest_and_exact():
    r = _time(ARC["q"], ALONG, True)
    assert np.all(r.traj.qd[0] == 0.0) and np.all(r.traj.qd[-1] == 0.0)
    assert np.max(np.abs(r.traj.q[[0, -1]] - ARC["q"][[0, -1]])) <= 1e-12
    assert r.s[0] == 0.0 and abs(r.s[-1] - ALONG[-1]) < 1e-12 and np.all(np.diff(r.s) > 0)
    assert r.deviation <= 1.5e-4 and r.width == pytest.approx(np.diff(ALONG).min())


# --------------------------------------------------------------------------- 4. CPU


def test_cpu():
    """CPU time of one process, the least disturbed of seven runs; the ceiling is ten times
    what was measured."""
    from test_kernel_retime import LIMITS as SYN_LIMITS, RULES as SYN_RULES, _tip, draw_case
    wq, ws = draw_case(np.c_[np.linspace(0, 0.5, 2000), 0.02 * np.sin(np.linspace(0, 30, 2000))])
    cases = {"arc (109 samples)": (ARC["q"], ALONG, ARM.tip, LIMITS, RULES, 0.1),
             "wavy (2000 samples)": (wq, ws, _tip, SYN_LIMITS, SYN_RULES, 0.8)}
    print()
    for name, (q, s, tip, lim, rules, ceiling) in cases.items():
        for sm in (False, True):
            best = np.inf
            for _ in range(7):
                t0 = time.process_time()
                out = retime(JointPath(q), lim, rules, s=s, tip_of=tip, smooth=sm)
                best = min(best, time.process_time() - t0)
            print(f"{name:20s} smooth={sm!s:5s} {best * 1e3:6.1f} ms CPU  ({out.t[-1]:.2f} s, "
                  f"{len(out.t)} samples)")
            assert best < ceiling


# --------------------------------------------------------------------------- 2. the arm cases


def _plans(rig, arm_id):
    import arm_cases as ac
    import local_cases as lc
    from aris.local.planner import plan as local_plan
    arm, obs, rules, gates = lc.problem(rig, arm_id)
    cases = ac.case_lines(rig, arm_id)
    out = {}
    for name in ("word", "lines"):
        lines = [rig.to_base(arm_id, x) for x in cases[name]]
        bunches, _ = local_plan(arm, lines, obs, rules, gates, workers=16)
        out[name] = [p for b in bunches for p in b.plans]
    return arm, rules, out


@pytest.mark.slow                  # 5 to 15 minutes: the local planner, then 1 700 timings
def test_arm_cases():
    """Acceptance 2: every plan the local planner returns for the word and the random lines,
    arms 13 and 31, timed both ways.  Slowest mid-line speed as the checker measures it."""
    rig = Rig.load(ROOT / "config")
    print()
    for arm_id in ("1L", "2L"):
        arm, rules, sets = _plans(rig, arm_id)
        for name, plans in sets.items():
            got = {}
            for sm in (False, True):
                rows = []
                for p in plans:
                    along = np.abs(p.s - p.s[0])
                    r = retime_detailed(JointPath(p.q), arm.limits, rules, s=along,
                                        tip_of=arm.tip, smooth=sm)
                    assert isinstance(r, RetimeResult), (p.piece.line_id, sm, r)
                    _, v, x = pen_along(r, along, p.tip_base, arm.tip)
                    corner = np.degrees(_pen_turn(p.tip_base).max())
                    rows.append((r.traj.t[-1], slowest(v), tip_speed(r, arm.tip).max(),
                                 off_line(x, p.tip_base), corner, check(r.traj, arm.limits).inside))
                a = np.array(rows)
                got[sm] = a
                print(f"arm {arm_id} {name:5s} smooth={sm!s:5s} {len(a)} plans, drawing "
                      f"{a[:, 0].sum():7.1f} s; slowest mid-line p1/p5/p50/min "
                      f"{np.percentile(a[:, 1], 1) * 1e3:.2f}/{np.percentile(a[:, 1], 5) * 1e3:.2f}/"
                      f"{np.percentile(a[:, 1], 50) * 1e3:.2f}/{a[:, 1].min() * 1e3:.2f} mm/s; "
                      f"fastest pen {a[:, 2].max() / rules.draw_speed * 100 - 100:+.2f} %; "
                      f"off line {a[:, 3].max() * 1e3:.4f} mm")
            a = got[True]
            assert a[:, 5].all()
            assert a[:, 3].max() <= 1e-4
            assert a[:, 2].max() <= 1.005 * rules.draw_speed
            assert a[:, 0].sum() <= got[False][:, 0].sum() * 1.001
            # Below 2 mm/s only at a sharp corner of the drawn line (the word's "w": three
            # 148 degree corners, which the corner model takes at 1.0-1.1 mm/s).
            sharp = a[:, 4] > 120.0
            assert a[~sharp, 1].min() >= 0.002
            assert np.all(a[sharp, 1] >= 0.001)
