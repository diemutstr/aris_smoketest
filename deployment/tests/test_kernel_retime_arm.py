"""The timing step on drawing paths made by the real arm model.  Run with -s for the numbers.

Pen-tip lines on the paper 0.97 m below the base: straight lines, arcs and zigzags at random
places and directions, 1 mm apart, turned into joint paths by `Arm.hand_pose` + `Arm.ik`,
following one IK branch (the same slot) from start to end with a fixed spin and q7.
"""
from __future__ import annotations

import numpy as np
import pytest

from aris.kernel.retime import check, retime_detailed, sample
from aris.types import DrawRules, JointPath, Refusal

arm_module = pytest.importorskip("aris.kernel.arm")
ARM = arm_module.Arm(arm_module.default_tool())
RULES = DrawRules()
NORMAL = np.array([0.0, 0.0, -1.0])
PAPER_Z = 0.97


def _densify(corners, step=1e-3):
    pts = [corners[0]]
    for a, b in zip(corners[:-1], corners[1:]):
        n = int(np.ceil(np.linalg.norm(b - a) / step))
        pts += [a + (b - a) * k / n for k in range(1, n + 1)]
    return np.array(pts)


def shapes(seed, n=12):
    """(name, pen points (M, 2), corner arc lengths) at random places on the paper."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        r, phi, turn = rng.uniform(0.3, 0.75), rng.uniform(-np.pi, np.pi), rng.uniform(-np.pi, np.pi)
        c = r * np.array([np.cos(phi), np.sin(phi)])
        d = np.array([np.cos(turn), np.sin(turn)])
        e = np.array([-d[1], d[0]])
        kind = ("line", "arc", "zigzag")[i % 3]
        if kind == "line":
            p = _densify(np.array([c - 0.08 * d, c + 0.08 * d]))
        elif kind == "arc":
            th = np.linspace(0.0, np.pi, 315)
            p = c + 0.1 * (np.cos(th)[:, None] * d + np.sin(th)[:, None] * e)
        else:
            k = np.arange(7)
            p = _densify(c + np.outer((k - 3) * 0.02, d) + np.outer(0.02 * (k % 2), e))
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        corners = s[np.round(np.linspace(0, len(s) - 1, 7)).astype(int)][1:-1] if kind == "zigzag" \
            else np.array([])
        out.append((f"{kind}@({c[0]:+.2f},{c[1]:+.2f})", p, s, corners))
    return out


def joint_path(p2):
    """One IK branch followed along the whole line, inside the gates; None if there is none."""
    tips = np.c_[p2, np.full(len(p2), PAPER_Z)]
    for spin in np.linspace(-np.pi, np.pi, 8, endpoint=False):
        T = ARM.hand_pose(tips, NORMAL, np.full(len(p2), spin), np.zeros((len(p2), 2)))
        for q7 in (0.0, 0.8, -0.8):
            Q, ok = ARM.ik(T, np.full(len(p2), q7))
            for b in range(Q.shape[1]):
                if not ok[:, b].all():
                    continue
                q = Q[:, b]
                if (np.abs(np.diff(q, axis=0)).max() < 0.05 and ARM.limit_margin(q).min() > 0.15
                        and ARM.sigma_min(q).min() > 0.08):
                    return q
    return None


def _run(name, p2, s, corners, **kw):
    q = joint_path(p2)
    if q is None:
        return None
    r = retime_detailed(JointPath(q), ARM.limits, RULES, s=s, tip_of=ARM.tip, **kw)
    assert not isinstance(r, Refusal), (name, r)
    t = np.arange(0.0, r.traj.t[-1], 1e-3)
    tip = ARM.tip(sample(r.traj, t)[0])
    speed = np.linalg.norm(np.diff(tip, axis=0), axis=1) / 1e-3
    s_t = np.interp(t[1:], r.traj.t, r.s)
    free = (s_t > 0.01) & (s_t < s[-1] - 0.01)
    for c in corners:
        free &= np.abs(s_t - c) > 0.005
    # joint speed the line needs at draw speed, against the allowed speed
    need = np.max(np.abs(np.diff(q, axis=0)) / np.diff(s)[:, None] * RULES.draw_speed
                  / (RULES.speed_fraction * ARM.limits.qd_max))
    binds = np.mean(speed[free] < 0.98 * RULES.draw_speed)
    return dict(name=name, r=r, err=np.max(np.abs(speed[free] / RULES.draw_speed - 1.0)),
                over=float(speed.max() / RULES.draw_speed - 1.0),
                tip_mm=r.tip_deviation * 1e3, need=need, binds=binds,
                slowest=speed[5:-5].min(), rep=check(r.traj, ARM.limits))


@pytest.mark.slow                  # about 4 s: 36 shapes through IK and retime
def test_drawing_through_the_real_arm():
    rows = [x for x in (_run(*sh) for sh in shapes(3, 36)) if x is not None]
    assert len(rows) >= 15
    print()
    for x in rows:
        print(f"{x['name']:22s} T={x['r'].traj.t[-1]:6.2f}s  speed error {x['err'] * 100:6.3f} %  "
              f"pen deviation {x['tip_mm']:.4f} mm  joint speed needed {x['need']:.3f} of allowed  "
              f"limit binds {x['binds'] * 100:.1f} % of the time")
    n_bind = sum(x["need"] > 1.0 for x in rows)
    print(f"{len(rows)} shapes drawn; a joint limit binds at 20 mm/s on {n_bind} of them; "
          f"worst pen deviation {max(x['tip_mm'] for x in rows):.4f} mm; worst speed error "
          f"{max(x['err'] for x in rows if x['need'] <= 1.0) * 100:.3f} % where nothing binds")
    print(f"fastest pen anywhere: {max(x['over'] for x in rows) * 100:+.2f} % of the draw speed")
    for x in rows:
        assert x["over"] <= 0.01
        assert x["tip_mm"] <= 0.1
        assert x["slowest"] > 0.0
        assert x["rep"].inside
        if x["need"] <= 0.95:
            assert x["err"] <= 0.02


def test_tip_budget_is_met_when_tight():
    name, p2, s, corners = next(sh for sh in shapes(3) if sh[0].startswith("zigzag"))
    loose = _run(name, p2, s, corners)
    tight = _run(name, p2, s, corners, tip_budget_m=0.005e-3)
    print(f"\n{name}: pen deviation {loose['tip_mm'] * 1e3:.1f} um at the 0.1 mm default, "
          f"{tight['tip_mm'] * 1e3:.2f} um with a 5 um budget; time {loose['r'].traj.t[-1]:.2f} s "
          f"-> {tight['r'].traj.t[-1]:.2f} s")
    assert tight["tip_mm"] <= 0.005
    assert tight["rep"].inside


def test_tip_budget_needs_tip_of():
    q = joint_path(shapes(3)[0][1])
    r = retime_detailed(JointPath(q), ARM.limits, RULES, tip_budget_m=1e-4)
    assert isinstance(r, Refusal) and r.reason == "bad_rules"


def letters(seed, n=6):
    """Letter-like lines: 8 strokes of up to 7 cm in random directions, sampled every 2 mm."""
    rng = np.random.default_rng(seed)
    out = []
    for k in range(n):
        start = rng.uniform(0.3, 0.6) * np.array([np.cos(a := rng.uniform(-np.pi, np.pi)),
                                                  np.sin(a)])
        corners = start + np.cumsum(rng.uniform(-0.05, 0.05, (8, 2)), axis=0)
        p = _densify(corners, 2e-3)
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        out.append((f"letter {seed}.{k}", p, s))
    return out


def test_pen_never_faster_than_the_draw_speed():
    """Through the real arm's tip, sampled at 1 kHz: at most 1 % over the draw speed anywhere,
    also where the planner's joint path moves the pen faster than its arc length says."""
    worst, drawn = 0.0, 0
    for name, p2, s in letters(1) + letters(2):
        q = joint_path(p2)
        if q is None:
            continue
        drawn += 1
        r = retime_detailed(JointPath(q), ARM.limits, RULES, s=s, tip_of=ARM.tip)
        assert not isinstance(r, Refusal), (name, r)
        t = np.arange(0.0, r.traj.t[-1], 1e-3)
        tip = ARM.tip(sample(r.traj, t)[0])
        over = np.linalg.norm(np.diff(tip, axis=0), axis=1).max() / 1e-3 / RULES.draw_speed - 1
        worst = max(worst, over)
    print(f"\n{drawn} letter shapes: fastest pen {worst * 100:+.2f} % of the draw speed")
    assert drawn >= 4
    assert worst <= 0.01


@pytest.mark.parametrize("k", [0, 1])
def test_pen_speed_along_the_rim(k):
    """Two drawings of arm 31 along the rim of its reach from the end-to-end run, where the pen
    read 3.7 % over the draw speed: slow joints brake by a small absolute amount, and the output
    samples were too sparse for the cubic to follow the braking.  The saved samples and planned
    pen positions are timed again as a joint path with its arc length."""
    from pathlib import Path
    from aris.rig import Rig
    d = np.load(Path(__file__).parent / "data" / f"arm_speed_31_lines_{k}.npz")
    arm = Rig.load(Path(__file__).parents[1] / "config").arm(31)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(d["tip_base"], axis=0), axis=1))])
    r = retime_detailed(JointPath(d["q"]), arm.limits, RULES, s=s, tip_of=arm.tip)
    assert not isinstance(r, Refusal), r
    t = np.arange(0.0, r.traj.t[-1], 1e-3)
    speed = np.linalg.norm(np.diff(arm.tip(sample(r.traj, t)[0]), axis=0), axis=1) / 1e-3
    print(f"\nrim line {k}: fastest pen {speed.max() / RULES.draw_speed * 100 - 100:+.2f} % of the "
          f"draw speed (the end-to-end run read +{float(str(d['note']).split()[2]) / 0.02 * 100 - 100:.1f} %)")
    assert speed.max() <= 1.01 * RULES.draw_speed
