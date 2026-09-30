"""Where does an arm's reach at the paper end, and which gate ends it?  A measurement for Pete.

    cd /home/franka/aris_project/aris3 && .venv/bin/python tests/reach_cases.py

Set-up: the paper plane 0.97 m below the base, square to the base axis (normal -z in the base
frame, the arm hanging above it).  Tip positions along one ray from the axis (+x; the arm turns
freely about its axis, and joint 1's limit is far from binding along +x) at 5 mm steps.  At each
radius: 16 hand spins, leans in the cone (0 to 35 deg in 5 deg rings, 8 directions each), the
analytic IK over 64 values of q7, all 8 slots.  A radius is "drawable" under a gate setting if at
least one configuration passes the joint-limit margin and the sigma_min gate with a lean inside
the allowed cone.  Obstacles and self-collision are ignored for the rim; the self column says
whether the best configuration at the rim also clears the arm's self-collision gate (23 mm).
"""
from __future__ import annotations

import numpy as np

from aris.kernel.arm import Arm, default_tool
from aris.kernel.collide import self_clearance
from aris.types import Gates

H = 0.97
RADII = np.round(np.arange(0.600, 1.0001, 0.005), 3)
SPINS = np.linspace(-np.pi, np.pi, 16, endpoint=False)
LEAN_DEG = np.arange(0, 36, 5)
SETTINGS = [  # (label, margin, sigma, lean_deg)
    ("1 today", 0.15, 0.08, 15),
    ("2 margin 0.10", 0.10, 0.08, 15),
    ("3 margin 0.05", 0.05, 0.08, 15),
    ("4 margin 0 (hard limits)", 0.0, 0.08, 15),
    ("5 sigma 0.04", 0.15, 0.04, 15),
    ("6 sigma 0", 0.15, 0.0, 15),
    ("7 lean 25", 0.15, 0.08, 25),
    ("8 lean 35", 0.15, 0.08, 35),
    ("9 loosest", 0.0, 0.0, 35),
]


def leans():
    out = [(0.0, 0.0, 0.0)]
    for d in LEAN_DEG[1:]:
        a = np.deg2rad(d)
        out += [(a * np.cos(t), a * np.sin(t), d) for t in np.linspace(0, 2 * np.pi, 8, False)]
    return np.array(out)


def sigma_min(arm, Q):
    """Smallest singular value of the tip Jacobian, via the 3x3 J J^T (fast for many)."""
    out = np.empty(len(Q))
    for s in range(0, len(Q), 200_000):
        J = arm.tip_jacobian(Q[s:s + 200_000])
        ev = np.linalg.eigvalsh(J @ J.transpose(0, 2, 1))[:, 0]
        out[s:s + 200_000] = np.sqrt(np.maximum(ev, 0.0))
    return out


def configs_at(arm, r, L):
    """All valid IK answers at radius r -> (Q (n,7), lean_deg (n,), margin, sigma, worst joint)."""
    q7s = np.linspace(arm.limits.q_min[6], arm.limits.q_max[6], 64)
    n = len(SPINS) * len(L)
    spin = np.repeat(SPINS, len(L))
    lean = np.tile(L[:, :2], (len(SPINS), 1))
    T = arm.hand_pose(np.tile([r, 0.0, H], (n, 1)), np.array([0.0, 0.0, -1.0]), spin, lean)
    T = np.repeat(T, len(q7s), 0)
    Q, ok = arm.ik(T, np.tile(q7s, n))
    m, _ = np.nonzero(ok)
    q = Q[ok]
    lean_deg = np.tile(L[:, 2], len(SPINS))[m // len(q7s)]
    lo, hi = q - arm.limits.q_min, arm.limits.q_max - q
    per_joint = np.minimum(lo, hi)
    return q, lean_deg, per_joint.min(1), sigma_min(arm, q), per_joint.argmin(1)


def main():
    arm = Arm(default_tool())
    L = leans()
    data = {}
    for r in RADII:
        data[r] = configs_at(arm, r, L)
    for ld in (15, 35):
        far = max((r for r in RADII if (data[r][1] <= ld + 1e-9).any()), default=None)
        print(f"geometric reach, any IK answer inside the limits, lean <= {ld} deg: {far} m")
    gates = Gates()
    print(f"{'setting':28s} {'rim (m)':>8s}  self-collision at the rim")
    rows = []
    for label, mg, sg, ld in SETTINGS:
        rim, best = None, None
        for r in RADII:
            q, lean, margin, sig, _ = data[r]
            ok = (margin >= mg) & (sig >= sg) & (lean <= ld + 1e-9)
            if ok.any():
                rim = r
                score = np.minimum((margin - mg) / 0.15, (sig - sg) / 0.08)[ok]
                qs = q[ok]
                best = (qs[np.argmax(score)], qs)
        if rim is None:
            print(f"{label:28s}  nothing drawable")
            continue
        body = arm.body(best[0][None])
        sc = self_clearance(body, arm.self_pairs, gates.self_margin)[0]
        allc = self_clearance(arm.body(best[1]), arm.self_pairs, gates.self_margin)
        txt = (f"{'passes' if sc >= 0 else 'FAILS'} ({1e3 * sc:+.0f} mm beyond 23 mm); "
               f"{int((allc >= 0).sum())} of {len(allc)} passing configurations clear it")
        rows.append((label, rim, txt))
        print(f"{label:28s} {rim:8.3f}  {txt}")

    # today's gates, 1 cm beyond today's rim: which gate fails first for the best configuration
    rim0 = rows[0][1]
    r = float(np.round(rim0 + 0.010, 3))
    q, lean, margin, sig, jw = data[r]
    inside = lean <= 15 + 1e-9
    q, margin, sig, jw = q[inside], margin[inside], sig[inside], jw[inside]
    score = np.minimum((margin - 0.15) / 0.15, (sig - 0.08) / 0.08)
    k = int(np.argmax(score))
    which = "joint-limit margin" if (margin[k] - 0.15) / 0.15 < (sig[k] - 0.08) / 0.08 \
        else "sigma_min"
    print(f"\n1 cm beyond today's rim, r = {r:.3f} m, lean <= 15 deg: {len(q)} IK answers; the best "
          f"has margin {margin[k]:.3f} rad (joint {jw[k] + 1} at {q[k, jw[k]]:+.3f}) and sigma_min "
          f"{sig[k]:.3f}; it fails first on the {which}")
    both = ((margin >= 0.15).sum(), (sig >= 0.08).sum())
    print(f"answers passing the margin alone: {both[0]}, sigma alone: {both[1]}")


if __name__ == "__main__":
    main()
