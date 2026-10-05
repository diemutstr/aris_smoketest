"""Simulated mark touches: what a person guiding the arms would register, from a true rig.

For tests and for the simulated driver: the kernel's forward kinematics is the truth.  The true
rig (its calibration files hold the true poses and the true pen tip) puts the tip on the true
mark, give or take the guiding error, in reachable hand orientations; the joints get noise.
The touches come back in the form `marks.solve_marks` takes.
"""
from __future__ import annotations

import numpy as np

from aris.types import Slot

SPINS = np.deg2rad(np.arange(0.0, 360.0, 30.0))    # the hand turns a person might use ...
LEANS = np.deg2rad([(0.0, 0.0), (30.0, 0.0), (-30.0, 0.0), (0.0, 30.0), (0.0, -30.0)])
# ... and tilts: turning about the vertical alone leaves the tip's height along that axis and
# the mark's height trading against each other, so a pivot needs tilts too
Q7 = np.linspace(-2.8, 2.8, 29)


def _reachable(arm, T_hand):
    """(M,4,4) -> (M,7) the best-conditioned IK answer per pose (NaN when none), (M,) sigma."""
    best_q, best_s = np.full((len(T_hand), 7), np.nan), np.full(len(T_hand), -1.0)
    for q7 in Q7:
        Q, ok = arm.ik(T_hand, np.full(len(T_hand), q7))
        for b in range(Q.shape[1]):
            m = np.nonzero(ok[:, b])[0]
            if len(m):
                s = arm.sigma_min(Q[m, b])
                up = s > best_s[m]
                best_q[m[up]], best_s[m[up]] = Q[m[up], b], s[up]
    return best_q, best_s


def _spread_pick(axes, k, start):
    """k indices whose axes spread most (greedy farthest point from `start`)."""
    pick = [start]
    while len(pick) < min(k, len(axes)):
        closest = np.max(axes @ axes[pick].T, axis=1)
        pick.append(int(np.argmin(closest)))
    return pick


def simulate_touches(rig_true, rig_nominal, slots, marks, noise=(0.3e-3, 0.3e-3), seed=0,
                     pivot_touches=6, independent=False) -> dict:
    """{slot: {mark: (K,7)}} for every slot in `slots` and every mark of `marks` ({name: true
    xy}) that the slot shares (rig_nominal.marks).  The slot's first reachable mark gets a pivot
    of `pivot_touches` spread orientations, every other mark one touch.  noise = (guiding error,
    m, a random horizontal offset of the tip; joint noise, rad, per joint).  The guiding error
    is one offset per slot and mark (a pen seated off a dimple's centre stays there while the
    hand turns), or with `independent` a fresh one per touch (a cross-hair drawn on the wood,
    the pen lifted and seated again by eye for every touch)."""
    rng = np.random.default_rng(seed)
    guide, joint = noise
    out = {}
    for slot in slots:
        arm = rig_true.arm(slot)
        T = rig_true.T_table_base(slot)
        R, t = T[:3, :3], T[:3, 3]
        normal = R.T @ np.array([0.0, 0.0, 1.0])
        out[slot] = {}
        for name, xy in marks.items():
            if slot not in rig_nominal.marks[name][1]:
                continue
            Q, s = _poses(arm, R, t, normal, xy, rng, guide, rig_true.paper_z, independent)
            ok = np.nonzero(np.isfinite(Q[:, 0]))[0]
            if not len(ok):
                raise ValueError(f"simulation: {slot} cannot reach mark {name}")
            if not out[slot] and len(ok) >= 3:
                axes = arm.fk(Q[ok])[:, :3, :3] @ arm.tool.pen_axis_hand
                chosen = Q[ok[_spread_pick(axes, pivot_touches, int(np.argmax(s[ok])))]]
            else:
                chosen = Q[[ok[int(np.argmax(s[ok]))]]]
            out[slot][name] = chosen + joint * rng.standard_normal(chosen.shape)
    return out


def _poses(arm, R, t, normal, xy, rng, guide, paper_z, independent=False):
    """IK at every orientation (spin x lean) for the tip on the mark plus the guiding error:
    one offset for all orientations, or a fresh one per orientation when `independent`."""
    leans = LEANS
    spin = np.repeat(SPINS, len(leans))
    lean = np.tile(leans, (len(SPINS), 1))
    ang = rng.uniform(0.0, 2 * np.pi, len(spin) if independent else 1)
    off = guide * np.column_stack([np.cos(ang), np.sin(ang), np.zeros(len(ang))])
    p_base = (np.array([xy[0], xy[1], paper_z]) + off - t) @ R
    p_base = np.broadcast_to(p_base, (len(spin), 3))
    return _reachable(arm, arm.hand_pose(p_base, normal, spin, lean))


def true_marks(rig, names, off=(0.02, 0.05), seed=0) -> dict:
    """Mark positions as a person would draw them on the wood by eye: nominal plus an offset of
    `off[0]`..`off[1]` m in a random direction."""
    rng = np.random.default_rng(seed)
    out = {}
    for n in names:
        a, r = rng.uniform(0.0, 2 * np.pi), rng.uniform(*off)
        out[n] = rig.marks[n][0] + r * np.array([np.cos(a), np.sin(a)])
    return out


def seam_error(rig_true, solved_T: dict, solved_tip: dict, point_xy, slots: tuple[Slot, ...]):
    """Where each slot's pen really lands when the recovered calibration aims it at `point_xy`
    on the paper (pen upright, best spin): -> {slot: (2,) landing minus aim, table frame}."""
    from aris.kernel.arm import Arm
    from aris.kernel.tool import with_tip
    out = {}
    for s in slots:
        tool = rig_true.arm(s).tool
        believed = Arm(with_tip(tool, solved_tip[s]))
        T = solved_T[s]
        p_base = (np.array([point_xy[0], point_xy[1], rig_true.paper_z]) - T[:3, 3]) @ T[:3, :3]
        n = T[:3, :3].T @ np.array([0.0, 0.0, 1.0])
        Th = believed.hand_pose(np.repeat(p_base[None], len(SPINS), 0), n, SPINS,
                                np.zeros((len(SPINS), 2)))   # upright, every spin
        Q, sig = _reachable(believed, Th)
        q = Q[int(np.argmax(sig))]
        real = rig_true.to_table(s, rig_true.arm(s).tip(q[None]))[0]
        out[s] = real[:2] - np.asarray(point_xy, float)
    return out
