"""The drawn-offsets calibration of one row (decided 2026-10-08: hand-guiding is impossible on
the site's FR3s outside Desk's programming mode).

Both arms of a row draw a small mark at each of their shared nominal spots, the L arm a cross,
the R arm a circle, each aiming with the pose the rig has now.  A person measures with a ruler,
per spot, the offset d_S = (R's mark - L's mark) in table axes (x across, y along).

Each arm lands where its true pose puts the point it aimed at: aiming at S with the believed
pose T^ commands the base-frame point T^-1 S, which lands at Rz(psi) (S - t^) + t^ + delta,
where (delta, psi) is how far the true pose is from the believed one.  The offsets see how the
two arms differ; moving both together changes no mark against the other, and turning both
together only turns the small offsets (a second-order effect).  So the pair's mean position and
mean yaw are put on the nominal mountings (the mark job's frame convention) and the solve finds
how the two differ.  Two spots give 4 equations for 3 unknowns:
the leftover is the residual (the two measured offsets disagree about the spots' distance).
The frame is built into the unknowns (`_pair`), the convention `planar.frame_motion` applies.

The result is a `MarkSolution` (method "offsets", no pivots, no marks solved: the spots are the
nominal ones) and is written with `files.write_mark_solution`; the server's one call is
`aris.server.offsets.calibrate_from_offsets`.  See docs/modules/calib.md.
"""
from __future__ import annotations

import numpy as np

from aris.calib import planar
from aris.calib.marks import POSE_SHIFT_MAX, POSE_YAW_MAX, MarkSolution, SlotFit
from aris.types import Slot

OFFSET_MAX_M = 0.060      # a measured offset larger than any mounting error: marks swapped
                          # (whose is whose), or a direction read the wrong way
RESIDUAL_MAX_M = 0.002    # the offsets do not fit one rigid correction: a misread ruler, or a
                          # mark measured from the wrong spot


def _refuse(why) -> MarkSolution:
    nan = float("nan")
    return MarkSolution(False, why, {}, {}, nan, nan, (), (), "")


def _pair(rig, S, d, slots):
    """-> ({slot: (x, y, yaw against the rig's rotation)}, per-spot residual norms).

    Unknowns: each arm's turn psi and shift delta against the pose the rig has.  The frame
    fixes psi_L + psi_R (mean yaw on nominal) and delta_L + delta_R (mean axis on nominal), so
    psi = c -/+ u and delta = (sum -/+ D) / 2 leave u and D: 3 unknowns, 2 per spot.  For a
    given u, D is linear (the mean over spots); u is found by a 1-D Gauss-Newton started from
    the rigid fit (exact to first order; the common turn only rotates the small offsets)."""
    L, R = slots
    TL, TR = rig.T_table_base(L), rig.T_table_base(R)
    tL, tR = TL[:2, 3], TR[:2, 3]
    eL, eR = S - tL, S - tR
    y0 = [planar.yaw_between(T[:3, :3], rig.nominal_pose(s)[:3, :3])
          for s, T in ((L, TL), (R, TR))]
    c = -(y0[0] + y0[1]) / 2.0
    total = 2.0 * np.mean([rig.nominal_pose(s)[:2, 3] for s in slots], axis=0) - tL - tR

    def spread(u):            # d minus what the turns explain, per spot; D is its mean
        rot = lambda a, e: e @ planar._rot2(a).T
        return d - (tR - tL) - rot(c + u, eR) + rot(c - u, eL)

    u = planar.kabsch2(eR, eR + d)[0] / 2.0
    for _ in range(20):
        f = lambda v: (spread(v) - spread(v).mean(axis=0)).reshape(-1)
        r0, h = f(u), 1e-7
        J = (f(u + h) - r0) / h
        step = -float(J @ r0) / float(J @ J)
        u += step
        if abs(step) < 1e-13:
            break
    D = spread(u).mean(axis=0)
    res = np.linalg.norm(spread(u) - D, axis=1)
    return {L: np.r_[tL + (total - D) / 2.0, c - u],
            R: np.r_[tR + (total + D) / 2.0, c + u]}, res


def solve_offsets(rig, spots: dict, measured: dict, slots: tuple[Slot, Slot]) -> MarkSolution:
    """The row's two poses from the measured offsets.

    spots: {name: (x, y)} the nominal table points both arms aimed at.  measured: {name:
    (dx, dy)} m, R's mark minus L's mark, table axes.  slots: (L, R).  Never raises on bad
    data: a refusal says what is wrong."""
    if len(slots) != 2 or slots[0] == slots[1]:
        return _refuse(f"two different slots are needed, got {slots}")
    for s in slots:
        if s not in rig.arm_ids:
            return _refuse(f"no arm in slot {s!r} (mounted: {rig.arm_ids})")
    names = [n for n in spots if n in measured]
    missing = sorted(set(spots) ^ set(measured))
    if missing:
        return _refuse(f"spots and measurements do not match: {missing}")
    if len(names) < 2:
        return _refuse(f"{len(names)} measured spot(s); two are needed (they fix the turn)")
    S = np.array([np.asarray(spots[n], float).reshape(2) for n in names])
    d = np.array([np.asarray(measured[n], float).reshape(2) for n in names])
    if not np.all(np.isfinite(d)):
        return _refuse("a measured offset is not a number")
    big = [n for n, v in zip(names, d) if np.linalg.norm(v) > OFFSET_MAX_M]
    if big:
        return _refuse(f"the measured offset at {', '.join(big)} is larger than any mounting "
                       f"error ({max(np.linalg.norm(d, axis=1)) * 1e3:.0f} mm, limit "
                       f"{OFFSET_MAX_M * 1e3:g}); check which mark is whose and the directions")
    if np.linalg.norm(S[0] - S[-1]) < 0.05:
        return _refuse("the spots are too close together to fix the turn")

    L, R = slots
    pose, res = _pair(rig, S, d, slots)
    fits, why = {}, []
    for s, (x, y, yaw) in pose.items():
        T0, Tn = rig.T_table_base(s), rig.nominal_pose(s)
        T = T0.copy()
        T[:3, :3] = planar.rz(yaw) @ T0[:3, :3]
        T[:2, 3] = (x, y)
        shift = float(np.linalg.norm(T[:2, 3] - Tn[:2, 3]))
        ynom = planar.yaw_between(T[:3, :3], Tn[:3, :3])
        fits[s] = SlotFit(s, T, T0, rig.arm(s).tool.tip_hand.copy(), None, "", shift, ynom,
                          float(np.sqrt(np.mean(res ** 2))), len(names), rig.pen_name, None,
                          "offsets")
        if shift > POSE_SHIFT_MAX or abs(ynom) > POSE_YAW_MAX:
            why.append(f"refused: {s} {shift:.3f} m and {np.rad2deg(ynom):+.2f} deg from its "
                       f"nominal pose — wrong slot or wrong robot?")
    k = int(np.argmax(res))
    if res[k] > RESIDUAL_MAX_M:
        why.append(f"the offsets do not fit one correction: {res[k] * 1e3:.2f} mm left at "
                   f"{names[k]} (limit {RESIDUAL_MAX_M * 1e3:g}); measure again")
    rms = float(np.sqrt(np.mean(res ** 2)))
    return MarkSolution(not why, "; ".join(why), fits, {}, rms, float(res.max()), (), (),
                        f"offsets of {L}/{R} at {names}; the pair's mean position and mean yaw "
                        f"on the nominal mountings")
