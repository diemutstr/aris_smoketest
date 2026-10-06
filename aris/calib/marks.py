"""Steps 2 and 3 of the calibration: every arm's x, y and yaw, from hand-guided touches of the
marks (DESIGN.md section 6, "Steps 2 and 3 as built", 2026-10-05).

Each mark is shared by two neighbouring slots.  At its first mark a slot is guided through six
hand orientations, tilted and turned, with the pen seated (a pivot): that gives the pen tip in
the hand and the mark in the base frame.  Every other mark is one touch, read with that tip.
Then one planar solve places every slot (x, y, yaw) and every mark (x, y) so that all touches
agree (`aris/calib/planar.py`).  The marks are drawn by hand, centimetres off their nominal
places, so they do not fix the frame: the solved slot axes are fitted rigidly onto the nominal
mountings (rig.json), or marks solved before hold it (subsets).  z, roll and pitch of each slot
stay what the rig has (the plane job's).  See docs/modules/calib.md.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.calib import planar
from aris.calib.pen import PenCalibration
from aris.types import Slot

PIVOT_MIN_TOUCHES = 3
PIVOT_SPREAD_MIN = np.deg2rad(15.0)  # pen axes closer than this: the tip's length along the pen
                                     # is barely seen, and the fit trades it against the mark
PIVOT_CONDITION_MIN = 0.1           # smallest singular value of the pivot's equations: turning
                                     # the hand about one axis only (spins, no tilt) leaves the
                                     # tip's part along that axis free; the tip's error is about
                                     # the touch noise divided by this number
# The next three are set for sharpie cross-hairs placed by eye, every touch a fresh seating
# (about 0.3-0.5 mm per touch); dimples, which centre the pen, would allow half.
PIVOT_RESIDUAL_MAX = 1.5e-3          # a pivot touch further than this off the common point:
                                     # the pen slipped off the mark during that touch
PAIR_DISAGREE_MAX = 2.5e-3           # two slots disagree on the distance between two marks: a
                                     # mark moved between them, or one touch is off
RMS_MAX = 1.0e-3                     # the touches do not fit one rigid layout (an RMS: below the
                                     # single-touch limit above, as honest touches scatter less)
POSE_SHIFT_MAX = 0.030               # after the frame fit, a slot this far from its nominal
POSE_YAW_MAX = np.deg2rad(3.0)       # mounting, or turned this much: wrong slot or wrong robot


@dataclass(frozen=True)
class Pivot:
    slot: Slot
    passed: bool
    why: str
    tip_hand: np.ndarray | None        # (3,) the pen tip in the hand frame
    point_base: np.ndarray | None      # (3,) the mark in the base frame
    residuals: np.ndarray              # (K,) m, each touch's distance from the common point
    spread: float                      # rad, the largest angle between two touches' pen axes


@dataclass(frozen=True)
class SlotFit:
    slot: Slot
    T_table_base: np.ndarray           # (4,4) solved x, y, yaw; z, roll, pitch as before
    T_before: np.ndarray               # (4,4) the pose the rig had
    tip_hand: np.ndarray               # (3,) from the pivot
    pivot: Pivot
    pivot_mark: str
    shift: float                       # m, solved axis from the nominal one (rig.nominal_pose)
    yaw: float                         # rad, turn about the vertical against the nominal pose
    rms: float                         # m, this slot's touches
    n_touches: int
    pen: str                           # the pen that is in (the pivot measured its tip)
    pivot_q: np.ndarray                # (7,) the pivot's first touch, for the pen part


@dataclass(frozen=True)
class MarkFit:
    name: str
    xy: np.ndarray                     # (2,) table frame
    from_nominal: np.ndarray           # (2,) xy minus rig.json's nominal place (information)
    state: str                         # "solved", or "known" (given, held fixed)
    residual: float                    # m, RMS of the touches at it
    by: tuple                          # the slots that touched it
    note: str                          # "" or "determined by one arm"


@dataclass(frozen=True)
class MarkSolution:
    passed: bool
    why: str
    slots: dict                        # slot -> SlotFit
    marks: dict                        # name -> MarkFit
    rms: float                         # m, over every touch
    max_residual: float
    pairs: tuple                       # (mark, mark, slot, slot, disagreement m) per checked pair
    notes: tuple                       # flags that do not refuse
    frame: str = ""                    # what fixed the frame


# ---------------------------------------------------------------------------- one slot

def pivot(rig, slot: Slot, Q) -> Pivot:
    """The common point of K >= 3 touches with the tip seated on one mark: least squares on
    R_k p + t_k = d for p (tip, hand frame) and d (mark, base frame)."""
    Q = np.zeros((0, 7)) if Q is None or np.size(Q) == 0 else np.asarray(Q, float)
    if Q.ndim != 2 or Q.shape[1] != 7:
        return Pivot(slot, False, f"{slot}: the pivot's touches are not K x 7 joint readings",
                     None, None, np.zeros(0), float("nan"))
    nothing = lambda why, res=np.zeros(0), spread=float("nan"): Pivot(
        slot, False, why, None, None, res, spread)
    if len(Q) < PIVOT_MIN_TOUCHES:
        return nothing(f"{slot}: the pivot has {len(Q)} touches, at least "
                       f"{PIVOT_MIN_TOUCHES} needed")
    if not np.all(np.isfinite(Q)):
        return nothing(f"{slot}: a pivot touch has no joint reading")
    arm = rig.arm(slot)
    T = arm.fk(Q)
    R, t = T[:, :3, :3], T[:, :3, 3]
    axes = R @ arm.tool.pen_axis_hand
    spread = float(np.arccos(np.clip(np.min(axes @ axes.T), -1.0, 1.0)))
    A = np.concatenate([R, -np.broadcast_to(np.eye(3), R.shape)], axis=2).reshape(-1, 6)
    x, _, _, sv = np.linalg.lstsq(A, -t.reshape(-1), rcond=None)
    p, d = x[:3], x[3:]
    res = np.linalg.norm(R @ p + t - d, axis=1)
    if spread < PIVOT_SPREAD_MIN:
        return Pivot(slot, False, f"{slot}: the pivot's pen axes span only "
                     f"{np.rad2deg(spread):.1f} deg (at least "
                     f"{np.rad2deg(PIVOT_SPREAD_MIN):g}): tilt and turn the hand more", p, d,
                     res, spread)
    if sv[-1] < PIVOT_CONDITION_MIN:
        return Pivot(slot, False, f"{slot}: the pivot's orientations do not pin the tip "
                     f"(condition {sv[-1]:.3f}, at least {PIVOT_CONDITION_MIN:g}): tilt the "
                     f"hand, not only turn it", p, d, res, spread)
    k = int(np.argmax(res))
    if res[k] > PIVOT_RESIDUAL_MAX:
        return Pivot(slot, False, f"{slot}: pivot touch {k} is {res[k] * 1e3:.2f} mm off the "
                     f"common point (limit {PIVOT_RESIDUAL_MAX * 1e3:g}): the pen slipped, "
                     f"redo it", p, d, res, spread)
    return Pivot(slot, True, "", p, d, res, spread)


def touch_point(rig, slot: Slot, q, tip) -> np.ndarray:
    """The tip in the base frame at joints q (7,) or (K,7), with `tip` in the hand frame."""
    q = np.asarray(q, float)
    T = rig.arm(slot).fk(q.reshape(-1, 7))
    p = T[:, :3, :3] @ np.asarray(tip, float) + T[:, :3, 3]
    return p[0] if q.ndim == 1 else p


def pen_from_pivot(rig, slot: Slot, pv: Pivot, mark: str, q0) -> PenCalibration:
    """The pivot's tip as the slot's pen part (a measured tip, for the pen that is in)."""
    nom, before = rig.nominal_tip(), rig.arm(slot).tool.tip_hand.copy()
    u = rig.arm(slot).tool.pen_axis_hand
    tip = pv.tip_hand
    nan = float("nan")
    return PenCalibration(
        slot=slot, pen=rig.pen_name, passed=pv.passed, why=pv.why, tip_hand=tip,
        tip_hand_nominal=nom, tip_hand_before=before,
        correction=nan if tip is None else float((tip - nom) @ u),
        change=nan if tip is None else float((tip - before) @ u),
        height_before=nan, touch_xy_table=None, reference_xy_table=None, from_reference=nan,
        q=np.asarray(q0, float).copy(), base_status=rig.calibration_status(slot)["base"],
        method="pivot",
        detail={"mark": mark, "touches": int(len(pv.residuals)),
                "residuals_mm": np.round(pv.residuals * 1e3, 4).tolist(),
                "spread_deg": round(float(np.rad2deg(pv.spread)), 3)})


# ---------------------------------------------------------------------------- all slots

def _refuse(why, pairs=(), notes=()) -> MarkSolution:
    nan = float("nan")
    return MarkSolution(False, why, {}, {}, nan, nan, tuple(pairs), tuple(notes))


def _observations(rig, touches):
    """-> (pivots {slot: (Pivot, mark, q0)}, [(slot, mark, e (2,))], why).  e is the touch seen
    from the base origin in table axes, horizontal part: the mark is at Rz(yaw) e + (x, y)."""
    pivots, obs = {}, []
    for slot, per_mark in touches.items():
        if slot not in rig.arm_ids:
            return None, None, f"no arm in slot {slot!r} (mounted: {rig.arm_ids})"
        if not per_mark:
            return None, None, f"{slot} has no touches"
        for name, Q in per_mark.items():
            Q = np.asarray(Q, float)
            if name not in rig.marks:
                return None, None, f"{slot} touched {name!r}, which is not a mark of this rig"
            if Q.ndim != 2 or Q.shape[1] != 7 or not len(Q) or not np.all(np.isfinite(Q)):
                return None, None, f"{slot} at {name}: the touches are not K x 7 joint readings"
        first = next(iter(per_mark))
        pv = pivot(rig, slot, per_mark[first])
        if not pv.passed:
            return None, None, f"{pv.why} (at {first}, {slot}'s first mark)"
        pivots[slot] = (pv, first, np.asarray(per_mark[first], float)[0])
        R = rig.T_table_base(slot)[:3, :3]
        for name, Q in per_mark.items():
            d = touch_point(rig, slot, np.asarray(Q, float).reshape(-1, 7), pv.tip_hand)
            obs += [(slot, name, (R @ di)[:2]) for di in d]
    return pivots, obs, ""


def _observable(by_mark, by_slot, fixed):
    """-> (notes, why).  Each slot needs two marks that something else pins down."""
    for s, names in by_slot.items():
        k = sum(1 for n in names if n in fixed or len(by_mark[n] - {s}))
        if k < 2:
            return (), f"{s} needs a partner: only {k} shared mark{'s' if k != 1 else ''}"
    notes = tuple(f"{n} determined by one arm ({next(iter(by_mark[n]))})"
                  for n in by_mark if len(by_mark[n]) == 1 and n not in fixed)
    return notes, ""


def _pairs(obs):
    """Two slots that touched the same two marks must measure the same distance between them."""
    mean = {}
    for s, n, e in obs:
        mean.setdefault((s, n), []).append(e)
    mean = {k: np.mean(v, axis=0) for k, v in mean.items()}
    slots = sorted({s for s, _ in mean})
    out = []
    for i, sa in enumerate(slots):
        for sb in slots[i + 1:]:
            common = sorted({n for s, n in mean if s == sa} & {n for s, n in mean if s == sb})
            for j, m1 in enumerate(common):
                for m2 in common[j + 1:]:
                    da = np.linalg.norm(mean[sa, m1] - mean[sa, m2])
                    db = np.linalg.norm(mean[sb, m1] - mean[sb, m2])
                    out.append((m1, m2, sa, sb, float(abs(da - db))))
    bad = [p for p in out if p[4] > PAIR_DISAGREE_MAX]
    why = "; ".join(f"{sa} and {sb} disagree by {d * 1e3:.2f} mm on the distance {m1}-{m2} "
                    f"(limit {PAIR_DISAGREE_MAX * 1e3:g}): a mark moved or a touch is off"
                    for m1, m2, sa, sb, d in bad)
    return tuple(out), why


def solve_marks(rig, touches, known=None, base_tips=None) -> MarkSolution:
    """Every touched slot's x, y, yaw and every touched mark's xy.

    touches: {slot: {mark: (K,7) joints}}, the slot's first mark with K >= 3 (its pivot; six
    planned).  known: {mark: xy} solved before; when two or more of the touched marks are known
    they hold the frame (subsets), else the solved slot axes are fitted rigidly onto their
    nominal mountings.  base_tips: {slot: the tip (hand frame) the slot's base z was measured
    with} (`files.base_tips`); given, z moves by the hand-z part of (pivot tip - that tip),
    exact for the plane job's upright touches; without it z is kept.  Never raises on bad
    data: a refusal names the slot, mark or pair."""
    known, base_tips = known or {}, base_tips or {}
    if not touches or not any(touches.values()):
        return _refuse("no touches")
    pivots, obs, why = _observations(rig, touches)
    if why:
        return _refuse(why)
    by_mark, by_slot = {}, {}
    for s, n, _ in obs:
        by_mark.setdefault(n, set()).add(s)
        by_slot.setdefault(s, [])
        if n not in by_slot[s]:
            by_slot[s].append(n)
    fixed = {n: np.asarray(xy, float).reshape(2) for n, xy in known.items() if n in by_mark}
    notes = []
    if len(fixed) == 1:
        notes.append(f"{next(iter(fixed))} was known but is solved again: one known mark "
                     f"cannot hold the frame")
        fixed = {}
    if len(by_slot) == 1 and not fixed:
        return _refuse(f"{next(iter(by_slot))} alone needs marks solved before (two of its "
                       f"marks known)", notes=notes)
    more, why = _observable(by_mark, by_slot, fixed)
    notes += more
    if why:
        return _refuse(why, notes=notes)
    pairs, why = _pairs(obs)
    if why:
        return _refuse(why, pairs, notes)
    solved = planar.solve(rig, obs, list(by_slot), list(by_mark), fixed)
    if isinstance(solved, str):
        return _refuse(solved, pairs, notes)
    pose, xy, r, frame = solved
    return _result(rig, obs, pose, xy, r, frame, pivots, by_mark, fixed, base_tips, pairs,
                   tuple(notes))


def _result(rig, obs, pose, xy, r, frame, pivots, by_mark, fixed, base_tips, pairs, notes):
    """The solved poses and marks, with the checks that refuse a solution that fits."""
    slots, why = {}, []
    for s, (x, y, a) in pose.items():
        pv, first, q0 = pivots[s]
        T0, Tn = rig.T_table_base(s), rig.nominal_pose(s)
        T = T0.copy()
        T[:3, :3] = planar.rz(a) @ T0[:3, :3]
        T[:2, 3] = (x, y)
        if s in base_tips:
            T[2, 3] += float(pv.tip_hand[2] - np.asarray(base_tips[s], float)[2])
        mine = np.array([t == s for t, _, _ in obs])
        shift = float(np.linalg.norm(T[:2, 3] - Tn[:2, 3]))
        yaw = planar.yaw_between(T[:3, :3], Tn[:3, :3])
        slots[s] = SlotFit(s, T, T0, pv.tip_hand, pv, first, shift, yaw,
                           float(np.sqrt(np.mean(r[mine] ** 2))), int(mine.sum()),
                           rig.pen_name, q0)
        if shift > POSE_SHIFT_MAX or abs(yaw) > POSE_YAW_MAX:
            why.append(f"refused: {s} {shift:.3f} m and {np.rad2deg(yaw):+.2f} deg from its "
                       f"nominal pose — wrong slot or wrong robot?")
    marks = {}
    for n, p in xy.items():
        at = np.array([m == n for _, m, _ in obs])
        marks[n] = MarkFit(n, p.copy(), p - rig.marks[n][0],
                           "known" if n in fixed else "solved",
                           float(np.sqrt(np.mean(r[at] ** 2))), tuple(sorted(by_mark[n])),
                           "determined by one arm" if len(by_mark[n]) == 1
                           and n not in fixed else "")
    rms = float(np.sqrt(np.mean(r ** 2)))
    if rms > RMS_MAX:
        why.append(f"the touches fit one layout only to {rms * 1e3:.2f} mm RMS (limit "
                   f"{RMS_MAX * 1e3:g}): a mark moved during the job, or a touch is off")
    return MarkSolution(not why, "; ".join(why), slots, marks, rms, float(r.max()), pairs,
                        notes, frame)
