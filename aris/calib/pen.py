"""The touch-off: the pen's length from one touch of the paper (DESIGN.md 4c, section 6 step 4).

The base part (the plane job) says where the paper is under this arm.  The pen touched the paper
once, near a reference point.  At the touch's joints the tip the rig uses now sits some height
above (or below) that paper; the pen really ends on the paper, so the tip is moved along the pen
axis (in the hand frame) until it does.  The pen leans 23 deg in the holder, so a length change
moves the tip down by cos 23 and sideways by sin 23 of it; moving along the axis gets both.
See docs/modules/calib.md.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.types import Slot

# The pass rule.
CORRECTION_MAX_M = 0.005     # tip moved more than this from the pen's nominal length: the wrong
                             # pen in the holder, a slipped or broken lead, or a pen not seated
REFERENCE_MAX_M = 0.030      # touch further than this from the reference point on the paper:
                             # the arm touched somewhere else than planned (wrong spot, wrong slot)
PEN_INTO_PAPER_MIN = np.cos(np.deg2rad(60.0))  # the pen must point into the paper at least this
                             # much (cosine to the paper's normal); flatter, a length error is
                             # mostly sideways and the touch says little about it


@dataclass(frozen=True)
class PenCalibration:
    """What one touch-off (or a pivot, `aris/calib/marks.py`) found.  Metres, hand frame for
    tips, table frame for xy."""
    slot: Slot
    pen: str
    passed: bool
    why: str                           # "" when passed
    tip_hand: np.ndarray | None        # (3,) the measured tip; None when nothing was measured
    tip_hand_nominal: np.ndarray       # (3,) the pen's nominal tip (rig.json pens table)
    tip_hand_before: np.ndarray        # (3,) the tip the rig used at the touch
    correction: float                  # measured minus nominal, along the pen axis (+ longer)
    change: float                      # measured minus before, along the pen axis
    height_before: float               # the before-tip's height above the paper at the touch
    touch_xy_table: np.ndarray | None  # (2,) where the measured tip met the paper
    reference_xy_table: np.ndarray | None  # (2,); None for a pivot (a mark is no place to
                                       # touch off: the pen would sit in its dimple)
    from_reference: float              # distance between the two (m)
    q: np.ndarray                      # (7,) the touch's joints, as given
    base_status: str                   # the rig's base status the plane came from
    method: str = "touchoff"           # or "pivot"
    detail: dict | None = None         # method-specific numbers for the file (plain data)


def touchoff(rig, slot: Slot, contact_q, reference_xy_table, pen_name: str) -> PenCalibration:
    """The pen tip from one touch at `reference_xy_table`, against the slot's measured paper.

    Never raises on bad data: a missing or failed base part, another pen than the rig's, a
    joint reading that is not 7 numbers, a pen lying flat, a correction over 5 mm or a touch
    over 3 cm from the reference come back with `passed` False and a one-line `why`."""
    q = np.asarray(contact_q, float).reshape(-1)
    ref = np.asarray(reference_xy_table, float).reshape(-1)[:2]
    arm = rig.arm(slot)
    tip_nom, tip_before = rig.nominal_tip(slot), arm.tool.tip_hand.copy()
    base = rig.calibration_status(slot)["base"]
    out = dict(slot=slot, pen=pen_name, tip_hand_nominal=tip_nom, tip_hand_before=tip_before,
               reference_xy_table=ref, q=q.copy(), base_status=base)
    nan = float("nan")

    def refuse(why, **found):
        f = dict(tip_hand=None, correction=nan, change=nan, height_before=nan,
                 touch_xy_table=None, from_reference=nan)
        f.update(found)
        return PenCalibration(passed=False, why=why, **out, **f)

    if not base.startswith("applied"):
        return refuse(f"the paper under {slot} is not measured ({base}): run the plane job "
                      f"first")
    if pen_name != rig.pen_name_in(slot):
        return refuse(f"the touch-off is for pen {pen_name!r} but {slot} has "
                      f"{rig.pen_name_in(slot)!r} "
                      f"in: fix the slot's pen in rig.json or the request")
    if q.shape != (7,) or not np.all(np.isfinite(q)) or ref.shape != (2,):
        return refuse(f"the touch needs 7 joint readings and an x, y reference, got "
                      f"{q.shape[0]} and {ref.shape[0]}")

    T = arm.fk(q[None])[0]
    paper = rig.paper(slot)
    u_base = T[:3, :3] @ arm.tool.pen_axis_hand
    into = -float(paper.normal @ u_base)
    if into < PEN_INTO_PAPER_MIN:
        off = np.rad2deg(np.arccos(np.clip(into, -1.0, 1.0)))
        return refuse(f"the pen is {off:.0f} deg off the paper's normal (limit 60): hold it "
                      f"upright for the touch-off")
    p_before = T[:3, :3] @ tip_before + T[:3, 3]
    h = float(paper.normal @ p_before - paper.offset)
    s = h / into                       # along the axis, out of the tip, to reach the paper
    tip = tip_before + s * arm.tool.pen_axis_hand
    corr = float((tip - tip_nom) @ arm.tool.pen_axis_hand)
    touch = rig.to_table(slot, (T[:3, :3] @ tip + T[:3, 3])[None])[0, :2]
    dist = float(np.linalg.norm(touch - ref))
    found = dict(tip_hand=tip, correction=corr, change=s, height_before=h,
                 touch_xy_table=touch, from_reference=dist)
    why = []
    if abs(corr) > CORRECTION_MAX_M:
        why.append(f"the pen is {corr * 1e3:+.2f} mm off its nominal length (limit "
                   f"{CORRECTION_MAX_M * 1e3:g}): the wrong pen, or a slipped or broken lead")
    if dist > REFERENCE_MAX_M:
        why.append(f"the touch is {dist * 1e3:.0f} mm from the reference point (limit "
                   f"{REFERENCE_MAX_M * 1e3:g}): it touched somewhere else than planned")
    if why:
        return refuse("; ".join(why), **found)
    return PenCalibration(passed=True, why="", **out, **found)
