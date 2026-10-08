"""Step 1 of the calibration: the paper plane under one arm, from its own touches.

The arm touched the paper at a grid of points, pen upright, the same hand spin everywhere.  Each
touch's joints give the pen tip in the arm's base frame (the kernel's forward kinematics with the
rig's pen tip for that arm).  A least-squares plane through those tips gives the paper's normal
and height as seen from the base.  The calibrated base pose is the nominal one turned about a
horizontal table axis (roll and pitch only, the smallest turn) so that the table's up direction
matches the fitted normal, then moved along table z so that the plane sits at the paper height.
x, y and the turn about the vertical stay nominal (they need the dimples, steps 2 and 3).

The pen tip in the hand frame is not touched here: with one hand orientation an error in the pen
length moves every touch alike and lands in the height, which is the height we want (where the
pen meets the paper).  See docs/modules/calib.md.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from aris.types import Slot

# The pass rule.  Each limit catches one kind of fault; the sentence for it is in `_verdict`.
# The first three were 9 points, 0.5 mm RMS and 1.5 mm largest: they refused every real plane
# on the wood on 2026-10-06 (RMS 0.87-0.92 mm, largest 1.69-1.83 mm with 8-12 points: the
# table's flatness and the touch noise).  These are the values the site passed them with.
MIN_POINTS = 8            # fewer touches cannot tell a tilt from one bad touch
MIN_SPREAD_M = 0.02       # RMS spread of the touches along their second direction: below this
                          # they lie on a line and no plane is defined
RMS_MAX_M = 0.0010        # scatter of all touches about the plane: a soft or loose paper, a
                          # bent table, or touches from two different set-ups mixed together
MAX_RESIDUAL_M = 0.0025   # one touch far off the plane: a slipped or early (false-onset) touch
TILT_MAX_RAD = np.deg2rad(3.0)  # the paper leans far from what rig.json says: the wrong arm's
                          # touches, or an arm not mounted as rig.json says
HEIGHT_MAX_M = 0.030      # the paper is far above or below where rig.json puts it: the wrong
                          # pen, a wrong base height, or the touch went past the paper


@dataclass(frozen=True)
class PlaneCalibration:
    """Everything step 1 found for one arm.  Lengths in metres, angles in radians; the file
    written from it uses millimetres and degrees for the human-facing numbers."""
    slot: Slot
    passed: bool
    why: str                           # "" when passed
    n_points: int
    T_table_base: np.ndarray           # (4,4) calibrated; the nominal one when no plane was fit
    T_table_base_nominal: np.ndarray   # (4,4)
    normal_base: np.ndarray | None     # (3,) fitted paper normal, toward the base; None if none
    offset_base: float                 # n . p = offset for the fitted plane, base frame
    roll: float                        # turn of the base about table x (rad)
    pitch: float                       # turn of the base about table y (rad)
    tilt: float                        # the whole turn, sqrt(roll^2 + pitch^2) (rad)
    height_change: float               # change of the base's table z (m)
    rms: float                         # residual RMS (m)
    max_residual: float                # largest |residual| (m)
    worst_index: int                   # which touch has it (-1 when no plane)
    points_table: np.ndarray           # (N,3) touches in the calibrated table frame: the
                                       # height map; z - paper_z is each touch's residual
    residuals: np.ndarray              # (N,) signed, + is above the plane (toward the arm)
    # The pen the touches were made with and the tip the rig gave it (hand frame): the height
    # found includes that pen's length error against this tip.
    pen: str = ""
    tip_hand: np.ndarray | None = None


def fit_plane(points) -> tuple[np.ndarray, float, np.ndarray]:
    """Least-squares plane through (N,3) points, N >= 3.  -> (normal, offset, residuals).

    `normal . p = offset` on the plane; the unit normal points toward the frame's origin (the
    arm's base, when the points are in the base frame).  Residuals are `normal . p - offset`,
    positive on the origin's side.  Fewer than 3 points is a caller error (ValueError); points
    on a line give some plane through that line: check the spread first (`calibrate_plane` does).
    """
    p = np.asarray(points, float).reshape(-1, 3)
    if len(p) < 3:
        raise ValueError(f"a plane needs at least 3 points, got {len(p)}")
    centre = p.mean(axis=0)
    _, _, Vt = np.linalg.svd(p - centre)
    n = Vt[2]
    if n @ centre > 0.0:               # make the origin lie on the positive side
        n = -n
    offset = float(n @ centre)
    return n, offset, p @ n - offset


def _spread(p: np.ndarray) -> float:
    """RMS extent of the points along their second principal direction (m)."""
    s = np.linalg.svd(p - p.mean(axis=0), compute_uv=False)
    return float(s[1] / np.sqrt(len(p))) if len(s) > 1 else 0.0


def _horizontal_turn(m_table: np.ndarray) -> np.ndarray:
    """The rotation vector (rx, ry, 0) of the smallest turn taking unit `m_table` onto table +z.
    Its axis m x z is horizontal, so it has no part about the vertical."""
    axis = np.cross(m_table, [0.0, 0.0, 1.0])
    s, c = np.linalg.norm(axis), float(m_table[2])
    if s < 1e-15:
        return np.zeros(3)
    return axis / s * np.arctan2(s, c)


def _rotvec_to_matrix(v: np.ndarray) -> np.ndarray:
    a = float(np.linalg.norm(v))
    if a < 1e-15:
        return np.eye(3)
    k = v / a
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(a) * K + (1.0 - np.cos(a)) * (K @ K)


def _refusal(slot, T_nom, n_points, why) -> PlaneCalibration:
    return PlaneCalibration(slot, False, why, n_points, T_nom.copy(), T_nom.copy(), None,
                            float("nan"), float("nan"), float("nan"), float("nan"),
                            float("nan"), float("nan"), float("nan"), -1, np.zeros((0, 3)),
                            np.zeros(0))


def _verdict(n, rms, worst, k, tilt, dz) -> str:
    """The pass rule; "" when it passes, else every failed limit in one line."""
    why = []
    if n < MIN_POINTS:
        why.append(f"only {n} touches, at least {MIN_POINTS} needed")
    if rms > RMS_MAX_M:
        why.append(f"the touches scatter {rms * 1e3:.2f} mm RMS about the plane (limit "
                   f"{RMS_MAX_M * 1e3:g}): the paper is not one flat surface under this arm")
    if worst > MAX_RESIDUAL_M:
        why.append(f"touch {k} is {worst * 1e3:.2f} mm off the plane (limit "
                   f"{MAX_RESIDUAL_M * 1e3:g}): a slipped or early touch, repeat it")
    if tilt > TILT_MAX_RAD:
        why.append(f"the paper leans {np.rad2deg(tilt):.2f} deg from nominal (limit "
                   f"{np.rad2deg(TILT_MAX_RAD):g}): the wrong arm's touches, or the arm is not "
                   f"mounted as rig.json says")
    if abs(dz) > HEIGHT_MAX_M:
        why.append(f"the paper is {dz * 1e3:+.1f} mm from its nominal height (limit "
                   f"{HEIGHT_MAX_M * 1e3:g}): a far-off plane, check the pen and the base height")
    return "; ".join(why)


def calibrate_plane(rig, slot: Slot, contacts_q) -> PlaneCalibration:
    """The paper plane under `slot` from its contact configurations (N,7), in touch order.

    "Nominal" is the pose the rig has now (rig.json's, or an earlier base part's): the touches
    were planned and are read with it.  Never raises on bad data: too few touches, unreadable
    joints or touches on a line come back as a refusal (`passed` False, `why` set, the pose the
    rig has kept)."""
    return replace(_fit(rig, slot, contacts_q), pen=rig.pen_name_in(slot),
                   tip_hand=rig.arm(slot).tool.tip_hand.copy())


def _fit(rig, slot: Slot, contacts_q) -> PlaneCalibration:
    T_nom = rig.T_table_base(slot)
    Q = np.asarray(contacts_q, float)
    if Q.size == 0:
        return _refusal(slot, T_nom, 0, "no touches")
    if Q.ndim != 2 or Q.shape[1] != 7:
        return _refusal(slot, T_nom, len(Q), f"contacts must be N x 7 joints, got {Q.shape}")
    if not np.all(np.isfinite(Q)):
        bad = int(np.nonzero(~np.all(np.isfinite(Q), axis=1))[0][0])
        return _refusal(slot, T_nom, len(Q), f"touch {bad} has no joint reading")
    if len(Q) < 3:
        return _refusal(slot, T_nom, len(Q),
                        f"only {len(Q)} touches, at least {MIN_POINTS} needed")
    p_base = rig.arm(slot).tip(Q)
    if _spread(p_base) < MIN_SPREAD_M:
        return _refusal(slot, T_nom, len(Q), "the touches lie on a line (or one spot): they "
                        "define no plane, touch a grid")
    n, c, res = fit_plane(p_base)

    # The table's up direction seen from the base must become n: R_new^T z = n.  With
    # R_new = R_turn @ R_nom this asks R_turn to take m = R_nom n onto z.
    R_nom = T_nom[:3, :3]
    v = _horizontal_turn(R_nom @ n)
    T = T_nom.copy()
    T[:3, :3] = _rotvec_to_matrix(v) @ R_nom
    # A plane point's table z is n . p + t_z = c + t_z; set it to the paper height.
    T[2, 3] = rig.paper_z - c
    dz = float(T[2, 3] - T_nom[2, 3])
    rms = float(np.sqrt(np.mean(res ** 2)))
    k = int(np.argmax(np.abs(res)))
    tilt = float(np.linalg.norm(v))
    why = _verdict(len(Q), rms, float(abs(res[k])), k, tilt, dz)
    return PlaneCalibration(
        slot=slot, passed=not why, why=why, n_points=len(Q), T_table_base=T,
        T_table_base_nominal=T_nom.copy(), normal_base=n, offset_base=c,
        roll=float(v[0]), pitch=float(v[1]), tilt=tilt, height_change=dz, rms=rms,
        max_residual=float(abs(res[k])), worst_index=k,
        points_table=p_base @ T[:3, :3].T + T[:3, 3], residuals=res)


def calibration_from_events(rig, slot: Slot, rows) -> PlaneCalibration:
    """`calibrate_plane` on the contact rows of `slot` in an event list, in their order."""
    Q = []
    for r in rows:
        if r.get("event") == "contact" and r.get("arm") == slot:
            q = r.get("q")
            ok = isinstance(q, (list, tuple)) and len(q) == 7
            Q.append(np.asarray(q, float) if ok else np.full(7, np.nan))
    return calibrate_plane(rig, slot, np.array(Q).reshape(-1, 7))
