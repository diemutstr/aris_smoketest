"""`check`: the verdict on one motion of one arm in one phase, before it may reach the robot."""
from __future__ import annotations

import numpy as np

from aris.check import timing
from aris.check.config import read_rig
from aris.check.drawing import pen_report
from aris.check.scene import CLASSES, build_scene, clearance
from aris.check.sweep import sweep
from aris.check.verdict import Verdict, measure, verdict
from aris.types import DrawRules, Motion, Phase

REST = 1e-6           # rad/s, "at rest"; rad, "starts where the arm is"
MOVES = 1e-5          # rad, a motion that turns no joint further than this does nothing
POSITION_TOL = 1e-7   # rad, the driver's own tolerance on the joint position box

_TITLE = dict(steel="clearance steel", paper="clearance paper (body)",
              pen="pen above paper", walls="clearance walls", parked="clearance parked arms",
              self="clearance self")


def check(config_dir, arm_id: int, motion: Motion, phase: Phase, q_before=None, *,
          step: float = 1e-3, tol: float = 2.5e-4, rate_tol: float = 0.05,
          tip_height_tol: float = 5e-4, line_tol: float = 2e-4, back_tol: float = 1e-5,
          draw_speed: float = DrawRules().draw_speed, speed_tol: float = 0.02,
          stop_fraction: float = 0.05) -> Verdict:
    """Everything that is measured, on the motion as it will be flown.

    step            m, the most any capsule point may move between two clearance samples
    tol             m, how far under the true minimum a reported clearance may lie
    rate_tol        largest relative difference allowed between the 1 kHz and 4 kHz readings
    tip_height_tol  m, drawing: tip within this of the paper plane
    line_tol        m, drawing: tip within this of the planned line
    back_tol        m, drawing: numerical allowance for "never goes backwards"
    draw_speed      m/s, drawing: the tip may not go faster than this (times 1 + speed_tol)
    stop_fraction   drawing: between getting going and the final stop the speed along the
                    line stays above this fraction of draw_speed
    """
    problem = _malformed(motion, q_before)
    if problem is None:
        try:
            rig = read_rig(config_dir)
        except (OSError, ValueError, KeyError) as e:
            problem = f"cannot read the rig: {e}"
    if problem is None:
        problem = _wrong_phase(rig, arm_id, phase)
    if problem is not None:
        return verdict([measure("well formed", 0.0, 1.0, "min", "", problem, ranked=False)])

    traj = motion.traj
    drawing = motion.kind == "draw"
    scene = build_scene(rig, arm_id, phase.walls, phase.parked, drawing)
    ms = [measure("well formed", 1.0, 1.0, "min", "", ranked=False)]
    ms += _ends(traj, q_before)                                      # items 1-2
    r1, r4 = timing.rates(traj.t, traj.q, traj.qd, 1000.0, sub=4)
    ms += _limits(scene.model, r1, r4, rate_tol)                     # item 3
    sw = sweep(scene, traj, step, tol)
    ms += _clearances(scene, sw, r4, drawing)                        # items 3-6, 8
    if drawing:                                                      # item 7
        ms += _pen(scene, traj, motion.tip_base, r1, tip_height_tol, line_tol, back_tol,
                   draw_speed, speed_tol, stop_fraction)
    ms.append(_hold(scene, traj))                                    # item 9
    worst = min(CLASSES, key=lambda c: sw.per_class[c].value)
    return verdict(ms, sw.per_class[worst].value,
                   f"{_TITLE[worst]}: {sw.per_class[worst].where} "
                   f"({sw.n_samples} samples, {sw.rounds} refinements)")


def _ends(traj, q_before):
    """It moves, starts where the arm is, starts and ends at rest."""
    excursion = float(np.abs(traj.q - traj.q[0]).max())
    ms = [measure("moves", excursion, MOVES, "min", "rad",
                  f"largest joint excursion; {traj.t[-1] - traj.t[0]:.3f} s")]
    if q_before is not None:
        ms.append(measure("starts at q_before", np.abs(traj.q[0] - q_before).max(), REST,
                          "max", "rad"))
    ms.append(measure("at rest at start", np.abs(traj.qd[0]).max(), REST, "max", "rad/s"))
    ms.append(measure("at rest at end", np.abs(traj.qd[-1]).max(), REST, "max", "rad/s"))
    return ms


def _clearances(scene, sw, r4, drawing):
    m = scene.model
    q_margin = min(sw.q_min_margin, float(np.min(np.minimum(r4.q - m.q_min, m.q_max - r4.q))))
    ms = [measure("joint positions", q_margin, 0.0, "min", "rad",
                  "closest approach to a joint limit", tol=POSITION_TOL)]
    for c in CLASSES:
        if c == "pen" and drawing:
            continue
        res = sw.per_class[c]
        ms.append(measure(_TITLE[c], res.value + scene.margin[c], scene.margin[c], "min", "m",
                          ("" if res.exact else "at least; ") +
                          (f"{res.where} at t = {res.t:.3f} s" if res.where else "")))
    return ms


def _hold(scene, traj):
    """The last configuration, standing still, at the demanded clearances."""
    end = clearance(scene, traj.q[-1:])
    c = min(CLASSES, key=lambda c: end.value[c][0])
    return measure("hold: clearance at the end", end.value[c][0], 0.0, "min", "m",
                   f"beyond demanded; {_TITLE[c]}: {end.closest(c, 0)}")


def _limits(model, r1, r4, rate_tol):
    out = []
    lims = (("velocity", "vel", model.qd_max), ("acceleration", "acc", model.qdd_max),
            ("jerk", "jerk", model.qddd_max))
    worst_diff, what = 0.0, ""
    for name, field, lim in lims:
        v1, v4 = getattr(r1, field) / lim, getattr(r4, field) / lim
        j = int(np.argmax(v1))
        out.append(measure(f"{name} at 1 kHz", v1[j], 1.0, "max", "of limit",
                           f"joint {j + 1}: {v1[j] * lim[j]:.4g} of {lim[j]:.4g}"))
        # A trajectory without corners reads the same at 1 and 4 kHz; a corner (a jump in
        # velocity or acceleration) reads higher the finer it is sampled.  Readings under a
        # tenth of the limit are not compared: there a difference is harmless.
        diff = np.abs(v4 - v1) / np.maximum(np.maximum(v1, v4), 0.1)
        diff = np.where(np.maximum(v1, v4) < 0.1, 0.0, diff)
        k = int(np.argmax(diff))
        if diff[k] >= worst_diff:
            worst_diff, what = float(diff[k]), (f"{name}, joint {k + 1}: 1 kHz {v1[k]:.3g}, "
                                                f"4 kHz {v4[k]:.3g} of limit")
    out.append(measure("1 kHz vs 4 kHz", worst_diff, rate_tol, "max", "relative", what))
    return out


def _pen(scene, traj, tip_base, r1, tip_height_tol, line_tol, back_tol, draw_speed,
         speed_tol, stop_fraction):
    p = pen_report(scene.model, scene.T_table_base, scene.paper_z, traj, tip_base, r1.t, r1.q)
    return [
        measure("tip on paper", p.height, tip_height_tol, "max", "m", "largest |tip - paper|"),
        measure("tip on line", p.off_line, line_tol, "max", "m", "largest distance from the line"),
        measure("never backwards", p.backwards, back_tol, "max", "m",
                f"{p.length * 1e3:.1f} mm drawn"),
        measure("never stops", p.slowest, stop_fraction * draw_speed, "min", "m/s",
                "slowest speed along the line mid-way"),
        measure("tip speed", p.fastest, draw_speed * (1 + speed_tol), "max", "m/s"),
    ]


def _malformed(motion, q_before) -> str | None:
    if not isinstance(motion, Motion):
        return "not a Motion"
    tr = motion.traj
    try:
        t, q, qd = (np.asarray(x, float) for x in (tr.t, tr.q, tr.qd))
    except (TypeError, ValueError):
        return "trajectory arrays are not numbers"
    if t.ndim != 1 or len(t) < 2:
        return "empty motion: fewer than two samples"
    if q.shape != (len(t), 7) or qd.shape != (len(t), 7):
        return f"q {q.shape} and qd {qd.shape} do not match t ({len(t)},)"
    if not (np.all(np.isfinite(t)) and np.all(np.isfinite(q)) and np.all(np.isfinite(qd))):
        return "NaN or infinity in the trajectory"
    if np.any(np.diff(t) <= 0):
        return "sample times do not increase"
    if motion.kind not in ("draw", "free"):
        return f"unknown kind {motion.kind!r}"
    if motion.kind == "draw":
        if motion.tip_base is None or np.shape(motion.tip_base) != (len(t), 3):
            return "a drawing motion needs tip_base, one point per sample"
        if not np.all(np.isfinite(motion.tip_base)):
            return "NaN in tip_base"
    if q_before is not None and (np.shape(q_before) != (7,) or
                                 not np.all(np.isfinite(q_before))):
        return "q_before is not 7 numbers"
    return None


def _wrong_phase(rig, arm_id, phase) -> str | None:
    if arm_id not in rig.mounts:
        return f"no arm {arm_id} on this rig"
    if arm_id not in phase.active:
        return f"arm {arm_id} does not move in {phase.name}"
    unknown = [p for p in phase.parked if p not in rig.mounts or p == arm_id]
    if unknown:
        return f"parked arms {unknown} are not other arms of this rig"
    return None
