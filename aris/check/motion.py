"""`check`: the verdict on one motion of one arm in one phase, before it may reach the robot."""
from __future__ import annotations

from dataclasses import asdict, replace

import numpy as np

from aris.check import timing
from aris.check.config import Tolerances, read_rig
from aris.check.drawing import pen_report
from aris.check.model import tip
from aris.check.retreat import limit_rows, recovering, retreat_rows
from aris.check.scene import CLASSES, build_scene, clearance
from aris.check.sweep import sweep
from aris.check.verdict import Verdict, measure, verdict
from aris.types import Motion, Phase, Slot, Trajectory

REST = 1e-6           # rad/s, "at rest"; rad, "starts where the arm is"
MOVES = 1e-5          # rad, a motion that turns no joint further than this does nothing
POSITION_TOL = 1e-7   # rad, the driver's own tolerance on the joint position box
# Setting the pen down and taking it up: the pen may reach the drawing surface (the press
# below the paper) at one end, where its round end reads up to 1.3 mm below the tip (the tip
# is the contact point).  It may never go deeper than this below the drawing surface.
PEN_FLOOR = -0.002    # m, pen capsule against the drawing surface, "lower" and "lift" motions

_TITLE = dict(steel="clearance steel", links="clearance paper (links)",
              tool="clearance paper (tool)", pen="clearance paper (pen)",
              walls="clearance walls", parked="clearance parked arms",
              self="clearance self")
PEN_DEPTH = "pen depth (lower, lift)"      # the pen row of a "lower" or "lift" motion
ON_SURFACE = "tip on surface (lower, lift)"  # a lower ends, a lift starts, on the drawing surface
TOUCH_PAPER = "tip on paper"                 # a touch's descent ends, its climb starts, there


def check(config_dir, slot: Slot, motion: Motion, phase: Phase, q_before=None, standing=None,
          surface_z: float | None = None, *, tolerances: Tolerances | None = None) -> Verdict:
    """Everything that is measured, on the motion as it will be flown.

    slot            the arm, by its slot on the frame ("2R"); `phase` names slots too
    standing        {slot: (7,) joints}: other arms standing still somewhere other than their
                    park (park and calibrate jobs); built at those joints like a parked arm
                    at its park, held to the demanded arm-to-arm clearance (the "parked" row,
                    named "standing2R:..." where one is closest).  A slot listed here and in
                    `phase.parked` stands where `standing` says.
    surface_z       m, table frame: the height every drawing tip, lower end and lift start must
                    sit on, in place of the paper less the pen's press (None).  For the air run,
                    the whole plan flown above the paper: the server passes paper + 0.030.  The
                    links, the tool and the lifted pen still clear the real paper; a touch still
                    probes the real paper.
    tolerances      replaces the checker's numerical allowances, which otherwise come from
                    rig.json `checker` (see `config.Tolerances`).  For tests only.
    """
    problem = _malformed(motion, q_before)
    if problem is None:
        try:
            rig = read_rig(config_dir)
        except (OSError, ValueError, KeyError) as e:
            problem = f"cannot read the rig: {e}"
    if problem is None:
        problem = _wrong_phase(rig, slot, phase) or _bad_standing(rig, slot, standing)
    if problem is None and surface_z is not None and not np.isfinite(surface_z):
        problem = "surface_z is not a number"
    if problem is not None:
        return verdict([measure("well formed", 0.0, 1.0, "min", "", problem, ranked=False)])

    standing = {p: np.asarray(q, float) for p, q in (standing or {}).items()}
    if surface_z is not None:            # the press is what puts the surface where it is
        rig = replace(rig, press=rig.paper_z - float(surface_z), paper_map=None)
    opts = dict(asdict(tolerances or rig.tolerances), standing=standing)
    others = dict.fromkeys((*phase.parked, *standing))
    notes = rig.notes + sum((rig.mounts[s].notes for s in (slot, *others)), ())
    if motion.kind in ("draw", "lower", "lift"):
        notes += (f"drawing surface: {_surface(rig)}",)
    if motion.kind == "touch":
        return _touch(rig, slot, motion, phase, q_before, notes, opts)
    ms, worst, at = _one(rig, slot, motion, phase, q_before, opts)
    return verdict(ms, worst, at, notes)


def _one(rig, slot, motion, phase, q_before, o, surface_title=ON_SURFACE):
    """The measurements of one draw, free, lower or lift motion, the smallest clearance beyond
    the demanded one, and where it is."""
    traj = motion.traj
    drawing = motion.kind == "draw"
    setting = motion.kind in ("lower", "lift")
    # The pen's floor is the drawing surface, which lies the press below the real paper plane
    # the scene measures against.
    scene = build_scene(rig, slot, phase.walls, phase.parked, drawing, standing=o["standing"],
                        pen_floor=PEN_FLOOR - rig.press - (rig.paper_z - rig.paper_low)
                        if setting else None)
    ms = [measure("well formed", 1.0, 1.0, "min", "", ranked=False)]
    ms += _ends(traj, q_before)                                      # items 1-2
    r1, r4 = timing.rates(traj.t, traj.q, traj.qd, 1000.0, sub=4)
    ms += _limits(scene.model, r1, r4, o["rate_tol"])                # item 3
    sw = sweep(scene, traj, o["step"], o["tol"])
    titles = dict(_TITLE, pen=PEN_DEPTH) if setting else _TITLE
    # A retreat starts inside the arm-to-arm clearance: it is judged by its direction instead.
    classes = tuple(c for c in CLASSES if c != "parked") if motion.kind == "retreat" else CLASSES
    back = None
    if motion.kind == "retreat":                                     # item 11
        back = recovering(scene.model, traj.q[0], rig.limit_gate)
        ms += retreat_rows(scene, traj, o["step"]) + limit_rows(scene.model, r4.q, back,
                                                                rig.limit_gate)
    ms += _clearances(scene, sw, r4, drawing, titles, classes,       # items 3-6, 8
                      None if back is None else ~back)
    if setting:                                                      # item 8
        ms.append(_on_surface(scene, rig, traj, motion.kind, o["tip_height_tol"],
                              surface_title))
    if drawing:                                                      # item 7
        ms += _pen(scene, rig, traj, motion.tip_base, r1, o["tip_height_tol"], o["line_tol"],
                   o["back_tol"], o["speed_tol"], o["stop_speed"])
    ms.append(_hold(scene, traj, titles, classes))                   # item 9
    worst = min(classes, key=lambda c: sw.per_class[c].value)
    return (ms, sw.per_class[worst].value,
            f"{titles[worst]}: {sw.per_class[worst].where} "
            f"({sw.n_samples} samples, {sw.rounds} refinements)")


def _touch(rig, slot, motion, phase, q_before, notes, o):
    """The calibration's probe for the real paper: its descent checked as a lower and its
    climb as a lift, both against the paper itself (no press: the touch looks for the paper,
    it does not draw).  The extra depth the arm may go on for is not part of the planned path
    and is not checked.  One row per measurement, from the half where it is tighter."""
    paper = replace(rig, press=0.0, paper_map=None)   # planned to the nominal plane
    tr, k = motion.traj, _bottom(motion.traj)
    down = Trajectory(tr.t[:k + 1], tr.q[:k + 1], tr.qd[:k + 1])
    up = Trajectory(tr.t[k:], tr.q[k:], tr.qd[k:])
    halves = [(_one(paper, slot, Motion("lower", down), phase, q_before, o, TOUCH_PAPER),
               "descent"),
              (_one(paper, slot, Motion("lift", up), phase, tr.q[k], o, TOUCH_PAPER), "climb")]
    rows = {}
    for (ms, _, _), half in halves:
        for m in ms:
            m = replace(m, detail=f"{half}; {m.detail}" if m.detail else half)
            if m.name not in rows or _tighter(m, rows[m.name]):
                rows[m.name] = m
    (_, c1, at1), _ = halves[0]
    (_, c2, at2), _ = halves[1]
    worst, at = (c1, f"descent: {at1}") if c1 <= c2 else (c2, f"climb: {at2}")
    return verdict(rows.values(), worst, at, notes)


def _bottom(traj) -> int:
    """The sample where the descent ends: the one furthest, in joints, from the first."""
    return int(np.argmax(np.linalg.norm(traj.q - traj.q[0], axis=1)))


def _tighter(a, b) -> bool:
    if a.passed != b.passed:
        return not a.passed
    ua, ub = a.used, b.used
    if np.isnan(ub):
        return not np.isnan(ua)
    return not np.isnan(ua) and ua > ub


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


def _clearances(scene, sw, r4, drawing, titles=_TITLE, classes=CLASSES, joints=None):
    """`joints` (7,) bool: the joints held to their limits here (all when None; a retreat
    leaves out the ones it is bringing back inside, see retreat.py)."""
    m = scene.model
    margin = np.minimum(r4.q - m.q_min, m.q_max - r4.q)
    if joints is None:
        q_margin = min(sw.q_min_margin, float(np.min(margin)))
    else:                               # per joint: the 4 kHz samples (the driver's own grid)
        q_margin = float(np.min(margin[:, joints])) if joints.any() else np.inf
    ms = [measure("joint positions", q_margin, 0.0, "min", "rad",
                  "closest approach to a joint limit", tol=POSITION_TOL)]
    for c in classes:
        if c == "pen" and drawing:
            continue
        res = sw.per_class[c]
        ms.append(measure(titles[c], res.value + scene.margin[c], scene.margin[c], "min", "m",
                          ("" if res.exact else "at least; ") +
                          (f"{res.where} at t = {res.t:.3f} s" if res.where else "")))
    return ms


def _hold(scene, traj, titles=_TITLE, classes=CLASSES):
    """The last configuration, standing still, at the demanded clearances."""
    end = clearance(scene, traj.q[-1:])
    c = min(classes, key=lambda c: end.value[c][0])
    return measure("hold: clearance at the end", end.value[c][0], 0.0, "min", "m",
                   f"beyond demanded; {titles[c]}: {end.closest(c, 0)}")


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


def _on_surface(scene, rig, traj, kind, tol, title=ON_SURFACE):
    """A lower ends, a lift starts, with the tip on the drawing surface (paper less press)."""
    q = traj.q[-1:] if kind == "lower" else traj.q[:1]
    x, y, z = tip(scene.model, q, scene.T_table_base)[0]
    return measure(title, abs(z - float(rig.surface_at(x, y))), tol, "max", "m",
                   f"tip {(z - rig.paper_z) * 1e3:+.2f} mm from the paper at the "
                   f"{'end' if kind == 'lower' else 'start'}; surface {_surface(rig)}")


def _surface(rig):
    if rig.press < 0.0:
        return f"{-rig.press * 1e3:.1f} mm above the plane of the paper (surface_z given)"
    paper = rig.surface_about
    if rig.press == 0.0:
        return f"{paper}, no press"
    return f"{paper}, less {rig.press * 1e3:.1f} mm (press of pen {rig.pen})"


def _pen(scene, rig, traj, tip_base, r1, tip_height_tol, line_tol, back_tol, speed_tol,
         stop_speed):
    p = pen_report(scene.model, scene.T_table_base, rig.surface_at, traj, tip_base, r1.t, r1.q)
    return [
        measure("tip on paper", p.height, tip_height_tol, "max", "m",
                f"largest |tip - drawing surface|, {_surface(rig)}"),
        measure("tip on line", p.off_line, line_tol, "max", "m", "largest distance from the line"),
        measure("never backwards", p.backwards, back_tol, "max", "m",
                f"{p.length * 1e3:.1f} mm drawn"),
        measure("never stops", p.slowest, stop_speed, "min", "m/s",
                "slowest speed along the line mid-way"),
        measure("tip speed", p.fastest, rig.draw_speed * (1 + speed_tol), "max", "m/s",
                f"pen {rig.pen} draws at {rig.draw_speed * 1e3:.1f} mm/s"),
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
    if motion.kind not in ("draw", "free", "lower", "lift", "touch", "retreat"):
        return f"unknown kind {motion.kind!r}"
    if motion.kind == "touch" and not 0 < _bottom(tr) < len(t) - 1:
        return "a touch goes down and comes back: its bottom must lie between its ends"
    if motion.kind == "draw":
        if motion.tip_base is None or np.shape(motion.tip_base) != (len(t), 3):
            return "a drawing motion needs tip_base, one point per sample"
        if not np.all(np.isfinite(motion.tip_base)):
            return "NaN in tip_base"
    if q_before is not None and (np.shape(q_before) != (7,) or
                                 not np.all(np.isfinite(q_before))):
        return "q_before is not 7 numbers"
    return None


def _wrong_phase(rig, slot, phase) -> str | None:
    if slot not in rig.mounts:
        return (f"no arm in slot {slot!r} on this rig (slots: {', '.join(rig.mounts)})")
    if slot not in phase.active:
        return f"slot {slot} does not move in {phase.name}"
    unknown = [p for p in phase.parked if p not in rig.mounts or p == slot]
    if unknown:
        return f"parked slots {unknown} are not other arms of this rig"
    return None


def _bad_standing(rig, slot, standing) -> str | None:
    for p, q in (standing or {}).items():
        if p not in rig.mounts or p == slot:
            return f"standing slot {p!r} is not another arm of this rig"
        if np.shape(q) != (7,) or not np.all(np.isfinite(np.asarray(q, float))):
            return f"standing slot {p}: the configuration is not 7 numbers"
    return None
