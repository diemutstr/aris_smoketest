"""The touch-off job (`aris touchoff <slot>`): one touch at a reference point on the paper,
after every pen switch or handling of the pencil (DESIGN.md section 4c).  With a geometric
press the pen's length is the tone, so it is measured against the measured paper plane.

The point: the slot's `reference_touch` in its calibration file when the pen part exists, else
the grid point nearest the slot's axis inside the drawing area (and that one is remembered as
the reference: the solver's result writes it into the file).  Planned like one point of the
calibrate job (calibrate.py): hover, touch, home.  The contact joints go to the calib
solver's touch-off (`aris.calib.touchoff(rig, slot, contact_q, reference_xy, pen_name)`), and
a passing result is written as the file's `pen` part (`aris.calib.files.write_pen`); `base` is
not touched.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aris.server.calibrate import grid_points, misses


TRIES = 8               # grid points tried, nearest the axis first


def candidates(st, slot, cfg) -> tuple[list, str]:
    """-> ([(x, y), ...] to try in order, where they came from): the file's reference touch
    alone when the pen part has one; else the grid points, nearest the slot's axis first (the
    one straight under the shoulder is often out of the arm's reach)."""
    from aris.calib import files
    f = files.read(st.config_dir, slot) or {}
    ref = (f.get("pen") or {}).get("reference_touch") or {}
    xy = ref.get("xy_table_m") if isinstance(ref, dict) else None
    if xy is not None and len(xy) == 2:
        return [(float(xy[0]), float(xy[1]))], "the calibration file's reference touch"
    pts = grid_points(st.rig, slot, st.drawing_area, cfg, st.drawing_centre)
    axis = st.rig.T_table_base(slot)[:2, 3]
    order = np.argsort(np.linalg.norm(pts - axis, axis=1), kind="stable")[:TRIES]
    return [(float(pts[k][0]), float(pts[k][1])) for k in order], \
        "the grid point nearest the slot's axis that the arm can touch"


@dataclass(frozen=True)
class TouchPlan:
    """The touch-off's plan: the calibrate job's Plan for one point, and that point."""
    plan: object
    ref: tuple | None
    source: str

    @property
    def steps(self):
        return [] if self.plan is None else self.plan.steps

    @property
    def why(self):
        return "no point to touch in the drawing area" if self.plan is None else self.plan.why

    def failed(self) -> dict:
        out = {} if self.plan is None else self.plan.failed()
        return dict(out, reference=dict(xy_table_m=self.ref, source=self.source))


def plan(st, slot, where, cfg) -> TouchPlan:
    """The first candidate point that can be touched (or the last refusal)."""
    from aris.server.calibrate import plan_calibrate
    pts, source = candidates(st, slot, cfg)
    p = None
    for xy in pts:
        p = plan_calibrate(st, slot, where, cfg, points=[xy], name="touchoff", min_points=1)
        if not p.why:
            return TouchPlan(p, xy, source)
    return TouchPlan(p, pts[0] if pts else None, source)


def job_report(st, rec, tp: TouchPlan, run, planning_s):
    state, why, result, written = solve(st, rec, tp.plan, run, tp.ref, tp.source)
    rep = report(st, rec, tp.plan, run, result, written, planning_s, state, why, tp.ref,
                 tp.source)
    return rep, state, why


def solve(st, rec, plan, run, ref, source):
    """The touch-off solver on the contact; a passing result is the file's pen part."""
    from aris.calib import touchoff as solver
    from aris.calib.files import write_pen
    slot = plan.arm
    if rec.stop.is_set():
        return "stopped", "stop requested", None, None
    rows = rec.log.read()
    contacts = [r for r in rows if r.get("event") == "contact" and r.get("arm") == slot]
    if not contacts:
        why = run.why if run.status != "done" else "the touch met no paper"
        return "failed", f"no contact at {ref}: {why}", None, None
    q = np.asarray(contacts[-1]["q"], float)
    result = solver(st.rig, slot, q, np.asarray(ref, float), st.rig.pen_name)
    if not result.passed:
        return "failed", f"the touch-off did not pass: {result.why}", result, None
    from aris.server.robots import write_kwargs
    path = write_pen(result, st.config_dir, **write_kwargs(st, write_pen, slot))
    st.reload()
    return "done", "", result, str(path)


def report(st, rec, plan, run, result, written, planning_s, state, why, ref, source) -> dict:
    out = dict(state=state, why=why, kind="touchoff", arm=plan.arm,
               reference=dict(xy_table_m=list(ref), source=source),
               contacts=sum(1 for r in rec.log.read() if r.get("event") == "contact"
                            and r.get("arm") == plan.arm),
               missed=misses(plan, rec.log.read(), plan.arm), written=written,
               planning_s=planning_s, spin_deg=None if plan.spin is None
               else round(float(np.rad2deg(plan.spin)), 3),
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               assumptions=st.assumptions())
    if result is not None:
        r, mm = result, lambda x: None if x is None else round(float(x) * 1e3, 4)
        out["touchoff"] = dict(
            passed=bool(r.passed), why=r.why, pen=r.pen,
            tip_hand_m=None if r.tip_hand is None else np.asarray(r.tip_hand).tolist(),
            correction_mm=mm(r.correction), change_mm=mm(r.change),
            height_before_mm=mm(r.height_before), from_reference_mm=mm(r.from_reference),
            touch_xy_table_m=None if r.touch_xy_table is None
            else np.asarray(r.touch_xy_table).tolist())
    return out
