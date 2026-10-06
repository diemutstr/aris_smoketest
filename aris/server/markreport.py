"""The mark job's end: the joint solve over every arm of the group, the files when it passes,
and the report (mark.py runs the job)."""
from __future__ import annotations

import numpy as np

from aris.server.jobs import arm_progress


def _angle(R) -> float:
    return float(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)))


def _end(rec, why, run) -> tuple[str, str]:
    if rec.stop.is_set():
        return "stopped", "stop requested"
    if why:
        return "failed", why
    if run.status != "done":
        return "failed", run.why
    return "done", ""


def finish(st, rec, job, slots, book, run, why, before) -> None:
    """Solve (`aris.calib.solve_marks`, the marks solved before as known, the base parts' tips
    for the height), write when it passes (`files.write_mark_solution`), report.  Nothing is
    written unless the joint solve passes."""
    from aris.calib import files, solve_marks
    from aris.server import runner
    from aris.server.mark import solver_input
    rig = st.rig
    state, why = _end(rec, why, run)
    sol, written = None, []
    if state == "done":
        # a full calibration (every controlled slot) solves the marks afresh; a subset keeps
        # the marks solved before
        known = {} if set(slots) == set(rig.arm_ids) else \
            {n: rig.mark_xy(n) for n in rig.marks if rig.mark_state(n) == "solved"}
        touches = solver_input(book.touches)
        if not touches or not all(touches.values()):      # never an empty set to the solver
            sol = None
            state, why = "failed", "no usable touches to solve from (every one skipped)"
        else:
            sol = solve_marks(rig, touches, known, files.base_tips(st.config_dir, slots))
        if sol is not None and not sol.passed:
            state, why = "failed", f"the solve did not pass: {sol.why}"
        elif sol is not None:
            written = [str(p) for p in files.write_mark_solution(rig, sol, st.config_dir)]
            st.reload()
    rows, first = arm_progress(rec.log.read())
    rep = dict(state=state, why=why, kind="mark", slots=list(slots),
               touches=len([t for t in book.touches if not t["skipped"]]),
               buttons=book.buttons, notes=book.notes, redone=book.redone, written=written,
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               first_motion_s=None if first is None else first - rec.t_received,
               assumptions=st.assumptions())
    if sol is not None:
        rep.update(solution(rig, sol, before))
    runner.finish_job(rec, job.dir, rep, state, why)


def solution(rig, sol, before) -> dict:
    """Per slot the pose and tip change against the job's start and its residuals; per mark
    its position, state and residual; the pairs of arms' disagreement; the solver's notes."""
    mm = lambda x: round(float(x) * 1e3, 4)
    per_slot = {}
    for s, f in (sol.slots or {}).items():
        T0, tip0 = before.get(s, (f.T_before, None))
        T = np.asarray(f.T_table_base, float)
        per_slot[s] = dict(moved_mm=[mm(x) for x in T[:3, 3] - T0[:3, 3]],
                           turned_mrad=mm(_angle(T[:3, :3] @ T0[:3, :3].T)),
                           yaw_mrad=mm(f.yaw), from_nominal_axis_mm=mm(f.shift),
                           tip_change_mm=None if tip0 is None or f.tip_hand is None else
                           [mm(x) for x in np.asarray(f.tip_hand) - tip0],
                           pivot_residuals_mm=[mm(x) for x in f.pivot.residuals],
                           residual_rms_mm=mm(f.rms), touches=int(f.n_touches),
                           pivot_mark=f.pivot_mark)
    marks = {n: dict(xy_m=[round(float(v), 6) for v in m.xy], state=m.state,
                     from_nominal_mm=[mm(v) for v in (np.asarray(m.from_nominal)
                                                       if getattr(m, "from_nominal", None)
                                                       is not None else
                                                       np.asarray(m.xy) - rig.marks[n][0])],
                     residual_mm=mm(m.residual), by=list(m.by), note=m.note)
             for n, m in (sol.marks or {}).items()}
    pairs = [dict(marks=[a, b], slots=[s, t], disagreement_mm=mm(d))
             for a, b, s, t, d in (sol.pairs or ())]
    return dict(per_slot=per_slot, marks=marks, pairs=pairs,
                rms_mm=None if not np.isfinite(sol.rms) else mm(sol.rms),
                notes_solver=list(sol.notes or ()), solver_why=sol.why)
