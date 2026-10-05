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


def finish(st, rec, job, slots, book, run, why, before, solve) -> None:
    """Solve, write when it passes, report.  Nothing is written unless the joint solve passes."""
    from aris.calib import files
    from aris.server import runner
    rig = st.rig
    state, why = _end(rec, why, run)
    sol, written = None, []
    if state == "done":
        known = {n: rig.mark_xy(n) for n in rig.marks if rig.mark_state(n) == "solved"}
        sol = solve(rig, book.touches, known)
        if not sol.passed:
            state, why = "failed", f"the solve did not pass: {sol.why}"
        else:
            for s in slots:
                if s in sol.slots:
                    written += [str(files.write_base(sol.slots[s], st.config_dir)),
                                str(files.write_pen(sol.slots[s], st.config_dir))]
            written.append(str(files.write_marks(sol, st.config_dir)))
            st.reload()
    rows, first = arm_progress(rec.log.read())
    rep = dict(state=state, why=why, kind="mark", slots=list(slots),
               touches=len(book.touches), buttons=book.buttons, notes=book.notes,
               redone=book.redone, written=written,
               phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                       for n, p, t, c in run.phase_ends],
               first_motion_s=None if first is None else first - rec.t_received,
               assumptions=st.assumptions())
    if sol is not None:
        rep.update(solution(rig, sol, slots, before, book.touches))
    runner.finish_job(rec, job.dir, rep, state, why)


def solution(rig, sol, slots, before, touches) -> dict:
    """Per slot the pose and tip change against the job's start, its residuals; per mark its
    position, the distance between where its two arms put it after the solve, and the
    solver's own word."""
    res = np.asarray(getattr(sol, "touch_residuals", []) or [], float)
    per_slot = {}
    for s in slots:
        r = sol.slots.get(s) if hasattr(sol, "slots") else None
        mine = [k for k, t in enumerate(touches) if t["slot"] == s and k < len(res)]
        entry = dict(touches=len([t for t in touches if t["slot"] == s]),
                     residual_rms_mm=None if not mine else
                     round(float(np.sqrt(np.mean(res[mine] ** 2))) * 1e3, 4),
                     residual_max_mm=None if not mine else
                     round(float(np.max(np.abs(res[mine]))) * 1e3, 4))
        if r is not None:
            T0, tip0 = before[s]
            T = np.asarray(r.T_table_base, float)
            d = (T[:3, 3] - T0[:3, 3]) * 1e3
            tip = getattr(r, "tip_hand", None)
            entry.update(moved_mm=[round(float(x), 4) for x in d],
                         turned_mrad=round(_angle(T[:3, :3] @ T0[:3, :3].T) * 1e3, 4),
                         tip_change_mm=None if tip is None else
                         [round(float(x), 4) for x in (np.asarray(tip) - tip0) * 1e3])
        per_slot[s] = entry
    marks = {n: dict(xy_m=[round(float(v), 6) for v in xy], state="solved")
             for n, xy in (getattr(sol, "marks", {}) or {}).items()}
    seen = {}
    for t in touches:                        # where each arm, as solved, puts its touches
        r = sol.slots.get(t["slot"]) if hasattr(sol, "slots") else None
        if r is None or t["orientation"] != 0:
            continue
        H = np.asarray(r.T_table_base, float) @ rig.arm(t["slot"]).fk(
            np.asarray(t["q"], float)[None])[0]
        tip = np.asarray(getattr(r, "tip_hand", None) if getattr(r, "tip_hand", None)
                         is not None else rig.arm(t["slot"]).tool.tip_hand, float)
        seen.setdefault(t["mark"], {})[t["slot"]] = H[:3, :3] @ tip + H[:3, 3]
    for n, by in seen.items():
        if len(by) == 2 and n in marks:
            a, b = by.values()
            marks[n]["between_arms_mm"] = round(float(np.linalg.norm(a - b)) * 1e3, 4)
    return dict(per_slot=per_slot, marks=marks, solver_why=getattr(sol, "why", ""))
