"""Running a job in the background: the one place that touches both planning and the arms.

A drawing job: fit the drawing, create the job directory, start the coordinator (it waits for
phases), then plan -> check -> queue (pipeline.py) while the arms already run what is queued.
When the planner is done and the coordinator has finished, the report is written.

A park job: plan every arm's way to its park (park.py), queue it, run it.

Stop, at any time: every arm stops at once and holds (the coordinator stops every driver),
the planner is abandoned, and the job is finished: what is left is reported as leftovers.
Nothing resumes a stopped job (Pete has not decided on resume).
"""
from __future__ import annotations

import json
import threading
import time
import traceback

import numpy as np

from aris.execute import Coordinator, Job
from aris.execute.queue import digest, header_for
from aris.server import drawing, pipeline, report
from aris.server.jobs import JobRecord, JobStore, arm_progress
from aris.server.park import at_park, plan_park
from aris.types import Refusal


def prepare_arms(st) -> dict | Refusal:
    """Where every arm stands.  An arm stopped by an earlier job's stop is released (the stop
    was the operator's own); an arm with a fault refuses the job."""
    where = {}
    for a, d in st.drivers.items():
        s = d.state()
        if not s.ok:
            if any(f.startswith("fault") for f in s.flags):
                return Refusal("arm_fault", f"arm {a}: {', '.join(s.flags)}; a person must "
                               "look at it (recovering a fault is not built in the server)")
            r = d.recover()
            if not r.done:
                return Refusal("arm_not_ready", f"arm {a}: {r.why}")
            s = d.state()
        where[a] = np.asarray(s.q, float)
    return where


def _header(st, rec, extra) -> dict:
    h = header_for(st.rig, rec.lines, st.rules) if rec.lines else dict(
        rig_digest=st.digests()["rig_digest"],
        calibration_digest=st.digests()["calibration_digest"], rules=None)
    h.update(kind=rec.kind, name=rec.name, uncalibrated=st.uncalibrated,
             driver=st.driver_kind, speed=str(st.speed), **extra)
    return h


def _finish(rec: JobRecord, job_dir, rep: dict, state: str, why: str) -> None:
    rep = dict(rep, id=rec.id, total_s=time.time() - rec.t_received)
    (job_dir / "report.json").write_text(json.dumps(rep, indent=1, default=_plain))
    rec.report = json.loads(json.dumps(rep, default=_plain))
    rec.set_state(state, why, drawn_m=rep.get("drawn_m"), left_m=rep.get("left_m"))


def _plain(o):
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    return str(o)


# --------------------------------------------------------------------------- drawing


def submit_draw(st, store: JobStore, lines, name: str = "") -> JobRecord | Refusal:
    """Admit a drawing job and start it.  A drawing that cannot be fitted is a failed job."""
    rec = store.admit("draw", name)
    if isinstance(rec, Refusal):
        return rec
    fitted = drawing.fit(lines, st.drawing_area)
    rec.lines = list(lines) if isinstance(fitted, Refusal) else fitted[0]
    rec.fit = None if isinstance(fitted, Refusal) else fitted[1]
    job = Job.create(rec.dir, _header(st, rec, dict(
        drawing_area=list(st.drawing_area), drawing_digest_in=digest(list(lines)),
        scale=None if rec.fit is None else rec.fit.scale)))
    rec.set_state("received", lines=len(lines), received=rec.t_received)
    if isinstance(fitted, Refusal):
        rec.report = dict(state="failed", why=f"{fitted.reason}: {fitted.detail}",
                          assumptions=st.assumptions())
        _finish(rec, job.dir, rec.report, "failed", rec.report["why"])
        return rec
    rec.set_state("fitted", scale=rec.fit.scale, bbox=rec.fit.bbox_out, bbox_in=rec.fit.bbox_in)
    rec.thread = threading.Thread(target=_run_draw, args=(st, rec, job), daemon=True,
                                  name=f"job {rec.id}")
    rec.thread.start()
    return rec


def _arm_configs(st, where) -> dict | Refusal:
    """Arms away from their park, which the planner starts from where they stand.  Only the
    arms of the first phase may be away (every later phase assumes the others parked)."""
    first = st.rig.phase(1).active
    away = {a: q for a, q in where.items() if not at_park(st.rig, a, q)}
    bad = [a for a in away if a not in first]
    if bad:
        return Refusal("not_parked", f"arms {bad} are not at their park; park all arms first")
    return away


def _run_draw(st, rec: JobRecord, job: Job) -> None:
    coord, result, out = None, [], pipeline.Outcome()
    try:
        where = prepare_arms(st)
        configs = where if isinstance(where, Refusal) else _arm_configs(st, where)
        if isinstance(configs, Refusal):
            job.end_phases(configs.reason)
            _finish(rec, job.dir, dict(state="failed", why=configs.detail,
                                       assumptions=st.assumptions()), "failed", configs.detail)
            return
        coord = Coordinator(job, st.drivers, st.config_dir, st.rig)
        rec.coordinator = coord
        ct = threading.Thread(target=lambda: result.append(coord.run()), daemon=True)
        ct.start()
        rec.set_state("planning")
        out = pipeline.plan_into(st, job, rec.lines, configs, rec,
                                 on_first=lambda: rec.set_state("drawing"))
        if rec.stop.is_set():
            coord.stop()
        ct.join()
        run = result[0]
        state, why = _end_state(rec, out, run)
        rows, first = arm_progress(rec.log.read())
        done = {k: r["done"] for k, r in rows.items()}
        rep = report.draw_report(st, rec, job, out, run, done,
                                 None if first is None else first - rec.t_received, state, why)
        _finish(rec, job.dir, rep, state, why)
    except Exception as e:                       # a bug: the arms stop, the job fails loudly
        if coord is not None:
            coord.stop()
        rec.log.write("error", why=traceback.format_exc())
        _finish(rec, job.dir, dict(state="failed", why=f"internal error: {e!r}",
                                   assumptions=st.assumptions()), "failed",
                f"internal error: {e!r}")


def _end_state(rec, out, run) -> tuple[str, str]:
    if rec.stop.is_set():
        return "stopped", "stop requested"
    if out.error:
        return "failed", f"the planner raised: {out.error}"
    if out.refusal is not None:
        return "failed", f"the planner refused: {out.refusal.reason}: {out.refusal.detail}"
    if run.status != "done":
        return "failed", run.why
    return "done", ""


# --------------------------------------------------------------------------- park


def submit_park(st, store: JobStore) -> JobRecord | Refusal:
    rec = store.admit("park", "park all arms")
    if isinstance(rec, Refusal):
        return rec
    job = Job.create(rec.dir, _header(st, rec, {}))
    rec.set_state("received", received=rec.t_received)
    rec.thread = threading.Thread(target=_run_park, args=(st, rec, job), daemon=True,
                                  name=f"job {rec.id}")
    rec.thread.start()
    return rec


def _run_park(st, rec: JobRecord, job: Job) -> None:
    coord = None
    try:
        where = prepare_arms(st)
        if isinstance(where, Refusal):
            job.end_phases(where.reason)
            _finish(rec, job.dir, dict(state="failed", why=where.detail, kind="park",
                                       assumptions=st.assumptions()), "failed", where.detail)
            return
        rec.set_state("planning")
        t0 = time.perf_counter()
        steps = plan_park(st, where)
        planning_s = time.perf_counter() - t0
        moving = [s for s in steps if s.motion is not None and not s.why]
        for s in moving:
            job.add_phase(s.phase)
            q = job.queue(s.phase.name, s.arm)
            q.append(s.motion, s.verdict)
            q.close()
            if s.fields:
                pipeline.save_context(pipeline.context_path(q), s.phase, s.fields)
            rec.count_queued(s.phase.name, s.arm)
        job.end_phases("park planned")
        coord = Coordinator(job, st.drivers, st.config_dir, st.rig)
        rec.coordinator = coord
        if rec.stop.is_set():
            coord.stop()
        if moving:
            rec.set_state("moving")
        run = coord.run()
        _finish(rec, job.dir, _park_report(st, rec, steps, run, planning_s), *_park_end(
            st, rec, steps, run))
    except Exception as e:
        if coord is not None:
            coord.stop()
        rec.log.write("error", why=traceback.format_exc())
        _finish(rec, job.dir, dict(state="failed", why=f"internal error: {e!r}", kind="park",
                                   assumptions=st.assumptions()), "failed",
                f"internal error: {e!r}")


def _park_end(st, rec, steps, run) -> tuple[str, str]:
    if rec.stop.is_set():
        return "stopped", "stop requested"
    left = [f"arm {a}" for a, q in run.where.items() if not at_park(st.rig, a, q)]
    if run.status != "done":
        return "failed", run.why
    if left:
        return "failed", "not parked: " + ", ".join(left) + "; " + "; ".join(
            f"arm {s.arm}: {s.why}" for s in steps if s.why and s.phase is not None)
    return "done", ""


def _park_report(st, rec, steps, run, planning_s) -> dict:
    state, why = _park_end(st, rec, steps, run)
    checked = [s.verdict for s in steps if s.verdict is not None]
    rows, first = arm_progress(rec.log.read())
    return dict(state=state, why=why, kind="park",
                arms={str(s.arm): dict(
                    result=s.why or "parked",
                    motion_s=None if s.motion is None else float(s.motion.traj.t[-1]),
                    at_park=bool(at_park(st.rig, s.arm, run.where[s.arm])))
                    for s in steps},
                checker=dict(checked=len(checked), passed=sum(bool(v.passed) for v in checked),
                             tightest_clearance_m=min((float(v.min_clearance) for v in checked),
                                                      default=None)),
                phases=[dict(name=n, end_check_passed=bool(p), tightest=t, clearance_m=c)
                        for n, p, t, c in run.phase_ends],
                planning_s=planning_s,
                first_motion_s=None if first is None else first - rec.t_received,
                assumptions=st.assumptions())


# --------------------------------------------------------------------------- stop


def stop(st, rec: JobRecord) -> Refusal | None:
    """Every arm stops now and holds; the job ends as stopped."""
    if rec.finished:
        return Refusal("finished", f"job {rec.id} is already {rec.state}")
    rec.stop.set()
    if rec.coordinator is not None:
        rec.coordinator.stop()
    for d in st.drivers.values():
        d.stop()
    rec.log.write("stop requested", job=rec.id)
    return None
