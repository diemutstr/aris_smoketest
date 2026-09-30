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


def reported_where(st, need_all: bool) -> dict | Refusal:
    """--driver robot: where the operator PC last said each arm stands.  `need_all`: an arm
    that never reported refuses (park); otherwise it is taken to stand at its park (the
    runner refuses to move an arm that is not at a motion's start)."""
    pos = st.positions.all()
    missing = [a for a in st.rig.arm_ids if a not in pos]
    if need_all and missing:
        return Refusal("no_position", "no position reported for arm "
                       + ", ".join(str(a) for a in missing))
    return {a: (pos[a]["q"] if a in pos else st.rig.park_q(a)) for a in st.rig.arm_ids}


def _arm_configs(st, where) -> dict | Refusal:
    """Arms away from their park, which the planner starts from where they stand.  Only the
    arms of the first phase may be away (every later phase assumes the others parked).  With
    the robot, an arm within the executor's start tolerance of its park counts as parked."""
    first = st.rig.phase(1).active
    tol = st.rig.execution().start_tolerance if st.remote else None
    near = (lambda a, q: float(np.max(np.abs(np.asarray(q) - st.rig.park_q(a)))) <= tol) \
        if tol is not None else (lambda a, q: at_park(st.rig, a, q))
    away = {a: q for a, q in where.items() if not near(a, q)}
    bad = [a for a in away if a not in first]
    if bad:
        return Refusal("not_parked", f"arms {bad} are not at their park; park all arms first")
    return away


def _run_draw(st, rec: JobRecord, job: Job) -> None:
    if st.remote:
        return _run_draw_remote(st, rec, job)
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


ROBOT_STOP_WAIT = 30.0      # s the server waits for the operator PC to confirm a stop


def _run_draw_remote(st, rec: JobRecord, job: Job) -> None:
    """--driver robot: plan, check and queue here; the operator PC's runner copies the queues,
    runs them and posts its events.  The job ends with the runner's own "job ..." row, or,
    after a stop, when the runner confirms it (at most ROBOT_STOP_WAIT; at once if no runner
    ever reported)."""
    try:
        configs = _arm_configs(st, reported_where(st, need_all=False))
        if isinstance(configs, Refusal):
            job.end_phases(configs.reason)
            _finish(rec, job.dir, dict(state="failed", why=configs.detail,
                                       assumptions=st.assumptions()), "failed", configs.detail)
            return
        rec.set_state("planning")
        out = pipeline.plan_into(st, job, rec.lines, configs, rec,
                                 on_first=lambda: rec.set_state("drawing"))
        run = _wait_robot(rec)
        state, why = _end_state(rec, out, run)
        rows, first = arm_progress(rec.log.read())
        done = {k: r["done"] for k, r in rows.items()}
        rep = report.draw_report(st, rec, job, out, run, done,
                                 None if first is None else first - rec.t_received, state, why)
        _finish(rec, job.dir, rep, state, why)
    except Exception as e:
        rec.log.write("error", why=traceback.format_exc())
        _finish(rec, job.dir, dict(state="failed", why=f"internal error: {e!r}",
                                   assumptions=st.assumptions()), "failed",
                f"internal error: {e!r}")


def _wait_robot(rec: JobRecord):
    """Until the operator PC reports the job's end (or a stop it does not confirm)."""
    while not rec.robot_end.wait(0.1):
        if rec.stop.is_set() and (rec.robot_rows == 0 or time.time() - (
                rec.stop_time or time.time()) > ROBOT_STOP_WAIT):
            break
    return _robot_run(rec)


def _robot_run(rec: JobRecord):
    """The operator PC's run, from the rows it posted: like the coordinator's JobRun."""
    from aris.execute import JobRun
    run = JobRun(status="stopped", why="the operator PC did not report the end of the job")
    rows = [r for r in rec.log.read() if r.get("source") == "robot"]
    run.phase_ends = [(r["phase"], r["passed"], r.get("tightest"), r.get("min_clearance"))
                      for r in rows if r.get("event") == "phase end check"]
    run.phases_done = [r["phase"] for r in rows if r.get("event") == "phase done"]
    end = rec.robot_final
    if end is not None:
        run.status, run.why = end["event"].split(" ", 1)[1], end.get("why", "")
        run.where = {int(a): np.asarray(q, float) for a, q in end.get("where", {}).items()}
    return run


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
    if st.remote:
        where = reported_where(st, need_all=True)
        if isinstance(where, Refusal):
            return where
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
        where = reported_where(st, need_all=True) if st.remote else prepare_arms(st)
        if isinstance(where, Refusal):
            job.end_phases(where.reason)
            _finish(rec, job.dir, dict(state="failed", why=where.detail, kind="park",
                                       assumptions=st.assumptions()), "failed", where.detail)
            return
        rec.set_state("planning")
        t0 = time.perf_counter()
        steps = plan_park(st, where)
        planning_s = time.perf_counter() - t0
        moving = [s for s in steps if s.motions and not s.why]
        for name in dict.fromkeys(s.phase.name for s in moving):     # phases in order
            these = [s for s in moving if s.phase.name == name]
            job.add_phase(these[0].phase)
            for s in these:
                q = job.queue(name, s.arm)
                for m, v in zip(s.motions, s.verdicts):
                    q.append(m, v)
                    rec.count_queued(name, s.arm)
                q.close()
                if s.fields:
                    pipeline.save_context(pipeline.context_path(q), s.phase, s.fields)
        job.end_phases("park planned")
        if moving:
            rec.set_state("moving")
        if st.remote:                        # the operator PC runs it and reports
            run = _wait_robot(rec) if moving else _robot_run(rec)
            if not moving:
                run.status, run.why = "done", ""
            run.where = {**where, **run.where}
        else:
            coord = Coordinator(job, st.drivers, st.config_dir, st.rig)
            rec.coordinator = coord
            if rec.stop.is_set():
                coord.stop()
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
    tol = st.rig.execution().start_tolerance if st.remote else 1e-6
    left = [f"arm {a}" for a, q in run.where.items()
            if float(np.max(np.abs(np.asarray(q) - st.rig.park_q(a)))) > tol]
    if run.status != "done":
        return "failed", run.why
    if left:
        return "failed", "not parked: " + ", ".join(left) + "; " + "; ".join(
            f"arm {s.arm}: {s.why}" for s in steps
            if s.why and s.why != "already at its park")
    return "done", ""


def _park_arms(st, steps, run) -> dict:
    """Per arm: parked, already at its park, or why not; its motions and how long they take."""
    out = {}
    for a in st.rig.arm_ids:
        mine = [s for s in steps if s.arm == a]
        why = next((s.why for s in mine if s.why), "")
        ms = [m for s in mine if not s.why for m in s.motions]
        out[str(a)] = dict(result=why or ("parked" if ms else "already at its park"),
                           motions=[m.kind for m in ms],
                           motion_s=sum(float(m.traj.t[-1] - m.traj.t[0]) for m in ms),
                           at_park=a in run.where and bool(at_park(st.rig, a, run.where[a])))
    return out


def _park_report(st, rec, steps, run, planning_s) -> dict:
    state, why = _park_end(st, rec, steps, run)
    checked = [v for s in steps for v in s.verdicts]
    rows, first = arm_progress(rec.log.read())
    return dict(state=state, why=why, kind="park",
                arms=_park_arms(st, steps, run),
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
    rec.stop_time = time.time()
    rec.stop.set()
    if rec.coordinator is not None:
        rec.coordinator.stop()
    for d in st.drivers.values():
        d.stop()
    rec.log.write("stop requested", job=rec.id)
    return None
