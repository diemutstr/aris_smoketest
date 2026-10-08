"""Running a job in the background: the one place that touches both planning and the arms.

A drawing job: fit the drawing, create the job directory, start the coordinator (it waits for
phases), then plan -> check -> queue (pipeline.py) while the arms already run what is queued.
When the planner is done and the coordinator has finished, the report is written.

Every job, of whatever kind, goes through the same frame (`start`): admitted, its directory
made, its work run in a thread, announced to the operator PC.  The drawing job's work is here;
park, calibrate and touch-off are a short list of planned steps (steps.py `steps_work`).

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


def announce(st, rec) -> None:
    """With the robot, every admitted job is a "run" command for the operator PC."""
    if st.remote and st.operator is not None:
        st.operator.push("run", job=rec.id, kind=rec.kind)


def job_header(st, rec, extra) -> dict:
    h = header_for(st.rig, rec.lines, st.rules) if rec.lines else dict(
        rig_digest=st.digests()["rig_digest"],
        calibration_digest=st.digests()["calibration_digest"], rules=None)
    # The job describes itself: the pen that is in (its entry of rig.json's pens table, with
    # its name and press) and the person's note (the material, ...).  The
    # operator PC applies the header's values.
    h.update(code=st.code, operator_pc_code=st.operator.code if st.remote else None,
             pen=st.pen(), note=rec.note, kind=rec.kind, name=rec.name,
             uncalibrated=st.uncalibrated, driver=st.driver_kind, speed=str(st.speed), **extra)
    return h


def finish_job(rec: JobRecord, job_dir, rep: dict, state: str, why: str) -> None:
    rep = dict(rep, id=rec.id, total_s=time.time() - rec.t_received, code=rec.code)
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


def submit_draw(st, store: JobStore, lines, name: str = "", note: str = "",
                rest_of: str | None = None, air_mm: float = 0.0) -> JobRecord | Refusal:
    """Admit a drawing job and start it.  A drawing that cannot be fitted is a failed job.
    `rest_of`: the job whose leftovers these lines are (already in place: not refitted).
    `air_mm`: an air run, the validation every first drawing on the hardware starts with: the
    whole job planned and checked with the drawing surface that far above the paper, so every
    draw is flown in the air; phases, checker and queues otherwise identical."""
    if st.area_problem:                        # nowhere to draw: the planner would refuse all
        return Refusal("no_drawing_area", st.area_problem)
    from aris.server.retreat import too_close
    near = {a: st.positions.known(a)[0] for a in st.rig.arm_ids} if st.remote else \
        {a: np.asarray(d.state().q, float) for a, d in st.drivers.items()}   # read only
    near = {a: (q if q is not None else st.rig.park_q(a)) for a, q in near.items()}
    if set(near) == set(st.rig.arm_ids):
        why = too_close(st, near)
        if why:
            return Refusal("too_close", why)
    if not air_mm >= 0.0:                      # below the paper is a deeper press, not air
        return Refusal("air", f"--air {air_mm} mm is not a height above the paper")
    fitted = drawing.fit(lines, st.drawing_area, st.drawing_centre)

    def prepare(rec):
        rec.note, rec.rest_of, rec.air_mm = note, rest_of, float(air_mm)
        rec.lines = list(lines) if isinstance(fitted, Refusal) else fitted[0]
        rec.fit = None if isinstance(fitted, Refusal) else fitted[1]
        return dict(drawing_area=list(st.drawing_area), drawing_centre=list(st.drawing_centre),
                    drawing_digest_in=digest(list(lines)), rest_of=rest_of,
                    air_mm=float(air_mm),
                    scale=None if rec.fit is None else rec.fit.scale)

    def created(rec, job):
        # the drawing as it is planned, so its leftovers can be drawn again later (--rest-of)
        (job.dir / "drawing.json").write_text(json.dumps(drawing.to_dict(rec.lines)))
        if rec.fit is not None:
            rec.set_state("fitted", scale=rec.fit.scale, bbox=rec.fit.bbox_out,
                          bbox_in=rec.fit.bbox_in)

    refused = None if not isinstance(fitted, Refusal) else f"{fitted.reason}: {fitted.detail}"
    return start(st, store, "draw", name, _draw_work(refused), prepare, created)


# --------------------------------------------------------------------------- the job frame


def code_line(st) -> tuple[bool | None, str]:
    """(same?, the line `aris arms` prints) for the operator PC's code against the server's;
    None when the operator PC has not said (or with the simulated arms, which run here)."""
    from aris.version import describe, same
    if not st.remote:
        return None, f"code {describe(st.code)} (the arms are simulated here)"
    op = st.operator.code
    if op is None:
        return None, "operator PC code: not reported yet"
    if same(st.code, op):
        return True, f"operator PC code: same ({describe(st.code)})"
    return False, (f"operator PC code: DIFFERENT — server {describe(st.code)}, operator PC "
                   f"{describe(op)} — update both machines to the same commit")


def code_mismatch(st) -> str:
    """Why a job must not start: the operator PC runs other code than this server ("")."""
    ok, line = code_line(st)
    return line if ok is False else ""


def start(st, store: JobStore, kind: str, name: str, work, prepare=None, created=None,
          need_positions: bool = False) -> JobRecord | Refusal:
    """Every job the same way: refuse (another job runs; with the robot and
    `need_positions`, an arm that never reported where it stands), admit, create the job
    directory with its header, start `work(st, rec, job)` in a thread (an exception in it
    fails the job loudly and stops the arms), and announce it to the operator PC.
    `prepare(rec) -> dict`: fills the record before the header is written and gives the
    header's extra fields; `created(rec, job)`: right after the directory exists."""
    if need_positions and st.remote:
        where = reported_where(st, need_all=True)
        if isinstance(where, Refusal):
            return where
    why = code_mismatch(st)
    if why:
        return Refusal("wrong_code", why)
    rec = store.admit(kind, name)
    if isinstance(rec, Refusal):
        return rec
    rec.code = dict(server=st.code, operator_pc=st.operator.code if st.remote else None)
    extra = prepare(rec) if prepare is not None else {}
    job = Job.create(rec.dir, job_header(st, rec, extra))
    rec.set_state("received", received=rec.t_received)
    if created is not None:
        created(rec, job)
    rec.thread = threading.Thread(target=_guarded, args=(st, rec, job, work), daemon=True,
                                  name=f"job {rec.id}")
    rec.thread.start()
    announce(st, rec)
    return rec


def _guarded(st, rec: JobRecord, job: Job, work) -> None:
    try:
        work(st, rec, job)
    except Exception as e:                       # a bug: the arms stop, the job fails loudly
        if rec.coordinator is not None:
            rec.coordinator.stop()
        rec.log.write("error", why=traceback.format_exc())
        fail_job(st, rec, job, f"internal error: {e!r}")


def fail_job(st, rec: JobRecord, job: Job, why: str, **extra) -> None:
    """End the job as failed before or instead of running it."""
    phases = job.dir / "phases.jsonl"
    if not phases.exists() or '"end"' not in phases.read_text():
        job.end_phases("failed")
    finish_job(rec, job.dir, dict(state="failed", why=why, kind=rec.kind,
                                  assumptions=st.assumptions(), **extra), "failed", why)


def where_now(st, need_all: bool) -> dict | Refusal:
    """Where every arm stands: the simulated arms' own state, or what the operator PC last
    reported (`need_all`: an arm that never reported refuses)."""
    return reported_where(st, need_all) if st.remote else prepare_arms(st)


def rest_lines(store: JobStore, jid: str) -> tuple[list, dict] | Refusal:
    """The leftovers of a finished drawing job (done, stopped, or failed: a link drop mid-job)
    as lines of a new drawing (`<line>#rest`, then
    `#rest2`, ... when a line has several), from its directory: the drawing as it was planned
    (drawing.json) and the report's leftover stretches.  -> (lines, the old report)."""
    rec = store.get(jid)
    if rec is not None and not rec.finished:
        return Refusal("running", f"job {jid} is still {rec.state}; stop it or let it end first")
    d = store.root / jid
    if "/" in jid or ".." in jid or not (d / "report.json").exists():
        return Refusal("no_job", f"no finished job {jid}")
    rep = json.loads((d / "report.json").read_text())
    if rep.get("kind") != "draw" or rep.get("state") not in ("done", "stopped", "failed") \
            or not (d / "drawing.json").exists():
        return Refusal("not_a_drawing", f"job {jid} is not a finished drawing "
                       f"({rep.get('kind')}, {rep.get('state')})")
    by_id = {x.id: x for x in drawing.parse((d / "drawing.json").read_bytes())}
    if "leftovers" not in rep:                   # failed before anything was accounted
        from aris.system import line_length
        rep = dict(rep, leftovers=[dict(line=x.id, s0=0.0, s1=line_length(x))
                                   for x in by_id.values()])
    if not rep.get("leftovers"):
        return Refusal("nothing_left", f"job {jid} left nothing over")
    out, count = [], {}
    for x in rep["leftovers"]:
        n = count[x["line"]] = count.get(x["line"], 0) + 1
        lid = f"{x['line']}#rest" + ("" if n == 1 else str(n))
        out.append(drawing.stretch(by_id[x["line"]], float(x["s0"]), float(x["s1"]), lid))
    return out, rep


def submit_rest(st, store: JobStore, jid: str, note: str = "",
                air_mm: float = 0.0) -> JobRecord | Refusal:
    """A new drawing job of what job `jid` left over."""
    got = rest_lines(store, jid)
    if isinstance(got, Refusal):
        return got
    lines, rep = got
    return submit_draw(st, store, lines, f"rest of {jid}", note or rep.get("note") or "",
                       rest_of=jid, air_mm=air_mm)


def reported_where(st, need_all: bool) -> dict | Refusal:
    """--driver robot: where the operator PC last said each arm stands.  An arm whose last
    reading is no reading (null, all zeros, or stale) refuses the job: nothing is planned from
    zeros.  `need_all`: an arm that never reported refuses too (park, calibrate, mark);
    otherwise it is taken to stand at its park (the runner refuses to move an arm that is not
    at a motion's start)."""
    out, missing = {}, []
    for a in st.rig.arm_ids:
        q, why = st.positions.known(a)
        if why:
            return Refusal("no_joint_states", why)
        if q is None:
            missing.append(a)
            q = st.rig.park_q(a)
        out[a] = q
    if need_all and missing:
        return Refusal("no_position", "no position reported for arm "
                       + ", ".join(str(a) for a in missing))
    return out


def _arm_configs(st, where) -> dict | Refusal:
    """Arms away from their park, which the planner starts from where they stand.  Only the
    arms of the first phase may be away (every later phase assumes the others parked).  With
    the robot, an arm within the executor's start tolerance of its park counts as parked."""
    first = st.rig.phase(1).active
    tol = st.rig.execution().start_tolerance if st.remote else None
    near = (lambda a, q: float(np.max(np.abs(np.asarray(q) - st.rig.park_q(a)))) <= tol) \
        if tol is not None else st.rig.at_park
    away = {a: q for a, q in where.items() if not near(a, q)}
    bad = [a for a in away if a not in first]
    if bad:
        return Refusal("not_parked", f"arms {bad} are not at their park; park all arms first")
    return away


ROBOT_STOP_WAIT = 30.0      # s the server waits for the operator PC to confirm a stop


def _draw_work(refused: str | None):
    """A drawing job: plan, check and queue (pipeline.py) while the arms already run what is
    queued: the simulated arms with the coordinator here; with `--driver robot` the operator
    PC's runner, whose events end the job (or, after a stop, its confirmation: at most
    ROBOT_STOP_WAIT; at once if no runner ever reported)."""
    def work(st, rec: JobRecord, job: Job) -> None:
        if refused:
            return fail_job(st, rec, job, refused)
        where = where_now(st, need_all=False)
        configs = where if isinstance(where, Refusal) else _arm_configs(st, where)
        if isinstance(configs, Refusal):
            return fail_job(st, rec, job, configs.detail)
        result, ct = [], None
        if not st.remote:
            coord = rec.coordinator = Coordinator(job, st.drivers, st.config_dir, st.rig)
            ct = threading.Thread(target=lambda: result.append(coord.run()), daemon=True)
            ct.start()
        rec.set_state("planning")
        out = pipeline.plan_into(st, job, rec.lines, configs, rec,
                                 on_first=lambda: rec.set_state("drawing"))
        if ct is None:
            run = wait_robot(rec)
        else:
            if rec.stop.is_set():
                rec.coordinator.stop()
            ct.join()
            run = result[0]
        state, why = _end_state(rec, out, run)
        rows, first = arm_progress(rec.log.read())
        done = {k: r["done"] for k, r in rows.items()}
        rep = report.draw_report(st, rec, job, out, run, done,
                                 None if first is None else first - rec.t_received, state, why)
        finish_job(rec, job.dir, rep, state, why)
    return work


def wait_robot(rec: JobRecord):
    """Until the operator PC reports the job's end (or a stop it does not confirm)."""
    while not rec.robot_end.wait(0.1):
        if rec.stop.is_set() and (rec.robot_rows == 0 or time.time() - (
                rec.stop_time or time.time()) > ROBOT_STOP_WAIT):
            break
    return robot_run(rec)


def robot_run(rec: JobRecord):
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
        run.where = {str(a): np.asarray(q, float) for a, q in end.get("where", {}).items()}
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


# --------------------------------------------------------------------------- stop


def stop(st, rec: JobRecord) -> Refusal | None:
    """Every arm stops now and holds; the job ends as stopped.  A finished job: nothing to do."""
    if rec.finished:
        return None
    rec.stop_time = time.time()
    rec.stop.set()
    if rec.coordinator is not None:
        rec.coordinator.stop()
    for d in st.drivers.values():
        d.stop()
    rec.log.write("stop requested", job=rec.id)
    return None
