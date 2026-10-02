"""The one command: `aris ...`.

    aris serve  [--host --port --driver sim --speed --uncalibrated --cache --jobs]
    aris draw   <drawing.json|.npz> [--note ..] [--server URL]   submit, follow, report; exit
                                                     0 on PASS; --rest-of <job> draws its leftovers
    aris status | stop | park | rig   [--server URL]
    aris calibrate <slot>                            touch the paper on a grid: the base part
    aris touchoff <slot>                             one touch: the pen part
    aris recover <slot>                              release an arm after a fault
    aris plan   <drawing> [--out dir]                plan and check only: no server, no arms
    aris check  <job dir>                            the checker again on every queued motion

Every command prints its assumptions (rig and calibration digests, calibration state, driver,
speed) once and one PASS or FAIL line at the end.  Only `serve`, `plan` and `check` touch the
planners or the checker; the others talk to the server only.  (Over 400 lines: thirteen
short subcommands and the report printer side by side; the next one splits it by kind.)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

DEFAULT_SERVER = "http://127.0.0.1:8420"
CONFIG = Path(__file__).resolve().parents[1] / "config"


class Http:
    """The server's endpoints over HTTP; `get` and `post` answer (status, JSON)."""

    def __init__(self, base: str):
        self.base = base.rstrip("/")

    def _call(self, method, path, body=None):
        req = urllib.request.Request(self.base + path, data=body, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return r.status, json.loads(r.read() or b"null")
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"null")

    def get(self, path):
        return self._call("GET", path)

    def post(self, path, body: bytes | None = None):
        return self._call("POST", path, body if body is not None else b"")


def say(line: str = "") -> None:
    print(line, flush=True)


def verdict(ok: bool, what: str) -> int:
    say(f"{'PASS' if ok else 'FAIL'}: {what}")
    return 0 if ok else 1


def assumptions_line(a: dict) -> str:
    cal = a.get("calibration", {})
    ok = lambda s: (all(str(v).startswith("applied") for v in s.values()) and bool(s)
                    if isinstance(s, dict) else str(s).startswith("applied"))
    applied = sum(ok(s) for s in cal.values())
    return (f"rig {a.get('rig_digest')}, calibration {a.get('calibration_digest')} "
            f"({applied} of {len(cal)} slots calibrated, base and pen"
            f"{'; UNCALIBRATED: nominal poses' if a.get('uncalibrated') else ''}), "
            f"pen {a.get('pen')}, tracking {a.get('tracking')}, "
            f"driver {a.get('driver')}, speed {a.get('speed')}")


# --------------------------------------------------------------------------- the report


def _mm(x) -> str:
    return "-" if x is None else f"{1e3 * float(x):.1f} mm"


def report_lines(rep: dict) -> list[str]:
    out = [f"state        {rep.get('state')}" + (f" ({rep['why']})" if rep.get("why") else "")]
    if rep.get("kind") == "touchoff":
        ref = rep.get("reference", {})
        out.append(f"slot         {rep.get('arm')}: touch at {ref.get('xy_table_m')} "
                   f"({ref.get('source')}), {rep.get('contacts', 0)} contact")
        t = rep.get("touchoff")
        if t:
            out.append(f"pen          {'passed' if t.get('passed') else 'FAILED: ' + str(t.get('why'))}"
                       + "".join(f", {k} {v}" for k, v in t.items() if k not in ("passed", "why")))
        if rep.get("written"):
            out.append(f"written      {rep['written']}")
    if rep.get("kind") == "calibrate":
        out.append(f"arm          {rep.get('arm')}: {rep.get('points', 0)} points touched "
                   f"({rep.get('contacts', 0)} contacts), {len(rep.get('dropped', []))} out of "
                   f"reach, spin {rep.get('spin_deg')} deg")
        f = rep.get("fit")
        if f:
            out.append(f"plane        {'passed' if f['passed'] else 'FAILED: ' + f['why']}; "
                       f"rms {f['rms_mm']} mm, worst {f['max_residual_mm']} mm, tilt "
                       f"{f['tilt_deg']} deg (roll {f['roll_deg']}, pitch {f['pitch_deg']}), "
                       f"height {f['height_change_mm']} mm")
        if rep.get("written"):
            out.append(f"written      {rep['written']}")
    if rep.get("kind") == "park":
        for a, r in rep.get("arms", {}).items():
            out.append(f"arm {a:<8} {r['result']}")
    if "drawn_m" in rep:
        d = rep.get("drawing", {})
        out.append(f"scale        {d.get('scale', 1.0):.3f}")
        out.append(f"drawn        {rep['drawn_m']:.3f} m of {rep['length_m']:.3f} m")
        out.append(f"left over    {rep['left_m']:.3f} m")
        for reason, m in sorted(rep.get("left_by_reason", {}).items()):
            out.append(f"  {reason:<11}{m:.3f} m")
    c = rep.get("checker")
    if c:
        if "all_queued_checked" in c:
            out.append(f"checker      {c['checked']} motions checked in the planners, "
                       f"{c['refused']} refused (their pieces are left over as failed_check); "
                       f"every queued motion checked: {'yes' if c['all_queued_checked'] else 'NO'}")
            out.append(f"tightest     {_mm(c.get('tightest_clearance_m'))}"
                       + (f" ({c['tightest_at']})" if c.get("tightest_at") else ""))
        else:
            out.append(f"checker      {c['passed']} of {c['checked']} motions passed, tightest "
                       f"clearance {_mm(c.get('tightest_clearance_m'))}")
    for p in rep.get("phases", []):
        out.append(f"phase end    {p['name']}: {'passed' if p['end_check_passed'] else 'FAILED'}"
                   f", {p['tightest']} {_mm(p['clearance_m'])}")
    for key, label in (("first_motion_s", "first motion"), ("planning_s", "planning"),
                       ("total_s", "total")):
        if rep.get(key) is not None:
            out.append(f"{label:<13}{float(rep[key]):.1f} s")
    return out


def job_passed(rep: dict) -> bool:
    """PASS: the job ran to its end and every queued motion carries a passing check.
    Leftovers are reported, not a failure."""
    return bool(rep.get("passed", rep.get("state") == "done"))


def _summary(rep: dict) -> str:
    left = ", ".join(f"{r} {m:.3f} m" for r, m in sorted(rep.get("left_by_reason", {}).items()))
    return (f"{rep.get('state')}, drawn {rep.get('drawn_m', 0):.3f} m of "
            f"{rep.get('length_m', 0):.3f} m, left over {rep.get('left_m', 0):.3f} m"
            + (f" ({left})" if left else ""))


# --------------------------------------------------------------------------- server commands


def _progress(v: dict) -> str:
    arms = v.get("arms", [])
    q, d = sum(r["queued"] for r in arms), sum(r["done"] for r in arms)
    cur = [f"arm {r['arm']} motion {r['current']['index']} ({r['current']['kind']})"
           for r in arms if r.get("current")]
    return (f"[{v.get('elapsed_s', 0.0):6.1f} s] {v['state']}: {d} of {q} queued motions done"
            + (f"; {', '.join(cur)}" if cur else ""))


def follow(http, jid: str, poll: float = 0.5) -> dict:
    """Prints a progress line whenever something changes, until the job ends."""
    last = None
    while True:
        code, v = http.get(f"/jobs/{jid}")
        if code != 200:
            raise SystemExit(f"aris: job {jid}: {v}")
        line = _progress(v)
        key = line.split("]", 1)[1]
        if key != last:
            say(line)
            last = key
        if v["state"] in ("done", "stopped", "failed") and v.get("report") is not None:
            return v
        time.sleep(poll)


def _assume(http) -> dict | None:
    code, r = http.get("/rig")
    if code != 200:
        say(f"no server: {r}")
        return None
    say(assumptions_line(r))
    return r


def cmd_draw(a, http) -> int:
    q = urllib.parse.urlencode(dict(note=a.note)) if a.note else ""
    if a.rest_of:
        if a.drawing:
            return verdict(False, "give a drawing or --rest-of, not both")
        if _assume(http) is None:
            return verdict(False, "the server does not answer")
        code, r = http.post(f"/jobs?rest_of={urllib.parse.quote(a.rest_of)}"
                            + (f"&{q}" if q else ""), b"")
        return _follow_draw(a, http, code, r)
    if not a.drawing:
        return verdict(False, "no drawing given (or --rest-of <job id>)")
    path = Path(a.drawing)
    if path.suffix.lower() == ".npz":
        from aris.server import drawing
        lines = drawing.load(path)
        if not isinstance(lines, list):
            return verdict(False, f"{path}: {lines.reason}: {lines.detail}")
        body = json.dumps(drawing.to_dict(lines)).encode()
    else:
        body = path.read_bytes()
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    code, r = http.post(f"/jobs?name={urllib.parse.quote(path.name)}" + (f"&{q}" if q else ""),
                        body)
    return _follow_draw(a, http, code, r)


def _follow_draw(a, http, code, r) -> int:
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    rep = v["report"]
    return verdict(job_passed(rep), _summary(rep))


def cmd_touchoff(a, http) -> int:
    return cmd_calibrate(a, http, "touchoff")


def cmd_calibrate(a, http, kind: str = "calibrate") -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    code, r = http.post(f"/{kind}/{a.arm}")
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    rep = v["report"]
    return verdict(v["state"] == "done", f"{kind} {a.arm}: {v['state']}"
                   + (f", written {rep['written']}" if rep.get("written") else ""))


def cmd_recover(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    code, r = http.post(f"/arms/{a.arm}/recover")
    if code != 200:
        return verdict(False, f"{r}")
    if "queued" in r:
        return verdict(True, f"recover arm {a.arm}: asked the operator PC")
    return verdict(bool(r.get("recovered")), f"recover arm {a.arm}: {r.get('why') or 'done'}")


def cmd_park(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    code, r = http.post("/park")
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "done", f"park {v['state']}")


def cmd_status(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    code, jobs = http.get("/jobs")
    if not jobs:
        say("no jobs yet")
        return verdict(True, "the server answers")
    v = http.get(f"/jobs/{jobs[-1]['id']}")[1]
    say(f"job {v['id']} ({v['kind']}): " + _progress(v))
    for r in v.get("arms", []):
        say(f"  {r['phase']:<12} arm {r['arm']:<3} queued {r['queued']:<4} done {r['done']:<4}"
            f" {r['status']}")
    if v.get("report"):
        for line in report_lines(v["report"]):
            say(line)
    return verdict(True, f"job {v['id']} {v['state']}")


def cmd_stop(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    running = [j for j in http.get("/jobs")[1] if j["state"] not in ("done", "stopped",
                                                                       "failed")]
    if not running:
        return verdict(True, "no job is running")
    jid = running[0]["id"]
    code, r = http.post(f"/jobs/{jid}/stop")
    if code != 200:
        return verdict(False, f"{r}")
    v = follow(http, jid, a.poll)
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "stopped", f"job {jid} {v['state']}")


def cmd_rig(a, http) -> int:
    r = _assume(http)
    if r is None:
        return verdict(False, "the server does not answer")
    c = r.get("drawing_area_centre_m") or [0.0, 0.0]
    say(f"drawing area {r['drawing_area_m'][0]:.3f} x {r['drawing_area_m'][1]:.3f} m around "
        f"({c[0]:+.3f}, {c[1]:+.3f}), canvas {r['canvas_m'][0]:.3f} x {r['canvas_m'][1]:.3f} m")
    say(f"pen {(r.get('pen_in') or {}).get('name')}, tracking {r.get('tracking')}")
    files = r.get("calibration_files", {})
    for aid, arm in r["arms"].items():
        T = arm["T_table_base"]
        f = files.get(aid, {})
        parts = "; ".join(f"{k} {'passed' if f[k]['passed'] else 'FAILED'} {f[k]['date']}"
                          for k in ("base", "pen") if k in f) or "no file"
        say(f"slot {aid:<3} axis ({T[0][3]:+.4f}, {T[1][3]:+.4f}) m  calibration "
            f"{arm['calibration']}  [{parts}]")
    return verdict(True, "rig read")


# --------------------------------------------------------------------------- local commands


def _station(a, with_arms: bool):
    from aris.server import open_station
    from aris.system.settings import Settings
    return open_station(a.config, driver=getattr(a, "driver", "sim"),
                        speed=getattr(a, "speed", 1.0), uncalibrated=a.uncalibrated,
                        cache_dir=None if a.cache in ("", "none") else a.cache,
                        jobs_dir=getattr(a, "jobs", "out/jobs"), workers=a.workers,
                        settings=Settings(grid_step=a.map_grid), with_arms=with_arms,
                        sim_paper=_sim_paper(getattr(a, "sim_paper", None)),
                        tracking=getattr(a, "tracking", "position"))


def _sim_paper(text):
    """"dz_mm,roll_deg,pitch_deg" -> (dz m, roll deg, pitch deg), or None."""
    if not text:
        return None
    dz, roll, pitch = (float(x) for x in text.split(","))
    return dz * 1e-3, roll, pitch


def cmd_serve(a, _http=None) -> int:
    from aris.server.server import serve
    st = _station(a, with_arms=True)
    if not hasattr(st, "rig"):
        return verdict(False, f"the server will not start: {st.reason}: {st.detail}")
    say(assumptions_line(st.assumptions()))
    say(f"drawing area {st.drawing_area[0]:.3f} x {st.drawing_area[1]:.3f} m; "
        f"listening on http://{a.host}:{a.port}")
    serve(st, a.host, a.port)
    return verdict(True, "server stopped")


def cmd_plan(a, _http=None) -> int:
    from aris.execute import Job
    from aris.server import drawing, pipeline, report, runner
    from aris.server.jobs import JobRecord
    st = _station(a, with_arms=False)
    if not hasattr(st, "rig"):
        return verdict(False, f"{st.reason}: {st.detail}")
    say(assumptions_line(st.assumptions()))
    lines = drawing.load(a.drawing)
    if not isinstance(lines, list):
        return verdict(False, f"{lines.reason}: {lines.detail}")
    fitted = drawing.fit(lines, st.drawing_area)
    if not isinstance(fitted, tuple):
        return verdict(False, f"{fitted.reason}: {fitted.detail}")
    out_dir = Path(a.out or f"out/plans/{time.strftime('%Y%m%d-%H%M%S')}-{Path(a.drawing).stem}")
    rec = JobRecord(out_dir.name, "draw", out_dir, Path(a.drawing).name, time.time())
    rec.lines, rec.fit = fitted
    job = Job.create(out_dir, runner.job_header(st, rec, dict(scale=rec.fit.scale)))
    rec.set_state("planning")
    out = pipeline.plan_into(st, job, rec.lines, None, rec)
    bad = out.error or (out.refusal and f"{out.refusal.reason}: {out.refusal.detail}")
    state = "failed" if bad else "done"
    rep = report.draw_report(st, rec, job, out, None, None, None, state, bad or "")
    rep["total_s"] = time.time() - rec.t_received
    (out_dir / "report.json").write_text(json.dumps(rep, indent=1, default=str))
    rec.set_state(state, bad or "")
    say(f"queues in {out_dir}")
    for line in report_lines(rep):
        say(line)
    return verdict(job_passed(rep), "planned: " + _summary(rep))


def cmd_check(a, _http=None) -> int:
    from aris.server.recheck import recheck
    return recheck(Path(a.job), Path(a.config), a.check_workers, say, verdict,
                   assumptions_line)


# --------------------------------------------------------------------------- arguments


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aris", description="the six-arm drawing rig")
    sub = p.add_subparsers(dest="cmd", required=True)

    def local(s):
        s.add_argument("--config", default=str(CONFIG), help="the rig's config folder")
        s.add_argument("--uncalibrated", action="store_true",
                       help="run on the nominal poses when calibration files are missing")
        s.add_argument("--cache", default="out/cache",
                       help="drawable maps and kinematic table ('none': build every time)")
        s.add_argument("--workers", type=int, default=None, help="planner processes")
        s.add_argument("--map-grid", type=float, default=0.02,
                       help="m between the drawable maps' grid points")

    s = sub.add_parser("serve", help="run the drawing server")
    local(s)
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=8420)
    s.add_argument("--driver", default="sim",
                   help="sim: simulated arms here; robot: the operator PC runs the arms")
    s.add_argument("--speed", type=float, default=1.0, help="simulated arm: times real time")
    s.add_argument("--tracking", default="position", choices=("position", "impedance"),
                   help="how the operator PC flies motions: position (mode A, the default) or "
                   "impedance (mode B, the pen force)")
    s.add_argument("--sim-paper", default=None,
                   help="simulated arms' paper: dz_mm,roll_deg,pitch_deg against the nominal")
    s.add_argument("--jobs", default="out/jobs", help="where the job directories go")
    for name, what in (("draw", "submit a drawing and follow it"),):
        s = sub.add_parser(name, help=what)
        s.add_argument("drawing", nargs="?", default=None)
        s.add_argument("--note", default="", help="the material and anything else: kept in "
                       "the job's header and report")
        s.add_argument("--rest-of", default=None, metavar="JOB",
                       help="draw what that finished job left over")
    for name, what in (("calibrate", "touch the paper on a grid: the slot's base calibration"),
                       ("touchoff", "one touch at the reference point: the slot's pen calibration"),
                       ("recover", "release an arm after a fault, once a person has looked")):
        s = sub.add_parser(name, help=what)
        s.add_argument("arm", help="the slot, e.g. 2R")
    for name, what in (("status", "the current or last job"), ("stop", "stop the job"),
                       ("park", "park all arms"), ("rig", "the rig the server runs")):
        sub.add_parser(name, help=what)
    for s in (sub.choices[n] for n in ("draw", "status", "stop", "park", "rig", "calibrate",
                                       "touchoff", "recover")):
        s.add_argument("--server", default=DEFAULT_SERVER)
        s.add_argument("--poll", type=float, default=0.5, help=argparse.SUPPRESS)
    s = sub.add_parser("plan", help="plan and check a drawing; no server, no arms")
    s.add_argument("drawing")
    s.add_argument("--out", default=None, help="the job directory to write")
    local(s)
    s = sub.add_parser("check", help="check every motion of a job's queues again")
    s.add_argument("job")
    s.add_argument("--config", default=str(CONFIG))
    s.add_argument("--check-workers", type=int, default=8)
    return p


COMMANDS = dict(serve=cmd_serve, draw=cmd_draw, status=cmd_status, stop=cmd_stop,
                park=cmd_park, rig=cmd_rig, plan=cmd_plan, check=cmd_check,
                calibrate=cmd_calibrate, recover=cmd_recover, touchoff=cmd_touchoff)


def main(argv=None, http=None) -> int:
    """`http`: an object with `get(path)` and `post(path, body)` answering (status, JSON), in
    place of HTTP to `--server` (tests)."""
    a = parser().parse_args(argv)
    if http is None and hasattr(a, "server"):
        http = Http(a.server)
    return COMMANDS[a.cmd](a, http)


if __name__ == "__main__":
    sys.exit(main())
