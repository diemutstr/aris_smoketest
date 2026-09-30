"""The one command: `aris ...`.

    aris serve  [--host --port --driver sim --speed --uncalibrated --cache --jobs]
    aris draw   <drawing.json|.npz> [--server URL]   submit, follow, report; exit 0 if all drawn
    aris status | stop | park | rig   [--server URL]
    aris plan   <drawing> [--out dir]                plan and check only: no server, no arms
    aris check  <job dir>                            the checker again on every queued motion

Every command prints its assumptions (rig and calibration digests, calibration state, driver,
speed) once and one PASS or FAIL line at the end.  Only `serve`, `plan` and `check` touch the
planners or the checker; the others talk to the server only.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_SERVER = "http://127.0.0.1:8420"
CONFIG = Path(__file__).resolve().parents[1] / "config"
TINY = 1e-6


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
    applied = sum(str(s).startswith("applied") for s in cal.values())
    return (f"rig {a.get('rig_digest')}, calibration {a.get('calibration_digest')} "
            f"({applied} of {len(cal)} arms calibrated"
            f"{'; UNCALIBRATED: nominal poses' if a.get('uncalibrated') else ''}), "
            f"driver {a.get('driver')}, speed {a.get('speed')}")


# --------------------------------------------------------------------------- the report


def _mm(x) -> str:
    return "-" if x is None else f"{1e3 * float(x):.1f} mm"


def report_lines(rep: dict) -> list[str]:
    out = [f"state        {rep.get('state')}" + (f" ({rep['why']})" if rep.get("why") else "")]
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
        out.append(f"checker      {c['passed']} of {c['checked']} motions passed, tightest "
                   f"clearance {_mm(c.get('tightest_clearance_m'))}"
                   + (f" ({c['tightest_at']})" if c.get("tightest_at") else ""))
    for p in rep.get("phases", []):
        out.append(f"phase end    {p['name']}: {'passed' if p['end_check_passed'] else 'FAILED'}"
                   f", {p['tightest']} {_mm(p['clearance_m'])}")
    for key, label in (("first_motion_s", "first motion"), ("planning_s", "planning"),
                       ("total_s", "total")):
        if rep.get(key) is not None:
            out.append(f"{label:<13}{float(rep[key]):.1f} s")
    return out


def all_drawn(rep: dict) -> bool:
    return rep.get("state") == "done" and float(rep.get("left_m", 0.0)) <= TINY


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
    code, r = http.post(f"/jobs?name={urllib.request.quote(path.name)}", body)
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    rep = v["report"]
    return verdict(all_drawn(rep), f"{rep.get('state')}, drawn {rep.get('drawn_m', 0):.3f} m, "
                   f"left over {rep.get('left_m', 0):.3f} m")


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
    say(f"drawing area {r['drawing_area_m'][0]:.3f} x {r['drawing_area_m'][1]:.3f} m, "
        f"canvas {r['canvas_m'][0]:.3f} x {r['canvas_m'][1]:.3f} m")
    for aid, arm in r["arms"].items():
        T = arm["T_table_base"]
        say(f"arm {aid:<3} axis ({T[0][3]:+.4f}, {T[1][3]:+.4f}) m  park "
            + " ".join(f"{x:+.3f}" for x in arm["park_q"]) + f"  calibration {arm['calibration']}")
    return verdict(True, "rig read")


# --------------------------------------------------------------------------- local commands


def _station(a, with_arms: bool):
    from aris.server import open_station
    from aris.system.settings import Settings
    return open_station(a.config, driver=getattr(a, "driver", "sim"),
                        speed=getattr(a, "speed", 1.0), uncalibrated=a.uncalibrated,
                        cache_dir=None if a.cache in ("", "none") else a.cache,
                        jobs_dir=getattr(a, "jobs", "out/jobs"), workers=a.workers,
                        check_workers=a.check_workers,
                        settings=Settings(grid_step=a.map_grid), with_arms=with_arms)


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
    job = Job.create(out_dir, runner._header(st, rec, dict(scale=rec.fit.scale)))
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
    ok = all_drawn(rep) and out.passed == out.checked
    return verdict(ok, f"planned {rep['drawn_m']:.3f} m of {rep['length_m']:.3f} m, "
                   f"{out.passed} of {out.checked} motions passed the checker")


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
        s.add_argument("--check-workers", type=int, default=None, help="checker processes")
        s.add_argument("--map-grid", type=float, default=0.02,
                       help="m between the drawable maps' grid points")

    s = sub.add_parser("serve", help="run the drawing server")
    local(s)
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=8420)
    s.add_argument("--driver", default="sim", help="the arm driver (only 'sim' is built)")
    s.add_argument("--speed", type=float, default=1.0, help="simulated arm: times real time")
    s.add_argument("--jobs", default="out/jobs", help="where the job directories go")
    for name, what in (("draw", "submit a drawing and follow it"),):
        s = sub.add_parser(name, help=what)
        s.add_argument("drawing")
    for name, what in (("status", "the current or last job"), ("stop", "stop the job"),
                       ("park", "park all arms"), ("rig", "the rig the server runs")):
        sub.add_parser(name, help=what)
    for s in (sub.choices[n] for n in ("draw", "status", "stop", "park", "rig")):
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
                park=cmd_park, rig=cmd_rig, plan=cmd_plan, check=cmd_check)


def main(argv=None, http=None) -> int:
    """`http`: an object with `get(path)` and `post(path, body)` answering (status, JSON), in
    place of HTTP to `--server` (tests)."""
    a = parser().parse_args(argv)
    if http is None and hasattr(a, "server"):
        http = Http(a.server)
    return COMMANDS[a.cmd](a, http)


if __name__ == "__main__":
    sys.exit(main())
