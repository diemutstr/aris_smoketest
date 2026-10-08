"""The drawing jobs and their state: draw, park, status, stop; plan and check without the
server."""
from __future__ import annotations

import json
import time
import urllib.parse
from pathlib import Path

from aris.cli.common import (_assume, _progress, follow, job_passed, report_lines, say, verdict,
                             assumptions_line, _summary)
from aris.cli.rig import _station


def cmd_draw(a, http) -> int:
    q = urllib.parse.urlencode({k: v for k, v in (("note", a.note), ("air_mm", a.air)) if v})
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
    rig = _assume(http)
    if rig is None:
        return verdict(False, "the server does not answer")
    if path.suffix.lower() == ".svg":
        d = _svg(a, path, rig.get("drawing_area_centre_m") or (0.0, 0.0))
        if isinstance(d, str):
            return verdict(False, d)
        body = json.dumps(d).encode()
    else:
        body = path.read_bytes()
    code, r = http.post(f"/jobs?name={urllib.parse.quote(path.name)}" + (f"&{q}" if q else ""),
                        body)
    return _follow_draw(a, http, code, r)


def _svg(a, path: Path, centre) -> dict | str:
    """The SVG as a drawing dict, or why not."""
    from aris.server import svg
    if a.width is None:
        return "an SVG needs --width (metres along the table's x)"
    d = svg.to_drawing(path, a.width, a.at if a.at else centre)
    return d if isinstance(d, dict) else f"{d.reason}: {d.detail}"


def cmd_import(a, _http=None) -> int:
    """An SVG as the drawing JSON, written to -o; nothing is drawn."""
    d = _svg(a, Path(a.svg), a.at or (0.0, 0.0))
    if isinstance(d, str):
        return verdict(False, d)
    out = Path(a.out or Path(a.svg).with_suffix(".json"))
    out.write_text(json.dumps(d) + "\n")
    n = sum(len(x["points"]) for x in d["lines"])
    return verdict(True, f"wrote {out}: {len(d['lines'])} lines, {n} points")


def _follow_draw(a, http, code, r) -> int:
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    rep = v["report"]
    return verdict(job_passed(rep), _summary(rep))


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


def cmd_plan(a, _http=None) -> int:
    from aris.execute import Job
    from aris.server import drawing, pipeline, report, runner
    from aris.server.jobs import JobRecord
    st = _station(a, with_arms=False)
    if not hasattr(st, "rig"):
        return verdict(False, f"{st.reason}: {st.detail}")
    say(assumptions_line(st.assumptions()))
    if st.area_problem:
        return verdict(False, f"no_drawing_area: {st.area_problem}")
    lines = drawing.load(a.drawing)
    if not isinstance(lines, list):
        return verdict(False, f"{lines.reason}: {lines.detail}")
    fitted = drawing.fit(lines, st.drawing_area, st.drawing_centre)
    if not isinstance(fitted, tuple):
        return verdict(False, f"{fitted.reason}: {fitted.detail}")
    out_dir = Path(a.out or f"out/plans/{time.strftime('%Y%m%d-%H%M%S')}-{Path(a.drawing).stem}")
    rec = JobRecord(out_dir.name, "draw", out_dir, Path(a.drawing).name, time.time())
    rec.lines, rec.fit = fitted
    rec.air_mm = float(a.air)
    job = Job.create(out_dir, runner.job_header(st, rec, dict(scale=rec.fit.scale,
                                                              air_mm=rec.air_mm)))
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
