"""What every command shares: the server over HTTP, the output lines, following a job and
printing its report."""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

DEFAULT_SERVER = "http://127.0.0.1:8420"
CONFIG = Path(__file__).resolve().parents[2] / "config"


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
            f"pen {a.get('pen')}, "
            f"driver {a.get('driver')}, speed {a.get('speed')}")


# --------------------------------------------------------------------------- the report


def _mm(x) -> str:
    return "-" if x is None else f"{1e3 * float(x):.1f} mm"


def report_lines(rep: dict) -> list[str]:
    out = [f"AIR RUN      every draw flown {rep['air_mm']:g} mm above the paper, no contact"] \
        if rep.get("air_mm") else []
    out.append(f"state        {rep.get('state')}" + (f" ({rep['why']})" if rep.get("why") else ""))
    if rep.get("kind") == "mark":
        out.append(f"marks        {rep.get('touches', 0)} touches by {', '.join(rep.get('slots', []))}"
                   f"; buttons {rep.get('buttons', {})}")
        for n in rep.get("notes", []) + [f"redone: {x}" for x in rep.get("redone", [])]:
            out.append(f"  {n}")
        for sl, e in rep.get("per_slot", {}).items():
            out.append(f"slot {sl:<7} moved {e.get('moved_mm')} mm, yaw {e.get('yaw_mrad')} mrad, "
                       f"tip {e.get('tip_change_mm')} mm; rms {e.get('residual_rms_mm')} mm, "
                       f"pivot {e.get('pivot_residuals_mm')} mm")
        for n, e in rep.get("marks", {}).items():
            out.append(f"mark {n:<7} {e.get('state')} at {e.get('xy_m')} m, "
                       f"{e.get('from_nominal_mm')} mm from nominal, rms "
                       f"{e.get('residual_mm')} mm, by {e.get('by')}"
                       + (f" ({e['note']})" if e.get("note") else ""))
        for p in rep.get("pairs", []):
            out.append(f"pair         {p['slots']} on {p['marks']}: {p['disagreement_mm']} mm")
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
        if rep.get("missed"):
            out.append(f"no contact   {rep['missed']}")
        f = rep.get("fit")
        if f:
            out.append(f"plane        {'passed' if f['passed'] else 'FAILED: ' + f['why']}; "
                       f"rms {f['rms_mm']} mm, worst {f['max_residual_mm']} mm, tilt "
                       f"{f['tilt_deg']} deg (roll {f['roll_deg']}, pitch {f['pitch_deg']}), "
                       f"height {f['height_change_mm']} mm")
        if rep.get("written"):
            out.append(f"written      {rep['written']}")
    if rep.get("kind") == "grip":
        w = lambda x: "-" if x is None else f"{1e3 * float(x):.1f} mm"
        out.append(f"gripper      {rep.get('slot')} {rep.get('verb')}: width "
                   f"{w(rep.get('width_before_m'))} -> {w(rep.get('width_after_m'))}"
                   + ("" if rep.get("grasped") is None else
                      f", {'grasped' if rep['grasped'] else 'nothing grasped'}"))
    if rep.get("kind") == "park":
        for a, r in rep.get("arms", {}).items():
            out.append(f"arm {a:<8} {r['result']}")
    if rep.get("kind") == "calibrate" and rep.get("paper_surface"):
        out.append(f"paper map    {paper_line(rep['paper_surface'])}")
    if "drawn_m" in rep:
        d = rep.get("drawing", {})
        out.append(f"scale        {d.get('scale', 1.0):.3f}")
        u = rep.get("paper_under_drawing")
        out.append("paper        flat (no height map)" if not u else
                   f"paper        height map under the drawing: {1e3 * u['z_min_m']:+.2f} to "
                   f"{1e3 * u['z_max_m']:+.2f} mm about the nominal paper "
                   f"({u['range_mm']:.2f} mm)")
        if rep.get("account_error"):
            out.append(f"ACCOUNT      {rep['account_error']}")
        out.append(f"drawn        {rep['drawn_m'] or 0:.3f} m of {rep['length_m']:.3f} m")
        out.append(f"left over    {rep['left_m'] or 0:.3f} m")
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


def paper_line(p: dict) -> str:
    """The paper height map in one line (`aris rig`, the calibrate report)."""
    if not p or not p.get("exists"):
        return "none: the flat paper (run `aris calibrate <slot>` to measure it)"
    rng = ("" if p.get("z_min_m") is None else
           f", {1e3 * p['z_min_m']:+.2f} to {1e3 * p['z_max_m']:+.2f} mm about the nominal paper")
    return f"{p.get('points', 0)} points{rng}, from {p.get('date') or 'an unknown date'}"


def job_passed(rep: dict) -> bool:
    """PASS: the job ran to its end and every queued motion carries a passing check.
    Leftovers are reported, not a failure."""
    return bool(rep.get("passed", rep.get("state") == "done"))


def _summary(rep: dict) -> str:
    left = ", ".join(f"{r} {m:.3f} m" for r, m in sorted(rep.get("left_by_reason", {}).items()))
    return (f"{rep.get('state')}, drawn {rep.get('drawn_m') or 0:.3f} m of "
            f"{rep.get('length_m') or 0:.3f} m, left over {rep.get('left_m') or 0:.3f} m"
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
