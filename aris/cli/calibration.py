"""The calibration jobs: calibrate (the plane), touchoff (the pen), mark (x, y, yaw and tip),
and recover; and the gripper (grip)."""
from __future__ import annotations

import json
import urllib.parse

from aris.cli.common import _assume, follow, report_lines, say, verdict


def cmd_mark(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    q = urllib.parse.urlencode({k: v for k, v in (("slots", ",".join(a.slots)),
                                                  ("group", a.group or "")) if v})
    code, r = http.post("/mark" + (f"?{q}" if q else ""))
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}: at each mark, the arm stops 3 cm above it; then, at the arm:")
    say("  pinch the enabling button, put the pen on the cross, let go: 2 s still registers it")
    say("  pinching again before the 2 s are up starts the 2 s again (to correct the pen)")
    say("  a brief pinch without moving skips that touch; the arm then moves on by itself")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "done", f"mark {' '.join(v['report'].get('slots', []))}: "
                   f"{v['state']}")


def cmd_grip(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    body = dict(verb=a.verb, **({} if a.width is None else dict(width_m=a.width)))
    code, r = http.post(f"/grip/{a.slot}", json.dumps(body).encode())
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "done", f"grip {a.slot} {a.verb}: {v['state']}")


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
