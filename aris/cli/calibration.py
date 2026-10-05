"""The calibration jobs: calibrate (the plane), touchoff (the pen), mark (x, y, yaw and tip),
and recover."""
from __future__ import annotations

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
    say(f"job {r['id']}: the arms' lights and the pilot buttons from here on "
        "(white: guide the pen onto the mark, then ✓; ✗ redo; ○ skip)")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "done", f"mark {' '.join(v['report'].get('slots', []))}: "
                   f"{v['state']}")


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
