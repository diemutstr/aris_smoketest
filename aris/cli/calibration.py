"""The calibration jobs: calibrate (the plane), touchoff (the pen), mark (x, y, yaw: the tips
brought together), crosses (its visual check), and recover; and the gripper (grip)."""
from __future__ import annotations

import json
import urllib.parse

from aris.cli.common import _assume, follow, report_lines, say, verdict


def cmd_crosses(a, http) -> int:
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    q = urllib.parse.urlencode({k: v for k, v in (("slots", ",".join(a.slots)),
                                                  ("group", a.group or "")) if v})
    code, r = http.post("/crosses" + (f"?{q}" if q else ""))
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    say(f"job {r['id']}: each arm draws its shapes at the shared spots (pens in, paper down)")
    v = follow(http, r["id"], a.poll)
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "done", f"crosses: {v['state']}")


def cmd_mark(a, http) -> int:
    """`aris mark [slots | --group g] [--yaw]`: each row's two pen tips brought together."""
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    q = urllib.parse.urlencode({k: v for k, v in (("slots", ",".join(a.slots)),
                                                  ("group", a.group or ""),
                                                  ("yaw", "true" if a.yaw else "")) if v})
    code, r = http.post("/mark" + (f"?{q}" if q else ""))
    if code != 200:
        return verdict(False, f"refused: {r.get('refused')}: {r.get('detail')}")
    from aris.server.mark import TO_DO
    say(f"job {r['id']}: each pair of neighbouring arms flies above a spot, their tips 100 to 500 mm apart; then")
    for line in TO_DO:
        say(f"  {line}")
    v = follow(http, r["id"], a.poll, show_rows=("instruction",))
    for line in report_lines(v["report"]):
        say(line)
    return verdict(v["state"] == "done", f"mark: {v['state']}")


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
