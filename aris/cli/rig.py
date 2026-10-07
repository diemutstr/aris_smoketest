"""The rig and the server: rig, arms, serve."""
from __future__ import annotations

from aris.cli.common import _assume, assumptions_line, paper_line, say, verdict


def cmd_arms(a, http) -> int:
    """Each slot: its robot, its joints (or why there is no reading), at its park, when."""
    if _assume(http) is None:
        return verdict(False, "the server does not answer")
    code, arms = http.get("/arms")
    if code != 200:
        return verdict(False, f"{arms}")
    say(f"{'slot':<5} {'robot':<12} {'joints (rad)':<58} {'park':<5} reported")
    unknown = []
    for slot, r in arms.items():
        q = r.get("q")
        joints = " ".join(f"{x:+.3f}" for x in q) if q else (r.get("reading") or "no reading")
        if not q:
            unknown.append(slot)
        park = "-" if r.get("at_park") is None else ("yes" if r["at_park"] else "no")
        age = r.get("age_s")
        when = "" if age is None else f"{age:.0f} s ago"
        flags = ", ".join(r.get("flags", [])) if "flags" in r else ""
        say(f"{slot:<5} {str(r.get('robot') or '-'):<12} {joints:<58} {park:<5} "
            f"{when or flags}")
    return verdict(not unknown, "every slot has a reading" if not unknown else
                   f"no reading for {', '.join(unknown)}")


def cmd_rig(a, http) -> int:
    r = _assume(http)
    if r is None:
        return verdict(False, "the server does not answer")
    c = r.get("drawing_area_centre_m") or [0.0, 0.0]
    say(f"drawing area {r['drawing_area_m'][0]:.3f} x {r['drawing_area_m'][1]:.3f} m around "
        f"({c[0]:+.3f}, {c[1]:+.3f}), canvas {r['canvas_m'][0]:.3f} x {r['canvas_m'][1]:.3f} m")
    say(f"pen {(r.get('pen_in') or {}).get('name')}")
    files = r.get("calibration_files", {})
    for aid, arm in r["arms"].items():
        T = arm["T_table_base"]
        f = files.get(aid, {})
        parts = "; ".join(f"{k} {'passed' if f[k]['passed'] else 'FAILED'} {f[k]['date']}"
                          for k in ("base", "pen") if k in f) or "no file"
        say(f"slot {aid:<3} axis ({T[0][3]:+.4f}, {T[1][3]:+.4f}) m  calibration "
            f"{arm['calibration']}  [{parts}]")
    say(f"paper map {paper_line(r.get('paper_surface'))}")
    if r.get("drawing_area_problem"):
        say(f"NO DRAWING: {r['drawing_area_problem']} (park, calibrate and marks still run)")
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
                        sim_truth=getattr(a, "sim_truth", None),
                        sim_base_error=None if not getattr(a, "sim_base_error", None) else
                        tuple(float(x) for x in a.sim_base_error.split(",")),
                        sim_mark_error=getattr(a, "sim_mark_error", None))


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
