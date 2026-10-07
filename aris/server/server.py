"""The drawing server: the one front door.  A FastAPI app over one Station.

    POST /jobs              a drawing (the JSON file as the request body) -> the job id;
                            ?note=... lands in the header and report; ?rest_of=<id> draws
                            what that finished job left over (no body)
    GET  /jobs              every job of this server run
    GET  /jobs/{id}         state, fitted drawing, per phase and arm progress, the report
    GET  /jobs/{id}/events  the job's event log
    POST /jobs/{id}/stop    every arm stops and holds; the job ends as stopped
    POST /park              park all arms (a job like any other)
    GET  /rig               arms, parks, drawing area, calibration, digests, driver, speed
    GET  /arms              each arm's configuration and its driver's state
    and the operator PC's four (remote.py): header, phases, queues, events
    POST /calibrate/{arm}, the operator channel, recover, calibration files (operator.py)
    POST /grip/{slot}, POST/GET /drawings, GET /jobs/{id}/report, GET /gui (and /)
    The full table: aris/server/API.md.

One job at a time: a second job while one runs is refused (409).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from fastapi import FastAPI, Form, HTTPException, Query, Request, UploadFile
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse

from aris.calib import files as calib_files
from aris.server import paper as paper_mod
from aris.server import drawing, drawings, grip, operator, remote, runner
from aris.server import park as park_job
from aris.server.jobs import JobStore, view
from aris.types import Refusal


def plain(o):
    """Anything -> what JSON can carry (inf and nan as strings)."""
    if isinstance(o, dict):
        return {str(k): plain(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [plain(v) for v in o]
    if isinstance(o, np.ndarray):
        return plain(o.tolist())
    if isinstance(o, np.generic):
        return plain(o.item())
    if isinstance(o, float) and not math.isfinite(o):
        return str(o)
    return o


def _refused(code: int, r: Refusal) -> JSONResponse:
    return JSONResponse(status_code=code, content=dict(refused=r.reason, detail=r.detail))


def rig_view(st) -> dict:
    rig = st.rig
    return plain(dict(
        arms={str(a): dict(T_table_base=rig.T_table_base(a), park_q=rig.park_q(a),
                           calibration=rig.calibration_status(a), robot=st.robots.get(a))
              for a in rig.arm_ids},
        mark_groups={str(k): list(v) for k, v in rig.mark_groups.items()},
        drawing_area_m=list(st.drawing_area), drawing_area_centre_m=list(st.drawing_centre),
        canvas_m=rig.canvas_size, pen_in=st.pen(),
        calibration_files={f["slot"]: f for f in calib_files.listing(st.config_dir)
                           if f.get("slot")},                  # the slots' files, not paper.json
        drawing_area_from_maps_m=list(st.maps_area),
        paper_surface=paper_mod.describe(st.surface),
        drawing_area_problem=st.area_problem or None, code=st.code,
        **st.assumptions()))


def arms_view(st) -> dict:
    if st.remote:                      # what the operator PC last said, per controlled slot
        import time
        seen, out, now = st.positions.all(), {}, time.time()
        for a in st.rig.arm_ids:
            p = seen.get(a)
            q, why = st.positions.known(a, now)
            out[str(a)] = dict(
                robot=(p or {}).get("robot") or st.robots.get(a), q=q,
                reading="fresh" if q is not None else (why or "never reported"),
                at_park=None if q is None else bool(
                    np.max(np.abs(q - st.rig.park_q(a))) <= st.rig.execution().start_tolerance),
                reported_at=None if p is None else p["reported_at"],
                age_s=None if p is None else now - p["received_at"],
                source="operator PC", job=None if p is None else p["job"])
        return plain(out)
    out = {}
    for a, d in st.drivers.items():
        s = d.state()
        out[str(a)] = dict(robot=st.robots.get(a), q=s.q, qd=s.qd, ok=s.ok, flags=list(s.flags),
                           at_park=bool(st.rig.at_park(a, s.q)))
    return plain(out)


def create_app(st) -> FastAPI:
    store = JobStore(st.jobs_dir)
    app = FastAPI(title="aris drawing server")
    app.state.station, app.state.store = st, store

    def record(jid: str):
        rec = store.get(jid)
        if rec is None:
            raise HTTPException(404, f"no job {jid}")
        return rec

    @app.post("/jobs")
    async def submit(request: Request, name: str = "drawing.json", note: str = "",
                     rest_of: str = "", air_mm: float = 0.0, drawing_id: str = Query("",
                                                                         alias="drawing")):
        if rest_of:                      # the leftovers of a finished job, as a new drawing
            rec = runner.submit_rest(st, store, rest_of, note, air_mm)
            if isinstance(rec, Refusal):
                return _refused(409, rec)
            return plain(dict(id=rec.id, state=rec.state, why=rec.why))
        if drawing_id:                   # an uploaded drawing (POST /drawings)
            body = drawings.read(st, drawing_id)
            if isinstance(body, Refusal):
                return _refused(404, body)
            name = name if name != "drawing.json" else drawing_id
        else:
            body = await request.body()
        lines = drawing.parse(body)
        if isinstance(lines, Refusal):
            return _refused(400, lines)
        rec = runner.submit_draw(st, store, lines, name, note, air_mm=air_mm)
        if isinstance(rec, Refusal):
            return _refused(409, rec)
        return plain(dict(id=rec.id, state=rec.state, why=rec.why))

    @app.get("/jobs")
    def jobs():
        return plain(store.listing())

    @app.get("/jobs/{jid}")
    def job(jid: str):
        rec = store.get(jid)
        if rec is None:
            old = store.from_disk(jid)
            if old is None:
                raise HTTPException(404, f"no job {jid}")
            return plain(dict(id=jid, state=old.get("state"), report=old))
        return plain(view(rec))

    @app.get("/jobs/{jid}/report")
    def job_report(jid: str):
        rec = store.get(jid)
        rep = rec.report if rec is not None else store.from_disk(jid)
        if rep is None:
            raise HTTPException(404, f"no report for job {jid} (unknown, or not finished)")
        return plain(rep)

    @app.post("/grip/{slot}")
    async def grip_slot(slot: str, request: Request):
        try:
            body = json.loads(await request.body() or b"{}")
        except ValueError as e:
            return _refused(400, Refusal("not_json", str(e)))
        if not isinstance(body, dict):
            return _refused(400, Refusal("not_json", "the body is a JSON object"))
        rec = grip.submit_grip(st, store, slot, str(body.get("verb", "")), body)
        if isinstance(rec, Refusal):
            return _refused(409, rec)
        return plain(dict(id=rec.id, state=rec.state))

    @app.post("/drawings")
    async def upload(file: UploadFile, width: float | None = Form(None),
                     at: str | None = Form(None)):
        xy = None if not at else tuple(float(v) for v in at.replace(" ", ",").split(",") if v)
        got = drawings.store(st, file.filename or "drawing.json", await file.read(), width, xy)
        if isinstance(got, Refusal):
            return _refused(400, got)
        return plain(got)

    @app.get("/drawings")
    def stored_drawings():
        return plain(drawings.listing(st))

    @app.get("/")
    def root():
        return RedirectResponse("/gui/")

    gui = Path(__file__).resolve().parent / "gui"

    @app.get("/gui")
    def gui_root():
        return RedirectResponse("/gui/")

    @app.get("/gui/{path:path}")
    def gui_file(path: str = ""):
        f = (gui / (path or "index.html")).resolve()
        if gui.resolve() not in f.parents or not f.is_file():
            raise HTTPException(404, f"no GUI file {path or 'index.html'}")
        return FileResponse(f)

    @app.get("/jobs/{jid}/events")
    def events(jid: str):
        rec = record(jid)
        return plain(rec.log.read() if rec.dir.exists() else [])

    @app.post("/jobs/{jid}/stop")
    def stop(jid: str):
        rec = record(jid)
        r = runner.stop(st, rec)
        if isinstance(r, Refusal):
            return _refused(409, r)
        return dict(id=jid, stopping=True)

    @app.post("/park")
    def park():
        rec = park_job.submit_park(st, store)
        if isinstance(rec, Refusal):
            return _refused(409, rec)
        return plain(dict(id=rec.id, state=rec.state))

    @app.get("/rig")
    def rig():
        return rig_view(st)

    @app.get("/arms")
    def arms():
        same, line = runner.code_line(st)
        return dict(arms=arms_view(st),
                    code=dict(same=same, line=line, server=st.code,
                              operator_pc=st.operator.code if st.remote else None,
                              operator_pc_reported_at=st.operator.code_at))

    remote.add_routes(app, st, store)
    operator.add_routes(app, st, store)
    return app


def serve(st, host: str = "127.0.0.1", port: int = 8420) -> None:
    import uvicorn
    uvicorn.run(create_app(st), host=host, port=port, log_level="warning")


__all__ = ["create_app", "serve", "plain"]
