"""The drawing server: the one front door.  A FastAPI app over one Station.

    POST /jobs              a drawing (the JSON file as the request body) -> the job id
    GET  /jobs              every job of this server run
    GET  /jobs/{id}         state, fitted drawing, per phase and arm progress, the report
    GET  /jobs/{id}/events  the job's event log
    POST /jobs/{id}/stop    every arm stops and holds; the job ends as stopped
    POST /park              park all arms (a job like any other)
    GET  /rig               arms, parks, drawing area, calibration, digests, driver, speed
    GET  /arms              each arm's configuration and its driver's state
    and the operator PC's four (remote.py): header, phases, queues, events

One job at a time: a second job while one runs is refused (409).
"""
from __future__ import annotations

import math

import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse

from aris.server import drawing, remote, runner
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
                           calibration=rig.calibration_status(a)) for a in rig.arm_ids},
        drawing_area_m=list(st.drawing_area), canvas_m=rig.canvas_size,
        drawing_area_from_maps_m=list(st.maps_area),
        **st.assumptions()))


def arms_view(st) -> dict:
    if st.remote:                      # what the operator PC last said
        return plain({str(a): dict(p, at_park=bool(np.max(np.abs(p["q"] - st.rig.park_q(a)))
                                                     <= st.rig.execution().start_tolerance))
                      for a, p in sorted(st.positions.all().items())})
    out = {}
    for a, d in st.drivers.items():
        s = d.state()
        out[str(a)] = dict(q=s.q, qd=s.qd, ok=s.ok, flags=list(s.flags),
                           at_park=bool(np.max(np.abs(s.q - st.rig.park_q(a))) <= 1e-6))
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
    async def submit(request: Request, name: str = "drawing.json"):
        lines = drawing.parse(await request.body())
        if isinstance(lines, Refusal):
            return _refused(400, lines)
        rec = runner.submit_draw(st, store, lines, name)
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
        rec = runner.submit_park(st, store)
        if isinstance(rec, Refusal):
            return _refused(409, rec)
        return plain(dict(id=rec.id, state=rec.state))

    @app.get("/rig")
    def rig():
        return rig_view(st)

    @app.get("/arms")
    def arms():
        return arms_view(st)

    remote.add_routes(app, st, store)
    return app


def serve(st, host: str = "127.0.0.1", port: int = 8420) -> None:
    import uvicorn
    uvicorn.run(create_app(st), host=host, port=port, log_level="warning")


__all__ = ["create_app", "serve", "plain"]
