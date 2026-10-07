"""The operator channel: what the server asks the operator PC to do.

The operator PC runs one resident process (DESIGN.md section 4) that pulls its work from here,
so it needs no open port and the server stays the one front door:

    GET  /operator/next?wait=30    the oldest command not yet acknowledged, or 204 after
                                   `wait` seconds: {"id": n, "command": "run", "job": id,
                                   "kind": ...} | {"id", "command": "recover", "arm": a} |
                                   {"id", "command": "report"}
    POST /operator/ack             {"id": n}: the command was taken; it is not given again
    POST /operator/rows            {"source": "robot", "rows": [...]}: what the operator PC
                                   says outside any job (started, where, report, recovered,
                                   stack died, ...); kept in `operator.jsonl` beside the jobs,
                                   and every `where` / `q` in them updates the arm positions
    GET  /operator                 the commands waiting, when the operator PC was last heard,
                                   its stacks as it last reported them, its last rows

With `--driver robot` every admitted job (drawing, park, calibrate) is a "run" command.  A
command is given again until it is acknowledged, so a runner that died before acknowledging
does not lose it.  Nothing here is persistent: a restarted server has an empty channel.
"""
from __future__ import annotations

import threading
import time


class Channel:
    def __init__(self):
        self._cond = threading.Condition()
        self._commands: list[dict] = []
        self._next_id = 0

    def push(self, command: str, **fields) -> dict:
        with self._cond:
            cmd = dict(id=self._next_id, command=command, queued_at=time.time(), **fields)
            self._next_id += 1
            self._commands.append(cmd)
            self._cond.notify_all()
            return dict(cmd)

    def peek(self) -> dict | None:
        """The oldest command not acknowledged, now."""
        with self._cond:
            return dict(self._commands[0]) if self._commands else None

    def next(self, wait: float) -> dict | None:
        """The oldest command not acknowledged, waiting up to `wait` seconds for one."""
        deadline = time.monotonic() + max(0.0, float(wait))
        with self._cond:
            while not self._commands:
                left = deadline - time.monotonic()
                if left <= 0.0:
                    return None
                self._cond.wait(left)
            return dict(self._commands[0])

    def ack(self, cid: int) -> bool:
        with self._cond:
            for i, c in enumerate(self._commands):
                if c["id"] == cid:
                    del self._commands[i]
                    return True
            return False

    def pending(self) -> list[dict]:
        with self._cond:
            return [dict(c) for c in self._commands]


class OperatorLog:
    """The operator PC's own rows: appended to `operator.jsonl`, the last ones kept here."""

    KEEP = 50

    def __init__(self, path):
        from aris.execute.log import EventLog
        self.log = EventLog(path)
        self._lock = threading.Lock()
        self.last: list[dict] = []
        self.last_seen: float | None = None
        self.stacks = None

    def take(self, rows: list, positions) -> int:
        n = 0
        for r in rows:
            if not isinstance(r, dict) or "event" not in r:
                continue
            fields = {k: v for k, v in r.items() if k != "event"}
            fields.setdefault("source", "robot")
            self.log.write(str(r["event"]), **fields)
            positions.from_row(r, "operator")
            with self._lock:
                self.last = (self.last + [dict(r)])[-self.KEEP:]
                if r.get("stacks") is not None:
                    self.stacks = r["stacks"]
            n += 1
        with self._lock:
            self.last_seen = time.time()
        return n

    def view(self) -> dict:
        with self._lock:
            return dict(last_seen=self.last_seen, stacks=self.stacks, last_rows=list(self.last))


# --------------------------------------------------------------------------- endpoints


def add_routes(app, st, store) -> None:
    import asyncio
    from fastapi import Body, HTTPException
    from fastapi.responses import JSONResponse, Response
    from aris.calib import files as calib_files
    from aris.server import calibrate
    oplog = OperatorLog(store.root / "operator.jsonl")
    app.state.operator_log = oplog

    def refused(code, r):
        return JSONResponse(status_code=code, content=dict(refused=r.reason, detail=r.detail))

    @app.get("/operator/next")
    async def next_command(wait: float = 30.0):
        deadline = time.monotonic() + min(max(wait, 0.0), 300.0)
        while True:
            cmd = st.operator.peek()
            if cmd is not None:
                return cmd
            if time.monotonic() >= deadline:
                return Response(status_code=204)
            await asyncio.sleep(0.05)

    @app.post("/operator/ack")
    def ack(body: dict = Body(...)):
        # (annotations here are strings, so only builtins: a Request would not resolve)
        return dict(acknowledged=st.operator.ack(int(body.get("id", -1))))

    @app.post("/operator/rows")
    def operator_rows(body: dict = Body(...)):
        return dict(accepted=oplog.take(list(body.get("rows", [])), st.positions))

    @app.post("/operator/report")
    def report():
        return st.operator.push("report")

    @app.get("/operator")
    def pending():
        return dict(pending=st.operator.pending(), **oplog.view())

    @app.post("/arms/{arm}/recover")
    def recover(arm: str):
        if arm not in st.rig.arm_ids:
            raise HTTPException(404, f"no arm {arm} on this rig")
        if st.remote:
            return dict(queued=st.operator.push("recover", arm=arm))
        r = st.drivers[arm].recover()
        return dict(recovered=bool(r.done), why=r.why)

    @app.post("/calibrate/{arm}")
    def calibrate_arm(arm: str):
        rec = calibrate.submit_calibrate(st, store, arm)
        if not hasattr(rec, "id"):
            return refused(409, rec)
        return dict(id=rec.id, state=rec.state)

    @app.post("/mark")
    def mark_job(slots: str = "", group: str = ""):
        from aris.server import mark
        rec = mark.submit_mark(st, store, tuple(x for x in slots.split(",") if x),
                               group or None)
        if not hasattr(rec, "id"):
            return refused(409, rec)
        return dict(id=rec.id, state=rec.state)

    @app.post("/touchoff/{arm}")
    def touchoff_arm(arm: str):
        rec = calibrate.submit_calibrate(st, store, arm, kind="touchoff")
        if not hasattr(rec, "id"):
            return refused(409, rec)
        return dict(id=rec.id, state=rec.state)

    @app.get("/calibration")
    def calibration():
        files = [f for f in calib_files.listing(st.config_dir) if f.get("slot")]
        return dict(arms=sorted(f["slot"] for f in files), files=files,
                    status={str(a): st.rig.calibration_status(a) for a in st.rig.arm_ids})

    @app.get("/calibration/{arm}")
    def calibration_file(arm: str):
        f = calib_files.read(st.config_dir, arm)
        if f is None:
            raise HTTPException(404, f"no calibration file for arm {arm}")
        return f
