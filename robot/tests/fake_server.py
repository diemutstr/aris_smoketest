"""A stand-in for the drawing server's job endpoints, serving job directories on disk.

It is also the reference for the server's owner: the four endpoints the operator PC uses
(see aris_robot/remote.py), implemented as the runner expects them.
"""
from __future__ import annotations

import asyncio
import json
import socket
import threading
import time
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse

from aris.execute import Job
from aris.execute.queue import Queue


def _slug_path(job: Job, phase: str, arm: int) -> Path:
    return job.queue(phase, arm).path


async def _tail(path: Path, offset: int, finished, poll: float = 0.01, max_bytes=None):
    """The file from `offset` on, following it as it grows, until `finished(path)` and
    everything has been sent.  `max_bytes`: hang up after that many (a flaky link)."""
    while True:
        if path.exists():
            with open(path, "rb") as f:
                f.seek(offset)
                data = f.read(max_bytes if max_bytes else -1)
            if data:
                offset += len(data)
                yield data
                if max_bytes:
                    return
            elif finished(path):
                return
        await asyncio.sleep(poll)


def _phases_finished(path: Path) -> bool:
    text = path.read_text()
    return any(json.loads(x).get("end") for x in text[:text.rfind("\n") + 1].splitlines() if x)


def create_app(jobs_dir, max_bytes=None) -> FastAPI:
    jobs_dir = Path(jobs_dir)
    app = FastAPI()
    app.state.stop = set()          # job ids stopped on the server
    app.state.received = {}         # job id -> rows posted by the robot, in order

    def job(jid: str) -> Job:
        d = jobs_dir / jid
        if not (d / "job.json").exists():
            raise HTTPException(404, f"no job {jid}")
        return Job(d)

    @app.get("/jobs/{jid}/header")
    def header(jid: str):
        return job(jid).header()

    @app.get("/jobs/{jid}/phases")
    def phases(jid: str, offset: int = 0):
        path = job(jid).dir / "phases.jsonl"
        return StreamingResponse(_tail(path, offset, _phases_finished),
                                 media_type="application/x-ndjson")

    @app.get("/jobs/{jid}/queues/{phase}/{arm}")
    def queue(jid: str, phase: str, arm: int, offset: int = 0):
        path = _slug_path(job(jid), phase, arm)
        if not path.exists():
            raise HTTPException(404, "not written yet")
        return StreamingResponse(_tail(path, offset, lambda p: Queue(p).end() is not None,
                                       max_bytes=max_bytes),
                                 media_type="application/octet-stream")

    @app.post("/jobs/{jid}/events")
    async def events(jid: str, request: Request):
        job(jid)
        body = await request.json()
        rows = app.state.received.setdefault(jid, [])
        taken = 0
        for r in body["rows"]:
            if r["seq"] == len(rows):           # in order, each once
                rows.append(r)
                taken += 1
        return dict(accepted=taken, next_seq=len(rows), stop=jid in app.state.stop)

    return app


class Served:
    """The app on a free local port, in a background thread."""

    def __init__(self, app):
        import uvicorn
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            self.port = s.getsockname()[1]
        self.server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=self.port,
                                                    log_level="warning"))
        self.thread = threading.Thread(target=self.server.run, daemon=True)

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def __enter__(self):
        self.thread.start()
        t_end = time.monotonic() + 10
        while not self.server.started and time.monotonic() < t_end:
            time.sleep(0.01)
        return self

    def __exit__(self, *exc):
        self.server.should_exit = True
        self.server.force_exit = True        # do not wait for streams that never end
        self.thread.join(timeout=0.5)        # a daemon; a held-open stream may linger
