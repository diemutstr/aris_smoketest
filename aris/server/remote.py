"""The four endpoints the operator PC uses (robot/aris_robot/remote.py is the client).

    GET  /jobs/{id}/header                    job.json
    GET  /jobs/{id}/phases?offset=B           phases.jsonl from byte B on, held open while it
                                              grows, closed after its end line
    GET  /jobs/{id}/queues/{phase}/{arm}?offset=B
                                              that queue file from byte B on, held open while
                                              it grows, closed after its end marker; 404 until
                                              the file exists
    POST /jobs/{id}/events                    {"source", "rows": [{"seq": n, ...}]}: appended
                                              to the job's log once each, in seq order; answers
                                              {"accepted", "next_seq", "stop"}

The files are sent byte for byte as they are on disk; the operator PC keeps a copy and
resumes from its own length after a lost link.  A stream also ends when its job is finished
here and nothing more will be written (a job that failed before its queues were closed).
"""
from __future__ import annotations

import asyncio
import io
import json
from pathlib import Path

import numpy as np

from fastapi import HTTPException, Request
from fastapi.responses import StreamingResponse

from aris.execute import Job
from aris.execute.queue import FRAME, MAGIC
from aris.server.jobs import accept_rows

POLL = 0.02            # s between looks at a growing file


CHUNK = 1 << 20         # bytes read from disk at a time


def _is_end_record(payload: bytes) -> bool:
    with np.load(io.BytesIO(payload), allow_pickle=False) as z:
        return json.loads(str(z["meta"])).get("type") == "end"


class QueueFrames:
    """Follows the records of a queue file as its bytes are sent, each byte examined once,
    and knows when the end record has gone by.  Only the record now passing is held.

    Resuming at `offset`, the records before it are stepped over by their 16-byte frames
    alone; only the one record that straddles `offset` (or ends exactly there) is read."""

    def __init__(self, path: Path, offset: int):
        self.buf, self.ended = bytearray(), False
        start, last = 0, None
        with open(path, "rb") as f:
            while True:
                f.seek(start)
                head = f.read(FRAME.size)
                if len(head) < FRAME.size:
                    break
                n = FRAME.unpack(head)[1]
                if start + FRAME.size + n > offset:
                    break
                last, start = start, start + FRAME.size + n
            if start < offset:                   # a record straddles the offset
                f.seek(start)
                self.buf += f.read(offset - start)
            elif last is not None and offset > 0:
                f.seek(last)
                rec = f.read(start - last)
                self.ended = _is_end_record(rec[FRAME.size:])

    def feed(self, data: bytes) -> None:
        self.buf += data
        while len(self.buf) >= FRAME.size:
            magic, n, _ = FRAME.unpack_from(self.buf, 0)
            if magic != MAGIC:
                raise ValueError("not a queue record")
            if len(self.buf) < FRAME.size + n:
                return
            if _is_end_record(bytes(self.buf[FRAME.size:FRAME.size + n])):
                self.ended = True
            del self.buf[:FRAME.size + n]


class PhaseLines:
    """The same for the phase list: an end line is only ever the last complete line sent."""

    def __init__(self, path: Path, offset: int):
        self.buf, self.ended = bytearray(), False
        if offset > 0:                           # the line that ends at or runs across offset
            with open(path, "rb") as f:
                f.seek(max(0, offset - 4096))
                head = f.read(offset - max(0, offset - 4096))
            cut = head.rfind(b"\n", 0, len(head) - 1 if head.endswith(b"\n") else len(head))
            self.feed_lines(head[cut + 1:])

    def feed_lines(self, data: bytes) -> None:
        self.buf += data
        end = self.buf.rfind(b"\n")
        if end < 0:
            return
        lines = [x for x in bytes(self.buf[:end]).splitlines() if x.strip()]
        if lines:
            self.ended = bool(json.loads(lines[-1]).get("end"))
        del self.buf[:end + 1]

    feed = feed_lines


async def _tail(path: Path, offset: int, follower, job_over):
    """The file from `offset` on, following it as it grows, until `follower` has seen its
    end go by (or its job is over and nothing more comes)."""
    seen = None
    while True:
        if seen is None and path.exists():
            seen = follower(path, offset)
        data = b""
        if seen is not None:
            if seen.ended:
                return
            with open(path, "rb") as f:
                f.seek(offset)
                data = f.read(CHUNK)
        if data:
            offset += len(data)
            seen.feed(data)
            yield data
            continue
        if job_over():
            return
        await asyncio.sleep(POLL)


def add_routes(app, st, store) -> None:
    def job_of(jid: str) -> Job:
        d = store.root / jid
        if "/" in jid or ".." in jid or not (d / "job.json").exists():
            raise HTTPException(404, f"no job {jid}")
        return Job(d)

    def over(jid: str):
        rec = store.get(jid)
        return (lambda: True) if rec is None else (lambda: rec.finished)

    @app.get("/jobs/{jid}/header")
    def header(jid: str):
        return job_of(jid).header()

    @app.get("/jobs/{jid}/phases")
    def phases(jid: str, offset: int = 0):
        path = job_of(jid).dir / "phases.jsonl"
        return StreamingResponse(_tail(path, offset, PhaseLines, over(jid)),
                                 media_type="application/x-ndjson")

    @app.get("/jobs/{jid}/queues/{phase}/{arm}")
    def queue(jid: str, phase: str, arm: int, offset: int = 0):
        path = job_of(jid).queue(phase, arm).path
        if not path.exists():
            raise HTTPException(404, "not written yet")
        return StreamingResponse(_tail(path, offset, QueueFrames, over(jid)),
                                 media_type="application/octet-stream")

    @app.post("/jobs/{jid}/events")
    async def events(jid: str, request: Request):
        job_of(jid)
        rec = store.get(jid)
        if rec is None:
            raise HTTPException(409, f"job {jid} is not a job of this server run")
        body = await request.json()
        return accept_rows(rec, list(body.get("rows", [])),
                           lambda row: st.positions.from_row(row, jid))
