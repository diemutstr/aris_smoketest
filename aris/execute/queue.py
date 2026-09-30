"""The queue: what was planned and checked for one arm in one phase, and what the executor runs.

A job is one directory:

    job.json                     the header: rig, calibration and drawing digests, rules, time
    phases.jsonl                 the phases in the order they run, one per line, then {"end": ..}
    <phase>__arm<id>.queue       one per (phase, arm), appended to as motions arrive
    events.jsonl                 what every executor and the coordinator did (log.py)

A queue file is a sequence of records.  A record is a 16-byte frame (b"ARQ1", payload length
as uint64, CRC-32 of the payload as uint32, little endian) followed by the payload, which is an
uncompressed `.npz` (numpy's own format): a JSON string `meta` and the motion's arrays, saved
bit for bit.  The first record names the phase and the arm, the last one is the end marker.
The writer appends each record with one write and flushes it to disk, so a reader that finds
fewer bytes than the frame announces has met a motion still being written and stops there:
it only ever sees complete motions.
"""
from __future__ import annotations

import hashlib
import io
import json
import os
import re
import struct
import time
import zlib
from dataclasses import dataclass, fields, is_dataclass
from pathlib import Path

import numpy as np

from aris.types import Motion, Phase, Piece, Refusal, Trajectory, Wall

MAGIC = b"ARQ1"
FRAME = struct.Struct("<4sQI")
FORMAT = 1
CONTINUITY = 1e-9       # rad, a motion starts exactly where the previous one ended


@dataclass(frozen=True)
class Entry:
    """One queued motion with the checker's key numbers."""
    index: int
    motion: Motion
    verdict: dict       # passed, tightest, tightest_value, tightest_limit, min_clearance, ...


@dataclass(frozen=True)
class End:
    """The end marker.  `complete` False: the queue was cut short (the note says why) and the
    arm is not where the plan for the next phase assumes it."""
    count: int
    complete: bool = True
    note: str = ""


# --------------------------------------------------------------------------- digests


def digest(*objs) -> str:
    """A content digest of plain data, numpy arrays and dataclasses (e.g. the Rig, the lines)."""
    h = hashlib.blake2b(digest_size=12)
    for o in objs:
        _feed(h, o)
    return h.hexdigest()


def _feed(h, o) -> None:
    if isinstance(o, np.ndarray):
        a = np.ascontiguousarray(o)
        h.update(f"A{a.dtype.str}{a.shape}".encode())
        h.update(a.tobytes())
    elif is_dataclass(o):
        h.update(f"D{type(o).__name__}".encode())
        for f in fields(o):
            h.update(f.name.encode())
            _feed(h, getattr(o, f.name))
    elif isinstance(o, dict):
        h.update(f"M{len(o)}".encode())
        for k in sorted(o, key=str):
            _feed(h, str(k))
            _feed(h, o[k])
    elif isinstance(o, (list, tuple)):
        h.update(f"L{len(o)}".encode())
        for x in o:
            _feed(h, x)
    elif isinstance(o, (bool, int, float, str, type(None), np.generic)):
        h.update(f"S{type(o).__name__}:{o!r}".encode())
    else:
        raise TypeError(f"cannot digest a {type(o).__name__}")


def header_for(rig, lines, rules) -> dict:
    """The job header for a drawing: what the queues were planned against."""
    calib = {a: (m.T_table_base, m.tip_hand, m.calibration) for a, m in rig.mounts.items()}
    return dict(rig_digest=digest(rig), calibration_digest=digest(calib),
                drawing_digest=digest(list(lines)), rules=_plain(rules))


def _plain(o):
    if is_dataclass(o):
        return {f.name: _plain(getattr(o, f.name)) for f in fields(o)}
    if isinstance(o, np.generic):
        return o.item()
    return o


# --------------------------------------------------------------------------- records


def _record(meta: dict, arrays: dict) -> bytes:
    buf = io.BytesIO()
    np.savez(buf, meta=np.array(json.dumps(meta, sort_keys=True)), **arrays)
    payload = buf.getvalue()
    return FRAME.pack(MAGIC, len(payload), zlib.crc32(payload)) + payload


def _records(data: bytes, offset: int):
    """(meta, arrays, next offset) for every complete record from `offset` on."""
    while len(data) - offset >= FRAME.size:
        magic, n, crc = FRAME.unpack_from(data, offset)
        if magic != MAGIC:
            raise ValueError(f"not a queue record at byte {offset}")
        start = offset + FRAME.size
        if len(data) - start < n:
            return                                  # still being written
        payload = data[start:start + n]
        if zlib.crc32(payload) != crc:
            raise ValueError(f"damaged queue record at byte {offset}")
        with np.load(io.BytesIO(payload), allow_pickle=False) as z:
            arrays = {k: z[k] for k in z.files}
        offset = start + n
        yield json.loads(str(arrays.pop("meta"))), arrays, offset


def _motion_record(index: int, m: Motion, verdict: dict) -> bytes:
    meta = dict(type="motion", index=index, kind=m.kind, intensity=float(m.intensity),
                verdict=verdict,
                piece=None if m.piece is None else [m.piece.line_id, m.piece.s0, m.piece.s1])
    arrays = dict(t=m.traj.t, q=m.traj.q, qd=m.traj.qd)
    if m.tip_base is not None:
        arrays["tip_base"] = m.tip_base
    return _record(meta, arrays)


def _entry(meta: dict, a: dict) -> Entry:
    p = meta["piece"]
    m = Motion(meta["kind"], Trajectory(a["t"], a["q"], a["qd"]),
               None if p is None else Piece(p[0], float(p[1]), float(p[2])),
               a.get("tip_base"), meta["intensity"], checked=meta["verdict"])
    return Entry(meta["index"], m, meta["verdict"])


def verdict_numbers(v) -> dict:
    """The checker's key numbers, kept with the motion (`aris.check.Verdict`).  Plain data:
    this is also what `Motion.checked` carries."""
    out = dict(passed=bool(v.passed), tightest=str(v.tightest),
               min_clearance=float(v.min_clearance), min_clearance_at=str(v.min_clearance_at))
    try:
        t = v.get(v.tightest)
        out.update(tightest_value=float(t.value), tightest_limit=float(t.limit),
                   tightest_used=float(t.used))
    except KeyError:
        pass
    return out


# --------------------------------------------------------------------------- the queue


class Queue:
    """One arm, one phase.  Append-only; one writer, any number of readers."""

    def __init__(self, path, phase: str = "", arm_id: int = -1):
        self.path = Path(path)
        self.phase, self.arm_id = phase, arm_id
        self._count, self._last_q, self._closed = None, None, False

    # ---- writing

    def _open_for_writing(self) -> None:
        if self._count is not None:
            return
        entries, end = self._scan() if self.path.exists() else ([], None)
        if not self.path.exists():
            self._write(_record(dict(type="head", format=FORMAT, phase=self.phase,
                                     arm=self.arm_id), {}))
        self._count, self._closed = len(entries), end is not None
        self._last_q = entries[-1].motion.q_end if entries else None

    def _write(self, rec: bytes) -> None:
        with open(self.path, "ab") as f:
            f.write(rec)
            f.flush()
            os.fsync(f.fileno())

    def append(self, motion: Motion, verdict=None) -> int | Refusal:
        """Queue a motion the checker passed.  Returns its index, or a Refusal.  `verdict`: the
        checker's Verdict; None takes the checker's word the motion carries (`Motion.checked`,
        which the planners fill when they plan with `verify`).  Nothing unchecked is queued."""
        self._open_for_writing()
        if self._closed:
            return Refusal("closed", f"{self.path.name} has its end marker")
        if verdict is None:
            numbers = motion.checked
            if numbers is None:
                return Refusal("unchecked", "the motion carries no checker verdict")
        elif not getattr(verdict, "passed", False):
            return Refusal("failed_check", f"the checker did not pass it: {verdict.tightest}")
        else:
            numbers = verdict_numbers(verdict)
        if not numbers.get("passed", False):
            return Refusal("failed_check",
                           f"the checker did not pass it: {numbers.get('tightest', '')}")
        if self._last_q is not None:
            gap = float(np.max(np.abs(motion.q_start - self._last_q)))
            if gap > CONTINUITY:
                return Refusal("not_continuous",
                               f"starts {gap:.3g} rad from where motion {self._count - 1} ended")
        self._write(_motion_record(self._count, motion, numbers))
        self._count += 1
        self._last_q = motion.q_end
        return self._count - 1

    def close(self, complete: bool = True, note: str = "") -> None:
        """Write the end marker.  `complete` False when the plan for this arm was cut short."""
        self._open_for_writing()
        if not self._closed:
            self._write(_record(dict(type="end", count=self._count, complete=complete,
                                     note=note), {}))
            self._closed = True

    # ---- reading

    def _scan(self, offset: int = 0):
        data = self.path.read_bytes() if self.path.exists() else b""
        entries, end = [], None
        for meta, arrays, _ in _records(data, offset):
            if meta["type"] == "motion":
                entries.append(_entry(meta, arrays))
            elif meta["type"] == "end":
                end = End(meta["count"], meta["complete"], meta["note"])
        return entries, end

    def read(self) -> list[Entry]:
        """Every complete motion written so far."""
        return self._scan()[0]

    def end(self) -> End | None:
        """The end marker, if written."""
        return self._scan()[1]

    def watch(self, poll: float = 0.01, stop=None):
        """Yields each Entry as it appears, then the End.  Waits for the file if it does not
        exist yet.  `stop`: an object with `is_set()` (a threading.Event); when set, the watch
        ends without an End."""
        cursor = self.cursor()
        while stop is None or not stop.is_set():
            for item in cursor.poll():
                yield item
                if isinstance(item, End):
                    return
            _pause(stop, poll)

    def cursor(self) -> "Cursor":
        return Cursor(self.path)


class Cursor:
    """Reads a queue from the start; each `poll()` returns what appeared since the last one."""

    def __init__(self, path):
        self.path, self.offset, self.ended = Path(path), 0, False

    def poll(self) -> list:
        if self.ended or not self.path.exists():
            return []
        with open(self.path, "rb") as f:
            f.seek(self.offset)
            chunk = f.read()
        out, used = [], 0
        for meta, arrays, used in _records(chunk, 0):
            if meta["type"] == "motion":
                out.append(_entry(meta, arrays))
            elif meta["type"] == "end":
                out.append(End(meta["count"], meta["complete"], meta["note"]))
                self.ended = True
                break
        self.offset += used
        return out


# --------------------------------------------------------------------------- the job


def slug(name: str) -> str:
    """A phase name as it appears in file names ("phase 1" -> "phase_1")."""
    return re.sub(r"[^A-Za-z0-9]+", "_", name).strip("_")


_slug = slug


def _phase_json(p: Phase) -> dict:
    return dict(name=p.name, active=list(p.active), parked=list(p.parked),
                walls=[dict(name=w.name, arms=list(w.arms), point=w.point_table.tolist(),
                            normal=w.normal_table.tolist()) for w in p.walls])


def _phase_of(d: dict) -> Phase:
    walls = tuple(Wall(w["name"], tuple(w["arms"]), np.array(w["point"]), np.array(w["normal"]))
                  for w in d["walls"])
    return Phase(d["name"], tuple(d["active"]), tuple(d["parked"]), walls)


class Job:
    """One directory: the header, the phase list, a queue per (phase, arm), the event log."""

    def __init__(self, directory):
        self.dir = Path(directory)

    @staticmethod
    def create(directory, header: dict) -> "Job":
        d = Path(directory)
        d.mkdir(parents=True, exist_ok=False)
        head = dict(format=FORMAT, created=time.strftime("%Y-%m-%dT%H:%M:%S%z"), **header)
        tmp = d / "job.json.tmp"
        tmp.write_text(json.dumps(head, indent=1, sort_keys=True, default=_plain))
        os.replace(tmp, d / "job.json")
        return Job(d)

    def header(self) -> dict:
        return json.loads((self.dir / "job.json").read_text())

    def queue(self, phase: str, arm_id: int) -> Queue:
        return Queue(self.dir / f"{_slug(phase)}__arm{arm_id}.queue", phase, arm_id)

    @property
    def log_path(self) -> Path:
        return self.dir / "events.jsonl"

    # ---- the phase list: appended by the writer as phases begin, read by the coordinator

    def add_phase(self, phase: Phase) -> None:
        _append_line(self.dir / "phases.jsonl", _phase_json(phase))

    def end_phases(self, note: str = "") -> None:
        _append_line(self.dir / "phases.jsonl", dict(end=True, note=note))

    def watch_phases(self, poll: float = 0.01, stop=None):
        """Yields each Phase as it is added; returns at the end line (or when `stop` is set)."""
        path, seen = self.dir / "phases.jsonl", 0
        while stop is None or not stop.is_set():
            rows = _complete_lines(path)
            for d in rows[seen:]:
                seen += 1
                if d.get("end"):
                    return
                yield _phase_of(d)
            _pause(stop, poll)


def _pause(stop, poll: float) -> None:
    if stop is not None:
        stop.wait(poll)
    else:
        time.sleep(poll)


def _append_line(path: Path, d: dict) -> None:
    fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        os.write(fd, (json.dumps(d, sort_keys=True) + "\n").encode())
        os.fsync(fd)
    finally:
        os.close(fd)


def _complete_lines(path: Path) -> list[dict]:
    """The lines of a JSON-lines file that are complete (end in a newline)."""
    if not path.exists():
        return []
    text = path.read_text()
    return [json.loads(x) for x in text[:text.rfind("\n") + 1].splitlines() if x]
