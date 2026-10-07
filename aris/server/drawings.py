"""Uploaded drawings: stored under `out/drawings/` (beside the jobs directory), each as the
drawing JSON `<id>.json` (an SVG converted on upload, `svg.to_drawing`) and the file as sent.
The id is the file's name without its suffix, made safe, plus 8 hex digits of its content's
digest, so the same file uploaded twice is one drawing.  `POST /jobs?drawing=<id>` draws it.
"""
from __future__ import annotations

import hashlib
import json
import re
import time
from pathlib import Path

from aris.server import drawing, svg
from aris.types import Refusal

SAFE = re.compile(r"[^A-Za-z0-9_.-]+")


def folder(st) -> Path:
    return Path(st.jobs_dir).parent / "drawings"


def store(st, name: str, data: bytes, width_m: float | None = None,
          at=None) -> dict | Refusal:
    """A .json or .svg upload -> {id, name, lines, points, ...}; the drawing checked first."""
    suffix = Path(name).suffix.lower()
    if suffix not in (".json", ".svg"):
        return Refusal("format", f"{name}: a drawing is a .json or .svg file")
    stem = SAFE.sub("_", Path(name).stem)[:60] or "drawing"
    did = f"{stem}-{hashlib.blake2b(data, digest_size=4).hexdigest()}"
    d = folder(st)
    d.mkdir(parents=True, exist_ok=True)
    original = d / f"{did}.original{suffix}"
    original.write_bytes(data)
    if suffix == ".svg":
        if width_m is None:
            original.unlink()
            return Refusal("width", "an SVG needs its width on the table, in metres")
        got = svg.to_drawing(original, width_m,
                             st.drawing_centre if at is None else at)
        if isinstance(got, Refusal):
            original.unlink()
            return got
        body = json.dumps(got).encode()
    else:
        body = data
    lines = drawing.parse(body)
    if isinstance(lines, Refusal):
        original.unlink()
        return lines
    (d / f"{did}.json").write_bytes(body)
    meta = dict(id=did, name=name, kind=suffix[1:], stored_at=time.time(), lines=len(lines),
                points=sum(len(x.points) for x in lines), bbox_m=drawing.bbox(lines),
                width_m=width_m, at_m=None if at is None else [float(x) for x in at])
    (d / f"{did}.meta.json").write_text(json.dumps(meta))
    return meta


def listing(st) -> list[dict]:
    d = folder(st)
    if not d.exists():
        return []
    out = [json.loads(p.read_text()) for p in d.glob("*.meta.json")]
    return sorted(out, key=lambda m: m.get("stored_at", 0.0))


def read(st, did: str) -> bytes | Refusal:
    """The stored drawing JSON of id `did`."""
    p = folder(st) / f"{did}.json"
    if SAFE.search(did) or "/" in did or not p.exists():
        return Refusal("no_drawing", f"no stored drawing {did!r}")
    return p.read_bytes()
