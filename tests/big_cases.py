"""Very big drawings for measuring the drawing server, made deterministically.

    python tests/big_cases.py          writes tests/data/server_big.json and server_long.json

big:  10 000 lines inside the drawing area, lengths 2 cm to 1 m (skewed toward short ones),
      about 2 km of ink; half straight, half circular arcs.
long: 2 000 lines of 1.2 to 1.5 m each, the same mix.

Both lie inside the 1.56 x 3.56 m drawing area with 1 cm to spare, so the server does not
scale them.  Points every 1 cm along arcs; straight lines are two points.  Millimetres, to
0.001 mm.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

AREA = np.array([1.56, 3.56])
SPARE = 0.01
DATA = Path(__file__).resolve().parent / "data"


def _one(rng, length: float, arc: bool) -> np.ndarray | None:
    half = 0.5 * AREA - SPARE
    p0 = rng.uniform(-half, half)
    heading = rng.uniform(0.0, 2 * np.pi)
    if not arc:
        pts = np.array([p0, p0 + length * np.array([np.cos(heading), np.sin(heading)])])
    else:
        # curvature up to one full turn over the length, never tighter than 5 cm radius
        k = rng.uniform(-1.0, 1.0) * min(2 * np.pi / length, 20.0)
        n = max(2, int(np.ceil(length / 0.01)) + 1)
        s = np.linspace(0.0, length, n)
        th = heading + k * s
        if abs(k) < 1e-9:
            xy = np.column_stack([np.cos(heading) * s, np.sin(heading) * s])
        else:
            xy = np.column_stack([(np.sin(th) - np.sin(heading)) / k,
                                  (np.cos(heading) - np.cos(th)) / k])
        pts = p0 + xy
    return pts if np.all(np.abs(pts) <= half) else None


def make(n: int, lengths, seed: int, name: str) -> dict:
    rng = np.random.default_rng(seed)
    lines = []
    for i in range(n):
        while True:
            pts = _one(rng, float(lengths(rng)), arc=bool(i % 2))
            if pts is not None:
                break
        lines.append(dict(id=f"{name}:{i}", points=np.round(pts * 1e3, 3).tolist()))
    return dict(units="mm", frame="table", lines=lines)


def big() -> dict:
    # log-uniform from 2 cm to 1 m, skewed so that the mean is about 0.2 m
    return make(10_000, lambda r: 0.02 * 50.0 ** (r.uniform() ** 1.2), 11, "big")


def long() -> dict:
    return make(2_000, lambda r: r.uniform(1.2, 1.5), 12, "long")


def ink_m(d: dict) -> float:
    return float(sum(np.sum(np.linalg.norm(np.diff(np.asarray(x["points"]), axis=0), axis=1))
                     for x in d["lines"]) * 1e-3)


if __name__ == "__main__":
    for name, fn in (("server_big.json", big), ("server_long.json", long)):
        d = fn()
        (DATA / name).write_text(json.dumps(d, separators=(",", ":")))
        print(f"{name}: {len(d['lines'])} lines, {ink_m(d):.0f} m of ink, "
              f"{(DATA / name).stat().st_size / 1e6:.1f} MB")
