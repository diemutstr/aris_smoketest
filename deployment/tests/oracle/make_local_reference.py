"""Reference data for tests/test_local.py, made with the OLD code.

Run as its own process (the old package reads its rig and tool at import):

    cd deployment && ARIS_RIG=proposed ARIS_TOOL=lateral ../.venv/bin/python \
        tests/oracle/make_local_reference.py [--workers 30]

Writes
  tests/data/local_corpus.npz     the five bench drawings (aris_sixarm/bench), old canvas frame:
                                  <name>_n strokes, <name>_<i> (M, 2) points
  tests/data/local_reference.npz  the old single-stroke planner on every test line of
                                  tests/local_cases.py, per arm (a<arm>_...):
                                    ids, length        the line
                                    drawn              metres the old planner certifies
                                    seconds            its wall time on the line
                                    status             its first answer: ok / split / degenerate / bug

How the old planner is asked.  `stroke_api.plan_stroke` as scripts/day1.py calls it (lateral
holder, tilt cone 15 degrees, objective as shipped, the arm's real pen depth).  It answers "ok"
(the whole line), or "split" with a certified head [0, s_star]; the rest of the line is not
planned.  So that it gets the same chance as the new planner, which plans every stretch it
can, the rest is offered again from 1 cm past the end of the certified head (or past
s_reach, where the old planner says its redundancy band ends, if that is further), up to 30
calls per line.  The old planner knows the paper and its own mount, not the steel of the
frame and not the parked arms.
"""
import argparse
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path

assert os.environ.get("ARIS_RIG") == "proposed" and os.environ.get("ARIS_TOOL") == "lateral", \
    "run with ARIS_RIG=proposed ARIS_TOOL=lateral"

import numpy as np

HERE = Path(__file__).resolve().parent
DEPLOY = HERE.parents[1]
sys.path.insert(0, str(DEPLOY))
sys.path.insert(0, str(DEPLOY / "tests"))

from aris_sixarm import bench, frames, pwl, stroke_api                   # noqa: E402
from aris_sixarm.fleet import FLEET                                      # noqa: E402

TILT_MAX_DEG = 15.0
MAX_CALLS = 30
SKIP = 0.01                    # m past a refused head before the rest is offered again


def write_corpus(path: Path) -> None:
    out = {}
    for name in bench.ORDER:
        strokes, _ = bench.make(name)
        out[f"{name}_n"] = np.array(len(strokes))
        for i, s in enumerate(strokes):
            out[f"{name}_{i}"] = np.asarray(s["pts"], float)
    np.savez_compressed(path, **out)


def _length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def old_drawn(job):
    """-> (metres certified, seconds, first status) for one line (old canvas xy)."""
    arm, xy = job
    spec = FLEET[arm]
    opts = dict(objective=pwl.OBJECTIVE, tilt_max_deg=TILT_MAX_DEG, pen_ext=frames.ext_of(None))
    L = _length(xy)
    pos, drawn, first = 0.0, 0.0, None
    t0 = time.perf_counter()
    for _ in range(MAX_CALLS):
        if L - pos < 1e-9:
            break
        sub = stroke_api.truncate_polyline(xy, pos / L, 1.0)
        L_sub = _length(sub)
        if L_sub < stroke_api.DEFAULTS["min_length"]:
            break
        r = stroke_api.plan_stroke(sub, spec, opts)
        status = r.get("status")
        first = first or status
        if status == "ok":
            drawn += L_sub
            break
        if status != "split":
            break
        s_star = float(r.get("s_star") or 0.0)
        s_reach = float(r.get("s_reach") or 0.0)
        drawn += s_star * L_sub
        pos += max(s_star, s_reach) * L_sub + SKIP
    return drawn, time.perf_counter() - t0, first or "degenerate"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=30)
    ap.add_argument("--arms", default="31,13")
    a = ap.parse_args()
    data = DEPLOY / "tests" / "data"
    write_corpus(data / "local_corpus.npz")

    from aris.rig import Rig
    import local_cases
    rig = Rig.load(DEPLOY / "config")
    ref = {}
    for arm in (int(x) for x in a.arms.split(",")):
        lines = [x for v in local_cases.cases(rig, arm).values() for x in v]
        jobs = [(arm, np.asarray(x.points, float)[:, :2] - local_cases.OLD_TO_TABLE)
                for x in lines]
        t = time.perf_counter()
        with Pool(a.workers) as pool:
            res = pool.map(old_drawn, jobs, chunksize=1)
        print(f"arm {arm}: {len(lines)} lines in {time.perf_counter() - t:.0f} s")
        ref[f"a{arm}_ids"] = np.array([x.id for x in lines])
        ref[f"a{arm}_length"] = np.array([_length(j[1]) for j in jobs])
        ref[f"a{arm}_drawn"] = np.array([r[0] for r in res])
        ref[f"a{arm}_seconds"] = np.array([r[1] for r in res])
        ref[f"a{arm}_status"] = np.array([r[2] for r in res])
    np.savez_compressed(data / "local_reference.npz", **ref)


if __name__ == "__main__":
    main()
