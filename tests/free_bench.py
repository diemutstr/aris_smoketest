"""Measure the free-space planner on the fixed test set (tests/data/free_cases_<arm>.npz).

    OMP_NUM_THREADS=1 ../.venv/bin/python tests/free_bench.py [--workers 16] [--no-lift]
        [--cap 20000] [--arms 2R 1R] [--out tests/data/free_bench.npz]

Times are per plan on one core (workers run in parallel, one plan each at a time).
"""
from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import free_cases  # noqa: E402

import importlib  # noqa: E402

from aris.kernel import collide  # noqa: E402
from aris.rig import Rig  # noqa: E402
from aris.types import Motion  # noqa: E402

_scene = {}


def _run(args):
    arm_id, i, opts, package = args
    plan_detailed = importlib.import_module(package).plan_detailed
    if arm_id not in _scene:
        _scene[arm_id] = free_cases.scene(Rig.load(free_cases.CONFIG), arm_id)
    arm, obs, rules = _scene[arm_id]
    d = free_cases.load(arm_id)
    res, st = plan_detailed(arm, d["q_start"][i], d["q_goal"][i], obs, rules, options=opts)
    ok = isinstance(res, Motion)
    return dict(arm=arm_id, i=i, ok=ok, reason="" if ok else res.reason,
                detail="" if ok else res.detail, method=st.method, lifted=st.lifted,
                t_search=st.t_search, t_shorten=st.t_shorten, t_time=st.t_time,
                t_check=st.t_check, cpu=st.cpu, checked=st.checked, rounds=st.rounds,
                length=st.length, straight=st.straight, duration=st.duration, budget=st.budget,
                clearance=st.clearance[0] if st.clearance else np.nan)


def run(arms, opts, workers: int, subset=None, package: str = "aris.free"):
    jobs = [(a, i, opts, package) for a in arms for i in (subset if subset is not None else range(1000))]
    with ProcessPoolExecutor(workers) as ex:
        return list(ex.map(_run, jobs, chunksize=8))


def report(rows):
    groups = free_cases.GROUPS
    print(f"collision backend: {collide.backend()}; load average {os.getloadavg()}")
    print("times in ms per plan: wall-clock (t) and CPU (cpu); phase columns are wall medians")
    for a in sorted({r["arm"] for r in rows}):
        d = free_cases.load(a)
        print(f"\narm {a}")
        print(f"{'group':13s} {'solved':>7s} {'straight':>8s} {'lifted':>6s} {'tree':>5s} "
              f"{'lift s/g':>9s} "
              f"{'t med':>6s} {'t p95':>6s} {'t max':>6s} {'cpu md':>6s} {'cpu95':>6s} | "
              f"{'search':>6s} {'short':>6s} "
              f"{'time':>6s} {'check':>6s} | {'checked':>7s} {'len/str':>7s} {'dur med':>7s} "
              f"{'dur p95':>7s}")
        for gi, g in enumerate(groups + ("all",)):
            rs = [r for r in rows if r["arm"] == a and (g == "all" or d["group"][r["i"]] == gi)]
            ok = [r for r in rs if r["ok"]]
            tot = np.array([r["t_search"] + r["t_shorten"] + r["t_time"] + r["t_check"]
                            for r in ok]) * 1e3
            cpu = np.array([r["cpu"] for r in ok]) * 1e3
            med = lambda k: np.median([r[k] for r in ok]) * 1e3
            up = [r["lifted"] for r in rs if r["method"] in ("lifted", "tree")]
            lift = np.mean([u[0] for u in up] or [np.nan]), np.mean([u[1] for u in up] or [np.nan])
            ratio = np.median([r["length"] / r["straight"] for r in ok])
            print(f"{g:13s} {len(ok):4d}/{len(rs):<3d}"
                  f"{sum(r['method'] == 'straight' for r in ok):8d} "
                  f"{sum(r['method'] == 'lifted' for r in ok):6d} "
                  f"{sum(r['method'] == 'tree' for r in ok):5d} "
                  f"{lift[0]:4.2f}/{lift[1]:4.2f} "
                  f"{np.median(tot):6.1f} {np.percentile(tot, 95):6.1f} {tot.max():6.0f} "
                  f"{np.median(cpu):6.1f} {np.percentile(cpu, 95):6.1f} | "
                  f"{med('t_search'):6.1f} {med('t_shorten'):6.1f} {med('t_time'):6.1f} "
                  f"{med('t_check'):6.1f} | {np.median([r['checked'] for r in ok]):7.0f} "
                  f"{ratio:7.2f} {np.median([r['duration'] for r in ok]):7.2f} "
                  f"{np.percentile([r['duration'] for r in ok], 95):7.2f}")
        bad = [r for r in rows if r["arm"] == a and not r["ok"]]
        reasons = {}
        for r in bad:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        print(f"unsolved: {reasons}")
        for r in bad[:10]:
            print(f"  case {r['i']} ({groups[d['group'][r['i']]]}): {r['reason']}: {r['detail']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=16)
    ap.add_argument("--no-lift", action="store_true")
    ap.add_argument("--cap", type=int, default=None)
    ap.add_argument("--arms", nargs="+", default=["2R", "1R"])
    ap.add_argument("--cases", type=int, nargs="*", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--package", default="aris.free",
                    help="another copy of the planner to measure side by side (on PYTHONPATH)")
    a = ap.parse_args()
    Opt = importlib.import_module(a.package).Options
    opts = Opt(**({"lift_to": None} if a.no_lift else {}),
               **({} if a.cap is None else {"max_edges": a.cap}))
    rows = run(a.arms, opts, a.workers, a.cases, a.package)
    report(rows)
    if a.out:
        keys = [k for k in rows[0] if k not in ("detail", "lifted")]
        np.savez_compressed(a.out, **{k: np.array([r[k] for r in rows]) for k in keys},
                            lifted=np.array([r["lifted"] for r in rows]))


if __name__ == "__main__":
    main()
