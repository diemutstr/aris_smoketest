"""The acceptance cases of the system planner, and the script that measures them.

Whole drawings in the table frame, over the admissible drawing area of rig.json (canvas,
drawing_area_m: 1.56 x 3.56 m, centred on the table):
  word       the word "unknown" (tests/local_cases.py), scaled to 0.55 m wide and centred on the
             table, so it sits between slots 2L and 2R as on the hardware day
  hatch      40 parallel lines 1.5 m long across the canvas (along x), evenly spaced along it
  scatter    80 short random lines (5 to 20 cm) anywhere on the canvas
  starburst  24 rays from the table centre to the edge of the canvas
  spiral     one spiral of 4 turns, stretched to the canvas
  duotone    10 wavy bands the length of the canvas
  random     300 random straight lines anywhere on the canvas, 5 cm to 1.5 m long
All seeds are fixed.  Every motion goes through the independent checker with its phase, and
`check_phase_end` between phases.

Run from the repository root:
    .venv/bin/python tests/system_cases.py [--cases word,hatch,...] [--workers 30] [--cache DIR]
It prints the numbers of docs/modules/system.md, draws docs/modules/figures/system_<case>.png
and stores the numbers in tests/data/system_<case>.npz.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import local_cases as lc  # noqa: E402

from aris.rig import Rig  # noqa: E402
from aris.system import account, phase_named, plan_detailed  # noqa: E402
from aris.system.phases import is_fill  # noqa: E402
from aris.types import Line  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
FIGURES = ROOT / "docs" / "modules" / "figures"
DATA = Path(__file__).resolve().parent / "data"
HALF = np.array([0.9017, 1.81532])          # half the canvas
# The drawings lie inside the admissible drawing area of rig.json (the system planner refuses
# anything outside it), scaled to it.
AREA = np.asarray(json.loads((CONFIG / "rig.json").read_text())["canvas"]["drawing_area_m"])
AHALF = 0.5 * AREA
INSET = 0.01                                # m kept from the edge of the drawing area
CASES = ("word", "hatch", "scatter", "starburst", "spiral", "duotone", "random")


def _line(lid, xy) -> Line:
    xy = np.asarray(xy, float)
    return Line(lid, np.column_stack([xy, np.zeros(len(xy))]), "table")


def word(width: float = 0.55) -> list[Line]:
    raw = [np.asarray(s["pts"], float) for s in json.loads(lc.WORD_FILE.read_text())["strokes"]]
    allp = np.concatenate(raw)
    lo, hi = allp.min(0), allp.max(0)
    k = width / (hi[0] - lo[0])
    return [_line(f"word:{i}", (p - 0.5 * (lo + hi)) * k) for i, p in enumerate(raw)]


def hatch() -> list[Line]:
    ys = np.linspace(-AHALF[1] + 0.05, AHALF[1] - 0.05, 40)
    x = np.linspace(-0.75, 0.75, 31)
    return [_line(f"hatch:{i}", np.column_stack([x, np.full_like(x, y)]))
            for i, y in enumerate(ys)]


def scatter(seed: int = 1) -> list[Line]:
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < 80:
        c = rng.uniform(-AHALF + INSET, AHALF - INSET)
        t, L = rng.uniform(0, np.pi), rng.uniform(0.05, 0.20)
        d = 0.5 * L * np.array([np.cos(t), np.sin(t)])
        xy = np.array([c - d, c + d])
        if np.all(np.abs(xy) <= AHALF - INSET):
            out.append(_line(f"scatter:{len(out)}", xy))
    return out


def starburst() -> list[Line]:
    out = []
    for i, t in enumerate(np.arange(24) * 2 * np.pi / 24 + np.pi / 48):
        d = np.array([np.cos(t), np.sin(t)])
        r = np.min((AHALF - INSET) / np.maximum(np.abs(d), 1e-12))
        u = np.linspace(0.02, r, max(2, int(np.ceil(r / 0.05)) + 1))
        out.append(_line(f"starburst:{i}", u[:, None] * d))
    return out


def spiral() -> list[Line]:
    t = np.linspace(0, 4 * 2 * np.pi, 4000)
    r = 0.03 + (1 - 0.03) * t / t[-1]
    xy = np.column_stack([r * np.cos(t), r * np.sin(t)]) * (AHALF - 0.03)
    return [_line("spiral:0", xy)]


def duotone(seed: int = 3) -> list[Line]:
    rng = np.random.default_rng(seed)
    y = np.linspace(-AHALF[1] + 0.03, AHALF[1] - 0.03, 600)
    out = []
    for i, x0 in enumerate(np.linspace(-(AHALF[0] - 0.06), AHALF[0] - 0.06, 10)):
        amp, waves, ph = rng.uniform(0.02, 0.05), rng.uniform(2, 4), rng.uniform(0, 2 * np.pi)
        x = x0 + amp * np.sin(2 * np.pi * waves * (y - y[0]) / (y[-1] - y[0]) + ph)
        out.append(_line(f"duotone:{i}", np.column_stack([x, y])))
    return out


def random_lines(n: int = 300, seed: int = 7) -> list[Line]:
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        c = rng.uniform(-AHALF + INSET, AHALF - INSET)
        t, L = rng.uniform(0, np.pi), rng.uniform(0.05, 1.5)
        d = 0.5 * L * np.array([np.cos(t), np.sin(t)])
        xy = np.array([c - d, c + d])
        if np.all(np.abs(xy) <= AHALF - INSET):
            k = max(2, int(np.ceil(L / 0.05)) + 1)
            out.append(_line(f"random:{len(out)}", xy[0] + np.linspace(0, 1, k)[:, None]
                             * (xy[1] - xy[0])))
    return out


def drawing(name: str) -> list[Line]:
    return {"word": word, "hatch": hatch, "scatter": scatter, "starburst": starburst,
            "spiral": spiral, "duotone": duotone, "random": random_lines}[name]()


# --------------------------------------------------------------------------- checking


def _check_one(job):
    from aris.check import check
    arm_id, phase_name, motion, q_before = job
    rig = Rig.load(CONFIG)
    v = check(CONFIG, arm_id, motion, phase_named(rig, phase_name), q_before)
    failed = tuple(m.name for m in v.measurements if not m.passed)
    return v.passed, failed, v.min_clearance


def check_all(rig, tagged, workers: int = 16):
    """Every motion through the checker (q_before: where the same arm's previous motion ended,
    or its park), and `check_phase_end` after every phase.
    -> (motion verdicts [(passed, failed names, min clearance)], phase-end verdicts
    [(phase, passed, tightest)])."""
    from aris.check import check_phase_end
    q = {a: rig.park_q(a) for a in rig.arm_ids}
    jobs, ends, order = [], {}, []
    for ph, a, m in tagged:
        jobs.append((a, ph, m, q[a]))
        q[a] = m.q_end
        if ph not in ends:
            order.append(ph)
        ends.setdefault(ph, {b: q[b] for b in phase_named(rig, ph).active})[a] = m.q_end
    if workers <= 1:
        verdicts = [_check_one(j) for j in jobs]
    else:
        with ProcessPoolExecutor(workers) as pool:
            verdicts = list(pool.map(_check_one, jobs, chunksize=4))
    phase_ends = []
    for ph in order:
        v = check_phase_end(CONFIG, phase_named(rig, ph), ends[ph])
        phase_ends.append((ph, v.passed, v.tightest))
    return verdicts, phase_ends


# --------------------------------------------------------------------------- one case


def run_case(rig, name, lines, cache_dir, workers, check_workers=16, do_check=True):
    rules = rig.rules()
    load = os.getloadavg()[0]
    tagged, left, rep = plan_detailed(rig, lines, rules, cache_dir=cache_dir, workers=workers)
    acc = account(lines, tagged, left, rules.min_piece)
    checks = check_all(rig, tagged, check_workers) if do_check else None
    return dict(name=name, lines=lines, tagged=tagged, left=left, rep=rep, acc=acc,
                checks=checks, load=load)


def numbers(res) -> dict:
    """The measured numbers of one case, flat, for the page and the stored npz."""
    rep, acc = res["rep"], res["acc"]
    out = dict(length=acc.length, drawn=acc.drawn, left=acc.left, twice=acc.twice,
               drawing_time=rep.drawing_time, cpu=rep.cpu, wall=rep.wall,
               first_wall=rep.first_wall, map_cpu=rep.map_cpu,
               motions=len(res["tagged"]), load=res["load"], cuts=rep.cuts,
               phases_run=len(rep.phases), phases_skipped=len(rep.skipped))
    lead = sum(r.drawn for p in rep.phases if not is_fill(p) for r in p.arms.values())
    out["drawn_phases_12"] = lead
    out["drawn_fill"] = sum(r.drawn for p in rep.phases if is_fill(p) for r in p.arms.values())
    for reason, m in acc.left_by_reason.items():
        out[f"left_{reason}"] = m
    if res["checks"] is not None:
        v, ends = res["checks"]
        out["checked"], out["passed"] = len(v), sum(x[0] for x in v)
        out["min_clearance"] = min((x[2] for x in v), default=np.nan)
        out["phase_ends"], out["phase_ends_passed"] = len(ends), sum(e[1] for e in ends)
    return out


def summary(res) -> list[str]:
    rep, acc, n = res["rep"], res["acc"], numbers(res)
    L = max(acc.length, 1e-12)
    out = [f"{res['name']}: {len(res['lines'])} lines, {acc.length:.2f} m, machine load "
           f"{res['load']:.0f}",
           f"  drawn {acc.drawn:.2f} m ({acc.drawn / L:.3f}); phases 1+2 "
           f"{n['drawn_phases_12']:.2f} m ({n['drawn_phases_12'] / L:.3f}), fill "
           f"{n['drawn_fill']:.2f} m ({n['drawn_fill'] / L:.3f}); joins drawn twice "
           f"{acc.twice:.3f} m; {rep.cuts} cuts",
           "  skipped: " + ("; ".join(f"{p} ({why})" for p, why in rep.skipped) or "none"),
           "  left over: " + (", ".join(f"{k} {m:.3f} m" for k, m in
                                        sorted(acc.left_by_reason.items())) or "nothing"),
           f"  planning: CPU {rep.cpu:.1f} s, wall {rep.wall:.1f} s (maps CPU {rep.map_cpu:.1f} s);"
           f" first motion after {rep.first_wall:.1f} s wall",
           f"  on the rig: {rep.drawing_time:.0f} s = "
           + " + ".join(f"{p.duration:.0f} ({p.name})" for p in rep.phases)]
    for p in rep.phases:
        for a, r in p.arms.items():
            out.append(f"    {p.name:8s} arm {a:>3}: {r.stretches:3d} stretches, offered "
                       f"{r.offered:6.2f} m, drawn {r.drawn:6.2f} m ({r.drawn / L:.3f}), "
                       f"{r.motions:4d} motions, {r.motion_time:6.0f} s; plan CPU {r.cpu:6.1f} "
                       f"wall {r.wall:6.1f} s, first motion {r.first_phase_wall:5.1f} s; back: "
                       + (", ".join(f"{k} {m:.2f}" for k, m in sorted(r.handed_back.items()))
                          or "-"))
    if res["checks"] is not None:
        v, ends = res["checks"]
        fails = {}
        for x in v:
            if not x[0]:
                fails[x[1]] = fails.get(x[1], 0) + 1
        out.append(f"  checker: {n['passed']} of {n['checked']} motions pass (smallest clearance "
                   f"beyond demanded {n['min_clearance'] * 1e3:.1f} mm); phase ends "
                   f"{n['phase_ends_passed']} of {n['phase_ends']} pass"
                   + "".join(f"; {c} fail on {', '.join(k)}" for k, c in fails.items()))
    return out


# --------------------------------------------------------------------------- figures

PHASE_COLOUR = {"phase 1": "#2a78d6", "phase 2": "#eb6834", "fill": "#1baf7a"}
ARM_COLOUR = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#4a3aa7"]
LEFT_COLOUR = "#d03b3b"


def _walls(ax, rig, which=(1, 2)):
    for n, style in ((1, "-"), (2, "--")):
        if n not in which:
            continue
        for w in rig.phase(n).walls:
            d = np.array([-w.normal_table[1], w.normal_table[0]])
            p = w.point_table[:2] + np.linspace(-1.5, 1.5, 2)[:, None] * d
            ax.plot(p[:, 0], p[:, 1], style, color="#52514e", lw=1.0,
                    label=f"walls, phase {n}" if w is rig.phase(n).walls[0] else None)


def _frame(ax, rig, which=(1, 2)):
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle(-HALF, *(2 * HALF), fill=False, lw=0.8, ec="#8a8984"))
    ax.add_patch(Rectangle(-AHALF, *AREA, fill=False, lw=0.8, ls=":", ec="#8a8984"))
    _walls(ax, rig, which)
    for a in rig.arm_ids:
        c = rig.T_table_base(a)[:2, 3]
        ax.plot(*c, "o", ms=6, mfc="white", mec="#0b0b0b")
        ax.annotate(str(a), c, xytext=(6, 4), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.9, 1.9)
    ax.tick_params(labelsize=7)


def _left_xy(res, x):
    line = next(l for l in res["lines"] if l.id == x.piece.line_id)
    from aris.system.stretch import of_line
    return of_line(line).sub(x.piece.s0, x.piece.s1).points


def figure(rig, res, path) -> None:
    """Two top views of the drawing: coloured by phase, and by arm; leftovers in red."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(9, 8.2))
    arms = list(rig.arm_ids)
    for key, c in PHASE_COLOUR.items():                  # legends in a fixed order
        axes[0].plot([], [], color=c, lw=1.2, label=key)
    for a in arms:
        axes[1].plot([], [], color=ARM_COLOUR[arms.index(a)], lw=1.2, label=f"arm {a}")
    for ph, a, m in res["tagged"]:
        if m.kind != "draw":
            continue
        p = rig.to_table(a, m.tip_base)
        key = "fill" if ph.startswith("fill") else ph
        axes[0].plot(p[:, 0], p[:, 1], color=PHASE_COLOUR[key], lw=1.2)
        axes[1].plot(p[:, 0], p[:, 1], color=ARM_COLOUR[arms.index(a)], lw=1.2)
    for i, x in enumerate(res["left"]):
        p = _left_xy(res, x)
        for ax in axes:
            ax.plot(p[:, 0], p[:, 1], color=LEFT_COLOUR, lw=2.2, zorder=5,
                    label="left over" if i == 0 else None)
    for ax, title in zip(axes, ("by phase", "by arm")):
        _frame(ax, rig)
        ax.set_title(f"{res['name']}: {title}", fontsize=10)
        ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.0, 1.0), frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def maps_figure(rig, maps, path) -> None:
    """Which arm can draw where: phase 1, phase 2, and every arm alone (the fill phases)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb
    from aris.system.maps import DRAWABLE
    from aris.system.phases import phases
    arms = list(rig.arm_ids)
    fig, axes = plt.subplots(1, 3, figsize=(10, 6.2))
    groups = [("phase 1", [rig.phase(1)]), ("phase 2", [rig.phase(2)]),
              ("fill: each arm alone (overlaps darker)", [p for p in phases(rig) if is_fill(p)])]
    for ax, (title, phs) in zip(axes, groups):
        m0 = maps[(phs[0].name, phs[0].active[0])]
        img = np.ones((len(m0.y), len(m0.x), 3)) * 0.97
        count = np.zeros((len(m0.x), len(m0.y)))
        for p in phs:
            for a in p.active:
                d = maps[(p.name, a)].state == DRAWABLE
                count += d
                img[d.T] = np.array(to_rgb(ARM_COLOUR[arms.index(a)])) * 0.55 + 0.45 \
                    if len(phs) == 1 else img[d.T]
        if len(phs) > 1:
            shade = 0.97 - 0.28 * np.clip(count, 0, 3).T
            img = np.repeat(shade[:, :, None], 3, axis=2)
        st = m0.x[1] - m0.x[0]
        ax.imshow(img, origin="lower", extent=(m0.x[0] - st / 2, m0.x[-1] + st / 2,
                                                m0.y[0] - st / 2, m0.y[-1] + st / 2))
        _frame(ax, rig, (1,) if title == "phase 1" else (2,) if title == "phase 2" else (1, 2))
        ax.set_title(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", default=",".join(CASES))
    ap.add_argument("--workers", type=int, default=30)
    ap.add_argument("--check-workers", type=int, default=30)
    ap.add_argument("--cache", default=str(ROOT / "out" / "cache"), help="maps and kinematic table (out/ is not versioned)")
    ap.add_argument("--no-check", action="store_true")
    ap.add_argument("--no-figure", action="store_true")
    a = ap.parse_args()
    rig = Rig.load(CONFIG)
    if not a.no_figure:
        from aris.system import maps as mp
        from aris.system.phases import phases
        from aris.system.settings import Settings
        M = mp.load_or_build(rig, phases(rig), rig.rules().gates, Settings(), a.cache, a.workers)
        print("coverage:", json.dumps(mp.coverage(M, phases(rig))))
        maps_figure(rig, M, FIGURES / "system_maps.png")
    for name in a.cases.split(","):
        t = time.perf_counter()
        res = run_case(rig, name, drawing(name), a.cache, a.workers, a.check_workers,
                       not a.no_check)
        print("\n".join(summary(res)), f"\n  (case took {time.perf_counter() - t:.0f} s)",
              flush=True)
        np.savez(DATA / f"system_{name}.npz", **{k: v for k, v in numbers(res).items()})
        if not a.no_figure:
            figure(rig, res, FIGURES / f"system_{name}.png")


if __name__ == "__main__":
    main()
