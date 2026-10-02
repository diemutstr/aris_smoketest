"""The acceptance cases of the arm planner, and the script that measures them.

For arm 2L (phase 2 obstacles) and arm 1L (phase 1), from the park configuration back to it:
  word          the word "unknown", placed under the arm as in tests/local_cases.py
  corpus:<name> each of the five old corpus drawings, cut to 0.80 m from the arm's axis
  lines         100 random straight lines (every second one of local_cases.random_lines)
Every motion goes through the independent checker (`aris.check.check`) with the arm's phase.

Run from the repository root:
    ../.venv/bin/python tests/arm_cases.py [--arms 2L,1L] [--cases word,...] [--figure]
It prints the numbers of docs/modules/sequencer.md and arm_planner.md.
"""
from __future__ import annotations

import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import local_cases as lc  # noqa: E402

from aris import arm_planner  # noqa: E402
from aris.rig import Rig  # noqa: E402

DEPLOY = Path(__file__).resolve().parents[1]
CONFIG = DEPLOY / "config"
ARMS = lc.ARMS                                    # arm -> the phase in which it leads
FIGURE = DEPLOY / "docs" / "modules" / "figures" / "arm_word_31.png"
DRAW_SPEED = Rig.load(CONFIG).rules().draw_speed
OLD_WORD_2L = dict(plan_s=66.8, motion_s=67.8)    # the old planner, word, arm 2L


def drain(gen):
    """All motions of a planner's generator, and what it returns: -> (motions, leftovers)."""
    motions = []
    while True:
        try:
            motions.append(next(gen))
        except StopIteration as stop:
            return motions, stop.value


def plan_detailed(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None,
                  **kw):
    """`arm_planner.plan` collected: -> (motions, leftovers, PlanStats)."""
    st = arm_planner.PlanStats()
    ms, left = drain(arm_planner.plan(arm, lines, obstacles, q_start, rules, q_end, workers,
                                      cache_dir, stats=st, **kw))
    return ms, left, st


def plan_all(*a, **kw):
    """`arm_planner.plan` collected: -> (motions, leftovers)."""
    return plan_detailed(*a, **kw)[:2]


def on_surface(rig: Rig, lines) -> list:
    """The lines on the drawing surface, as the system planner hands them: z = paper - press."""
    from aris.types import Line
    z = rig.paper_z - rig.rules().press
    return [Line(x.id, np.column_stack([np.asarray(x.points, float)[:, :2],
                                        np.full(len(x.points), z)]), "table", x.intensity)
            for x in lines]


def case_lines(rig: Rig, arm_id: str) -> dict:
    """{case name: table-frame lines on the drawing surface}."""
    axis = rig.T_table_base(arm_id)[:2, 3]
    out = {"word": lc.word(axis, rig.T_table_base(lc.WORD_ARM)[:2, 3])}
    for line in lc.corpus(axis):
        out.setdefault("corpus:" + line.id.split(":")[1], []).append(line)
    out["lines"] = lc.random_lines(axis, lc.SEED[arm_id])[::2]
    return {k: on_surface(rig, v) for k, v in out.items()}


def plan_case(rig: Rig, arm_id: str, lines_table, cache_dir=None, workers: int = 1,
              draw_speed=None, **kw):
    """-> (motions, leftovers, stats, machine load at the start)."""
    from dataclasses import replace
    arm, obs, rules, _ = lc.problem(rig, arm_id)
    if draw_speed is not None:
        rules = replace(rules, draw_speed=draw_speed)
    lines = [rig.to_base(arm_id, x) for x in lines_table]
    load = os.getloadavg()[0]
    q = rig.park_q(arm_id)
    motions, leftovers, st = plan_detailed(arm, lines, obs, q, rules, workers=workers,
                                           cache_dir=cache_dir, **kw)
    return motions, leftovers, st, load


def _verify(config_dir, arm_id: str, phase_n: int, motion, q_before) -> dict:
    """The independent checker as the planners' `verify`: -> {"passed", "tightest", ...}."""
    from aris.check import check
    v = check(config_dir, arm_id, motion, Rig.load(config_dir).phase(phase_n), q_before)
    worst = next((m for m in v.measurements if m.name == v.tightest), None)
    return {"passed": bool(v.passed), "tightest": str(v.tightest),
            "detail": "" if worst is None else str(worst.detail),
            "min_clearance": float(v.min_clearance)}


def checker_verify(arm_id: str):
    """`verify` for one arm in the phase it leads: the independent checker, picklable."""
    from functools import partial
    return partial(_verify, CONFIG, arm_id, ARMS[arm_id])


def roles(motions) -> list[str]:
    """What each motion is in the tour: move, set-down, draw, lift-off."""
    names = {"draw": "draw", "lower": "set-down", "lift": "lift-off", "free": "move"}
    return [names[m.kind] for m in motions]


def _check_one(job):
    from aris.check import check
    arm_id, motion, q_before, draw_speed = job
    rig = Rig.load(CONFIG)
    v = check(CONFIG, arm_id, motion, rig.phase(ARMS[arm_id]), q_before)  # draw speed: rig.json
    worst = next(m for m in v.measurements if m.name == v.tightest) if v.tightest else None
    row = "" if worst is None else (f"{worst.name} {worst.value:.5g} (limit {worst.limit:.4g} "
                                    f"{worst.unit}) {worst.detail}")
    failed = tuple((m.name, m.value, m.limit, m.detail) for m in v.measurements if not m.passed)
    slowest = next((m.value for m in v.measurements if m.name == "never stops"), None)
    return v.passed, failed, row, v.min_clearance, slowest


def save_motion(path, arm_id: str, motion, q_before, note: str = "") -> None:
    """One motion as an npz, for the checker's tests (trajectory, tips, piece, q_before)."""
    p = motion.piece
    np.savez_compressed(path, arm_id=arm_id, kind=motion.kind, t=motion.traj.t,
                        q=motion.traj.q, qd=motion.traj.qd, q_before=q_before,
                        tip_base=np.zeros((0, 3)) if motion.tip_base is None else motion.tip_base,
                        line_id="" if p is None else p.line_id,
                        s0=np.nan if p is None else p.s0, s1=np.nan if p is None else p.s1,
                        intensity=motion.intensity, note=note)


def check_all(rig: Rig, arm_id: str, motions, workers: int = 16, draw_speed=None) -> list:
    """Every motion through the checker, q_before = where the previous one ended.
    -> [(passed, failed rows, tightest row, min clearance)] in order."""
    from aris.types import DrawRules
    speed = DrawRules().draw_speed if draw_speed is None else draw_speed
    q = [rig.park_q(arm_id)] + [m.q_end for m in motions[:-1]]
    jobs = [(arm_id, m, qb, speed) for m, qb in zip(motions, q)]
    if workers <= 1:
        return [_check_one(j) for j in jobs]
    with ProcessPoolExecutor(workers) as pool:
        return list(pool.map(_check_one, jobs))


def summary(name: str, motions, leftovers, st, load, checks=None, arm_id=None) -> list[str]:
    t = st.tour
    out = [f"{name}: {st.lines} lines, {st.bunches} pieces offered, machine load {load:.0f}",
           f"  planning: CPU {st.cpu:.1f} s, wall {st.wall:.1f} s (tour CPU {t.cpu:.1f} s); first motion "
           f"after CPU {st.first_cpu:.2f} s, wall {st.first_wall:.2f} s",
           f"  tour: {t.pieces} pieces, {t.lifts} lifts, {t.motions} motions; drawing "
           f"{t.draw_time:.1f} s, pen up {t.penup_time:.1f} s (moves {t.move_time:.1f} s), share "
           f"{t.penup_share:.3f}; longest free motion {t.longest_free:.2f} s; moves "
           f"{t.free_length:.1f} rad of joint travel; free-space calls {t.free_calls}, refused "
           f"{sum(t.refusals.values())} {t.refusals or ''}"
           + (f"; MOVE HOME REFUSED: {t.end_refusal}" if t.end_refusal else "")]
    by = {}
    for x in leftovers:
        n, m = by.get(x.reason, (0, 0.0))
        by[x.reason] = (n + 1, m + x.piece.s1 - x.piece.s0)
    out.append("  leftovers: " + (", ".join(f"{k} {n} ({m:.3f} m)" for k, (n, m) in
                                            sorted(by.items())) or "none"))
    if getattr(t, "cut", 0.0):
        out.append(f"  cut off at piece ends (no straight lift-off): {t.cut * 1e3:.0f} mm")
    if checks is not None:
        rl = roles(motions)
        passed = sum(c[0] for c in checks)
        tight = min(c[3] for c in checks) if checks else float("nan")
        out.append(f"  checker: {passed} of {len(checks)} motions pass; smallest clearance "
                   f"beyond demanded {tight * 1e3:.1f} mm")
        slow = sorted((c[4], m.piece.line_id) for m, c in zip(motions, checks)
                      if m.kind == "draw" and c[4] is not None)
        if slow:
            floor = 0.245 * DRAW_SPEED       # a quarter of the draw speed, less the rounding
            under = [x for x in slow if x[0] < floor]
            out.append(f"  slowest mid-line pen speed per drawing: lowest {slow[0][0] * 1e3:.2f} "
                       f"mm/s ({slow[0][1]}), median {slow[len(slow) // 2][0] * 1e3:.1f} mm/s; "
                       f"{len(under)} of {len(slow)} under {floor * 1e3:.2f} mm/s"
                       + ("".join(f"; {i} {v * 1e3:.2f}" for v, i in under[:6])))
        fails = {}
        for r, c in zip(rl, checks):
            if not c[0]:
                fails.setdefault((r, tuple(f[0] for f in c[1])), []).append(c[2])
        for (r, names), rows in sorted(fails.items()):
            out.append(f"    {len(rows)} {r} fail on {', '.join(names)}; tightest e.g. {rows[0]}")
    return out


def big_lines(rig: Rig, arm_id: str, n: int = 1000) -> list:
    """The first n lines of tests/big_cases.big() that lie wholly within 0.80 m of the arm's
    axis (table frame)."""
    import big_cases
    from aris.types import Line
    axis = rig.T_table_base(arm_id)[:2, 3]
    out = []
    for x in big_cases.big()["lines"]:
        xy = np.asarray(x["points"], float) * 1e-3
        if np.all(np.linalg.norm(xy - axis, axis=1) <= lc.CORPUS_RADIUS):
            out.append(Line(x["id"], np.column_stack([xy, np.zeros(len(xy))]), "table"))
            if len(out) == n:
                break
    return on_surface(rig, out)


def compare_verify(rig: Rig, arm_id: str, name: str, lines, cache=None) -> list[str]:
    """The same case planned without and with the checker in the loop."""
    ms0, left0, st0, load = plan_case(rig, arm_id, lines, cache)
    ms1, left1, st1, _ = plan_case(rig, arm_id, lines, cache, verify=checker_verify(arm_id))
    t1 = st1.tour
    bad = [x for x in left1 if x.reason == "failed_check"]
    ok = sum(1 for m in ms1 if m.checked is not None and m.checked["passed"])
    out = [f"arm {arm_id} {name}: load {load:.0f}; without verify: CPU {st0.cpu:.1f} s, wall "
           f"{st0.wall:.1f} s, {len(ms0)} motions; with verify: CPU {st1.cpu:.1f} s, wall "
           f"{st1.wall:.1f} s ({t1.verify_wall:.1f} s inside verify, {t1.verified} calls), "
           f"{len(ms1)} motions, {ok} of them carry checked passed; first motion after "
           f"{st0.first_wall:.2f} / {st1.first_wall:.2f} s wall; failed_check {len(bad)}"
           + (f"; MOVE HOME: {t1.end_refusal}" if t1.end_refusal else "")]
    out += [f"    {x.piece} {x.detail}" for x in bad]
    return out


def figure(rig: Rig, arm_id: str, motions, path=FIGURE) -> None:
    """Top view of a tour: drawing in one colour, free moves in another, numbered in order."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    arm = rig.arm(arm_id)
    fig, ax = plt.subplots(figsize=(11, 7))
    axis = rig.T_table_base(arm_id)[:2, 3]
    ax.plot(*axis, "k+", ms=14, mew=2)
    ax.annotate(f"arm {arm_id} (axis)", axis, (-30, -18), textcoords="offset points", fontsize=8)
    n = 0
    rl = roles(motions)
    for i, (m, r) in enumerate(zip(motions, rl)):
        if r == "draw":
            xy = rig.to_table(arm_id, m.tip_base)[:, :2]
            ax.plot(xy[:, 0], xy[:, 1], color="#1f5fa8", lw=2.2, zorder=3)
            if rl[i - 1] == "set-down":                  # a new piece starts here
                n += 1
                ax.plot(*xy[0], "o", color="#1f5fa8", ms=4, zorder=4)
                ax.annotate(str(n), xy[0], (5, 5 if n % 2 else -12), textcoords="offset points",
                            fontsize=8, color="#1f5fa8", zorder=5,
                            bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.8))
        elif r == "move":
            xy = rig.to_table(arm_id, arm.tip(m.traj.q))[:, :2]
            ax.plot(xy[:, 0], xy[:, 1], color="#d9822b", lw=1.0, ls="--", zorder=2)
    park = rig.to_table(arm_id, arm.tip(rig.park_q(arm_id)[None]))[0, :2]
    ax.plot(*park, "s", color="#d9822b", ms=6, zorder=4)
    ax.annotate("park (start and end)", park, (8, -4), textcoords="offset points", fontsize=8,
                color="#a35a12")
    ax.plot([], [], color="#1f5fa8", lw=2.2, label="drawing (number: order of the pieces, dot: "
            "where the pen goes down)")
    ax.plot([], [], color="#d9822b", lw=1.0, ls="--", label="pen up, the pen tip seen from above")
    ax.set_aspect("equal")
    ax.set_xlabel("table x (m)")
    ax.set_ylabel("table y (m)")
    ax.set_title(f"arm {arm_id}: the tour of the word, from park back to park (top view)")
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="plan and check the arm planner's cases")
    ap.add_argument("--arms", default="2L,1L")
    ap.add_argument("--cases", default="")
    ap.add_argument("--workers", type=int, default=1, help="local planner processes")
    ap.add_argument("--check-workers", type=int, default=16)
    ap.add_argument("--cache", default="", help="kinematic table directory")
    ap.add_argument("--figure", action="store_true")
    ap.add_argument("--draw-speed", type=float, default=None, help="m/s, instead of the rules'")
    ap.add_argument("--big", type=int, default=0, help="add a case of this many big_cases lines")
    ap.add_argument("--verify", action="store_true",
                    help="plan each case with and without the checker in the loop, compare")
    a = ap.parse_args()
    rig = Rig.load(CONFIG)
    cache = a.cache or None
    for arm_id in a.arms.split(","):
        cases = case_lines(rig, arm_id)
        if a.big:
            cases[f"big{a.big}"] = big_lines(rig, arm_id, a.big)
        for name, lines in cases.items():
            if a.cases and name not in a.cases.split(","):
                continue
            if a.verify:
                print("\n".join(compare_verify(rig, arm_id, name, lines, cache)), flush=True)
                continue
            ms, left, st, load = plan_case(rig, arm_id, lines, cache, a.workers, a.draw_speed)
            t = time.perf_counter()
            checks = check_all(rig, arm_id, ms, a.check_workers, a.draw_speed)
            print("\n".join(summary(f"arm {arm_id} {name}", ms, left, st, load, checks, arm_id)))
            print(f"  (checking took {time.perf_counter() - t:.0f} s wall)", flush=True)
            q_before = [rig.park_q(arm_id)] + [m.q_end for m in ms[:-1]]
            stops = [i for i, c in enumerate(checks) if any(f[0] == "never stops" for f in c[1])]
            for k, i in enumerate(stops):
                path = DEPLOY / "tests" / "data" / f"arm_stop_{arm_id}_{name.replace(':', '_')}_{k}.npz"
                save_motion(path, arm_id, ms[i], q_before[i], checks[i][2])
                print(f"  saved the motion that fails 'never stops' to {path}")
            if a.figure and arm_id == "2L" and name == "word":
                figure(rig, arm_id, ms)
