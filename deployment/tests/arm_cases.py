"""The acceptance cases of the arm planner, and the script that measures them.

For arm 31 (phase 2 obstacles) and arm 13 (phase 1), from the park configuration back to it:
  word          the word "unknown", placed under the arm as in tests/local_cases.py
  corpus:<name> each of the five old corpus drawings, cut to 0.80 m from the arm's axis
  lines         100 random straight lines (every second one of local_cases.random_lines)
Every motion goes through the independent checker (`aris.check.check`) with the arm's phase.

Run from deployment/:
    ../.venv/bin/python tests/arm_cases.py [--arms 31,13] [--cases word,...] [--figure]
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

from aris.arm_planner import plan_detailed  # noqa: E402
from aris.rig import Rig  # noqa: E402

DEPLOY = Path(__file__).resolve().parents[1]
CONFIG = DEPLOY / "config"
ARMS = lc.ARMS                                    # arm -> the phase in which it leads
FIGURE = DEPLOY / "docs" / "modules" / "figures" / "arm_word_31.png"
OLD_WORD_31 = dict(plan_s=66.8, motion_s=67.8)    # the old planner, word, arm 31


def case_lines(rig: Rig, arm_id: int) -> dict:
    """{case name: table-frame lines}."""
    axis = rig.T_table_base(arm_id)[:2, 3]
    out = {"word": lc.word(axis, rig.T_table_base(lc.WORD_ARM)[:2, 3])}
    for line in lc.corpus(axis):
        out.setdefault("corpus:" + line.id.split(":")[1], []).append(line)
    out["lines"] = lc.random_lines(axis, arm_id)[::2]
    return out


def plan_case(rig: Rig, arm_id: int, lines_table, cache_dir=None, workers: int = 1,
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


def roles(motions) -> list[str]:
    """What each motion is in the tour: move, set-down, draw, lift-off."""
    out = []
    for i, m in enumerate(motions):
        if m.kind == "draw":
            out.append("draw")
        elif i + 1 < len(motions) and motions[i + 1].kind == "draw":
            out.append("set-down")
        elif i > 0 and motions[i - 1].kind == "draw":
            out.append("lift-off")
        else:
            out.append("move")
    return out


def _check_one(job):
    from aris.check import check
    arm_id, motion, q_before, draw_speed = job
    rig = Rig.load(CONFIG)
    v = check(CONFIG, arm_id, motion, rig.phase(ARMS[arm_id]), q_before, draw_speed=draw_speed)
    worst = next(m for m in v.measurements if m.name == v.tightest) if v.tightest else None
    row = "" if worst is None else (f"{worst.name} {worst.value:.5g} (limit {worst.limit:.4g} "
                                    f"{worst.unit}) {worst.detail}")
    failed = tuple((m.name, m.value, m.limit, m.detail) for m in v.measurements if not m.passed)
    return v.passed, failed, row, v.min_clearance


# Two places where the checker (as of 2026-09-30, 09:40) and the planners disagree by design;
# see docs/modules/sequencer.md, "What the checker says".
LINK1_OWN_STEEL = 0.020   # m: link 1 against its own hanger steel (commit f371483; the checker
                          # still asks the 0.050 of all steel)
PEN_DIP = 0.005           # m: the pen capsule's round end dips below the tip it touches the paper
                          # with by at most its radius (5 mm; 0.3 to 1.3 mm at the pen's lean)


def known_disagreement(arm_id: int, role: str, check) -> bool:
    """True if every failed row of a verdict is one of the two known disagreements:
    - link 1 near its own struts, plate or clamp, at 0.020 m or more (the checker asks 0.050)
    - a set-down or lift-off whose pen touches the paper at its drawing end (the checker asks
      the lifted pen's 0.003 m of every free motion)."""
    own = (f"strut{arm_id}_", f"plate{arm_id}", f"clamp{arm_id}")
    for name, value, limit, detail in check[1]:
        link1 = "link1." in detail and any(o in detail for o in own)
        pen = role in ("set-down", "lift-off") and "pen / paper" in detail
        if name == "clearance steel" and link1 and value >= LINK1_OWN_STEEL:
            continue
        if name.startswith("hold") and link1 and value + 0.050 >= LINK1_OWN_STEEL:
            continue
        if name == "clearance paper (pen)" and pen and value >= -PEN_DIP:
            continue
        if name.startswith("hold") and role == "set-down" and "clearance paper (pen)" in detail \
                and value >= -0.003 - PEN_DIP:
            continue
        return False
    return True


def check_all(rig: Rig, arm_id: int, motions, workers: int = 16, draw_speed=None) -> list:
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
           f"  planning: CPU {st.cpu:.1f} s, wall {st.wall:.1f} s (local planner CPU "
           f"{st.local_cpu:.1f} s, wall {st.local_wall:.1f}; tour CPU {t.cpu:.1f} s); first motion "
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
    if checks is not None:
        rl = roles(motions)
        passed = sum(c[0] for c in checks)
        known = sum(not c[0] and known_disagreement(arm_id, r, c) for r, c in zip(rl, checks))
        tight = min(c[3] for c in checks) if checks else float("nan")
        out.append(f"  checker: {passed} of {len(checks)} motions pass; {known} more fail only on "
                   f"the two known disagreements; smallest clearance beyond demanded "
                   f"{tight * 1e3:.1f} mm")
        fails = {}
        for r, c in zip(rl, checks):
            if not c[0]:
                key = (r, tuple(f[0] for f in c[1]), known_disagreement(arm_id, r, c))
                fails.setdefault(key, []).append(c[2])
        for (r, names, k), rows in sorted(fails.items()):
            out.append(f"    {len(rows)} {r} fail on {', '.join(names)}"
                       f"{' (known)' if k else ''}; tightest e.g. {rows[0]}")
    return out


def figure(rig: Rig, arm_id: int, motions, path=FIGURE) -> None:
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
    ap.add_argument("--arms", default="31,13")
    ap.add_argument("--cases", default="")
    ap.add_argument("--workers", type=int, default=1, help="local planner processes")
    ap.add_argument("--check-workers", type=int, default=16)
    ap.add_argument("--cache", default="", help="kinematic table directory")
    ap.add_argument("--figure", action="store_true")
    ap.add_argument("--draw-speed", type=float, default=None, help="m/s, instead of the rules'")
    a = ap.parse_args()
    rig = Rig.load(CONFIG)
    cache = a.cache or None
    for arm_id in (int(x) for x in a.arms.split(",")):
        for name, lines in case_lines(rig, arm_id).items():
            if a.cases and name not in a.cases.split(","):
                continue
            ms, left, st, load = plan_case(rig, arm_id, lines, cache, a.workers, a.draw_speed)
            t = time.perf_counter()
            checks = check_all(rig, arm_id, ms, a.check_workers, a.draw_speed)
            print("\n".join(summary(f"arm {arm_id} {name}", ms, left, st, load, checks, arm_id)))
            print(f"  (checking took {time.perf_counter() - t:.0f} s wall)", flush=True)
            if a.figure and arm_id == 31 and name == "word":
                figure(rig, arm_id, ms)
