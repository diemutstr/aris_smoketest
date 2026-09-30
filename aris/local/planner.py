"""The local planner: lines on the paper -> how one arm draws each of them.

    plan(arm, lines, obstacles, rules, gates, workers=1) -> (list[Bunch], list[Leftover])

Per line (lines do not depend on each other):
  1. the graph, narrow: zero lean only                                   lattice.py
  2. the sweep: cheapest route along the whole line, lifts and gaps priced  search.py
  3. where that route lifts or leaves a gap, open the lean and sweep again
  4. every piece of the route re-solved densely and verified; a failure bans
     the nodes around it and the sweep runs again                         backout.py
  5. per piece, the best route of each family (slot, spin at both ends),
     re-solved, verified and timed: the alternatives                      pieces.py
  6. the undrawn stretches become leftovers, with the reason              pieces.py
See docs/modules/local.md.
"""
from __future__ import annotations

import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from multiprocessing import get_context

import numpy as np

from aris.kernel.retime import retime_detailed
from aris.local import polyline
from aris.local.backout import Dense, Verdict, dense_path, reach_out, verify
from aris.local.gates import Judge
from aris.local.lattice import Lattice
from aris.local.pieces import REPAIRED, families, gap_leftovers
from aris.local.search import Route, best_route, piece_routes
from aris.local.settings import Settings
from aris.local.table import load_or_build
from aris.types import Bunch, DrawPlan, DrawRules, Gates, JointPath, Leftover, Line, Obstacles, \
    Piece, Refusal


@dataclass
class LineStats:
    """What planning one line took, for reports."""
    line_id: str
    length: float = 0.0
    layers: int = 0
    nodes: int = 0                   # stored nodes (inside the gates without obstacles)
    ik_poses: int = 0                # poses handed to the IK, graph and exact paths
    lookups: int = 0                 # graph nodes read from the kinematic table
    checked: int = 0                 # nodes checked against the obstacles
    rounds: int = 0                  # searches over the whole line
    widened: int = 0                 # layers opened to the lean
    repairs: int = 0                 # sweeps run again after an exact path failed
    seconds: dict = field(default_factory=lambda: dict(graph=0.0, search=0.0, dense=0.0,
                                                       verify=0.0, timing=0.0))  # CPU time


class _Clock:
    def __init__(self, stats: LineStats):
        self.stats = stats

    def __call__(self, name, fn, *args):
        t = time.process_time()
        out = fn(*args)
        self.stats.seconds[name] += time.process_time() - t
        return out


def plan(arm, lines, obstacles: Obstacles, rules: DrawRules, gates: Gates | None = None,
         workers: int = 1, settings: Settings | None = None, cache_dir=None):
    """-> (bunches, leftovers).  Every line's pieces and leftovers cover it exactly once.

    With `workers` > 1 the lines are planned in that many new processes ("spawn"), with the
    same result; as always with spawned processes, the calling script needs its
    `if __name__ == "__main__":` guard.  With `cache_dir` the graph's nodes come from the
    kinematic table kept there (built on first use, table.py); without, they are solved.
    """
    bunches, leftovers, _ = plan_detailed(arm, lines, obstacles, rules, gates, workers, settings,
                                          cache_dir)
    return bunches, leftovers


def plan_detailed(arm, lines, obstacles, rules, gates=None, workers=1, settings=None,
                  cache_dir=None):
    """`plan`, plus one `LineStats` per line."""
    gates = rules.gates if gates is None else gates
    cfg = Settings() if settings is None else settings
    for line in lines:
        if line.frame != "base":
            raise ValueError(f"line {line.id} is in the {line.frame} frame; the local planner "
                             "takes base-frame lines")
    judge = Judge(arm, obstacles, gates)
    table = None if cache_dir is None else \
        load_or_build(arm, judge.paper, gates, rules.lean_max, cfg, cache_dir)
    jobs = [(arm, line, obstacles, rules, gates, cfg, cache_dir) for line in lines]
    if workers > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(workers, mp_context=get_context("spawn")) as pool:
            results = list(pool.map(_plan_job, jobs, chunksize=max(1, len(jobs) // (4 * workers))))
    else:
        results = [plan_line(arm, line, judge, rules, cfg, table) for line in lines]
    bunches = [b for r in results for b in r[0]]
    leftovers = [x for r in results for x in r[1]]
    return bunches, leftovers, [r[2] for r in results]


def _plan_job(job):
    arm, line, obstacles, rules, gates, cfg, cache_dir = job
    judge = Judge(arm, obstacles, gates)
    table = None if cache_dir is None else \
        load_or_build(arm, judge.paper, gates, rules.lean_max, cfg, cache_dir)
    return plan_line(arm, line, judge, rules, cfg, table)


class LinePool:
    """A pool of `workers` processes that keeps everything but the lines: the arm, the
    obstacles, the rules, the gates, the settings and the kinematic table are handed to each
    worker once, when it starts; a job is then a list of lines, by value.  For callers that
    plan many small groups of lines over time (the arm planner's batches).

        pool = LinePool(arm, obstacles, rules, gates, workers, settings, cache_dir)
        fut = pool.submit(lines)          # -> Future of (bunches, leftovers, stats)
        pool.close()

    The answer for a list of lines is the same as `plan_detailed` gives for it."""

    def __init__(self, arm, obstacles, rules, gates=None, workers: int = 1, settings=None,
                 cache_dir=None):
        gates = rules.gates if gates is None else gates
        cfg = Settings() if settings is None else settings
        self._pool = ProcessPoolExecutor(
            workers, mp_context=get_context("spawn"), initializer=_pool_start,
            initargs=(arm, obstacles, rules, gates, cfg, cache_dir))

    def submit(self, lines):
        for line in lines:
            if line.frame != "base":
                raise ValueError(f"line {line.id} is in the {line.frame} frame; the local "
                                 "planner takes base-frame lines")
        return self._pool.submit(_pool_job, list(lines))

    def close(self, cancel: bool = False) -> None:
        self._pool.shutdown(wait=True, cancel_futures=cancel)


# The one thing a pool worker keeps between jobs: what LinePool handed it when it started.
# It lives only in that worker process, is set once and never changes.
_WORKER: tuple | None = None


def _pool_start(arm, obstacles, rules, gates, cfg, cache_dir) -> None:
    global _WORKER
    judge = Judge(arm, obstacles, gates)
    table = None if cache_dir is None else \
        load_or_build(arm, judge.paper, gates, rules.lean_max, cfg, cache_dir)
    _WORKER = (arm, judge, rules, cfg, table)


def _pool_job(lines):
    arm, judge, rules, cfg, table = _WORKER
    results = [plan_line(arm, line, judge, rules, cfg, table) for line in lines]
    return ([b for r in results for b in r[0]], [x for r in results for x in r[1]],
            [r[2] for r in results])


# --------------------------------------------------------------------------- one line


def plan_line(arm, line: Line, judge: Judge, rules: DrawRules, cfg: Settings, table=None):
    """-> (bunches, leftovers, stats) for one base-frame line."""
    stats = LineStats(line.id)
    clock = _Clock(stats)
    points, s_points = polyline.clean(line.points)
    length = float(s_points[-1]) if len(points) else 0.0
    stats.length = length
    if len(points) < 2 or length < rules.min_piece:
        what = "an empty line" if length == 0.0 else f"{length * 1e3:.1f} mm long"
        return [], [Leftover(Piece(line.id, 0.0, length), "too_short", what)], stats
    s = polyline.layer_positions(length, cfg.step)
    lat = clock("graph", Lattice, arm, judge, polyline.at(points, s_points, s), s,
                rules.lean_max, cfg, table)
    route = _search(lat, cfg, clock, stats)
    for _ in range(3):                                   # open the lean where the plan fails
        wanted = _trouble(route, lat, cfg)
        if not len(wanted) or len(lat.leans) == 1:
            break
        opened = clock("graph", lat.widen, wanted)
        stats.widened += opened
        if not opened:
            break
        route = _search(lat, cfg, clock, stats)
    route, checked, lost = _repair(lat, judge, route, points, s_points, cfg, clock, stats)
    bunches, leftovers = [], []
    gap_from, gap_to = {}, {}                  # where the undrawn stretches really begin and end
    for run in route.runs:
        ends = clock("dense", _extend, lat, judge, route, run, checked, points, s_points, cfg,
                     max(s[max(run.k0 - 1, 0)], gap_from.get(run.k0 - 1, 0.0)))
        gap_to[run.k0], gap_from[run.k1] = ends
        piece = Piece(line.id, *ends)
        if piece.s1 - piece.s0 < rules.min_piece:
            leftovers.append(Leftover(piece, "too_short", f"{(piece.s1 - piece.s0) * 1e3:.1f} mm "
                                      "between two lifts"))
            continue
        plans, why = _alternatives(lat, judge, run, piece, checked, points, s_points, rules,
                                   cfg, clock, stats)
        if plans:
            bunches.append(Bunch(piece, tuple(plans)))
        else:
            leftovers.append(Leftover(piece, "unreachable", f"cannot be timed: {why}"))
    # the reason of an undrawn stretch ("blocked by ...") needs its nodes checked
    clock("graph", lat.check_layers, sorted({k for a, b in route.gaps for k in range(a, b + 1)}))
    leftovers += gap_leftovers(lat, judge, line.id, route.gaps, gap_from, gap_to)
    leftovers += [Leftover(Piece(line.id, float(s[k0]), float(s[k1])), *REPAIRED)
                  for k0, k1 in lost]
    leftovers.sort(key=lambda x: x.piece.s0)
    stats.layers = lat.n_layers
    stats.nodes = int(sum(len(L.q) for L in lat.layers))
    stats.ik_poses += lat.ik_poses
    stats.lookups = lat.lookups
    stats.checked = lat.checked
    return bunches, leftovers, stats


def _search(lat: Lattice, cfg: Settings, clock, stats) -> Route:
    """The cheapest route whose nodes are all free of the obstacles.

    Lazily (`cfg.lazy`) the graph's nodes count as free until checked; so the route is found,
    its nodes and their neighbours are checked in one batch, and if any of the route's own
    nodes is not free the search runs again.  After `lazy_rounds` searches every surviving node
    of the line is checked, as in the eager planner."""
    for _ in range(cfg.lazy_rounds if cfg.lazy else 1):
        route = clock("search", best_route, lat, cfg.lift_cost, cfg.gap_cost)
        stats.rounds += 1
        if not cfg.lazy:
            return route
        picks = [(r.k0 + i, lat.band(r.k0 + i, [n])) for r in route.runs
                 for i, n in enumerate(r.nodes)]
        clock("graph", lat.check, picks)
        if all(lat.layers[r.k0 + i].free[n] for r in route.runs for i, n in enumerate(r.nodes)):
            return route
    clock("graph", lat.check_layers, range(lat.n_layers))
    stats.rounds += 1
    return clock("search", best_route, lat, cfg.lift_cost, cfg.gap_cost)


def _trouble(route: Route, lat: Lattice, cfg: Settings) -> np.ndarray:
    """Layers around every lift and every gap of the route, not yet open to the lean."""
    spots = [r.k0 for r in route.runs[1:] if any(q.k1 == r.k0 for q in route.runs)]
    spans = [(a, b) for a, b in route.gaps] + [(k, k) for k in spots]
    reach = int(np.ceil(cfg.collar / cfg.step))
    want = set()
    for a, b in spans:
        want.update(range(max(0, a - reach), min(lat.n_layers, b + reach + 1)))
    return np.array(sorted(k for k in want if not lat.opened[k].all()), int)


def _repair(lat, judge, route, points, s_points, cfg, clock, stats):
    """Re-solve every piece densely; ban around a failure and sweep again until all pass.

    -> the final route (only verified runs), the verified exact paths by (k0, nodes), and the
    stretches given up after `max_repairs` sweeps."""
    checked: dict = {}
    for attempt in range(cfg.max_repairs + 1):
        bad = []
        for run in route.runs:
            key = (run.k0, run.nodes.tobytes())
            if key not in checked:
                d, v, poses, smooth = _exact(clock, lat, judge, run.k0, run.nodes, points,
                                             s_points, cfg)
                stats.ik_poses += poses
                if v.ok:
                    checked[key] = (d, v, smooth)
                else:
                    bad.append((run, d, v))
        if not bad or attempt == cfg.max_repairs:
            break
        for run, d, v in bad:
            k = int(d.layer[v.where])
            for kk in (k, k + 1):
                if run.k0 <= kk <= run.k1:
                    lat.ban(kk, [run.nodes[kk - run.k0]])
        stats.repairs += 1
        route = _search(lat, cfg, clock, stats)
    lost = [(r.k0, r.k1) for r, _, _ in bad]
    good = tuple(r for r in route.runs if (r.k0, r.k1) not in lost)
    return Route(good, route.gaps, route.cost), checked, lost


def _exact(clock, lat, judge, k0, nodes, points, s_points, cfg, ends=None):
    """The exact path along a route, verified: the averaged curve first, then the curve
    through every node.  -> (path, verdict, IK poses, smoothing used); the last try's if
    neither passes."""
    poses = 0
    for smooth in (cfg.smooth_layers, 0.0) if cfg.smooth_layers > 0 else (0.0,):
        d = clock("dense", dense_path, lat, k0, nodes, points, s_points, cfg, smooth, ends)
        poses += d.ik_poses
        v = clock("verify", verify, judge, d.q, d.tip, cfg)
        if v.ok:
            break
    return d, v, poses, smooth


def _extend(lat, judge, route, run, checked, points, s_points, cfg, before_limit):
    """Where the piece of `run` really starts and ends.

    Next to an undrawn stretch the route stops at a layer, but the arm can usually go a little
    further: the exact solve walks on past the end layer (and back before the first) until a
    gate or the clearance stops it, never into the neighbouring piece.  The longer piece is
    kept only if its exact path passes `verify`; its path is stored in `checked`.
    -> (s0, s1)."""
    s = lat.s
    d, v, smooth = checked[(run.k0, run.nodes.tobytes())]
    starts = {r.k0 for r in route.runs}
    stops = {r.k1 for r in route.runs}
    e0, e1 = float(s[run.k0]), float(s[run.k1])
    if run.k0 > 0 and run.k0 not in stops:
        e0 = reach_out(lat, judge, run.k0, run.nodes, points, s_points, cfg, smooth, False,
                       float(before_limit))
    if run.k1 < lat.n_layers - 1 and run.k1 not in starts:
        e1 = reach_out(lat, judge, run.k0, run.nodes, points, s_points, cfg, smooth, True,
                       float(s[run.k1 + 1]))
    if (e0, e1) == (float(s[run.k0]), float(s[run.k1])):
        return e0, e1
    d2, v2, _, sm2 = _exact(lambda n, f, *a: f(*a), lat, judge, run.k0, run.nodes, points,
                            s_points, cfg, (e0, e1))
    if not v2.ok:
        return float(s[run.k0]), float(s[run.k1])
    checked[(run.k0, run.nodes.tobytes(), e0, e1)] = (d2, v2, sm2)
    return e0, e1


def _family(lat, k0, k1, first, last, sectors):
    A, B = lat.layers[k0], lat.layers[k1]
    n = len(lat.spins)
    return (int(B.slot[last]), int(A.spin[first]) * sectors // n,
            int(B.spin[last]) * sectors // n)


def _alternatives(lat, judge, run, piece, checked, points, s_points, rules, cfg, clock,
                  stats):
    """Up to `n_alternatives` verified, timed plans for one piece: the best route of each
    family, cheapest family first, each starting or ending at least `distinct` away (in some
    joint) from every plan already taken."""
    for _ in range(cfg.lazy_rounds if cfg.lazy else 1):
        sweep = clock("search", piece_routes, lat, run.k0, run.k1)
        ends = families(lat, sweep, cfg.spin_sectors)[: 3 * cfg.n_alternatives]
        if not cfg.lazy:
            break
        routes = [sweep.route(e) for e in ends]
        picks = [(run.k0 + i, lat.band(run.k0 + i, [r[i] for r in routes]))
                 for i in range(run.k1 - run.k0 + 1)]
        clock("graph", lat.check, picks)
        if all(lat.layers[run.k0 + i].free[r[i]] for r in routes for i in range(len(r))):
            break
    else:
        clock("graph", lat.check_layers, range(run.k0, run.k1 + 1))
        sweep = clock("search", piece_routes, lat, run.k0, run.k1)
        ends = families(lat, sweep, cfg.spin_sectors)
    primary = _family(lat, run.k0, run.k1, run.nodes[0], run.nodes[-1], cfg.spin_sectors)
    extent = (piece.s0, piece.s1)
    layers = (float(lat.s[run.k0]), float(lat.s[run.k1]))

    def key_of(nodes):
        return (run.k0, nodes.tobytes()) + (() if extent == layers else extent)

    plans, why = [], "no route"
    for end in ends[: 3 * cfg.n_alternatives]:
        nodes = sweep.route(end)
        fam = _family(lat, run.k0, run.k1, nodes[0], nodes[-1], cfg.spin_sectors)
        key = key_of(nodes)
        if key not in checked:
            d, v, poses, smooth = _exact(clock, lat, judge, run.k0, nodes, points, s_points,
                                         cfg, None if extent == layers else extent)
            stats.ik_poses += poses
            if v.ok:
                checked[key] = (d, v, smooth)
            elif fam == primary:
                key = key_of(run.nodes)
            else:
                continue
        d, v, _ = checked[key]
        if not _distinct(d.q, plans, cfg.distinct):
            continue
        timed = clock("timing", _timed_plan, lat.arm, judge, piece, d, v, rules, cfg)
        if isinstance(timed, str):
            why = timed
            continue
        plans.append(timed)
        if len(plans) == cfg.n_alternatives:
            break
    primary_key = key_of(run.nodes)
    if not plans and primary_key in checked:
        timed = clock("timing", _timed_plan, lat.arm, judge, piece, *checked[primary_key][:2],
                      rules, cfg)
        if isinstance(timed, str):
            why = timed
        else:
            plans.append(timed)
    return plans, why


def _distinct(q: np.ndarray, plans, gap: float) -> bool:
    return all(max(np.abs(q[0] - p.q_start).max(), np.abs(q[-1] - p.q_end).max()) > gap
               for p in plans)


def _timed_plan(arm, judge: Judge, piece: Piece, d: Dense, v: Verdict, rules: DrawRules,
                cfg: Settings):
    """A DrawPlan, timed in one call to the timing step; or the reason it cannot be flown.

    The pen tip of the flown path stays within `tip_budget` of the line, and the flown joint
    path within `timing_deviation` of this one; `verify` already demanded the clearance that
    covers this.
    """
    res = retime_detailed(JointPath(d.q), arm.limits, rules, s=d.s,
                          deviation=cfg.timing_deviation, tip_budget_m=cfg.tip_budget,
                          tip_of=arm.tip)
    if isinstance(res, Refusal):
        return f"{res.reason}: {res.detail}"
    # The flown path stays within `deviation` (joint space) of this one, so its clearance is
    # at most `lipschitz * deviation` below the clearance measured along this path.
    flown = v.clearance - judge.lipschitz * res.deviation
    if flown < 0.0:
        return f"the timed path comes {-flown * 1e3:.2f} mm too close"
    travel = float(np.abs(np.diff(d.q, axis=0)).sum())
    return DrawPlan(piece=piece, q=d.q, s=d.s, tip_base=arm.tip(d.q), score=float(flown),
                    joint_travel=travel, draw_time=float(res.traj.t[-1]), spin=d.spin,
                    lean=d.lean)


# --------------------------------------------------------------------------- from outside


def verify_plan(arm, plan: DrawPlan, line: Line, obstacles: Obstacles, gates: Gates,
                settings: Settings | None = None) -> Verdict:
    """Check a returned plan (either direction) against its line from its joint samples alone."""
    cfg = Settings() if settings is None else settings
    judge = Judge(arm, obstacles, gates)
    points, s_points = polyline.clean(line.points)
    return verify(judge, plan.q, polyline.at(points, s_points, plan.s), cfg)


def reverse_plan(plan: DrawPlan) -> DrawPlan:
    """The same plan drawn from its other end.  `s` still says where on the line each sample is."""
    return DrawPlan(piece=plan.piece, q=plan.q[::-1].copy(), s=plan.s[::-1].copy(),
                    tip_base=plan.tip_base[::-1].copy(), score=plan.score,
                    joint_travel=plan.joint_travel, draw_time=plan.draw_time,
                    spin=None if plan.spin is None else plan.spin[::-1].copy(),
                    lean=None if plan.lean is None else plan.lean[::-1].copy())
