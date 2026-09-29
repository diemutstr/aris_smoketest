"""The free-space planner: one arm, pen up, from one configuration to another without hitting
anything.  See docs/modules/free.md.

    plan(arm, q_start, q_goal, obstacles, rules, gates=None, seed_extra=b"") -> Motion | Refusal

Everything is in the arm's base frame; obstacles arrive as geometry.  Seven steps:
1. refuse at once if an end is outside the joint limits (with the gate's margin) or not free
2. try the straight joint-space move
3. raise both ends off the paper (an attempt: an end whose lift is not free stays where it is)
4. bidirectional RRT between the (raised) ends, capped by a count of configurations checked
5. shorten, checking every replacement
6. time it, with a corner-rounding budget the path's clearance can pay for
7. check what is flown: the timed trajectory's own samples, obstacles, itself and the limits
"""
from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass, field

import numpy as np

from aris.free.check import Checker
from aris.free.flown import flown_verdict
from aris.free.lift import paper_plane, raise_end
from aris.free.rrt import connect
from aris.free.shortcut import length, shorten
from aris.kernel import collide
from aris.kernel.retime import retime_detailed
from aris.types import DrawRules, Gates, JointPath, Motion, Obstacles, Refusal, Trajectory


@dataclass(frozen=True)
class Options:
    lift_to: float | None = 0.06   # m above the paper for the raised ends; None: no lift
    max_edges: int = 20_000         # straight edges the tree search may check (its cap)
    step: float = 1.5              # rad, the tree's step
    batch: int = 4                 # random targets per tree round
    run_step: float = np.inf       # rad, pieces in which a tree runs at the other's new node
                                   # (inf: one straight edge, all or nothing)
    shortcut_rounds: int = 1
    budget_max: float = 0.03       # rad, the largest corner-rounding budget tried
    budget_min: float = 1.5e-4     # rad, retime's default; below it we give up
    corner_rate: float = 1.3       # m/rad, for the first corner-rounding guess (see below)
    backend: str | None = None     # collision engine, "native" or "numpy"; None: the kernel's
    pad: float = 0.002             # m of clearance the search keeps beyond the margins, where
                                   # the ends have it (at most half of theirs)


@dataclass
class Stats:
    """What one plan did.  Not part of the answer; for measurement."""
    method: str = ""               # "straight", "lifted" (up, across, down), "tree"; "" if refused
    lifted: tuple = (False, False)
    t_search: float = 0.0          # s, steps 1-4
    t_shorten: float = 0.0
    t_time: float = 0.0
    t_check: float = 0.0
    cpu: float = 0.0               # s of this process's CPU time, all steps
    checked: int = 0               # straight edges checked, all steps
    checked_search: int = 0        # of which by steps 1-4
    checked_shorten: int = 0       # of which by step 5
    rounds: int = 0                # tree rounds
    length: float = 0.0            # rad, joint-space length of the timed path
    straight: float = 0.0          # rad, |q_goal - q_start|
    duration: float = 0.0          # s
    budget: float = 0.0            # rad
    clearance: tuple = ()          # (clearance of the flown trajectory, m,): obstacles and self
    notes: list = field(default_factory=list)


def seed_of(q_start, q_goal, obstacles: Obstacles, seed_extra: bytes = b"") -> int:
    """64-bit seed from the bytes of the question (never Python's hash())."""
    h = hashlib.blake2b(digest_size=8)
    for a in (q_start, q_goal):
        h.update(np.ascontiguousarray(a, np.float64).tobytes())
    P = collide.pack(obstacles)
    for a in P.scene:
        h.update(np.ascontiguousarray(a).tobytes())
    h.update(seed_extra)
    return int.from_bytes(h.digest(), "little")


def plan(arm, q_start, q_goal, obstacles: Obstacles, rules: DrawRules, gates: Gates | None = None,
         seed_extra: bytes = b"", options: Options = Options()) -> Motion | Refusal:
    return plan_detailed(arm, q_start, q_goal, obstacles, rules, gates, seed_extra, options)[0]


def plan_detailed(arm, q_start, q_goal, obstacles: Obstacles, rules: DrawRules,
                  gates: Gates | None = None, seed_extra: bytes = b"",
                  options: Options = Options()) -> tuple[Motion | Refusal, Stats]:
    """`plan`, plus what it did (`Stats`: times, counts, lengths)."""
    st = Stats()
    c0 = time.process_time()
    result = _plan(arm, q_start, q_goal, obstacles, rules, gates, seed_extra, options, st)
    st.cpu = time.process_time() - c0
    return result, st


def _plan(arm, q_start, q_goal, obstacles, rules, gates, seed_extra, options, st):
    t0 = time.perf_counter()
    gates = rules.gates if gates is None else gates
    q_start = np.asarray(q_start, float).reshape(-1)
    q_goal = np.asarray(q_goal, float).reshape(-1)
    if q_start.shape != (7,) or q_goal.shape != (7,) or not np.all(np.isfinite([q_start, q_goal])):
        return Refusal("bad_input", "q_start and q_goal must be 7 finite joint angles")
    checker = Checker(arm, obstacles, gates, backend=options.backend)
    st.straight = float(np.linalg.norm(q_goal - q_start))
    bad, c_ends, s_ends = _ends_refusal(arm, checker, gates, q_start, q_goal)
    if bad is not None:
        st.t_search = time.perf_counter() - t0
        return bad
    if st.straight < 1e-12:
        traj = Trajectory(np.zeros(1), q_start[None].copy(), np.zeros((1, 7)))
        st.method = "straight"
        return Motion("free", traj)
    rng = np.random.default_rng(seed_of(q_start, q_goal, obstacles, seed_extra))
    room = min(float(c_ends.min()), float(s_ends.min()))
    checker.set_need(min(options.pad, 0.5 * float(c_ends.min())),
                     min(options.pad, 0.5 * float(s_ends.min())), min(5e-4, 0.25 * room))

    path, head, tail = _search(arm, checker, q_start, q_goal, obstacles, rng, options, st)
    st.t_search = time.perf_counter() - t0
    st.checked = st.checked_search = checker.n_edges
    if path is None:
        return Refusal("no_free_path", f"no free path found within {options.max_edges} "
                       f"edges checked ({st.rounds} tree rounds)")
    t1 = time.perf_counter()
    if len(path) > 2:
        path = shorten(checker, path, rng, options.shortcut_rounds, head, tail)
    st.t_shorten = time.perf_counter() - t1
    st.checked_shorten = checker.n_edges - st.checked_search
    result = _time_and_verify(checker, arm, path, rules, gates, options, st)
    st.checked = checker.n_edges
    return result


def _ends_refusal(arm, checker, gates, q_start, q_goal):
    """-> (Refusal or None, obstacle clearance of both ends, self clearance of both ends)."""
    Q = np.stack([q_start, q_goal])
    margin = arm.limit_margin(Q)
    for name, i in (("start", 0), ("goal", 1)):
        if margin[i] < gates.limit_margin:
            j = int(np.argmin(np.minimum(Q[i] - arm.limits.q_min, arm.limits.q_max - Q[i])))
            return Refusal("outside_limits", f"{name}: joint {j + 1} is {margin[i]:.4f} rad from "
                           f"its limit, the gate asks {gates.limit_margin} rad"), None, None
    det, own = checker.ends(Q)
    for name, i in (("start", 0), ("goal", 1)):
        if det.value[i] < 0.0:
            return Refusal("blocked", f"{name}: {det.capsule_names[det.capsule[i]]} is "
                           f"{-det.value[i] * 1e3:.1f} mm inside the margin of "
                           f"{det.obstacle_names[det.obstacle[i]]}"), None, None
        if own[i] < 0.0:
            return Refusal("self_collision", f"{name}: the arm is {-own[i] * 1e3:.1f} mm inside "
                           f"its own self margin {gates.self_margin} m"), None, None
    return None, det.value, own


def _search(arm, checker, q_start, q_goal, obstacles, rng, options, st):
    """Steps 2-4: a free piecewise-straight joint path from q_start to q_goal, or None."""
    if checker.edges(q_start[None], q_goal[None])[0]:
        st.method = "straight"
        return np.stack([q_start, q_goal]), 0, 0
    paper = paper_plane(obstacles)
    up_s = raise_end(arm, checker, q_start, paper, options.lift_to)
    up_g = raise_end(arm, checker, q_goal, paper, options.lift_to)
    st.lifted = (up_s is not None, up_g is not None)
    head = up_s if up_s is not None else q_start[None]
    tail = (up_g if up_g is not None else q_goal[None])[::-1]
    ends = len(head) - 1, len(tail) - 1            # lift pieces at either end
    if (up_s is not None or up_g is not None) and checker.edges(head[-1:], tail[:1])[0]:
        st.method = "lifted"                       # up, straight across, down: no tree needed
        return np.concatenate([head, tail]), *ends
    st.method = "tree"
    found = connect(checker, head[-1], tail[0], rng, options.max_edges, options.step,
                    options.batch, options.run_step)
    st.rounds = found.rounds
    if found.path is None:
        return None, 0, 0
    return np.concatenate([head[:-1], found.path, tail[1:]]), *ends


def _time_and_verify(checker, arm, path, rules, gates, options, st) -> Motion | Refusal:
    """Steps 6-7.  Timing rounds the corners; a joint-space deviation d moves every point of the
    arm by at most |reach column| x d.  The first budget is a guess: the clearance at the
    path's corners over `corner_rate`, a typical rather than the worst lever.  If the flown
    trajectory fails, the next is what the whole path's lower bound provably pays for, then
    halving.  Each is kept only if the flown trajectory passes `flown_verdict`."""
    r_obst = float(np.max(np.linalg.norm(checker.reach, axis=0)))
    r_self = 2.0 * r_obst                           # a pair closes by both capsules' moves
    pays = lambda o, s: 0.9 * max(min(o / r_obst, s / r_self), 0.0)
    if len(path) > 2:
        det, own = checker.ends(path[1:-1])
        first = 0.9 * max(min(float(det.value.min()), float(own.min())), 0.0) / options.corner_rate
    else:
        first = options.budget_max                  # nothing to round
    budgets = [float(np.clip(first, options.budget_min, options.budget_max))]
    why = "not tried"
    while budgets:
        budget = budgets.pop(0)
        t = time.perf_counter()
        res = retime_detailed(JointPath(path), arm.limits, rules, deviation=budget)
        st.t_time += time.perf_counter() - t
        if isinstance(res, Refusal):
            why = f"retime: {res.reason} {res.detail}"
        else:
            t = time.perf_counter()
            why, st.clearance = flown_verdict(checker, arm, res, gates, path)
            st.t_check += time.perf_counter() - t
            if why is None:
                st.budget, st.duration = budget, float(res.traj.t[-1])
                st.length = length(path)
                return Motion("free", res.traj)
        st.notes.append(f"budget {budget:.2e}: {why}")
        if not budgets and budget > options.budget_min:
            t = time.perf_counter()
            flat = float(np.min(checker.edge_margins(path[:-1], path[1:])))
            safe = pays(flat, flat)
            st.t_check += time.perf_counter() - t
            budgets = [max(options.budget_min, min(0.5 * budget, safe))]
    return Refusal("cannot_time", f"no timing whose flown path is free: {why}")
