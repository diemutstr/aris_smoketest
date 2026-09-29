"""Tests of the free-space planner (aris/free).

Quick set: `../.venv/bin/python -m pytest tests/test_free.py -m "not slow" -q` (under a minute).
The fixed test set (2 x 1 000 pairs, tests/data/free_cases_<arm>.npz, built by
tests/free_cases.py) runs in the slow set.  Run with -s to see the numbers.
"""
from __future__ import annotations

import hashlib
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import free_cases  # noqa: E402

from aris.free import Options, plan, plan_detailed, seed_of  # noqa: E402
from aris.free.bound import accel_bound  # noqa: E402
from aris.free.check import Checker  # noqa: E402
from aris.free.lift import lift_path, paper_plane  # noqa: E402
from aris.kernel import collide, retime  # noqa: E402
from aris.rig import Rig  # noqa: E402
from aris.types import Box, Motion, Obstacles, Refusal  # noqa: E402

DEPLOY = Path(__file__).resolve().parents[1]

# Solve-rate floors on the fixed set, reached 2026-09-29 (see docs/modules/free.md).
FLOOR = {31: 1000, 13: 1000}


@pytest.fixture(scope="module")
def rig():
    return Rig.load(free_cases.CONFIG)


@pytest.fixture(scope="module")
def scenes(rig):
    return {a: free_cases.scene(rig, a) for a in free_cases.ARMS}


@pytest.fixture(scope="module")
def cases():
    return {a: free_cases.load(a) for a in free_cases.ARMS}


def verify(arm, obs, rules, motion: Motion, q0, q1):
    """Everything test 1 demands of a returned motion, measured independently of the planner's
    own verdict: ends, rest, limits at 1 kHz, clearance at every 1 kHz sample and along it."""
    traj = motion.traj
    assert motion.kind == "free"
    assert np.max(np.abs(traj.q[0] - q0)) <= 1e-9 and np.max(np.abs(traj.q[-1] - q1)) <= 1e-9
    assert np.max(np.abs(traj.qd[0])) <= 1e-12 and np.max(np.abs(traj.qd[-1])) <= 1e-12
    rep = retime.check(traj, arm.limits)
    assert rep.inside, (rep.qd_ratio, rep.qdd_ratio, rep.qddd_ratio)
    g = rules.gates
    assert np.all(rep.q_low >= arm.limits.q_min + g.limit_margin)
    assert np.all(rep.q_high <= arm.limits.q_max - g.limit_margin)
    t = np.arange(0.0, traj.t[-1], 1e-3)
    Q = np.concatenate([retime.sample(traj, t)[0], traj.q[-1:]])
    tables = collide.arm_tables(arm)
    c = collide.clearance_q(tables, Q, obs)
    s = collide.self_clearance_q(tables, Q, arm.self_pairs, g.self_margin)
    path = collide.path_clearance_q(tables, arm.reach, Q, obs, tol=1e-4)
    assert c.min() >= 0.0 and s.min() >= 0.0 and path >= -1e-4
    return float(c.min()), float(s.min()), float(path)


def spread(n_per_group: int):
    """Case indices spread over the four groups of a fixed set."""
    return [g * free_cases.PER_GROUP + k * (free_cases.PER_GROUP // n_per_group)
            for g in range(4) for k in range(n_per_group)]


# --------------------------------------------------------------------------- 1. valid motions


def test_motions_are_free_timed_and_exact(scenes, cases):
    rows = []
    for a in (31, 13):
        arm, obs, rules = scenes[a]
        d = cases[a]
        for i in spread(2):
            res, st = plan_detailed(arm, d["q_start"][i], d["q_goal"][i], obs, rules)
            assert isinstance(res, Motion), (a, i, res)
            c, s, p = verify(arm, obs, rules, res, d["q_start"][i], d["q_goal"][i])
            rows.append((a, i, st.method, st.duration, c, s, p, st.cpu))
    print("\narm case method  duration  clearance at 1 kHz  self  path bound  cpu")
    for a, i, m, dur, c, s, p, cpu in rows:
        print(f"{a:3d} {i:4d} {m:8s} {dur:6.2f} s  {c * 1e3:6.2f} mm  {s * 1e3:6.1f} mm "
              f"{p * 1e3:6.2f} mm  {cpu * 1e3:5.0f} ms")
    assert {"straight", "tree"} <= {r[2] for r in rows}


def test_travel_bound_holds(scenes):
    """The first-order bound on how far a capsule moves is never exceeded (dense sampling)."""
    arm, obs, rules = scenes[31]
    ck = Checker(arm, obs, rules.gates)
    rng = np.random.default_rng(3)
    lim = arm.limits
    worst = 0.0
    for scale in (1.0, 0.1, 0.01):
        qa = rng.uniform(lim.q_min, lim.q_max, (100, 7))
        qb = np.clip(qa + rng.normal(size=(100, 7)) * scale, lim.q_min, lim.q_max)
        U = qb - qa
        va = ck.measure(qa, U)[2]
        vb = ck.measure(qb, U)[2]
        A = accel_bound(np.abs(U), ck.reach)
        Z = np.abs(U) @ ck.reach
        da, db = np.minimum(Z, va + A / 2), np.minimum(Z, vb + A / 2)
        s = np.linspace(0.0, 1.0, 101)
        b = arm.body((qa[:, None] + s[None, :, None] * U[:, None]).reshape(-1, 7))
        P0, P1 = b.p0.reshape(100, 101, -1, 3), b.p1.reshape(100, 101, -1, 3)
        move = lambda ref: np.maximum(np.linalg.norm(P0 - P0[:, ref], axis=-1),
                                      np.linalg.norm(P1 - P1[:, ref], axis=-1))
        ok_a = move([0]) <= da[:, None] * s[None, :, None] + 1e-12
        ok_b = move([-1]) <= db[:, None] * (1 - s)[None, :, None] + 1e-12
        assert ok_a.all() and ok_b.all()
        with np.errstate(divide="ignore", invalid="ignore"):
            worst = max(worst, np.nanmax(move([0])[:, 1:] / (da[:, None] * s[None, 1:, None])))
    print(f"\nlargest displacement / bound: {worst:.6f}")


def test_lift_keeps_the_hand_and_rises_along_the_normal(scenes, cases):
    arm, obs, rules = scenes[31]
    paper = paper_plane(obs)
    done = 0
    for q in cases[31]["q_start"][:20]:
        path = lift_path(arm, q, paper, 0.06)
        if path is None:
            continue
        done += 1
        T = arm.fk(path)
        assert np.max(np.abs(T[:, :3, :3] - T[0, :3, :3])) < 1e-6
        rise = arm.tip(path) - arm.tip(q[None])
        assert np.max(np.linalg.norm(np.cross(rise, paper.normal), axis=1)) < 1e-6
        height = arm.tip(path[-1:])[0] @ paper.normal - paper.offset
        assert abs(height - 0.06) < 1e-6
        assert np.max(np.abs(np.diff(path, axis=0))) < 0.1
        assert np.all(path[:, 6] == q[6])
    assert done >= 15


# --------------------------------------------------------------------------- 2. determinism


_DIGEST_SCRIPT = """
import sys, hashlib
sys.path.insert(0, {tests!r})
import free_cases
from aris.free import plan
from aris.rig import Rig
arm, obs, rules = free_cases.scene(Rig.load(free_cases.CONFIG), {arm})
d = free_cases.load({arm})
m = plan(arm, d["q_start"][{i}], d["q_goal"][{i}], obs, rules)
h = hashlib.sha256()
for a in (m.traj.t, m.traj.q, m.traj.qd):
    h.update(a.tobytes())
print(h.hexdigest())
"""


def _digest(m: Motion) -> str:
    h = hashlib.sha256()
    for a in (m.traj.t, m.traj.q, m.traj.qd):
        h.update(a.tobytes())
    return h.hexdigest()


def test_same_question_same_answer_in_any_process(scenes, cases):
    arm, obs, rules = scenes[31]
    d = cases[31]
    i = next(i for i in spread(4) if plan_detailed(arm, d["q_start"][i], d["q_goal"][i], obs,
                                                   rules)[1].rounds > 0)
    first = plan(arm, d["q_start"][i], d["q_goal"][i], obs, rules)
    again = plan(arm, d["q_start"][i], d["q_goal"][i], obs, rules)
    assert _digest(first) == _digest(again)
    env = dict(os.environ, PYTHONHASHSEED="12345")
    out = subprocess.run([sys.executable, "-c", _DIGEST_SCRIPT.format(
        tests=str(DEPLOY / "tests"), arm=31, i=i)], env=env, capture_output=True, text=True,
        cwd=DEPLOY, check=True)
    assert out.stdout.strip() == _digest(first)
    other = plan(arm, d["q_start"][i], d["q_goal"][i], obs, rules, seed_extra=b"another")
    assert seed_of(d["q_start"][i], d["q_goal"][i], obs) != seed_of(
        d["q_start"][i], d["q_goal"][i], obs, b"another")
    assert isinstance(other, Motion)


# --------------------------------------------------------------------------- 3. refusals


def fence(gap: float = 0.2, z_split: float = 0.5, margin: float = 0.05) -> tuple[Box, ...]:
    """A wall in the arm's base x-z plane (y_base = 0), from the mounting plate to the paper.

    Below z_split (base z points down, the paper is at 0.97) it is whole, across the arm's axis.
    Above, it leaves a gap of `gap` either side of the axis for the shoulder, which turns there.
    To get from one side to the other the elbow, 0.316 m below the shoulder (at z 0.333), has
    to cross the plane: within the gap it is below z_split, and above z_split it is outside the
    gap.  No path with 50 times the search cap (1 000 000 configurations) found a way."""
    boxes = []
    for sgn in (1.0, -1.0):
        T = np.eye(4)
        T[:3, 3] = [sgn * (gap + 0.7), 0.0, z_split / 2]
        boxes.append(Box(f"fence_upper{int(sgn)}", T, np.array([0.7, 0.005, z_split / 2]), margin))
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, (z_split + 0.96) / 2]
    boxes.append(Box("fence_lower", T, np.array([1.5, 0.005, (0.96 - z_split) / 2]), margin))
    return tuple(boxes)


def test_refusals(rig, scenes, cases):
    arm, obs, rules = scenes[31]
    d = cases[31]
    q0, q1 = d["q_start"][0], d["q_goal"][0]

    # the start inside an obstacle: a box around the pen holder
    T = np.eye(4)
    T[:3, 3] = arm.tip(q0[None])[0]
    trap = Obstacles(obs.boxes + (Box("block", T, np.full(3, 0.02), 0.05),), obs.planes,
                     obs.capsules)
    r = plan(arm, q0, q1, trap, rules)
    assert isinstance(r, Refusal) and r.reason == "blocked"
    assert r.detail.startswith("start") and "block" in r.detail

    # the goal outside the joint limits (inside them, but closer than the gate's margin)
    bad = q1.copy()
    bad[3] = arm.limits.q_max[3] - 0.05
    r = plan(arm, q0, bad, obs, rules)
    assert isinstance(r, Refusal) and r.reason == "outside_limits"
    assert r.detail.startswith("goal: joint 4")

    # not seven finite numbers
    r = plan(arm, q0, np.full(7, np.nan), obs, rules)
    assert isinstance(r, Refusal) and r.reason == "bad_input"

    # two ends on either side of a wall with no way round
    walled = Obstacles(rig.obstacles(31).boxes + fence(), rig.obstacles(31).planes)
    q, _, _ = free_cases.pool(rig, 31, 7)
    q = q[collide.clearance_q(collide.arm_tables(arm), q, walled) >= 0.0]
    side = arm.tip(q)[:, 1]
    a, b = q[side < -0.2][0], q[side > 0.2][0]
    c0 = time.process_time()
    r = plan(arm, a, b, walled, rules, options=Options(max_checks=3000))
    print(f"\nwalled: {r.reason}: {r.detail} ({time.process_time() - c0:.2f} s cpu)")
    assert isinstance(r, Refusal) and r.reason == "no_free_path"


# --------------------------------------------------------------------------- 4. the fixed set

_scene_cache = {}


def _one(job):
    arm_id, i = job
    if arm_id not in _scene_cache:
        _scene_cache[arm_id] = free_cases.scene(Rig.load(free_cases.CONFIG), arm_id)
    arm, obs, rules = _scene_cache[arm_id]
    d = free_cases.load(arm_id)
    res, st = plan_detailed(arm, d["q_start"][i], d["q_goal"][i], obs, rules)
    ok = isinstance(res, Motion)
    flown = verify(arm, obs, rules, res, d["q_start"][i], d["q_goal"][i]) if ok else None
    return arm_id, i, ok, None if ok else res.reason, st.cpu, st.method, flown


@pytest.mark.slow
def test_fixed_set_solve_rate():
    jobs = [(a, i) for a in free_cases.ARMS for i in range(2 * 2 * free_cases.PER_GROUP)]
    with ProcessPoolExecutor(min(24, os.cpu_count() or 1)) as ex:
        rows = list(ex.map(_one, jobs, chunksize=8))
    for a in free_cases.ARMS:
        mine = [r for r in rows if r[0] == a]
        solved = sum(r[2] for r in mine)
        cpu = np.array([r[4] for r in mine if r[2]])
        print(f"\narm {a}: solved {solved}/{len(mine)}, straight "
              f"{sum(r[5] == 'straight' for r in mine if r[2])}, cpu per plan median "
              f"{np.median(cpu) * 1e3:.0f} ms, p95 {np.percentile(cpu, 95) * 1e3:.0f} ms, "
              f"worst {cpu.max() * 1e3:.0f} ms; refusals "
              f"{sorted({r[3] for r in mine if not r[2]})}")
        assert solved >= FLOOR[a]
        assert np.median(cpu) < 10 * 0.21          # generous: ten times the measured median
