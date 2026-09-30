"""Tests of the local planner (aris/local).  See docs/modules/local.md.

The fixed set (tests/local_cases.py) is planned once per arm with every core; that takes a few
minutes.  `-k "not fixed_set"` runs the rest, in seconds.  The acceptance numbers themselves:
`../.venv/bin/python tests/local_cases.py`.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from aris.local import plan, plan_detailed, reverse_plan, verify_plan
from aris.rig import Rig
from aris.types import Box, Line, Obstacles

sys.path.insert(0, str(Path(__file__).resolve().parent))
import local_cases as lc                                    # noqa: E402

DEPLOY = Path(__file__).resolve().parents[1]
RIG = Rig.load(DEPLOY / "config")
WORKERS = max(1, min(32, (os.cpu_count() or 2) - 2))

# Floors: the shares measured on the fixed set (docs/modules/local.md), rounded down.  A
# change that draws less than this is a regression.
# Measured 2026-09-30 at the rig's gates (sigma_min 0.04), real obstacles, no table: drawn
# 0.935 / 0.894 / 0.951 / 0.822 for arm 31 and 0.935 / 0.954 / 0.955 / 0.877 for arm 13; length
# drawn whole in one piece 0.918 / 0.267 / 0.715 / 0.595 and 0.918 / 0.797 / 0.754 / 0.700.
FLOOR_DRAWN = {(31, "word"): 0.93, (31, "corpus"): 0.89, (31, "lines"): 0.95, (31, "curves"): 0.82,
               (13, "word"): 0.93, (13, "corpus"): 0.95, (13, "lines"): 0.95, (13, "curves"): 0.87}
FLOOR_SINGLE = {(31, "word"): 0.91, (31, "corpus"): 0.26, (31, "lines"): 0.71, (31, "curves"): 0.59,
                (13, "word"): 0.91, (13, "corpus"): 0.79, (13, "lines"): 0.75, (13, "curves"): 0.69}


def _problem(arm_id=31):
    return lc.problem(RIG, arm_id)


def _base(arm_id, xy, lid="t"):
    xy = np.asarray(xy, float).reshape(-1, 2)
    return RIG.to_base(arm_id, Line(lid, np.column_stack([xy, np.zeros(len(xy))]), "table"))


def covers_once(line: Line, bunches, leftovers) -> bool:
    """Pieces and leftovers of `line` tile [0, length] with no gap and no overlap."""
    p = np.asarray(line.points, float)
    length = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0
    spans = sorted([(b.piece.s0, b.piece.s1) for b in bunches if b.piece.line_id == line.id]
                   + [(x.piece.s0, x.piece.s1) for x in leftovers if x.piece.line_id == line.id])
    if not spans:
        return False
    ok = spans[0][0] == 0.0 and abs(spans[-1][1] - length) <= 1e-9
    return ok and all(a[1] == b[0] and a[0] <= a[1] for a, b in zip(spans, spans[1:]))


def _check_bunches(job):
    """Failures among these bunches' plans, each checked forwards and reversed."""
    arm_id, lines, bunches = job
    arm, obs, rules, gates = _problem(arm_id)
    by_id = {x.id: x for x in lines}
    bad = []
    for b in bunches:
        for i, p in enumerate(b.plans):
            for q in (p, reverse_plan(p)):
                v = verify_plan(arm, q, by_id[b.piece.line_id], obs, gates)
                if not v.ok:
                    bad.append((b.piece, i, v.reason))
            if not (p.draw_time > 0.0 and abs(p.s[0] - b.piece.s0) < 1e-12
                    and abs(p.s[-1] - b.piece.s1) < 1e-12):
                bad.append((b.piece, i, "ends or time"))
    return bad


def plans_check(arm_id, lines, bunches, workers=1):
    """Every plan and its reverse pass the verification from outside."""
    if workers == 1:
        return _check_bunches((arm_id, lines, bunches))
    from concurrent.futures import ProcessPoolExecutor
    from multiprocessing import get_context
    chunks = [bunches[i::4 * workers] for i in range(4 * workers)]
    with ProcessPoolExecutor(workers, mp_context=get_context("spawn")) as pool:
        out = pool.map(_check_bunches, [(arm_id, lines, c) for c in chunks if c])
    return [x for r in out for x in r]


# --------------------------------------------------------------------------- small, fast


class _Toy:
    """A hand-made graph for the search: `nodes[k]` usable flags, `edge[k]` a dict
    (node in k+1) -> [(node in k, cost)]."""

    def __init__(self, nodes, edge):
        self.nodes, self.edge = [np.asarray(n, bool) for n in nodes], edge
        self.n_layers = len(nodes)

    def usable(self, k):
        return self.nodes[k]

    def edges(self, k):
        n1 = len(self.nodes[k + 1])
        pred, cost = np.full((n1, 2), -1), np.full((n1, 2), np.inf)
        for j, lst in self.edge[k].items():
            for i, (p, c) in enumerate(lst):
                pred[j, i], cost[j, i] = p, c
        return pred, cost


def test_search_lifts_only_when_it_pays_and_leaves_gaps_where_it_must():
    from aris.local.search import best_route
    # two nodes per layer; staying on node 0 costs 1 per step except one step that costs 20
    nodes = [[1, 1]] * 4
    edge = [{0: [(0, 1.0)], 1: [(1, 1.0)]}, {0: [(0, 20.0)], 1: [(1, 1.0)]},
            {0: [(0, 1.0)], 1: [(1, 30.0)]}]
    r = best_route(_Toy(nodes, edge), lift_cost=5.0, gap_cost=1000.0)
    assert [(x.k0, x.k1) for x in r.runs] == [(0, 2), (2, 3)] and not r.gaps
    assert r.cost == 1.0 + 1.0 + 5.0 + 1.0
    r = best_route(_Toy(nodes, edge), lift_cost=50.0, gap_cost=1000.0)
    assert [(x.k0, x.k1) for x in r.runs] == [(0, 3)] and r.cost == 22.0
    # a layer with no usable node: the step on either side is left undrawn
    nodes = [[1, 1], [1, 1], [0, 0], [1, 1], [1, 1]]
    edge = [{0: [(0, 1.0)]}, {}, {}, {0: [(0, 1.0)]}]
    r = best_route(_Toy(nodes, edge), lift_cost=5.0, gap_cost=1000.0)
    assert [(x.k0, x.k1) for x in r.runs] == [(0, 1), (3, 4)] and r.gaps == ((1, 3),)


def test_degenerate_inputs_are_leftovers_with_reasons():
    arm, obs, rules, gates = _problem(31)
    axis = RIG.T_table_base(31)[:2, 3]
    strut = Box("test_strut", np.eye(4), np.array([0.03, 0.03, 0.2]), 0.05)
    T = np.eye(4)
    T[:3, 3] = RIG.to_base(31, Line("c", np.array([[axis[0] + 0.4, 0.0, 0.1]]), "table")).points[0]
    strut = Box("test_strut", T, np.array([0.03, 0.03, 0.2]), 0.05)
    with_strut = Obstacles(obs.boxes + (strut,), obs.planes, obs.capsules)
    lines = [
        _base(31, [[axis[0] + 1.3, -0.1], [axis[0] + 1.3, 0.1]], "far"),
        _base(31, [[axis[0] + 0.4, -0.1], [axis[0] + 0.4, 0.1]], "strut"),
        _base(31, [[axis[0] + 0.4, 0.3], [axis[0] + 0.403, 0.3]], "3mm"),
        Line("empty", np.zeros((0, 3)), "base"),
        _base(31, [[axis[0] + 0.4, 0.3], [axis[0] + 0.4, 0.3]], "same"),
    ]
    bunches, leftovers = plan(arm, lines, with_strut, rules, gates)
    reasons = {x.piece.line_id: {y.reason for y in leftovers if y.piece.line_id == x.piece.line_id}
               for x in leftovers}
    assert reasons["far"] == {"unreachable"}
    assert "blocked" in reasons["strut"]
    assert reasons["3mm"] == reasons["empty"] == reasons["same"] == {"too_short"}
    assert not [b for b in bunches if b.piece.line_id in ("far", "3mm", "empty", "same")]
    for line in lines:
        assert covers_once(line, bunches, leftovers), line.id
    strut_left = [x for x in leftovers if x.piece.line_id == "strut" and x.reason == "blocked"]
    assert all("test_strut" in x.detail for x in strut_left)


def test_frame_is_checked():
    arm, obs, rules, gates = _problem(31)
    with pytest.raises(ValueError):
        plan(arm, [Line("t", np.zeros((2, 3)), "table")], obs, rules, gates)


def test_a_simple_line_is_one_piece_with_real_alternatives():
    arm, obs, rules, gates = _problem(31)
    axis = RIG.T_table_base(31)[:2, 3]
    line = _base(31, [[axis[0] + 0.3, -0.2], [axis[0] + 0.3, 0.2]])
    bunches, leftovers = plan(arm, [line], obs, rules, gates)
    assert not leftovers and len(bunches) == 1
    b = bunches[0]
    assert 2 <= len(b.plans) <= 4
    starts = {tuple(np.round(p.q_start, 6)) for p in b.plans}
    ends = {tuple(np.round(p.q_end, 6)) for p in b.plans}
    assert len(starts) == len(b.plans) or len(ends) == len(b.plans)
    assert not plans_check(31, [line], bunches)
    tip_err = max(np.abs(p.tip_base - arm.tip(p.q)).max() for p in b.plans)
    assert tip_err < 1e-9


def _few():
    """Short lines: two letters, a short line, a short line along the rim."""
    cases = lc.base_cases(RIG, 31)
    return cases["word"][1:3] + [x for x in cases["lines"] if x.id in ("line:any:0",
                                                                      "line:rim:160")]


def test_same_answer_with_workers_and_in_a_fresh_process(tmp_path):
    arm, obs, rules, gates = _problem(31)
    lines = _few()
    one = plan(arm, lines, obs, rules, gates, workers=1)
    many = plan(arm, lines, obs, rules, gates, workers=8)
    assert _digest(one) == _digest(many)
    out = tmp_path / "fresh.txt"
    code = ("import sys; sys.path.insert(0, %r); import test_local as t; "
            "open(%r, 'w').write(t._fresh())" % (str(Path(__file__).parent), str(out)))
    env = dict(os.environ, PYTHONHASHSEED="12345")
    subprocess.run([sys.executable, "-c", code], check=True, env=env, cwd=str(DEPLOY))
    assert out.read_text() == _digest(one)


def _fresh() -> str:
    arm, obs, rules, gates = _problem(31)
    lines = _few()
    return _digest(plan(arm, lines, obs, rules, gates))


def _digest(result) -> str:
    import hashlib
    h = hashlib.sha256()
    bunches, leftovers = result
    for b in bunches:
        h.update(repr(b.piece).encode())
        for p in b.plans:
            for a in (p.q, p.s, p.tip_base, p.spin, p.lean):
                h.update(np.ascontiguousarray(a).tobytes())
            h.update(np.array([p.score, p.joint_travel, p.draw_time]).tobytes())
    for x in leftovers:
        h.update(repr(x).encode())
    return h.hexdigest()


# --------------------------------------------------------------------------- the fixed set


@pytest.fixture(scope="module", params=sorted(lc.ARMS))
def fixed_set(request):
    """The whole fixed set of one arm, planned with every core (minutes)."""
    arm_id = request.param
    return arm_id, lc.run(RIG, arm_id, WORKERS)


@pytest.mark.slow
def test_fixed_set(fixed_set):
    arm_id, results = fixed_set
    for name, (lines, bunches, leftovers, stats, _) in results.items():
        for line in lines:
            assert covers_once(line, bunches, leftovers), line.id
        assert not plans_check(arm_id, lines, bunches, WORKERS)
        length = sum(s.length for s in stats)
        drawn = sum(b.piece.s1 - b.piece.s0 for b in bunches)
        rows = lc.per_line(lines, bunches, leftovers, stats)
        single = np.array(rows["single"])
        one = np.array(rows["length"])[single].sum() / length
        print(f"arm {arm_id} {name}: drawn {drawn / length:.4f}, whole in one piece {one:.4f}")
        assert drawn / length >= FLOOR_DRAWN[arm_id, name], (name, drawn / length)
        assert np.array(rows["length"])[single].sum() / length >= FLOOR_SINGLE[arm_id, name]


@pytest.mark.slow
def test_the_table_guides_to_the_same_drawing(tmp_path):
    """With the kinematic table the plans are the same up to the grid: same share drawn, and
    every plan still passes the verification (which never looks at the table)."""
    arm, obs, rules, gates = _problem(31)
    cases = lc.base_cases(RIG, 31)
    lines = cases["word"] + cases["lines"][:40:4]
    live = plan(arm, lines, obs, rules, gates, workers=WORKERS)
    table = plan(arm, lines, obs, rules, gates, workers=WORKERS, cache_dir=tmp_path)
    length = sum(float(np.linalg.norm(np.diff(x.points, axis=0), axis=1).sum()) for x in lines)
    drawn = [sum(b.piece.s1 - b.piece.s0 for b in r[0]) / length for r in (live, table)]
    assert abs(drawn[0] - drawn[1]) <= 0.005, drawn
    assert not plans_check(31, lines, table[0], WORKERS)
    assert len(list(tmp_path.glob("local_table_*.npy"))) == 2
