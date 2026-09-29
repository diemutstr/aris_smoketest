"""The fixed test set of the free-space planner: 1 000 pairs of lift-off configurations for arm
31 (phase 2) and 1 000 for arm 13 (phase 1).

Run `../.venv/bin/python tests/free_cases.py` from deployment/ to (re)build
`tests/data/free_cases_<arm>.npz`.  Everything is seeded, so a rebuild gives the same file.

How a case is made
- tips: uniform on the paper within 1.0 m of the arm's axis (table frame, clipped to the canvas),
  `rules.lift_height` above the paper, random spin, no lean, joint 7 uniform inside its margin
- configurations: every IK answer (slot = IK branch) that passes the gates: joint-limit margin,
  sigma_min, free of the obstacles of the arm's phase and of itself
- four groups of 250 pairs: near (tips under 0.15 m apart), far (over 0.6 m), same IK branch,
  different IK branch (the last two at any distance)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from aris.kernel import collide
from aris.rig import Rig
from aris.types import DrawRules

DEPLOY = Path(__file__).resolve().parents[1]
CONFIG = DEPLOY / "config"
DATA = DEPLOY / "tests" / "data"
ARMS = {31: 2, 13: 1}                  # arm -> the phase in which it moves (leads)
GROUPS = ("near", "far", "same_branch", "other_branch")
PER_GROUP = 250
N_POSES = 12_000                        # candidate tip poses per arm
REACH_XY = 1.0                          # m, tips are drawn within this of the arm's axis


def scene(rig: Rig, arm_id: int):
    """(arm, obstacles, rules) of `arm_id` in the phase it leads."""
    phase = rig.phase(ARMS[arm_id])
    rules = DrawRules(gates=rig.gates())
    return rig.arm(arm_id), rig.obstacles_for(arm_id, phase), rules


def pool(rig: Rig, arm_id: int, seed: int):
    """Gated, free lift-off configurations: q (P,7), tip_table (P,3), branch (P,)."""
    arm, obs, rules = scene(rig, arm_id)
    g = rules.gates
    rng = np.random.default_rng(seed)
    axis = rig.T_table_base(arm_id)[:2, 3]
    half = rig.canvas_size / 2
    r = REACH_XY * np.sqrt(rng.uniform(0, 1, N_POSES))
    a = rng.uniform(-np.pi, np.pi, N_POSES)
    xy = axis + np.stack([r * np.cos(a), r * np.sin(a)], 1)
    xy = np.clip(xy, -half, half)
    tip_table = np.column_stack([xy, np.full(N_POSES, rig.paper_z + rules.lift_height)])
    T = rig.T_base_table(arm_id)
    tip_base = tip_table @ T[:3, :3].T + T[:3, 3]
    normal = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    spin = rng.uniform(-np.pi, np.pi, N_POSES)
    lim = arm.limits
    q7 = rng.uniform(lim.q_min[6] + g.limit_margin, lim.q_max[6] - g.limit_margin, N_POSES)
    Q, ok = arm.ik(arm.hand_pose(tip_base, normal, spin, np.zeros((N_POSES, 2))), q7)
    m, b = np.nonzero(ok)
    q = Q[m, b]
    keep = (arm.limit_margin(q) >= g.limit_margin) & (arm.sigma_min(q) >= g.sigma_min)
    m, b, q = m[keep], b[keep], q[keep]
    tables = collide.arm_tables(arm)
    keep = ((collide.clearance_q(tables, q, obs) >= 0.0)
            & (collide.self_clearance_q(tables, q, arm.self_pairs, g.self_margin) >= 0.0))
    return q[keep], tip_table[m[keep]], b[keep]


def pairs(q, tip, branch, seed: int):
    """(PER_GROUP * 4) index pairs into the pool, and the group of each."""
    rng = np.random.default_rng(seed)
    n = len(q)
    out, group = [], []
    tests = {
        "near": lambda i, j: np.linalg.norm(tip[i] - tip[j], axis=-1) < 0.15,
        "far": lambda i, j: np.linalg.norm(tip[i] - tip[j], axis=-1) > 0.6,
        "same_branch": lambda i, j: branch[i] == branch[j],
        "other_branch": lambda i, j: branch[i] != branch[j],
    }
    for gi, name in enumerate(GROUPS):
        got = []
        while len(got) < PER_GROUP:
            i = rng.integers(0, n, 20_000)
            j = rng.integers(0, n, 20_000)
            good = (i != j) & tests[name](i, j)
            got += list(zip(i[good], j[good]))
        out += got[:PER_GROUP]
        group += [gi] * PER_GROUP
    return np.array(out), np.array(group)


def build(arm_id: int, seed: int = 20260929) -> Path:
    rig = Rig.load(CONFIG)
    q, tip, branch = pool(rig, arm_id, seed + arm_id)
    ij, group = pairs(q, tip, branch, seed + 1000 + arm_id)
    path = DATA / f"free_cases_{arm_id}.npz"
    np.savez_compressed(
        path, arm_id=arm_id, phase=ARMS[arm_id], groups=np.array(GROUPS),
        q_start=q[ij[:, 0]], q_goal=q[ij[:, 1]], group=group,
        tip_start_table=tip[ij[:, 0]], tip_goal_table=tip[ij[:, 1]],
        branch_start=branch[ij[:, 0]], branch_goal=branch[ij[:, 1]], pool_size=len(q))
    return path


def load(arm_id: int) -> dict:
    with np.load(DATA / f"free_cases_{arm_id}.npz") as f:
        return {k: f[k] for k in f.files}


if __name__ == "__main__":
    for aid in (ARMS if len(sys.argv) < 2 else [int(a) for a in sys.argv[1:]]):
        p = build(aid)
        d = load(aid)
        dist = np.linalg.norm(d["tip_start_table"] - d["tip_goal_table"], axis=1)
        print(f"arm {aid}: pool {int(d['pool_size'])}, {len(dist)} pairs -> {p.name}")
        for gi, name in enumerate(GROUPS):
            s = d["group"] == gi
            print(f"  {name:13s} tip distance median {np.median(dist[s]):.3f} m, "
                  f"same branch {np.mean(d['branch_start'][s] == d['branch_goal'][s]):.2f}")
