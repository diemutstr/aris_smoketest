"""Step 2: the followers draw in the leader phases, against their leader's footprint.

Once a leader has planned its phase, everywhere its body goes (all its motions, and its park,
where it stands before and after) becomes a distance field (`kernel.footprint`, 1 cm cells,
margin the arm-to-arm clearance), handed to its row partner in the partner's base frame.  The
follower's obstacles are then its steel, the paper, its leader's walls and that footprint; its
drawable map is computed against them, after the leader has planned, so it is never cached.
No timing between the two: the follower avoids everywhere the leader will ever be.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from multiprocessing import get_context

import numpy as np

from aris.kernel.footprint import footprint, transform_field
from aris.system import maps as maps_mod
from aris.system.phases import follower_phase
from aris.types import Trajectory

CELL = 0.01          # m, the footprint's grid


def pairs(rig, phase) -> list[tuple[int, int]]:
    """(leader, follower) for every leader whose row partner stands parked in `phase`."""
    return [(a, rig.row_partner(a)) for a in phase.active
            if rig.row_partner(a) is not None and rig.row_partner(a) in phase.parked]


def setup(job):
    """-> (follower, Field in its frame, Obstacles, Map).  Runs in a worker process."""
    rig, phase, lead, f, trajs, gates, cfg = job
    park = rig.park_q(lead)
    stand = Trajectory(np.array([0.0, 1.0]), np.stack([park, park]), np.zeros((2, 7)))
    margin = rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]
    fp = footprint(rig.arm(lead), list(trajs) + [stand], cell=CELL, name=f"footprint{lead}",
                   margin=margin)
    field = transform_field(fp, rig.T_base_table(f) @ rig.T_table_base(lead))
    fph = follower_phase(rig, phase, f)
    obs = replace(rig.obstacles(f, (), fph.walls), fields=(field,))
    return f, field, obs, maps_mod.build(rig, phase, f, gates, cfg, obstacles=obs)


def setup_all(rig, phase, trajs_by_leader: dict, gates, cfg, workers: int = 1) -> dict:
    """{follower: (Field, Obstacles, Map)} for every (leader, follower) pair of `phase`."""
    jobs = [(rig, phase, lead, f, trajs_by_leader.get(lead, []), gates, cfg)
            for lead, f in pairs(rig, phase)]
    if workers > 1 and len(jobs) > 1:
        with ProcessPoolExecutor(min(workers, len(jobs)), mp_context=get_context("spawn")) as ex:
            out = list(ex.map(setup, jobs))
    else:
        out = [setup(j) for j in jobs]
    return {f: (field, obs, m) for f, field, obs, m in out}
