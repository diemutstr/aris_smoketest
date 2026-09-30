"""The phases the system planner runs, in order.

A phase is who moves, who stands parked, and the walls between those who move.  Phases 1 and 2
come from the rig (the leaders of each row, diagonal walls between them).  After them come the
fill phases: arms that cannot touch each other in any configuration move together, without
walls, everything else parked.  On this rig: 13 with 2, 17 with 97 (the two ends of a column,
2.42 m apart), then 31 alone and 71 alone.  The system planner skips a phase in which nobody
has anything to do.
"""
from __future__ import annotations

import numpy as np

from aris.types import Phase, Wall

LEADER_PHASES = (1, 2)


def horizontal_reach(rig, arm_id: int) -> float:
    """A bound on how far any part of the arm (links, hand, holder, pen; base included) can be
    from its own first joint's axis, over every configuration.  From the arm model's bound on
    each capsule's distance from joint 1's axis (the triangle inequality along the chain,
    kernel.arm `reach`); the fixed base capsules, which joint 1 does not move, measured."""
    arm = rig.arm(arm_id)
    body = arm.body(np.zeros((1, 7)))
    fixed = np.zeros(len(body.names), bool) if body.is_fixed is None else body.is_fixed
    moving = float(np.max(arm.reach[0][~fixed]))
    base = np.maximum(np.hypot(*body.p0[0][fixed][:, :2].T), np.hypot(*body.p1[0][fixed][:, :2].T))
    return max(moving, float(np.max(base + body.radius[fixed], initial=0.0)))


def axis_gap(rig, a: int, b: int) -> float:
    """The smallest horizontal distance between the two arms' first-joint axes, between the
    paper and the higher base (the axes may lean slightly after a calibration)."""
    Ta, Tb = rig.T_table_base(a), rig.T_table_base(b)
    zs = np.linspace(rig.paper_z, max(Ta[2, 3], Tb[2, 3]), 11)

    def at(T, z):                        # the axis at height z
        d = T[:3, 2]
        return T[:2, 3] + d[:2] * (z - T[2, 3]) / d[2]
    return float(min(np.linalg.norm(at(Ta, z) - at(Tb, z)) for z in zs))


def cannot_touch(rig, a: int, b: int) -> bool:
    """True when arms a and b stay the arm-to-arm clearance apart in every configuration."""
    margin = rig.clearance["arm_to_arm_m"] + rig.allowance["arm_to_arm_m"]
    return axis_gap(rig, a, b) - horizontal_reach(rig, a) - horizontal_reach(rig, b) >= margin


def fill_groups(rig) -> list[tuple[int, ...]]:
    """Arms in rig order, each joining the first group it cannot touch any member of."""
    groups: list[list[int]] = []
    for a in rig.arm_ids:
        for g in groups:
            if all(cannot_touch(rig, a, b) for b in g):
                g.append(a)
                break
        else:
            groups.append([a])
    return [tuple(g) for g in groups]


def fill(rig, arms) -> Phase:
    """These arms move together, no walls; every other arm stands parked."""
    arms = (arms,) if isinstance(arms, int) else tuple(arms)
    return Phase("fill " + "+".join(str(a) for a in arms), arms,
                 tuple(a for a in rig.arm_ids if a not in arms), ())


def phases(rig) -> list[Phase]:
    """Every phase, in the order they run: 1, 2, then the fill phases."""
    return [rig.phase(n) for n in LEADER_PHASES] + [fill(rig, g) for g in fill_groups(rig)]


def phase_named(rig, name: str) -> Phase:
    """The phase with this name (what `plan` tags every motion with)."""
    for p in phases(rig):
        if p.name == name:
            return p
    raise KeyError(f"no phase {name!r}")


def is_fill(phase: Phase) -> bool:
    return phase.name.startswith("fill ")


def follower_phase(rig, phase: Phase, arm_id: int) -> Phase:
    """What a follower moves in, during a leader phase: alone, nothing parked (every arm
    moves), behind the same walls as its leader (each wall next to the leader, held on the
    follower's own side).  Its leader is a footprint, handed over separately."""
    lead = rig.row_partner(arm_id)
    walls = tuple(Wall(f"{w.name}_for_{arm_id}", (arm_id, w.arms[1] if w.arms[0] == lead
                                                  else w.arms[0]),
                       w.point_table, w.normal_table)
                  for w in phase.walls if lead in w.arms)
    return Phase(phase.name, (arm_id,), (), walls)
