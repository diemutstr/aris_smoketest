"""The meetings calibration, in one call (aris/calib/meetings.py does the maths).

Pairs of arms that share a spot were hand-guided until their pen tips touched in the air (a
row pair once or twice, a column pair at its seam spot); the joints of both were read at
standstill.
"""
from __future__ import annotations


def calibrate_from_meetings(config_dir, meetings, slots=None, date=None):
    """Solve every slot that met from `meetings` ([(slot_a, q_a, slot_b, q_b, spot), ...],
    joints in rad) and, when it passes, write every solved slot's base part
    (`files.write_mark_solution`, method "meetings").  `slots`: the slots that must take part
    (each must meet someone).  -> (MarkSolution, written paths)."""
    from aris.calib.files import write_mark_solution
    from aris.calib.meetings import solve_meetings
    from aris.rig import Rig
    rig = Rig.load(config_dir)
    sol = solve_meetings(rig, meetings, slots)
    paths = write_mark_solution(rig, sol, config_dir, date) if sol.passed else []
    return sol, paths
