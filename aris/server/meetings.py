"""The meetings calibration of one row, in one call (aris/calib/meetings.py does the maths).

Both arms of the row were hand-guided until their pen tips touched in the air, once or more
(two meetings far apart give the yaw); the joints of both arms were read at standstill.
"""
from __future__ import annotations


def calibrate_from_meetings(config_dir, row_slots, meetings, date=None):
    """Solve the row's two poses from `meetings` ([{L: q_L, R: q_R}, ...], joints in rad) and,
    when it passes, write both slots' base parts (`files.write_mark_solution`, method
    "meetings").  -> (MarkSolution, written paths)."""
    from aris.calib.files import write_mark_solution
    from aris.calib.meetings import solve_meetings
    from aris.rig import Rig
    rig = Rig.load(config_dir)
    sol = solve_meetings(rig, meetings, tuple(row_slots))
    paths = write_mark_solution(rig, sol, config_dir, date) if sol.passed else []
    return sol, paths
