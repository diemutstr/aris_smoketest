"""The independent checker.  Shares no code with the planners; see docs/modules/check.md."""
from aris.check.motion import check
from aris.check.phase import check_phase_end
from aris.check.verdict import Measurement, Verdict

__all__ = ["check", "check_phase_end", "Measurement", "Verdict"]
