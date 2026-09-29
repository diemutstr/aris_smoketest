"""The independent checker.  Shares no code with the planners; see docs/modules/check.md."""
from aris.check.motion import check
from aris.check.verdict import Measurement, Verdict

__all__ = ["check", "Measurement", "Verdict"]
