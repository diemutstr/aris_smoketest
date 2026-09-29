"""Local planner: lines on the paper -> how one arm draws each of them.  See docs/modules/local.md."""
from aris.local.planner import plan, plan_detailed, reverse_plan, verify_plan
from aris.local.settings import Settings

__all__ = ["plan", "plan_detailed", "reverse_plan", "verify_plan", "Settings"]
