"""The lateral pen holder that every arm carries, as a `Tool` in the hand frame.

The holder is clamped at the far end of the Fat Franka Finger blades, 66.5 mm out along hand x
and 103.4 mm down hand z.  Its bore leans 23 deg off the hand's approach axis, toward +x.  The
graphite stands 20 mm past the cap's outer face, which is 30.001 mm from the grip along the
bore, so the tip is 50.001 mm down the bore from the grip (old frames.py, 2026-09-07).
"""
import numpy as np

from aris.types import Capsule, Tool

PEN_LEAN = np.deg2rad(23.0)                  # the bore's lean off hand z, toward hand +x
# Tip in the hand frame: the old code's (PEN_LAT_HOLDER, 0, D_HAND_TCP + PEN_EXT_HOLDER),
# copied digit for digit so that tips agree with the old code exactly.
TIP_HAND = np.array([0.0860369, 0.0, 0.1034 + 0.0460262])
PEN_AXIS_HAND = np.array([np.sin(PEN_LEAN), 0.0, np.cos(PEN_LEAN)])

# Capsules in the hand frame: (name, a, b, radius).  holder.* (housing and cap meshes,
# assets/system_model/meshes/penholder) and pen_tail.* (the graphite stick's 72.5 mm tail behind
# the housing, pointing back at the wrist; a modelled length, never measured on the real
# pencil) were measured by tests/oracle/fit_arm_capsules.py: every vertex inside, and with the
# hand leaned up to 15 deg the lowest capsule point at most 1.2 mm below the lowest vertex (the
# 2 mm capsules are mop-ups along the edges nearest the paper).  pen is the 20 mm of graphite
# past the cap, a 3.5 mm stick in a 5 mm capsule whose segment stops 5 mm short of the tip, so
# its surface reaches exactly to the tip and a pen resting on the paper is at distance zero.
_R_STICK = 0.005
_CAPSULES = tuple((n, a, b, r) for n, _f, a, b, r in (
    ("holder.0", 9, (0.06955, -0.00349, 0.10970), (0.06955, 0.00349, 0.10970), 0.02500),
    ("holder.1", 9, (0.05314, 0.01688, 0.05676), (0.05430, -0.03905, 0.09607), 0.02501),
    ("holder.2", 9, (0.05350, 0.02459, 0.09240), (0.07950, 0.02459, 0.09240), 0.00201),
    ("holder.3", 9, (0.05247, 0.02525, 0.11529), (0.05233, 0.02532, 0.11567), 0.00200),
    ("holder.4", 9, (0.08067, -0.02500, 0.08969), (0.08093, -0.02500, 0.09029), 0.00198),
    ("holder.5", 9, (0.05346, 0.01151, 0.09008), (0.05313, 0.01227, 0.09018), 0.00200),
    ("holder.6", 9, (0.07464, 0.02314, 0.09327), (0.06086, 0.00045, 0.13203), 0.00201),
    ("holder.7", 9, (0.05796, 0.01425, 0.08871), (0.05796, 0.01425, 0.08871), 0.00201),
    ("holder.8", 9, (0.07842, -0.02440, 0.08907), (0.07825, -0.02421, 0.08922), 0.00201),
    ("pen_tail.0", 9, (0.01756, 0.00000, -0.01446), (0.04484, 0.00000, 0.04980), 0.00450),
    ("pen_tail.1", 9, (0.04332, 0.00544, 0.04617), (0.04343, -0.00026, 0.05018), 0.00450),
)) + (
    ("pen", (0.0782223, 0.0, 0.1310161), tuple(TIP_HAND - _R_STICK * PEN_AXIS_HAND), _R_STICK),
)


def default_tool() -> Tool:
    """The lateral pen holder as mounted on every arm."""
    caps = tuple(Capsule(name=n, p0=np.array(a, float), p1=np.array(b, float), radius=float(r),
                         margin=0.0)
                 for n, a, b, r in _CAPSULES)
    return Tool(tip_hand=TIP_HAND.copy(), pen_axis_hand=PEN_AXIS_HAND.copy(),
                capsules_hand=caps, pen_names=("pen",))


def with_tip(tool: Tool, tip_hand) -> Tool:
    """The same tool with its pen tip moved to `tip_hand` (hand frame), e.g. after calibration.

    The pen capsules keep their direction and radius; each is moved onto the line through the
    new tip along the pen axis, keeps its far end where the old one projects onto that line,
    and ends so that its surface reaches exactly to the new tip.  A pen shorter than its own
    radius collapses to a ball touching the tip.
    """
    tip = np.asarray(tip_hand, float).reshape(3).copy()
    u = tool.pen_axis_hand
    caps = []
    for c in tool.capsules_hand:
        if c.name in tool.pen_names:
            p1 = tip - c.radius * u
            p0 = p1 - max(float((p1 - c.p0) @ u), 0.0) * u
            c = Capsule(name=c.name, p0=p0, p1=p1, radius=c.radius, margin=c.margin)
        caps.append(c)
    return Tool(tip_hand=tip, pen_axis_hand=u.copy(), capsules_hand=tuple(caps),
                pen_names=tool.pen_names)
