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

# Capsules in the hand frame: (name, a, b, radius).  holder.0-2 were measured on the housing
# and cap meshes (assets/system_model/meshes/penholder) by tests/oracle/fit_arm_capsules.py;
# radius = exact mesh maximum rounded up to the millimetre (16.0, 20.5, 24.5 mm).
# pen_tail is the graphite stick's 72.5 mm tail behind the housing, pointing back at the wrist
# (a modelled length, never measured on the real pencil).  pen is the 20 mm of graphite past
# the cap.  Both are 3.5 mm sticks in 5 mm capsules; the pen capsule's segment stops 5 mm short
# of the tip, so the capsule's surface reaches exactly to the tip and a pen resting on the
# paper is at distance zero from it.
_R_STICK = 0.005
_CAPSULES = (
    ("holder.0", (0.0566, -0.0025, 0.0781), (0.0451, -0.0018, 0.0522), 0.016),
    ("holder.1", (0.0678, -0.0259, 0.0970), (0.0600, 0.0258, 0.0904), 0.021),
    ("holder.2", (0.0726, -0.0257, 0.1244), (0.0760, 0.0258, 0.1213), 0.025),
    ("pen_tail", (0.0166432, 0.0, -0.0140708), (0.0449767, 0.0, 0.0526787), _R_STICK),
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
