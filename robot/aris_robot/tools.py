"""Operator tools for one arm: which arm is where, small moves, the park, the calibration touch.

None of the moves here comes from the planner, so none is certified.  Each is refused unless
it is small: a jog of at most `JOG_MAX` on one joint; a park by straight joint interpolation
only from within `PARK_NEAR` of the park on every joint (anything further goes through the
server's park job: `aris park` on the planning PC, then `aris-robot run --job <id>`); a touch
that descends at most `TOUCH_MAX` straight down, slowly, under position control (touch.py).
"""
from __future__ import annotations

import re
import socket

import numpy as np

from aris.kernel.retime import retime
from aris.types import JointPath, Refusal

JOG_MAX = 0.10          # rad
PARK_NEAR = 0.05        # rad, on every joint
TOUCH_MAX = 0.06        # m


def straight(rig, arm_id: int, q0, q1):
    """A straight joint move from q0 to q1, timed inside the rig's limits."""
    return retime(JointPath(np.array([q0, q1], float)), rig.arm(arm_id).limits, rig.rules())


def jog_target(rig, arm_id: int, q, joint: int, delta: float):
    """q with joint `joint` (1..7) moved by `delta`, or a Refusal."""
    if not 1 <= joint <= 7:
        return Refusal("bad_joint", "joints are numbered 1 to 7")
    if abs(delta) > JOG_MAX:
        return Refusal("too_far", f"a jog moves at most {JOG_MAX} rad")
    lim = rig.arm(arm_id).limits
    out = np.array(q, float)
    out[joint - 1] += delta
    if not (lim.q_min[joint - 1] + 0.05 <= out[joint - 1] <= lim.q_max[joint - 1] - 0.05):
        return Refusal("limit", f"joint {joint} would come within 0.05 rad of its limit")
    return out


def park_move(rig, arm_id: int, q):
    """The uncertified straight move to the park, only from close by; or a Refusal."""
    park = rig.park_q(arm_id)
    gap = np.abs(np.asarray(q, float) - park)
    if gap.max() <= 1e-4:
        return Refusal("at_park", f"arm {arm_id} stands at its park")
    if gap.max() > PARK_NEAR:
        j = int(gap.argmax())
        return Refusal("too_far", f"arm {arm_id} is {gap[j]:.3f} rad from its park on joint "
                       f"{j + 1} (straight moves only within {PARK_NEAR}); run the server's park "
                       f"job: `aris park` there, then `aris-robot run --job <id>` here")
    return straight(rig, arm_id, q, park)


def identify(site, rig, timeout: float = 3.0) -> list[dict]:
    """Per arm of the site: does its address answer, does its domain answer, which address
    does the hardware there use, the robot mode, and how far it stands from each park."""
    from aris_robot.rosarm import MODES, ArmNode
    import time
    rows = []
    for sa in site.arms:
        row = dict(arm=sa.id, ip=sa.ip, domain=sa.domain, mounted=sa.mounted,
                   ip_answers=_answers(sa.ip))
        node = ArmNode(sa, site.joint_names())
        try:
            t_end = time.monotonic() + timeout
            while node.joints()[0] is None and time.monotonic() < t_end:
                time.sleep(0.05)
            q, _ = node.joints()
            mode, errors = node.mode_and_errors()
            row.update(domain_answers=q is not None, mode=MODES.get(mode, mode),
                       errors=errors, hardware_ip=_hardware_ip(node, timeout),
                       controllers=sorted(node.active_controllers(timeout) or []))
            if q is not None:
                dist = {a: float(np.abs(q - rig.park_q(a)).max()) for a in rig.arm_ids}
                row.update(q=[round(float(x), 4) for x in q],
                           nearest_park=min(dist, key=dist.get),
                           nearest_park_rad=round(min(dist.values()), 4))
        finally:
            node.close()
        rows.append(row)
    return rows


def _answers(ip: str) -> bool:
    try:
        with socket.create_connection((ip, 443), timeout=1.0):   # Desk
            return True
    except OSError:
        return False


def _hardware_ip(node, timeout: float) -> str | None:
    """The robot_ip in the robot description the arm's stack was launched with."""
    from rcl_interfaces.srv import GetParameters
    from aris_robot.rosarm import wait
    cli = node.node.create_client(GetParameters,
                                  f"/{node.arm.namespace}/robot_state_publisher/get_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    res = wait(cli.call_async(GetParameters.Request(names=["robot_description"])), timeout)
    if res is None or not res.values:
        return None
    m = re.search(r'name="robot_ip"[^>]*>([^<]+)<', res.values[0].string_value)
    return m.group(1).strip() if m else None
