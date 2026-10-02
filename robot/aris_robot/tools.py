"""Read-only diagnostics: which robot answers where (`aris-robot identify`), for when the
drawing server is down.  Nothing here moves an arm.
"""
from __future__ import annotations

import re
import socket

import numpy as np

from aris_robot.site import identity


def identify(site, rig, timeout: float = 3.0) -> list[dict]:
    """Per slot of the site: the robot the site table names; does its address answer, does
    its domain answer, which address the stack there was launched with, the robot mode, and
    which slot's park it stands nearest.  No serial: FCI and ROS report none (nor libfranka's
    server version), so the identity stays "unverified"."""
    from aris_robot.rosarm import MODES, ArmNode
    import time
    rows = []
    for sa in site.arms:
        row = dict(arm=sa.id, robot=sa.robot, table_sure=sa.sure, ip=sa.ip, domain=sa.domain,
                   mounted=sa.mounted, ip_answers=_answers(sa.ip), serial_found=None,
                   identity=identity(sa, None))
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
