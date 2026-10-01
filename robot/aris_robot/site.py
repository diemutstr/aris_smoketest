"""The site file: which arm answers where, which arms are bolted in, the server, and what is a
fact of this site or arm about force (each arm's force sign, the tare limits, the contact
detection, the touch).  How hard the pen presses is a fact of pen and paper: rig.json `pen`.

Geometry is not here: poses, parks and pens come from config/rig.json through `aris.rig`.
A malformed site file is a broken installation, so reading it raises with the reason.
"""
from __future__ import annotations

import ipaddress
import json
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SiteArm:
    id: int
    ip: str
    domain: int              # ROS_DOMAIN_ID; by convention the arm id
    mounted: bool            # bolted in and answering; an unmounted arm is never driven
    inverted: bool           # hangs from the frame (the launch hangs the model too)
    force_sign: float = 1.0  # +1: the paper pushing the pen up reads positive on this arm

    @property
    def namespace(self) -> str:
        return f"arm_{self.id}"


@dataclass(frozen=True)
class Site:
    path: Path
    server_url: str
    joint_prefix: str
    rmw: str
    force: dict              # the "force" block: tare and contact detection (ForceSettings.from_parts)
    touch: dict              # the "touch" block, read by touch.TouchSettings.from_site
    arms: tuple[SiteArm, ...]

    def arm(self, arm_id: int) -> SiteArm:
        for a in self.arms:
            if a.id == arm_id:
                return a
        raise KeyError(f"arm {arm_id} is not in {self.path}")

    @property
    def mounted(self) -> tuple[int, ...]:
        return tuple(a.id for a in self.arms if a.mounted)

    def joint_names(self) -> list[str]:
        return [f"{self.joint_prefix}_joint{i}" for i in range(1, 8)]


def load(path) -> Site:
    path = Path(path)
    d = json.loads(path.read_text())
    arms = []
    for row in d["arms"]:
        ipaddress.ip_address(row["ip"])                     # raises on a malformed address
        dom = int(row["domain"])
        if not 0 <= dom <= 101:
            raise ValueError(f"arm {row['id']}: DDS domain {dom} outside 0..101")
        arms.append(SiteArm(int(row["id"]), str(row["ip"]), dom, bool(row["mounted"]),
                            bool(row["inverted"]), float(row.get("force_sign", 1.0))))
    for key in ("id", "ip", "domain"):
        seen = [getattr(a, key) for a in arms]
        if len(set(seen)) != len(seen):
            raise ValueError(f"{path}: two arms share the same {key}")
    ros = d.get("ros", {})
    return Site(path, str(d["server_url"]).rstrip("/"), str(ros.get("joint_prefix", "fr3")),
                str(ros.get("rmw", "rmw_fastrtps_cpp")), dict(d["force"]),
                dict(d.get("touch", {})), tuple(arms))
