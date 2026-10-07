"""The site: which robot hangs in which slot and where it answers, and what this operator PC
needs to know about it.  One source for each fact:

  site/<name>.json (repository root, the site table; named by robot/site.json `site_table`)
      per slot: the robot (e.g. "fr3-71"), its control-box IP, its DDS domain, optionally its
      serial, and whether the row was checked ("sure")
  robot/site.json (this operator PC)
      the server URL, ROS settings, per slot `mounted` (bolted in and switched on),
      `force_sign` and `rt_core`, the touch (`touch`), the collision thresholds (`collision`)

Arms are slots, `1L 1R 2L 2R 3L 3R` (DESIGN 4c): string ids everywhere.  The DDS domain is an
integer and comes from the site table (today the old robot id, as the live stacks use it).
Geometry is not here: poses, parks and pens come from config/rig.json through `aris.rig`, and
how hard the pen presses is the job header's `pen`.  A malformed file is a broken
installation, so reading it raises with the reason.
"""
from __future__ import annotations

import ipaddress
import json
from dataclasses import dataclass, field
from pathlib import Path

SLOTS = ("1L", "1R", "2L", "2R", "3L", "3R")


@dataclass(frozen=True)
class SiteArm:
    id: str                  # the slot
    robot: str               # which robot the site table says hangs there, e.g. "fr3-71"
    ip: str
    domain: int              # ROS_DOMAIN_ID, from the site table
    mounted: bool            # bolted in and answering; an unmounted slot is never driven
    force_sign: float = 1.0  # +1: the paper pushing the pen up reads positive on this arm
    serial: str | None = None  # the robot's serial, when the site table knows it
    sure: bool = False       # the table row was checked against the hardware
    rt_core: int | None = None  # the isolated CPU core this arm's stack runs on (taskset)
    never: bool = False      # the site table: a robot this rig never drives (no stack at all)

    @property
    def namespace(self) -> str:
        return f"arm_{self.id}"


@dataclass(frozen=True)
class Site:
    path: Path
    table_path: Path
    server_url: str
    joint_prefix: str
    rmw: str
    touch: dict              # the touch (touch.TouchSettings.from_site)
    arms: tuple[SiteArm, ...]
    collision: dict = field(default_factory=dict)   # "job" and "normal" thresholds
    desk: dict = field(default_factory=dict)        # Desk's web API: "mode_endpoint"
    execution: dict = field(default_factory=dict)   # rest_qd, settle_s, auto_recover
    hardware_component: str = "FrankaHardwareInterface"
    rt_priority: int = 95                           # SCHED_FIFO of each arm's stack

    def arm(self, slot: str) -> SiteArm:
        for a in self.arms:
            if a.id == str(slot):
                return a
        raise KeyError(f"slot {slot} is not in {self.path}")

    @property
    def mounted(self) -> tuple[str, ...]:
        return tuple(a.id for a in self.arms if a.mounted)

    def joint_names(self) -> list[str]:
        return [f"{self.joint_prefix}_joint{i}" for i in range(1, 8)]


def load(path) -> Site:
    path = Path(path).resolve()
    d = json.loads(path.read_text())
    table_path = (path.parents[1] / d["site_table"]).resolve()
    table = json.loads(table_path.read_text())["slots"]
    arms = []
    for slot, mine in d["slots"].items():
        if slot not in SLOTS:
            raise ValueError(f"{path}: {slot!r} is not a slot ({' '.join(SLOTS)})")
        if slot not in table:
            raise ValueError(f"{path}: slot {slot} is not in the site table {table_path}")
        row = table[slot]
        ipaddress.ip_address(row["ip"])                     # raises on a malformed address
        dom = int(row["domain"])
        if not 0 <= dom <= 101:
            raise ValueError(f"slot {slot}: DDS domain {dom} outside 0..101")
        core = mine.get("rt_core")
        never = row.get("controlled") == "never" or bool(row.get("absent", False))
        if mine["mounted"] and never:
            raise ValueError(f"{path}: slot {slot} is mounted, but the site table says its "
                             f"robot is never driven by this rig (or the slot is empty)")
        arms.append(SiteArm(slot, str(row["robot"]), str(row["ip"]), dom, bool(mine["mounted"]),
                            float(mine.get("force_sign", 1.0)), row.get("serial"),
                            bool(row.get("sure", False)), None if core is None else int(core),
                            never))
    cores = [a.rt_core for a in arms if a.rt_core is not None]
    if len(set(cores)) != len(cores):
        raise ValueError(f"{path}: two slots share an rt_core")
    for key in ("ip", "domain", "robot"):
        seen = [getattr(a, key) for a in arms]
        if len(set(seen)) != len(seen):
            raise ValueError(f"{table_path}: two slots share the same {key}")
    ros = d.get("ros", {})
    return Site(path, table_path, str(d["server_url"]).rstrip("/"),
                str(ros.get("joint_prefix", "fr3")), str(ros.get("rmw", "rmw_fastrtps_cpp")),
                dict(d.get("touch", {})), tuple(arms),
                dict(d.get("collision", {})), dict(d.get("desk", {})),
                dict(d.get("execution", {})),
                str(ros.get("hardware_component", "FrankaHardwareInterface")),
                int(ros.get("rt_priority", 95)))


def identity(arm: SiteArm, found_serial: str | None) -> str:
    """Whether the robot at the slot's address is the one the table names: "verified",
    "mismatch: ..." or "unverified" (no serial on one side or the other)."""
    if arm.serial is None or found_serial is None:
        return "unverified"
    if str(found_serial) == str(arm.serial):
        return "verified"
    return f"mismatch: the table names {arm.robot} (serial {arm.serial}), found {found_serial}"
