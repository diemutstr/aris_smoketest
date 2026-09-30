"""The launch arguments and controller settings of every arm, from rig.json and site.json.

For each arm of the site it writes, into one folder:
  arm_<id>.json               what `ros2 launch aris_bringup arm.launch.py args:=<file>` needs:
                              robot_ip, namespace, DDS domain, fake hardware or not, and the
                              base pose for the operator patches of franka_description
                              (mount_to_world, mroll, mpitch, myaw, mz)
  arm_<id>_controllers.yaml   aris_bringup/config/controllers.yaml with this arm's pen tip
                              (tip_offset_flange) and joint names; for fake hardware, the
                              trajectory controller on the position interface and no model

The mount: the patched fr3.urdf.xacro hangs `base` from a per-arm `world` frame on the paper
right under the arm's axis, by a height `mz` and a roll-pitch-yaw.  So only the rotation and
the height of `T_table_base` are used; the arm's x, y on the table are not (the controllers
never use this transform; it is for the robot model and RViz).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import yaml

TEMPLATE = (Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "aris_bringup" / "config"
            / "controllers.yaml")


def rpy(R) -> tuple[float, float, float]:
    """URDF roll, pitch, yaw of a rotation: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    R = np.asarray(R, float)
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0])) + 0.0     # no -0.0
    if math.cos(pitch) > 1e-9:
        return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), pitch, 0.0          # gimbal lock: yaw folded in


def tip_in_flange(arm) -> np.ndarray:
    """The pen tip in the flange frame (libfranka's kFlange), from the kernel's chain."""
    c = arm.chain_table()
    f = c.names.index("hand")
    return c.R[f] @ np.asarray(arm.tool.tip_hand, float) + c.t[f]


def launch_args(rig, site, arm_id: int, controllers_file, fake: bool) -> dict:
    sa = site.arm(arm_id)
    T = rig.T_table_base(arm_id)
    roll, pitch, yaw = rpy(T[:3, :3]) if sa.inverted else (0.0, 0.0, 0.0)
    return dict(arm=arm_id, namespace=sa.namespace, domain=sa.domain, robot_ip=sa.ip,
                mounted=sa.mounted, use_fake_hardware=bool(fake), rmw=site.rmw,
                mount_to_world=bool(sa.inverted), mroll=roll, mpitch=pitch, myaw=yaw,
                mz=float(T[2, 3]) if sa.inverted else 0.0,
                controllers=str(controllers_file))


def controllers(site, tip_flange, fake: bool, template=TEMPLATE) -> dict:
    """The template with this arm's settings filled in."""
    d = yaml.safe_load(Path(template).read_text())
    prefix = site.joint_prefix
    jtc = d["/**/fr3_arm_controller"]["ros__parameters"]
    imp = d["/**/aris_joint_impedance_controller"]["ros__parameters"]
    d["/**/franka_robot_state_broadcaster"]["ros__parameters"]["arm_id"] = prefix
    imp["arm_id"] = prefix
    imp["tip_offset_flange"] = [float(x) for x in tip_flange]
    names = site.joint_names()
    jtc["joints"] = names
    jtc["gains"] = {n: g for n, g in zip(names, jtc["gains"].values())}
    cons = jtc["constraints"]
    per_joint = [cons.pop(k) for k in list(cons) if k.startswith("fr3_joint")]
    cons.update({n: c for n, c in zip(names, per_joint)})
    if fake:
        # fake hardware mirrors a position command and ignores torques
        jtc["command_interfaces"] = ["position"]
        jtc.pop("gains")
        imp["use_model"] = False
    return d


def write(rig, site, out_dir, fake: bool = False, template=TEMPLATE) -> list[Path]:
    """Writes both files for every arm of the site; returns the paths of the argument files."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written = []
    for sa in site.arms:
        ctl = out / f"arm_{sa.id}_controllers.yaml"
        tip = tip_in_flange(rig.arm(sa.id))
        ctl.write_text(yaml.safe_dump(controllers(site, tip, fake, template), sort_keys=False))
        args = out / f"arm_{sa.id}.json"
        args.write_text(json.dumps(launch_args(rig, site, sa.id, ctl.resolve(), fake), indent=1))
        written.append(args)
    return written
