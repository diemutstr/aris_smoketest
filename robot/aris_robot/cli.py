"""The operator PC's one command: `aris-robot <verb>`.

  serve     the resident process: started once (at boot by systemd), writes the launch
            arguments, keeps the arms' ROS stacks up and takes every command from the drawing
            server; everything a person does, they do on the planning PC
  identify  read-only diagnostics for when the server is down: per slot, the robot the site
            table names, whether its address and domain answer, its mode, where it stands
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from aris.rig import Rig

from aris_robot import bringup, site as site_mod, tools
from aris_robot.remote import Remote

REPO = Path(__file__).resolve().parents[2]


def _say(ok: bool, what: str) -> int:
    print(("PASS " if ok else "FAIL ") + what)
    return 0 if ok else 1


def robots(site) -> dict:
    """Per mounted slot: the robot the site table names and whether that is verified.  FCI and
    ROS report no serial, so until one is readable every slot is "unverified"."""
    return {sa.id: dict(robot=sa.robot, ip=sa.ip, serial_found=None,
                        identity=site_mod.identity(sa, None)) for sa in site.arms if sa.mounted}


def cmd_identify(a, site, rig) -> int:
    rows = tools.identify(site, rig)
    for r in rows:
        print(json.dumps(r))
    bad = [r["arm"] for r in rows if r["mounted"] and not r.get("domain_answers")]
    return _say(not bad, "every mounted arm answers" if not bad else f"no answer from {bad}")


def calibration_driver(a, site, drivers):
    """make_calib(slot, rig, say) for serve: panda-py and Desk for real arms (needs
    aris_robot[calib] and robot/secrets.json), the simulated arm with a Desk that presses check
    at once for --sim-speed; None when neither is available (mark jobs are then refused)."""
    from aris_robot.calib import CalibArm, PandaFci
    from aris_robot.desk import PandaDesk, SimDesk, credentials
    if a.sim_speed is not None:
        from aris_robot.simarm import SimFci
        return lambda slot, rig, say: CalibArm(slot, rig, lambda: SimFci(drivers[slot]),
                                               SimDesk(["check"] * 1000), say=say)
    if a.fake:
        return None                     # fake hardware has no Desk
    try:
        import panda_py  # noqa: F401
    except ImportError:
        return None
    secrets = Path(a.site).parent / "secrets.json"

    def make(slot, rig, say):
        sa = site.arm(slot)
        user, pw = credentials(secrets, sa.robot)
        desk = PandaDesk(sa.ip, user, pw, site.desk["mode_endpoint"],
                         say=lambda event, **f: say(event, arm=slot, **f))
        return CalibArm(slot, rig, lambda: PandaFci(sa.ip), desk, say=say)
    return make


def cmd_serve(a, site, rig) -> int:
    import logging
    import signal
    from aris_robot.serve import Operator, Rows, Stacks, launch_commands
    log_dir = Path(a.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=log_dir / "serve.log", level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    remote = Remote(site.server_url)
    rows = Rows(remote, log_dir)
    mounted = [i for i in site.mounted if i in rig.arm_ids]
    stacks = None
    if a.sim_speed is not None:
        from aris_robot.simarm import SimTouchArm
        drivers = {i: SimTouchArm(rig, i, rig.park_q(i), speed=a.sim_speed,
                                  paper_m=a.fake_paper_mm / 1000.0) for i in mounted}
    else:
        from aris_robot.driver import RosArm
        files = [f for f in bringup.write(rig, site, a.out, fake=a.fake)
                 if json.loads(f.read_text())["arm"] in mounted]
        stacks = Stacks(launch_commands(files), rows, log_dir).start()
        drivers = {i: RosArm(site, rig, i, fake=a.fake, fake_paper_m=a.fake_paper_mm / 1000.0)
                   for i in mounted}
    op = Operator(remote, a.config, a.work, drivers, log_dir, stacks, rows=rows,
                  robots=robots(site), make_calib=calibration_driver(a, site, drivers))

    def leave(signum, frame):
        op.quit.set()
        if op.busy.is_set():
            for d in drivers.values():
                d.stop()

    signal.signal(signal.SIGTERM, leave)
    signal.signal(signal.SIGINT, leave)
    try:
        op.serve()
    finally:
        if stacks is not None:
            stacks.stop()
        for d in drivers.values():
            getattr(d, "close", lambda: None)()
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aris-robot", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--site", default=str(REPO / "robot" / "site.json"))
    p.add_argument("--config", default=str(REPO / "config"))
    p.add_argument("--fake", action="store_true", help="the arms run on fake hardware")
    sub = p.add_subparsers(dest="verb", required=True)
    sv = sub.add_parser("serve")
    sv.add_argument("--out", default=str(REPO / "robot" / "generated"))
    sv.add_argument("--work", default=str(REPO / "out" / "robot_jobs"))
    sv.add_argument("--log-dir", default=str(REPO / "out" / "operator"))
    sv.add_argument("--sim-speed", type=float, default=None,
                    help="simulated arms at this many times real time (inf: at once); no ROS")
    sv.add_argument("--fake-paper-mm", type=float, default=0.0,
                    help="with --fake or --sim-speed: the fake paper, mm above the nominal one")
    sub.add_parser("identify")
    return p


def main(argv=None) -> int:
    a = parser().parse_args(argv)
    if getattr(a, "sim_speed", None) is not None and not a.sim_speed > 0:
        raise SystemExit("--sim-speed must be positive (inf allowed)")
    site = site_mod.load(a.site)
    rig = Rig.load(a.config)
    print(f"site {site.path}; server {site.server_url}; mounted {list(site.mounted)}"
          + ("; FAKE HARDWARE" if a.fake else ""))
    verbs = dict(identify=cmd_identify, serve=cmd_serve)
    return verbs[a.verb](a, site, rig)


if __name__ == "__main__":
    sys.exit(main())
