"""The operator PC's one command: `aris-robot <verb>`.

  serve     the resident process: started once (at boot), takes every command from the
            drawing server; once it runs, nothing below is needed

Hardware-day tools:
  bringup   write the launch arguments and controller settings of every arm
  identify  which arm answers on which address and domain, and where it stands
  run       fetch a job's queues from the server and run them on the mounted arms
  park      straight to the park, only from close by (else the server's park job)
  jog       one joint by a little
  touch     the calibration touch by hand: straight down under position control until the
            force onset, report the joints, back up
  switch    hand an arm to the trajectory or the impedance controller
  recover   after a fault or a stop, once a person has looked: error recovery, then the
            trajectory controller takes the arm
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from aris.rig import Rig
from aris.types import Refusal

from aris_robot import bringup, site as site_mod, tools
from aris_robot.remote import Remote

REPO = Path(__file__).resolve().parents[2]


def _say(ok: bool, what: str) -> int:
    print(("PASS " if ok else "FAIL ") + what)
    return 0 if ok else 1


def _driver(site, rig, arm_id: int, fake: bool):
    from aris_robot.driver import RosArm
    if arm_id not in site.mounted and not fake:
        raise SystemExit(f"FAIL arm {arm_id} is not mounted in {site.path}")
    return RosArm(site, rig, arm_id, fake=fake)


def cmd_bringup(a, site, rig) -> int:
    files = bringup.write(rig, site, a.out, fake=a.fake)
    for f in files:
        arm = json.loads(f.read_text())
        mark = "" if arm["mounted"] or a.fake else "   # not mounted"
        print(f"ros2 launch aris_bringup arm.launch.py args:={f.resolve()}{mark}")
    return _say(True, f"{len(files)} arms written to {a.out}")


def cmd_identify(a, site, rig) -> int:
    rows = tools.identify(site, rig)
    for r in rows:
        print(json.dumps(r))
    bad = [r["arm"] for r in rows if r["mounted"] and not r.get("domain_answers")]
    return _say(not bad, "every mounted arm answers" if not bad else f"no answer from {bad}")


def cmd_run(a, site, rig) -> int:
    from aris_robot.runner import run_job
    if a.sim_speed is not None:
        from aris.execute.drivers.sim import SimArm
        drivers = {i: SimArm(i, rig.park_q(i), speed=a.sim_speed) for i in site.mounted}
    else:
        drivers = {i: _driver(site, rig, i, a.fake) for i in site.mounted}
    try:
        res = run_job(Remote(site.server_url), a.job, rig, a.config, a.work, drivers)
    finally:
        for d in drivers.values():
            getattr(d, "close", lambda: None)()
    if isinstance(res, Refusal):
        return _say(False, f"{res.reason}: {res.detail}")
    print(json.dumps(dict(status=res.status, why=res.why, phases=res.phases_done,
                          phase_ends=[list(map(str, p)) for p in res.phase_ends])))
    return _say(res.status == "done", f"job {a.job} {res.status}")


def _move(drv, traj) -> int:
    if isinstance(traj, Refusal):
        return _say(False, f"{traj.reason}: {traj.detail}")
    r = drv.move(traj)
    return _say(r.done, "moved" if r.done else r.why)


def cmd_park(a, site, rig) -> int:
    drv = _driver(site, rig, a.arm, a.fake)
    try:
        return _move(drv, tools.park_move(rig, a.arm, drv.state().q))
    finally:
        drv.close()


def cmd_jog(a, site, rig) -> int:
    drv = _driver(site, rig, a.arm, a.fake)
    try:
        q = drv.state().q
        target = tools.jog_target(rig, a.arm, q, a.joint, a.delta)
        return _move(drv, target if isinstance(target, Refusal)
                     else tools.straight(rig, a.arm, q, target))
    finally:
        drv.close()


def cmd_touch(a, site, rig) -> int:
    from aris_robot.touch import Kinematics, manual_touch
    drv = _driver(site, rig, a.arm, a.fake)
    try:
        if a.depth > tools.TOUCH_MAX:
            return _say(False, f"a touch descends at most {tools.TOUCH_MAX} m")
        kin = Kinematics.of(rig, a.arm)
        m = manual_touch(kin, drv.state().q, a.depth, a.extra)
        if isinstance(m, Refusal):
            return _say(False, f"{m.reason}: {m.detail}")
        r = drv.touch(m)
        if r.done:
            tip = kin.tip(r.q)
            print(json.dumps(dict(arm=a.arm, q=r.q.tolist(), tip_base=tip[0].tolist(),
                                  tip_table=rig.to_table(a.arm, tip)[0].tolist(),
                                  **drv.last_report)))
        return _say(r.done, "touched and back at the start" if r.done else r.why)
    finally:
        drv.close()


def cmd_switch(a, site, rig) -> int:
    drv = _driver(site, rig, a.arm, a.fake)
    try:
        why = drv.switch(a.controller)
        return _say(not why, f"{a.controller} has arm {a.arm}" if not why else why)
    finally:
        drv.close()


def cmd_recover(a, site, rig) -> int:
    drv = _driver(site, rig, a.arm, a.fake)
    try:
        r = drv.recover()
        return _say(r.done, f"arm {a.arm} may move" if r.done else r.why)
    finally:
        drv.close()


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
    op = Operator(remote, a.config, a.work, drivers, log_dir, stacks, rows=rows)

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
    b = sub.add_parser("bringup")
    b.add_argument("--out", default=str(REPO / "robot" / "generated"))
    sv = sub.add_parser("serve")
    sv.add_argument("--out", default=str(REPO / "robot" / "generated"))
    sv.add_argument("--work", default=str(REPO / "out" / "robot_jobs"))
    sv.add_argument("--log-dir", default=str(REPO / "out" / "operator"))
    sv.add_argument("--sim-speed", type=float, default=None,
                    help="simulated arms at this many times real time (inf: at once); no ROS")
    sv.add_argument("--fake-paper-mm", type=float, default=0.0,
                    help="with --fake or --sim-speed: the fake paper, mm above the nominal one")
    sub.add_parser("identify")
    r = sub.add_parser("run")
    r.add_argument("--job", required=True)
    r.add_argument("--work", default=str(REPO / "out" / "robot_jobs"))
    r.add_argument("--sim-speed", type=float, default=None,
                   help="simulated arms at this many times real time (inf: at once); no ROS")
    for name in ("park", "jog", "touch", "switch", "recover"):
        s = sub.add_parser(name)
        s.add_argument("arm", type=int)
        if name == "jog":
            s.add_argument("--joint", type=int, required=True)
            s.add_argument("--delta", type=float, required=True, help="rad")
        if name == "touch":
            s.add_argument("--depth", type=float, default=0.04, help="m, at most 0.06")
            s.add_argument("--extra", type=float, default=0.01,
                           help="m further, at 2 mm/s, if no contact by --depth")
        if name == "switch":
            s.add_argument("controller", choices=["trajectory", "impedance"])
    return p


def main(argv=None) -> int:
    a = parser().parse_args(argv)
    if getattr(a, "sim_speed", None) is not None and not a.sim_speed > 0:
        raise SystemExit("--sim-speed must be positive (inf allowed)")
    site = site_mod.load(a.site)
    rig = Rig.load(a.config)
    print(f"site {site.path}; server {site.server_url}; mounted {list(site.mounted)}"
          + ("; FAKE HARDWARE" if a.fake else ""))
    verbs = dict(bringup=cmd_bringup, identify=cmd_identify, run=cmd_run, park=cmd_park,
                 jog=cmd_jog, touch=cmd_touch, switch=cmd_switch, recover=cmd_recover,
                 serve=cmd_serve)
    return verbs[a.verb](a, site, rig)


if __name__ == "__main__":
    sys.exit(main())
