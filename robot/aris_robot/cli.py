"""The operator PC's one command: `aris-robot <verb>`.

  bringup   write the launch arguments and controller settings of every arm
  identify  which arm answers on which address and domain, and where it stands
  run       fetch a job's queues from the server and run them on the mounted arms
  park      straight to the park, only from close by (else the server's park job)
  jog       one joint by a little
  touch     the calibration touch: descend with zero force until the pen meets the paper
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
    drv = _driver(site, rig, a.arm, a.fake)
    try:
        q0 = drv.state().q
        traj = tools.descent(rig, a.arm, q0, a.depth)
        if isinstance(traj, Refusal):
            return _say(False, f"{traj.reason}: {traj.detail}")
        r, report = drv.touch(traj)
        if r.done:
            tip = rig.arm(a.arm).tip(r.q[None])
            print(json.dumps(dict(arm=a.arm, q=r.q.tolist(), tip_base=tip[0].tolist(),
                                  tip_table=rig.to_table(a.arm, tip)[0].tolist(), **report)))
            back = drv.move(tools.straight(rig, a.arm, r.q, q0))   # up the way it came
            if not back.done:
                return _say(False, f"touched, but the way back failed: {back.why}")
        return _say(r.done, "touched" if r.done else r.why)
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


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aris-robot", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--site", default=str(REPO / "robot" / "site.json"))
    p.add_argument("--config", default=str(REPO / "config"))
    p.add_argument("--fake", action="store_true", help="the arms run on fake hardware")
    sub = p.add_subparsers(dest="verb", required=True)
    b = sub.add_parser("bringup")
    b.add_argument("--out", default=str(REPO / "robot" / "generated"))
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
                 jog=cmd_jog, touch=cmd_touch, switch=cmd_switch, recover=cmd_recover)
    return verbs[a.verb](a, site, rig)


if __name__ == "__main__":
    sys.exit(main())
