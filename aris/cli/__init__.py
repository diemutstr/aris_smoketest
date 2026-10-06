"""The one command: `aris ...`.

    aris serve  [--host --port --driver sim --speed --uncalibrated --cache --jobs]
    aris draw   <drawing.json> [--note ..] [--server URL]   submit, follow, report; exit
                                                     0 on PASS; --rest-of <job> draws its leftovers
    aris status | stop | park | rig | arms   [--server URL]
    aris calibrate <slot>                            touch the paper on a grid: the base part
    aris touchoff <slot>                             one touch: the pen part
    aris recover <slot>                              release an arm after a fault
    aris mark [slots | --group g]                    the marks: guide the pens, x/y/yaw and tip
    aris plan   <drawing> [--out dir]                plan and check only: no server, no arms
    aris check  <job dir>                            the checker again on every queued motion

Every command prints its assumptions (rig and calibration digests, calibration state, driver,
speed) once and one PASS or FAIL line at the end.  Only `serve`, `plan` and `check` touch the
planners or the checker; the others talk to the server only.

One module per kind of command: jobs.py (draw, park, status, stop, plan, check),
calibration.py (calibrate, touchoff, mark, recover), rig.py (rig, serve); common.py holds what
they share (the server over HTTP, the output lines, following a job, printing its report).
"""
from __future__ import annotations

import argparse

from aris.cli.calibration import cmd_calibrate, cmd_mark, cmd_recover, cmd_touchoff
from aris.cli.common import (CONFIG, DEFAULT_SERVER, Http, _summary, assumptions_line, follow,
                             job_passed, report_lines, say, verdict)
from aris.cli.jobs import cmd_check, cmd_draw, cmd_park, cmd_plan, cmd_status, cmd_stop
from aris.cli.rig import cmd_arms, cmd_rig, cmd_serve

__all__ = ["main", "parser", "Http", "job_passed", "report_lines", "follow", "say", "verdict",
           "assumptions_line", "_summary"]


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aris", description="the six-arm drawing rig")
    sub = p.add_subparsers(dest="cmd", required=True)

    def local(s):
        s.add_argument("--config", default=str(CONFIG), help="the rig's config folder")
        s.add_argument("--uncalibrated", action="store_true",
                       help="run on the nominal poses when calibration files are missing")
        s.add_argument("--cache", default="out/cache",
                       help="drawable maps and kinematic table ('none': build every time)")
        s.add_argument("--workers", type=int, default=None, help="planner processes")
        s.add_argument("--map-grid", type=float, default=0.02,
                       help="m between the drawable maps' grid points")

    s = sub.add_parser("serve", help="run the drawing server")
    local(s)
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=8420)
    s.add_argument("--driver", default="sim",
                   help="sim: simulated arms here; robot: the operator PC runs the arms")
    s.add_argument("--speed", type=float, default=1.0, help="simulated arm: times real time")
    s.add_argument("--tracking", default="position", choices=("position", "impedance"),
                   help="how the operator PC flies motions: position (mode A, the default) or "
                   "impedance (mode B, the pen force)")
    s.add_argument("--sim-truth", default=None, metavar="CONFIG_DIR",
                   help="the simulated arms' true rig for the mark job (another config folder)")
    s.add_argument("--sim-base-error", default=None, metavar="MM,MRAD",
                   help="the simulated arms' true bases: every base moved by this much")
    s.add_argument("--sim-mark-error", type=float, default=None, metavar="CM",
                   help="the simulated marks: every one taped this far from its nominal place")
    s.add_argument("--sim-paper", default=None,
                   help="simulated arms' paper: dz_mm,roll_deg,pitch_deg against the nominal")
    s.add_argument("--jobs", default="out/jobs", help="where the job directories go")
    for name, what in (("draw", "submit a drawing and follow it"),):
        s = sub.add_parser(name, help=what)
        s.add_argument("drawing", nargs="?", default=None)
        s.add_argument("--note", default="", help="the material and anything else: kept in "
                       "the job's header and report")
        s.add_argument("--air", type=float, default=0.0, metavar="MM",
                       help="an air run: the whole job planned this far above the paper (no "
                       "contact), to validate the plan's geometry and timing first")
        s.add_argument("--rest-of", default=None, metavar="JOB",
                       help="draw what that finished job left over")
    for name, what in (("calibrate", "touch the paper on a grid: the slot's base calibration"),
                       ("touchoff", "one touch at the reference point: the slot's pen calibration"),
                       ("recover", "release an arm after a fault, once a person has looked")):
        s = sub.add_parser(name, help=what)
        s.add_argument("arm", help="the slot, e.g. 2R")
    for name, what in (("status", "the current or last job"), ("stop", "stop the job"),
                       ("park", "park all arms"), ("rig", "the rig the server runs"),
                       ("arms", "each slot: robot, joints or no reading, at park, when")):
        sub.add_parser(name, help=what)
    s = sub.add_parser("mark", help="calibrate x, y and yaw by guiding the pens onto the marks")
    s.add_argument("slots", nargs="*", help="the slots (default: the group)")
    s.add_argument("--group", default=None, help="all, row2, rows12, rows23 (default: all)")
    for s in (sub.choices[n] for n in ("draw", "status", "stop", "park", "rig", "arms",
                                       "calibrate",
                                       "touchoff", "recover", "mark")):
        s.add_argument("--server", default=DEFAULT_SERVER)
        s.add_argument("--poll", type=float, default=0.5, help=argparse.SUPPRESS)
    s = sub.add_parser("plan", help="plan and check a drawing; no server, no arms")
    s.add_argument("drawing")
    s.add_argument("--out", default=None, help="the job directory to write")
    s.add_argument("--air", type=float, default=0.0, metavar="MM",
                   help="plan an air run: the drawing surface this far above the paper")
    local(s)
    s = sub.add_parser("check", help="check every motion of a job's queues again")
    s.add_argument("job")
    s.add_argument("--config", default=str(CONFIG))
    s.add_argument("--check-workers", type=int, default=8)
    return p


COMMANDS = dict(arms=cmd_arms, serve=cmd_serve, draw=cmd_draw, status=cmd_status, stop=cmd_stop,
                park=cmd_park, rig=cmd_rig, plan=cmd_plan, check=cmd_check,
                calibrate=cmd_calibrate, recover=cmd_recover, touchoff=cmd_touchoff,
                mark=cmd_mark)


def main(argv=None, http=None) -> int:
    """`http`: an object with `get(path)` and `post(path, body)` answering (status, JSON), in
    place of HTTP to `--server` (tests)."""
    a = parser().parse_args(argv)
    if http is None and hasattr(a, "server"):
        http = Http(a.server)
    return COMMANDS[a.cmd](a, http)
