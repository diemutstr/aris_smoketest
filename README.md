# Aris: the six-arm drawing rig

Six Franka FR3 arms hang upside down from a frame over a paper-covered table and draw with
pens. This repository is the clean rebuild of the planning and execution software (branch
`aris3`). The old code lives on branch `aris2` only; its documentation is kept under
`legacy_docs/` because it is the record of what the old planner got wrong.

This page says how to use it: which computer runs what, how to install each, what a drawing
day looks like as commands, the drawing file, where the results end up, and what is not
built yet. The design is in `docs/DESIGN.md`, the layout and rules in `docs/BUILD.md`, the
state of each module in `docs/STATUS.md`, and every module has one page under `docs/modules/`.

## 1. Two computers, one repository

| | planning PC | operator PC |
|---|---|---|
| where | anywhere on the rig's network | next to the arms, wired to the six control boxes |
| software | Python only, no ROS | ROS 2 Jazzy with the vendored franka_ros2, plus the same Python package |
| runs | the drawing server: `aris serve` | one ROS launch per mounted arm, and the runner: `aris-robot run` |
| what it does | reads the drawing, fits it to the drawing area, plans every arm, checks every motion with the independent checker, writes the queues of motions, keeps the job log and the report | fetches the queues, plays them on the arms with the pen-force controller, posts back every event |
| the only thing it knows about the arms | the rig description (`config/rig.json`) and the calibration files | the addresses and which arms are mounted (`robot/site.json`) |
| talks to | the operator PC over HTTP, port 8420 | the planning PC (the server's URL in `site.json`) and the arms over FCI |

Both machines run the **same commit** of this repository (`git log -1` on both must agree;
the job header carries the rig digest and the runner refuses a job whose rig differs from its
own). Everything else in the repository, tests, the Meshcat viewer, the docs, runs on any
machine with the Python package installed. The lab station where this was built has no arms
and no ROS.

Without any arms at all, the planning PC runs the whole thing by itself: `aris serve` with the
default simulated arms plays every job in software, and `aris plan` plans and checks a
drawing without a server.

## 2. Install

**Planning PC** (and any development machine):

```
git clone git@github.com:wernerpe/aris_smoketest.git aris3 && cd aris3 && git checkout aris3
python3 -m venv .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime   # g++ and cmake needed
pip install -e ".[gui,dev]"
python -m pytest tests -q -m "not slow"
```

The three `native/` packages are compiled; without them the same code runs on numpy
fallbacks, slower. The Meshcat viewer and the checker's tests need `pip install drake`.

**Operator PC**: the same Python install on top of a ROS 2 Jazzy shell, plus the ROS
workspace in `robot/ros2_ws`. The steps, and the checks that could not be made here, are in
`robot/README.md`. In short: source ROS and the franka_ros2 workspace, `colcon build` the
three packages under `robot/ros2_ws/src`, make the virtual environment with
`--system-site-packages` so it sees rclpy, `pip install -e . && pip install -e robot`, then
edit `robot/site.json` (server URL, control-box IPs, which arms are mounted).

## 3. A drawing day, as commands

On the **planning PC**, once, in a terminal that stays open:

```
aris serve --host 0.0.0.0 --driver robot --uncalibrated
```

`--uncalibrated` is needed until the calibration files exist (every report then says so and
the pen is kept 20 mm above the paper when lifted). Leave out `--driver robot` to draw with
simulated arms instead of the real ones.

**Arms are named by slot** — the place on the frame they hang from: `1L 1R 2L 2R 3L 3R` (row 1
at the −y end, L at −x). Which robot (serial, address) hangs in which slot is one table,
`site/aris_2026-10.json`; nothing else in the code knows a robot.

**Which slots are controlled** is a fact of the rig file. `config/rig.json` is the full rig; the
arms that are actually driven today are in `config/two_arms/` (slots 2R and 3R, written from
the full rig by `tools/mounted_rig.py`, drawing area 0.36 × 2.0 m about (0.40, 0.605)). Use it
with `--config config/two_arms` on every `aris` command; the empty hangers stay as steel, the
phases and walls follow from the controlled slots, and the slots whose arms hang there switched
off are fenced: a plane halfway toward them (toward row 1, and the x = 0 plane between the
columns) that every controlled arm's whole body stays behind in every phase and job — the dead
arms themselves are not modelled. When more arms go up, run the tool again with the new list,
centre and area (the maps allow 0.40 × 2.23 m about that centre for these two; the server
refuses an area larger than the maps allow).

On the **operator PC** one process runs, `aris-robot serve`, installed once as a systemd
service (`robot/aris-robot.service`, `robot/README.md` section 8). It brings up and keeps up the
ROS stack of every mounted arm, takes every command from the planning PC's server over the
network (run a drawing, park or calibration job; recover an arm; report), fetches the
calibration files from the server before each job and reports where the arms stand while idle.
Nobody types anything on that PC after it is installed; the e-stop is the only thing on that
side. (The hardware-day tools `aris-robot bringup / identify / touch / jog` exist for the first
runs, with serve stopped.)

Before the first drawing, once per slot and again whenever an arm or the frame was moved; and
one touch after every pen switch or handling of the pencil:

```
aris calibrate 2R                     # touches the paper on a grid: height, roll, pitch -> base part of config/calibration/2R.json
aris calibrate 3R
aris touchoff 2R                      # one touch at a reference point: the pen's length -> pen part of the file
aris touchoff 3R
```

Drawing runs in **mode A** by default: joint position control, the plan 3.5 mm below the paper
(the pen's `press_m` in rig.json), 15 mm/s on the paper — the recipe that drew the first word on
the rig. `aris serve --tracking impedance` chooses mode B (the pen-force controller).

Then a drawing, from the planning PC or any machine that reaches the server
(`--server http://<planning pc>:8420`):

```
aris draw drawings/today.json --note "4H on 120 g paper"   # submits, prints a line on every change, then the report
aris draw --rest-of <job id>          # what a stopped job left, as a new drawing
aris status                           # the current or last job, any time
aris stop                             # every arm stops at once and holds; the job is finished
aris park                             # every arm back to its park, one at a time (pens lifted first)
aris recover 2R                       # after a fault, once a person has looked
aris rig                              # what the server runs: arms, drawing area, calibration state
```

The operator PC picks every accepted job up by itself and posts back what happens.

To try a drawing without any arms: `aris plan drawings/today.json --uncalibrated` plans it,
checks every motion and prints the report; `aris check <job dir>` runs the checker again on
every queued motion of a job.

## 4. The drawing file

One JSON file, one pen, positions in the table frame (origin at the table centre on the
paper, x across the table, y along it, millimetres or metres):

```
{"units": "mm", "frame": "table",
 "lines": [{"id": "a", "points": [[-100, 0], [100, 0]]},
           {"id": "b", "points": [[0, -100], [0, 100]], "intensity": 0.8}]}
```

Every line has a distinct id and at least two points; `intensity` (0 to 1, how hard to
press) is optional. The arms can reach a rectangle of 1.56 by 3.56 m around the table
centre (the drawing area, `aris rig` prints it); a drawing that sticks out is scaled
uniformly about the centre until it fits, and refused if that would halve it. Anything the
arms cannot draw is not an error: it comes back in the report as left over, stretch by
stretch, with a reason (out of reach, blocked by a wall or a parked arm, too short, ...).

## 5. Where things end up

Everything a job produces is in one directory, `out/jobs/<job id>/` on the planning PC
(`--jobs` moves it):

| file | what it is |
|---|---|
| `job.json` | the header: rig and calibration digests, the fit, the pen, the tracking mode, your note |
| `drawing.json` | the drawing as planned (after the fit) |
| `phases.jsonl` | the phases, in order: who moves, who stands parked |
| `<phase>__<slot>.queue` | the motions of that slot in that phase, in order, each one checked |
| `events.jsonl` | the log: job states, every motion started and ended, every stop and fault, from both machines |
| `report.json` | drawn and left over, by line and by reason; the same thing `aris draw` prints |
| `refused/<phase>__<slot>__<n>.npz` | every motion the checker refused while planning, with where the arm stood and what the checker said (the piece is then left over or drawn by a later phase) |

`out/jobs/operator.jsonl` is what the operator PC said outside any job (positions, its arm
stacks, commands taken). `config/calibration/<slot>.json` has two parts: `base` (height, roll,
pitch — later x, y, yaw too) written by `aris calibrate`, and `pen` (the pen's length) written by
`aris touchoff`; every later job reads it, and the operator PC fetches these files, it never has
its own.

The operator PC keeps a copy of each job it ran under `out/robot_jobs/<id>/`. `out/cache/`
holds the drawable maps and the kinematic table (built on first use, minutes; reused as long
as the rig does not change). The rig itself is one file, `config/rig.json`: arm poses, steel,
clearances, gates and rules, the drawing area; the calibration files will sit next to it.

To look at a plan in 3D, `tools/animate_meshcat.py` plays the six arms in Meshcat
(`--still` for the parked rig alone); today it plays the built-in test drawings, not a job
directory.

## 6. Not built yet

- **Calibration beyond the plane**: `aris calibrate` finds each slot's height, roll and pitch
  and `aris touchoff` the pen's length. A slot's position on the table and its turn (1–2 cm off
  today) stay nominal until the dimple plates and the pin exist (Pete's hardware); until then the
  wall clearance is 40 mm and every job needs `--uncalibrated` when a slot has no file.
- **First run on the operator PC**: nothing under `robot/` has been built against the real
  ROS headers or run on an arm yet. The order is fake hardware first, then one real arm, then
  the touch, then a drawing in the air.
- **Followers**: only the three leaders of each phase draw; the followers stay parked.
- **A GUI**, and playing a job directory in the Meshcat viewer.
- **SVG input**: drawings are JSON polylines; the converter from SVG is not written.

## Layout

```
aris/             the package: types, rig, planners, checker, execution, server, command line
native/           the three compiled parts (inverse kinematics, collision, timing)
config/           the rig description and, later, the calibration files
robot/            the operator PC side: ROS packages, controller, the aris-robot command
tests/            tests, fixed test sets, reference data
docs/             design, build plan, status, one page per module
tools/            the Meshcat viewer and the rig drawing
assets/           the installation model (URDF, meshes), the vendor arm description, drawings
legacy_docs/      the old planner's documentation: decisions, lessons, audits, drawings
```
