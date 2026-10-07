# server — the drawing server and the `aris` command

**Job.** The one front door. You give it a drawing; it fits the drawing to the area the arms
can reach, plans it, checks every motion with the independent checker, queues it and runs it
on the arms, and tells you as it goes what is happening. At the end it says what was drawn and
what was not, and why. It is the only part that touches both the planners and the arms; the
command line (and later the GUI) talk only to it.

Files: `aris/server/`: `drawing.py` (reading and fitting a drawing), `jobs.py` (job state and
store), `pipeline.py` (planner -> checker -> queues), `park.py` (park all arms), `runner.py`
(a job in the background, stop), `report.py` (what was drawn and what was not), `recheck.py`
(`aris check`), `station.py` (the rig, calibration, drivers, knobs), `server.py` (the web
app). `aris/cli/` is the command (one module per kind of command).

## Before it starts

Arms are named by their slot on the frame, `1L 1R 2L 2R 3L 3R` (row 1 at the −y end, L at −x):
in queue names, event rows (`"arm": "2R"`), endpoints, commands and reports. The old robot ids
map 13→1L, 17→1R, 31→2L, 71→2R, 2→3L, 97→3R.

It loads the rig. If any slot's calibration is not applied in both parts (base and pen; today
none is), it refuses to start unless it is started with `--uncalibrated`; then every job report
says "UNCALIBRATED: every arm runs on its nominal pose and pen". Only the simulated arm driver is
built (`--driver sim`, the default); `--speed` sets how many times faster than real time the
simulated arms play.

**One tracking mode: position control.** The operator PC flies every motion through the
joint-trajectory controller and the press is geometric (the drawing surface lies the pen's
press below the paper, or below the paper's height map when there is one). Every job header
carries the pen that is in (`rig.pen()`: its entry of
rig.json's pens table with its name and press) and the person's note (`aris draw --note
"4H on 120 g paper"`), so a job describes itself on both machines; the report repeats them.

**`--driver robot`**: the arms are on the operator PC (`robot/`). The server then runs no
executors: it plans, checks and writes the queues, and the operator PC's runner copies them
(the last four endpoints below), runs them and posts its event log back. The job's state
follows those events: it ends with the runner's own "job done / failed / stopped". A stop
here is passed on in the answer to the runner's next post; the server waits up to 30 s for
the runner to confirm it (not at all if no runner ever reported). The server keeps the newest
joints the runner reported per arm (`GET /arms`); park plans from them. Times in the report
then come from the operator PC's clock.

**Never planned from zeros.** A reading that is no reading — `"q": null` (the arm's stack is
down), joints all zero, or a reading older than 60 s — means "position unknown". A job that
needs that slot's start is refused with "no joint states for 2R (FCI off?)"; `aris arms`
shows "no reading". The phase-end check (in the coordinator, on whichever machine runs it)
asks again for up to 10 s for a slot that reads all zeros, and fails the phase with the same
words if no reading comes, instead of judging the arm at q = 0.

## A job

One job at a time; a second one while one runs is refused.

| drawing job | park job | meaning |
|---|---|---|
| received | received | the job directory exists |
| fitted | | the drawing was read and fitted to the drawing area |
| planning | planning | the planner is working; nothing moves yet |
| drawing | moving | the first motion is queued and the arms run; planning goes on meanwhile |
| done / stopped / failed | same | finished; the report is written |

Every change is a "job state" line in the job's event log, next to the lines the arms and the
coordinator write. Each job has its own directory under `--jobs` (default `out/jobs`): the
header, the phase list, one queue per phase and arm, the event log and `report.json`.

**Stop**, at any time: every arm stops at once and holds, the planner is abandoned, and the
job is finished. What was not drawn is reported as left over, reason "stopped". **Nothing
resumes a stopped job** — Pete has not decided on resume; what is left would be a new drawing.
The next job releases the stopped arms (the stop was the operator's own). An arm with a fault
refuses every job; clearing a fault is not built in the server.

**Arms away from their park.** The planner can start only the arms of phase 1 from where they
stand. If any other arm is away from its park, the drawing job fails with "park all arms first".

## Endpoints

| endpoint | what it does |
|---|---|
| `POST /jobs?name=...&note=...` | the drawing file (JSON) as the body; answers the job id; 409 if a job runs, 400 if the file is bad |
| `POST /jobs?rest_of=<id>` | a new drawing of what that finished (done, stopped, or failed: a link drop mid-job) job left over; 409 while it runs or when nothing is left |
| `GET /jobs` | every job of this server run |
| `GET /jobs/{id}` | state; the fitted drawing (bounding box before and after, scale); per phase and arm: motions queued, done, the current motion, what the planner handed back so far, checker refusals; at the end the report |
| `GET /jobs/{id}/events` | the event log |
| `POST /jobs/{id}/stop` | stop (409 if already finished) |
| `POST /park` | park all arms |
| `GET /rig` | slots (pose, park configuration, calibration state of both parts, and each file's parts with their dates), the drawing area and its centre, the pen that is in, the rig and calibration digests, the paper height map (points, height range, date), driver and speed |
| `GET /arms` | each arm's joints, speeds, whether it can move, its driver's flags, whether it is at its park; with `--driver robot`, per controlled slot: the robot it named, the joints the operator PC last reported or `null` with why (`reading`: fresh, no joint states, too old, never reported), at its park (`null` without a reading), when (its clock), how long ago the server heard it, and the job |
| `GET /jobs/{id}/header` | the operator PC: the job's header (`job.json`), with the rig and calibration digests it checks against its own |
| `GET /jobs/{id}/phases?offset=B` | the operator PC: the phase list from byte B on, held open while it grows, closed after its end line |
| `GET /jobs/{id}/queues/{phase}/{slot}?offset=B` | the operator PC: that queue file from byte B on, byte for byte, held open while it grows, closed after its end marker; 404 until it exists |
| `POST /jobs/{id}/events` | the operator PC: `{source, rows: [{seq, ...}]}`; each row appended to the job's log once, in seq order; answers `{accepted, next_seq, stop}` (stop: the job was stopped here) |
| `POST /calibrate/{slot}` | the calibrate job for one slot (below): the file's `base` part; 409 if a job runs |
| `POST /mark?slots=2L,2R&group=rows12` | the mark job (below), for the slots named or a group (default: the group "all", else every controlled slot) |
| `POST /touchoff/{slot}` | the touch-off job (below): the file's `pen` part |
| `GET /operator/next?wait=30` | the operator PC: the oldest command it has not acknowledged, or 204 after the wait: `run` a job, `recover` an arm, `report` |
| `POST /operator/ack` | the operator PC: `{"id": n}`, the command was taken (it is given again until then) |
| `POST /operator/rows` | the operator PC: `{source, rows}`, what it says outside any job (started, where, report, recovered, stack died, ...); kept in `operator.jsonl` beside the jobs; every `where` or `q` updates the arm positions |
| `GET /operator`, `POST /operator/report` | the commands waiting, when the operator PC was last heard, its stacks, its last rows; ask it to report |
| `POST /arms/{slot}/recover` | release an arm after a fault, once a person has looked: the simulated arm at once; with `--driver robot`, a `recover` command for the operator PC |
| `GET /calibration`, `GET /calibration/{slot}` | `{"arms": [...], "files": [...]}`: the slots that have a calibration file under the server's config, each file with a digest and its parts' pass flags and dates; one file as it is (404 if none) |

## The command

| command | what it does |
|---|---|
| `aris serve [--host --port --driver sim\|robot --speed --sim-paper dz_mm,roll,pitch --uncalibrated --cache --jobs --config]` | start the server (default `127.0.0.1:8420`); `--sim-paper` gives the simulated arms a paper that is not where the rig says |
| `aris draw <drawing> [--note ...] [--server URL]` | submit, print a progress line whenever something changes, then the report; exit code 0 on PASS |
| `aris draw <drawing> --air 30` | an air run, the validation every first drawing on the hardware starts with: the whole job planned and checked with the drawing surface 30 mm above the paper (the planner gets a press of −30 mm, the checker `surface_z` = paper + 30 mm), so every draw is flown in the air; the header says `air_mm`, the report starts with AIR RUN; `aris plan --air` too. |
| `aris arms` | a table: slot, robot, joints or "no reading", at park, reported when; FAIL when a slot has no reading |
| `aris draw --rest-of <job id>` | draw what that stopped, failed or finished job left over (a job that failed before anything was accounted: the whole drawing): its leftover stretches as lines `<line>#rest` (`#rest2`, ... when a line has several), not refitted |
| `aris status`, `aris stop`, `aris park`, `aris rig` | the current or last job; stop it; park all arms; the rig |
| `aris calibrate <slot>` | touch the paper on a grid with that slot's arm: the `base` part of its calibration file |
| `aris mark [slots ...] [--group all\|row2\|rows12\|rows23]` | the mark job: guide each arm's pen onto its marks; x, y, yaw and the pen tip |
| `aris touchoff <slot>` | one touch at the slot's reference point: the `pen` part (after every pen switch or handling of the pencil) |
| `aris recover <slot>` | release an arm after a fault |
| `aris plan <drawing> [--out dir]` | plan and check only, no server, no arms; writes the job directory and prints the report |
| `aris check <job dir>` | the checker again on every queued motion, one line per motion |

Every command prints its assumptions once (rig digest, calibration digest and state, driver,
speed) and ends with one PASS or FAIL line. For `draw` and `plan`, PASS means the job ran to
its end and every queued motion carries a passing check; the line then gives the drawn and
left-over lengths by reason. Leftovers are a report, not a failure. `--cache` (default `out/cache`) keeps the drawable
maps and the kinematic table; `--map-grid` (default 2 cm) is the maps' grid; `--workers` the planner's processes (the checker runs inside them); `aris check` takes
`--check-workers`.

## The drawing file

```
{"units": "mm", "frame": "table",
 "lines": [{"id": "a", "points": [[x, y], [x, y], ...], "intensity": 0.8}, ...]}
```

Units "mm" or "m"; the frame must be "table" (origin at the table centre, on the paper, x
across, y along). `intensity` (0 to 1, how hard to press; clamped into that range) is
optional, default 1. One pen. A repeated id is renamed (`a#2`, ...); every line needs at least
two points.

**SVG** (`aris/server/svg.py`, by `svgelements` from PyPI): `aris draw pic.svg --width 1.70
[--at X Y]` turns every shape's outline (paths, lines, polylines, polygons, rectangles,
circles, ellipses; Béziers and arcs flattened so no chord strays more than 0.5 mm; every
transform applied) into lines `svg1`, `svg2`, ... of the format above: the picture's bounding
box `--width` metres along the table's x, centred on `--at` (default: the drawing area's
centre), the SVG's y (down the page) along the table's −y. Text is not drawn (convert it to
paths). The job's `drawing.json` is that drawing as fitted. `aris import pic.svg --width 1.70
-o pic.json` writes the JSON without drawing. `aris draw ... --pen NAME` is refused when the
rig has another pen in (its length and press are what is planned).

## The fit rule

The drawing area is rig.json's rectangle (`canvas.drawing_area_m`) around its centre
(`canvas.drawing_area_centre_m`, `Rig.drawing_area_centre_m`): the system planner refuses
anything outside it. The server works out the same rectangle from the drawable maps at start;
when rig.json's is larger by more than one grid cell (or the maps are empty about the centre:
no mounted arm draws there), drawing jobs are refused with that reason (`no_drawing_area`) and
`aris rig` says NO DRAWING, while park, calibrate and marks still run. Smaller is allowed on
purpose. `/rig` shows both and the centre. `tools/mounted_rig.py --arms 1R --out DIR` without
`--area` computes an area and centre from the mounted arms' maps. If a point of
the drawing lies outside the area, the whole drawing is scaled uniformly **about the area's
centre** until it fits;
the scale is in the job state and the report, however small. A drawing that fits is not
touched. The drawing is not
moved, only scaled: a small drawing near an edge shrinks toward the area's centre.

## Planning, checking, queueing

The checker runs inside the planning loop. The server hands the system planner a `verify`
(`aris/server/verify.py`): it runs the independent checker on a motion, in whichever planner
process calls it, and answers the checker's numbers. The arm planner checks every motion of a
piece's group (the move to it, lower, draw, lift) before it builds on them; a piece whose
group does not all pass is left over as `failed_check` with the checker's word, and the arm
plans on from where it was. A refused motion is saved in the job directory (below).

So every motion the planner hands over already carries a passing check (`Motion.checked`),
and the server queues it at once, in the order it came; the queue itself refuses a motion
without one ("nothing unchecked is ever queued"). A phase is written when its first motion
arrives, so the arms start while the planner still works. `aris check` re-checks every
queued motion offline (it keeps its own `--check-workers`).

**Park all arms.** From where every arm stands: with the simulated arms, as they report it;
with `--driver robot`, as the operator PC last reported it (every event row about an arm
carries its joints, the runner's first and last rows carry every arm's). An arm that never
reported refuses the park job by name ("no position reported for arm 2R"); the runner itself
refuses to move an arm that is not at the start of its first motion, so a stale position
cannot move an arm the wrong way.
1. An arm whose pen stopped within its clearance of the paper (rig.json's lifted-pen clearance,
   plus 5 mm; as after a stop mid-drawing) first raises it straight up by that clearance plus
   5 mm (the sequencer's lift-off rule). A pen stopped between the drawing surface and that
   height (in the middle of a set-down or a lift-off) is first set down onto the surface at the
   landing speed, so the lift-off starts where a lift-off starts. A stop can leave an arm closer
   to a joint limit than planning would choose; if the planning gates refuse the rise, it is
   tried once more with the joint-limit margin halved (0.075 rad; the checker still holds the
   real limits). Always one arm per phase ("lift pens 1L", "lift pens 1R", ...), in rig order,
   every other arm standing (the phase-end check accepts a standing pen at the paper that a
   later phase raises). An arm whose pen cannot rise stays and does not park.
2. Then one arm at a time in rig order, each in its own phase ("park 1L", ...): a free motion
   to its park, planned around the others (parked ones at their parks; ones not yet parked as
   their bodies where they stand, and for the checker as `standing` joints), checked, queued,
   run. An arm already at its park is left alone. An arm the planners or the checker refuse
   stays; the job then fails and says which.

The drawing job's "park all arms first" check uses the same positions (with the robot, an arm
within the start tolerance, 5 mrad, of its park counts as parked; one that never reported is
taken to be at its park, and the runner refuses to move it if it is not).

## The calibrate job

Step 1 of the calibration (DESIGN.md section 6): the paper under one arm, found with the arm's
own pen and joints. `aris calibrate 2R`.

1. From where every arm stands, as the park job sees it: the arm need not be parked (a pen at
   the paper first rises, as in park); the other arms are parked arms or, where they are not
   at their parks, bodies where they stand.
2. Grid points inside the drawing area and within 0.6 m of the arm's axis (5 x 5 before the
   cut). One hand spin for all of them, pen upright: the 24 spins 15 degrees apart are tried
   in order and the first that reaches every point any spin reaches is taken; a point no spin
   reaches is dropped and named in the report. The spin never changes between points, so an
   unknown pen length moves every touch alike and the tilt stays exact.
3. At each point: a free move to a hover 60 mm above the nominal paper, then a `touch`: the
   pen straight down to the nominal paper and back up (an IK answer every 2 mm at the same hand
   orientation, the sequencer's rise rule), 5 mm/s along the line, with 20 mm of extra depth
   for the paper's uncertainty. Back to the park at the end. One phase, "calibrate 2R".
4. The checker checks every motion, a touch as a touch (its descent to the real paper and its
   climb; the press is a drawing matter, not a probe matter). The extra depth is declared, not
   checked.
5. The arm stops each touch where it meets the paper and logs a "contact" row with its joints.
   The solver (`aris.calib.calibration_from_events`) fits the plane; a passing result is
   written as the `base` part of `calibration/<slot>.json` under the server's `--config`
   (`aris.calib.files.write_base`; the `pen` part is kept) and the server reloads the rig, so
   `aris rig` shows the part and its date and the next drawing uses it. The report
   gives the points touched and dropped, the spin, the residuals (RMS, worst, each), the tilt,
   roll, pitch and height change, and pass or fail.
6. A touch that meets no paper within the extra depth is not a fault: the executor logs "no
   contact" and goes on, the point is named in the report, and the job fails only if fewer
   than the plane fit's minimum (`aris.calib.plane.MIN_POINTS`, 8) remain. Every contact
   is used: the robot's touch detector arms only once the descent runs at constant speed, so a
   trip early in the descent is real contact.

## The mark job

Steps 2 and 3 of the calibration (DESIGN.md section 6), as `docs/figures/mark_protocol.png`
draws it: `aris mark` once, then the arms' lights and the pilot buttons.

- **Who.** The slots named, or a group's (`--group all | row2 | rows12 | rows23`; default the
  group "all", or every controlled slot when the rig has no such group), in rig order, one arm
  at a time, the others standing (parked, or given to the checker as their joints).
- **What each arm touches.** Its marks among the group's (`rig.marks_for(slots)`, the ones it
  shares); a slot sharing fewer than two marks within the job (a single slot, or a group cut
  through a pair) touches every mark it shares with any controlled slot. Those need places
  solved by an earlier run: if none is solved the job is refused before anything moves ("2L
  alone needs marks solved by an earlier run; run `aris mark` (2L, 2R) first"), so the solver
  never gets an empty set: at its first mark six hand orientations — the pivot: the pen upright and five
  tilts of 30 degrees, 72 degrees apart (turned by up to 60 degrees where a tilt is out of
  reach; tilting, not only turning, is what pins the pen length) — and one upright touch at
  every other mark. The first mark is the first in rig.json order where at least three of the
  tilts can be reached, else the next. Planning is `mark_plan.py`; `mark.py` runs the job.
- **Planned up front.** For each touch a free move to the hover (the pen 30 mm above the mark,
  `rig.mark_xy`: the solved position when there is one), by way of 80 mm up when turning the
  pen near the paper is cramped, then a `guide` there; then home. Every motion is checked
  (the guide stands at the hover, which the checker held as the move's end), the arm's whole
  phase is queued at once.
- **The guide.** The driver hands the arm to the person (light white), who seats the pen on the
  mark and presses a pilot button; the executor writes the "registered" row (joints, button,
  mark); the driver lifts the pen 3 cm and returns to the hover itself, so the next move starts
  there. ✓ check: the touch counts. ✗ cross: never reaches the server (the driver waits for the
  next button). ○ circle: the touch is marked skipped for the solver; the rest runs. A guide
  whose hand-over fails fails the job; the arm holds at its hover.
- **After each arm.** The pivot's own fit (`aris.calib.marks.pivot`): a touch it names as off
  the common point gets one small extra phase ("mark 2L again": to that hover, the guide,
  home); then the next arm.
- **At the end.** The joint solve over every arm of the group (`aris.calib.solve_marks`). A full
  calibration (every controlled slot) solves the marks afresh; a smaller group or single slots
  keep the marks solved before as known (`rig.mark_state`; a single arm needs two solved marks,
  else "needs a partner"). The frame is the fit of the solved arms onto their nominal
  mountings; the marks are found wherever they were taped (2 to 5 cm off nominal is normal), the base parts' tips for the height. When it
  passes, `aris.calib.files.write_mark_solution` writes every slot's `base` (method "marks")
  and `pen` (the pivot's tip) and `calibration/marks.json`, and the rig reloads. When it fails,
  the report's why names the slot, mark or touch and **nothing is written**. The report gives
  per slot the move and yaw against the job's start, the distance from rig.json's nominal axis,
  the tip change, the pivot's residuals and the slot's RMS; per mark its position, state,
  residual and who touched it; per pair of arms their disagreement; the buttons, skips and
  redone touches.

**In simulation** the person is simulated (`aris/server/simtruth.py`): in a "true" world
(`aris serve --sim-truth <config dir>`, or `--sim-base-error 3,2`: every base 3 mm and 2 mrad
off in x, y, yaw, the quantities the marks find), they seat the true pen tip on the true mark in
the hover's hand orientation (turned a little and with another elbow where it must be), 0.3 mm
off, and press ✓ (tests script ✗, ○ and a failed hand-over); `--sim-mark-error 3.5` tapes every
true mark 3.5 cm off its nominal place. The report prints each mark's solved position and its
offset from nominal.

## The touch-off job

`aris touchoff 2R`, after every pen switch or handling of the pencil: with a geometric press
the pen's length is the tone. One touch, planned like one point of the calibrate job (hover,
touch, home), at the slot's reference point: the `reference_touch` in its calibration file when
the `pen` part exists; else the grid point nearest the slot's axis that the arm can touch (the
one straight under the shoulder often cannot be), which then becomes the reference. The
contact goes to the calib solver (`aris.calib.touchoff`), which measures the tip against the
slot's measured paper; a passing result is written as the file's `pen` part
(`aris.calib.files.write_pen`; `base` is kept) and the rig reloads. The report gives the
reference and where it came from, the correction against the nominal pen length, the change
against the tip before, and the height the tip was believed at.

**The simulated paper.** `aris serve --sim-paper -12,0,1` gives the simulated arms a paper
12 mm low at the table centre and turned 1 degree about table y. A touch then meets that
paper where its planned descent, carried straight on, crosses it (joints found by bisection
along the descent), and the calibration can be tested end to end.

## The operator channel

The operator PC runs one resident process (`aris-robot serve`, DESIGN.md section 4) that pulls
its work: `GET /operator/next?wait=30` answers the oldest command it has not acknowledged —
`run` a job (with `--driver robot` every admitted job: drawing, park, calibrate), `recover` an
arm (`aris recover <arm>`), `report` — or nothing after the wait; `POST /operator/ack` marks it
taken. What the operator PC says outside any job (`POST /operator/rows`) goes to
`operator.jsonl` beside the job directories, and every arm position in it (`where`, `q`)
updates the positions park and calibrate plan from. The operator PC needs no open port; the
server stays the only front door. The commands live in the server's memory: a restarted server
starts with none.

**The server owns the calibration files.** They are written only by the calibrate job, under
the server's `--config`. The operator PC fetches them (`GET /calibration` lists them with a
digest of each file; `GET /calibration/{arm}` gives one) before every run, and removes any of
its own files the server does not have: the server's set is the truth.

## Where things end up

One directory per job under `--jobs` (for `aris plan`, the `--out` directory):

| file | what is in it |
|---|---|
| `job.json` | the header: rig, calibration and drawing digests, rules, the pen that is in (`rig.pen()`: name, press, force rules, which the operator PC applies), the note, the drawing area and its centre, scale, `rest_of`, driver, speed |
| `drawing.json` | the drawing as planned (after the fit), so its leftovers can be drawn again (`--rest-of`) |
| `phases.jsonl` | the phases in the order they run, then an end line; a park or calibrate phase also names the arms standing still off their parks (`standing`: {slot: joints}), which `aris check` gives the checker |
| `<phase>__<slot>.queue` | the checked motions of one arm in one phase (format: execute.md) |
| `refused/<phase>__<slot>__<n>.npz` | every motion the checker refused while planning: `t`, `q`, `qd`, `tip_base` (drawing), `kind`, `piece` (line id and arc lengths, JSON), `intensity`, `q_before`, the failed measurements and the whole verdict as text |
| `operator.jsonl` (beside the job directories) | the operator PC's rows outside any job |
| `events.jsonl` | every state change: the job's, the coordinator's, each arm's (with `--driver robot`, the operator PC's rows, marked `source: robot`) |
| `report.json` | the report below |

## The report

| field | meaning |
|---|---|
| `state`, `why` | done, stopped or failed, and why |
| `note`, `pen`, `rest_of`, `paper_under_drawing` | the person's note, the pen that was in, the job this one drew the rest of, the paper map's height range under the drawing (null: flat paper) |
| `drawing` | lines, scale, bounding box before and after the fit |
| `length_m`, `drawn_m` | the fitted drawing's length; what motions that ran to the end drew |
| `left_m`, `left_by_reason`, `leftovers` | everything not drawn, as stretches of lines with a reason: the planner's (unreachable, blocked, too short, ...), `failed_check`, `stopped` or `failed` (queued and not run, or not planned yet). Drawn plus left over is the whole drawing; on a done job anything else would show as `unaccounted` |
| `checker` | motions checked inside the planners, how many were refused (and their files in `refused/`), whether every queued motion carries a passing check, the tightest clearance beyond the demanded one and where |
| `passed` | the verdict: the job ran to its end and every queued motion was checked |
| `phases` | each phase-end check, passed, and its tightest clearance |
| `first_motion_s`, `planning_s`, `total_s` | from the job arriving to the first motion starting; planning; the whole job |
| `where`, `at_park` | where every arm ended |
| `assumptions` | rig and calibration digests and state, uncalibrated or not, driver, speed |

A park job's report says per arm "parked", "already at its park" or why not.

## Measured (2026-09-30, machine load about 5, simulated arms)

- **The word "unknown"** (13 lines, 1.296 m) through `aris draw` against a live `aris serve`
  (subprocess, 8 planner and 8 checker processes, arms at 20 x real time, empty cache): the
  server was up in 15 s (drawable maps built). Job: first motion after **6.6 s**, planning
  done at 11.4 s, **done at 13.4 s**. All drawn by arm 71 (now slot 2R) in phase 1; 53 of 53 motions passed
  the checker; phase-end check passed (tightest: arm 31 (2L) against the steel, 78.6 mm beyond the
  demanded clearance); every arm back at its park.
- Small drawing (two short lines, arms 13 and 71, now 1L and 2R; 5 cm maps, no cache): first motion 7.2 s,
  done at 7.9 s, 10 of 10 motions pass.
- Park all arms from random configurations up to 0.05 rad from their parks (five arms to move):
  planned and checked in 7.9 s, done at 8.2 s (then; the standing arms' footprints took most of it,
  and are gone since: the checker now builds the standing arms' bodies itself)
  (1.1 s each).
- **Very big drawings** (`tests/big_cases.py`; `aris plan`, 30 planner and 30 checker
  processes, warm cache, machine load about 4): 2 000 lines of 1.2 to 1.5 m (2 680 m): 17 min
  wall, 19 384 motions, all pass, 8 mm left over (unreachable), 648 MB of queues, first motion
  at 86 s; through the server at speed inf: 16 min, first motion 84 s. 10 000 lines of 2 cm to
  1 m (1 912 m): 17 min wall, 42 296 motions, one refused by the checker, which dropped the rest
  of that arm's phase (243 m). The checker takes more CPU (8 000 to 10 000 s) than the planner
  (3 500 to 4 500 s). `aris check` on the 2 000 lines: 12 min. Peak memory of all processes
  together: 10 to 13.5 GB (the 30 checker processes all along, plus 30 local-planner processes
  at the start of each leader phase).
- **The checker inside the planners** (same 10 000 lines, `aris plan`, 30 processes, arm
  planner in batches of 32 nearest first): 29.7 min wall, 6 700 s CPU in all (half the
  earlier 14 600), 36 processes, 10.3 GB peak; first motion after 4 to 11 s in every phase;
  42 304 motions all checked and queued, 1 refused (its piece drawn in a later phase), 1 911.95
  of 1 912.01 m drawn, 65 mm out of reach; tightest 0.2 mm. Through `aris serve` at speed
  inf: 27.2 min, the same drawn length, first motion 10 s. Each arm now checks its own motions
  one after another, so a leader phase takes twice as long on the wall clock as before.
- Quick tests (server, operator PC): 14 in 41 s; slow: 3 in 48 s.

## What is not built

- Resume after a stop or a failure; re-planning after a failure.
- SVG drawings.
- Steps 4 and 5 of the calibration (the pen length against the pin, the drawn check); the
  mark job (steps 2 and 3) is built.
- The real arm driver in this process: with `--driver sim` every arm is simulated here; with `--driver robot` the operator PC runs them.
- Pause.
- Clearing an arm's fault from the server.
- Only the arms of phase 1 may start a drawing away from their parks.

## Tests

`tests/test_server.py`. Quick: the file format and the fit (inside untouched, scaled, refused
however far; bad files refused, a repeated id renamed); the server refuses to start uncalibrated or with an unbuilt
driver; the rig and arms endpoints; a small drawing runs to done while a second job and a park
are refused, with the state events in order; `aris plan` and `aris check`; a drawing outside
the area is scaled, and a stop in the middle of drawing leaves every arm stopped and holding
with the rest left over as "stopped"; a refused drawing file makes `aris draw` fail; `aris
park` parks every arm from a random near-park configuration and `aris check` confirms the park
queues. Slow: the word through `aris draw` against a live `aris serve` at 20 x.

The mark job (`tests/test_server_mark.py`, simulated person, real executor, queues and solver):
on the two-arm rig with every base 3 mm and 2 mrad off and both marks 3.5 cm off nominal, 14
touches (a ✗ handled inside the driver), the files written, the seam (2R seen from 2L) within
0.15 mm and 0.06 mrad of the truth, each mark found within 1.5 mm of where it was taped (34
and 36 mm from nominal), the rig reloaded with base and pen applied; a ○ on 2R's second mark:
the solve refuses ("needs a partner"), nothing written, both arms home; a failed hand-over: the
job fails, the arm holds at its hover; a touch named bad: one extra phase; `--group rows12` on
the six-slot rig: 32 touches, four slots applied, 3L untouched. 6 tests, 32 s.

The site-day fixes (`tests/test_server_field.py`): `aris mark 2L` on a fresh two-arm rig is
refused before moving, and after a full run touches A and B (7 touches) with the marks known;
a null, all-zero or 60 s old reading is no reading, refuses park, mark and a drawing with "no
joint states for 2R (FCI off?)", and shows in `aris arms`; the phase-end check waits for a
reading rather than judging zeros; `aris arms` on the simulated arms; the rest of a job that
failed before it moved.
8 tests (2 slow).

This round adds: slots everywhere (queue names, rows, endpoints); the fit about the area's
centre; the header's pen and note; `--rest-of` (a job stopped midway, its leftovers
drawn as a new drawing, the two drawn lengths adding up to the whole within the shortest
piece); the touch-off after the calibrate job on the two-arm rig (the pen part written beside
the base part, the next touch-off at the remembered reference); park after a stop with a pen
hovering (set down, then lifted).

## Refusals (reviewed 2026-10-07)

Kept, each for a real hazard or something that cannot work: no fresh joint reading / never
reported (planning from an unknown position); not parked outside the first phase (the plan
assumes them parked); an arm fault or not ready (a person must look); uncalibrated without
`--uncalibrated` (wrong geometry); a motion the checker refused; a job from another server run
(`/operator`, 409: stale work between machines); one job at a time (two jobs on one arm); the
pen named is not the pen in (its length and press); `--air` below zero (a deeper press, not
air); no drawing area the maps cover (the planner could plan nothing); the rig not loading, an
unbuilt driver, a speed not above zero; a drawing file that cannot be read (not JSON, units,
frame, a line without two finite points, no lines); rest-of a job still running, unknown, not
a drawing, or with nothing left; mark slots or a group the rig does not have, a slot with no
mark, a single slot whose marks no earlier run solved; calibrate a slot not on the rig.

Removed: the shrink-below-half limit (scaled however far, the report says how much); the
200 mm ceiling on `--air`; two lines with one id (renamed); an intensity outside 0..1
(clamped); stopping a finished job (nothing to do, 200); pens down on row partners (they rise
one arm per phase); the stale-rig start refusal (now refuses drawings only, and says when the
maps are empty).
