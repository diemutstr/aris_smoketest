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
app). `aris/cli.py` is the command.

## Before it starts

It loads the rig. If any arm has no passing calibration file (today: none has), it refuses to
start unless it is started with `--uncalibrated`; then every job report says
"UNCALIBRATED: every arm runs on its nominal pose and pen". Only the simulated arm driver is
built (`--driver sim`, the default); `--speed` sets how many times faster than real time the
simulated arms play.

**`--driver robot`**: the arms are on the operator PC (`robot/`). The server then runs no
executors: it plans, checks and writes the queues, and the operator PC's runner copies them
(the last four endpoints below), runs them and posts its event log back. The job's state
follows those events: it ends with the runner's own "job done / failed / stopped". A stop
here is passed on in the answer to the runner's next post; the server waits up to 30 s for
the runner to confirm it (not at all if no runner ever reported). The server keeps the newest
joints the runner reported per arm (`GET /arms`); park plans from them. Times in the report
then come from the operator PC's clock.

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
| `POST /jobs?name=...` | the drawing file (JSON) as the body; answers the job id; 409 if a job runs, 400 if the file is bad |
| `GET /jobs` | every job of this server run |
| `GET /jobs/{id}` | state; the fitted drawing (bounding box before and after, scale); per phase and arm: motions queued, done, the current motion, what the planner handed back so far, checker refusals; at the end the report |
| `GET /jobs/{id}/events` | the event log |
| `POST /jobs/{id}/stop` | stop (409 if already finished) |
| `POST /park` | park all arms |
| `GET /rig` | arms (pose, park configuration, calibration state), the drawing area, the rig and calibration digests, driver and speed |
| `GET /arms` | each arm's joints, speeds, whether it can move, its driver's flags, whether it is at its park; with `--driver robot`, the joints the operator PC last reported, when (its clock), when the server heard it, and the job |
| `GET /jobs/{id}/header` | the operator PC: the job's header (`job.json`), with the rig and calibration digests it checks against its own |
| `GET /jobs/{id}/phases?offset=B` | the operator PC: the phase list from byte B on, held open while it grows, closed after its end line |
| `GET /jobs/{id}/queues/{phase}/{arm}?offset=B` | the operator PC: that queue file from byte B on, byte for byte, held open while it grows, closed after its end marker; 404 until it exists |
| `POST /jobs/{id}/events` | the operator PC: `{source, rows: [{seq, ...}]}`; each row appended to the job's log once, in seq order; answers `{accepted, next_seq, stop}` (stop: the job was stopped here) |
| `POST /calibrate/{arm}` | the calibrate job for one arm (below); 409 if a job runs |
| `GET /operator/next?wait=30` | the operator PC: the oldest command it has not acknowledged, or 204 after the wait: `run` a job, `recover` an arm, `report` |
| `POST /operator/ack` | the operator PC: `{"id": n}`, the command was taken (it is given again until then) |
| `POST /operator/rows` | the operator PC: `{source, rows}`, what it says outside any job (started, where, report, recovered, stack died, ...); kept in `operator.jsonl` beside the jobs; every `where` or `q` updates the arm positions |
| `GET /operator`, `POST /operator/report` | the commands waiting, when the operator PC was last heard, its stacks, its last rows; ask it to report |
| `POST /arms/{id}/recover` | release an arm after a fault, once a person has looked: the simulated arm at once; with `--driver robot`, a `recover` command for the operator PC |
| `GET /calibration`, `GET /calibration/{arm}` | `{"arms": [...], "files": [...]}`: the arms that have a calibration file under the server's config, each file with a digest; one file as it is (404 if none) |

## The command

| command | what it does |
|---|---|
| `aris serve [--host --port --driver sim\|robot --speed --sim-paper dz_mm,roll,pitch --uncalibrated --cache --jobs --config]` | start the server (default `127.0.0.1:8420`); `--sim-paper` gives the simulated arms a paper that is not where the rig says |
| `aris draw <drawing> [--server URL]` | submit, print a progress line whenever something changes, then the report; exit code 0 on PASS |
| `aris status`, `aris stop`, `aris park`, `aris rig` | the current or last job; stop it; park all arms; the rig |
| `aris calibrate <arm>` | touch the paper with that arm and write its calibration file |
| `aris recover <arm>` | release an arm after a fault |
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
across, y along). `intensity` (0 to 1, how hard to press) is optional, default 1. One pen.
Ids must be distinct; every line needs at least two points. The same content is accepted as a
`.npz` file by `aris draw` and `aris plan` (`lines` an object array); the command turns it
into JSON before sending, because the server never unpacks pickles from the network. **SVG is
not read in this round.**

## The fit rule

The drawing area is the rectangle centred on the table that the system planner accepts,
worked out from its drawable maps (1.56 x 3.56 m at the 2 cm grid). That is the law. rig.json
carries the same rectangle (`canvas.drawing_area_m`); `/rig` shows both, and the server refuses
to start when they differ by more than one grid cell ("the rig file is stale"). The check is
live once `rig.py` reads that entry (as `Rig.drawing_area_m`); until then `/rig` shows it as
not read. If a point of the drawing lies
outside it, the whole drawing is scaled uniformly **about the table centre** until it fits;
the scale is in the job state and the report. A drawing that fits is not touched. A drawing
that would shrink below half its size is refused (the job fails at once). The drawing is not
moved, only scaled: a small drawing near an edge shrinks toward the centre.

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
arrives, so the arms start while the planner still works. A leader phase lists the leaders'
row partners as moving too: a follower draws against its leader's footprint and is checked in
its own view of the phase with that footprint (saved next to its queue so `aris check` can
repeat it); a follower with nothing to do holds. `aris check` re-checks every queued motion
offline (it keeps its own `--check-workers`).

**Park all arms.** From where every arm stands: with the simulated arms, as they report it;
with `--driver robot`, as the operator PC last reported it (every event row about an arm
carries its joints, the runner's first and last rows carry every arm's). An arm that never
reported refuses the park job by name ("no position reported for arm 71"); the runner itself
refuses to move an arm that is not at the start of its first motion, so a stale position
cannot move an arm the wrong way.
1. An arm whose pen stopped within its clearance of the paper (rig.json's lifted-pen clearance,
   plus 5 mm; as after a stop mid-drawing) first raises it straight up by that clearance plus
   5 mm (the sequencer's lift-off
   rule). All such arms rise together in one phase, "lift pens", behind the walls they were
   drawing behind (every phase-end check fails while any pen is down).
2. Then one arm at a time in rig order, each in its own phase ("park 13", ...): a free motion
   to its park, planned around the others (parked ones at their parks; ones not yet parked as
   their bodies where they stand, and for the checker as their footprints), checked, queued,
   run. An arm already at its park is left alone. An arm the planners or the checker refuse
   stays; the job then fails and says which.

The drawing job's "park all arms first" check uses the same positions (with the robot, an arm
within the start tolerance, 5 mrad, of its park counts as parked; one that never reported is
taken to be at its park, and the runner refuses to move it if it is not).

## The calibrate job

Step 1 of the calibration (DESIGN.md section 6): the paper under one arm, found with the arm's
own pen and joints. `aris calibrate 31`.

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
   for the paper's uncertainty. Back to the park at the end. One phase, "calibrate 31".
4. The checker checks every motion; a touch as its descent (a lower) and its climb (a lift),
   since the checker knows no touch. The extra depth is declared, not checked.
5. The arm stops each touch where it meets the paper and logs a "contact" row with its joints.
   The solver (`aris.calib.calibration_from_events`) fits the plane; a passing result is
   written as `calibration/<arm>.json` under the server's `--config` and the server reloads
   the rig, so `aris rig` shows the arm calibrated and the next drawing uses it. The report
   gives the points touched and dropped, the spin, the residuals (RMS, worst, each), the tilt,
   roll, pitch and height change, and pass or fail.
6. A touch that meets no paper within the extra depth is skipped and named; the job fails only
   if fewer than 9 contacts remain. (This needs the executor to log "no contact" and go on;
   today a missed touch still stops the arm, and the job then fails naming the point.)

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
| `job.json` | the header: rig, calibration and drawing digests, rules, the pen's force rules (rig.json `pen`, which the operator PC applies), scale, driver, speed |
| `phases.jsonl` | the phases in the order they run, then an end line |
| `<phase>__arm<id>.queue` | the checked motions of one arm in one phase (format: execute.md) |
| `<phase>__arm<id>.check.npz` | where a queue is not checked in its named phase (a follower, a park): that phase and the footprints |
| `refused/<phase>__arm<id>__<n>.npz` | every motion the checker refused while planning: `t`, `q`, `qd`, `tip_base` (drawing), `kind`, `piece` (line id and arc lengths, JSON), `intensity`, `q_before`, the failed measurements and the whole verdict as text |
| `operator.jsonl` (beside the job directories) | the operator PC's rows outside any job |
| `events.jsonl` | every state change: the job's, the coordinator's, each arm's (with `--driver robot`, the operator PC's rows, marked `source: robot`) |
| `report.json` | the report below |

## The report

| field | meaning |
|---|---|
| `state`, `why` | done, stopped or failed, and why |
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
  done at 11.4 s, **done at 13.4 s**. All drawn by arm 71 in phase 1; 53 of 53 motions passed
  the checker; phase-end check passed (tightest: arm 31 against the steel, 78.6 mm beyond the
  demanded clearance); every arm back at its park.
- Small drawing (two short lines, arms 13 and 71; 5 cm maps, no cache): first motion 7.2 s,
  done at 7.9 s, 10 of 10 motions pass.
- Park all arms from random configurations up to 0.05 rad from their parks (five arms to move):
  planned and checked in 7.9 s, done at 8.2 s; the standing arms' footprints take most of it
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
- Steps 2 to 5 of the calibration (dimples, pen length, the drawn check).
- The real arm driver in this process: with `--driver sim` every arm is simulated here; with `--driver robot` the operator PC runs them.
- Pause.
- Clearing an arm's fault from the server.
- Only the arms of phase 1 may start a drawing away from their parks.

## Tests

`tests/test_server.py`. Quick: the file format and the fit (inside untouched, scaled, refused
below half; bad files refused); the server refuses to start uncalibrated or with an unbuilt
driver; the rig and arms endpoints; a small drawing runs to done while a second job and a park
are refused, with the state events in order; `aris plan` and `aris check`; a drawing outside
the area is scaled, and a stop in the middle of drawing leaves every arm stopped and holding
with the rest left over as "stopped"; a drawing too big to fit makes `aris draw` fail; `aris
park` parks every arm from a random near-park configuration and `aris check` confirms the park
queues. Slow: the word through `aris draw` against a live `aris serve` at 20 x.
