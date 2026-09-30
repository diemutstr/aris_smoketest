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
| `GET /arms` | each arm's joints, speeds, whether it can move, its driver's flags, whether it is at its park |

## The command

| command | what it does |
|---|---|
| `aris serve [--host --port --driver sim --speed --uncalibrated --cache --jobs]` | start the server (default `127.0.0.1:8420`) |
| `aris draw <drawing> [--server URL]` | submit, print a progress line whenever something changes, then the report; exit code 0 only if everything was drawn |
| `aris status`, `aris stop`, `aris park`, `aris rig` | the current or last job; stop it; park all arms; the rig |
| `aris plan <drawing> [--out dir]` | plan and check only, no server, no arms; writes the job directory and prints the report |
| `aris check <job dir>` | the checker again on every queued motion, one line per motion |

Every command prints its assumptions once (rig digest, calibration digest and state, driver,
speed) and ends with one PASS or FAIL line. `--cache` (default `out/cache`) keeps the drawable
maps and the kinematic table; `--map-grid` (default 2 cm) is the maps' grid; `--workers` and
`--check-workers` the processes for planning and checking.

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

The planner runs in its own thread and hands over motions as they come. Each is checked at
once in a pool of processes, from where that arm's previous motion ended, and queued strictly
in the planner's order. A phase is written when its first motion arrives, so the arms start
while the planner still works. A leader phase lists the leaders' row partners as moving too:
a follower draws against its leader's footprint and is checked in its own view of the phase
with that footprint (saved next to its queue so `aris check` can repeat it); a follower with
nothing to do holds. A motion the checker refuses is not queued; the rest of that arm's phase
is dropped, and the job fails after that phase.

**Park all arms.** One arm at a time in rig order, each in its own phase, so no two arms move
at once: a free motion from where it stands to its park, planned around the others (parked
ones at their parks; ones not yet parked as their bodies where they stand, and for the checker
as their footprints), checked, queued, run. An arm already at its park is left alone. An arm
the planner or checker refuses stays; the job then fails and says which.

## The report

| field | meaning |
|---|---|
| `state`, `why` | done, stopped or failed, and why |
| `drawing` | lines, scale, bounding box before and after the fit |
| `length_m`, `drawn_m` | the fitted drawing's length; what motions that ran to the end drew |
| `left_m`, `left_by_reason`, `leftovers` | everything not drawn, as stretches of lines with a reason: the planner's (unreachable, blocked, too short, ...), `failed_check`, `stopped` or `failed` (queued and not run, or not planned yet). Drawn plus left over is the whole drawing; on a done job anything else would show as `unaccounted` |
| `checker` | motions checked and passed, the tightest clearance beyond the demanded one and where |
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
- Quick tests: 9 in 40 s.

## What is not built

- Resume after a stop or a failure; re-planning after a failure.
- SVG drawings.
- Calibration jobs (only "park all arms" is built of the other kinds of job).
- The real arm driver: every arm is the simulated arm, in the server's own process.
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
