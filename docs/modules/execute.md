# execute: from a planned motion to an arm

**Job.** Run what the planners made, one arm at a time, and say exactly what happened. Planning
and execution never call each other: between them sits a queue of checked motions per arm and
phase, on disk. This round builds everything with a simulated arm; the real arm's driver
(`drivers/ros.py`, on the operator PC) is a later task.

Files: `aris/execute/`
- `queue.py`: the queue file, the job directory, content digests
- `executor.py`: one arm, one queue
- `coordinator.py`: a whole job, phase by phase
- `feed.py`: the planner's stream through the checker into the queues (the drawing server's
  job, here so the path can be run and tested whole)
- `log.py`: the event log
- `drivers/__init__.py`: the driver verbs; `drivers/sim.py`: the simulated arm

## The six laws, as built

1. **A motion is the only thing that crosses** (`aris.types.Motion`). Each starts where the
   previous motion of the same arm ended and ends where the arm can hold. The queue refuses a
   motion that starts more than 1e-9 rad from where the previous one ended.
2. **A queue per arm and phase, append-only, a file.** `Queue.append(motion, verdict)` takes
   a motion only with a verdict of the independent checker that passed, and stores the
   verdict's key numbers with it. `read()` gives every complete motion, `watch()` yields them
   as they appear and then the end marker, `close()` writes the end marker. A refusal
   (checker failed, not continuous, queue already closed) is returned, not raised.
3. **An executor per arm.** It takes the next motion, confirms the arm is able to move and
   stands at the motion's start (to `start_tol`, 5 mrad per joint for now), sends it (`draw`
   for a drawing motion, `move` for the rest), waits, logs it done, and goes on. Empty queue:
   the arm holds. The first failure ends the run with the motion's number and the reason;
   the arm stops and holds, the rest of its queue is not run. It never plans and knows nothing
   about other arms.
4. **A driver is a handful of verbs** for one arm: `state()` (joints, speeds, able to move,
   flags), `move(trajectory)`, `draw(motion)`, `hold()`, `stop()` (at once, then held),
   `recover()`. `move` and `draw` block and answer done, or failed with why.
5. **A coordinator per job.** For each phase it starts one executor per moving arm, waits
   until every one has reached its end marker and stands parked (standing still where its last
   motion ended), asks the checker's `check_phase_end` about every arm where it actually is,
   and starts the next phase. It reads the phases from the job as the writer adds them, so
   the arms can start while the planner is still working. `stop()` stops every arm at once;
   the result says where every arm is.
6. **No timing across arms.** Inside a phase each arm runs on its own clock; the walls keep
   them apart. The coordinator only waits at phase ends.

**When something goes wrong.** A failure stops only its own arm; the other arms of the phase
finish their queues, and then the job ends as failed, before the next phase (whose plan assumed
every arm back at its park). A queue whose plan was cut short (the checker refused one of its
motions) is marked so in its end marker and ends the job the same way. **A stopped or failed
job is finished: nothing resumes it. What is left to draw is a new drawing**, planned from where
the arms actually stand. (Pete has not decided otherwise; re-planning after a failure, DESIGN.md
section 4, is not built.)

## The job directory

| file | what is in it |
|---|---|
| `job.json` | the header: digests of the rig, the calibration and the drawing, the drawing rules, the time |
| `phases.jsonl` | the phases in the order they run (who moves, who stands parked, the walls), one per line, then an end line |
| `<phase>__arm<id>.queue` | one per phase and moving arm: the queue (below) |
| `events.jsonl` | the event log (below) |

**A queue file** is a sequence of records, each written in one piece and flushed to disk:

| part | size | contents |
|---|---|---|
| frame | 16 bytes | `ARQ1`, the payload length, a CRC-32 of the payload |
| payload | as stated | an uncompressed numpy `.npz`: `meta` (JSON text) and the arrays |
| first record | | `meta`: format, phase, arm |
| each motion | | `meta`: its number, kind, piece (line id and arc lengths), pressure, the checker's numbers (passed, the tightest measurement with its value and limit, the smallest clearance and where); arrays `t`, `q`, `qd` and, for drawing, `tip_base`, bit for bit as planned |
| last record | | the end marker: how many motions, complete or cut short, a note |

A reader that finds fewer bytes than a frame announces has met a motion still being written
and stops there, so it only ever sees complete motions. Only numpy and the standard library
are used.

## What the executor logs

One JSON line per state change, each with the time, the arm and the phase: `started`,
`holding` (queue empty), `motion started` (number, kind, duration), `motion done` (where the arm
is), `finished` (how many, parked or not, complete or not), `failed` or `stopped` (which motion,
why, where the arm is). The coordinator adds `job started`, `phase started`,
`phase end check` (passed, tightest, smallest clearance), `phase done`, `phase failed` and
`job done / failed / stopped` with where every arm is. Each line is one appending write, so the
drawing server can read the file while it grows.

## The simulated arm

`SimArm(arm_id, q0, speed=1.0, fail_at=None)` plays each trajectory in a background thread,
reading the cubic between the samples at `speed` times real time (`math.inf`: at once), and
lands exactly on the last sample. `fail_at` makes it fault at that second of its motion clock
(the seconds flown, at the trajectories' own timing): it stops where it is, holds, and refuses
to move until `recover()`. Like a joint trajectory controller, it refuses a trajectory that
starts more than 1 mrad from where it stands.

It does **not** simulate: contact with the paper or the pen force, compliance, any tracking
or controller error, braking (it stops dead), communication delays or dropouts, joint limits
or collisions (the checker has settled those before a motion is queued). It proves the
bookkeeping and the order of events, not the arm.

## What the real driver must provide

The same six verbs for one arm, on the operator PC, and nothing else knows about ROS: read the
joint states; send a whole trajectory to the arm's joint trajectory controller
(`follow_joint_trajectory`) with its own times, **never re-timed** (the checker's limits hold at
that timing only; lessons L44 to L46); for `draw`, switch to the impedance controller and add
the pen force along the same joint path, with the planned joints as the reference (the old
stack threw them away and re-solved, L8 and HARDWARE_DAY1 section 5.3); switch back for `move`;
report a controller or robot fault as a failure with the reason; `stop` at once and hold.
Hardware findings to respect: an arm that finishes holds where its queue ends, which is always
a certified holding pose (the park at a phase end; L13); arms of one phase need no shared clock
because the walls separate them, but an arm must never start the next phase before the
coordinator says so (L140).

## Measured (2026-09-30, machine load about 9)

- Quick set (`tests/test_execute.py -m "not slow"`), 10 tests in 2.1 s: a queue written and
  read back bit for bit, draw motions with pieces and tips included; the same watched from
  another thread while written; a record cut in half is not seen; three checked motions of arm
  13 (1.73 s of motion) run at 50x in 0.04 s and land on the last configuration to 1e-9; an
  injected fault halfway through motion 1 stops the executor at motion 1 with its reason, and
  the arm holds there; an arm 20 mrad off on joint 5 is refused before anything moves; the
  coordinator runs two phases on arms 13 and 2 while the job is still being written, both
  phase-end checks pass, both arms end at their parks to 1e-9; a stop holds every arm
  mid-motion; a queue cut short ends the job after its phase.
- Slow set, the word "unknown" end to end: planned by the system planner (all of it goes to
  arm 71 in phase 1: 53 motions, 98.1 s of motion; the other five arms have nothing to do and
  hold at their parks), every motion checked and queued while six simulated arms run at 20x.
  Planned, checked and queued in 29.1 s wall (drawable maps and kinematic table built from
  scratch); the first motion started 24.4 s in, and the job was done 5.1 s later, 0.5 s after
  the last motion was queued. The phase-end check passed (tightest: arm 31 against the steel,
  78.6 mm beyond the demanded clearance). Every arm ends at its park to 1e-9, and the queues on
  disk equal what was planned, bit for bit. Numbers in `tests/data/execute_word.npz`.

## What it cannot do (yet)

- No re-planning after a failure, no resume after a stop (see above).
- The start tolerance (5 mrad) is a constant here; it should come from the rig (`rig.json`)
  once measured on the real arms.
- `feed` checks the motions one after another in one process (0.1 to 0.6 s each); for large
  drawings the drawing server should check in parallel and queue in order.
- No real driver yet: `drivers/ros.py` is built on the operator PC.
