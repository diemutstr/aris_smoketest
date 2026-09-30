# arm_planner — one arm, a list of lines, the motions that draw them

**Job.** Everything one arm does in one phase. It takes the lines the arm should draw and the
obstacles it must avoid, and hands out the arm's motions one at a time, in order, and at the end
what it could not draw and why. It is the local planner followed by the sequencer; it knows
nothing about other arms except as obstacles, and never talks to a robot.

File: `aris/arm_planner.py`.

## In and out

`plan(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None, verify=None, batch=32)`

- **In:** the arm, the lines (pen-tip polylines in the arm's base frame, each with its pressure),
  the obstacles in the base frame (paper, steel, walls, parked arms), where the arm is, the
  drawing rules, where it should end (default: where it started). `workers`: processes for the
  local planner. `cache_dir`: where the local planner keeps its kinematic table (built once, 14 s
  of CPU, then read).
- `verify(motion, q_before) -> dict`: the independent checker, handed to the sequencer: a
  piece is drawn only if every motion of its group passes, else it is left over as
  `failed_check`; every motion handed on then carries the checker's word (`Motion.checked`).
  See sequencer.md, "The checker in the loop".
- **Out:** a generator of `Motion`s (drawing and free, each timed, each starting where the one
  before ended and ending where the arm can stand), which at the end returns every leftover: the
  local planner's (stretches it cannot draw: unreachable, blocked, too short) followed by the
  sequencer's (pieces it cannot fly to or time).
- `plan_all(...)` collects both. `plan_detailed(...)` also returns `PlanStats`: lines, pieces
  offered, the local planner's CPU and wall time, the time to the first motion and the total,
  CPU and wall, and the sequencer's `TourReport` (see sequencer.md). CPU counts this process and
  its finished worker processes.

## How it works: batches, nearest first

1. The lines are sorted by the distance from the pen tip at `q_start` to the line's nearest
   point and cut into batches of `batch` lines (32). All of them go to one pool of `workers`
   processes at once, in that order, so the pool plans the nearest lines first and keeps
   planning the later batches in the background while the arm draws. A batch is handed to the
   sequencer when all its lines are planned. (With one worker, a batch is planned when the
   sequencer asks for it.) `batch=None` plans every line before the tour starts, as before.
2. The sequencer chooses among the pieces it has; whenever fewer than `batch` pieces are left,
   it takes the next batch, waiting for it if it has not arrived.

**Why the tour does not depend on timing.** When the sequencer takes the next batch depends only
on how many pieces are still to draw, never on whether the batch is ready: if it is not, the
sequencer waits. So the same drawing gives the same tour with 1 or 8 workers, on a quiet or a
busy machine (a test slows the batches down on purpose and gets the same motions).

A caller that runs each motion as it comes and a caller that collects everything first use the
same code; the only difference is when the executor starts. Times reported by `plan` include
whatever the caller does between two motions; `plan_detailed` measures planning alone.

## What it cannot do

- Batches cost pen-up time on big drawings: the sequencer chooses its next piece among at most
  about 32 to 64 pieces, not among all lines (below: 16 to 19 % more pen-up time on 1 000 lines, 2 to 3 %
  more time on the rig). Not tuned (a larger `batch` trades a later first motion for less).
- It does not re-plan after a failure on the rig; the drawing server calls it again with the
  lines still to draw and where the arm is (DESIGN.md section 4).

## Batches against all at once (2026-09-30, 8 workers, machine load 4 to 10)

`tests/arm_cases.py --batches --big 1000`; "1 000 lines" are the first 1 000 lines of
`tests/big_cases.big()` wholly within 0.80 m of the arm's axis. First motion: wall time from the
call. Planning CPU counts the workers.

| arm, case | first motion, all at once / batches | planning CPU | planning wall | pen-up share | pen-up time | time on the rig |
|---|---|---|---|---|---|---|
| 13 word | 1.08 / 0.96 s | 6.7 / 6.2 s | 2.0 / 1.9 s | 0.262 / 0.262 | same | same |
| 13 100 lines | 5.41 / 3.12 s | 44.6 / 50.6 s | 12.6 / 11.9 s | 0.091 / 0.092 | +1.5 % | +0.1 % |
| 13 1 000 lines | 19.44 / 1.60 s | 184.6 / 221.2 s | 67.1 / 62.6 s | 0.141 / 0.163 | +18.8 % | +2.8 % |
| 31 word | 1.17 / 1.05 s | 7.2 / 6.5 s | 2.0 / 1.9 s | 0.277 / 0.277 | same | same |
| 31 100 lines | 5.87 / 3.23 s | 50.4 / 54.2 s | 14.2 / 13.2 s | 0.092 / 0.098 | +6.7 % | +0.7 % |
| 31 1 000 lines | 21.41 / 1.20 s | 203.5 / 225.3 s | 73.7 / 62.9 s | 0.146 / 0.165 | +15.9 % | +2.2 % |

- The first motion no longer grows with the drawing: 1.0 to 3.2 s for any size here (with the
  kinematic table already on disk; building it the first time adds about 5 s).
- Planning CPU with batches (measured at first, one job per line with the arm and obstacles
  sent every time) was 8 to 20 % higher. The pool is now `local.LinePool`: the arm, obstacles,
  rules, gates, settings and kinematic table go to each worker once, when it starts, and a job
  is only the line; it gives the same bunches as `local.plan` (digest test). What is left:
  1 000 lines, arm 13, load 12 to 13: 234.7 s all at once, 256.3 s in batches; of that, the
  tour 53.7 against 67.4 s (its free-space moves are longer: 1 090 against 841 s of moves) and
  the local planner 181 against 189 s. Between runs of the same case the CPU moves by up to
  25 % with the machine's load.
- Pen-up time: more than 5 % over all-at-once on arm 31's 100 lines (+6.7 %) and on both 1 000
  line sets (+16 to 19 %); the drawing time is the same, so the time on the rig grows 0.7 to
  2.8 %.


### When to take the next batch (`refill`), batches of 32 (2026-09-30, 8 workers, load 6 to 32)

Pen-up time and time on the rig against all at once; first motion wall time.

| arm, case | all at once: first motion | refill 32 | refill 64 | refill 128 |
|---|---|---|---|---|
| 13 100 lines | 5.5 s | 2.8 s; pen up +1.5 %, rig +0.1 % | 4.7 s; +0.6 %, 0.0 % | 5.0 s; +0.1 %, 0.0 % |
| 13 1 000 lines | 19.5 s | 1.8 s; +18.8 %, +2.8 % | 3.0 s; +12.2 %, +1.7 % | 4.2 s; +5.5 %, +0.8 % |
| 31 100 lines | 7.1 s | 4.1 s; +6.7 %, +0.7 % | 5.0 s; -0.5 %, +0.5 % | 6.6 s; 0.0 %, 0.0 % |
| 31 1 000 lines | 24.1 s | 1.0 s; +15.9 %, +2.2 % | 2.1 s; +20.2 %, +2.9 % | 3.9 s; +4.9 %, +0.5 % |

Refill 128 keeps the first motion under 5 s on 1 000 lines and the pen-up cost at 5.5 % or less
(under 1 % of the time on the rig). It is the default (chosen by the orchestrator 2026-09-30:
the time on the rig is what counts, and 4 s to the first motion is fine); 32 starts in 1 to 2 s
but costs up to 19 % of pen-up time.

## Measured (2026-09-30, branch aris3, one process, machine load 4 to 15)

Planning CPU and time to the first motion, from park back to park, kinematic table on (full
table of cases in sequencer.md):

| arm, case | lines | planning CPU / wall s | of which local planner CPU s | first motion CPU / wall s | motion on the rig s |
|---|---|---|---|---|---|
| 31 word | 13 | 2.3 / 2.3 | 1.6 | 1.6 / 1.7 | 166.0 |
| 31 scatter | 27 | 4.1 / 4.1 | 2.8 | 2.8 / 2.9 | 223.8 |
| 31 lines | 100 | 37.9 / 38.3 | 31.5 | 31.5 / 31.9 | 2420.7 |
| 13 word | 13 | 2.0 / 2.0 | 1.5 | 1.5 / 1.5 | 153.2 |
| 13 hatch | 43 | 17.9 / 18.2 | 14.7 | 14.8 / 15.1 | 1578.7 |
| 13 lines | 100 | 30.7 / 30.8 | 25.1 | 25.2 / 25.3 | 2386.9 |

- Planning is 1 to 2 % of the motion's duration; the arm never waits for the planner after the
  first motion.
- **Word, arm 31, against the old planner:** planned in 2.3 s of CPU against 66.8 s; first
  motion after 1.6 s. At the old drawing speed (80 mm/s) the motion takes 82.7 s against the old
  67.8 s; at the new rules' 20 mm/s, 166.0 s. The new tour lifts the pen 13 times and draws
  2.42 m of the word; 0.198 m of the last "n" lies beyond the arm's reach.
- With `workers` the local planner's wall time drops with the number of processes (see
  local.md); the sequencer runs in one process.

Tests: `tests/test_arm_planner.py`. Quick: two lines for arm 31 end to end (every motion joined,
timed, holdable; the same bits from a fresh process); the first motion is handed out while the
second piece is still unplanned. Slow: the word for both arms with every motion through the
independent checker (all must pass: 53 and 49 motions); ceilings from the numbers above. `tests/arm_cases.py` plans and
checks every case and draws the figure of sequencer.md.
