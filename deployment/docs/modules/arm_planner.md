# arm_planner — one arm, a list of lines, the motions that draw them

**Job.** Everything one arm does in one phase. It takes the lines the arm should draw and the
obstacles it must avoid, and hands out the arm's motions one at a time, in order, and at the end
what it could not draw and why. It is the local planner followed by the sequencer; it knows
nothing about other arms except as obstacles, and never talks to a robot.

File: `aris/arm_planner.py`.

## In and out

`plan(arm, lines, obstacles, q_start, rules, q_end=None, workers=1, cache_dir=None)`

- **In:** the arm, the lines (pen-tip polylines in the arm's base frame, each with its pressure),
  the obstacles in the base frame (paper, steel, walls, parked arms), where the arm is, the
  drawing rules, where it should end (default: where it started). `workers`: processes for the
  local planner. `cache_dir`: where the local planner keeps its kinematic table (built once, 14 s
  of CPU, then read).
- **Out:** a generator of `Motion`s (drawing and free, each timed, each starting where the one
  before ended and ending where the arm can stand), which at the end returns every leftover: the
  local planner's (stretches it cannot draw: unreachable, blocked, too short) followed by the
  sequencer's (pieces it cannot fly to or time).
- `plan_all(...)` collects both. `plan_detailed(...)` also returns `PlanStats`: lines, pieces
  offered, the local planner's CPU and wall time, the time to the first motion and the total,
  CPU and wall, and the sequencer's `TourReport` (see sequencer.md). CPU counts this process and
  its finished worker processes.

## How it works

1. The local planner plans every line (`workers` processes, kinematic table from `cache_dir`).
   Lines do not depend on each other; this is the only step that waits for all of them.
2. The sequencer builds the tour from the bunches and hands out each motion as soon as the
   piece it belongs to is settled. So the first motion exists once the local planner is done
   plus the time for one piece, not after the whole tour.

A caller that runs each motion as it comes and a caller that collects everything first use the
same code; the only difference is when the executor starts. Times reported by `plan` include
whatever the caller does between two motions; `plan_detailed` measures planning alone.

## What it cannot do

- The first motion waits for the local planner on every line. Handing the sequencer the lines
  as the local planner finishes them would start the arm sooner (the first piece is usually a
  near one; that needs a sequencer that can wait for more pieces).
- It does not re-plan after a failure on the rig; the drawing server calls it again with the
  lines still to draw and where the arm is (DESIGN.md section 4).

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
