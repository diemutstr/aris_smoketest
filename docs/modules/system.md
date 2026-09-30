# system — which arm draws which line, in which phase

**Job.** The system planner takes a whole drawing and decides which arm draws each line, in
which phase and behind which walls. It runs the arm planners and hands out their motions as
they come, each tagged with its phase and arm. At the end it says what was not drawn and why.
It only ever calls the arm planner. This is step 1 of Pete's scheme (DESIGN.md section 3):
leaders only, followers parked.

Files: `aris/system/`:
- `planner.py`: the one call
- `phases.py`: the phases
- `maps.py`: the drawable maps
- `area.py`: the drawing area
- `allocate.py`: the laws of who draws what
- `run.py`: the arm planners of a phase, in parallel
- `account.py`: the no-drop account
- `stretch.py`: part of a line
- `settings.py`: the knobs

## In and out

`plan(rig, lines, rules=None, arm_configs=None, cache_dir=None, workers=1)`

- **In:**
  - the rig;
  - the drawing: lines in the table frame, each with its own id;
  - the drawing rules: default `rig.rules()`, from rig.json, which is the only source;
  - where each arm stands: default, its park;
  - a directory for the drawable maps and the local planner's kinematic table (none: built
    every time);
  - how many processes to use.
- **Out:** a generator of `(phase name, arm id, Motion)`, in the order the arm planners produce
  them. At the end it returns the leftovers.
  - Every drawing motion's `piece` is in the input line's own id and arc length.
  - `plan_all` collects everything; `plan_detailed` also returns a report: per phase and arm,
    times, lengths and what came back; the phases skipped and why; the drawing area.
  - `phase_named(rig, name)` gives the `Phase` a name stands for, which is what the checker
    needs.
- **Refusals:**
  - A drawing with any point outside the drawing area (below) is refused before anything is
    planned. The generator yields nothing and returns a `Refusal("outside_drawing_area")`
    naming the line, the point and the area.
  - An arm away from its park that does not move in phase 1 is an error.

## The laws

1. **A phase is one entry of a fixed list.** Every arm that moves in it plans alone, from its
   park back to its park, against its own obstacles: the paper, its steel, the walls next to
   it and the parked arms.

   | phase | moving | parked | walls |
   |---|---|---|---|
   | 1 | 13, 71, 2 | 17, 31, 97 | 13-71, 71-2 |
   | 2 | 17, 31, 97 | 13, 71, 2 | 17-31, 31-97 |
   | fill 13+2 | 13, 2 | the other four | none |
   | fill 17+97 | 17, 97 | the other four | none |
   | fill 31, fill 71 | that arm alone | the other five | none |

2. **A stretch goes to the first phase and arm whose drawable map holds every point of it.**
   The arm planner is the judge. What it hands back flows to the next phases.
3. **A line no map holds whole is cut into the longest stretches some map holds**, in phase
   order, with one join of `min_piece` (1 cm, drawn twice) between neighbouring stretches.
   **A stretch goes to a fill phase only when the leader phases hold less than 80 % of it.**
4. **Parts of one line given to the same arm in the same phase are one job.**
5. **The account: drawn plus left over equals the input; anything else is a bug.**

- A phase with nothing to do is not run.
- A phase whose arms drew nothing is not reported as a phase.
- Both kinds of phase are listed as skipped, with the reason.
- The walls keep the arms of a phase apart by construction; the independent checker confirms
  it.

**Why 13 with 2 and 17 with 97 may fill together.** The ends of a column hang 2.42 m apart.
No part of an arm, pen included, gets further than 1.150 m from its own first joint's axis.
That is a bound from the arm model, not a sample; sampled, the furthest is 1.02 m. So the two
stay 0.121 m apart, against 0.053 m demanded. Arms 31 and 71 each reach another arm, so they
fill alone (`phases.fill_groups`, tested).

## Drawable maps

For every arm in every phase there is a grid over the canvas, 2 cm apart. A grid point is
drawable when at least one way of holding the pen there passes the local planner's own node
test:
- joint limits;
- the smallest singular value;
- the arm against the paper and against itself;
- the obstacles of that phase.

The pen is upright, with 8 hand turns and every elbow value and arm shape. A point of a line
is held by a map when its nearest grid point is drawable. The maps are cached under a digest
of the arm, its pose and tool, the phase's obstacles, the gates and the grid. They take 5.5 to
7 s of CPU each, 12 s for all twelve in 12 processes.

Coverage of the canvas at the gates of rig.json (sigma_min 0.04; the same as at 0.08, to the
grid):

| phase | per arm | together |
|---|---|---|
| 1 | 13: 0.238, 71: 0.243, 2: 0.240 | 0.721 |
| 2 | 17: 0.238, 31: 0.242, 97: 0.239 | 0.719 |
| fill | 13: 0.253, 2: 0.255; 17: 0.254, 97: 0.254; 31: 0.273; 71: 0.274 | |
| all phases | | 0.992 |

![drawable maps](figures/system_maps.png)

## The drawing area

The drawing area is the largest rectangle centred on the table, with sides along the table,
that lies inside the union of all maps shrunk by 2 cm (`area.py`). It is **1.56 x 3.56 m**,
against the canvas's 1.80 x 3.63 m (86 % along x, 98 % along y, 85 % of the area). It is in
`config/rig.json` (`canvas.drawing_area_m`, with how and at which gates). Drawings must lie
inside it; the drawing server will scale them to fit. The rig does not read that entry yet
(see below), so the planner computes the same rectangle from the maps; the slow test checks
that the two agree.

## Measured (2026-09-30, gates of rig.json, machine load 8 to 35, 30 processes, maps cached)

The seven drawings of `tests/system_cases.py`, each scaled to the drawing area:
- word: "unknown", 0.55 m wide at the table centre;
- hatch: 40 lines 1.5 m long;
- scatter: 80 short lines;
- starburst: 24 rays;
- spiral: 4 turns;
- duotone: 10 wavy bands the length of the area;
- random 300: 300 random lines, 5 cm to 1.5 m long.

Drawing speed 20 mm/s. "Phases 1 + 2" answers Pete's question: how much one 1-2-1 / 2-1-2
alternation draws.

| case | length m | drawn | phases 1 + 2 | fill | cuts | left over | planning CPU / wall s | first motion s | on the rig s (phases) | checker |
|---|---|---|---|---|---|---|---|---|---|---|
| word | 1.30 | 1.000 | 1.000 | 0 | 0 | none | 17 / 6.4 | 6.0 | 96 (96) | 53 / 53 |
| hatch | 60.00 | 1.000 | 1.002 | 0.004 | 42 | < 1 mm | 120 / 13 | 3.6 | 1600 (781 + 798 + 14 + 8) | 377 / 377 |
| scatter | 9.59 | 1.000 | 0.979 | 0.021 | 0 | none | 63 / 6.3 | 2.5 | 323 (229 + 75 + 19) | 331 / 331 |
| starburst | 29.36 | 1.000 | 0.999 | 0.006 | 16 | none | 68 / 8.3 | 2.4 | 1052 (524 + 516 + 12) | 180 / 180 |
| spiral | 16.91 | 1.000 | 1.003 | 0.007 | 17 | none | 47 / 8.1 | 2.1 | 499 (266 + 220 + 13) | 100 / 100 |
| duotone | 35.29 | 1.000 | 1.004 | 0.002 | 22 | none | 81 / 9.0 | 3.3 | 697 (342 + 346 + 9) | 184 / 184 |
| random 300 | 179.67 | 1.000 | 0.993 | 0.013 | 95 | < 1 mm | 281 / 30 | 8.1 | 4520 (2369 + 2032 + 81 + 38) | 1665 / 1665 |

- Inside the drawing area every drawing is drawn completely. Phases 1 + 2 draw 98 to 100 %
  of it (over 1 where the joins are drawn twice).
- **Checker:** all 2 890 motions pass `aris.check.check` with their phase, and the smallest
  clearance beyond the demanded one is 0.8 mm. `check_phase_end` passes after all 21 phases
  that ran.
- **Skipped:** phases with nothing to do (2 to 5 per case). One phase whose arm drew nothing:
  hatch, fill 31, which handed back a 1 cm rim stretch as unreachable.
- **Rig time:** each phase lasts as long as its busiest arm. In phase 1 of the random lines,
  arm 71 works 2 369 s while its partners work less. Nothing balances the arms yet.
- The word's first motion waited for the local planner's kinematic table, rebuilt once for
  the new gates; the other cases read it.
- **Before the drawing area** (whole-canvas drawings at the old gates), the same laws left
  only what lies beyond every arm's reach: spiral 13 mm, starburst 0.25 m, duotone 0.05 m,
  random 0.27 m.

![word](figures/system_word.png)
![hatch](figures/system_hatch.png)
![spiral](figures/system_spiral.png)

Figures for the other cases: `figures/system_scatter.png`, `system_starburst.png`,
`system_duotone.png`, `system_random.png`. The left panel is coloured by phase, the right by
arm. Leftovers are red. Phase 1 walls are solid and phase 2 walls dashed. The dotted
rectangle is the drawing area.

## What it cannot do (yet)

- **Followers do not draw (step 2).** In phases 1 and 2 the three followers stand parked.
  Step 2 would hand each leader's footprint (everywhere its body goes in the phase) to its row
  partner as one more obstacle, and the partner would draw what it can next to it. That
  would also share out the busiest leaders' work.
- **Nothing balances the arms of a phase.** The rig time is the sum of the phases, each as
  long as its busiest arm.
- A stretch an arm refuses is offered to the phases after only, never to the same arm again.
- The maps judge single points with the pen upright. A stretch they hold can still fail as a
  whole; it then flows on to the next phases.
- The drawing area is written in rig.json, but `rig.py` does not read it yet, so the planner
  recomputes it from the maps.

## Tests

`tests/test_system.py`, quick set in 36 s:
- the maps of one phase build, are cached and read back;
- hand-made lines land where expected;
- the laws on toy maps: whole, the cut with its join, the 80 % rule, a hole in a leader's map
  filled with joins at both ends, nothing holding a line;
- parts that meet are one;
- the fill pairs cannot touch;
- the account raises on a drop, a double and a stranger;
- a drawing outside the area is refused;
- a small drawing runs end to end on arms 13 and 71 in two processes, joined from park to
  park, with the same result from one process.

Slow test: the word end to end through the checker and `check_phase_end`, and the drawing
area of rig.json equal to the one the maps give.
