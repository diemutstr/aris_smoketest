# system — which arm draws which line, in which phase

**Job.** Takes a whole drawing and decides which arm draws each line, in which phase and
behind which walls; runs the arm planners and hands out their motions as they come, each
tagged with its phase and arm; at the end, says what was not drawn and why. It only ever
calls the arm planner. This is step 1 of Pete's scheme (DESIGN.md section 3): leaders only,
followers parked.

Files: `aris/system/` — `planner.py` (the one call), `phases.py`, `maps.py` (drawable maps),
`allocate.py` (who draws what, and the cut), `run.py` (the arm planners of a phase, in
parallel), `account.py` (the no-drop account), `stretch.py` (part of a line), `settings.py`.

## In and out

`plan(rig, lines, rules, arm_configs=None, cache_dir=None, workers=1)`

- **In:** the rig, the drawing (lines in the table frame, each with its own id), the drawing
  rules, where each arm stands (default: its park), a directory for the drawable maps and the
  local planner's kinematic table (none: built every time), and how many processes to use.
- **Out:** a generator of `(phase name, arm id, Motion)`, in the order the arm planners produce
  them, which at the end returns the leftovers. Every drawing motion's `piece` is in the input
  line's own id and arc length. `plan_all` collects everything; `plan_detailed` also returns a
  report (below). `phase_named(rig, name)` gives the `Phase` a name stands for, which is what
  the checker needs.
- An arm away from its park must move in the first phase (phase 1); otherwise `plan` refuses
  with an error, since every phase assumes the arms it does not move stand parked.

## Phases and walls

A phase is who moves, who stands parked, and the walls between those who move.

| phase | moving | parked | walls |
|---|---|---|---|
| 1 | 13, 71, 2 | 17, 31, 97 | 13-71, 71-2 |
| 2 | 17, 31, 97 | 13, 71, 2 | 17-31, 31-97 |
| fill 13+2 | 13, 2 | the other four | none |
| fill 17+97 | 17, 97 | the other four | none |
| fill 31, fill 71 | that arm alone | the other five | none |

The fill phases run after phases 1 and 2. Arms that cannot touch each other in any
configuration move together (`phases.fill_groups`). The two ends of a column hang 2.42 m apart,
and no part of an arm, pen included, can get further than 1.150 m from its own first joint's
axis. That is a bound from the arm model (the triangle inequality along the chain), not a
sample; sampled, the furthest is 1.02 m. So those two arms stay at least 0.121 m apart, more
than the 0.053 m demanded with the planning allowance. Arms 31 and 71 each reach one of the
others, so they fill alone.

A phase runs only if something was allocated to it. A phase whose arms planned and drew
nothing is not reported as a phase either (the old code's "EMPTY stage" lesson). Both kinds go
to the report's `skipped` list, with the reason. Every moving arm plans alone against its own obstacles (the
paper, its steel, the walls next to it, the parked arms it can reach), from where it stands
back to its park. The walls keep moving arms apart by construction, so no arm needs to know
about another; the independent checker confirms it afterwards. Every phase ends with every arm
at its park, so every phase starts from all arms parked.

## Drawable maps

For every arm in every phase: a grid over the canvas, 2 cm apart. A grid point counts as
drawable when at least one way of holding the pen there passes the local planner's own test of
a configuration (joint limits, singular value, the arm against the paper and itself, then the
obstacles of that phase). Pen square to the paper, 8 hand turns, every elbow value and arm
shape. The test is the local planner's graph layer itself, so the map and the planner judge a
configuration the same way. 12 maps, 5.5 to 7 s of CPU each (8 s for all of them with 12
processes), kept in the cache directory under a digest of the arm, its pose and tool, the
phase's obstacles, the gates and the grid.

Share of the canvas each map covers:

| phase | per arm | together |
|---|---|---|
| 1 | 13: 0.238, 71: 0.243, 2: 0.240 | 0.721 |
| 2 | 17: 0.238, 31: 0.242, 97: 0.239 | 0.719 |
| fill (no walls) | 13: 0.253, 2: 0.255; 17: 0.254, 97: 0.254; 31: 0.273; 71: 0.274 | 13+2: 0.509, 17+97: 0.508 |
| every phase together | | 0.992 |

What no map covers (0.8 %) is the canvas rim beyond every arm's reach: the four corners and the
long edges level with the walls.

![drawable maps](figures/system_maps.png)

## Who draws what

Before each phase, every stretch still waiting for an arm is allocated over this phase and the
ones after it:

- **Leaders whole first.** A line goes to the first phase 1 or 2 arm whose map holds all of it:
  every point, 4 mm apart, judged against the map shrunk by one grid step, so that a point at
  the map's edge does not count.
- **A fill phase only when it is worth it.** A fill phase may take part of a line only if the
  leader phases hold less than 80 % of it between them, or some fill map holds a run at least
  1.5 times the longest run a leader map holds (`Settings.leader_share`, `fill_factor`). If so,
  and a fill map holds all of the line, it takes it whole.
- **Cut with overlap.** Otherwise the line is cut. From its start, take the leader arm whose map
  holds the longest run from there. A fill arm is taken instead only where fill phases are
  allowed and its run is at least 1.5 times as long, or where no leader holds the line at all.
  The next piece starts where that run ends, reaching back 1 cm (`rules.min_piece`) over the
  join, so the join is drawn twice rather than not at all.
- **Out of reach.** Where no map holds the line, that stretch is left over as "unreachable".
- A stretch keeps the arm it was given. Stretches under 1 cm are left over as "too short".

The arms of a phase then plan at the same time, one process each; the motions go out as they
arrive. What an arm could not draw (blocked, no path, anything) goes back into the pool with
its reason and is offered to the phases after; if none of them can take it, it is a leftover
with that reason.

## The no-drop account

`account(lines, motions, leftovers)` checks the result against the drawing, from the motions
actually produced (a drawing motion's `piece`), never from the plan. For every line, the drawn
pieces and the leftovers together must cover it from end to end; nothing may lie outside a
line or belong to no line; two stretches may overlap only by a join (1 cm). Anything else is a
bug in the planner and raises `NoDropViolation`. It passed on every case below.

## Measured (2026-09-30, machine load 15 to 31 on 32 cores, 30 processes, maps cached)

Whole-canvas drawings (`tests/system_cases.py`):
- word: the word "unknown", 0.55 m wide at the table centre, between arms 31 and 71;
- hatch: 40 lines 1.5 m long across the canvas;
- scatter: 80 short lines;
- starburst: 24 rays from the centre;
- spiral: 4 turns stretched to the canvas;
- duotone: 10 wavy bands the length of the canvas;
- random 300: 300 random lines, 5 cm to 1.5 m long.

Drawing speed 20 mm/s. "Phases 1 + 2" answers Pete's question: how much one 1-2-1 / 2-1-2
alternation draws. Measured on the committed sequencer (the lift rule is being changed in
parallel).

| case | length m | drawn | phases 1 + 2 | fill | cuts | left over m | planning CPU / wall s | first motion s | on the rig s (phases) | checker |
|---|---|---|---|---|---|---|---|---|---|---|
| word | 1.30 | 1.000 | 1.000 | 0 | 0 | none | 12 / 1.6 | 1.2 | 97 (97) | 53 / 53 |
| hatch | 60.00 | 1.000 | 0.952 | 0.055 | 40 | none | 117 / 15 | 4.0 | 1655 (747 + 760 + 49 + 99) | 353 / 353 |
| scatter | 9.59 | 1.000 | 0.980 | 0.020 | 0 | none | 60 / 6.5 | 2.5 | 329 (229 + 86 + 14) | 327 / 327 |
| starburst | 32.57 | 0.989 | 0.839 | 0.155 | 14 | unreachable 0.34 | 72 / 14 | 2.9 | 1315 (540 + 550 + 147 + 78) | 205 / 205 |
| spiral | 17.79 | 0.949 | 0.661 | 0.295 | 13 | no path 0.82, unreachable 0.08 | 44 / 12 | 1.9 | 590 (158 + 139 + 96 + 198) | 88 / 88 |
| duotone | 35.99 | 0.993 | 0.921 | 0.077 | 16 | unreachable 0.25 | 73 / 12 | 3.6 | 827 (332 + 335 + 83 + 78) | 192 / 192 |
| random 300 | 185.91 | 0.994 | 0.927 | 0.072 | 88 | unreachable 0.58, no path 0.52, too short 0.004 | 287 / 37 | 7.2 | 4696 (2139 + 1848 + 247 + 127 + 243 + 90) | 1639 / 1639 |

**What the two rules changed.** The first version gave each cut to the longest run in any phase
and had every arm fill alone. On the same sequencer:

| case | phases 1 + 2 before → after | cuts before → after | on the rig s: first version | + fill pairs | + leaders first |
|---|---|---|---|---|---|
| word | 1.000 → 1.000 | 0 → 0 | 97 | 97 | 97 |
| hatch | 0.762 → 0.952 | 40 → 40 | 1773 | 1606 | 1655 |
| scatter | 0.980 → 0.980 | 0 → 0 | 329 | 329 | 329 |
| starburst | 0.807 → 0.839 | 14 → 14 | 1158 | 1158 | 1315 |
| spiral | 0.451 → 0.661 | 12 → 13 | 686 | 686 | 590 |
| duotone | 0.642 → 0.921 | 17 → 16 | 1334 | 1334 | 827 |
| random 300 | 0.739 → 0.927 | 70 → 88 | 5619 | 5228 | 4696 |
| all seven | | | 10 996 | 10 738 | 9 609 |

- **Leaders first** lifts phases 1 + 2 to 0.84 to 1.00 of every drawing. The rig time falls
  where the fill work had been large (duotone -38 %, random -10 %, spiral -14 %). It rises where
  the extra leader work lands on an arm that was already the busiest of its phase: starburst
  +14 %, hatch +3 %. A phase lasts as long as its busiest arm, and nothing balances the arms
  yet.
- **Fill pairs** (13 with 2, 17 with 97) save time only where both ends of a column have fill
  work: hatch -9 %, random -7 %; elsewhere nothing.
- **Skipped phases:**
  - Phases with nothing allocated: 5 for the word; 2 or 3 on each of hatch, scatter,
    starburst and duotone; 1 on the spiral; none on random.
  - Phases whose arms drew nothing: one, the spiral's fill 13+2. Arm 13 handed back a 1.77 m
    piece with no lift-off at one end, the same piece it had handed back in phase 1.
- **Checker:** all 2 857 motions pass `aris.check.check` with their phase, the fill pairs
  included; the smallest clearance beyond the demanded one is 0.8 mm. `check_phase_end` passes
  after all 26 phases that ran.
- Shares are of the drawing's length. Phases 1 + 2 and fill add up to a little more than
  "drawn" because the joins are drawn twice (0 to 0.85 m per case).
- Per arm, its first motion comes 0.7 to 8 s after the start of its phase.
- The word lands whole on arm 71 in phase 1: its region reaches over the whole word, so arm 31
  never draws any of it.
- "No path" leftovers are pieces with no lift-off at one end (see sequencer.md). The sequencer's
  lift rule is being changed; these numbers will move.
- Planning CPU counts every process, including the start-up of the local planner's workers.
  Reading the cached maps takes 0.2 s.

![word](figures/system_word.png)
![hatch](figures/system_hatch.png)
![spiral](figures/system_spiral.png)

Figures for the other cases: `figures/system_scatter.png`, `system_starburst.png`,
`system_duotone.png`, `system_random.png`. The left panel is coloured by phase, the right by
arm; leftovers are red, phase 1 walls solid, phase 2 walls dashed.

## What it cannot do (yet)

- **Followers do not draw (step 2).** In phases 1 and 2 the three followers stand parked.
  Step 2: after a leader has planned its phase, its footprint (everywhere its body goes) is
  handed to its row partner as one more obstacle, and the partner draws what it can next to it.
  It would also share out the busiest leaders' work.
- The rig time is the sum of the phases, each as long as its busiest arm. Nothing balances the
  work between the arms of a phase. In phase 1 of the random lines, arm 71 works 2 139 s and
  arm 2 works 1 341 s. That is the next lever for the rig time.
- A piece refused by one arm is offered to later phases only; it is never re-planned by the
  same arm with its ends moved.
- The maps judge single points with the pen upright. A line whose points all pass can still
  fail as a whole (no continuous motion, no lift-off); it then comes back to the pool.

Tests: `tests/test_system.py`. Quick (24 s), in order:
- the maps of one phase build, are cached and read back;
- hand-made lines land where expected: one inside a region, one across a wall to phase 2, one
  out of reach left over, and a 1.5 m line cut in two with the 1 cm join;
- leaders come first: a line arm 31 alone could take whole goes to two leaders instead, and a
  line the leaders hold less than 80 % of goes whole to fill 31;
- the cut prefers a leader unless a fill run is 1.5 times longer;
- the fill pairs cannot touch: the bound, and sampled against it;
- the account raises on a drop, a double and a stranger;
- a small drawing runs end to end on arms 13 and 71 in two processes, every motion joined
  from park to park, with the same result from one process.

Slow: the word end to end, every motion through the checker and `check_phase_end`, with the
numbers above as floors.
