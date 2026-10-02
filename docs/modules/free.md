# free — the free-space planner

**Job.** Move one arm, pen up, from one configuration to another without hitting anything, and
hand back a timed motion that is checked exactly as it will be flown. The same code serves every
arm. Everything is in the arm's base frame. It knows nothing about the table, the canvas or the
other arms: walls, parked arms, steel and the paper all arrive as geometry.

Files: `aris/free/planner.py` (the seven steps), `check.py` (what "free" means, in batches),
`lift.py`, `rrt.py`, `shortcut.py`, `flown.py` (the final verdict). Tests: `tests/test_free.py`;
the fixed test set: `tests/free_cases.py` builds `tests/data/free_cases_{2L,1L}.npz`;
measurement: `tests/free_bench.py` (`--package` measures another copy side by side).

## In and out

`plan(arm, q_start, q_goal, obstacles, rules, gates=None, seed_extra=b"")`

- **In:** the arm model, two joint configurations, the obstacles (base frame, each with the
  clearance it demands), the drawing rules (speed share of the joint limits, and the gates:
  joint-limit margin 0.15 rad, self margin), optional extra bytes for the random seed.
- **Out:** a `Motion` of kind "free": a timed trajectory that starts exactly at `q_start` and
  ends exactly at `q_goal` (to 1e-9 rad), at rest at both ends. Or a `Refusal` with a reason:
  `outside_limits` or `blocked` or `self_collision` (an end is not usable; the detail names the
  end, the joint or the part of the arm and the obstacle), `no_free_path` (the search cap was
  reached), `cannot_time`, `bad_input`. Never an exception.
- `plan_detailed` also returns what it did: which way it found the path, times per step, how
  many straight edges it checked, path length, flown duration, clearance as flown.

## How it works: seven steps

1. **Check the ends.** Each end must be inside the joint limits by the gate's margin, clear of
   every obstacle and clear of itself. If not, refuse at once and say which.
2. **Try the straight joint-space move.** 26 % to 31 % of the test pairs need nothing more.
3. **Raise both ends.** A lift-off configuration has the pen tip 25 mm above the paper: the
   hardest place to grow a search from. If the obstacles contain the paper, each end is moved
   straight up along the paper normal to 0.06 m, the hand keeping its orientation and joint 7
   its angle, the arm following its own shape (every step of 12 mm takes the IK answer nearest
   the last one). If that short move is not free (it often reaches a joint-limit margin or a
   wall), the end stays where it is. Then the straight move between the raised ends is tried.
4. **Bidirectional tree search** (RRT-Connect) between the (raised) ends: each round grows one
   tree a step of up to 1.5 rad toward 4 random configurations, then tries one straight edge
   from the other tree to every new node; the trees swap every round. Capped by 20 000 edges
   checked, not by time, so a result does not depend on the machine.
5. **Shorten.** Drop every waypoint whose neighbours can see each other, one round of cutting
   corners between 16 pairs of random points along the path, then drop again; a lift is kept or
   skipped whole (near the paper every check is expensive). Every replacement is checked before
   it is accepted, and each round's candidates go in one batch.
6. **Time it** with `kernel.retime`. Timing rounds the corners, and a larger rounding budget is
   both faster to compute and faster to fly. The budget is chosen from the clearance at the
   path's corners; if the flown check fails, a budget the path provably pays for, then halving.
7. **Check what is flown.** Timing keeps the straight pieces of the path exactly. A piece of the
   timed trajectory that lies on a straight piece of the path (measured, with its extremes
   computed exactly and any rounding error charged) is covered by that piece's bound, the one
   the search proved with room to spare. The rest, the rounded corners, is bounded now: the
   kernel's bound on the chord between points of the flown curve, plus how far the cubic can
   bend away from it, charged per piece. The joint limits come from retime's exact extremes, with
   the gate's margin. If the verdict fails, the budget is lowered; nothing unchecked is returned.

**What "free" means while searching.** Every straight joint-space edge goes to the kernel in
batches (`collide.edges_clearance_q`): it bounds the clearance all along each edge, obstacles
and the arm against itself, and stops as soon as an edge is proven free or a point on it is
found not free. The search keeps 2 mm beyond every margin (at most half of what the ends have),
asked of the kernel by adding it to the margins, so the path still reads free after its corners
are rounded.

**Same question, same answer.** The random seed is a hash of the bytes of both ends, every
obstacle array and `seed_extra`. The test re-plans in a fresh process with another hash seed and
gets a bit-identical trajectory. The compiled and the numpy collision engines give bit-identical
trajectories too (100 of 100 compared).

**2026-10-02, the rig with slots** (walls 40 mm from each arm until x/y are calibrated, the
fences of the switched-off rows), the set rebuilt from the local planner's lift-offs at the
22 mm lift: 998 of 1 000 solved for arm 2L and 998 for arm 1L (each refuses 2 as "no free
path"); CPU per plan median 46 / 32 ms, 95 % 173 / 91 ms.

## What it cannot do

- On a quiet machine it meets the 0.1 s target (tree moves 48 and 75 ms median, 118 and 177 ms
  at the 95th percentile); on a busy one it does not (106 and 191 ms median at load 58). 94 % of
  the time is inside the kernel's edge call, not in the planner's Python. What would cut it: a tighter bound
  on how far the arm moves along an edge inside the kernel. The kernel charges each joint's
  largest lever; a bound from the joints' actual speeds at the edge's ends (first order, plus
  the reach table for the change) measured 2.5 to 3 times fewer evaluations per edge in this
  module's earlier numpy version (17 to 20 against about 55).
- A refusal with `no_free_path` costs the whole cap: 20 000 edges, 10 to 30 s. The hardest
  solvable pair of the test set needed 15 000.
- Paths are shortened, not optimised: median path length 1.08 to 1.12 times the straight
  joint-space distance. One round of corner cutting costs about 45 % of a tree plan's time and
  saves 0.55 s of flying per tree move (5.7 against 6.3 s median); `shortcut_rounds=0` trades
  that back.
- It does not prefer the arm's own side of the table or keep the arm tidy (that would shrink a
  leader's footprint, OPTIMIZATION_NOTES 16).
- A refusal with `no_free_path` means "not found within the cap", not "no path exists". The
  test's walled scenario (a wall across the arm's axis below the shoulder, the ends on either
  side) found nothing with 1 000 000 configurations checked, which is evidence, not proof.
- Raising the ends is only an attempt, and it measurably helps little (below).

## Measured (2026-09-29, compiled collision engine, one core per plan, 16 plans in parallel)

The fixed set: 1 000 pairs of lift-off configurations for arm 2L (phase 2 obstacles: parked
arms 3L, 1L, 2R, walls 1R-2L and 2L-3R, steel, paper) and 1 000 for arm 1L (phase 1: parked 1R,
2L, wall 1L-2R); tips 25 mm above the paper, random spin, no lean; four groups of 250: near
(tips under 0.15 m apart), far (over 0.6 m), same IK branch, different IK branch.

CPU time per plan, milliseconds. Before (the first version: its own halving loop in numpy) and
now were run back to back on the same kernel and rig at machine load 62 and 58 on 32 cores;
"now, quiet" is the same code at load 8.

| arm | version | solved | straight / raised / tree | all: median, 95 % | straight: median, 95 % | raised | tree: median, 95 % | edges checked (tree) | length / straight | flown duration median, 95 % |
|---|---|---|---|---|---|---|---|---|---|---|
| 1L | before, load 62 | 1000 | 310 / 62 / 628 | 221, 476 | 36, 73 | 163 | 279, 528 | (1 998 configurations) | 1.03 | 4.6, 7.0 s |
| 1L | now, load 58 | 1000 | 306 / 60 / 634 | 74, 227 | 11, 26 | 33 | 106, 252 | 45 | 1.08 | 4.9, 8.2 s |
| 1L | now, load 8 | 1000 | 306 / 60 / 634 | 37, 103 | 9, 16 | 18 | 48, 118 | 45 | 1.08 | 4.9, 8.0 s |
| 2L | before, load 62 | 999 | 265 / 75 / 659 | 321, 694 | 48, 113 | 217 | 387, 822 | (2 131 configurations) | 1.04 | 4.4, 7.9 s |
| 2L | now, load 58 | 1000 | 261 / 75 / 664 | 133, 424 | 17, 46 | 42 | 191, 470 | 65 | 1.12 | 5.0, 9.2 s |
| 2L | now, load 8 | 1000 | 261 / 75 / 664 | 53, 154 | 12, 20 | 20 | 75, 177 | 65 | 1.11 | 5.0, 9.0 s |

- Split of a tree plan, quiet (wall medians, arm 1L / arm 2L): search 21 / 32 ms, shortening
  20 / 31, timing 2 / 2, flown check 4 / 5. A straight move: 7 / 10 ms to check the ends and the
  edge, 1.5 ms timing, 0.2 ms flown check. 94 % of a tree plan's time is inside the kernel's
  edge call.
- Times on this machine move by a factor of two to three with the load; counts, paths and
  durations do not.
- Every one of the 2 000 motions is re-checked independently in the slow test: at 1 kHz,
  clearance at every sample at least 0, the arm against itself at least 0, limits and margins
  held, ends exact.
- The first version: with 20 times its cap every then-unsolved pair was found (22 000 to 43 000
  configurations); the numpy collision engine gave the same trajectories, 4 times slower.
- **Raising the ends** (first version, cap 20 000 configurations): arm 1L solved 998 either
  way, arm 2L 994 raised against 990 not; raised-straight replaces the tree for 6 to 14 % of
  pairs; flown duration 0.1 s shorter; planning time not better. A raise fails for a quarter of
  the ends, for example where the lifted arm would come within 0.15 rad of a joint limit.
