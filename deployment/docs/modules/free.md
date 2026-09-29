# free — the free-space planner

**Job.** Move one arm, pen up, from one configuration to another without hitting anything, and
hand back a timed motion that is checked exactly as it will be flown. The same code serves every
arm. Everything is in the arm's base frame. It knows nothing about the table, the canvas or the
other arms: walls, parked arms, steel and the paper all arrive as geometry.

Files: `aris/free/planner.py` (the seven steps), `check.py` (what "free" means, in batches),
`bound.py` (how far the arm can move between two checked configurations), `lift.py`,
`rrt.py`, `shortcut.py`, `flown.py` (the final verdict). Tests: `tests/test_free.py`; the fixed
test set: `tests/free_cases.py` builds `tests/data/free_cases_{31,13}.npz`; measurement:
`tests/free_bench.py`.

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
  many configurations it checked, path length, flown duration, clearance as flown.

## How it works: seven steps

1. **Check the ends.** Each end must be inside the joint limits by the gate's margin, clear of
   every obstacle and clear of itself. If not, refuse at once and say which.
2. **Try the straight joint-space move.** 13 % to 17 % of the test pairs need nothing more.
3. **Raise both ends.** A lift-off configuration has the pen tip 25 mm above the paper, where the
   pen holder is 5 mm from the paper's 20 mm margin: the hardest place to grow a search from. If
   the obstacles contain the paper, each end is moved straight up along the paper normal to
   0.06 m, the hand keeping its orientation and joint 7 its angle, the arm following its own
   shape (every step takes the IK answer nearest the last one). If that short move is not free
   (it often reaches a joint-limit margin or a wall), the end stays where it is. Then the straight
   move between the raised ends is tried.
4. **Bidirectional tree search** (RRT-Connect) between the (raised) ends: each round grows one
   tree a step of up to 1 rad toward 8 random configurations and lets the other tree run straight
   at every new node; the trees swap every round. Capped by 50 000 configurations checked, not by
   time, so a result does not depend on the machine.
5. **Shorten.** Drop every waypoint whose neighbours can see each other, then three rounds of
   cutting corners between random points along the path, then drop again. Every replacement is
   checked before it is accepted.
6. **Time it** with `kernel.retime`. Timing rounds the corners, and a larger rounding budget is
   both faster to compute and faster to fly (on the same path 0.15 mrad took 40 times the computing of
   4 mrad). The budget is chosen from the clearance at the path's corners
   (median 6.7 mrad).
7. **Check what is flown.** The verdict is taken on the timed trajectory itself: the kernel's
   `path_clearance` over its samples, the arm against itself the same way, the joint limits (with
   the gate's margin) at 1 kHz. Between two samples a trajectory is a cubic, not a straight
   piece; how far it can bend away is charged on top. If the verdict fails, the budget is
   lowered and the trajectory checked again; nothing unchecked is ever returned.

**What "free" means while searching.** Between two configurations the arm moves along a straight
line in joint space. From the joints' speeds at each end and how fast those can change, the
planner bounds how far any point of each capsule can travel (`bound.py`; checked on random moves
against dense sampling: never exceeded, worst 0.9997 of the bound). That turns the clearance
measured at both ends into a lower bound all along; where the bound cannot prove the move free,
the move is halved and the middle measured. The search also keeps 2 mm beyond every margin (at
most half of what the ends have), so the path still reads free after its corners are rounded.

**Same question, same answer.** The random seed is a hash of the bytes of both ends, every
obstacle array and `seed_extra`. The test re-plans in a fresh process with another hash seed and
gets a bit-identical trajectory. The compiled and the numpy collision engines give bit-identical
trajectories too (100 of 100 compared).

## What it cannot do

- It is slower than the 0.1 s target when the tree is needed: 0.15 to 0.25 s median CPU per
  plan. About half the time is Python bookkeeping around the collision calls (about 2 000
  configurations checked per plan). A compiled edge check (the halving loop of `check.edges`
  inside the collision module) is the next step; retiming (20 to 50 ms) is the second.
- Paths are shortened, not optimised: median path length 1.05 to 1.09 times the straight
  joint-space distance, but up to 1.9 times at the 95th percentile.
- It does not prefer the arm's own side of the table or keep the arm tidy (that would shrink a
  leader's footprint, OPTIMIZATION_NOTES 16).
- A refusal with `no_free_path` means "not found within the cap", not "no path exists". The
  test's walled scenario (a wall across the arm's axis below the shoulder, the ends on either
  side) finds nothing
  with 50 times the cap, which is evidence, not proof.
- Raising the ends is only an attempt, and it measurably helps little (below).

## Measured (2026-09-29, compiled collision engine, one core per plan, 16 plans in parallel; machine load 37 on 32 cores)

The fixed set: 1 000 pairs of lift-off configurations for arm 31 (phase 2 obstacles: parked
arms 2, 13, 71, walls 17-31 and 31-97, steel, paper) and 1 000 for arm 13 (phase 1: parked 17,
31, wall 13-71); tips 25 mm above the paper, random spin, no lean; four groups of 250: near
(tips under 0.15 m apart), far (over 0.6 m), same IK branch, different IK branch.

| arm | solved | straight / raised-straight / tree | CPU per plan: median, 95 %, worst | wall: search, shorten, time, check (medians) | checked | length / straight | flown duration median, 95 % |
|---|---|---|---|---|---|---|---|
| 13 | 1000 / 1000 | 173 / 130 / 697 | 141, 322, 2 930 ms | 31, 58, 22, 19 ms | 1 764 | 1.05 | 4.7, 7.5 s |
| 31 | 1000 / 1000 | 133 / 142 / 725 | 207, 501, 4 515 ms | 51, 83, 29, 31 ms | 2 002 | 1.09 | 4.8, 8.3 s |

- By group (arm 31, CPU median): near 188 ms, far 251, same branch 200, other branch 184. Far
  pairs need the tree most (229 of 250). A straight move alone takes 31 to 46 ms median, of which
  retiming is about 20 ms.
- A second run under load 55 read 185 and 242 ms CPU median: times on this machine move by a
  third with the load. Counts, paths and durations do not change.
- The numpy collision engine: 4 times slower (straight 179 ms, tree 702 ms CPU median on a
  100-pair sample), same trajectories.
- Every one of the 2 000 motions was re-checked independently in the test: at 1 kHz, clearance
  at every sample at least 0 (smallest 0.14 mm), the arm against itself at least 0, limits and
  margins held, ends exact.
- **Cap.** With a cap of 20 000 configurations, 8 pairs were unsolved (2 on arm 13, 6 on arm
  31). All had a path: with 20 times the cap each was found within 22 000 to 43 000. Hence the
  default of 50 000. One was a timing failure (the goal only 0.36 mm clear, the flown check too
  coarse); the verdict now re-checks a failure on every sample before it counts.
- **Raising the ends** (both at the 20 000 cap): arm 13 solved 998 either way, arm 31 994 raised
  against 990 not; raised-straight replaces the tree for 13 % of pairs; flown duration 0.1 s
  shorter; planning time not better (arm 13 144 against 150 ms, arm 31 180 against 152 ms CPU
  median). A raise fails for a quarter of the ends, for example where the lifted arm would
  come within 0.15 rad of a joint limit.
