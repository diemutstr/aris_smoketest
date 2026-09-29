# Collision check (`aris/kernel/collide.py`, `collide_path.py`, `collide_native.py`, `collide_groups.py`, `geometry.py`, `native/collide/`)

## Job

Say how far the arm's collision body is from everything it must stay clear of, for many arm
configurations at once. Pure geometry: it does not know what any obstacle is.

## In and out

- **In:** the arm's body (`Body`: K capsules at each of N configurations; each capsule is a
  line segment with a radius), or the joint angles plus the arm's tables (below); and the
  obstacles (`Obstacles`: boxes of any orientation, flat planes, capsules), each carrying the
  clearance it demands (its margin).
- **Out:** clearance in metres, per configuration.

**Clearance** of one capsule against one obstacle is the gap between their surfaces minus
that obstacle's margin. The clearance of a configuration is the smallest of these over all
pairs. At least 0 means free. Below 0 means the margin is violated. A capsule cutting into a
box or another capsule reads a gap of 0. A capsule through a plane reads how deep it goes.

Special rules:
- The pen capsules use the plane's `pen_margin` (if it has one) instead of its `margin`.
- The tool capsules (`is_tool`: the pen holder) use the plane's `tool_margin` (if it has one).
  Boxes and capsules do not care about either.
- With `drawing=True` the pen is not checked against planes of kind "paper", because it is on
  the paper on purpose. The tool stays checked, with its `tool_margin`.
- Capsules the body marks as fixed (`is_fixed`: the base, inside the arm's own mount) are
  never checked against obstacles.

## The calls

| call | gives |
|---|---|
| `pack(obstacles)` | the obstacles as flat arrays, with their groups; pack once (about 1 ms), reuse in every call |
| `clearance(body, obstacles, drawing=False)` | (N,) clearance |
| `clearance_detail(...)` | the same, plus which capsule and which obstacle is closest |
| `capsule_clearance(..., span=inf)` | (N, K) per capsule: its clearance, but never more than the configuration's minimum plus `span`. With the default `span` every capsule is exact; with a small `span` only the capsules near the minimum are, the others are lower bounds, and it is faster |
| `self_clearance(body, pairs, margin)` | (N,) the arm against itself, for the given capsule pairs |
| `path_clearance(body_of, q, obstacles, reach, drawing=False, tol=5e-4)` | a lower bound on the clearance along a whole joint path, motion between samples included |
| `arm_tables(arm)` | the arm as tables: its joint chain and its capsules |
| `path_self_clearance(...)` | the same kind of lower bound along a path, for the arm against itself |
| `clearance_q`, `clearance_detail_q`, `self_clearance_q`, `path_clearance_q`, `path_self_clearance_q` | the same, from joint angles and the tables, without building a `Body` |
| `edges_clearance_q(tables, reach, Qa, Qb, obstacles, self_pairs=None, self_margin=0, floor=0)` | one lower bound per straight joint-space edge, many edges in one compiled call (below) |
| `backend()` | "native" when the compiled module is installed, otherwise "numpy" |

Every call takes `backend="numpy"` or `"native"` (for tests) and `threads=` (compiled only; the
default is 1). The answer does not depend on the thread count.

## Two engines, one answer

The same method is written twice: once in numpy (`collide.py`, `geometry.py`) and once in
C++ (`native/collide/`, installed with `pip install ./native/collide` as the package
`aris_collide_native`). The collide module uses the compiled one when it is installed and numpy
otherwise. The C++ is a line-by-line copy of the numpy: the same candidates, the same order of
operations, the same choice of closest pair. On the same capsules the two engines agree bit
for bit in the tests.

The `_q` calls run the whole chain in C++: joint angles, then the forward kinematics of the
arm (DH rows, then the flange and hand frames), then the capsule ends, then the distances. No
Python runs per configuration. The robot's numbers are not written into the C++; they arrive
as tables taken from the `Arm` object.

## How the distances are computed (exact, no search)

- **Segment to segment:** the gap is smallest either at one point in the middle of both
  segments (from two linear equations) or with one end of a segment against the other segment
  (a projection). All five candidates are computed and the smallest wins.
- **Segment to plane:** the lower end decides.
- **Segment to box:** in the box's frame the squared distance along the segment is a bowl
  made of a few parabola pieces. They join where the segment crosses the plane of a face.
  From the slope at those crossings and at both ends, one interpolation finds the lowest
  point. This replaces the old 36-step search.
- **Skipping far groups, then far pairs.** Most pairs are nowhere near the minimum, so they
  are ruled out in two steps before any exact distance is taken.
  - The body is split into groups by name: one per link, and one per part of the tool (hand,
    each finger, holder, pen). Each configuration puts a sphere around each group.
  - `pack` splits the obstacles into groups. A parked arm becomes one group per link and tool
    part (about a dozen), because its capsules never move. The steel boxes are gathered
    greedily: a box joins a group whose first box is within 0.4 m. Other capsules are grouped
    by a 0.5 m grid. Every group gets one fat capsule that contains all its members; this is
    the coarse stand-in used for the far test. The planes are always checked one by one; they
    are cheap. Changing margins with `dataclasses.replace` is fine, because the groups follow
    the margins. Moving an obstacle needs a new `pack`.
  - First the closest pair of groups is measured in full, which gives a good current best.
    Then any pair of groups whose sphere and fat capsule are further apart than that best is
    skipped whole. Inside the remaining groups, each capsule's midpoint gives a cheap bound
    per pair. Only pairs that could still be the closest get the exact distance.
  - The arm against itself works the same way: its pairs are grouped by the two capsules'
    groups, and the spheres rule out links that are far apart.
- **What "the same answer" means.** A skipped pair is proven to be above the configuration's
  minimum plus `span`. So every capsule's value is defined exactly: its true clearance, cut
  off at the minimum plus `span` (`span` is 0 for `clearance`). Skipping more or less, or
  switching engines, cannot change it. `clearance`, `clearance_detail` (value and closest
  pair), `self_clearance`, the path bounds and the edge batch are identical, to 1e-12, with
  the groups on or off (`groups=False`, `prune=False` in the tests), in both engines.
- **What did change on 2026-09-29.** Per-capsule values for capsules that are not the closest
  are now cut off at the minimum plus `span`. Before, they were cheap midpoint lower bounds.
  Along a path each capsule is kept exact to 0.05 m above the minimum of its sample
  (`collide.SPAN`). The path and edge bounds therefore moved by up to 0.35 mm, nearly always
  upward (at most 0.01 mm down), within `tol`; they are still lower bounds within `tol` of
  the truth. No caller in `aris/`
  reads per-capsule values of capsules that are not the closest. `local/gates.py` uses
  `capsule_clearance` only in a fallback branch that is off for the real arm, and there the
  new default makes every capsule exact.

## How the motion between samples is covered

Between samples the arm moves along straight lines in joint space. `Arm.reach` says, per joint
and capsule, how far the capsule can be from that joint's axis. So a joint step moves every
point of the capsule by at most `sum of reach × |step|`, and clearance cannot drop faster than
that. Each interval is charged once, from both ends. A capsule that does not move is charged
nothing. Where this bound falls more than `tol` below the smallest clearance measured at any
sample, the interval is halved and measured again. The answer ends up at most `tol` below the
true minimum, however coarse or fine the path came in.

Refining stops as soon as it cannot change the answer. An interval is halved only while its
bound is below both the smallest clearance measured so far and a cap of 0.25 m (`CAP`).
Nobody needs the exact figure above the cap, so there the answer is only promised to be at
least 0.25 m less `tol`. A path far from everything therefore costs one body evaluation per
sample. With nothing to check against (no obstacles, or only paper for a pen while drawing)
the answer is +inf at once. `path_self_clearance` and `edges_clearance_q` behave the same.

## Many edges at once

A free-space planner checks thousands of short straight moves. `edges_clearance_q` takes them
all in one call and runs the whole halving loop per edge in compiled code, optionally split
over threads. With `self_pairs` the arm against itself is bounded the same way: a pair can
close no faster than both its capsules move together. The answer is the smaller of the two
bounds. With `floor=None` each edge gets exactly the bound `path_clearance_q` gives. With a
number (default 0), an edge stops as soon as its bound is proven at least `floor` (free) or a
point on it is found below 0 (not free). That is all a planner asking "free or not" needs.

## What it cannot do

- A capsule inside a box or another capsule reads a gap of 0, not how deep it is.
- `path_clearance` covers straight joint-space motion between samples, not a timed curve.
- One configuration at a time costs Python overhead (numpy about 0.3 ms per call, compiled
  about 20 µs). Ask in batches.

## Measured

The exact distances were checked against brute force on 10 000 random pairs each, including
crossing, touching, parallel and point cases. Segment to segment is within 1.6e-13 m;
segment to box is within 3.3e-16 m. The old code's box search was up to 3.9e-10 m too high.
Compiled against numpy: 0 difference on the distance cases, forward kinematics within 5e-16 m
of `Arm.body`, path bounds within 6e-17 m.

Speed of arm 31 in phase 2: the real FR3 body (62 capsules, 55 checked, 33 of them on the tool)
against 23 steel boxes, the paper, walls to arms 17 and 97, and parked arm 71's 62 capsules,
which is 4 840 pairs. The arm has 784 self pairs. Everything is CPU time on one thread unless
it says otherwise. "Before" is the midpoint pass alone, measured at machine load about 11;
"after" adds the group skip, measured at load about 8, on the same configurations and paths.

| | before | after |
|---|---|---|
| compiled, batch 1, configurations/s | 33 000 | 44 000 |
| compiled, batch 100 | 36 000 | 300 000 |
| compiled, batch 10 000 | 35 000 | 244 000 (1 180 M pairs/s) |
| compiled, batch 10 000, 8 / 32 threads (wall clock) | 66 000 / 109 000 (at load 60) | 1.7 M / 2.6 M |
| numpy, batch 1 / 100 / 10 000 | 1 800 / 3 700 / 5 000 | 1 000 / 3 600 / 4 500 |
| self check, 784 pairs, compiled, batch 10 000 | 47 000 | 226 000 |
| 50-sample path check, compiled, median of 20 (mean) | 6.2 ms (6.9) | 0.9 ms (1.2) |
| 50-sample path check, numpy | 29 ms | 28 ms |
| exact distances per configuration (of 4 675 box and capsule pairs) | 3.9 | 3.1 |

The exact distances were already few before. The gain comes from the first pass: about 300
group tests and a few hundred midpoint tests per configuration, where there used to be 4 675
midpoint tests. Batch 1 is held back by the Python call itself, about 20 µs.

Edges on arm 31, one thread, CPU time, per second (after). The Python loop calls
`path_clearance_q` once per edge, which gives the same numbers as `floor=None`:

| edge length | floor 0 | floor 0, with self check | floor None | Python loop |
|---|---|---|---|---|
| 0.05 rad | 70 000 | 20 000 | 32 000 | 17 000 |
| 0.3 rad | 26 000 | 8 500 | 10 000 | 8 000 |
| 1.0 rad | 13 000 | 4 600 | 2 800 | 2 700 |

Before the group skip these were about 8 100, 3 600 and 1 700 per second with floor 0.

For comparison, 12 capsules against 49 obstacles (588 pairs) runs at 37 000 per second in
numpy and 470 000 compiled on one thread.
