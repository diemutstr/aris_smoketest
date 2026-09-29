# Collision check (`aris/kernel/collide.py`, `collide_path.py`, `collide_native.py`, `geometry.py`, `native/collide/`)

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
| `pack(obstacles)` | the obstacles as flat arrays; pack once, reuse in every call |
| `clearance(body, obstacles, drawing=False)` | (N,) clearance |
| `clearance_detail(...)` | the same, plus which capsule and which obstacle is closest |
| `capsule_clearance(...)` | (N, K) per capsule: exact for the closest capsule, a lower bound for the others |
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
- **Skipping far pairs:** each capsule's midpoint is compared with every box and obstacle
  capsule first. That gives a cheap lower and upper bound per pair. A pair whose lower bound
  is above the best upper bound of its configuration cannot be the closest, so it is skipped.
  The result is identical with and without skipping.

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
which is 4 840 pairs. The arm has 784 self pairs. Speeds were measured in CPU time while other
jobs loaded the machine heavily (load average about 60 on 32 cores), so the figures may be low.
The threaded rows are wall clock and suffer most from the load.

| batch | numpy, configurations/s | compiled, 1 thread | compiled, 8 threads | compiled, 32 threads |
|---|---|---|---|---|
| 1 | 1 100 | 19 000 | | |
| 100 | 4 100 | 22 000 | | |
| 10 000 | 4 100 | 22 000 (106 M pairs/s) | 66 000 | 109 000 |

Edges on arm 31, one thread, CPU time, per second. The old way calls `path_clearance_q` in a
Python loop, which gives the same numbers as `floor=None`:

| edge length | floor 0 | floor 0, with self check | floor None | Python loop |
|---|---|---|---|---|
| 0.05 rad | 8 100 | 4 700 | 4 200 | 3 600 |
| 0.3 rad | 3 600 | 1 900 | 1 200 | 1 200 |
| 1.0 rad | 1 700 | 1 200 | 390 | 440 |

Most of the gain comes from stopping early at the floor. Once an edge has to be refined all
the way, the refinement dominates and the Python loop costs little extra.

One path check over a 50-sample path takes 29 ms in numpy and 4.1 ms compiled. For
comparison, 12 capsules against 49 obstacles (588 pairs) runs at 31 000 per second in numpy
and 146 000 compiled on 1 thread. With the compiled engine, most of the time goes to the
first pass (each capsule's midpoint against every obstacle, a few nanoseconds per pair). Only
a handful of pairs per configuration need the exact distance.
