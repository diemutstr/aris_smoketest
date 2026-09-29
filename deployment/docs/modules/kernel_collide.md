# Collision check (`aris/kernel/collide.py`, `aris/kernel/geometry.py`)

## Job

Say how far the arm's collision body is from everything it must stay clear of, for many arm
configurations at once. Pure geometry: it does not know what any obstacle is.

## In and out

- **In:** the arm's body (`Body`: K capsules at each of N configurations; each capsule is a
  line segment with a radius) and the obstacles (`Obstacles`: boxes of any orientation, flat
  planes, capsules), each obstacle carrying the clearance it demands (its margin).
- **Out:** clearance in metres, per configuration.

**Clearance** of one capsule against one obstacle = the gap between their surfaces minus that
obstacle's margin. The clearance of a configuration is the smallest of these over all pairs.
At least 0 means free. Below 0 means the margin is violated; a capsule cutting into a box or
another capsule reads a gap of 0, a capsule through a plane reads how deep it goes.

Special rules:
- the pen capsules use the plane's `pen_margin` (if it has one) instead of its `margin`;
- with `drawing=True` the pen is not checked against planes of kind "paper" (it is on the
  paper on purpose);
- capsules the body marks as fixed (`is_fixed`: the base, inside the arm's own mount) are
  never checked against obstacles.

## The calls

| call | gives |
|---|---|
| `clearance(body, obstacles, drawing=False)` | (N,) clearance |
| `clearance_detail(...)` | the same, plus which capsule and which obstacle is closest, for reasons and reports |
| `capsule_clearance(...)` | (N, K) per capsule (exact for the closest capsule, a lower bound for the others) |
| `self_clearance(body, pairs, margin)` | (N,) the arm against itself, for the given capsule pairs |
| `path_clearance(body_of, q, obstacles, reach, drawing=False, tol=5e-4)` | a lower bound on the clearance along a whole joint path, including the motion between samples |

## How the distances are computed (all exact, no search)

- **Segment to segment:** the distance is smallest either at one point in the middle of both
  segments (found by solving two linear equations) or with one end of a segment against the
  other segment (a projection). All five candidates are computed and the smallest is kept.
  Parallel segments are covered by the end cases.
- **Segment to plane:** the height above the plane changes linearly along the segment, so the
  lower end decides.
- **Segment to box:** in the box's own frame, walk along the segment. The squared distance to
  the box is a bowl-shaped curve made of a few parabola pieces, joined where the segment
  crosses the plane of a face (at most six places). Its slope is known in closed form at each
  crossing and at both ends; the lowest point lies between the last place where the slope is
  not positive and the first where it is, and on that stretch the slope is a straight line, so
  one interpolation finds it. This replaces the old 36-step search per capsule and box.
- **Skipping far pairs:** first every capsule's midpoint is compared with every box and
  obstacle capsule. That gives, per pair, a cheap lower bound (midpoint distance minus half the
  capsule's length) and a cheap upper bound (midpoint distance). A pair whose lower bound is
  above the best upper bound of its configuration cannot be the closest pair, so it is not
  computed exactly. The answer is identical with and without this step; a test checks that.

## How the motion between samples is covered

The arm moves along straight lines in joint space between the samples of a path. The caller
passes `reach`: for each joint and capsule, how far that capsule can be from that joint's axis.
A joint step `dq` then moves any point of the capsule by at most `sum over joints of reach x |dq|`.
Clearance cannot change faster than the capsule moves, so on each interval the clearance is at
least what both ends allow once that travel is charged (each interval is charged once, from
both ends, and a capsule that does not move is charged nothing). Where this bound is more than
`tol` below the smallest clearance seen at any sample, the interval is halved and both halves
are measured again. At the end the answer lies at most `tol` below the true minimum, whatever
the sampling the path came in: the same path handed over coarse or fine gives the same answer
within `tol`. A cap on halvings keeps it finite; hitting the cap only makes the bound looser,
never wrong.

## What it cannot do

- It does not model the arm; `reach` must come from the arm model (not yet provided by
  `kernel/arm.py`; the tests compute it from the FR3 link lengths).
- A capsule inside a box or another capsule reads 0 gap, not how deep it is.
- `path_clearance` covers straight joint-space motion only, not a timed curve between samples.
- One configuration at a time is slow (about 0.3 ms of Python overhead per call): ask in batches.

## Measured (one core of a Ryzen 9 7950X3D, numpy)

Exactness, 10 000 random pairs each including crossing, touching, parallel, nearly parallel and
point-like cases, against brute force (dense sampling with zoom): segment-segment 1.6e-13 m,
segment-box 3.3e-16 m. Against the old code: its box search is up to 3.9e-10 m too high (a
search can only overshoot; this module agrees with brute force to 2e-16); its segment-segment
distance agrees to 4e-16 m.

Path bound: the same path at four sampling densities gives answers 0.1 mm apart at most
(tolerance 0.5 mm), and never above the densely sampled minimum.

Speed, 12 capsules against 32 boxes, 3 planes and 14 capsules (588 pairs per configuration):

| batch | configurations/s | capsule-obstacle pairs/s |
|---|---|---|
| 1 | about 3 000 | 1.8 M |
| 100 | about 46 000 | 27 M |
| 10 000 | about 46 000 | 27 M |

Without the skipping step, about 10 000 configurations/s (6 M pairs/s). The real FR3 body (33
checked capsules, 1 617 pairs) runs at about 18 000 configurations/s in batches. A path check on
the real arm takes 4 to 8 ms and 50 to 400 body evaluations.
