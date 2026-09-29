# local — the local planner

**Job.** Given lines on the paper, find how one arm holds and moves the pen to draw each of them.
Same code for every arm, everything in the arm's base frame. It knows nothing about the table,
the canvas or the other arms: obstacles arrive as geometry, and the paper is the plane of kind
"paper" among them.

Files: `aris/local/` — `planner.py` (the one call), `lattice.py` (the graph), `search.py` (the
sweep), `backout.py` (the exact path and its check), `pieces.py` (pieces, leftovers,
alternatives), `gates.py` (what a configuration must pass), `polyline.py`, `settings.py`.

## In and out

`plan(arm, lines, obstacles, rules, gates, workers=1) -> (bunches, leftovers)`

- **In:** lines (pen-tip polylines in the base frame), the obstacles, the drawing rules (draw
  speed, largest lean, shortest piece), the gates (joint-limit distance, singular value,
  clearance to itself).
- **Out:** one `Bunch` per piece. A line drawn in one go is one piece; a line where the arm has
  to lift and change shape is several. A bunch holds up to four alternative plans that start
  or end in a really different arm shape; the sequencer picks one. Each plan is a joint path
  with the pen on the line, its arc length, the pen tip, the spin and lean, a score (the
  clearance of the flown path, metres), the joint travel and the draw time. Each plan can be
  run in either direction (`reverse_plan`).
- And a `Leftover` for every stretch not drawn: `unreachable` (no arm configuration inside the
  gates, or no continuous motion), `blocked` (there are configurations, but the obstacles remove
  them all; the detail names the obstacle), `too_short` (under `rules.min_piece`).
- Nothing is dropped: for every line, pieces and leftovers cover it exactly once.
- `plan_detailed` returns the same plus what each line cost; `verify_plan` checks a plan from
  outside.

## The graph

One **layer** every 1.5 cm along the line. A **node** is one choice of

- spin: the hand turned about the paper normal, 24 values (15 degrees apart, a circle);
- lean: the pen tilted off its nominal direction, a 2-vector; zero, or 12 more up to
  `rules.lean_max` (two rings of six);
- elbow value: joint 7, which the IK takes as given, 21 values 15 degrees apart;
- IK slot: which of the solver's up to 8 answers (the slot is the branch).

With the tip on the line that is one joint configuration (`arm.hand_pose`, then `arm.ik`).
A node is **usable** when it passes the gates. Cheap tests first: joint limits and singular
value on all answers; then the arm against itself and against the paper; the other obstacles
last, only on what survived. The arm's links keep the paper's 20 mm; the parts bolted to the hand
may come as close as touching (see "what it cannot do").

**Edges** go from one layer to the next only, between neighbours (same slot; same or next spin,
elbow value and lean; spin wraps round), and only when no joint moves more than 0.35 rad. That
cap keeps the arm on one branch while the pen is down. Spins and elbow values are spaced alike on
purpose: at zero lean both turn about the paper normal, so a step of both together is a small
move, and the diagonal edges are the cheap ones.

**Lift edges** stay at one point of the line and join any node to any other, at a fixed cost of
5 rad of joint motion. A line drawn in one go uses none; each lift edge on the chosen route is a
cut between two pieces. A step that cannot be drawn at all is left undrawn at a cost far above
anything else (1000 per step), which is how a leftover enters the search.

## The objective and the search

Stay on usable nodes (yes or no per node) and minimise the joint motion summed along the route
plus the lift costs. The search is one sweep over the layers (dynamic programming): for each
layer, the cheapest way to reach every node, from the previous layer along an edge, by a lift,
or by putting the pen down after an undrawn stretch. Array operations over whole layers.

**Narrow first.** The first graph has zero lean only. Where its route lifts or leaves a gap, the
layers within 6 cm are opened to every lean and the sweep runs again (at most three rounds).

## The exact path

Along the route, spin, lean and elbow value are read off a smooth curve through the route's
nodes (averaged over about two layers, so the grid's staircase becomes the drift it stands for),
and the IK is solved again every 2 mm, and at every corner of the line, in the route's slot.
Joint angles are never interpolated. Where the straight joint move between two samples would
take the pen more than 0.02 mm off the line, a sample is added halfway.

Then the path is checked from its joint samples alone: the tip on the line (1e-6 m) and near
it between samples; every gate at every sample; no joint moving more than 0.1 rad between
samples; the arm clear of itself; and the clearance along the whole path, motion between
samples included (`path_clearance`). If the smooth curve fails, the curve through every node is
tried; if that fails too, the nodes at the failure are banned and the sweep runs again. An
unverified plan is never returned.

## The alternatives

For each piece, one more sweep over its layers with no lifts gives, for every node of its last
layer, the best route there and where it started. Routes are grouped into families by IK slot
and the quarter-turn of the spin at each end; the best of each family, cheapest first, is
solved exactly and checked; a family whose start and end are both within 0.5 rad (every joint)
of a plan already taken is skipped. At most four. Each plan is timed with `kernel.retime`
(corner to corner: the pen stops at a sharp corner anyway); a plan that cannot be timed, or
whose flown path could come closer than allowed (the timing step keeps it within 0.15 mrad of
the path, which is charged against the clearance), is not returned.

## Figure

![graph of one example line](figures/local_example.png)

## What it cannot do

- The parts bolted to the hand (hand, finger blades, holder, pencil tail) are only required not
  to touch the paper while drawing. With the rig's 20 mm paper clearance for the whole body no
  drawing configuration exists: the holder's cap capsule is 0.03 mm above the paper at zero
  lean. This rule is local to this module (`gates.py`) until the kernel or the rig has one.
- Pieces end at layer positions, so a piece next to an unreachable stretch can stop up to one
  layer (1.5 cm) short of where the arm could still reach.
- The graph is rebuilt for every line; nothing is shared between lines (see
  `OPTIMIZATION_NOTES.md`, items 1, 2, 6, 7).
- The draw time is timed corner to corner; the sequencer re-times the motion it flies.

## Measured

(filled in below)
