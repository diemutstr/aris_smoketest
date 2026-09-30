# Redundancy: how it's encoded and how we'll resolve it

## The redundancy space

With the pen tip pinned to the paper at (x, y), the free DOF per arm are:

1. **q7 self-motion** — the He/Liu solver's redundancy parameter (≈ wrist/elbow
   circle). Continuous, up to ~6 rad of travel, up to 4 discrete branches on top.
2. **tool yaw** — the pen is rotationally symmetric, so the full 2π of yaw is free.
   **For a perpendicular pen this is not an independent DOF**: R = rotz(yaw)·rotx(π)
   puts joint 7's axis on the pen axis, so the solution at (yaw+δ, q7+δ) is the
   same arm with q1..q6 unchanged and q7 shifted. Yaw only becomes independent
   once the pen is tilted (see 3).
3. **pen tilt** — a cone around perpendicular, bounded by drawing quality
   (`tilt_max_deg`, default 15°; see the tilt study).

So each drawable point sits on a ~3-D manifold of configurations (+ discrete
branches). Strokes are paths through this manifold; trouble happens where the
manifold pinches.

## Encoding: where are we rich vs pinched? (in the atlas today)

Per cell, `atlas.py` stores:

- **`valid_frac`** — fraction of the (orientation × q7) candidate grid with a
  comfortable solution (margin ≥ 0.15). Global "how many options" score.
- **`q7_window`** — the widest CONTIGUOUS q7 interval (rad) that stays
  comfortable at one fixed tool orientation. This is the corridor a stroke can
  glide through **without a branch jump**; it's the metric that predicts
  whether a line can be drawn in one smooth motion. (Inherited idea: IKA
  step4_corridor, which checked q7-set overlap between consecutive waypoints.)
- **`n_sol`** — raw count of limit-valid IK solutions (includes all branches).
- The best-margin `q` itself — a warm start / seed for the planner.

`redundancy` layer in the meshcat scene = `q7_window` (bright = many options).
Rule of thumb: cells with `q7_window` < ~0.5 rad are single-option pinch points —
strokes through them must enter with the right branch already selected.

## Resolution strategy (implemented in `planner.py`)

Per stroke, after allocation to an arm:

1. **The fiber is q7 × branch — not yaw × q7 × branch.** For the perpendicular
   pen, yaw is degenerate with q7 (above), so a (yaw × q7) lattice is not a
   richer search space: it is the *same* space sampled twice, aliased into
   diagonal bands (with 8 yaws × 24 q7 the valid q7 indices formed a comb of
   period 3 = 0.785/0.256). A small ±1-index transition window cannot travel
   along that diagonal, so the DP reported artificial disconnects. Yaw is
   therefore pinned to 0 and q7 (48 samples over the FR3 range) carries the
   whole 1-D self-motion, × up to 4 analytic-IK branches. Nothing wraps.
2. **DP, not RRT, inside a stroke.** The pen must follow r(s) monotonically in
   s, so the search graph is *structurally* a DAG layered by s — every edge goes
   from step i to i+1. One forward sweep is therefore globally optimal, in
   Ns × (Nq × 4)² time; sampling-based search buys nothing where the layering is
   free. RRT is reserved for the **pen-up transits** between strokes, which are
   the only genuinely 7-D, non-monotone, obstacle-dominated motions.
3. **Maximin σ_min, not a weighted sum** (default objective). A stroke is only
   as controllable as its worst step — libfranka zeroes the external-wrench
   estimate near singularities — so the DP maximizes the *bottleneck*
   min_s σ_min(q(s)), with total ‖Δq‖² as the lexicographic tie-break:
   V[n] = min(σ[n], max_{p→n} V[p]). An additive cost averages a lethal dip
   away; the bottleneck cannot be traded off. The old weighted sum
   (smoothness + margin + σ shortfalls) is still available as `objective="additive"`.
4. **Nodes are gated, edges enforce continuity**: margin ≥ 0.15 rad and
   σ_min ≥ 0.08 per node (plus paper/boom clearance), ‖Δq‖∞ ≤ 0.35 rad per edge
   — which resolves branch *identity* without ever labelling branches.
5. **Disconnect = split, not failure.** If no node at step i+1 is reachable, the
   forward pass stops and reports the farthest reachable s*: the stroke's
   natural cut point for reallocation or a pen-up transit. Two mechanisms
   produce it — an empty fiber (the under-base dead zone: no IK at all) and a
   *reachable-set* collapse (the fiber is non-empty but only on an IK sheet a
   π-branch-flip away from the one the stroke is on). The second is the one
   worth reporting: the atlas says "reachable", the planner says "not from here".
6. **Branch chasing at execution resolution**: `ik.solve_cc` (case-consistent)
   follows the scheduled branch waypoint-to-waypoint; a failed chase = the DP
   grid was too coarse there → refine locally.
7. **Tilt as the escape valve**: tilt buys reach at the rim (bounded by material
   quality) and, unlike yaw, is a real extra dimension. It enters the lattice
   only on strokes that need it (keeps the lattice small).
8. **Nullspace posture on the real robot**: the impedance controller's
   `k_nullspace` spring pulls toward the planned q — the plan IS the redundancy
   resolution; the controller just tracks it compliantly.

## PWL in (s, q7) — planning the band before the joints (`pwl.py`, then
`smooth.py` and `pacing.py`)

The DP above answers "which configuration at each of the 131 steps?". The same
stroke can be planned one level up, as a *shape in the redundancy space*, and
only then turned into joints.

- **The band is terrain.** Pin the tip to the paper and the leftover freedom is
  q7 (× branch). Plot q7 against arc length s: the feasible set is a 2-D band
  whose height is σ_min and whose holes are gate failures. A plan is a curve
  q7(s) crossing it — and because the pen must advance monotonically in s, that
  curve is a **function**, not a general path. That is the whole reason this
  works: a function of s can be simplified; a 7-D trajectory cannot.
- **Sheets = connected components** of the (s × q7 × branch) lattice under the
  DP's own edge test (grid neighbours with ‖Δq‖∞ ≤ 0.35). Each sheet flattens
  to plain 2-D fields (σ, margin, representative q) — the branch axis
  disappears, without ever labelling branches. The rim arc splits into 38
  components, two of them spanning all of s (474 and 467 nodes — the
  elbow-up/elbow-down pair); the lattice DP ran entirely on the larger one.
  **Sheet extents alone locate a split**: on the under-base stroke the sheets
  reach s ≤ 0.370 and s ≥ 0.636, so the dead zone is read off the decomposition
  before any search runs (the DP's own answer is s\* = 0.370).
- **Clearance = singularity avoidance stated in the band.** A chamfer distance
  transform (scipy is not installed; two-pass numpy) gives the distance from
  each free cell to the nearest hole or joint-limit wall — the q7 rows outside
  the grid are walls, the s ends are not. The dense DP keeps the project's
  maximin-σ objective as its primary and uses clearance as the tie-break, so
  among equally controllable paths it picks the one down the middle of the
  corridor. That middle is what leaves room to straighten the path.
- **Simplification is corridor-checked, then IK-certified.** RDP on the dense
  q7(s) (deviation measured vertically, in q7 grid indices — a perpendicular
  distance would mix metres with radians) proposes chords; each candidate is
  screened on the grid and then **chased with `ik.solve_cc` at the lattice's own
  s resolution**, requiring a case-consistent solution, continuity and both
  gates at every sample. The grid is a 48-sample *sampling* of q7 and certifies
  nothing between its nodes, so the acceptance test is the real kinematics.
  Result: the 1.56 m rim arc is **2 knots** instead of 131 steps, the R bowl
  **6**. Do not zero-fill holes when interpolating the fields — that silently
  demands half a cell of clearance everywhere and hands back the staircase
  (36 knots instead of 2).
- **Backing out joints is a re-solve, never an interpolation.** `backout()`
  reads q7(s) off the polyline at 5 mm and re-solves the case-consistent IK at
  every sample from the sheet's node at s = 0. Interpolating *q* between plan
  points is the `writing.densify` pitfall: one q7 index is a null-space
  self-motion (q1/q3 counter-rotate, the tip barely moves), those
  configurations are not collinear in joint space, and a straight line between
  them leaves the constraint manifold — 7.7 mm off the paper in the ARIS demo.
  Only the *redundancy parameter* may be interpolated, never the configuration
  it indexes. Measured on the backed-out paths: tip error ≤ 2e-12 m, zero
  `solve_cc` fallbacks, and σ/margin at or above the lattice DP's (rim arc
  0.196/0.153 vs 0.195/0.150) — with **7× less joint travel** (5.4 rad vs 38.3),
  because the DP spends its ±0.35 rad continuity budget staircasing between q7
  indices and a straight segment cannot.

- **Smoothing = rounding the graph's corners, not fitting a spline**
  (`smooth.py`). Each interior knot becomes a quadratic Bézier over a symmetric
  window in s, the segments between windows untouched. Because q7(s) is a
  *graph over s*, monotone-s survives rounding with nothing to enforce; and with
  the window symmetric the Bézier's s-component is exactly linear, so q7 stays a
  closed-form quadratic **in s** and dq7/ds interpolates linearly between the two
  segment slopes — rounding can never make |dq7/ds| bigger, only take the step
  out of it. A global spline would be C2 but would dissolve the straight
  segments and put the whole stroke back up for re-certification.
- **Certified by the same chase, with a known-valid fallback.** The curve is
  sampled at 5 mm and chased with the identical `ik.solve_cc` helper the
  corridor screening and `backout` use (σ ≥ 0.10, margin ≥ 0.15, ‖Δq‖∞ ≤ 0.35).
  A failure at s\* is charged to the blend whose window contains it and *that
  window alone* is halved, converging on the sharp knot `plan_pwl` already
  certified — so the bisection terminates by construction. Outside the windows
  the curve is q7-identical to the polyline on the same branch, so a failure
  there is not the rounding's doing and is reported, not iterated on. R bowl:
  4 corners, windows ±0.018 in s, **none shrunk**, σ unchanged at 0.1792, margin
  0.187 → 0.190, tip error 2.1e-12 m, and the worst corner's step in dq/ds down
  3.3× (4.90 → 1.49 rad/m per sample). The rim arc is 2 knots — no interior
  corner — so smoothing is the identity there, which is the answer, not a gap.
- **Still no RRT inside a stroke, and the timing agrees.** Monotone s is
  *time-like*: the plan is a graph over it, so smoothing and re-timing both
  collapse to 1-D problems on a scalar function. Sampling stays reserved for the
  **pen-up transits** (genuinely 7-D, non-monotone, obstacle-dominated) and later
  for **coupled multi-arm** strokes, where two arms carry independent s, the
  space stops being layered by a single s, and staying out of each other's way
  in time makes it a state-time-space problem — the one place a tree earns what
  a DP cannot give.
- **Pacing = TOPP-lite** (`pacing.py`). q̇ = (dq/ds)·v is linear in tip speed, so
  with velocity limits alone the feasible set at each s is [0, v_i],
  v_i = safety·min_j(q̇_max_j / |dq_j/ds|) — pointwise, uncoupled, one pass, no
  switching-point search. Command min(v_draw, v_i), integrate ds/v. q̇_max is the
  URDF's ([2.62]×4, 5.26, 4.18, 5.26 rad/s), not the MoveIt config's (Panda
  values). At 20 mm/s nothing slows: peak q̇ is 1.2 % of limit on the rim arc,
  7.2 % on the R bowl, and the joints only bind above 1.30 and 0.22 m/s (65× and
  11× the drawing speed). Acceleration and torque limits are v2 — those really
  do couple neighbouring s and need the forward/backward integration this pass
  deliberately omits.

The 2-knot rim arc is the point of the exercise: the redundancy decision for a
1.56 m stroke is "start at q7 = −1.45, end at +0.82, linear in between", which
is small enough to log, diff, hand to a controller, or re-time — and it is
*more* controllable than the 131-step schedule it replaces.

## The band objective changed: gated min-travel, not maximin (2026-08-20)

**The gates were always hard, and the objective was doing their job twice.**
`plan_pwl` carves a free region out of the sheet with σ ≥ 0.10 and margin ≥ 0.15
*before* any search runs, so every cell the DP may stand on has already cleared
both gates. Maximising the bottleneck σ over that region buys controllability
the gate has already bought — and pays for it in q7 wander, because the maximin
path will climb the band to sit on a ridge and climb back down, and every radian
of that climb is a null-space self-motion the pen does not need. The default is
now **minimise total joint travel subject to the same two gates**, with
clearance as an exact tie-break; `maximin_sigma` stays as the automatic
fallback (below).

**Exactly lexicographic, not nearly.** Travel is quantised to 1e-6 rad and
accumulated as an integer in a float64, so `A == A.min()` is a true equality
test and the clearance tie-break is applied to exactly the set of optimal
paths. Both terms are additive, so — unlike the maximin DP, whose tie-break is
only greedy-lexicographic — this one is exact in *both* components.

**An edge is charged for the ramp, not the staircase.** The obvious edge cost is
the chord ‖Q[i+1, j+dj] − Q[i, j]‖₁, and it is wrong here: a dense path crossing
the band at half an index per step must alternate dj = 0, 1, 0, 1, and RDP then
straightens that staircase into a ramp the arm actually executes. Charging the
staircase optimises a quantity the simplification is about to discard. The
shipped cost splits the edge into the two motions it is made of — the
null-space step sideways plus following the stroke at the column it lands in —
so a staircase and its ramp cost the same, and the DP starts caring about the
net excursion and about where in the band dragging the pen forward is cheap.
Measured over the 34 CSAIL stroke/arm pairs both objectives certify: chord
scored +2.03 % of dense travel against separable's +1.49 %, and 99 knots
against 90.

**What it bought, over those 34 paired plans:**

| | maximin σ | gated min-travel | change |
|---|---|---|---|
| q7 span inside strokes | 10.41 rad | **3.27 rad** | **−68.6 %** (27 better, 0 worse) |
| lattice path travel | 223.6 rad | **151.6 rad** | −32.2 % |
| PWL knots | 104 | **90** | −13.5 % |
| certified dense travel | 133.6 rad | 135.6 rad | **+1.5 %** |
| worst σ over the corpus | 0.2349 | 0.2343 | gate 0.10, never touched |
| worst margin | 0.1645 | 0.1509 | gate 0.15, never touched |

**It is not the default, and the reason is the clock.** Everything above is
true of the BAND. What the band does not know is that `writing.draw_duration`
stretches every stroke until no joint exceeds `qd_frac` = 0.30 of its velocity
limit, and that drawing — not transit — is the dominant term in the makespan
floor. (0.30 is half the cap the measurement behind it actually used; raising
it to 0.60 is worth 18.9 % of the makespan on this very run, and was tried and
refused on 2026-08-20 — see `writing.QD_FRAC` and `docs/BENCH.md`. Everything
below is measured at 0.30, which is still what ships.) A shorter path through
the band is a more *constrained* path, it has a larger |dq/ds|, and the ink
slows down to match. Measured on the shipped CSAIL run, at identical coverage,
identical segments and identical metres:

| | maximin σ | gated min-travel |
|---|---|---|
| grey, arm 31: 9 segments, 2.002 m | draw **20.22 s** | draw **30.91 s** |
| grey, arm 71: 6 segments, 1.741 m | draw 20.73 s | draw 24.83 s |
| orange, arm 97: 13 segments, 3.79 m | draw 49.63 s | draw 78.07 s |
| worst per-sample ‖Δq‖∞ (grey, arm 31) | 0.082 | 0.105 |
| two-pass floor (busiest arm, both phases) | **84.3 s** | 93.2 s |

At the planner's own pacing the two are indistinguishable (+0.01 % over 34
paired plans), which is why this was not obvious: `pacing.pace` runs at
`SAFETY`, the timeline runs at 30 % of the joint limits, and only the second
one binds. So `pwl.OBJECTIVE` ships as `maximin_sigma`. The gated shortest path
is implemented, tested (`tests/test_menu.py`) and one keyword away
(`objective="min_travel"`), and it is the right objective the day the draw
speed stops being joint-limited — but the hierarchy is makespan first, and
against that it loses.

**And the row below is the honest one: dense joint travel did not fall
either, and it was never going to.** Regressing the certified travel of those
34 plans on what produces it gives

    travel ≈ 13.94 · L − 0.67 · q7span     (corr(travel, L) = 0.954)

— joint travel is about 95 % *arc length*, and the q7 coefficient is NEGATIVE.
Moving q7 does not merely cost travel, it can buy it back, because a null-space
motion that counter-rotates against the stroke-following motion makes ‖Δq‖
smaller, not larger. On one 0.173 m segment, pinning q7 flat took the travel
from 1.219 to 2.257 rad — nearly double. So "minimise joint travel" and
"stop the wrist wandering" are *different, partly opposed* objectives on this
robot, and the one worth having is the second: q7 wander is what a person
watching the rig sees, what decides which fiber a stroke ends on, and therefore
what the sequencer downstream has to pay for. The 1.5 % is reported rather than
tuned away.

**The fallback is why this cannot cost coverage.** A gated shortest path is
entitled to run along the gate boundary — every cell clears σ ≥ 0.10, but only
just — while the 5 mm chase that has the last word samples *between* the
lattice's 10 mm nodes, where "only just" can become "not quite". When the
min-travel plan certifies on no sheet, `stroke_api` re-runs the identical sheet
pass under `maximin_sigma`, which buys margin by construction, before conceding
a split. A split is therefore never the new objective's doing: it is a
statement about the band. Coverage on the shipped logo is unmoved at
**99.2139 %**.

**One existing test had its premise outgrown, and was re-tuned rather than
relaxed.** `test_splitting_beats_not_splitting_on_a_constructed_instance` built
three 0.40 m strokes in the band arms 31 and 71 share, and asserted cutting
beats not cutting. Under the new objective the whole-segment allocation
balances that instance to 13.85 s against 13.74 s — better than the 14.58 s the
old objective needed a *cut* to reach — so the splitter examined 20 candidates
and correctly refused all of them. The strokes are now 0.70 m, where the
whole-segment deal is 38.7 s against 26.1 s and cutting is needed again; every
assertion is unchanged and both objectives pass it, so what it pins is the
splitter and not the band objective.

## Open questions

- Cross-arm coordination: two arms in the 10% overlap band must also resolve
  *each other's* swept volumes — corridor intersection becomes a joint problem.
  Plan: keep overlap handoffs time-separated first (no simultaneous strokes in
  the same overlap cell); true simultaneous coordination later.
- Elbow sweep vs. cage/boom: q7 travel moves the elbow a lot on inverted arms;
  the DP cost needs the real scene geometry (aris_planning_scene) before we
  trust large q7 excursions near the booms.
