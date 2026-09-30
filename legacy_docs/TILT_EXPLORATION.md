# Pen tilt as a planning axis

*Exploration, 2026-08-24. Code: `aris_sixarm/tilt.py`, `scripts/tilt_explore.py`,
`tests/test_tilt.py`. Flag: `stroke_api.DEFAULTS["tilt_max_deg"]`, default `0.0`.*

The planner pins the pen perpendicular to the paper. A real pen does not need
to be: most media draw happily at a lean of 10–20°, and the drawing is
identical as long as the **tip** still traces the curve. This asks what
happens if the lean becomes a planning axis — what it buys, what it costs, and
whether it belongs in the shipping pipeline.

---

## Recommendation: **ship it partially** — as a rescue, capped at 15°, default off

Tilt is not a general-purpose improvement. It is a **local remedy for one
specific failure**, and the measurements below are unusually one-sided about
which:

* Inside arm 2's comfort donut it is transformative: **0 of 4 test strokes
  certify at the strict gate without it, 4 of 4 with a 15° cone**, and the
  donut's strict-GO cell coverage goes **56.9 % → 100 %**.
* Outside the donut it buys **nothing**: the 0.24–0.60 m band moves 99.2 % →
  99.3 % and the rim 87.2 % → 87.9 %. That is noise on 2 289 cells.
* On strokes that already work it is a **wash or a loss**: it can lift a
  bottleneck σ by 5–10 %, and it charges 21–25 % more joint travel to do it —
  and joint travel is the clock.

So the value is entirely in *turning refusals into plans*, not in improving
plans. That is exactly what a rescue is, and it is how the shipping candidate
(`tilt.plan_adaptive`) is built: **plan flat first, open the tilt axis only
where flat was not enough, and return the flat result whenever tilt fails to
beat it.** The flag can add certified strokes and cannot remove one.

**Cap it at 15°.** 30° never changed a verdict the 15° cone had not already
changed, cost 3.2× the IK, and on the R bowl actively *lost* a stroke that
15° drew (see "more cone is not better" below).

**Do not** make it the default, and do not open it on strokes that already
certify.

---

## Route taken: the grid, not the RRT

The grid is comfortably tractable and the fallback was never needed.

Rim arc, 1.5605 m, 131 arc-length steps, 48 q7 samples, 4 IK branches:

| lattice | tilt points | nodes | batch IK calls | edges (bound) | memory | build |
|---|---|---|---|---|---|---|
| flat (today) | 1 | 25 152 | 6 288 | 301 824 | 1.8 MB | 10 ms |
| tilt ≤ 15° | 19 | 477 888 | 119 472 | 40 142 592 | 34.9 MB | 149 ms |
| tilt ≤ 30° | 61 | 1 534 272 | 383 568 | 128 878 848 | 112.0 MB | 508 ms |

The predicted ~150 k batch IK calls per stroke was right, and it is **not the
bottleneck**: profiling the whole tilt-15 oracle put 149 ms in the lattice and
**1.79 s in the DP**. The analytic IK is 2 % of the wall clock. An RRT in
(s, q7, tx, ty) would trade an exact, globally-optimal one-sweep DP for a
sampling method in order to speed up the part that was already cheap, so it was
not prototyped.

The DP is a DAG in s exactly as before — edges join consecutive arc-length
steps only, `|dq7| ≤ 1` index, tilt to itself or a hex neighbour,
`||dq||_inf ≤ 0.35`. The candidate axis is uniform (3 × 7 × 4 = 84 slots per
target node), which is what lets the *same* DP serve the oracle and the
adaptive planner: work per step is proportional to the tilt points **active**
there, so a lattice whose collar is open on 25 of 131 steps pays for 25.

---

## The chart: why (tx, ty) and a hex disc

**The tilt is a vector, never (θ, φ).** `(θ, φ)` has a coordinate singularity at
the apex: every φ names the same pen axis at θ = 0, so a grid in those
coordinates puts N nodes on one pose, gives them N different tool frames, and
lets a ±1-index DP window travel around the apex for free. The chart used is
the rotation vector

```
w = (ty, −tx, 0),   R_tilt = exp(ŵ),   R = R_tilt · rotx(π)
```

so `(0, 0)` is exactly perpendicular and `R` is analytic in `(tx, ty)`
everywhere on the disc. Verified: the realised lean equals `|t|` to 1e-15 and
the realised azimuth to 5e-16, and the **pen tip lands on the commanded point
to 2e-16 at every tilt** — which is the whole premise of the feature.

**The q7 aliasing is not a problem here, it is the reason the chart is
complete.** Joint 7 rotates the flange about the tool z and the TCP lies on
that axis, so `q7 → q7 + d` maps the pose `(R, p)` to `(R·rotz(d), p)` with the
tip and the pen axis untouched. Fixing one tool-frame convention per pen-axis
direction and letting q7 sweep its grid therefore covers the tool-spin
dimension **exactly once**, with no double counting. Pinning tip *and* pen axis
leaves a 2-D self-motion; the disc is one dimension of it and q7 is the other.

**The disc is a hex lattice**, not rings of (θ, φ) and not tool-x/tool-y axes —
which is what `atlas.py`'s tilt rescue does, and it biases the answer toward
four compass directions. A hex lattice has one neighbour distance and six
neighbours everywhere; inside the cone it gives 19 points at `n_ring = 2` and
61 at 4. Adjacency for the DP is "same cell or one of the six neighbours".

**Discs are nested by fixed pitch.** `atlas._candidates` builds its cone as
`{tilt_max/2, tilt_max}`, so its 30° set is `{15°, 30°}` and its 15° set is
`{7.5°, 15°}` — *not* nested, and that alone can make a wider cone score worse.
Every comparison here uses a fixed 7.5° pitch, so tilt ≤ 30 offers every pose
tilt ≤ 15 offers and a loss is a real loss.

---

## 1. The standard test strokes — does tilt strengthen plans that already work?

`ARIS_RIG=sixarm`, arm 31, oracle (whole lattice), `maximin_sigma`. These two
strokes and their published numbers (1.5605 m, 2 knots, σ 0.196) live on the
legacy rig; on `final6_opt` the same construction runs off the 1.8034 m canvas.

**A_rim_arc** (1.5605 m):

| tilt | status | knots | min σ | min margin | travel (rad) | max lean | plan |
|---|---|---|---|---|---|---|---|
| 0° | ok | 2 | 0.1963 | 0.1528 | 5.395 | 0° | 59 ms |
| 15° | ok | 5 | 0.2170 **+10.5 %** | 0.1885 **+23.3 %** | 6.730 **+24.7 %** | 13.0° | 1 951 ms |
| 30° | ok | 5 | 0.2170 | 0.1885 | 6.730 | 13.0° | 5 738 ms |

**R_bowl** (0.8187 m):

| tilt | status | knots | min σ | min margin | travel (rad) | max lean | plan |
|---|---|---|---|---|---|---|---|
| 0° | ok | 5 | 0.1795 | 0.2107 | 9.941 | 0° | 36 ms |
| 15° | ok | 8 | 0.1896 **+5.6 %** | 0.3079 **+46.1 %** | 12.003 **+20.7 %** | 15.0° | 2 664 ms |
| 30° | **split** (`chase_failed`) | — | — | — | — | — | 9 369 ms |

Three things to take from this.

*The gains are real but small, and they are paid for in travel.* σ moves 5–10 %
and margin 23–46 %, for 21–25 % more joint travel. `writing.draw_duration`
stretches the ink until no joint exceeds its velocity cap, so travel is the
makespan. On these two strokes the clock did not move (78.0 s and 40.75 s,
unchanged) because at the rig's draw speed nothing is joint-limited — but the
headroom was spent, and `docs/REDUNDANCY.md` records what happens when it is
not there.

*The optimum is inside the 15° cone.* The rim arc's tilt-30 answer is the
tilt-15 answer, to the last digit, at 2.9× the cost. Its chosen lean is 13.0°.

*More cone is not better.* The R bowl at 30° **loses a stroke it draws at 15°**:
the DP, given more room, finds a band path with a better bottleneck that the
5 mm certification chase will not walk. The freedom is genuinely a hazard to
the search, which is the strongest argument in this document for capping it.

The alternative `min_travel` objective is implemented and behaves as expected —
on the R bowl at 15° it cuts travel 9.94 → 7.68 rad (−23 %) — but it buys none
of the margin, which is the thing tilt is actually for. `maximin_sigma` stays
the default.

---

## 2. The donut — how much becomes drawable

`ARIS_RIG=final6_opt`, arm 2, whose J1/J2 shoulder projects to **(1.2768,
1.2572)** on the paper. The annulus at r 0.10–0.24 m of that point is the known
"comfort donut": every cell has IK, σ is fine everywhere (0.15–0.23), and the
binding constraint is **q5 in 154 of 158 failing cells** — a wrist-roll fold,
median q5 = 2.508 against a limit of 2.807, i.e. margin ≈ 0.30. It fails a
comfort gate, not a reach test. That is precisely the kind of failure an
orientation change should be able to unfold, and it is.

Four strokes crossing the annulus — two radial, two arcs that never leave it —
at the **strict** gate (margin ≥ 0.30, σ ≥ 0.14):

| stroke | length | r range | tilt 0° | tilt 15° | tilt 30° |
|---|---|---|---|---|---|
| D1_radial_180 | 0.240 m | 0.060–0.300 | **split** (s\*=0) | ok, margin **0.301** | ok, 0.317 |
| D2_radial_225 | 0.240 m | 0.060–0.300 | **split** (s\*=0) | ok, margin **0.321** | ok, 0.357 |
| D3_arc_r17 | 0.326 m | 0.170 flat | **split** (s\*=0) | ok, margin **0.304** | ok, 0.304 |
| D4_arc_r13 | 0.159 m | 0.130 flat | **split** (s\*=0) | ok, margin **0.332** | ok, 0.353 |

**0 of 4 → 4 of 4.** Not one of them certified a single certified metre flat
(`s_star = 0.0` in every case); all four draw end to end with a 15° cone, every
one clearing the 0.30 margin gate, every one independently validated by
`validate.validate_plan`, tip error < 2e-3 m.

At the pipeline's own **permissive** gates (margin ≥ 0.15, σ ≥ 0.10) the two
radial strokes *still* split flat — their fiber is empty at s = 0 — and still
draw with tilt. The two arcs certify flat at margin 0.158 and 0.154, i.e.
0.008 and 0.004 rad above the gate, and tilt lifts them to 0.167 and 0.189.

---

## 3. Mini-atlas — where the coverage actually moves

A patch of 2 739 cells at 2 cm out to r ≤ 0.60 m of arm 2's shoulder, swept
with **tilt in the fiber**: every (tilt, q7, branch) is scored and the cell
keeps the best margin. This is a different question from `atlas.py`'s, which
tries the perpendicular candidates first and **returns on the first set that
yields any solution** — so a cell that already has a perpendicular solution
never sees a tilted one, which is why every donut cell in
`out/atlas_final6_opt` is recorded at `tilt_deg = 0`. The shipped `out_tilt*`
sweeps measure *reach*, not comfort.

Strict gate (margin ≥ 0.30, σ ≥ 0.14):

| annulus | cells | tilt 0° | tilt 15° | tilt 30° |
|---|---|---|---|---|
| r 0.00–0.10 (inner hole) | 79 | 0 (0.0 %) | 68 (**86.1 %**) | 79 (100 %) |
| **r 0.10–0.24 (the donut)** | 371 | 211 (56.9 %) | 371 (**100 %**) | 371 (100 %) |
| r 0.24–0.45 (band) | 1 142 | 1 133 (99.2 %) | 1 134 (99.3 %) | 1 134 (99.3 %) |
| r 0.45–0.60 (rim) | 1 147 | 1 000 (87.2 %) | 1 008 (87.9 %) | 1 008 (87.9 %) |
| **all** | 2 739 | 2 344 (85.6 %) | 2 581 (**94.2 %**) | 2 592 (94.6 %) |

Permissive gate (margin ≥ 0.15, σ ≥ 0.10):

| annulus | cells | tilt 0° | tilt 15° | tilt 30° |
|---|---|---|---|---|
| r 0.00–0.10 | 79 | 0 (0.0 %) | 79 (100 %) | 79 (100 %) |
| r 0.10–0.24 | 371 | 319 (86.0 %) | 371 (100 %) | 371 (100 %) |
| all | 2 739 | 2 461 (89.9 %) | 2 592 (94.6 %) | 2 592 (94.6 %) |

**+8.6 points of strict coverage, and essentially all of it is inside r < 0.24.**
The band and the rim move by 0.1 and 0.7 points. Sweep cost: 0.5 s / 9.4 s /
28.2 s for the three cones.

The 15° → 30° delta is 11 cells, all of them in the inner hole (r < 0.10),
which is under the arm and not somewhere the allocator wants to draw anyway.

---

## 4. Pipeline compatibility

What a tilted plan changes downstream, checked rather than assumed:

* **The validator needs no change for the tip.** `validate.validate_plan`
  computes the tip through `frames.tip_pos_many`, which steps `pen_ext` along
  the tool z **of the FK'd pose** — it is already orientation-aware, and a
  tilted plan passes it unmodified (tip error < 2e-3 m on every certified
  stroke here). What is genuinely missing is a **cone check**: nothing
  downstream currently verifies the lean stayed inside the material's
  allowance. `tilt.cone_check` is that check, re-derived from the kinematics
  and not read off a plan field; it belongs in `validate_plan` behind the same
  flag. (Use `arctan2`, not `arccos` — `arccos` is ill-conditioned exactly
  where a flat plan lives and reports ~1e-8 rad of phantom lean.)

  **DONE** (2026-08-24, `docs/FULL_COVERAGE.md`). `validate_plan` takes
  `tilt_max_deg`, defaulting to **0** — so every plan written before tilt
  existed is now checked against the perpendicular pen it was actually asked
  for, and only a plan that was granted a cone may use one. The lean is
  re-derived from `frames.fk` (`validate._pen_lean_deg`), never read off the
  plan; `scene_check` passes each segment's own cone, and `tilt.plan_adaptive`
  records the cone the plan NEEDS rather than the one the run allowed, so
  turning the flag on cannot weaken the certificate of a stroke that did not
  lean. Folding the tip, the pen axis and the chain points into ONE `fk_many`
  call made the check free (it was three calls).
* **The back-out extends by one argument.** `pwl.chase_cc` takes poses and a
  commanded q7 and knows nothing about where the poses came from, so the entire
  certification machinery — gates, branch consistency, tip-error report —
  carries over untouched. The only change needed in `pwl.py` is
  `pen_down_poses(..., tilt=None)` and `stroke_setup(..., tilt_of_s=None)`,
  both backward-compatible by default. `tilt.chase` demonstrates it; **no
  shipping file was modified for this exploration** beyond the opt-in flag.
* **PWL and smoothing extend from 1 field to 3.** The plan stops being
  `s → q7` and becomes `s → (q7, tx, ty)`. RDP generalises directly by
  measuring deviation in grid indices per axis and taking the worst axis
  (`tilt.simplify`). `smooth.RoundedPWL` would need the same treatment —
  quadratic-Bézier corners applied per component with shared windows. **The
  tilt plans here are sharp polylines with no corner rounding**, which is the
  main thing left undone; it matters for `|dq/ds|` and therefore the clock.
  Interpolate `(tx, ty)` as a **vector** — interpolating (θ, φ) across the apex
  would swing the azimuth through half a turn while the lean passes through
  zero, spinning the pen on the paper for nothing.

  **STILL UNDONE, and now said out loud rather than inferred from a missing
  key** (`tilt._flat_shaped_fields`). A tilted plan ships as the sharp
  polyline the DP certified, with every window zero. It costs nothing in the
  certificate — `chase` walks the same 5 mm samples under the same gates
  either way, and the flat pipeline already ships sharp polylines when
  rounding fails to certify — it costs `|dq/ds|` at the knots and therefore
  the clock. On the spans the feature exists for (50 mm and 10 mm rescues at
  the edge of an arm's reach) that is a few knots on a few centimetres. On a
  long tilted stroke it would matter and the generalisation would have to come
  first. What WAS done: `writing.densify` and `writing.lifted_config` take a
  tilt, so the frame fill and the hover pose are solved at the plan's
  orientation instead of silently at `rotx(pi)`, and the tilt vector — not the
  angles — is what gets interpolated.
* **RTff needs nothing.** Force projection uses the waypoint quaternion, so a
  tilted drawing is already executable per the upstream RTff design.
* **The allocator would need to be told.** `allocate.atlas_cells` keeps only
  tilt-0 permissive cells today, so the donut cells this unlocks are invisible
  to placement and colour partitioning until that filter learns about the flag.

  **STILL TRUE, and deliberately not done.** The atlas is the PREFILTER, and a
  prefilter that under-reports only costs probe time — every stroke it lets
  through is probed for real, and `--no-prefilter` recovers the full search. So
  the 100 % run of `docs/FULL_COVERAGE.md` reaches the tilt planner through
  `opts["tilt_max_deg"]` on the probe path and leaves the atlas alone. What
  that means in practice: **tilt cannot yet win a stroke the atlas prefiltered
  away.** It did not need to here.
* **Gate plumbing is a real gap.** `plan_stroke` cannot be asked for a strict
  margin — `pwl.MARGIN_GATE` / `pwl.SIGMA_GATE` are module constants, so an
  `opts["margin_gate"]` is silently ignored. Every strict-gate comparison here
  therefore runs both sides through `tilt.plan_adaptive`.
  `tests/test_tilt.py::test_strict_gates_are_not_a_plan_stroke_option` pins it.

  **FIXED** (2026-08-24). `margin_gate` and `sigma_gate` are `DEFAULTS` entries
  and reach all three places a gate has to arrive: the band DP
  (`pwl.plan_pwl`), the 5 mm certification chase (`smooth.certify`) and the
  independent validator. The test that pinned the bug now pins the behaviour
  (`test_strict_gates_reach_plan_stroke`). The defaults are the shipping
  constants, so no published number moved.

---

## 5. Runtime

**Zero regression when the flag is off or when flat succeeds** — the tilt
planner returns the shipping pipeline's answer *bit for bit* (`np.array_equal`
on the joint trajectory), having paid one status comparison:

| stroke | shipping | tilt ≤ 15° enabled | tilt ≤ 30° enabled |
|---|---|---|---|
| A_rim_arc | 68.7 ms | 68.0 ms (−1.0 %) | 66.7 ms (−2.8 %) |
| R_bowl | 39.6 ms | 39.8 ms (+0.5 %) | 39.7 ms (+0.2 %) |

**Rescues** — arm 2 donut strokes at the strict gate, flat pass then collar:

| stroke | flat pass | adaptive total | collar | tilt IK | oracle | speedup |
|---|---|---|---|---|---|---|
| D4_arc_r13 | 13 ms | **110 ms** | 21 steps | 6 048 | 631 ms | 5.7× |
| D1_radial_180 | 54 ms | **185 ms** | 25 steps | 7 200 | 1 223 ms | 6.6× |
| D2_radial_225 | 40 ms | **188 ms** | 25 steps | 7 200 | 1 345 ms | 7.2× |
| D3_arc_r17 | 54 ms | **279 ms** | 42 steps | 12 096 | 1 696 ms | 6.1× |

All four certified at **stage 0** — the 7-point coarse sublattice — so the fine
19-point disc was never built. Three of four land inside a 200 ms budget; D3,
the longest stroke with the widest collar, is 279 ms.

Getting there took three optimisations worth recording, because the naive
version was 830 ms and **none of the cost was where it was expected**:

1. **The frame-clearance check was 73 % of the wall clock**, against 2 % for
   the analytic IK it exists to gate. `rig_final.chain_static_clearance`
   ternary-searches each capsule against each box for 36 iterations.
   `tilt._frame_clear` screens first with two proved bounds —
   `d_seg ≤ min(d_A, d_B)` rejects, `d_seg ≥ min(d_A, d_B) − L/2` accepts — and
   only the nodes between them reach the exact routine. The mask is identical
   node for node (pinned by a test on 2 500 random configurations).
2. **The gate order was wrong for this workload.** The gates are ANDs over
   independent per-node quantities, so the order is free and only the cost
   differs. `planner` puts σ last because on a flat lattice clearance is cheap;
   with tilt open it is the dominant cost, so σ (one batched SVD) goes first
   and the steel only ever sees postures that already control.
3. **The coarse stage must span the cone, not the middle of it.** "Hex ring ≤ 1
   then everything" samples out to one pitch — 7.5° — so a donut needing 15°
   failed stage 0 for a reason unrelated to sampling density and paid for both
   stages. Taking every *m*-th hex cell instead keeps the centre and the six
   cells **on the cone edge**: 7 points at full lean, and a strict subset of the
   fine disc, so nothing is wasted if the fine stage runs. This alone took
   D1 from 609 ms to 231 ms.

Memory: the oracle lattice is 34.9 MB at 15° and 112.0 MB at 30° against
1.8 MB flat. The adaptive planner allocates the same full array even though it
materialises only the collar — the arrays are dense in the tilt axis. Sparse
columns are the obvious fix and were not needed at these sizes, but a 61-point
disc on a long stroke would want them.

---

## What broke, and what it taught

Four bugs, each of which is a fact about the problem rather than a slip:

* **The one-point disc had no self-edge.** `tilt_max = 0` returned an all-`−1`
  neighbour table, so the tilt axis could not hold still, no DP edge was ever
  feasible, and every stroke cut at step 1. "Tilt does not change" is an edge
  and has to be in the table.
* **Chord-wise feasibility does not compose into stroke-wise feasibility.** RDP
  vetoes each candidate chord with a chase seeded from that chord's own lattice
  node; the certification chase walks the whole stroke and arrives with
  whatever the preceding chords left it. Measured on the R bowl: nine knots
  each individually chaseable, `||dq||_inf = 2.56` rad end to end. Answered with
  a simplification ladder that falls back to keeping every lattice step.
* **The search budget and the execution budget are not the same number.**
  `JUMP_THRESH = 0.35` is what a finished trajectory must satisfy. Using it as
  the *search* edge test while both q7 and tilt move lets the DP propose steps
  `ik.solve_cc` will not walk. But tightening it unconditionally cost the rim
  arc its cheap 2-knot path (travel 5.40 → 19.4 rad), so it is a **fallback**:
  0.35 first, 0.25 only if that does not certify.
* **`arccos` near zero.** A perpendicular plan measured 1e-8 rad of lean and
  failed its own zero-degree cone.

---

## Reproducing

```
ARIS_RIG=sixarm     python3 scripts/tilt_explore.py strokes
ARIS_RIG=final6_opt python3 scripts/tilt_explore.py donut
ARIS_RIG=final6_opt python3 scripts/tilt_explore.py atlas
ARIS_RIG=final6_opt python3 scripts/tilt_explore.py runtime
python3 -m pytest tests/test_tilt.py -q
```

Each part writes `out/tilt_explore_<part>.json`.
