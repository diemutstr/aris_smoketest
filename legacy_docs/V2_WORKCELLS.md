# Staged work cells — can arms be given a region of paper and left alone?

Pete, 2026-09-11: *"I was also wondering if it could make sense to draw things by the one arm per column in a 121 pattern and make those arms the leaders and then have the remaining arms only plan to fill in safe spots they can handle while the leader arms are doing their thing and then afterwards reverse it. If there are ways to restrict the work cell of the arms temporarily that would allow us to do some asynchronous planning which might make things easier. And then we would also need some sort of protocols to decide when we can advance to the next stage."*

Everything below is measured by `scripts/workcell_envelopes.py` on the shipped atlas (`out/atlas_proposed_h0970_lat0860_gated63`, `is_current` = True for all six arms), the shipped capsules, and the arm-to-arm gate in force (`coordination.PAIR_MARGIN` = 50 mm). Numbers in `out/workcell_envelopes.json`. **No constant and no planner semantic changes in this commit.**

## What a stage is, and what it buys

A **stage** assigns each arm a **work cell**: a region of the paper it may draw in. Its **envelope** for that stage is the union of the arm's own metal over every certified pose it could hold while working there — the atlas's drawing pose at every certified cell of the region, the pen-up hover above each of those cells, and the park it starts and ends at. If two arms' envelopes are ≥ `PAIR_MARGIN` apart then **nothing either of them does inside its own region can touch the other**, in any order, at any speed, with any re-plan. Each is a *static* keep-out for the other: no shared clock, no conductor, no re-conducting when one arm is late. That is exactly the asynchrony Pete is asking for, and it is a geometric question with a number for an answer.

Because an envelope is a union over cells, the clearance between two envelopes is a minimum over cell pairs. The script therefore computes **one cell-to-cell clearance matrix per arm pair, once** (15 matrices, ~890 × ~890 cells on a 4 cm lattice, 31 s on 6 processes) using `coordination.ArmPath` + `clearance_matrix` — the shipped measured capsules, exact segment-to-segment distance — and every region, pattern and stage sequence below is a numpy reduction over a slice of it.

Two honest caveats, stated once: the default `--stride 2` samples every second atlas cell in each axis (4 cm lattice, ~890 of ~3 830 certified cells an arm); and the **pen-up leg between two hovers is not sampled** — both its endpoints are in the envelope, the joint-space line between them is not. Everything here is an envelope of *endpoints*. A stage that clears by millimetres is not certified by this script; the recommendation below clears by 86 mm.

## 1. The headline: the elbow, not the pen

Splitting the paper does not split the arms.

| arm | base x | elbow x over all certified drawing poses | pen tip x |
|---|---|---|---|
| 13 | 0.597 | 0.282 … 0.853 | 0.160 … 1.240 |
| 17 | 1.207 | 0.950 … 1.520 | 0.560 … 1.640 |
| 31 | 0.597 | 0.283 … 0.853 | 0.160 … 1.240 |
| 71 | 1.207 | 0.951 … 1.520 | 0.560 … 1.640 |
| 2 | 0.597 | 0.284 … 0.853 | 0.160 … 1.240 |
| 97 | 1.207 | 0.950 … 1.520 | 0.560 … 1.640 |

The two arms of a transverse pair sit 0.61 m apart and **both elbows swing to within 0.10 m of each other at the mid-line** (0.853 and 0.950) — with `ELBOW_R` = 0.117 m that is 135 mm of interpenetration before any other link is counted. Verified independently: arm 31 drawing at (0.44, 1.40) and arm 71 drawing at (1.24, 1.40) — pens **0.80 m apart** — are at **−160.8 mm**, and `scene_check.pair_clearance` returns the same −160.8 mm that `coordination.clearance_matrix` does.

So: on their own Voronoi blocks (which for a regular 2 × 3 base grid *are* the column-by-row blocks, so "own base zone", "thirds of a column band" and "halves of a row band" are three names for one partition), the fifteen arm pairs come out:

| pair | own block vs own block | verdict |
|---|---|---|
| 13–97, 17–2, 13–2, 17–97 | **+250.0 mm** (the broad-phase cap) | GO at 50 and at 80 |
| 13–71, 17–31, 31–97, 71–2 | −127 to −130 mm | NO |
| 13–31, 31–2, 17–71, 71–97 | −223 to −234 mm | NO |
| **13–17, 31–71, 2–97** (same row) | **−262.0 mm** | NO |

## 2. The separation frontier — what a dead band costs

For each pair, confine one arm to `u ≤ t_a` and the other to `u ≥ t_b` along paper axis `u`, and find the smallest band `t_b − t_a` that clears 50 mm. This is the weakest separation hypothesis there is, so it is the strongest statement of what separation costs.

| pair | axis | dead band needed | narrowest band at any placement |
|---|---|---|---|
| 13–2, 17–97, 13–97, 17–2 (two rows apart) | y | **0.00 m** | — |
| 13–31, 17–71, 31–2, 71–97 (adjacent rows, same column) | y | **0.40 m** | 0.32 m |
| 13–71, 17–31, 31–97, 71–2 (adjacent rows, other column) | y | **0.32 – 0.36 m** | 0.28 – 0.32 m |
| 13–71, 17–31, 31–97, 71–2 | x | 0.44 – 0.48 m | 0.20 – 0.28 m |
| **13–17, 31–71, 2–97** (same row) | **x** | **1.44 m** | 0.56 – 0.72 m |

The certified block is 1.48 m wide. **A same-row pair needs a 1.44 m dead band in x — there is no useful split.** Two arms of the same transverse pair can never both be drawing.

Two rows apart, the arms are free: no restriction at all, at the 250 mm clip.

## 3. Pete's 1-2-1, measured

Both readings put a **same-row pair** in the air at once, and that is the one pair geometry forbids.

**(a) leaders = the middle row (31, 71), end rows confined to the paper ends:**

| end arms confined to (south y ≤, north y ≥) | ink-vs-ink |
|---|---|
| 0.40 / 3.22 | −262.0 mm |
| 0.605 / 3.03 | −262.0 mm |
| 0.80 / 2.82 | −262.0 mm |
| 1.00 / 2.62 | −262.0 mm |

The binding pair is **31–71** and it never moves, because the followers' confinement does not touch it: 31 and 71 are the two leaders.

**(b) leaders = the four end arms, middle row confined to a band:** binding pair **13–17**, −262.0 mm at every band height from 0.20 m down. Same reason, other pair.

**Pete's instinct is right and his axis is wrong.** "One arm per column" makes the two leaders a transverse pair. The partition that works is **one arm per *row***, with the two columns alternating between stages — which, note, is exactly the grouping `scripts/csail_schedule.py --arm-phases disjoint` already produces (it colours arms adjacent when their bases are within 0.70 m; on the 2 × 3 grid the only edges are the three transverse pairs, so the colour classes are the two columns = one arm per row each). What that existing grouping does **not** do is impose the y dead band, and without the dead band it is −206 mm.

## 4. Pattern comparison

`cost` is the theoretical parallel makespan under uniform ink density: each block cell is charged to the first (stage, arm) that can draw it, a stage costs its busiest arm, the sequence costs the sum. The serial baseline is 3 458 cells (one arm at a time). `cover` is the fraction of the reachable certified block the sequence draws.

| pattern | stages | cover | ink-vs-ink | + shipped parks | cost | speedup |
|---|---|---|---|---|---|---|
| 6-active, blocks eroded 0.10 m, + seams | 3 | 99.8 % | −254.9 mm | −254.9 mm | 756 | 4.57× |
| 6-active, eroded 0.30 m, + seams | 3 | 99.8 % | −223.5 mm | −223.5 mm | 1046 | 3.31× |
| **1-2-1 (a)** middle row leads | 2 | 99.8 % | **−262.0 mm** | −262.0 mm | 855 | 4.04× |
| **1-2-1 (b)** end rows lead | 2 | 99.8 % | **−262.0 mm** | −262.0 mm | 969 | 3.57× |
| 3-active row bands, no dead band | 2 | 100.0 % | −206.3 mm | −206.3 mm | 1178 | 2.94× |
| 3-active own blocks, no dead band | 2 | 99.8 % | −129.7 mm | −129.7 mm | 1176 | 2.94× |
| 3-active row bands, 0.20 m y dead band, 2 seam stages | 4 | 95.5 % | **+85.8 mm** | +4.7 mm | 1312 | 2.64× |
| 3-active row bands, 0.20 m y dead band, 4 seam stages | 6 | 97.3 % | **+85.8 mm** | +4.7 mm | 1378 | 2.51× |
| the same, seam stages *crossed* (§4b) | 6 | 100.0 % | **−201.5 mm** | −201.5 mm | 1378 | 2.51× |
| **the same, 6 seam stages (§4b — the recommendation)** | 8 | **100.0 %** | **+85.8 mm** | +4.7 mm | 1470 | **2.35×** |
| 3-active own blocks, 0.15 m y dead band, 2 seam stages | 4 | 97.1 % | +1.9 mm | +1.9 mm | 1295 | 2.67× |
| 2-active, one per column | 3 | 99.8 % | −262.0 mm | −262.0 mm | 1746 | 1.98× |
| serial (one arm at a time) | 6 | 100.0 % | n/a | +4.7 mm | 3458 | 1.00× |

Only the two rows in bold with a positive ink figure are stages anyone may run. `3-active row bands, 0.15 m` misses by 3.8 mm and is the shape of the curve: the y dead band goes from −127.2 mm at 0.20 m to −3.8 mm at 0.30 m to **+85.8 mm at 0.40 m**, and there is nothing gradual about the last step — it is the moment the two adjacent rows' elbow sweeps stop overlapping.

## 4b. The seam stages, corrected — a seam needs an outer arm *and* a middle arm

The four-seam-stage version above leaves **92 cells (2.7 % of the block) uncovered, all of them at y ∈ [2.24, 2.40]** — five rows in a single strip, not scatter at the rim. `scripts/traces.py` (docs/V2_TRACES.md) found the same hole from the other end: CSAIL stroke 17 lies entirely inside it.

The cause is arithmetic, not taste. Tip reach over the certified block:

| arms | tip y | reach into SEAM0 = [1.010, 1.410] | into SEAM1 = [2.220, 2.620] |
|---|---|---|---|
| 13, 17 | 0.000 … 1.320 | 1.040 … 1.320 | — |
| 31, 71 | 1.080 … 2.560 | 1.080 … 1.400 | 2.240 … 2.560 |
| 2, 97 | 2.280 … 3.600 | — | 2.280 … 2.600 |

**SEAM0's top 80 mm is reachable only by 31/71 and its bottom 30 mm only by 13/17; SEAM1's bottom 40 mm is reachable only by 31/71 and its top 20 mm only by 2/97.** Each seam has to be offered to one arm of each kind. The four-stage version gave 31 and 71 only SEAM0, so SEAM1's floor belonged to nobody.

**The seam-stage pairing matrix** — arm on SEAM0 against arm on SEAM1, ink-vs-ink, the two bands 0.81 m apart in y:

| SEAM0 \ SEAM1 | 2 | 31 | 71 | 97 |
|---|---|---|---|---|
| **13** | +250.0 | **+194.3** | +250.0 | +250.0 |
| **17** | +250.0 | +250.0 | **+201.1** | +250.0 |
| **31** | +156.4 | *self* | **−163.1** | +250.0 |
| **71** | +238.6 | **−201.5** | *self* | +162.1 |

So, answering the question directly:

- **31 or 71 on SEAM1 against a row-0 arm on SEAM0 clears by +194.3 mm (13/31) and +201.1 mm (17/71)** — comfortably past 50 mm and past the old 80 mm.
- **31 or 71 on SEAM1 against 2 or 97 in the same stage is not a thing that can be arranged**: 2 and 97 only reach SEAM1, so both arms would be in the same 0.40 m band and would overlap outright. In the pairings that *are* possible — 31 or 71 on SEAM0 against 2 or 97 on SEAM1 — the clearance is +156.4 to +250.0 mm.
- **The tempting six-stage fix does not work.** Crossing the middle pair over the two seams (31 → SEAM0, 71 → SEAM1 and vice versa) covers 100 % of the block in the same six stages, and it is **−163.1 / −201.5 mm**. A transverse pair cannot be separated by putting them in different seams either: 0.81 m of y separation is not enough, because both elbows still occupy the same x column around the mid-line. **The transverse pair is unseparable on this rig, on either axis.**
- Against **parked** arms the binding number does not move: +4.7 mm, arms 13 and 17 against each other's parks, exactly as for every other pattern. The park set is the blocker for all of them (§5) and the seam correction neither helps nor hurts it.

**The smallest change that covers y ∈ [2.22, 2.40] is therefore two more seam stages**, not a different split and not a shifted dead band:

| stage | active | region |
|---|---|---|
| 1 | 13, 71, 2 | R0 / R1 / R2, full width, y dead band 0.40 m |
| 2 | 17, 31, 97 | the same bands, other column |
| 3 | 13 → SEAM0, 97 → SEAM1 | +250.0 mm |
| 4 | 17 → SEAM0, 2 → SEAM1 | +250.0 mm |
| 5 | 31 → SEAM0, 97 → SEAM1 | +250.0 mm |
| 6 | 71 → SEAM0, 2 → SEAM1 | +238.6 mm |
| **7** | **13 → SEAM0, 31 → SEAM1** | **+194.3 mm** |
| **8** | **17 → SEAM0, 71 → SEAM1** | **+201.1 mm** |

with `R0 = y ∈ [0.000, 1.010]`, `R1 = [1.410, 2.220]`, `R2 = [2.620, 3.620]`, `SEAM0 = [1.010, 1.410]`, `SEAM1 = [2.220, 2.620]`, all at the full block width `x ∈ [0.16, 1.64]`.

Per-stage and whole-sequence effect of the correction:

| | 6 stages (4 seam) | **8 stages (6 seam)** |
|---|---|---|
| block covered | 97.3 % (92 cells missing at y 2.24–2.40) | **100.0 %** |
| worst ink-vs-ink | +85.8 mm | **+85.8 mm** (unchanged) |
| cost (uniform-ink stage sum) | 1378 | 1470 |
| speedup vs serial | 2.51× | **2.35×** |
| ≥ 2 stage-compatible drawers | 41.6 % | **47.1 %** |

The two extra stages cost **6.4 % of the parallel speedup** (2.51× → 2.35×, because each cell is charged to the first stage that can draw it, so stages 7 and 8 only pick up what stages 3–6 could not) and they buy the last 2.7 % of the block *and* push stage-compatible redundancy from 41.6 % to **47.1 % — the atlas's own ceiling**, because SEAM1 cells now have both a middle arm and an outer arm offering to draw them. That is a better trade than it looks: the correction is the only version of this pattern that is simultaneously 100 % covering and maximally fault-tolerant.


**6-active never clears.** Even eroding every Voronoi block by 0.60 m — leaving 248 of 3 458 cells alive, 7 % of the block — the worst pair is −23.2 mm. All six arms at once is not a margin problem, it is the transverse pairs.

**2-active (one per column) is worse than useless**: it pairs the two arms of a column in one stage, which is fine, but it needs three stages and 1.98× is *below* what 3-active gets in two.

## 5. Stage transitions

**Are the shipped parks inside the arms' next-stage cells?**

| arm | park hover (canvas m) | inside the certified block | inside its own Voronoi cell |
|---|---|---|---|
| 2 | (0.416, 3.581) | yes | yes |
| 13 | (0.060, 0.915) | **no** (x = 0.060 < 0.16) | no |
| 17 | (1.622, 0.365) | yes | yes |
| 31 | (0.060, 2.125) | **no** | no |
| 71 | (1.683, 2.090) | **no** (x = 1.683 > 1.64) | no |
| 97 | (1.388, 3.581) | yes | yes |

Three of the six park *outside* the certified block, which is by design (`layout.PARK_GRID_PROPOSED` maximises park-vs-ink, and off the block is where that is). But the more important number is the second one:

**park vs every other arm's whole-block envelope, at the gate of 50 mm:**

| arm | worst | against |
|---|---|---|
| 2 | +55.4 mm | 97 |
| 31 | +45.4 mm | 71 |
| 97 | +13.7 mm | 2 |
| 71 | +11.9 mm | 31 |
| 13 | **+4.7 mm** | 17 |
| 17 | **+4.7 mm** | 13 |

### The x-erosion frontier for a park — measured 2026-09-11, and it is not a lever

The obvious fix for +4.7 mm is to take the paper under the parked partner's
shoulder away from the arm that is drawing. Measured, per stage, as the smallest
erosion of the ACTIVE arm's cell in x away from its transverse partner's base at
which the partner's shipped park clears the active arm's envelope by 50 mm
(`traces.park_erode`, the per-cell clearance of every certified cell in the
region against the parked pose, at `--stride 2`):

| stage | active → cell | parked | at 0.00 m | 0.40 m | **0.74 m** | erosion for 50 mm |
|---|---|---|---|---|---|---|
| 0 | 13 → R0 | 17 | +4.7 mm | +10.9 | **+26.6** | **1.04 m** |
| 0 | 2 → R2 | 97 | +13.7 | +13.7 | **+26.6** | 1.04 m |
| 0 | 71 → R1 | 31 | +45.4 | +45.4 | +58.0 | 0.44 m |
| 1 | 17 → R0 | 13 | +4.7 | +4.7 | **+59.9** | 0.76 m |
| 1 | 31 → R1 | 71 | +11.9 | +11.9 | **+26.8** | 1.04 m |
| 3, 5 | 2 → SEAM1 | 97 | +37.1 | +37.1 | **+39.5** | 0.80 m |
| 4 | 31 → SEAM0 | 71 | +34.9 | +34.9 | **+36.5** | 0.84 m |
| 2, 6, 7 | the rest | — | +55.4 … +107.4 | — | — | **already clear** |

**The curve is flat, and then it plateaus below the gate.** Nothing is bought at
all until 0.40 m, and by 0.74 m — which is exactly half the 1.48 m block, the
largest erosion that costs *no* coverage, because the two columns erode from
opposite sides and still tile the row band — the worst stage is at **+26.6 mm**
and three more sit at +36.5 to +39.5 mm. **The 50 mm gate is never reached by
an erosion that is free.** What the block costs at each erosion, recomputed over
the 13 650 certified cells with the eight-stage pattern:

| erosion | block covered | ≥ 2 stage-compatible drawers | mean drawers | speedup |
|---|---|---|---|---|
| **0.00 m (shipped)** | **100.00 %** | **48.1 %** (the ceiling) | 1.524 | **2.40×** |
| 0.40 m | 100.00 % | 48.1 % | 1.524 | 2.40× |
| 0.74 m (free limit) | 98.51 %† | 11.7 % | 1.117 | 2.44× |
| 0.84 m | 87.85 % | 11.9 % | 1.119 | 2.74× |
| **1.04 m (the frontier)** | **61.20 %** | **10.7 %** | 1.107 | 3.90× |

† 98.51 % at 0.76 m, the first erosion past the tiling point; 0.74 m itself is
100.00 %. The speedup column *rises* with erosion and that is an artefact worth
stating plainly: the stage sum falls because there is less ink, not because it is
drawn faster. **At the frontier the pattern draws 61 % of the block and keeps a
tenth of its redundancy** — it fails the two things the eight-stage pattern was
adopted for. Eroding in **y** toward the partner's base is worse still: 0.77 m of
a 1.01 m row band for 13 → R0, and 0.78 m of a 0.81 m band for 31 → R1, which
leaves 30 mm of paper.

### The alternative — a retracted park — does not work either

The stage-park search ranked 65 candidates an arm: 12 bearings × 3 radii
(0.55–0.70 m) × 2 hover heights (0.20, 0.35 m), all of them *reaching out over
the paper*. A shoulder cannot leave its base, but an arm can fold itself up and
away, and that is a family the grid never contained. Searched over **24 bearings
× 8 radii (0.30–0.78 m) × 6 hover heights (0.20–0.95 m)** — 485 to 492 certified
poses an arm, **7.5× the grid, spanning the whole reachable height range** — the
binding number **does not move**:

| stage | active | parked | shipped park | best of 490 poses |
|---|---|---|---|---|
| 0 | 13 → R0 | 17 | +4.7 mm | **+4.7 mm** (0 of 488 clear the gate) |
| 0 | 2 → R2 | 97 | +13.7 | **+13.7** (0 of 488) |
| 1 | 17 → R0 | 13 | +4.7 | +8.9 (0 of 485) |
| 1 | 31 → R1 | 71 | +11.9 | **+11.9** (0 of 492) |
| 3, 5 | 2 → SEAM1 | 97 | +37.1 | **+37.1** (0 of 488) |
| 4 | 31 → SEAM0 | 71 | +34.9 | **+34.9** (0 of 492) |

Every seam stage that already cleared has 228–268 poses that clear; every stage
that did not has **zero**, at any height, at any retraction. That is as strong a
confirmation of the shoulder argument as this rig can give: the binding element
is pose-invariant to within 4 mm over the arm's whole certified pose space.

### The recommendation

**Neither. Ship `park_erode_m = 0.0` — the pattern unchanged.** The knob exists
(`traces.zigzag_pattern(park_erode_m=…)`, `traces.park_erode`) so the frontier
can be re-measured and so Pete can see the trade, but at no erosion is the trade
worth taking: the free erosion does not clear the gate and the clearing erosion
costs 38.8 % of the block and three quarters of the redundancy.

**What is left is not geometry.** Three candidates, in cost order, none of them
this pass's to decide:

1. **A parked arm is not a moving arm.** `PAIR_MARGIN` = 50 mm is the gate for
   two arms in motion; a parked arm holds one known, measured, barrier-verified
   pose, and DECISION 2026-09-09 already established that *a known pose stops
   paying for a sweep*. Whether a stationary partner at a verified park deserves
   the same 50 mm as a mover is a question for whoever signs the gate — and if
   it does not, five of the eight stages pass immediately, three of them by
   13–15 mm. **This is the cheapest thing to ask and it changes no code.**
2. **Park the partner off the certified block.** Three of the six shipped parks
   already are; the 490-pose family is `certified_ready_pose`, which searches
   over the paper. A park search whose candidate set is the region *outside* the
   block has not been run.
3. **Do not have a same-row partner in the stage at all** — a five-arm stage,
   which costs a sixth of the fleet and is a scheduling answer, not a geometric
   one.

**The shipped park set is not stage-compatible.** It was searched (DECISIONS 2026-09-10) against the *allocated* ink of one programme and it clears that by 97.7 mm; against everything an arm could be *told* to draw inside a work cell it clears by 4.7 mm. This is the single blocker between the recommended pattern's +85.8 mm of ink-vs-ink and its +4.7 mm once parks are counted, and it is why the pattern table has two clearance columns.

**Is there a certified route from an arm's last hover to its stage park?** Not answered here and not answerable from an envelope: a route is a *path* and this document measures *sets of poses*. What is answerable and is answered: the park pose itself is a member of every envelope the script builds, so the numbers above already include "the arm is standing at its park while its neighbour works". The leg from the last hover to the park would be certified by `paper.route` + `scene_check.check_timeline` against the other arms' envelopes as static boxes, which is a new call, not a new capability.

### The barrier protocol

A stage change is a **rendezvous**: every arm retreats to a certified park inside (or compatible with) its *next* cell, and only then do the next stage's cells become active. Before releasing stage *s+1* the barrier must verify, **per arm, from measured state and not from the plan**:

1. `pen_up` — tip at or above `writing.LIFT_Z` (0.06 m).
2. `at_park` — measured joint vector within tolerance of *the specific* `q_park` that stage *s+1*'s envelope set was computed against. A park identity check, not a pose-shaped check: the guarantee is indexed by which park.
3. `stopped` — joint velocity under threshold. The envelope argument is about poses; `coordination.SWEEP_K` slack exists precisely because a moving link sweeps more than its samples.
4. `queue_drained` — every stroke assigned to this arm in stage *s* is completed, or explicitly re-queued (§6).
5. `no_fault` — no un-cleared error.

And fleet-level, once: 6. all six report 1–5; 7. the held park set equals the set stage *s+1* was certified against.

**What exists already**

| primitive | module | what it does for a stage |
|---|---|---|
| the pairwise envelope | `coordination.ArmPath`, `clearance_matrix`, `PAIR_MARGIN` | the gate itself; this script is the first caller that unions over a *region* |
| freeze / retreat | `idle.conduct` (`POLICY_FREEZE`), `_blocking_freezes`, `plan_retreat`, `retreat_candidates` | an arm that has finished holds a pose and is an obstacle; a pose in somebody's way is moved |
| park-vs-mover, park-vs-ink | `allocate.ParkProbe` | prunes a span whose path conflicts with a parked fleet, at exactly `SAFETY_M + CALIB_M` |
| the certified park set | `layout.certified_park_poses`, `PARK_GRID_PROPOSED`, `Q_PARK_PROPOSED` | the poses a barrier would rendezvous at |
| region-aware parks | `layout.region_aware_parks`, `phase_aside_parks`, `aside_candidates`, `repark_route` | the hook a per-stage park search plugs into — it already ranks parks against a target region |
| arm grouping | `scripts/csail_schedule.py --arm-phases disjoint`, `allocate.park_groups` | already produces exactly "one arm per row, columns alternating" |
| the independent check | `scene_check.check_static`, `check_timeline` | the certificate a stage's own timeline would carry |

**What is missing**

- **A work-cell object.** Nothing in the package takes "this arm may only draw here". `allocate` allocates on cost; the region filter would go on the atlas prefilter (`allocate.atlas_cells`).
- **Per-stage park sets.** `Q_PARK_PROPOSED` is global and, measured above, 4.7 mm from a neighbour's unrestricted envelope. A stage needs its own parks, searched against the stage's envelopes rather than against one programme's allocation. `region_aware_parks` is the right function and it currently ranks against a target *xy*, not a stage envelope.
- **The barrier itself.** There is no fleet-level rendezvous anywhere in the package — `stroke_api`'s "barrier" is an exception barrier. States 1–5 above are all readable; nothing gathers them.
- **The pen-up leg.** The envelope covers hover endpoints, not the joint-space line between them. One `scene_check.check_timeline` per stage per arm against the other arms' envelopes closes it, and it is the only remaining gap between "the envelopes clear" and "the stage is certified".

## 6. Fault tolerance under stages

Of the reachable certified block, **47.1 % has two or more certified drawers** and 3.4 % has three or more. That is the ceiling: it is what the atlas offers, before any stage structure.

A cell is only *re-queueable* if a second arm both certifies it **and** is offered it by some stage of the pattern — a stage assignment nobody measured is not a fallback, it is a new coordination problem. By that measure:

| pattern | block with ≥ 2 stage-compatible drawers | mean drawers per cell |
|---|---|---|
| ceiling (any arm, no stage structure) | 47.1 % | 1.51 |
| **3-active row bands, 0.40 m dead band, 6 seam stages (§4b)** | **47.1 %** — the ceiling | **1.51** |
| 3-active row bands, 0.40 m dead band, 4 seam stages | 41.6 % | 1.41 |
| 3-active row bands, no dead band | 39.0 % | 1.39 |
| 3-active row bands, 0.40 m dead band, 2 seam stages | 35.2 % | 1.31 |
| 6-active eroded 0.30 m + seams | 36.8 % | 1.37 |
| 3-active own blocks (with or without dead band) | 0.0 – 2.5 % | 1.00 |
| 1-2-1, either reading | 0.0 % | 1.00 |

The recommended pattern **reaches the ceiling**: every cell with a second certified drawer is offered to it by some stage, so nothing the atlas makes redundant is thrown away by the staging.


The row-band patterns keep most of the available redundancy (35–39 % of 47.1 %) because each row band is offered to both of its column arms, in different stages. The block patterns keep essentially none: each cell is offered to exactly one arm, so a faulted arm's ink is simply lost.

## 7. Recommendation

**Adopt `3-active-rowband-y20+6seams`: one arm per full-width row band, a 0.40 m dead band in y between bands, the two columns alternating between the two main stages, then the six 2-active seam stages tabulated in §4b.**

- **+85.8 mm** of ink-vs-ink clearance, against a 50 mm gate and comfortably past the 80 mm gate that was in force until 2026-09-09. Every one of the three active arms in a stage is a *static* keep-out for the other two, so within a stage the three plan and move asynchronously with no conductor. The tightest seam stage is +194.3 mm.
- **100.0 % of the certified block**, in 8 stages.
- **2.35× the serial makespan**, against a theoretical ceiling of 3× for three arms.
- **47.1 % of the block keeps a second stage-compatible drawer** for re-queueing a faulted arm's ink — the atlas's own ceiling, i.e. the staging throws none of the available redundancy away.
- It is a restriction of a grouping the repo already computes (`--arm-phases disjoint`), so nothing about allocation or conduction has to be re-derived — the new object is the y dead band, the seam stages, and the per-stage park set.

Do **not** ship the six-stage version: it misses y ∈ [2.24, 2.40] (92 cells, and CSAIL stroke 17 entirely), and the six-stage fix that would cover it — crossing 31 and 71 over the two seams — is at −163.1 mm. The two extra stages are the price of a hole-free block.

**Do not build the 1-2-1 as described.** Both readings require a transverse pair to draw at the same time, and that pair is at −262 mm however the paper is cut. The same idea with rows instead of columns is the recommendation above.

**Before it can run, two things must be built, in this order:**

1. **A per-stage park set.** The shipped parks are 4.7 mm from a neighbour's work-cell envelope and that, not the ink, is what fails the gate. Search parks against the stage's envelopes with `layout.region_aware_parks`' machinery; the stage is only as good as its barrier pose.
2. **The barrier** (states 1–7 of §5) plus one `scene_check.check_timeline` per arm per stage against the other arms' envelopes as static boxes, which closes the pen-up-leg gap.

Neither changes a constant, a gate, or a planner semantic.

## Reproducing

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 scripts/workcell_envelopes.py \
    --atlas out/atlas_proposed_h0970_lat0860_gated63 \
    --stride 2 --jobs 6 --json out/workcell_envelopes.json
```

`--stride 1` is the whole 2 cm atlas (≈16× the matrix work) and is the number to quote before anything is built. `--keep-sweep-band` keeps link1's revolution sweep, which every pose in an envelope has already paid for as a real link (DECISION 2026-09-09, "a known pose stops paying for a sweep"); dropping it is worth 8–60 mm here and is on by default.
