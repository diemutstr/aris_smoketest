# V2 — the architecture that draws a thousand lines and survives a fault

Pete's two sentences are the whole specification:

> *"structure the entire setup to make it fault tolerant and be able to handle
> on the order of 1000 lines, while starting to move relatively quickly (less
> than 10 seconds)"*, and *"we want to make sure that the planning times don't
> take too crazy long"*.

Today the CSAIL logo — 39 strokes, 16.805 m of ink — takes **3 712.4 s** from
picture to certified programme and does not move a joint until all of it is
done (`docs/V2_SCALING_BASELINE.md` §2.1). This document is the design that
replaces that, assembled out of three measurement passes taken on 2026-09-11
(`docs/V2_SCALING_BASELINE.md`, `docs/V2_WORKCELLS.md`, `docs/V2_TRACES.md`),
the seam correction that came out of them later the same day
(`docs/V2_WORKCELLS.md` §4b, commit `07da40e`), the hardware plan
(`docs/HARDWARE_LADDER.md`) and the decisions all of those recorded.
**Nothing in this document changes code.** Every number in it is cited to the
file that measured it.

---

## 1. The explainer, and the budget

### 1.1 The one-paragraph version

A picture becomes a set of lines. The lines are cut wherever the set of arms
that could draw them changes, and each fragment is assigned a drawer so that
the number of continuous **pieces** is smallest — an exact dynamic programme
that costs milliseconds. The pieces are grouped into **stages**. In a stage,
three arms each own a full-width band of paper with a 0.40 m gap between bands,
and because those bands are far enough apart that no pose one arm can hold
comes within the 50 mm gate of any pose another can hold, the three arms are
**static keep-outs for each other**: they plan and move asynchronously, with no
shared clock and no conductor. A **barrier** — every arm pen-up, stopped, at
its own certified park for the next stage, with its queue drained — advances
the fleet from one stage to the next. The bands swap columns, the arms draw
again, and six short two-arm stages come back for the seams between the bands.
Eight stages in all. Only those seam stages, where two arms genuinely share
space, go through the existing coordinated conductor. Planning runs *behind*
execution rather than in front of it, so the first arm starts moving inside a
second.

### 1.2 The planning-time budget

Everything in the "cost today" column is measured on GUI job
`out/gui_jobs/20260910-124546-ce72` (the v19 run) by `scripts/job_substages.py`
and `scripts/profile_stroke_costs.py`, reported in `docs/V2_SCALING_BASELINE.md`
§1–2, except the traces row, which is `aris_sixarm/traces.py` measured in
`docs/V2_TRACES.md` §4.

| step | cost today | v2 target | mechanism |
|---|---|---|---|
| picture to lines (trace) | 0.3 s for 39 strokes | unchanged | not a scaling term |
| lines to pieces | not a separate step; the allocator produces pieces as a by-product of a cost search | **< 1 s for 1 000 lines** | `traces.py`'s per-line min-pieces DP: 884 ms of capability-map reading plus **10 ms** of DP for 992 lines and 4 142 atoms |
| assignment and load balance | **1 968.8 s, 53.0 %** of the run (balance, split, merge, Held–Karp tours, the unsplit A/B re-allocation) | **0 s** | the capability map *is* the assignment; balance is a free lexicographic tie-break inside the same DP |
| one stroke's plan | **0.19 s median, 2.2 s p95** (`plan_stroke`, certified whole) | unchanged per call, but **off the critical path** | streamed behind execution on six workers; it is the only artwork-dependent term and there is nothing to precompute |
| one pen-up leg | **62 ms median / 4.2 s p95 cold**, 3.1 ms on a cache hit | **3.1 ms** | a persistent leg-certification cache keyed on `(rig, tool, atlas)` rather than on process memory |
| the whole allocation, cold against warm | **2 899.7 s cold, 79.4 s warm — 36.5×**; `replan` alone is **63.6×** cheaper the second time on the same spans | always warm | the same persistent cache; today every byte of that warmth dies with the process |
| conduct | **407.3 s** for 39 strokes: 2 380 priority-DP solves at **131 ms** each, and `∑ₖ P(n, k)` in the arm count — about **8 400 s** at 1 000 strokes, longer than the drawing | **zero for the main stages, bounded for the seam stages** | work cells make the three actives in a stage static keep-outs, so there is nothing to conduct; the seam stages conduct two arms over a short horizon |
| independent check | **0.323 s per second of timeline** on the real v19 programme (0.128–0.187 s/s synthetic) | the same rate, run **per window** rather than per phase | `scene_check` is not pair-dominated — 2.5× the pairs costs 1.46× the time — so it stays affordable and stays unconditional |
| **time to first motion** | **3 712.4 s** — the whole allocation gates it | **< 10 s** | **0.86 s at the median**, 6.9 s on a pessimistic p95 bound, at today's per-call prices with no planner change at all |

Two readings of that table decide the architecture. The first is that **the
3 712 s is not the optimiser being slow** — it is first-touch certification of
pen-up geometry landing in process-global dictionaries that are thrown away
when the process exits. The second is that **the 10 s goal was never a
planner-speed problem**; it is a question of what gates the start, and today the
answer is "everything".

---

## 2. The pipeline as it will be

### (a) Lines to pieces — `traces.py`

Sample each line every 4 mm and read its **capability set**: the `(stage, arm)`
pairs whose arm has a certified drawing pose there (`atlas.strict_go`, margin ≥
0.30 and σ_min ≥ 0.14) and whose work cell for that stage contains the point.
Cut wherever that set changes and bisect each cut to 0.01 mm. Between two cuts
every point of the line has exactly the same possible drawers, so the stretch is
indivisible and nothing is ever gained by cutting inside it. Choosing one pair
per atom to minimise the number of maximal runs is then a chain DP with unit
cost per change, and because the change cost does not depend on what you change
*from*, the `O(states²)` relaxation collapses to `O(states)` a step. Per line is
enough: the piece count is a sum over lines and one line's atoms constrain no
other, so minimising each line minimises the total.

Seams get opposite treatment by kind. Where two arms both certify a stretch, the
switch point inside it is arbitrary, so the seam goes at the **middle of the
overlap** and each side **overdraws** by δ = 5 mm in its own drawing direction,
clipped to the zone, so the strokes lap rather than abut. Where the overlap is
empty — a hard edge — the pieces meet at the transition point exactly and the
seam is only as good as the calibration. A run shorter than 10 mm is absorbed
into whichever neighbour can draw all of it; absorption can only reduce the
piece count, so it cannot break optimality, and measured on 400 random chains it
never fires because the DP's answer already contains no absorbable short piece.

Load balance enters as a **tie-break, not a weight**. The DP's value is the
lexicographic pair `(pieces + w × imbalance, imbalance)` with `w = 0` by
default, so the first component is the piece count and the second breaks its
ties toward the least-loaded state. Where the capability sets leave slack the
free tie-break takes all of it — a scribble set goes from 79.1 % spread to
7.5 % at zero cost. Where they do not, a real weight buys balance at about one
extra piece per 13 cm of imbalance, and a piece is a pen-up, a transit and a
seam. The lever stays for a stage that turns out makespan-bound rather than
piece-bound (`docs/V2_TRACES.md` §6).

Measured cost: 1 000 lines is 2 885 atoms and **814 ms**, of which the DP is
10 ms. The output is 1 661 pieces staged against 1 254 unstaged. *Those two
piece counts, and every other number in `docs/V2_TRACES.md`, were measured on
the superseded six-stage pattern* — `traces.zigzag_pattern` has not yet been
given stages 7 and 8 (build item 3). For the CSAIL logo the correction is known
and costs four pieces, 50 → 54, for the 5.2 % of ink the six-stage version could
not draw; at 1 000 lines the equivalent re-run has not been done.

### (b) The stage plan — the zigzag, as data

`traces.StageCell(stage, arm, region)` is the work-cell object: *"this arm may
draw inside this region during this stage, and nowhere else."* The recommended
pattern is `3-active-rowband-y20+6seams`, tabulated in `docs/V2_WORKCELLS.md`
§4b with the measured ink-vs-ink clearance of each seam stage:

| stage | arms and cells | ink-vs-ink |
|---|---|---|
| 1 | 13 → R0, 71 → R1, 2 → R2 | +85.8 mm |
| 2 | 17 → R0, 31 → R1, 97 → R2 | +85.8 mm |
| 3 | 13 → SEAM0, 97 → SEAM1 | +250.0 mm |
| 4 | 17 → SEAM0, 2 → SEAM1 | +250.0 mm |
| 5 | 31 → SEAM0, 97 → SEAM1 | +250.0 mm |
| 6 | 71 → SEAM0, 2 → SEAM1 | +238.6 mm |
| 7 | 13 → SEAM0, 31 → SEAM1 | +194.3 mm |
| 8 | 17 → SEAM0, 71 → SEAM1 | +201.1 mm |

with `R0 = y ∈ [0.000, 1.010]`, `R1 = [1.410, 2.220]`, `R2 = [2.620, 3.620]`,
`SEAM0 = [1.010, 1.410]`, `SEAM1 = [2.220, 2.620]`, all at the full block width
`x ∈ [0.16, 1.64]`.

Three measurements shape that table, and each one closed off something simpler.

The axis matters and it is not the obvious one. **Splitting the paper does not
split the arms**: every arm's elbow swings to within 0.10 m of the paper's
mid-line while it draws, and with `ELBOW_R` = 0.117 m the two arms of a
transverse pair interpenetrate by 135 mm before any other link is counted. Arm
31 drawing at (0.44, 1.40) and arm 71 drawing at (1.24, 1.40) — pens 0.80 m
apart, both poses certified — are at **−160.8 mm**, confirmed independently by
`scene_check.pair_clearance` and `coordination.clearance_matrix`. A same-row
pair needs a 1.44 m dead band in a 1.48 m block, which is to say there is no
useful split.

**And the other axis is no better.** The tempting six-stage fix for the SEAM1
hole was to cross the middle pair over the two seams — 31 onto SEAM0 and 71
onto SEAM1, and the reverse — which covers 100 % of the block in six stages.
Measured, it is **−163.1 mm and −201.5 mm**: 0.81 m of *y* separation does not
separate a transverse pair either, because both elbows still stand in the same
x column about the mid-line. The x frontier said a same-row pair needs 1.44 m of
a 1.48 m block; the y answer is that there is no y answer. **The transverse pair
is unseparable on this rig, on either axis — no pattern may ever put a same-row
pair in the air together.** One arm per *row*, with the two columns alternating
between stages, is the only partition that works, and it is a restriction of a
grouping the repo already computes, `scripts/csail_schedule.py --arm-phases
disjoint`.

The dead band between rows is a cliff rather than a slope: the binding pair is
at **−127.2 mm** with 0.20 m of band, **−3.8 mm** at 0.30 m, and **+85.8 mm** at
0.40 m, which is the moment two adjacent rows' elbow sweeps stop overlapping.
0.40 m is therefore the number, and it clears the 50 mm gate comfortably and
also clears the 80 mm gate that was in force until 2026-09-09. (A naming trap
worth stating once: the pattern is called `…-y20` and
`traces.zigzag_pattern`'s name is built from `int(dead_band_m × 50)`, because
`20` is the **per-side erosion in centimetres** — each row band gives up 0.20 m
at each internal boundary, for a 0.40 m band. `docs/V2_WORKCELLS.md` §4's table
labels the same row "0.20 m y dead band" while its own prose two paragraphs
below calls 0.20 m the −127.2 mm case. Both are the same pattern; only the
convention differs.)

**And a seam needs an outer arm *and* a middle arm, which is why there are six
seam stages rather than four.** Tip reach over the block is `[0.000, 1.320]` for
13 and 17, `[1.080, 2.560]` for 31 and 71, and `[2.280, 3.600]` for 2 and 97,
against seams at `[1.010, 1.410]` and `[2.220, 2.620]`. Each seam's far end is
therefore reachable by exactly one kind of arm and its near end by the other.
The four-seam version gave 31 and 71 only SEAM0, so SEAM1's floor —
`y ∈ [2.24, 2.40]`, 92 cells, 2.7 % of the block — belonged to nobody, and CSAIL
stroke 17 lies entirely inside that strip. `traces.py` found the same hole from
the other end, which is how it surfaced. The two extra stages cost **6.4 % of
the parallel speedup** (2.51× → **2.35×**, since each cell is charged to the
first stage that can draw it, so stages 7 and 8 only pick up what 3–6 could not)
and they buy the last 2.7 % of the block *and* push stage-compatible redundancy
from 41.6 % to **47.1 %, the atlas's own ceiling**. That is a better trade than
it looks: it is the only version of this pattern that is simultaneously
hole-free and maximally fault-tolerant.

The pattern draws **100.0 % of the certified block in 8 stages at 2.35× the
serial makespan**, against a theoretical ceiling of 3× for three arms, with the
tightest seam stage at +194.3 mm. The comparison that decides it: 6-active never
clears at all — eroding every Voronoi block by 0.60 m leaves 7 % of the block
alive and the worst pair still at −23.2 mm.

**Per-stage parks are part of the stage, not a detail.** The shipped
`Q_PARK_PROPOSED` was searched against the *allocated* ink of one programme and
clears that by 97.7 mm; against everything an arm could be *told* to draw inside
a work cell it clears by **4.7 mm** (arms 13 and 17 against each other), and
three of the six parks hover outside the certified block entirely. That 4.7 mm,
not the 85.8 mm of ink, is what fails the gate — and it fails it identically for
**every** pattern measured, so the seam correction neither helps nor hurts it.
It is the one blocker the choice of pattern cannot move.

### (c) Per-(arm, stage) planning and ordering — asynchronous and streamed

Inside a stage there is no conductor and no global optimisation. Each active arm
takes its own bucket of pieces, plans each one with `plan_stroke`, derives the
two endpoint hovers, certifies the pen-up leg between consecutive pieces through
`paper.route`, and orders its bucket with `sequence.py`. A piece is
`(stage, arm, polyline)`, which is exactly the `segs` contract
`sequence.cost_matrix` and `sequence.solve` already take, so one tour per
(stage, arm) bucket needs no new sequencer.

Three properties make this the whole of the planning cost. Per-stroke planning
is embarrassingly parallel across strokes. Everything downstream of the stroke
is length-independent — a hover is a pose and a leg is a pose pair, so
`lifted_or_lower` is 9 ms whatever the stroke and a route's cost is set by how
far down the shape ladder it fell, not by metres. And the whole of it runs
*behind* the arms: only the first piece of the first arm gates the first motion.

At 1 000 strokes this is **1 320 s on six workers cold and about 400 s with a
warm leg cache**, against a draw of roughly 4 300 s — three to eleven times the
headroom needed to keep up (`docs/V2_SCALING_BASELINE.md` §3.3).

### (d) Seam and residual stages — the conductor, and only here

The six seam stages are two-active, and two arms in adjacent cells are the one
case the envelope argument does not settle for free. Those go through
`idle.conduct` unchanged. The cost collapses for a structural reason rather than
an implementation one: the priority search enumerates `∑ₖ P(n, k)` orders, which
is 720 orders of six moving arms and **four** orders of two, and each solve is
linear in a horizon that is a fraction of the whole programme. The 2 380 DP
solves at 131 ms that made up 76.5 % of v19's conduct are a six-arm number.

This is the one structural change v2 cannot avoid. Conducting a 1 000-stroke
timeline as one phase is about **8 400 s** — the whole drawing over again — and
no amount of parallelism fixes it, because the search is combinatorial in the
arm count. Reducing the conducted arm count to two, over short horizons, is what
makes the conduct disappear from the budget.

### (e) Execution — per-arm state machine, barrier, soft fleet clock

Each arm runs a small state machine over its own queue of pieces: *approach* to
the piece's entry hover, *draw*, *ascend* to the exit hover, *transit* to the
next entry hover, and *park* when the queue drains. Within a stage the arms
advance independently, because the stage's certificate is geometric and does not
depend on when anybody is where.

A stage change is a **rendezvous**. Before releasing stage *s+1* the barrier
verifies, per arm and **from measured state rather than from the plan**: the pen
is up (tip at or above `writing.LIFT_Z` = 0.06 m); the measured joint vector
matches *the specific* `q_park` that stage *s+1*'s envelope set was computed
against, which is a park **identity** check because the guarantee is indexed by
which park; the arm is stopped, because the envelope argument is about poses and
`coordination.SWEEP_K` exists precisely because a moving link sweeps more than
its samples; the arm's stage queue is drained or explicitly re-queued; and there
is no un-cleared fault. Fleet-level, once: all six report all five, and the held
park set is the one the next stage was certified against.

The fleet clock stays **soft**. `execute.Governor` is one scalar rate for the
whole fleet and that is its entire safety argument — `scene_check` certified six
arms against each other on a shared clock, so scaling that clock keeps every
pair at configurations that were evaluated, and there is deliberately no per-arm
entry point anywhere in the package. Under work cells the argument is no longer
needed *within* a stage, because the actives are separated geometrically rather
than temporally; it is still needed inside a conducted seam stage. So the
governor stays, scoped to the conducted stages, and the main stages need only a
common notion of "which stage are we in".

### (f) The independent check, per window

`scene_check.check_timeline` stays unconditional and becomes the per-window
certificate rather than the per-phase one. It costs 0.323 s per second of
timeline on the real v19 programme, so at 1 000 strokes it is roughly 1 600 s
against a draw of 4 300 s — one core keeps up with a factor of nearly three in
hand. It is not pair-dominated, which is what separates it from
`coordination.free_cells`: most of its cost is per-arm work (self-collision,
frame, neighbour base columns, paper clearance, joint limits) that scales with
arms and samples rather than with pairs.

It also closes the one gap the envelope argument leaves. An envelope is a union
over *poses* — the drawing pose at each certified cell, the hover above it, and
the park — and the pen-up leg between two hovers is a *path*, whose interior is
not in the envelope. One `check_timeline` per arm per stage, against the other
arms' envelopes as static boxes, is the call that closes it, and it is a new
call rather than a new capability.

---

## 3. What must be built

Dependency-ordered. Size is S (a day or two), M (a week), L (more than a week or
needing a measurement first).

**1. A persistent leg-certification cache — M.** *Extends* `paper.route_key`,
`paper.cache_route` and `paper.cached_route` (`paper.py:233`, `:256`, `:261`),
which are already public precisely so that a route computed in a worker can be
filed by the parent. *The work is not serialisation.* Both memo keys begin with
`id(spec)` — a CPython object address (`paper.py:302` for routes, `paper.py:1409`
for legs) — so the keys are process-local **by construction**, and a persisted
cache means re-keying on a content signature before anything is written. The
certificate it must carry, by analogy with `atlas.model_signature`
(`atlas.py:86`, 810 entries, enforced by `atlas.is_current` at `atlas.py:106`):
the whole collision model, `SEARCH_POLICY`, `GATE_CONE_DEG`, plus
`paper.FRAME_FLOOR`, `STATIC_SAFE`, `SELF_SAFE` and the RRT flag, which are in
the memo key for a reason. It must also record the gate floor it was certified
at — the shipped atlas was swept at `rig_final.STATIC_MARGIN` = 50 mm while the
router flies at `FRAME_FLOOR` = 63 mm, so the shipped atlas is an optimistic
prefilter by up to 13 mm. *Buys:* the 4.2 s route p95 becomes a 3.1 ms lookup;
`replan`'s 63.6× warm speedup stops being an accident of running two allocations
in one process; the streaming figure for 1 000 strokes on six workers goes from
1 320 s to about 400 s; and the tail on time-to-first-motion disappears.

**2. Per-stage park sets — M.** *Extends* `layout.region_aware_parks`
(`layout.py:778`), `phase_aside_parks` (`:856`), `aside_candidates` (`:714`) and
`repark_route` (`:934`). The machinery is right; the signature is not —
`region_aware_parks` ranks candidates against a target *xy*, and a stage needs
them ranked against a stage **envelope**. *Certificate:* each stage's park set
re-run through `scripts/workcell_envelopes.py` and `scene_check.check_static` at
`PAIR_MARGIN`. *Buys:* no planning time at all. It is the single blocker between
the recommended pattern's +85.8 mm of ink-vs-ink and its +4.7 mm once parks are
counted, and a stage is only as good as its barrier pose.

**3. A work-cell object the allocator accepts, and stages 7–8 in the pattern —
S.** The object exists: `traces.StageCell` (`traces.py:254`) and
`traces.zigzag_pattern` (`:305`). Two things are missing. First,
`zigzag_pattern` still returns the **six-stage** pattern superseded by
`docs/V2_WORKCELLS.md` §4b (commit `07da40e`) — two `StageCell` rows, `13 →
SEAM0` with `31 → SEAM1` and `17 → SEAM0` with `71 → SEAM1`, plus a re-run of
every table in `docs/V2_TRACES.md`, and the `2.31 % uncovered` assertion in that
document becomes zero. Second, the region filter on the atlas prefilter —
`allocate.atlas_cells` (`allocate.py:3686`) and `allocate.prefilter` (`:3792`) —
so that the allocator can be told "this arm may only draw here". *Buys:* nothing
directly; prefilter is 0.3 s of a 3 712 s run. It is the enabling change for
everything stage-local, and therefore what makes the 1 968.8 s of global balance
removable.

**4. Stage assignment wired from `traces.py` into the pipeline — M.** *Extends*
`traces.plan_lines` and `sequence.cost_matrix` (`sequence.py:617`). The shapes
already match: a piece is `(stage, arm, polyline)`, which is the `segs` contract
`allocate` hands the sequencer today. The real work is the **refusal loop** —
every number in `docs/V2_TRACES.md` is what the 2 cm atlas *permits*, not a
plan, and a piece `plan_stroke` refuses has to split and re-enter the DP with a
capability map re-derived from the refusals. *Buys:* balance, split, merge and
the Held–Karp tours, **1 968.8 s down to under one second**.

**5. A typed program carrying per-piece hovers, joints and stage ids — M.**
*Extends* `program_schema`. `Segment` (`program_schema.py:203`) carries no joint
vector at all, and `export_bundle` (`:442`) drops the `q_first`/`q_last` that
`scripts/csail_allocate.py:395` already writes into `<stem>_program.json`.
`_from_dict` refuses unknown *and* missing keys, so this is a `SCHEMA_VERSION`
bump (currently 1, `program_schema.py:48`) rather than a silent extension —
which is the right property. *Buys:* re-queueing a piece to another arm becomes
a lookup instead of a `replan_same_span` call followed by a re-conduct; and a
stage id is what makes a barrier addressable at all.

**6. A per-arm runtime state machine and the barrier — L.** *Extends*
`aris_sixarm/execute/`. `Barrier` (`execute/program.py:56`) already has kinds
`("start", "pen_swap", "phase_end", "end")` and `Barrier.mismatch(measured,
tol=0.02)` is the only place in the whole stack where the real robot's position
is checked against the plan; `FleetProgram.barrier_before(t_s)` — exactly the
lookup a resume needs — exists at `execute/program.py:214` and **has no caller**.
`runner.play` (`execute/runner.py:72`) is open-loop everywhere except at a
barrier, by design. What is new: a `stage` barrier kind; the five per-arm
preconditions of §2(e) gathered from measured state; per-arm segment state,
which means reading `seg_<arm>` — already written into the `.npz` and never
read; and a per-arm queue with a re-queue entry point. *Certificate:* one
`scene_check.check_timeline` per arm per stage against the other arms'
envelopes as static boxes. *Buys:* the barrier is what lets three arms in a
stage run with no conductor, which is the 8 400 s.

**7. Interior-ascent recovery routes — L.** *Extends*
`allocate.PlacedIndex._hovers` (`allocate.py:1009`), which today solves
`writing.lifted_or_lower` for `k in (0, -1)` and for no other `k`, so an
interior sample of a stroke has a certified drawing pose at z = 0 and nothing
above it. The fix is to key hovers by **atlas cell** rather than by span
endpoint: one hover pose per strict-GO cell per arm is 6 × ~3 900 × 7 float32 =
**655 kB** and, at the measured 9 ms per cell, **212 s serial or about 35 s on
six workers**. *Buys:* no planning time; it is the fault-tolerance half of the
same artifact, and it is what a reflex stop at s = 0.4 needs.

**8. The hover roadmap's edge layers — L, and later.** Park-to-hover legs for
every node are 2 × 23 500 routes, **13.5 h serial or 2.3 h on six workers** at
the measured 1 151 ms cold mean. Hover-to-hover cannot be a complete graph —
3 900² is 15.2 M edges per arm — so it must be sparse: k-nearest-in-xy plus the
park as a hub, at k = 8 giving 187 000 edges, **25 h serial / 4.2 h on six
workers**, about **32 MB** stored. That is an overnight build per rig and tool,
bought once. *Buys:* the last of the pen-up cost, and a certified way out of
anywhere rather than only off the two endpoints.

Items 1–3 are independent of each other and can go in parallel. Item 4 needs 3;
item 6 needs 2 and 5; item 8 needs 1's key design and 7's node layer.

---

## 4. Fault tolerance

### 4.1 The failure model

| failure | what it needs | what exists | what is missing |
|---|---|---|---|
| **reflex stop mid-stroke** | a certified path off an **interior** point of a span | nothing: only `k ∈ (0, −1)` have a hover (`allocate.py:1009`) | item 7 — a hover per certified cell. At today's cold prices, improvising one at runtime is a `lifted_or_lower` plus a cold `paper.route`: 9 ms plus up to 4.2 s |
| **an arm out for the performance** | re-queue the piece, with its capability set, to a stage-compatible drawer | the capability set is already computed, and under the eight-stage pattern **47.1 % of the block keeps a second stage-compatible drawer — the atlas's own ceiling**, mean 1.51 drawers per cell, with **no orphan cells**: the staging throws away none of the redundancy the atlas offers | a runtime re-queue (items 5 and 6). `allocate.replan_same_span` (`allocate.py:1022`) is a *planning* call: it runs the full stroke planner and the result has to be re-conducted |
| **a lost or slipped pen** | a width read-back at the clamp, and a barrier that refuses | nothing in this repo. The GUI commands **0.0432 m at 70 N**, and libfranka only calls a grasp successful above `width − epsilon_inner`, so the reported number is a **lower bound** on the real jaw gap | `Fr3BundleBackend.read_state` raises `NotImplementedError`. `Barrier.mismatch` would catch a gross slip through joint drift, but not a pen that moved in the holder |
| **network or driver loss** | a timeout that brakes, and a fleet that stops together | on the driver side: `--fr3_command_timeout_sec` = 0.01 and a latching `--fr3_tracking_fault_rad` = 0.020 over 3 ticks | on this side, `Backend.stop(reason)` is a refusal-to-continue and `Fr3BundleBackend.stop` raises. **The physical e-stop is the abort path** — `fr3drivers`' own notes say the 2 rad/s² brake and the latching watchdog are not (`docs/HARDWARE_LADDER.md:51`) |

### 4.2 The two units that make recovery expressible

**The checkpoint is the stage.** A barrier is a state the whole fleet is known
to be in — pen-up, stopped, at an identified park, queues drained — and it is
the only state from which a re-plan is cheap, because it is the state every
stage's envelope set was certified against. Recovery therefore never has to
re-derive a mid-flight scene: it retreats to the nearest barrier and re-enters.

**The re-queueable unit is the piece, carrying its capability set.** A piece is
`(stage, arm, polyline)` and the DP already knows, for every atom of it, the
full set of `(stage, arm)` pairs that could have drawn it. Handing a faulted
arm's remaining pieces to a stage-compatible alternative is then a table lookup
plus a `plan_stroke`, not a coordination problem — provided the alternative was
**offered by some stage of the pattern**. A cell a second arm certifies but no
stage offers is not a fallback; it is a new coordination problem. The
six-seam-stage correction matters here for exactly that reason: it lifted the
stage-compatible figure from 41.6 % to 47.1 %, which is the atlas's own ceiling,
so the two numbers now coincide and there is no redundancy left on the table for
a future pattern to recover.

### 4.3 What the runtime knows today, honestly

`aris_sixarm/execute/` has three moving parts and no state machine.
`Governor` is one clock. `Barrier` is the only measured-versus-planned check.
`runner.play` advances the governor, samples every arm at the same program time
and sends the whole fleet. State that exists per run, in memory: the governor's
rate and program time, the per-arm tracks, the barrier queue, and a `RunLog`
returned at the end and written by nobody. **State that does not exist
anywhere**: which segment is in flight, which strokes are done, pen-down versus
pen-up, or any mapping from program time back to a stroke. The failure model
pinned by `tests/test_execute.py` is entirely stop-and-refuse: no retry, no
degraded mode. Two of the four gaps are cheap — hovers in the schema is a
version bump, and `seg_<arm>` is already in the `.npz` and merely unread.

---

## 5. Open questions for Pete

**1. Does the SEAM1 fix clear? — SETTLED, and it needs following into the
code.** `docs/V2_WORKCELLS.md` §4b ran the pairing matrix and the answer is yes:
13 on SEAM0 against 31 on SEAM1 clears by **+194.3 mm** and 17 against 71 by
**+201.1 mm**, both comfortably past the 50 mm gate and past the old 80 mm one.
Those two stages are now part of the recommendation. What remains is not a
measurement but a code change and a re-run: `traces.zigzag_pattern` still builds
the six-stage pattern, so the CSAIL logo's 50 pieces at 94.8 % coverage and the
1 000-line set's 1 661 pieces at 97.05 % are all figures for the superseded
version. The logo's corrected answer is known — 54 pieces at 100 % — and the
1 000-line figures are not. *This is build item 3 and it is small.*

**2. How wide should the dead band actually be?** The frontier is a cliff —
−127.2 mm at 0.20 m, −3.8 mm at 0.30 m, +85.8 mm at 0.40 m — and nobody has
measured between 0.30 and 0.40. Every centimetre of band is paper handed to a
two-active seam stage rather than drawn in a three-active main stage, which is
part of why the speedup is 2.35× against a ceiling of 3× — and now that the
seams need six stages rather than four, a narrower band is worth more than it
was. *The measurement:* the same script at `--stride 1` (the full 2 cm atlas,
≈16× the matrix work, and the number to quote before anything is built)
sweeping 0.32–0.40 m, plus the resulting piece counts from `traces.py` at each
band width.

**3. How many arms can be concurrent on the real hardware?** The architecture
assumes three arms moving independently, which on the hardware means three
processes, because libfranka's loop owns its thread and its socket. **There is
no cross-process fleet clock** — `Governor` is the right shape and lives in one
process — and it is not started. Also, **arm 71 has no IP and never has had
one**; the GUI's `LIVE_ARM_IDS` is five arms. *The measurement:* rung 4 of
`docs/HARDWARE_LADDER.md` — two arms, one phase barrier, realised inter-arm
clearance against the certified one, and barrier synchronisation jitter.

**4. What paces the arms, now that acceleration is the binding limit?**
Measured on v18 through the execution adapter: peak joint **speed** is a
comfortable 30 % of `QD_MAX` on every arm, and peak joint **acceleration** is
**33–37 rad/s²** against the `fr3drivers` gate's 10 — which the driver's own
flags call "the single most dangerous field". `pacing.py` says why in its own
words: the v profile is a ceiling, not a trajectory. Time scaling alone will not
fix it: it shrinks acceleration as `factor⁻²` but the peaks are at C⁰ corners.
*The measurement:* a bounded-acceleration re-parameterisation run against the
same certified path, then `Fr3BundleBackend.preflight`. The tube argument is
already explicit in the bundle format — a time scaling leaves every control
point untouched, so every collision certificate survives — so this is a
re-timing question, not a re-planning one.

**5. Should the capability map be eroded?** A continuous stroke sample rounds to
its nearest 2 cm cell and can sit up to 14 mm outside the region that cell
certifies, which is why `dead_spans.go_cells` erodes by one cell. `traces.py`
can (`--erode`) and by default does not, matching `certified_area` and
`workcell_envelopes`. Eroding makes every number more conservative and none of
them wrong. *The question:* whether a hand-over should be **claimed** on an
un-eroded map is for whoever signs off the seam.

---

## 6. How a thousand-line drawing would run

Times are the measured medians of `docs/V2_SCALING_BASELINE.md` §1 unless
marked. The drawing is 1 000 lines at the CSAIL median stroke of 0.347 m — 347 m
of ink, 20.7× the logo.

**t = 0.00 s.** The picture is traced. 0.3 s for the logo's 39 strokes; not a
scaling term.

**t ≈ 1.1 s.** `traces.py` reads the capability map at 4 mm and runs the DP:
**814 ms** measured on a 1 000-line set, of which the DP, absorption and seam
placement are **10 ms**. Out come roughly **1 700 pieces** across the eight
stages, with per-stage and per-arm loads, every hand-over classified as overlap
or hard edge, and every piece carrying the full set of `(stage, arm)` pairs that
could draw it. (The measured figure is 1 661 on the superseded six-stage
pattern; the logo's correction cost four pieces out of fifty, so a few percent
is the expected shape of it.) The segmentation is embarrassingly parallel and
tunable by `--ds` if this ever matters.

**t ≈ 1.3 s.** Stage 1 opens. Arms 13, 71 and 2 take R0, R1 and R2. Each
arm's planner takes the first piece of its bucket: **194 ms** to plan the
stroke, **18 ms** for the two endpoint hovers, and **62 ms** for the park → hover
leg cold, or **3 ms** once the persistent cache exists.

**t ≈ 2 s — the first arm moves.** The measured time-to-first-motion budget is
**0.86 s at the median** from the moment the picture is in hand, and the
pessimistic p95 bound, which assumes every term has its bad day at once, is
**6.9 s**. Add the traces pass and the whole thing is comfortably inside Pete's
10 s. Two terms own that tail and both are known: a park → hover leg whose shape
ladder is exhausted and falls through to the RRT costs 4.2 s against 9 ms for a
direct move, which is exactly what item 1 removes; and a `plan_stroke` that has
to search the tilt cone costs seconds rather than 194 ms, which is the planner's
and is not on this pass.

**t = 2 s onward — planning runs behind the arms.** The three actives draw
asynchronously. Each is a static keep-out for the other two by **+85.8 mm**, so
there is no clock to share, nothing to re-conduct when one arm is late, and no
re-plan that can invalidate a neighbour. The planner stays ahead of the pens at
a comfortable margin: 1 000 strokes is **1 320 s on six workers** cold and about
**400 s** warm, against a draw of roughly **4 300 s**. `scene_check` runs per
window at **0.323 s per second of timeline**, one core keeping up with nearly
three times the headroom it needs.

**Barrier 1.** Each arm finishes its stage-1 bucket and retreats to its
stage-2 park. The barrier verifies pen-up, park identity, stopped, queue drained
and no fault on all six, then confirms the held park set is the one stage 2 was
certified against, and releases.

**Stage 2.** Arms 17, 31 and 97 take the same three row bands. The columns have
swapped; the geometry is identical by symmetry.

**Barrier 2, then stages 3–8 — the seams.** Six two-arm stages come back for
the 0.40 m dead bands, each pairing an outer arm on one seam with a middle arm
on the other, clearing between +194.3 and +250.0 mm. These are the only stages
that go through `idle.conduct`, and the cost is not v19's 407.3 s: the priority
search enumerates `∑ₖ P(n, k)` orders, which is **four** for two arms against
**720** for six, over a horizon that is a fraction of the programme. Six short
two-arm conducts are still far cheaper than one six-arm conduct of the whole.
Most of what the seams cost in *pieces* is an arm handing a line over to
**itself** one stage later — 301 of the 1 000-stroke set's 533 hand-overs —
because the line crossed out of its row band into a dead band that belongs to a
seam stage. That is a barrier and a re-approach, not a registration risk: the
same arm, the same calibration.

**The end, with the honest caveats.** Staging costs **24–33 % more pieces** than
an unstaged plan — 1 661 against 1 254 for this set, before the seam correction
— and the unstaged plan's makespan is one arm's load rather than 2.35× the
serial. But the unstaged plan **cannot run**: six arms drawing at once never
clears. Three numbers in this walk-through are arithmetic rather than
measurement and should be read as such. The ~4 300 s draw is
`docs/V2_SCALING_BASELINE.md` §3.3's scaling of v19's six-arm conducted
makespan. Nobody has run `sequence.py` over 1 700 pieces to get the staged
makespan directly. And the piece count itself is the six-stage figure nudged by
the shape of the logo's correction, not a re-run. The **block coverage**,
though, is no longer a caveat: the eight-stage pattern draws 100.0 % of the
certified block and leaves no orphan cells. What is still a blocker, and the
only one the choice of pattern cannot move, is the **park set at +4.7 mm**.

---

## 7. Provenance

| claim | measured in |
|---|---|
| per-stroke and per-leg costs, the 3 712.4 s attribution, the warm re-run, the streaming prediction, time to first motion, the four recovery gaps | `docs/V2_SCALING_BASELINE.md`; `scripts/profile_stroke_costs.py`, `scripts/job_substages.py` |
| elbow interpenetration, the separation frontier, the pattern comparison, per-stage parks, the barrier's preconditions, redundancy under a pattern | `docs/V2_WORKCELLS.md` §1–7; `scripts/workcell_envelopes.py`, `out/workcell_envelopes.json` |
| the eight-stage pattern, the seam-pairing matrix, the transverse pair being unseparable in y as well as x | `docs/V2_WORKCELLS.md` §4b (commit `07da40e`); `out/workcell_envelopes.json` key `seam_pairings` |
| the atoms, the min-pieces DP, seams and overdraw, absorption, load balance as a tie-break, the SEAM1 coverage hole that prompted §4b | `docs/V2_TRACES.md`; `aris_sixarm/traces.py`, `tests/test_traces.py` — **measured on the superseded six-stage pattern** |
| the execution adapter, `fr3drivers` as the target, the acceleration gap, the pen-swap repositions, the missing cross-process clock | `docs/HARDWARE_LADDER.md` |
| the decisions those three passes recorded | `docs/DECISIONS.md`, 2026-09-11 (three entries) and 2026-09-09 (the 50 mm gate) |

All three measurement passes were taken against the shipped atlas
`out/atlas_proposed_h0970_lat0860_gated63` at `ARIS_RIG=proposed
ARIS_TOOL=lateral`, h = 0.970, with `atlas.is_current` true for all six arms,
except the per-call cost table, which was taken against v19's own un-regated
sweep `out/atlas_proposed_h0970_lat0860`.
