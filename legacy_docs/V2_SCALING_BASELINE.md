# WHAT ONE STROKE COSTS, AND WHERE THE 3 712 SECONDS WENT

The next design goal is a fleet that tolerates faults, handles **1 000 strokes**
and **starts moving inside 10 seconds**. Today the CSAIL logo — 39 strokes,
16.805 m of ink — takes **3 712.4 s** from picture to certified programme and
does not move a joint until all of it is done.

This document is the baseline that goal has to be measured against. It answers
four questions with numbers:

1. what one stroke costs, end to end, measured rather than estimated;
2. where the 3 712.4 s actually went, split into per-stroke work, global
   optimisation, conduction and checks;
3. which of it depends only on `(rig, tool, atlas)` and could be bought once,
   offline, per rig;
4. what a stroke interrupted mid-way can currently do, and what it cannot.

**Provenance.** The run is GUI job `out/gui_jobs/20260910-124546-ce72`
(v19, `ARIS_RIG=proposed ARIS_TOOL=lateral`, h = 0.970, atlas
`out/atlas_proposed_h0970_lat0860`, placement `out/csail_place_v19_placement.json`).
Its stage and substage totals come from `scripts/job_substages.py` reading that
job's own `events.jsonl`; its narrative lines come from the same job's
`log.txt`. (There is no `out/h097_v19.log` — the v19 record is the job
directory.) The per-call table comes from `scripts/profile_stroke_costs.py`,
committed alongside this, run on the same machine against the same inputs.

---

## 1. THE PER-STROKE COST TABLE

`scripts/profile_stroke_costs.py` calls the same functions the allocator calls,
one at a time, in one process with no fork pool. A substage's wall clock is
therefore **not** the sum of these — the allocator runs probes and route screens
on six workers — but these are what a streaming architecture has to budget per
stroke. The script's own docstring carries the four commands that reproduce
every row below.

### 1.1 The three corrections that shape the table

**The per-stroke planner never touches the atlas.** `stroke_api.py`,
`planner.py`, `pwl.py`, `smooth.py` and `validate.py` do not import
`aris_sixarm.atlas`. The lattice is solved live by `ik.py`. The atlas enters
allocation in exactly one place — `allocate.prefilter`
(`aris_sixarm/allocate.py:3792`) — which is a numpy reach mask, and it is timed
separately below.

**The per-stroke planner does not memoise.** There is no `lru_cache` anywhere
in the package and nothing on the `plan_stroke` path writes a memo. Cold and
warm are the same number, measured. Every cache in this system is on the
**pen-up** side: `paper._CACHE` (routes), `paper._LIFTS` (hover poses),
`paper._LEGS` (leg measurements), `paper._SELF` (self-clearance),
`writing._HOVERS` (the hover ladder's answers), `sequence._PRICED` (priced
crossings). All six are process-global dicts, all six are dropped by
`paper.clear_cache()`, and none of them is ever written to disk.

**A stroke plan carries no hovers.** `plan_stroke` returns a pen-down
trajectory. The two endpoint hovers are derived later and on demand by
`allocate.PlacedIndex._hovers` (`allocate.py:1009`), which calls
`writing.lifted_or_lower` for `k in (0, -1)` and keeps the answer in memory
only.

### 1.2 The measured costs

All figures in **milliseconds**, one call each, single process, no fork pool.
`ARIS_RIG=proposed ARIS_TOOL=lateral`, h = 0.970, 46 mm pen tip,
`tilt_max_deg = 15`, `objective = maximin_sigma`.

| measurement | median | p95 | mean | n | what it is |
|---|---:|---:|---:|---:|---|
| `allocate.prefilter`, per (stroke, arm) pair | 1.10 | — | — | 234 | **the only place the atlas is read**: 258.3 ms for the whole 39 x 6 reach mask, load included |
| `plan_stroke`, whole stroke certified (`ok`) | 193.5 | 2 208.8 | 478.1 | 62 | the ladder DP end to end |
| `plan_stroke`, refused as `split` | 22.3 | 3 217.1 | 614.2 | 172 | the arm cannot hold the whole span; `probe_stroke` then walks the gaps |
| `plan_stroke`, second call, same input | 31.8 | 3 170.1 | 579.4 | 234 | **no memo exists** — cold and warm are the same number |
| &nbsp;&nbsp;of which `prepare` (resample, clip, lattice) | 54.4 | 229.9 | 72.0 | 62 | `planner.build_lattice`, solved live by `ik.py` |
| &nbsp;&nbsp;of which `plan_from_ctx` (band DP, certify, validate) | 68.5 | 178.9 | 78.6 | 52 | `pwl.plan_pwl` + `smooth.certify` + `validate.validate_plan` |
| &nbsp;&nbsp;whole call with the validator off | 140.1 | 2 118.2 | 388.7 | 62 | `opts['validate'] = False` |
| `writing.hover_solve` (one ladder rung) | 8.9 | 10.8 | 9.2 | 24 | not memoised on this path |
| `writing.lifted_or_lower` (the 5-rung ladder), cold | 9.0 | 10.5 | 9.2 | 24 | HOVER_LADDER = (0.06, 0.045, 0.03, 0.09, 0.12); 24/24 endpoints got a hover |
| `writing.lifted_or_lower`, `_HOVERS` memo hit | 0.0 | 0.0 | 0.0 | 24 | microseconds |
| `paper.route` park → hover, **COLD** | 61.8 | 4 152.7 | 1 150.6 | 12 | `paper.clear_cache()` first — the ladder walk |
| `paper.route` park → hover, `_CACHE` hit | 3.1 | 3.3 | 3.2 | 12 | floor arithmetic runs *before* the lookup, so a hit is not free |
| `paper.route` park → hover, legs warm (`cache=False`) | 3.5 | 1 296.6 | 326.7 | 12 | what a route costs when every leg in it is already certified |
| `paper.leg_bounds` park → hover (one leg, 33 samples) | 4.8 | 5.0 | 6.1 | 12 | the atom a hover roadmap would store |
| `paper.route` hover → hover, **COLD** | 120.3 | 1 665.1 | 478.7 | 4 | `paper.clear_cache()` first — the ladder walk |
| `paper.route` hover → hover, `_CACHE` hit | 3.3 | 3.4 | 3.3 | 4 | floor arithmetic runs *before* the lookup, so a hit is not free |
| `paper.route` hover → hover, legs warm (`cache=False`) | 4.1 | 5.4 | 4.3 | 4 | what a route costs when every leg in it is already certified |
| `paper.leg_bounds` hover → hover (one leg, 33 samples) | 5.0 | 21.4 | 9.0 | 4 | the atom a hover roadmap would store |
| `transit.plan` called directly | 11.8 | 1 066.1 | 362.5 | 3 | permissive gate, so a **lower bound**; inside a route the tier costs ~2.5 s |
| `free_cells`, broad phase rejects | 0.3 | 1.8 | 0.5 | 6 | 10 s of timeline, dt = 1/48, 4 arms, 230 400 cells/pair |
| `free_cells`, every capsule pair live | 4 995.5 | 5 080.4 | 4 658.0 | 6 | same, `cap = 2.0` — the worst case |
| `free_cells`, broad phase rejects | 8.2 | 9.5 | 8.2 | 15 | 30 s of timeline, 6 arms, 2 073 600 cells/pair |
| `free_cells`, every capsule pair live | 27 879.0 | 29 098.6 | 21 128.3 | 15 | same, `cap = 2.0`; min 196.8, max 29 906.9 — a 152x spread across the pairs |
| `scene_check.check_timeline` | 1 284 | — | — | 1 | 10 s of timeline, 4 arms / 6 pairs, sub = 2 -> **0.128 s of check per second of timeline** |
| `scene_check.check_timeline` | 1 871 | — | — | 1 | same 10 s, 6 arms / 15 pairs -> **0.187 s/s**: 2.5x the pairs for 1.46x the time |

### 1.2b The pen-up leg, by how far down the ladder it fell

`paper.route` returns `tried` — the number of ladder shapes it certified before
one worked. It is the single best explanatory variable for a leg's cost:

| mode returned | `tried` | cold ms |
|---|---:|---:|
| `direct` | 1 | 8.6 – 9.3 |
| `lift_exit@8cm` / `lift_entry@8cm` | 3 – 4 | 114 – 148 |
| `skirt1@8cm` | 2 | 102 |
| `traverse30@8cm` | 13 | 953 |
| `traverse30@25cm` | 51 | 1 665 |
| `rrt2` / `rrt3` (the whole ladder exhausted, then `transit.plan`) | 88 | 4 149 – 4 236 |

That is **≈ 32 ms per ladder shape certified**, and the tail is entirely the
RRT tier: the step from `tried = 51` to `tried = 88` costs about **2.5 s**.
`transit.plan`'s own budget is `ATTEMPTS x TIME_BUDGET + SHORTCUT_TIME` =
2 x 2.5 + 1.0 = **6.0 s worst case**, and a refusal spends all of it.

### 1.2c The pairwise conduct check and `scene_check`

`coordination.free_cells(pi, pj, margin)` is the pairwise question. It is
**O(Ni x Nj)** — quadratic in timeline length — and its cost swings by four
orders of magnitude on whether the whole-path broad phase rejects:

| window | arms / pairs | samples | cells per pair | broad phase rejects | every capsule pair live (`cap = 2.0`) |
|---|---|---:|---:|---:|---:|
| 10 s | 4 / 6 | 481 | 230 400 | **0.3 ms** (0.0013 µs/cell) | **4 995 ms** (21.7 µs/cell) |
| 30 s | 6 / 15 | 1 441 | 2 073 600 | **8.2 ms** (0.0040 µs/cell) | **27 879 ms** (13.4 µs/cell) |

at `dt = 1/48` s (24 fps x 2 substeps, as v19 ran). `CAPSULE_TILE = 32`,
`SWEEP_K = 0.55`, `PAIR_MARGIN = SAFETY_M + CALIB_M = 0.050` m.

**The cell count is exactly quadratic in window length** — 3x the window is
9.00x the cells — but the measured wall clock grew only 5.6x, and the reason
matters. Both windows sample *the same* synthetic motion, so the 30 s case is a
finer sampling of a fixed path: a 32-sample tile spans a third as much
workspace and the tile-local broad phase retires more of it. A real longer
timeline covers *more* motion rather than the same motion more finely, so it
will track the cell count more closely than this pair of rows does. **Treat 9x
as the scaling and 5.6x as the floor.**

The spread within one window is the other half of the story: at 30 s the
fifteen pairs run from **196.8 ms to 29 906.9 ms**, a 152x range, entirely set
by whether the two arms ever come near each other.

`scene_check.check_timeline` at `sub = 2` — **96 fine samples per second of
timeline** before auto-refinement:

| | s of check per s of timeline |
|---|---:|
| synthetic, 4 arms / 6 pairs, no refinement | **0.128** |
| synthetic, 6 arms / 15 pairs, no refinement | **0.187** |
| v19 phase 1, 6 arms / 15 pairs, real programme | **0.381** |
| v19 phase 2, 6 arms / 15 pairs, real programme | **0.274** |
| v19 both phases | **0.323** |

**`scene_check` is not pair-dominated.** 2.5x the pairs costs 1.46x the time,
so most of it is the per-arm work — self-collision, frame, neighbour base
columns, paper clearance, joint limits — which scales with arms and samples, not
with pairs. That is why it stays affordable where the conduct does not.

For comparison the conduct itself costs **1.96 s per second of timeline** on
the same run — six times what checking it costs.

### 1.3 How each scales with stroke length

Over all 234 (stroke, arm) pairs of the v19 logo — strokes from 0.055 m to
1.253 m, median 0.347 m — the 62 that certify whole:

| stroke length | n | median ms | p95 ms | ms per metre |
|---|---:|---:|---:|---:|
| < 0.2 m | 18 | 117.6 | 143.7 | 948 |
| 0.2 – 0.4 m | 23 | 186.2 | 1 188.8 | 652 |
| 0.4 – 0.7 m | 13 | 270.4 | 2 724.0 | 589 |
| > 0.7 m | 8 | 668.7 | 5 111.8 | 788 |

**The median is linear in length with a fixed floor.** The sample counts are
linear by construction — `n_lattice = 100 x L_m` at `ds_lattice = 1 cm`,
`n_dense = 200 x L_m` at `ds_dense = 5 mm` — and the medians track them at
**~600–950 ms per metre over a ~60 ms floor**. A least-squares fit over all 62
gives `ms = 1425 x L_m - 17`, but at `r = 0.36` that fit is describing the
outliers, not the trend: use the bucketed medians.

**The p95 is not a length effect at all.** It is the tilt rescue. A cell where
the perpendicular pen refuses and `tilt.plan_adaptive` has to search the cone
costs one to four seconds regardless of how long the stroke is — the worst
single call in the sweep is 15.5 s on a `split`, and the worst `ok` is 5.1 s on
a 0.49 m stroke whose neighbour at the same length took 0.28 s.

**Everything downstream of the stroke is length-independent.** A hover is a
pose, a leg is a pose pair: `lifted_or_lower` is 9 ms whatever the stroke, and
`paper.route`'s cost is set by `tried` (§1.2b), not by metres. The pen-up half
of the pipeline scales with the **number of strokes**, not with the ink.

**And the conduct scales with neither.** `free_cells` is `O(Ni x Nj)` in
timeline samples, so it is quadratic in the *duration* of a phase: measured,
3x the window is **9.00x the cells and 5.6x the wall clock** (§1.2c). That is
what makes conducting 1 000 strokes as one phase the wall it is (§3.3).

---

## 2. WHERE THE 3 712.4 SECONDS WENT

### 2.1 By stage

| stage | seconds | share | what it is |
|---|---:|---:|---|
| trace | 0.3 | 0.01 % | the picture to 39 polylines |
| **allocation, primary** | **3 132.3** | **84.4 %** | both inks, splitting on, cold |
| allocation, unsplit A/B | 79.4 | 2.1 % | the same ink re-allocated without cutting, **warm** |
| **conduction** | **407.3** | **11.0 %** | two `idle.conduct` calls |
| `scene_check` | 67.1 | 1.8 % | 2 phases |
| export | 2.4 | 0.1 % | |
| un-bracketed | 23.6 | 0.6 % | whatever no substage named |
| **total** | **3 712.4** | | |

### 2.2 Inside the primary allocation

| substage | seconds | share of job | kind |
|---|---:|---:|---|
| balance | 1 860.9 | 50.1 % | **global optimisation** |
| replan | 845.5 | 22.8 % | per-span, but mostly pen-up routing |
| probe | 232.2 | 6.3 % | **per-stroke** |
| flycheck | 154.1 | 4.2 % | global, per arm (tour feasibility) |
| merge | 16.9 | 0.5 % | coverage |
| sequence | 11.6 | 0.3 % | global, per arm |
| guarantee | 10.7 | 0.3 % | safety sweep |
| prefilter | 0.3 | 0.01 % | per-stroke (the one atlas read) |
| repair | 0.0 | — | |

### 2.3 The four buckets the design question asks for

| bucket | seconds | share | contents |
|---|---:|---:|---|
| (i) intrinsically per-stroke | **1 078.0** | **29.0 %** | prefilter + probe + replan |
| (ii) global optimisation | **1 968.8** | **53.0 %** | balance + split + merge + sequence + the unsplit A/B re-allocation |
| (iii) conduction / phase packing | **407.3** | **11.0 %** | `idle.conduct` |
| (iv) checks | **231.9** | **6.2 %** | flycheck + guarantee + `scene_check` |
| (v) trace, export, un-bracketed | 26.3 | 0.7 % | |

### 2.4 The measurement that settles it: the warm re-run

The v19 run allocated the same 39 strokes **twice** — once with splitting on
(the programme that ships) and once without, so the conductor could rule on
whether the cuts paid (`csail_schedule.py:1343`). The second allocation ran in
the same process, against caches the first one had already filled.

| substage | primary (cold) | unsplit A/B (warm) | ratio |
|---|---:|---:|---:|
| replan | 845.5 s | 13.3 s | **63.6x** |
| balance | 1 860.9 s | 24.8 s | 75.0x |
| flycheck | 154.1 s | 22.3 s | 6.9x |
| sequence | 11.6 s | 10.0 s | 1.2x |
| guarantee | 10.7 s | 9.0 s | 1.2x |
| merge | 16.9 s | 0.0 s | — |
| **same six substages** | **2 899.7 s** | **79.4 s** | **36.5x** |

The A/B pass does slightly less balancing (no cutting: 124 candidate bags
priced against 255), so 36.5x is an upper bound on the pure cache effect. But
`replan`'s 63.6x is not: it re-plans the **same spans on the same arms** and it
is 63.6 times cheaper the second time.

**So the dominant term in the 3 712 s is not the optimisation logic. It is
first-touch certification of pen-up geometry, and it lands in process-global
dictionaries that are thrown away when the process exits.**

### 2.5 Inside the conduct

From the job's own log lines:

| | seconds | share of conduct |
|---|---:|---:|
| collision images (65 built, 6 processes) | 85.0 | 20.9 % |
| priority search (2 380 DP solves, 4 searches) | 311.7 | 76.5 % |
| everything else | 10.6 | 2.6 % |

**131 ms per DP solve**, and the search enumerates `sum_k P(n, k)` orders —
1 165 solves over the 720 orders of 6 moving arms in one of the four searches.
`docs/FAST_PLANNING.md` §6 found the geometry hiding inside the search line;
after that pass was done the geometry is now 21 % and the search is genuinely
76 %. The search is combinatorial in the **arm count** and each solve is linear
in the **horizon**, so conduct scales as `O(P(n) x strokes)`.

---

## 3. THE STREAMING PREDICTION

A streaming design means: assign each stroke to the first arm that certifies it,
plan it, hand it to that arm, and plan the next N while the first ones execute.
No global balance, no makespan-driven re-plan passes, no split search, no A/B
re-allocation.

### 3.1 What it removes

| removed | seconds | share of 3 712.4 s | why it goes |
|---|---:|---:|---|
| balance + split | 1 860.9 | 50.1 % | pure min-max load optimisation over the whole stroke set |
| the unsplit A/B re-allocation | 79.4 | 2.1 % | exists only to decide whether cutting paid for itself |
| merge | 16.9 | 0.5 % | coverage tidying over the whole cover |
| sequence (Held–Karp tours) | 11.6 | 0.3 % | a stream draws in arrival order |
| most of flycheck | ~144 | ~3.9 % | the whole-bag tour feasibility test collapses to one `depot_round_trip` per assigned span |
| most of probe | ~172 | ~4.6 % | 262 probes over 234 (stroke, arm) pairs becomes ~1–2 arms tried per stroke |
| most of replan | ~598 | ~16.1 % | 157 clean re-plans for 46 shipped segments becomes ~46 |
| **total removed** | **~2 883** | **~77.7 %** | |

What is left is ~830 s: conduct 407.3 s, per-stroke planning ~310 s, checks
~78 s, trace/export/overhead 26 s. **None of it gates the first motion.**

### 3.2 Time to first motion

If the only things standing between the picture and the first joint moving are
one stroke's plan and its first pen-up leg, then from the table in §1:

| step | median | p95 |
|---|---:|---:|
| trace the picture (all 39 strokes) | 265 ms | — |
| load the atlas, prefilter the whole 39 x 6 mask | 258 ms | — |
| 2–3 arms refuse the stroke as `split`, at 22.3 ms each | 67 ms | — |
| `plan_stroke` on the arm that certifies it | 194 ms | 2 209 ms |
| two `lifted_or_lower` endpoint hovers | 18 ms | 21 ms |
| `paper.route` park -> hover, cold | 62 ms | 4 153 ms |
| **time to first motion** | **≈ 0.86 s** | **≤ 6.9 s** |

(The p95 row is the sum of the per-term p95s, which is a pessimistic bound on
the p95 of the sum: it assumes every term has its bad day at once.)

**The 10 s goal is reachable at today's per-call prices, with no planner change
at all.** The median is well under a second and even the pessimistic bound is
inside 10 s. What has to change is *what gates the start*, and today that is
the whole allocation.

Two terms own the tail, and both are known. A park -> hover leg whose shape
ladder is exhausted and falls through to the RRT costs 4.2 s against 9 ms for a
direct move — that is exactly what a precomputed hover roadmap (§4.2) removes.
And a `plan_stroke` that needs the tilt cone searched costs seconds rather than
the 194 ms median — that one is the planner's, and it is not on this pass.

### 3.3 Whether it keeps up at 1 000 strokes

Per-stroke planning in a greedy stream is ~7.9 s/stroke
(`(232.2 x 0.26 + 845.5 x 46/157 + 0.3) / 39`), all of it embarrassingly
parallel across strokes.

At the CSAIL median stroke of 0.347 m, 1 000 strokes is 347 m of ink — 20.7x
this picture — so the conducted makespan scales from 207.9 s to roughly
**4 300 s**.

| planner | wall clock for 1 000 strokes | keeps up with a 4 300 s draw? |
|---|---:|---|
| single process | 7 900 s | **no**, 1.8x too slow |
| 6 workers | 1 320 s | yes, 3.3x headroom |
| 6 workers + a warm leg roadmap | ~400 s | yes, 11x headroom |

**Conduct is the term that does not stream this way.** The priority search is
`sum_k P(n, k)` DP solves and each solve is linear in the horizon, so
conducting a 4 300 s timeline as one phase is ~20x the 407.3 s this run spent —
about 8 400 s, which is the whole drawing over again. A rolling window of the
next few strokes per arm keeps each solve short and takes it off the critical
path; conducting the whole programme up front does not scale and is the single
structural change v2 has to make.

---

## 4. WHAT IS PRECOMPUTABLE PER RIG

### 4.1 What the atlas already is

`out/atlas_proposed_h0970_lat0860` is six `.npz`, **1.9 MB total**, 4 125–4 498
rows each at a **2 cm grid**, 18 columns (`atlas.py:37`):

```
x, y, margin, sigma_min, f_max, n_sol, valid_frac, q7_window, tilt_deg,
q1..q7, min_lean_deg, flat_margin
```

**One certified DRAWING pose per cell, at z = 0, and nothing else.** No hover
poses. No z ≠ 0 poses. No routes. No legs. The per-cell fiber — 8 tool yaws x
16 q7 x every IK branch — is searched and then discarded; only `valid_frac` and
`q7_window` survive as scalar summaries of how big it was.

It depends on `(rig, tool, height, gate parameters)` and on **nothing the
artwork knows**: `atlas.sweep_arm` reads the spec's base pose, its pen and its
static obstacles, and no stroke, colour or placement enters it. A `model`
signature (810 entries) stamps the whole collision model plus `SEARCH_POLICY`
and `GATE_CONE_DEG` into every file, and `atlas.is_current` refuses a stale one
— which is what makes it safe to treat as a certificate rather than as data.

Build cost: the six-arm sweep at one height is **1–5 minutes** on this box
(comparable recorded runs: 308 s for the mirrored sweep, 48 s per arm for the
cylinder-frozen one).

### 4.2 What a hover roadmap would add

A hover roadmap is the first **motion** artifact this project would persist.
There is no persisted leg cache on disk today — `paper._LEGS`, `paper._LIFTS`,
`paper._CACHE`, `writing._HOVERS` and `sequence._PRICED` are module-global
dicts, and there is no serialiser. The closest thing on disk is
`out/fw_h0970_final_map.npz` (7.1 kB), which stores a hover *height* and a
reachability *code* per cell — never a hover pose and never a leg.

What it would hold, and what it costs at the per-call prices in §1:

| layer | what | size | build cost |
|---|---|---:|---:|
| **nodes: hovers** | one hover pose per strict-GO atlas cell per arm, from `writing.lifted_or_lower` | 6 arms x ~3 900 GO cells x 7 float32 = **655 kB** | ~9 ms/cell x 23 500 = **212 s serial, ~35 s on 6 workers** |
| **edges: park legs** | `park -> hover` and `hover -> park` for every node | 2 x 23 500 route results | at the measured mean of 1 151 ms cold: **13.5 h serial, 2.3 h on 6 workers** |
| **edges: hover legs** | `hover -> hover` within one arm | the full graph is 3 900² = 15.2 M per arm — **not buildable** | — |

The hover-to-hover layer is the one that does not close. It has to be a sparse
roadmap, not a complete graph: k-nearest-in-xy neighbours plus the park node as
a hub. At **k = 8** it is 6 x 3 900 x 8 = **187 000 edges**, at the measured
cold mean of 479 ms that is **25 h serial / 4.2 h on six workers**, and the
stored graph is 187 000 x (2 node ids + up to 6 vias x 7 floats) ≈ **32 MB**.

That is an overnight build, per rig-and-tool, and it is bought once. Against it:

- the measured route cost falls from a **1 151 ms mean / 4 153 ms p95** to the
  **3.1 ms** `_CACHE`-hit price, which is floor arithmetic and a dict lookup;
- `replan`'s 63.6x warm speedup (§2.4) stops being an accident of running two
  allocations in one process and becomes the normal case;
- the p95 tail on time-to-first-motion disappears with it (§3.2);
- and a stroke interrupted at an interior point gets a certified way out
  (§5.1) — which is the fault-tolerance half of the same artifact.

**Two things it must carry to be a certificate rather than data.** The atlas's
`model` signature (`atlas.py:86`, 810 entries, checked by `atlas.is_current`)
stamps the whole collision model plus `SEARCH_POLICY` and `GATE_CONE_DEG`; a
roadmap needs the same, plus `paper.FRAME_FLOOR`, `STATIC_SAFE`, `SELF_SAFE`
and the RRT flag, because those are in `paper._CACHE`'s key for a reason. And it
must record the gate floor it was certified at: the shipped atlas was swept at
`rig_final.STATIC_MARGIN` = 50 mm while the router flies at `FRAME_FLOOR` =
63 mm, so the shipped atlas is an **optimistic prefilter by up to 13 mm**
(the 63 mm variant is `out/atlas_proposed_h0970_lat0860_gated63`).

### 4.3 What stays artwork-dependent

- `plan_stroke` itself. The ink is the artwork; there is nothing to precompute
  beyond the per-cell drawing pose the atlas already has, and the planner does
  not even read that.
- `paper._CACHE` (the *route*, as opposed to the *leg*). A route's key carries
  `tip_floor`, `chain_floor`, `q_home`, `STATIC_SAFE`, `SELF_SAFE` and the RRT
  flag — all clamped per pair by `effective_floors` — so the same pose pair
  genuinely has different answers under different questions. The **legs** are
  portable; the **routes** assembled out of them are not.
- Everything in `sequence`, `allocate.balance` and `idle.conduct`: these are
  functions of which strokes went to which arm, which is the artwork.

---

## 5. FAULT-RECOVERY PRIMITIVES: WHAT EXISTS, WHAT DOES NOT

### 5.1 A stroke interrupted mid-way

**There is no certified path off an interior point of a stroke.** The ascent leg
is certified for the stroke's *end* only:
`allocate.PlacedIndex._hovers` (`allocate.py:1009`) computes
`writing.lifted_or_lower(spec, qs[k], pts[k])` for `k in (0, -1)` and for no
other `k`. An interior sample `qs[j]` has a certified *drawing* pose at z = 0
and nothing above it.

Getting off an interior point at runtime would cost, at today's prices, one
`lifted_or_lower` plus one `paper.route` from a cold cache — which is exactly
the pair measured in §1, and is the reason a persisted hover roadmap is the
recovery story as much as it is the speed story.

### 5.2 What the typed program carries

Neither program representation carries hovers.

| | arm assignment | drawing poses | hovers | re-queueable? |
|---|---|---|---|---|
| `program_schema.Bundle` (the viewer schema, `SCHEMA_VERSION = 1`) | yes, `Segment.arm` | **no joints at all** | no | no |
| `out/<stem>_program.json` | yes, keyed by arm | `q_first`, `q_last` | no | only by re-planning |

`Segment` (`program_schema.py:203`) carries `arm, index, phase, stroke_id,
s_range, direction, flipped, pts` — `pts` is the paper xy of the span, not
joints. `export_bundle` (`program_schema.py:442`) reads the planner's richer
segment dict and **drops `q_first`/`q_last`**; they do not survive into the
bundle. `_from_dict` refuses unknown *and* missing keys, so adding hovers is a
schema-version bump, not a silent extension.

Re-queueing a stroke to another arm is possible from the program JSON, because
it carries `pts` + `s_range` + `direction` — which is exactly what
`allocate.replan_same_span` (`allocate.py:1022`) wants. But that is a *planning*
call, not a lookup: it runs the full stroke planner, and the result then has to
be re-conducted and re-`scene_check`ed, because the new arm's timeline is not
certified against the other five.

### 5.3 What the runtime keeps, and what it would have to become

`aris_sixarm/execute/` has exactly three moving parts:

- **`Governor`** (`execute/trajectory.py:203`) — **one scalar clock rate for the
  whole fleet**, and that is the entire safety argument: `scene_check` certified
  six arms against each other on a shared clock, so scaling that clock keeps
  every pair at configurations that were evaluated. There is deliberately no
  per-arm entry point anywhere in the package. `hold()` ramps at 2 rad/s² and
  reports `stop_distance_s`; `resume(rate)` ramps back; `seek(t)` exists.
- **`Barrier`** (`execute/program.py:56`) — kinds `("start", "pen_swap",
  "phase_end", "end")`. `Barrier.mismatch(measured, tol=0.02)` is **the only
  place in the whole stack where the real robot's position is checked against
  the plan**. `reposition()` names the arms whose `resume_q` differs from
  `hold_q`, i.e. uncertified motion across the pause.
- **`runner.play`** (`execute/runner.py:72`) — advance the governor, sample
  every arm at the same program time, `send` the whole fleet. Open loop
  everywhere except a barrier, by design.

State that exists, in memory, per run: the governor's `rate`/`target`/
`t_program`, `tracks = {arm: whole-programme trajectory}`, the barrier queue,
and a `RunLog` that is returned only at the end and written by nobody.

**State that does not exist anywhere:** which *segment* is in flight, which
strokes are done, pen-down vs pen-up, or any mapping from program time back to
a stroke. The `.npz` carries `seg_<arm>` per frame and `segoff_<arm>` per
segment — the viewer reads them; `FleetProgram` and `runner` never do.

### 5.4 Abort, retreat, resume

| primitive | exists? | what it actually is |
|---|---|---|
| e-stop | **no, not in this repo** | `docs/HARDWARE_LADDER.md:51` is explicit: the physical e-stop is the abort path; the gate's 2 rad/s² brake and latching watchdog are not. Both the gate and the GUI's `emergency_stop()` live outside this repo. |
| reflex / collision monitor | **no** | the only `reflex` hits describe the robot's own reflex inside the SIL model |
| abort | partial | `Backend.stop(reason)` is a refusal-to-continue; `Fr3BundleBackend.stop` raises `NotImplementedError` |
| retreat | yes, but it is a **planning** concept | `idle.plan_retreat` schedules a pose into the timeline ahead of time so an idle arm gets out of a neighbour's way. It never runs against measured state. |
| resume | clock only | `Governor.resume()` ramps the rate. There is no resume-from-mid-programme: `play` always starts at the start barrier. `FleetProgram.barrier_before(t_s)` — exactly the lookup a resume needs — exists and **has no caller**. |
| re-queue to another arm | no runtime primitive | `replan_same_span` is a planning call (§5.2) |

The failure model, pinned by `tests/test_execute.py`, is entirely **stop and
refuse**: no state machine, no retry, no degraded mode.

### 5.5 The four gaps, named

1. **No interior ascent.** Only the two endpoints of a span have a certified
   hover. A reflex stop at s = 0.4 has nowhere certified to go.
2. **No hovers in the schema.** Neither the bundle nor the program JSON carries
   the start/end hover pose, so re-queueing means re-planning, and re-planning
   means re-conducting.
3. **No runtime segment state.** Nothing in `execute/` knows which stroke is in
   flight. `seg_<arm>` is in the `.npz` and unread.
4. **No per-arm clock.** The `Governor` is fleet-wide on purpose, so "arm 71
   faulted, let the other five carry on" is not expressible without a new
   certificate.

---

## 6. WHAT THIS SAYS ABOUT V2

Six findings, in the order they change the design.

**1. 77.7 % of the run is optimisation a stream does not need.** Balance, split,
merge, the Held–Karp tours and the unsplit A/B re-allocation are 1 968.8 s —
53.0 % on their own — and the speculative planning the balancer buys to feed
them accounts for most of the rest. A greedy assignment keeps the certificates
and drops the optimisation.

**2. The dominant cost is first-touch pen-up geometry, and it is thrown away.**
The same six substages cost 2 899.7 s cold and 79.4 s warm in the same process
(§2.4); `replan` alone is 63.6x cheaper the second time on the same spans. Not
one byte of that is persisted. A per-rig hover roadmap (§4.2) is an overnight
build that turns the 1 151 ms mean route into a 3.1 ms lookup.

**3. Time to first motion is already under a second.** Plan one stroke, solve
two hovers, certify one park -> hover leg: **≈ 0.86 s at the median**, and even
the pessimistic per-term p95 bound is 6.9 s. The 10 s target is not a
planner-speed problem. It is a question of what gates the start, and today the
answer is "everything".

**4. Conduct is the term that does not stream.** 407.3 s, of which 76.5 % is
2 380 priority-DP solves at 131 ms each. The search is `sum_k P(n, k)` in the
arm count and each solve is linear in the horizon, so a 20x longer timeline is a
20x more expensive conduct — about 8 400 s at 1 000 strokes, longer than the
drawing itself. **Conducting a rolling window instead of a whole phase is the
one structural change v2 cannot avoid.**

**5. `scene_check` is affordable and should stay unconditional.** 0.128 s per
second of timeline at 4 arms / 6 pairs and 0.187 s/s at 6 arms / 15 pairs
synthetically — 2.5x the pairs for 1.46x the time, so it is **not**
pair-dominated and does not blow up the way `free_cells` does — against
0.323 s/s on the real v19 programme. Even at 1 000 strokes that is ~1 600 s,
and it is the final inter-arm veto. Run it per window, not per phase, but run
it.

**6. The fault-recovery primitives do not exist yet, and two of the four gaps
are cheap.** Carrying the endpoint hovers in the program schema is a
schema-version bump. Reading `seg_<arm>` — already in the `.npz` — into the
runtime would give the executor a notion of which stroke is in flight. The two
expensive ones are an interior ascent for every sample of every stroke (which
the hover roadmap gives, if hovers are keyed by cell rather than by span
endpoint) and a per-arm clock, which needs a certificate the current fleet-wide
`Governor` argument does not provide.
