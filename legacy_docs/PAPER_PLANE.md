# THE PAPER WAS NEVER AN OBSTACLE, AND THE ARMS WENT THROUGH IT

Two user-reported defects in the shipped `csail_final6` release, both real,
both now fixed and gated. This is the record of what was wrong, how far wrong,
and what the fix cost.

## 1. The defect: arms penetrate the image plane during transits

### 1.1 It is exactly the hypothesised mechanism

Everything that TOUCHES the paper was certified against it and nothing that
FLIES OVER it was:

- `stroke_api` certifies a stroke and `validate.validate_plan` re-derives its
  chain clearance (`Z_CLEAR`, 2 cm) sample by sample;
- `validate.check_pose` re-derives the same gate for a pose an arm stands in;
- a **pen-up transit is neither**. `writing.arm_program` laid it down as a
  straight line in joint space between two hover poses, both certified, and
  nothing ever looked at what the line did in between.

`scene_check.check_timeline` checked inter-arm clearance, frame boxes, per-segment
re-validation, monotone progress and playback — but not the paper. The one place
the paper appeared was `scene_check.PEN_PAPER`, a **deliberately ungated warning**
about a static pose, whose comment says it was left ungated so as not to
retroactively refuse a release. Between poses, nothing.

### 1.2 Inventory on the shipped timeline

Measured on `out/csail_schedule_final6.npz` as it shipped, densely sampled
(16 sub-samples per 12 fps frame) with the same FK the animation replays.
Two classes, and only one is a defect:

| class | blocks | worst tip | worst chain | verdict |
|---|---:|---:|---:|---|
| ink contact | 71 | −1.2 mm | +108.8 mm | **benign** — the pen is ON the paper while drawing; the dip is interpolation between frames, inside `validate.TIP_TOL` (2 mm) |
| **pen-up penetration** | **3** | **−253.6 mm** | **−156.1 mm** | **defect** |

The three offending blocks — **1 inter-segment transit and 2 go-home transits**:

| arm | block | t (s) | min pen tip | min chain point |
|---|---|---|---:|---:|
| 2 | transit, seg→seg | 27.08 – 30.08 | **−234.0 mm** @ 29.45 | **−156.1 mm** @ 29.43 |
| 2 | go-home | 37.08 – 50.08 | **−175.7 mm** @ 37.73 | −67.4 mm @ 37.73 |
| 97 | go-home | 48.67 – 50.08 | **−253.6 mm** @ 49.33 | −151.8 mm @ 49.33 |

39 of the 42 moving pen-up blocks clear 20 mm comfortably. **The failure is
surgical, and so is the fix.**

**The worst of it is not the depth, it is the dwell.** Arm 2's transit lifts to
+152 mm at t = 27.54, crosses z = 0 at t ≈ 27.94, bottoms at −206.8 mm — and
then **holds there, motionless, from t = 28.42 to t = 29.17**. That is a
conductor-inserted pause, scheduled while the arm was parked 207 mm underneath
the canvas, because nothing in the collision image knew the canvas was there.

**Why these three and not the other 39.** All three are long reconfigurations
whose two hover poses sit on **different IK branches** — arm 2 swings joint 1
from +0.85 to −1.83 rad and joint 7 half a turn. A straight line in joint space
between two branches is not a motion anybody chose; it is whatever the
interpolation traces, and it dives. Short entry/exit corridors are safe because
they are short and vertical, exactly as the report hypothesised.

## 2. The fix, at two layers

### 2.1 Construction — `aris_sixarm/paper.py`

The paper joins the frame boxes as a thing a pen-up move is certified against.

**Floors.** `CHAIN_CLEAR` = 2 cm, imported from `validate.Z_CLEAR` rather than
restated — the number every certified stroke already keeps. `TIP_CLEAR` = 2 cm
for the pen tip *while flying*, never demanded above the hover the arm actually
reached (`lifted_or_lower` gives up 60 mm for 45, 30, or none near the edge of
reach). `TIP_TOL` = 10 mm is the contact band, an order of magnitude above the
certified ink band and its resampling residual, and an order of magnitude below
the smallest real violation.

**`paper.route`** certifies the straight line and, when it is refused, returns
**via-configurations** — each a `writing.lifted_config` IK solution subject to
the same joint-limit margin every other hover keeps, so a via is a pose the arm
may legitimately stand in. Shapes are tried cheapest-first up a height ladder
(8 → 40 cm): direct, lift one end, lift both, arc over the midpoint, and the
one that earns its keep, the **Cartesian walk** — hovers every 20-30 cm along
the xy line, each solved nearest to the one before it, so the walk stays on one
analytic branch instead of jumping between two.

> **The subtlety that decides whether this works at all.** The walk must solve
> its OWN last hover by continuing. An earlier version handed the final hover
> to a pose solved near the target, which put it on the target's branch and
> handed the walk exactly the branch flip it was inserted to avoid. On arm 2's
> go-home — 0.94 m back across the mirror plane — that version found no route
> at any height; owning the last hover clears the paper by 20-60 mm.

**A via must not trade a paper hit for a frame hit.** The way out of the paper
is up, and up is where the top rails and corner posts are. An inserted route is
therefore also certified against `rig_final.chain_static_clearance` at
`STATIC_MARGIN`. This is not hypothetical: the first routed conduct was refused
at **49.9 mm of frame clearance against a 50 mm margin** with the paper gate
passing comfortably.

**Refusal is an answer.** When no shape certifies, `route` returns `None`, and
that propagates honestly:

- `sequence.cost_matrix` prices the crossing as **`inf`**, so the tour never
  proposes it — an unflyable transit leaves the search space instead of
  becoming an exception later. `_paper_surcharge` charges every routable-but-
  detoured crossing its real extra seconds, and both the plain and the fiber-menu
  (`--cluster`) matrices share one definition of it.
- `writing.arm_program` raises `PaperRefused` rather than lay a path it cannot
  certify. The pipeline degrades per profile rather than shipping.

**The cost model stays honest.** The pipeline's own cross-check reports
`sequencer cost model vs frozen timeline: worst disagreement 1.78e-15 s over
6 arms` — the routed transit costs the sequencer minimised are the seconds the
timeline pays, to float precision. With no vias, `_beat` is bit-identical to the
old `_dq_time` arithmetic, which is what keeps every pinned number in the corpus
reproducible on the 39 blocks that never needed fixing.

### 2.2 Ship gate — `scene_check`

`check_timeline` now measures **every arm's tip and chain against the paper at
every instant** and it is a **hard** gate (`PAPER_CHAIN` 20 mm, `PAPER_TIP`
−10 mm). The chain gate does the precise work; the tip gate is a second net.

The verdict is made **independent of the sampling rate**. The sweep residual is
a 1-Lipschitz bound on 3D displacement, which over-charges a tip *sliding along*
the paper — at 12 fps that alone reads 14.7 mm of false dip. The check
auto-refines until per-point motion is under `PAPER_STEP` (5 mm), after which the
numbers stop moving:

| `sub` | 1 | 2 | 4 | 8 |
|---|---|---|---|---|
| failed arms | [2, 97] | [2, 97] | [2, 97] | [2, 97] |
| clean-arm tip | −0.9 mm | −0.9 mm | −0.9 mm | −0.9 mm |
| offender chain | −157.0 mm | −156.9 mm | −156.9 mm | −156.9 mm |

**The shipped timeline FAILS the new check and a routed one PASSES** — pinned by
`tests/test_paper.py` against a checked-in copy of the pre-fix timeline
(`tests/data/csail_final6_prefix_timeline.npz`).

### 2.3 The re-conduct: what it costs, and what it buys

Same recipe, same budgets, same placement, same allocation:

```
ARIS_RIG=final6_opt python3 scripts/csail_schedule.py --arms all --max-probes 5 \
    --rotate 90 --target-width 0.8417 --offset 0.0 0.0 \
    --atlas out/atlas_final6_opt --tag _final6 \
    --draw-speed 0.15 --transit-speed 0.30 --fps 12 --substeps 4 \
    --final out/csail_final6_final.png --select-profile --program
```

| | shipped (pre-fix) | **re-conducted (routed)** |
|---|---|---|
| profile shipped | qd0.60 | **qd0.30+cluster** |
| **makespan** | **50.125 s** | **60.771 s  (+10.646 s, +21.2 %)** |
| frames @ 12 fps | 602 | 730 |
| coverage | 86.9704 % | **86.9704 % (unchanged)** |
| drawn / traced | 8.5488 / 9.8295 m | **identical** |
| certified segments | 38 | 38 |
| min inter-arm clearance | 81.8 mm | **82.9 mm** (margin 80) |
| min frame clearance | 65.6 mm | **65.6 mm** (margin 50) |
| paper: min chain | **−156.1 mm** | **+108.5 mm** (margin 20) |
| paper: min pen tip | **−253.6 mm** | **−1.5 mm** (floor −10) |
| `scene_check` | PASS (paper unmodelled) | **PASS incl. the paper gate** |
| animation | 29.5 MiB / 16.2 MiB zip | 30.5 MiB / **16.4 MiB zip** (budget 28) |
| tip vs drake FK | 0.169 mm | **0.175 mm** |

**Zero penetrations, by construction and by measurement.** Re-probing the
shipped timeline block by block: **0 of 42** moving pen-up blocks break the
paper (was 3). The worst pen tip anywhere in the run is **−0.5 mm** — ink
contact, well inside the certified 2 mm band — against −253.6 mm before, and the
worst chain point is **+109.5 mm** against −156.1 mm. Every arm's pen-up tip
minimum is at or above −0.4 mm.

**The cost is honest and it is not all the paper's.** The profile grid moved:
`qd0.60+cluster` (the cheapest floor) is now REFUSED on frame clearance
(42.3 mm) while passing the paper gate comfortably, and `qd0.60` certifies at
69.604 s, so the search settles on `qd0.30+cluster` at 60.771 s. Two things are
mixed into the +21.2 %: the via-configurations themselves, and the fact that a
different, paper-legal tour is a different scheduling problem for the conductor
(pauses went 37.6 s → 50.1 s). The coverage, the segment count and the ink are
bit-identical, so none of the cost was paid in drawing.

## 3. Dead zones, and where the 13.03 % actually goes

### 3.1 The atlas overstates death by 7.04 pp

`atlas.strict_go` gates at margin ≥ 0.30 and σ_min ≥ 0.14 — IKA numbers for a
cell you would like to stand in and work from. The planner that actually
certifies a stroke gates at `validate.MARGIN_GATE` = 0.15 and `SIGMA_GATE` =
0.10. Both tiers come out of the same atlas columns, so they nest by
construction.

| tier | cells | % of canvas |
|---|---:|---:|
| strict-GO (published) | 12 575 | 75.93 % |
| permissive-GO (planner gates) | 13 741 | 82.97 % |
| **atlas-strict dead (published)** | **3 987** | **24.07 %** |
| **permissive-dead — truly dead** | **2 821** | **17.03 %** |
| **strict-only-dead — overstatement** | **1 166** | **7.04 %** |

**Nearly a third of the canvas the rig calls dead is paper the planner can
reach.** Probe-verified: of 120 permissive-dead cells sampled deterministically
(every 7th, 50 mm test stroke), **4 were certified by the real planner** — so
even the red tier still overstates death by ≈ 3 %, which the overlay says out
loud rather than hiding.

### 3.2 The 86.97 %, attributed

1.2807 m of 9.8295 m traced was never drawn. Every span densified at 5 mm and
classified into four disjoint, exhaustive causes, then **re-planned end to end
by the real planner**:

| cause | metres | share |
|---|---:|---:|
| dead cell at the shipped placement | **0.0000 m** | **0.0 %** |
| conductor refusal | **0.0000 m** | **0.0 %** (structural — see below) |
| reachable, but by no arm holding that ink | 0.7897 m | 61.7 % |
| right-ink arm has the cells, the stroke still refuses | 0.4911 m | 38.3 % |

**Not one undrawn millimetre is a dead cell.** That is not luck — the placement
was chosen for 98.9 % predicted single-pass coverage, i.e. deliberately sited on
live paper. The loss is entirely ink partition and planner band.

**"Other" is not a shrug.** All 11 leftover spans were handed back to
`stroke_api.plan_stroke` for every arm holding the right colour. **None
certifies** — 0.0000 m recoverable. The planner's own reasons:

| planner verdict | spans | metres |
|---|---:|---:|
| `split/start_infeasible` — the span's own start point has no certified configuration | 9 | 1.2415 |
| `split/empty_fiber` — no surviving q7 fiber | (same spans, other arms) | — |
| `degenerate/too_short` — under the 20 mm minimum stroke length | 2 | 0.0192 |

So the honest statement is: **a cell being GO does not make a stroke through it
certifiable.** The planner needs a continuous certified band from end to end,
and these spans have an infeasible endpoint.

**Why conductor refusal is a structural zero.** A refusal never reaches the
dropped list. `allocate.leftover` derives the dropped spans as the *complement
of the shipped programmes*, and `idle.Unconductable` makes `csail_schedule`
re-sequence, fall back to the go-home policy, or abandon the whole profile — the
allocation is never edited. A refused profile contributes no dropped spans at
all; it appears only as a `reason` string in the profile grid.

### 3.3 What the 0.20 m anti-corner-post shift cost

`docs/MERGED_CANVAS.md` §3.3: the centred placement was vetoed by `scene_check`
at **40.9 mm of frame clearance against a 50 mm margin** — an inverted arm's
pen-up transit clipping the doubled `post_BL@A`/`post_BL@B` column at the left
end of the seam. Moving the logo 0.20 m right took frame clearance to 56.1 mm
and the run certified.

**In coverage the shift cost nothing measurable.** Both placements are the same
0.842 × 1.102 m logo at the same scale; the vetoed one was never conducted, so
it has no *drawn* coverage to compare. What can be said precisely is that the
shift moved the logo off the only structure that vetoed it, and that the loss it
is blamed for is not a reach loss: **0.0 % of the undrawn ink lies in a dead
cell** at the shipped placement, so no part of the 13.03 % is attributable to
the shift having pushed the logo into unreachable paper.

## 4. The overlay

`ARIS_RIG=final6_opt python3 scripts/make_final6_dead_view.py`

- `out/final6_dead_view.html` — the six-arm scene with two new toggleable
  layers: `dead_zones/permissive_dead` (solid dark red) and
  `dead_zones/strict_only_dead` (orange), plus `undrawn_ink/spans` laid on the
  paper where the dropped ink would have gone.
- `out/final6_dead_zones.npz` — the dense masks, so the viewer and the tests
  share one artefact.
- `out/final6_undrawn_attribution.json` — per-span causes, fractions, planner
  verdicts and the probe results.

Generation is deterministic (sorted arms, whole-array ops, fixed-stride
probing); a test pins bit-identical masks across runs.
