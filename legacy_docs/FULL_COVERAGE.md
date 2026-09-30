# THE LAST 59.9 MILLIMETRES, AND THE FIX THAT WAS NOT THE ONE EXPECTED

*2026-08-24. Code: `allocate.merge_remainders`, `validate.validate_plan`'s cone
gate, `stroke_api`'s gate plumbing. Tests: `tests/test_merge_spans.py`,
`tests/test_validate_cone.py`.*

`docs/TWO_PASS.md` ships **99.3907 %** of the CSAIL logo and names the two spans
it does not draw:

| stroke | span | metres | the reason on record |
|---|---|---:|---|
| 26 | s[0.6213, 0.7571] | 0.0497 | arm 2 certifies to 0.6213, arm 71 from 0.7571; **arm 97 can ink the gap and cannot fly to it** |
| 30 | s[0.6488, 0.7012] | 0.0102 | 10.2 mm, under the 20 mm minimum — `degenerate/too_short` |

Both are now drawn. Coverage is **100.0000 %**, and the interesting part is
that the fix is the same fix for both, it is four lines of policy rather than
new geometry, and it is **not** the pen tilt everybody expected to need.

---

## 1. What was actually wrong: a rule about pen-ups, applied to ink

Three separate places in the allocator refuse a short span, and all three are
right to:

* `stroke_api.plan_stroke` calls anything under `min_length` = 20 mm
  **degenerate** — it is not a curve you can build a lattice on;
* `allocate.MIN_SEG_M` = 25 mm is the shortest span `greedy_cover` will place,
  because a segment costs a pen-up, a hover, a transit and a visible seam;
* `repair_gaps` will not offer a window narrower than either.

Every one of those is a statement about **a segment of one's own**. None of
them is a statement about ink, and that is the confusion that left 59.9 mm of
the logo blank. Stroke 30's middle 10.2 mm is not worth a pen-up — true, and
irrelevant, because the arm holding the 127 mm below it does not need a pen-up
to draw it. It needs to not stop.

So the missing move is: **ask an arm to draw a span and the ink lying against it
as ONE segment.** `allocate.merge_remainders` is that move, in two passes over
the finished programme:

  **(a) absorb** a hole no arm would take into the segment whose endpoint it
  touches;
  **(b) coalesce** two segments of the same arm on the same stroke that touch,
  into one.

It runs **last** — after the cover, the repair pass, the balancer and the
splitter have all had first refusal — so a remainder that reaches it is one
nobody would place, and absorbing it cannot take paper away from a better
placement. The merged span goes back through `replan_same_span`, so what ships
is **one certified plan over the whole span**, with the same gates, the same
5 mm chase and the same independent certificate as any other segment. Nothing
is spliced and no micro-segment is created: the 25 mm floor still governs every
standalone piece, which is the question it was written to answer.

Two things had to be got right for it to be an improvement rather than a trade:

**A longer segment is a different segment to fly to.** The endpoint moves, so
the hover moves, so the tour `prune_unflyable` proved flyable is not
automatically the tour the arm now has. Every merge is re-checked and rolled
back if the arm loses its tour.

**Order matters, so it is fixed.** Largest hole first (a merge changes what its
neighbour could have merged with), and the two passes run to a fixed point,
because absorbing a hole can leave the grown segment touching the next one
along.

### What it recovers on the logo

```
merge: arm 97 extends stroke 26 [0.4055,0.6213] -> [0.4055,0.7571], +49.7 mm of ink, no extra pen-up
merge: arm 31 extends stroke 30 [0.7012,1.0000] -> [0.6488,1.0000], +10.2 mm of ink, no extra pen-up
```

**Both spans, flat, with the pen perpendicular, and not one extra segment.**

The stroke-26 line is worth reading twice. Arm 97 is the arm `docs/TWO_PASS.md`
§1.2 measured as *unable to fly to that span* — and it is, as a **standalone**
segment: every crossing into its hover dives 137–189 mm of chain below the
canvas and `paper.route` refuses all of them. But arm 97 is already drawing
s[0.4055, 0.6213] of that same stroke, and it can fly to *that*. Extending it
needs no crossing at all. The unflyability was never a property of the paper;
it was a property of **arriving there with the pen up**, and the merge does not
arrive — it continues.

---

## 2. The pen tilt: it works, it is integrated, and it is not what ships

The obvious reading of stroke 26 was that its two neighbours were fold-limited
rather than reach-limited, which is exactly what `docs/TILT_EXPLORATION.md`
measured a 15° cone unfolding. That reading is correct, and it was tested
first:

| span | arm | flat | tilt ≤ 15° |
|---|---|---|---|
| 26, s[0, 0.7571] (arm 2's segment extended through the hole) | 2 | **split** at 0.6213 | **ok**, margin 0.181, σ 0.146, lean 15.0° |
| 26, s[0.6213, 1] (arm 71's, extended down) | 71 | **split** | **ok**, margin 0.337, σ 0.149, lean 15.0° |
| 26, s[0.6213, 0.7571] alone, both directions | 2, 71 | **split**, s\* = 0 | **ok**, margin 0.176 / 0.231 |
| 30, whole stroke | 71 | split at 0.6488 | **ok**, lean 15.0° |

So the tilt rescue closes both spans on its own, and with the merge on top it
produces a **better allocation than the flat one — 41 certified segments
against 45**, because arm 2 can then hold stroke 26 from 0 to 0.7571 as a
single piece. `tests/test_merge_spans.py` pins all of it.

**It still is not what ships, and the reason is measured rather than
aesthetic.** Opening the cone changes which arms certify which strokes, and
that changes the load balance — including in the **grey** phase, which was
already at 100 % coverage and had nothing to gain. In the tilted allocation
arm 31 is handed a leaning grey stroke, and phase 1 is then refused:

```
!! phase 1: grey could not be conducted as allocated:
   arm 31: go-home at segment 0 cannot clear the paper plane
```

— in the split allocation and in the unsplit fallback, with hovers vertical and
with hovers leaning. **A rescue that is opened where nothing needed rescuing
costs a conduct.** `docs/TILT_EXPLORATION.md`'s own recommendation says this in
the small — "do not open it on strokes that already certify" — and the logo
says it again in the large: do not open it on a **phase** that already draws
all of its ink. `--tilt-max-deg` therefore stays at its default of 0 in the
shipped command, and the shipped programme is perpendicular everywhere.

That is the honest headline of this work: **the 59.9 mm was an allocator
problem wearing a kinematics costume.**

---

## 3. What had to be built before a tilted span could ship at all

Three items off `docs/TILT_EXPLORATION.md` §4's debt list. They are worth
keeping even though the shipped programme does not lean, because they are what
makes the flag *safe to turn on* — and because two of them found real bugs.

### 3.1 The validator now knows which way the pen points

`validate.validate_plan` re-derives every invariant of a plan from
`frames.fk`, and it could not see the one invariant a tilted plan rests on.
Its tip test is orientation-**agnostic** by construction: `tip_pos_many` steps
`pen_ext` along the tool z *of the FK'd pose*, so a pen leaning forty degrees
that still puts its tip on the curve passes it exactly as a perpendicular one
does. Every other gate is about the chain.

So `validate_plan` takes `tilt_max_deg`, and it **defaults to 0**. That is the
load-bearing decision: every plan written before tilt existed is now checked
against the perpendicular pen it was actually asked for, and only a plan that
was *granted* a cone may use one. The lean is re-derived
(`validate._pen_lean_deg`, `arctan2` and never `arccos`), never read off the
plan's own `tilt`/`max_lean_deg` fields — corrupt all three and the verdict
does not move. `scene_check` passes each segment's own cone, and
`tilt.plan_adaptive` records the cone the plan **needs** rather than the one the
run allowed, so turning the flag on cannot quietly weaken the certificate of the
hundreds of strokes that never lean.

Measured over every certified plan in the CSAIL corpus, the worst flat lean is
**6.3e-11 deg**, so the 1e-6 deg float slack is five orders of magnitude of
headroom.

It costs nothing: the tip, the pen axis and the chain points are three readings
of one forward kinematics, and folding them into a single `fk_many` call
removed two of the three FK evaluations `validate_plan` was already paying for.

### 3.2 The gates are arguments now, not module constants

`opts["margin_gate"]` was accepted by `plan_stroke` and **silently ignored** —
the band search read `pwl.MARGIN_GATE` off the module and the certification
chase read it again — so "plan this stroke at the strict comfort gate" was a
question the shipping entry point could not be asked. It answered the
permissive one and said nothing.
`tests/test_tilt.py::test_strict_gates_are_not_a_plan_stroke_option` pinned the
bug rather than the behaviour.

Both gates now reach all three places a gate has to arrive: the band DP, the
5 mm chase, and the independent validator — so an "ok" at 0.30 has been
searched, certified **and** re-derived at 0.30. Defaults are the shipping
constants, so no published number moved. The test now pins the behaviour
(`test_strict_gates_reach_plan_stroke`).

### 3.3 A tilted plan is shaped like a flat one, and is a sharp polyline

`smooth.RoundedPWL` rounds one scalar function of s; a tilted plan is three
(q7, tx, ty). That generalisation is **not** done, and `tilt._flat_shaped_fields`
now says so where a reader will find it instead of leaving it to be inferred
from a missing key. A tilted plan ships as the sharp polyline the DP certified,
every window zero. It costs nothing in the certificate — `chase` walks the same
5 mm samples under the same gates either way, and the flat pipeline already
ships sharp polylines when rounding fails to certify — it costs `|dq/ds|` at
the knots, and therefore the clock. On 50 mm and 10 mm rescues that is a few
knots on a few centimetres. On a long tilted stroke it would matter and the
generalisation would have to come first.

The rest of the shape was a **hard break**: `stroke_api.reverse_plan`
subscripts `windows` without a default and the tilt path had none, so a
tilt-rescued segment the sequencer wanted to draw backwards raised `KeyError`
in the middle of an allocation. It also named knot column 1 explicitly (silently
dropping the two tilt columns) and left `tilt`/`lean` in forward order beside
reversed joints. All three fixed and pinned.

`writing.densify` had the same class of problem one layer down: it re-solves IK
at Cartesian points *between* the plan's samples and hard-coded the pose to
`rotx(pi)`, so a tilted plan's own samples would survive verbatim while every
sample inserted between them was solved for a vertical pen — the executed
stroke would rock the pen to vertical and back between every pair of commanded
points. It takes the plan's `tilt` now, and interpolates the **vector**, never
the (lean, azimuth) pair.

### 3.4 Two bugs the new checks found

**The tilt chase did not reach the end of the stroke.** The `coverage_gap`
check added to `tilt._finish` failed immediately on a stroke that had been
reported "ok" for weeks. `planner.resample` steps by a fixed ds and stops at
the last WHOLE step, so chasing a 0.1373 m stroke at a raw 5 mm left the last
2.3 mm unplanned — and every gate in the certificate is **pointwise**, so not
one of them could see a tail that was never sampled. `stroke_api.prepare` fits
the step to the length for exactly this reason; the tilt path now does too.

**A tilted plan drew the chords of its own curve.** `tilt._prep` claims to do
what `stroke_api.prepare` does and replaced the stroke with the 10 mm *lattice*
resample unconditionally, where `prepare` only does so when the sheet clip
actually removed something. The certification chase was therefore handed the
chords instead of the curve — 0.65 mm of arc length on a 138 mm span of stroke
26, and a commanded path cutting every corner by the lattice's chord error.

**And one the pipeline's own cross-check found.** While the hover above a
leaning stroke briefly leaned with it, `sequence.endpoints` and
`writing.arm_program` disagreed about the pose, and `csail_schedule` said so on
the spot: *"sequencer priced transits the timeline does not pay: arm 71:
0.6230 s"*, on the one arm drawing a 15° span. The fleet keeps **one** hover
convention — vertical, above a leaning stroke as above a flat one — because
`paper.route` builds every detour it inserts out of vertical `lifted_config`
solutions, so a leaning hover does not avoid reorienting the pen during a
transit; it only moves where the reorientation happens. The pen reorients
during the **lift**, with the tip off the paper and the whole move swept by
`scene_check`. `writing.lifted_config`'s `tilt` argument keeps the argument on
the record.

---

## 4. The shipped run

```
ARIS_RIG=final6_opt python3 scripts/csail_schedule.py --arms all --max-probes 5 \
    --rotate 90 --target-width 0.8417 --offset 0.0 0.0 \
    --atlas out/atlas_final6_opt --tag _sub8 --two-pass --freeze-all-phases \
    --qd-frac 0.60 --subcheck 8 --draw-speed 0.15 --transit-speed 0.30 \
    --fps 12 --substeps 4 --final out/csail_final6_2pass_final.png --program

ARIS_RIG=final6_opt /home/franka/git/franka_manipulation_station/.venv/bin/python \
    scripts/csail_drawing_demo.py --schedule out/csail_schedule_sub8.npz \
    --summary out/csail_schedule_sub8.json --out out/csail_final6.html \
    --zip out/csail_final6.zip
```

Identical to `docs/TWO_PASS.md` §6 — the merge needs no flag, because it is not
an experiment. `--no-merge` recovers the old allocation.

| | `docs/TWO_PASS.md` (shipped) | **this run** |
|---|---|---|
| **coverage** | 99.3907 % | **100.0000 %** |
| traced / drawn | 9.8295 / 9.7796 m | 9.8295 / **9.8395 m** (splice overlap) |
| undrawn | 0.0599 m in 2 spans | **0.0000 m in 0 spans** |
| certified segments | 42 | **42** |
| segments independently re-validated | — | **42 / 42** (15 grey + 27 orange) |
| **makespan** | 81.688 s | **73.500 s** |
| — phase 1 (grey `[unsplit]`) | 27.167 s | **27.167 s** |
| — pen swap | 2.000 s | **2.000 s** |
| — phase 2 (orange) | 52.521 s | **44.333 s** |
| conducted pause | 47.2 s | **37.6 s** |
| min inter-arm clearance | 82.2 mm | **82.3 mm** (margin 80) |
| min frame clearance | 53.5 mm | **53.4 mm** (margin 50) |
| paper gate | PASS | **PASS**, chain 99.8 mm, tip −0.4 mm |
| pen-swap hold pose | 326.6 mm, 6/6 pass | **326.6 mm, 6/6 pass** |
| `scene_check` | PASS | **PASS, both phases** |
| animation | 32.6 / 16.5 MiB zip | **31.8 / 16.5 MiB zip** (budget 28) |
| tip vs drake FK | 0.195 mm | **0.195 mm** |
| **segments that lean** | — | **0** |

**Both spans closed and the clock went DOWN by 8.188 s.** Two things to keep
separate in reading that:

* **The merge is nearly free.** It added 59.9 mm of ink to two segments that
  already existed and created no new segment and no new pen-up — 42 segments
  before, 42 after, and the same 27 in the orange phase. Its own cost is the
  0.4 s of extra drawing.
* **The 8.2 s is the conductor's, not the merge's.** Phase 1 is unchanged to
  the millisecond (27.167 s, the same 15 segments on arms 2 and 97). Phase 2
  found a better priority order — `71 2 31 97` — with one retreat and three
  arms sent home, and came in at 44.333 s against 52.521 s. That search has a
  time budget, so it is a fair measurement of this run rather than a
  guaranteed property of the allocation.

Who draws what, with the merged spans marked:

| arm | mount | grey | orange |
|---|---|---:|---:|
| 2 | A, side | 1.6531 m | 0.8877 m |
| 97 | B, side | 1.8007 m | 1.5783 m — **+49.7 mm, stroke 26** |
| 31 | A, inverted | — | 2.2511 m — **+10.2 mm, stroke 30** |
| 71 | B, inverted | — | 1.6685 m |
| 13 / 17 | floor | — | — |

And the two strokes, end to end:

| stroke | arm | span | mm | lean |
|---|---|---|---:|---:|
| 26 | 2 | s[0.0000, 0.4192] | 153.3 | 0° |
| 26 | **97** | **s[0.4055, 0.7571]** — was [0.4055, 0.6213] | **128.6** | 0° |
| 26 | 71 | s[0.7571, 1.0000] | 88.8 | 0° |
| 30 | 71 | s[0.0000, 0.6488] | 126.7 | 0° |
| 30 | **31** | **s[0.6488, 1.0000]** — was [0.7012, 1.0000] | **68.6** | 0° |

---

## 5. What is left

**Nothing of the logo.** 9.8295 m traced, 9.8295 m drawn, 0 spans, 0.0000 m.

Three things this did **not** do, listed so the next person does not have to
rediscover them:

* **The atlas prefilter is still tilt-blind** (`allocate.atlas_cells` keeps only
  tilt-0 permissive cells). That only costs probe time — every stroke it lets
  through is probed for real — but it means **tilt cannot yet win a stroke the
  prefilter refused**. It did not need to here.
* **Corner rounding for tilted plans** (§3.3).
* **Phase 1 grey is fragile to conduct**, and was before this work: the split
  allocation is refused and the run falls back to the unsplit one, which is
  what `docs/TWO_PASS.md` shipped too (the phase is named "grey [unsplit]"
  there). Opening the tilt cone makes it worse, not better. That is a
  conductor/placement question, not an allocation one.
