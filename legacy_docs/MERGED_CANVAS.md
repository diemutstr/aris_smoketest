# THE MERGED CANVAS, AND THE TWO EXTENDED POLES

Two user decisions (2026-08-21) turn the mirrored six-arm layout from a
preview into a rig the pipeline runs end to end:

1. **The canvas is CONTINUOUS across the middle.** `rig_final6.MERGE_WEBS =
   True` — one drawable surface spanning both units *and* the 23.064 cm strip
   between the two webs. Extent **1.8034 × 3.63064 m** (`rig_final6.SHEET_FINAL6
   = (SHEET_FINAL[0], 2 × MIRROR_PLANE_CANVAS_Y)`), 6.548 m² of paper against
   2 × 3.066 = 6.132 m² before.
2. **Both side arms are re-clamped 20 cm lower on 20 cm of new pole** — arm 2
   and, by mirror symmetry, arm 97, at canvas **z 0.576** instead of 0.776.
   That is the optimum of `docs/ARM2_HEIGHT.md` §4, applied to both units.

> **FLAGGED IN EVERY OUTPUT.** The ~20 cm pole extensions are **not in the
> drawing**. Every number below assumes both are physically fitted. The same
> change is what makes the joint stiff (≈ 28 cm of bracket engagement instead
> of 5.8), which is the drawing's own *"conection base plate to boom pole is
> not stiff and stable"* — but until it is built **and** re-surveyed under
> load, `rig_final.CALIB_STATIC` stays at 0.03 for both side arms.

## 1. The named config, and how it becomes active

`aris_sixarm/rig_final6.py` now builds two six-arm fleets from the same
mirror, and `aris_sixarm/fleet.py` can install any of four rigs:

| name | arms | canvas | side arms |
|---|---|---|---|
| `final` (**default**) | 13, 31, 2 | 1.8034 × 1.700 | as drawn, z 0.776 |
| `final6` | + 17, 71, 97 | 1.8034 × 3.63064 | as drawn, z 0.776 |
| **`final6_opt`** | + 17, 71, 97 | 1.8034 × 3.63064 | **z 0.576, +20 cm pole** |
| `sixarm` | legacy six | 3.607 × 1.961 | legacy proxies |

```
ARIS_RIG=final6_opt python3 scripts/csail_schedule.py ...   # from a script
fleet.activate("final6_opt")                                # in process
```

**The default stays `final`.** Every published number in this repo and every
pinned test was earned on the 3-arm rig; leaving it as the default is what
keeps all of them reproducible by the command that produced them. The
env var is read by `aris_sixarm/__init__.py`, *before* anything can capture
`SHEET` — `atlas.py`, `viz/scene.py` and `bench/` all bind it into their own
namespace at import time, and a script's `from aris_sixarm.fleet import SHEET`
runs before its first statement does. `fleet.activate` covers the in-process
case: it mutates the `FLEET` dict **in place** (it is the one object every
`from .fleet import FLEET` holds) and rebinds `SHEET` in the three modules that
captured it, plus `allocate.ACTIVE` and `bench.HATCH_ARM`.

Three things the switch had to fix, each of which would otherwise have failed
silently:

- **`fleet.FLEET` was `fleet.FLEET_FINAL`** — the same dict object. Mutating it
  in place would have destroyed the registry. `FLEET` is now a copy.
- **`fleet.sheet_for`** returned the 3-arm web for any `rig="final"` spec, and
  it is the *only* sheet the planner sees (`stroke_api` → `clip_to_sheet`).
  Left alone, every unit-B stroke would come back `degenerate / off_sheet` and
  three of the six arms would silently draw nothing.
- **The lowered side arms had no ready pose.** `Q_READY_WALL` hovers 0.10 m
  over the paper from z 0.776; from z 0.576 the same joints put the pen
  **0.100 m under it**. `frames.Q_READY_WALL_LOW` is re-derived by the same
  recipe as the other final-rig ready poses (hover 0.10 m, 16 tool yaws ×
  `ik.Q7_GRID` × 4 branches, gated by `validate.check_pose` with the *extended*
  pole boxes active, best `min(margin, 2.5σ)`, restricted to y ≤ 1.35 so the
  park pose stays clear of the mirror plane): hover canvas (1.00, 0.99), margin
  **0.680**, σ **0.272**, min chain z 0.21 m, min frame clearance 0.557 m. All
  six ready poses pass their own gates and `scene_check.check_static` on the
  parked fleet reports **314 mm**.

`rig_final.py` — the drawing — is not modified. The extended-pole build is a
box list (`rig_final6.FRAME_BOXES6_OPT_W_CM`, via
`boxes_with_extended_pole()`) handed to `rig_final.frame_boxes_canvas(boxes=…)`,
and each `Arm6Spec` carries which build it stands in.

**What moves and what is new steel.** `side_plate`, `side_clamps` and
`side_bracket` are bolted to the arm and slide −20 cm with it — the same three
boxes `scripts/arm2_height_sweep.py` slides. `side_boom` is **not** slid: its
bottom drops 20 cm, because the whole point is that there is no pole down
there today. The sweep in `docs/ARM2_HEIGHT.md` kept the drawn pole and let the
bracket hang past its end, which is exactly what cannot be built — so the
numbers below are a **fresh sweep**, not that study's numbers re-quoted. The
lengthened pole is 20 cm of extra obstacle for every other arm; it costs them
nothing (§2).

## 2. What six arms cover on one continuous canvas

`python3 scripts/run_atlas6.py [--rig final6]` → `out/atlas_final6_opt/`,
`out/atlas_final6_opt.png` (and the `final6` pair). A **real** sweep of all six
arms over the whole 1.8034 × 3.63064 m canvas at the 2 cm grid, the standing
gates (margin ≥ 0.30, σ_min ≥ 0.14), the 15° tilt cone, the 110 mm pen and
**both** frames' boxes — 16 562 cells, 62–67 obstacle boxes per arm.

Everything before this was either a 3-arm sweep of one web
(`out/atlas_final`) or the symmetry PREVIEW (`scripts/make_atlas6_preview.py`),
which said in its own title that the seam strip and the cross-web region were
**reach-bound only, never scored**. Neither decision is expressible as a
reflection of the old atlas, so nothing here is mirrored.

| | as drawn (`final6`) | **extended poles (`final6_opt`)** |
|---|---:|---:|
| union strict-GO | 69.13 % (11 450 cells) | **75.93 % (12 575)** |
| reachable at all | 85.20 % | 86.29 % |
| dead (nobody) | 30.87 % | **24.07 %** |
| ≥ 2 arms | 7.86 % | **25.47 %** |
| ≥ 3 arms | 0.00 % | 0.64 % |
| ≥ 4 arms | 0.00 % | 0.03 % |
| most arms over one cell | 2 | **4** |
| **cross-unit** (an A arm and a B arm both GO) | 3.99 % | **8.46 %** |

Per arm, of the whole canvas:

| arm | mount | unit | as drawn | extended | of its own web | of the SEAM strip | on the OTHER web |
|---|---|---|---:|---:|---:|---:|---:|
| 2 | side | A | 9.89 % | **22.47 %** | 38.78 % | **49.63 %** | 197 cells |
| 97 | side | B | 9.89 % | **22.39 %** | 38.71 % | **48.90 %** | 217 cells |
| 31 | inverted | A | 14.98 % | 14.98 % | 25.67 % | 36.63 % | 112 cells |
| 71 | inverted | B | 14.90 % | 14.90 % | 25.55 % | 35.62 % | 128 cells |
| 13 | floor | A | 13.74 % | 13.74 % | 29.08 % | 0.00 % | 0 |
| 17 | floor | B | 13.59 % | 13.59 % | 29.09 % | 0.00 % | 0 |

**The four unaffected arms are unaffected to the cell** — 13, 31, 17 and 71
lose and gain exactly zero, which is what `docs/ARM2_HEIGHT.md` §6 predicted
for arm 2 and now holds for the longer pole too: the new profile sits at canvas
x 1.660–1.737, y 1.181–1.333 (and its mirror), 1.4–1.6 m from the inverted
arms' bases.

### The seam strip — the payoff

| band | area | GO | ≥ 2 arms | cross-unit |
|---|---:|---:|---:|---:|
| web A, y 0.000–1.700 | 3.130 m² | 68.66 → **75.17 %** | 5.72 → **22.40 %** | 1.61 → 4.36 % |
| **SEAM, y 1.700–1.93064** | **0.437 m²** | 77.47 → **88.00 %** | 41.67 → **77.20 %** | 41.30 → **74.91 %** |
| web B, y 1.93064–3.620 | 3.094 m² | 68.52 → **75.13 %** | 5.48 → **21.86 %** | 1.41 → 3.93 % |
| middle band (seam ± 0.40 m) | 1.893 m² | 68.36 → **77.37 %** | 15.30 → **46.09 %** | 13.97 → **29.61 %** |

The strip that used to be a 23 cm gap is now **the best-covered band on the
canvas** — 88.00 % GO against 75.1 % on either web — and 77.20 % of it is
reachable by two or more arms, three quarters of it by an arm from **each**
unit. The preview's headline number was *"cross-unit overlap on paper 0.00 %,
the mirror buys NO redundancy in the middle band"*. With one web and the two
pole extensions it is **74.91 % of the seam and 8.46 % of the whole canvas**,
and the overlap histogram gains a class that did not exist: 101 cells reachable
by 3 arms and 5 by 4.

### What is still dead, and why

24.07 % of the canvas (3 987 cells). The structure of it is the 3-arm rig's,
doubled: the left strip x ≲ 0.12 (feed roll + inverted-arm boom), the right
strip x ≳ 1.66 (guide rods + `paper_curl` — a *transport clearance* problem, not
a reach one), the four under-shoulder holes, and the corners. **The binding
constraint on this canvas is its SHORT axis**: 0.26 m of the 1.8034 m width is
dead on both units at every y, leaving ≈ 1.54 m of usable width against
3.63 m of usable length.

## 3. The CSAIL logo on this rig

### 3.1 Placement — and the canvas's short axis is what binds

`ARIS_RIG=final6_opt python3 scripts/csail_place.py --tag final6opt --arms all
--atlas out/atlas_final6_opt --rotate 0,90 --scales 0.40 1.0 13 --offset 1.0
--offset-step 0.2 --top 6` → `out/csail_placement_final6opt.{json,png}`.
742 placements proxy-scored, **156 allocated for real**.

Two things had to be fixed before the search meant anything:

- **`--rotate` is new.** The logo is 1.31× wider than tall; the canvas is 2.01×
  taller than wide. `trace.to_sheet(rotate_deg=…)` turns the pixel polylines
  about their own bbox centre before the aspect-preserving fit, so a rotation
  is a placement variant and never a distortion. At every scale the 90° logo is
  the same width and 1.31× longer — 1.683 × **2.204** m at scale 1.0 against
  1.683 × 1.286 m upright, **1.71× the area**.
- **`--base-width auto` is new, and the old constant was silently dead.**
  `BASE_WIDTH = 2.4106 m` is the margin-limited width of the *legacy 3.607 m*
  sheet. On the 1.8034 m canvas even 0.7 × 2.4106 = 1.687 m exceeds the 1.6834 m
  the margin allows, so *every* scale collapsed onto the same logo — visible in
  `out/csail_placement_final.json`, where seven scales report one width and one
  coverage. The base width is now measured on the active canvas per rotation.

| rotation | best small | at max size |
|---|---|---|
| 0° | 0.758 × 0.579 m, **99.1 %** | 1.683 × 1.286 m (2.16 m²), 88.9 % |
| 90° | 0.673 × 0.882 m, **99.1 %** | 1.683 × 2.204 m (3.71 m²), 74.1 % |

Above ~1.3 m² of logo the 90° curve is strictly better than the 0° one at the
same area; below it they tie. **The standing rule** — largest within 1 pp of the
best real coverage anyone achieved — therefore picks **90°, 0.842 × 1.102 m
(0.928 m²) at offset (−0.20, +0.20), 98.9 % drawn single-pass**, against
0.758 × 0.579 m = 0.44 m² for the best 0° candidate in the same band. Rotation
buys **2.1× the area at the same coverage**.

**Why size costs coverage here, and it is not the long axis.** The dead strips
are on the canvas's *short* axis and are there at every y: x ≲ 0.12 (feed roll
+ inverted boom) and x ≳ 1.66 (guide rods + `paper_curl`). 0.26 m of the
1.8034 m width is gone, leaving ≈ 1.54 m usable. Any logo wider than that eats
dead paper whichever way round it is — which is exactly where both curves fall
off, and why rotating (which does not need more width) is the lever that works.

### 3.2 The middle band: coverage redundancy is NOT concurrency

`python3 scripts/middle_band_diag.py` → `out/middle_band_diag.json`. For every
arm pair it takes the atlas's own certified poses in a band and measures the
capsule clearance of every (pose of A, pose of B) pair — the pure geometric
question, no schedule involved. Margin = safety 0.05 + calib 0.03 = **80 mm**.

In the seam strip (2 cm grid, subsampled 1-in-4):

| pair | | shared cells | pose pairs ≥ 80 mm | median | worst |
|---|---|---:|---:|---:|---:|
| **31 vs 71** | cross-unit, both inverted | 22 | **33.9 %** | **21 mm** | −180 mm |
| **2 vs 97** | cross-unit, both side | 51 | **45.9 %** | 63 mm | −180 mm |
| 2 vs 31 | same unit | 1 | 93.7 % | 250 mm | −180 mm |
| 71 vs 97 | same unit | 6 | 94.6 % | 250 mm | −160 mm |
| 2 vs 71 | cross-unit, diagonal | 4 | 94.9 % | 250 mm | −140 mm |
| 31 vs 97 | cross-unit, diagonal | 1 | 95.1 % | 250 mm | −139 mm |

**The two pairs that own the seam's redundancy are the two pairs that cannot be
in it at once.** 31/71 clear each other in only a third of pose pairs with a
median clearance of 21 mm — a quarter of what is required. The *diagonal*
cross-unit pairs (2↔71, 31↔97) are 95 % clear: they are 1.5 m apart, not 1.1 m.

That is the whole coordination story of this rig, and the conductor says the
same thing in its own words. Every six-arm CSAIL run on a seam-centred
placement was **refused** by `coordination.coordinate`, at all four execution
profiles, after the full re-sequence ladder and conductor v1's go-home:

```
arm 97 has no monotone pause schedule inside 54 s; 0 priority orders were
tried; v1 does not re-route — try a placement that keeps the arms further
apart; the impossible indices are INK: arm 2 segment(s) [3] — no order fixes
that, only a different allocation
```

so the extended poles' cross-seam reach is real **redundancy** — either side arm
can draw the strip, and the atlas is right about that — but it is not
**concurrency**, and conductor v1 (which may only insert pauses) has no move
that recovers it. Dropping one contender does not help by itself: with arm 2
removed the refusal simply moves to 71 vs 97, and with both side arms removed
to 31 vs 71, the worst pair of all.

**What would recover it** (none of it is v1's job): a conductor that can
re-route a pen-up rather than only wait; a park pose per arm that is provably
outside every other arm's remaining tube for the whole run (the frozen-pose
refusals are literally `arm 97 cannot stop clear of 71 (−19 mm)`); or spacing
the two units so the hanging mounts are more than ~1.1 m apart, which is a
`rig_final6.GAP_CM` question and a fabrication one.

The pressure is not only cross-unit. In each unit's OWN half (y 0.60–1.50) the
side/inverted pair is **83.6 %** clear (2 vs 31, 46 shared cells) and 83.3 % in
unit B (71 vs 97) — against a 3-arm rig where that pair has never been a
problem. The 20 cm drop is what changed: the side arm now reaches r ≈ 0.82
instead of 0.64 and its whole body sits 20 cm closer to the paper and to its
own unit's inverted arm. **The coverage the extended poles buy and the
contention they create are the same fact.**

### 3.3 Which runs conduct, and the one variable that decides it

Twelve pipeline configurations were conducted against this placement family.
The pattern is not about the fleet or the placement:

| # | fleet | passes | placement | transit | outcome |
|---|---|---|---|---|---|
| A | all 6 | two | seam-centred, chosen | 0.8 | REFUSED (all 4 profiles) |
| B | 5 (no 2) | two | chosen | 0.8 | REFUSED |
| C | all 6 | two | 0.673 × 0.882 on the plane | 0.8 | REFUSED |
| D | 13,31,17,71 | two | chosen | 0.8 | REFUSED (31 vs 71) |
| F | 5 (no 97) | two | chosen | 0.8 | REFUSED |
| G | 13,31,17,97 diagonal | two | chosen | 0.8 | REFUSED |
| H | 13,2,17,71 diagonal | two | chosen | 0.8 | REFUSED |
| I | all 6 | two | shifted −0.40 | 0.8 | REFUSED |
| J | all 6 | two | entirely inside web A | 0.8 | REFUSED (2 vs 31) |
| K | all 6 | two | chosen | 0.30 | REFUSED |
| M | 13,31,2 (unit A) | two | shifted −0.40 | 0.8 | REFUSED |
| L | all 6 | ONE | chosen | 0.30 | conducted; `scene_check` vetoed it |
| N | all 6 | two, `--freeze-all-phases` | chosen | 0.30 | REFUSED |
| O | all 6, **as drawn** | ONE | chosen | 0.30 | same 40.9 mm veto |
| P | all 6, `--sequencer nn` | ONE | chosen | 0.30 | REFUSED |
| **Q** | **all 6** | **ONE** | **on the plane, 0.20 m right** | **0.30** | **CERTIFIED, shipped** |

**Every refusal was `phase 1: grey could not be conducted`** — and phase 1 of a
two-pass run is the pass that must GO HOME at the end, because the pass after it
starts from the ready pose (`csail_schedule.build_phases`). On this rig the trip
home is what cannot be scheduled: six arms whose park poses sit in the middle
band, each having to cross the others' swept tubes to reach them. It is
`docs/IDLE.md`'s thesis pushed past its breaking point — there, 79 % of every
pause-second was an arm waiting for permission to reach its park pose; here the
permission never comes. A single pass FREEZES in place and conducts; a two-pass
run with `--freeze-all-phases` is the obvious next thing to try and is why that
flag exists.

**And then the validator has its own objection, which is about steel and not
about arms.** The single-pass run conducts — 48.0 s makespan, 60.0 s of pauses,
39/39 segments valid, progress monotone, **83.1 mm minimum inter-arm clearance
against an 80 mm margin** — and `scene_check` still refuses it, on

```
min frame clearance 40.9 mm (margin 50 mm), arm 71 at t=19.53 s; FAIL: arms [71]
```

i.e. an inverted arm's PEN-UP TRANSIT clipping the frame. The planner certifies
STROKES against the frame boxes; a transit is a straight joint-space line
between two hover poses and nothing checks it until `scene_check` does. The
same 40.9 mm appears on the AS-DRAWN rig at the same profile, so it is **not**
the pole extension: the reachable boxes nearest an inverted arm are its own
unit's back-left corner post and brace and — because the two frames abut — the
OTHER unit's, at 0.60–0.68 m. `post_BL@A` and `post_BL@B` stand together as one
full-height column at canvas x −0.212…−0.136, y 1.739…1.892, which is exactly
the left end of the seam. **Merging the webs put drawable paper next to a
doubled corner post**, and an elbow swinging left near the seam finds it.

Levers, in the order they cost least: place the logo clear of that corner (the
canvas's usable width is 1.54 m and the corner eats the left end of it); raise
`writing.LIFT_Z` from 60 mm so transits arc higher — currently a module
constant and not a knob, and it would have to be threaded through
`sequence.cost_matrix` too or the sequencer would price a transit the timeline
does not fly; or teach the planner to route pen-ups, which is conductor v2.

**The cheapest lever worked.** Moving the logo 0.20 m right — off the corner,
onto the mirror plane — took the frame clearance from **40.9 mm to 56.1 mm**
and everything else fell out (run Q).

### 3.4 The shipped run

```
ARIS_RIG=final6_opt python3 scripts/csail_schedule.py --arms all --max-probes 5 \
    --rotate 90 --target-width 0.8417 --offset 0.0 0.0 \
    --atlas out/atlas_final6_opt --tag _final6 \
    --draw-speed 0.15 --transit-speed 0.30 --fps 12 --substeps 4 \
    --final out/csail_final6_final.png --select-profile --program

ARIS_RIG=final6_opt /home/franka/git/franka_manipulation_station/.venv/bin/python \
    scripts/csail_drawing_demo.py --schedule out/csail_schedule_final6.npz \
    --summary out/csail_schedule_final6.json --out out/csail_final6.html \
    --zip out/csail_final6.zip
```

**0.842 × 1.102 m, turned 90°, centred at canvas (0.9017, 1.81532) — dead on
the mirror plane.** 38 certified segments, 8.5488 of 9.8295 m drawn =
**86.97 % coverage**, single pass.

| profile | qd_frac | cluster | floor | makespan | outcome |
|---|---|---|---|---|---|
| qd0.30 | 0.30 | off | 64.161 s | — | not conducted (floor cannot beat 50.125 s) |
| qd0.30+cluster | 0.30 | on | 48.041 s | 54.542 s | certified |
| **qd0.60** | 0.60 | off | 37.898 s | **50.125 s** | **certified — shipped** |
| qd0.60+cluster | 0.60 | on | 31.842 s | 51.375 s | certified |

Three of four certified; the fourth was pruned on its own floor. `scene_check`
**PASS**: minimum inter-arm clearance **81.8 mm** against an 80 mm margin
(arms 2 and 31 at t = 23.10 s), minimum frame clearance **65.6 mm** against
50 mm, 38/38 segments re-validated, progress monotone, all six frozen poses
through their own gates. Animation: 602 frames @ 12 fps = 50.1 s, 348 ink
chunks, pen tip on the commanded curve to **0.169 mm** against drake's own
kinematics, 29.5 MiB of HTML / **16.2 MiB zipped** (budget 28).

| arm | unit | mount | ink | segments | metres | of it in the seam strip |
|---|---|---|---|---:|---:|---:|
| 97 | B | side | grey | 13 | 2.738 | 0.822 |
| 71 | B | inverted | orange | 11 | 2.292 | 0.385 |
| 31 | A | inverted | orange | 7 | 2.241 | 0.683 |
| 2 | A | side | orange | 7 | 1.278 | 0.605 |
| 13 / 17 | A / B | floor | — | 0 | 0.000 | — |

Priority order 71 → 31 → 2 → 97, 37.6 s of conducted pauses.

**The payoff, as a number: 2.4946 m — 29.18 % of all the ink — lies inside the
old 23.064 cm seam strip**, and **6 of the 38 segments (21.44 % of the ink)
cross a former web edge.** On the two-web layout that paper does not exist, and
neither does that ink. All four hanging arms contribute to it, two from each
unit, which is the cross-unit redundancy of §2 being spent rather than merely
measured.

**The two floor arms draw nothing**, and that is geometry, not allocation: 13
and 17 sit at canvas y −0.127 and 3.757, and every placement whose coverage is
worth having sits in the middle band 1.5 m from both. It reproduces the
three-arm era's finding on a longer canvas — the outer thirds are one arm's
each, and no logo small enough to be well covered reaches them.

## 4. Assumptions, all of them

1. **The two 20 cm pole extensions are fitted.** Not drawn steel. Without them
   the rig is `final6`, and the table in §2 is the left-hand column.
2. **The paper is one web, flat and drawable across the 23.064 cm seam.** The
   strip lies over the two frames' abutting top rails, not over a tabletop, and
   nothing in the drawing shows a bridging surface there. It is now *scored*;
   whether it can be *drawn on* is a build question.
3. **The two frames abut with zero gap** (`rig_final6.GAP_CM = 0`). Levelling
   pads overhang the leg lines by 0.33 cm, so pads touching would hold the
   structural faces 0.66 cm apart. One knob moves everything.
4. **Unit-B ids 17 / 71 / 97** are assumed; no drawing names them. (The user
   did not object when asked, so they stand.)
5. **Base-plate yaws are unconfirmed** (`docs/FINAL_RIG.md` Flags #6) and the
   mirror propagates that to unit B unchanged.
6. Every flag in `docs/FINAL_RIG.md` applies to **both** units.
