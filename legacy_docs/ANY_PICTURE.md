# ANY PICTURE, ONE DOOR — AND WHAT THE TROLLFACE COST

*2026-08-24. Code: `scripts/draw.py`, `aris_sixarm/artwork.py`, `trace.py`
sections 7–8. Tests: `tests/test_draw.py`.*

Everything in this repo up to here draws **the CSAIL mark**. The pipeline is
general — probe, cover, repair, balance, split, merge, sequence, conduct, check
— but the three stages in front of it were not: the tracer knew that logo's two
RGB values, its 222 px gif, and a hand-tuned erosion count that told its
wordmark from its buildings. `scripts/draw.py` is the front door that removes
those three assumptions and nothing else. It **plans nothing of its own**; every
stage below the tracer is the call `scripts/csail_*.py` already makes.

```
ARIS_RIG=final6_opt python3 scripts/draw.py PICTURE [--inks auto|N]
    [--placement auto|off|FILE.json] [--out NAME]
```

---

## 1. The three assumptions, and what replaced each

| the CSAIL tracer assumed | `trace_art` measures |
|---|---|
| two inks, at known RGB | `detect_inks` — k-means over the picture's own ink, `--inks auto` picking the smallest k whose clusters are tighter than 46 in RGB |
| a 222 px source, upsampled 4x | `load_art` resizes to a WORKING RESOLUTION (`--work-px`, default 900), because every tuned pixel constant in the module is a length at some resolution |
| `LETTER_ERODE = 6` | `auto_fill_erode` — 1.35 x the median distance transform ON THE SKELETON, which is the picture's own line half-width |

Nothing below the palette changed. Zhang-Suen thinning, the skeleton graph,
the junction-through pairing that keeps an X two strokes, spur pruning,
endpoint bridging, marching-squares contours, RDP and `to_sheet` are the same
code the logo goes through, and `trace_logo` still exists and still returns
what it always did.

### 1.1 Read the background AFTER the composite, not before

`load_art`'s first version took the modal colour of the border, skipping
transparent pixels. On `assets/csail/csail_old_med.gif` that returns **orange**:
1 245 of its 1 480 border pixels are transparent and the 235 that are opaque are
the orange rule running off the edge. The unmixer is then handed the logo's own
ink as its paper, four grey "inks" are detected in a two-ink picture, and the
tracer returns **928 strokes** of interleaved confetti.

Composite first and the same border is 84 % white. Reading a background off
pixels that are not there is the failure mode; after a composite there are no
pixels that are not there.

### 1.2 Count the inks on the INTERIOR, not on the ink

Every dark line on white paper is bordered by a band of every grey in between,
and at 900 px that rim is ~30 % of the ink by area. It is enough for k-means to
find "black", "a dark grey" and "a mid grey" in a picture drawn with one pen —
and then `ink_masks` gives each pixel to whichever it is nearest and cuts every
stroke into three interleaved fragments.

`ink_core` erodes twice and asks only the interior, where a pen's colour is the
only colour there is:

| picture | cluster spread at k=2, on the ink | on the interior | inks chosen |
|---|---|---|---|
| `csail_old_med.gif` | 50.5 / 46.6 | **18.7 / 25.0** | **2** (was 4) |
| `trollface.png` (k=1) | 12.7 | **0.9** | **1** |
| `Trollface.png`, 222 px (k=1) | 109.2 | **10.3** | **1** (was 3) |

`--inks 3` on a one-pen drawing now collapses back to one (`_merge_close`)
rather than shredding it.

### 1.3 A line gets its centreline, a fill gets its edge

This is the tracer's only real judgement and it is the one that decides whether
a picture reads. A pen has one width, so a uniform-width stroke wants its
CENTRELINE. A solid region has no centreline worth drawing — the skeleton of a
filled letterform is a stick figure, and the skeleton of the Trollface's mouth
is a fork — but its BOUNDARY is exactly what a person would draw with a pen:
the grin curve plus one closed loop per tooth.

`auto_fill_erode` measures where to put the line. The distance transform
sampled on the skeleton is the local half-width of whatever the skeleton runs
down, and the skeleton is one pixel per unit LENGTH, so its median is
length-weighted: a picture that is mostly line reports its lines however large a
fill it also carries. On the CSAIL gif at 900 px it returns **5** against the
hand-tuned `LETTER_ERODE = 6` measured at 888 px. On the Trollface it returns
**8**, and 8 is the only value that works — see §3.1.

## 2. `bridge_endpoints` is the same answer, 38x faster

`bridge_endpoints` takes the globally best facing pair, merges it, and re-derives
every endpoint — which is what makes its result independent of stroke order, and
also what makes the naive form quadratic PER MERGE. On a picture that arrives
from the skeleton as 350 fragments (which is what a mis-detected background does
to any tracer) that is 180 million Python-level distance evaluations and a full
minute inside a function nobody suspects.

The search is now one numpy expression per merge, with the arrays indexed
`[i, j, ei, ej]` precisely so that C-order flattening reproduces the old loop
nesting and the winner is chosen on `(turn + approach, gap, position)`. It is
the same answer, ties included:

| strokes in | merges | old | new | identical |
|---:|---:|---:|---:|---|
| 30 | 4 | 0.03 s | 0.00 s | yes |
| 120 | 56 | 5.25 s | 0.16 s | yes |
| 220 | 122 | 37.73 s | 0.99 s | yes |

## 3. The Trollface

`assets/artworks/trollface/trollface.png` — 1 499 x 1 216, black line art on
transparency. The Wikipedia file page's copy
(`assets/artworks/trollface/trollface_wp.png`, from
`upload.wikimedia.org/wikipedia/en/7/73/Trollface.png`) is 222 x 180 and traces
correctly too, at less detail.

### 3.1 The trace, and the one number that matters

**46 strokes, 1.0 s** — 24 centrelines and 22 boundaries, one ink,
`black #000000`, fill-erode **8** measured, line half-width 6.0 px.

The fill-erode window is narrow and the picture says why. Measured per connected
component at the 900 px working resolution:

| component | area px | skeleton half-width, median | traced as |
|---|---:|---:|---|
| mouth (solid, with the teeth as holes) | 69 977 | 9 | **fill** |
| jaw outline + shading network | 35 851 | 5 | **line** |
| right eye (solid pupil) | 9 233 | 7 | **fill** |
| left eye (solid pupil) | 8 775 | 6 | **fill** |
| chin/cheek shading strokes | 1 920 / 1 592 | 2 | **line** |

At erode **11** the left eye flips to "line" and its filled pupil skeletonises
into a stick — visibly, and asymmetrically, since the right eye is fatter and
stays a fill. At erode **4** the bold jaw outline flips to "fill" and the face
is drawn as a double line. The measured 8 is inside the window; nothing was
hand-tuned for this picture.

**No confetti.** 2 of 46 strokes are under 20 working pixels and 7 under 40,
and the 25 mm floor at paper scale (`--min-len`, applied by `to_sheet`) drops
the last of them. The dense wavy interior shading — the stress test — comes
through as long single strokes because they are long single strokes in the
source: the junction-through pairing carries them past the places they touch
the jaw.

### 3.2 Placement — and the canvas's short axis binds again

```
ARIS_RIG=final6_opt python3 scripts/draw.py \
    assets/artworks/trollface/trollface.png --out trollface \
    --title "the Trollface" --inks auto --placement auto \
    --atlas out/atlas_final6_opt --arms all --max-probes 5 \
    --rotate 0,90 --scales 0.30 0.85 8 --search-offset 0.4 --offset-step 0.2 \
    --top 3 --jobs 16 --place-only
```

260 placements proxy-scored, **48 allocated for real**, 368 s on 16 workers.

**The search is a RANKING, so it pays only for what it ranks on.** Measured on
one 16.27 m placement of this picture, at the full allocator defaults:

| allocation | wall | of which balance+split | coverage | segments |
|---|---:|---:|---:|---:|
| `max_probes=3`, balance + split + `opt` sequencer | **602.3 s** | **544.4 s** | 100.53 % | 63 |
| `max_probes=2`, no balance, no split, `nn` sequencer, merge on | **39.4 s** | — | 94.55 % | 59 |

and on the chosen 17.46 m placement the shipped allocation spends **676.8 s of
its 724.1 s in the balancer**, against 14.5 s probing and 0.9 s sequencing.

`balance` only re-assigns spans a second arm already certified at the same
endpoints and `split` re-measures coverage and is invariant by construction —
**neither can move the number being ranked.** So `scripts/draw.py` passes
`balance=False, split=False` to the search. It keeps two things at the run's
own values: `max_probes`, because the probe budget is the one knob that *does*
move coverage, so a ranking at a different budget ranks a different question;
and the SEQUENCER, because `prune_unflyable` bans the spans whose tour comes
back infeasible and a worse tour bans more of them — on this picture the
nearest-xy chain is `inf` outright — and because it costs 0.9 s of 724.

That is the difference between a 48-placement search in 6 minutes and one in
8 hours. `scripts/csail_place.py` keeps the full defaults, so its published
placements still reproduce.

**The proxy is an upper bound and the ranking is a ranking.** The chosen cell
scored 100.0 % in the search and the shipped allocation of the same cell draws
99.71 %: the search does not see the real `draw_speed`, `qd_frac` or
`return_home`, and those change which spans an arm can fly to and therefore
which get banned. 0.29 pp on a number used only to order candidates.

### 3.3 A dense picture inverts `docs/BENCH.md`'s premise

`csail_schedule.select_profile` is built on the finding that *conducting* a cell
is the expensive half and *allocating* one is seconds, so it allocates all four
execution profiles and then conducts them in floor order, pruning any cell whose
lower bound cannot beat the incumbent. On the logo's 42 segments that is right
and it cut 24 cells to 8 over the bench corpus.

On this picture it inverts. The Trollface's 63 segments — a 3.6 m closed jaw
outline needs many handoffs — put **676.8 s of a 724.1 s allocation inside
`balance_loads`' split search**, and the four cells were 56 minutes of strictly
serial single-core work with 31 cores idle.

`--profile-jobs N` maps the four allocations over processes. Each cell is a
pure function of `(args, profile)`, so it **cannot change any result**, and
`tests/test_draw.py::test_allocating_the_profiles_in_parallel_changes_only_the_clock`
pins that: same shipped profile, same makespan to nine decimal places, same
floors, same prune decisions. The default is 1, so every published number
reproduces on the serial path. The *conducts* stay serial on purpose — each one
prunes the next on its certified makespan, and running them at once would throw
that away.

| rotation | best small | at 1.0 m² | at max size searched |
|---|---|---|---|
| 0° | 0.505 × 0.408 m, 100.0 % | — (1.034 × 0.835 = 0.86 m², 100.2 %) | 1.431 × 1.155 m, 94.3 % |
| 90° | 0.505 × 0.626 m, 100.0 % | **0.902 × 1.117 m, 100.0 %** | 1.431 × 1.772 m, 95.7 % |

The standing rule — largest area within 1 pp of the best real coverage anyone
achieved (100.3 %, at 0° and 0.66 m²) — picks **90°, 0.902 × 1.117 m
(1.007 m²) at offset (−0.20, −0.20), 100.0 % drawn**, against 0.86 m² for the
best 0° candidate in the same band.

It is the CSAIL finding again and for the same reason. **The 0° curve falls off
a cliff above ~1.1 m²** — 100.2 % at 0.86 m², 94.1 % at 1.36 m² — because the
dead strips are on the canvas's SHORT axis (x ≲ 0.12 and x ≳ 1.66, at every y),
so ≈ 1.54 m of the 1.803 m width is usable and any drawing wider than that eats
dead paper whichever way round it is. The 90° curve decays gently instead
(98.3 % at 1.33 m², 95.7 % at 2.54 m²) because turning the picture does not need
more width. Rotation buys **1.17× the area at the same coverage** here, against
2.1× for the CSAIL mark, which is what you would expect from a picture that is
1.23× wider than tall rather than 1.31×.

> **Coverage above 100 % is the splice overlap**, not a bug: a handoff cut
> draws `SPLIT_OVERLAP_M` of ink twice, so `drawn_len` can exceed `total_len`.
> `docs/FULL_COVERAGE.md` records the same thing (9.8295 traced / 9.8395 drawn).

## 4. What the Trollface actually cost

**Source.** `assets/artworks/trollface/trollface.png`, 1499 × 1216 RGBA black
line art on transparency (`SOURCE.md` records the provenance and the rights
note). One ink, so one pass and no pen swap.

### 4.1 The trace

`load_art` composites onto paper, reads white as the background, and
`detect_inks` returns **k = 1, black #000000**. `auto_fill_erode` measures the
line half-width at **6.0 px** at the 900 px working resolution and picks
**fill-erode 8** from it — no constant was typed.

| | |
|---|---|
| working resolution | 900 × 730 |
| strokes | **46** — 24 centreline + 22 boundary |
| path | 13 723 px |
| trace wall clock | 1.0 s |

The picture is its own answer to "does this confetti?" Its thin interior
shading — the brow hatching, the cheek and chin curves, the crease under the
mouth — is exactly the case the CSAIL logo never posed, and it survives as
**24 centreline strokes**, not as fragments: `out/trollface_trace.png` panel 4
shows the jaw outline closed, the mouth curve whole, the full teeth grid, both
eyes with pupils and highlights, and the brows. **No threshold needed
tuning.** The default `--min-px` and the endpoint bridging were enough, which
is the point of measuring `fill_erode` instead of declaring it: a picture whose
thick outline is 12 px across and whose shading is 2 px across gets one number
that separates them because the number came from the picture.

The 22 boundary strokes are the thick outline, and they are why the drawn
result looks *outlined* rather than *drawn with a fat pen*: a 12 px-wide black
band is traced round its edge, as two lines, because the rig's pen is 1 px wide
and has no other way to fill it.

### 4.2 The placement the search chose, and why it never ran

The search ranked **90°, 0.902 × 1.117 m at offset (−0.20, −0.20)** first, on
the rule in §3.2 — largest area within a point of the best coverage. Real
allocation confirmed it: **17.4559 m traced, 99.7132 % covered, 63 segments.**

Then **all four execution profiles refused it.** The interesting part is *how*:

| profile | how far it got | why it stopped |
|---|---|---|
| qd0.60 | conductor, then v1's go-home | `arm 97: go-home at segment 10 cannot clear the paper plane` |
| qd0.30 | conductor, then v1's go-home | `arm 97: go-home at segment 11 cannot clear the paper plane` |
| qd0.60+cluster | frozen-pose search | `arm 31 cannot stop clear of 2 (−117 mm)` |
| qd0.30+cluster | **conducted, 108.2 s** | `scene_check` **FAIL: frame 38.2 mm < 50 mm**, arms 31 and 71 |
| qd0.30+cluster, unsplit | **conducted, 127.8 s** | `scene_check` **FAIL: frame 43.2 mm < 50 mm**, arms 31 and 71 |

Read the last two rows carefully, because they say something the CSAIL runs
never did. **Inter-arm clearance passed both times** — 81.1 mm and 81.0 mm
against an 80 mm margin — and the veto came from the **frame**. The conductor
can only insert pauses; it cannot move an arm's elbow away from a rail it was
always going to touch. On a 1.007 m² drawing on this canvas, arms 31 and 71
reach across far enough that their chains foul the frame *while drawing*, and
no schedule fixes a static geometry problem.

`docs/MERGED_CANVAS.md` §3.3 predicts the refusal; it does not predict which
gate. It is worth recording that the binding constraint at this size was
**frame clearance, not inter-arm contention** — the six arms were never the
problem, the reach was.

### 4.3 What shipped

Confining the drawing to **web A**, where only unit A's arms reach, removes the
cross-unit contention *and* keeps every drawing arm off the far rail:

| | |
|---|---|
| placement | **0.850 × 0.686 m**, 0°, centred (0.902, 0.865) |
| traced / drawn | 13.2176 m / 13.1146 m |
| left empty | 0.1590 m in 3 spans |
| **coverage** | **98.7970 %** over **55 certified segments** |
| profile | **qd0.60+cluster** (qd_frac 0.60, cluster on, band `min_travel`) |
| **makespan** | **108.375 s** against a floor of 73.373 s |
| conducted pause | 93.167 s |
| min inter-arm clearance | **82.9 mm** (margin 80 mm) |
| `scene_check` | **PASS** |

| arm | mount | metres | segments | drawing | pen-up |
|---|---|---|---|---|---|
| 2 | web A | 7.650 | 28 | 61.2 s | 25.0 s |
| 31 | web A | 3.937 | 22 | 30.7 s | 17.8 s |
| 13 | web A | 1.527 | 5 | 25.3 s | 4.6 s |
| 17, 71, 97 | web B | 0.000 | 0 | — | parked as obstacles |

The profile grid shows the floor pruning paying for itself: `qd0.60` refused
after 491.9 s of conduct, `qd0.60+cluster` certified after 2 243.6 s, and both
qd0.30 profiles were then **pruned without conducting** — floors of 138.516 s
and 109.600 s cannot beat a certified 108.375 s. Two conducts, not four.

`--profile-jobs 4` did the four allocations in **1 082.2 s**. Whole run,
trace to zipped animation: **3 823.6 s**.

**The honest trade.** 0.583 m² certified against 1.007 m² refused — the shipped
Trollface is 58 % of the area the search wanted, and it sits in the lower third
of a 1.803 × 3.630 m sheet rather than across it. Three arms draw and three
watch. That is what this rig can certify for a picture of this shape, and the
refusal above is the reason, stated by the gate that refused it rather than
guessed at.


