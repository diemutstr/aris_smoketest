# Cut at every transition, then merge — the fewest pieces a picture draws in

Pete, 2026-09-11: *"what if we cut lines everywhere where there is a transition
and then figured out some sort of greedy merging and allocation trying to
minimize the overall number of lines?"*

Yes — and it does not need to be greedy. Cutting at every transition produces
an **alphabet of atoms**, and choosing a drawer for each atom so that the number
of **pieces** is smallest is an exact per-line dynamic programme: `O(atoms ×
states)`, **10 ms for a thousand lines**, provably optimal per line. The greedy
merge Pete was bracing for is unnecessary.

Everything below is `aris_sixarm/traces.py` on the shipped atlas
(`out/atlas_proposed_h0970_lat0860_gated63`, `is_current` True for all six arms)
and the stage pattern docs/V2_WORKCELLS.md recommends. **No constant, no planner
semantic and no existing module changes** — this is a new module, its tests and
this page.

```
ARIS_RIG=proposed ARIS_TOOL=lateral python3 -m aris_sixarm.traces \
    --lines out/csail_schedule_h097_v19_strokes.json \
    --synthetic strokes:1000 --synthetic hatch:1000 --synthetic scribble:1000
```

## 1. The method, in four steps

**1. The capability set.** Sample each line every `SAMPLE_DS` = 4 mm and read at
each sample the set of **(stage, arm)** pairs that may draw that point: the arm
has a certified drawing pose there in the atlas (`atlas.strict_go` — `margin ≥
0.30`, `sigma_min ≥ 0.14`, the same gate every envelope in V2_WORKCELLS was
measured under) **and** the point lies inside that arm's work cell for that
stage. 4 mm against a 2 cm atlas lattice means the map's own features are five
samples wide.

**2. The atoms.** Cut wherever that set changes, and bisect each cut to
`REFINE_TOL` = 0.01 mm. Between two cuts every point of the line has *exactly
the same* set of possible drawers, so the stretch is indivisible — nothing is
ever gained by cutting inside it. The CSAIL logo's 39 strokes become 88 atoms;
a thousand hatch lines become 4 142.

**3. The DP.** Assign each atom one pair from its set, unit cost per change. The
standard chain recurrence, and because the change cost is the same whatever you
change *from*, the `O(states²)` relaxation collapses to "stay where you are, or
switch from the cheapest predecessor" — `O(states)` a step. Pieces = maximal
runs = changes + 1. **Per line is enough**: the piece count is a sum over lines
and one line's atoms constrain no other line, so minimising each line minimises
the total.

**4. The seams.** Two kinds, wanting opposite treatment:

- **In an overlap zone** both arms certify a stretch, and the DP's switch point
  inside it is arbitrary. Put the seam at the **middle of the overlap** — the
  furthest either arm is from the edge of what it can do — and **overdraw** each
  side by δ = 5 mm in its own drawing direction, so the two strokes lap rather
  than abut and a registration error under δ leaves no white gap. The lap is
  clipped to the zone, so a 6 mm overlap yields a 6 mm lap, not a 10 mm one.
- **At a hard edge** the overlap is empty: the last point one arm certifies is
  the first the other does. Nothing to lap into; the pieces meet at the
  transition point exactly and the seam is only as good as the calibration.

A run shorter than `MIN_PIECE_M` = 10 mm is **absorbed** into whichever
neighbour can draw all of it (the longer one, if both can). A 6 mm piece is two
pen-ups and two registrations for less ink than the lift costs. Absorption can
only ever *reduce* the piece count, so it cannot break the DP's optimality — and
measured on 400 random chains it never fires, because the DP's answer already
contains no absorbable short piece. It is there for the map, not for the DP.

## 2. The stage cells, as data

`traces.StageCell(stage, arm, region)` is the work-cell object V2_WORKCELLS §5
lists as missing: *"this arm may only draw here, during this stage"*. The
default `zigzag_pattern()` is verbatim the stage list
`scripts/workcell_envelopes.py` evaluates as `3-active-rowband-y20+6seams`:

| stage | arms and cells | ink-vs-ink |
|---|---|---|
| 0 | 13 → R0, 71 → R1, 2 → R2 | +85.8 mm |
| 1 | 17 → R0, 31 → R1, 97 → R2 | +85.8 mm |
| 2 | 13 → SEAM0, 97 → SEAM1 | +250.0 mm |
| 3 | 17 → SEAM0, 2 → SEAM1 | +250.0 mm |
| 4 | 31 → SEAM0, 97 → SEAM1 | +250.0 mm |
| 5 | 71 → SEAM0, 2 → SEAM1 | +238.6 mm |
| **6** | **13 → SEAM0, 31 → SEAM1** | **+194.3 mm** |
| **7** | **17 → SEAM0, 71 → SEAM1** | **+201.1 mm** |

with `R0 = y ∈ [0.000, 1.010]`, `R1 = [1.410, 2.220]`, `R2 = [2.620, 3.620]`,
`SEAM0 = [1.010, 1.410]`, `SEAM1 = [2.220, 2.620]`, all at the full block width
`x ∈ [0.16, 1.64]` — one arm per full-width **row** band, a **0.40 m** dead band
in y, the two columns alternating down the rows, then **six** 2-active seam
stages. +85.8 mm of ink-vs-ink, 2.35× the serial makespan.

**Stages 6 and 7 are the seam correction** (V2_WORKCELLS §4b, commit `07da40e`;
this module's own stages 6 and 7 are the document's stages 7 and 8). A seam
needs an outer arm *and* a middle arm: SEAM1's floor, `y ∈ [2.24, 2.40]`, is
reachable only by 31 or 71, and the four-seam version offered SEAM1 to nobody
but 2 and 97. That hole — 2.31 % of the block, and CSAIL stroke 17 entirely — is
what §5 of this document found from the other end, and it is now closed. Every
number below is the eight-stage pattern; the deltas against the superseded
six-stage figures are marked where they matter.

Stages 4 and 5 re-offer arms 97 and 2 *exactly* what stages 2 and 3 do, and
stages 6 and 7 re-offer arms 13 and 17 the same way, so `capability()` folds
them into one state each (18 cells → 14 states); splitting a seam's ink between
two interchangeable stages would be noise in the load table and an arbitrary
choice in the DP. Only **two** of the four new cells are new states — 31 and 71
on SEAM1 — which is exactly the coverage the correction was for.

`single_stage_pattern()` is the control: every arm everywhere in the block, no
cells at all. **It is not a runnable pattern** — six arms drawing at once never
clears, and eroding every Voronoi block by 0.60 m still leaves the worst pair at
−23.2 mm. Its piece count is the floor staging is charged against, nothing more.

## 3. The CSAIL logo at v19's placement

The 39 traced strokes of `out/csail_schedule_h097_v19_strokes.json` (16.805 m of
ink; byte-identical to the copy in GUI job `20260910-124546-ce72`), which v19
allocated as **46 segments**.

| | pieces | hand-overs | of which same arm | gaps | covered | ms |
|---|---|---|---|---|---|---|
| **v19, as shipped** | **46** | — | — | — | 100.2 % (`coverage`, drawn/traced) | 3 713 s job |
| **single stage** (no cells) | **40** | 1 (1 overlap) | 0 | 0 | 100.0 % | 17 |
| **zigzag + 6 seams** | **54** | 15 (1 overlap, 14 hard) | 8 | **0** | **100.0 %** | 30 |
| *(superseded: zigzag + 4 seams)* | *50* | *8 (0 / 8)* | *6* | *10* | *94.8 %* | *25* |

**And it is 40 on v19's own map too.** v19 ran against
`out/atlas_proposed_h0970_lat0860` — the *un*-regated sweep — with
`tilt_max_deg = 15` and the allocator's permissive prefilter, so the table above
is the DP on a strictly harder map than v19 had. Run on v19's exact atlas, gate
and tilt allowance (`--atlas out/atlas_proposed_h0970_lat0860 --gate flat
--tilt-max-deg 15`), the answer is the same: **40 unstaged, 50 staged**, at
95.6 % staged coverage. The piece count is not an artefact of which map it reads.

**40 against 46.** With no cells the DP draws 38 of the 39 strokes in a single
piece each. The one exception is **stroke 26** — the stroke `tests/
test_merge_spans.py` pins as the logo's hardest, where *"arm 2's certified
interval stops at 0.6213 and arm 71's starts at 0.7571"* — and the DP hands it
over 71 → 2 once, inside the overlap the atlas does offer. That the piece
minimiser and the shipped regression independently land on the same stroke is
the best single check this module has.

The six extra segments v19 carries over the DP's 40 are not a defect in the
allocator; they are the price of a search that optimises *cost* with pieces as a
by-product, against a module that optimises pieces directly. **Caveat, stated
once:** 40 is what the *atlas* permits at 2 cm resolution, which is the
allocator's own prefilter and not a plan. Each of the 40 still has to be
accepted by `plan_stroke`, and a piece that refuses would split again.

**Staged, the logo costs 14 pieces and none of its ink.** The four extra pieces
over the six-stage version are the price of the 5.2 % of ink (stroke 17 and
0.868 m) that the six-stage version simply could not draw; the gap count goes
from 10 to **zero**. ARCHITECTURE_V2 predicted exactly this — "the correction is
known and costs four pieces, 50 → 54, for the 5.2 % of ink the six-stage version
could not draw" — and this is the run that confirms it.

## 4. A thousand lines

Three synthetic sets over the certified block, deterministic seeds
(`traces.synthetic`): 1 000 long random strokes, a 1 000-line 15° hatch (992
survive clipping), and a 1 000-curve scribble.

| set | pattern | lines | atoms | **pieces** | hand-overs (ovl / hard / same arm) | gaps | covered | **ms** |
|---|---|---|---|---|---|---|---|---|
| strokes, 660.3 m | zigzag | 1 000 | 3 130 | **1 756** | 756 (216 / 540 / 412) | **0** | **100 %** | 888 |
| strokes | single | 1 000 | 2 614 | **1 254** | 254 (234 / 20 / 0) | 0 | 100 % | 558 |
| hatch, 1 368.6 m | zigzag | 992 | 4 495 | **2 170** | 1 178 (764 / 414 / 247) | **0** | **100 %** | 1 005 |
| hatch | single | 992 | 4 105 | **1 869** | 877 (855 / 22 / 0) | 0 | 100 % | 703 |
| scribble, 213.4 m | zigzag | 1 000 | 1 693 | **1 166** | 166 (19 / 147 / 104) | **0** | **100 %** | 478 |
| scribble | single | 1 000 | 1 551 | **1 024** | 24 (19 / 5 / 0) | 0 | 100 % | 286 |

**The deltas against the superseded six-stage pattern**, zigzag rows only: the
1 000-stroke set goes 1 661 → **1 756** pieces (+5.7 %) and 97.05 → **100 %**;
the hatch 2 129 → **2 170** (+1.9 %) and 97.45 → **100 %**; the scribble
1 148 → **1 166** (+1.6 %) and 97.71 → **100 %**. Every gap — 315, 304 and 79
stretches with no drawer at all — becomes **zero**. The atom counts rise
(2 885 → 3 130, 4 142 → 4 495, 1 597 → 1 693) because two more states mean two
more ways for the capability set to change along a line, and the DP cost rises
with them: 814 → 888 ms on the 1 000-stroke set, still **under a second**, of
which the DP itself is 10 ms.

Load, metres of ink per arm, and the theoretical stage makespan (a stage costs
its busiest arm, because within a stage the three actives are static keep-outs
for each other and need no conductor; the sequence costs the sum):

| set | pattern | 2 | 13 | 17 | 31 | 71 | 97 | makespan |
|---|---|---|---|---|---|---|---|---|
| strokes | zigzag | 113.4 | 105.3 | 101.6 | 115.2 | 113.9 | 113.1 | 302.2 m |
| strokes | single | 113.8 | 104.9 | 102.9 | 113.4 | 113.8 | 113.9 | 113.9 m |
| hatch | zigzag | 211.4 | 238.8 | 219.0 | 228.8 | 231.0 | 247.0 | 623.7 m |
| hatch | single | 204.7 | 243.0 | 217.4 | 226.0 | 232.8 | 253.2 | 253.2 m |
| scribble | zigzag | 33.7 | 38.0 | 37.9 | 35.3 | 35.0 | 33.7 | 101.0 m |
| scribble | single | 34.7 | 37.4 | 37.4 | 34.8 | 34.7 | 34.7 | 37.4 m |

The two extra stages move the load **onto the middle arms**, which is the ink
they could not be given before: 31 goes 94.5 → 115.2 m and 71 92.6 → 113.9 m on
the 1 000-stroke set, and the six arms' loads are now within 13 % of each other
where they were within 35 %. The stage makespan rises with the stage count
(281.9 → 302.2 m), which is the same 6.4 % of parallel speedup
V2_WORKCELLS §4b charged the correction: two more stages each cost their
busiest arm, and stages 6 and 7 only pick up what stages 2–5 could not.

Per-stage loads are in the tool's own output; for the hatch the two main stages
carry 527.8 m and 544.0 m and the four seam stages 101.0 / 89.6 / 31.2 / 47.2 m.

**Where the time goes.** For the hatch's 992 lines and 4 142 atoms:
**segmentation 884 ms, DP + absorption + seams 10 ms.** Pete's "milliseconds" is
right about the merge and wrong about the cutting — 99 % of the cost is reading
the capability map at 4 mm, which is embarrassingly parallel and entirely
tunable (`--ds`). The optimisation itself is free.

## 5. Staged against unstaged

**Staging costs 24 – 33 % more pieces**, and the mechanism is not what it looks
like:

| set | single | zigzag | extra | of the extra, same arm in a later stage |
|---|---|---|---|---|
| CSAIL | 40 | 54 | +14 | 8 of 15 hand-overs |
| strokes | 1 254 | 1 756 | +502 | 412 of 756 |
| hatch | 1 869 | 2 170 | +301 | 247 of 1 178 |
| scribble | 1 024 | 1 166 | +142 | 104 of 166 |

Most of what staging costs is **an arm handing a line over to itself**, one
stage later, because the line crossed out of its row band into the dead band and
the dead band belongs to a seam stage. That is a barrier and a re-approach, not
a registration risk: the same arm, the same calibration, and the seam is exactly
where the two cells abut. It is also the reason **every dead-band crossing is a
hard edge** — the stage cells tile the block with no overlap, so there is
nothing to lap into. Staged, δ buys almost nothing; unstaged, it is used at
almost every hand-over.

The other direction is the point of staging and this module does not measure it:
unstaged, the makespan is one arm's load (113.9 m for the strokes set) because
nothing is serialised — but nothing clears, either. Staged, the makespan is
281.9 m, 2.5× worse on paper and the only one of the two that can actually run.

**A staged hand-over is cheaper than it looks in one more way**, and under the
eight-stage pattern it is as cheap as it can possibly be: **48.1 % of the
certified block keeps ≥ 2 stage-compatible drawers (mean 1.524 per cell), which
is the atlas's own ceiling to three decimal places** — the staged and unstaged
capability maps now give the identical figure, measured cell by cell over the
13 650 certified cells of the block. The six-stage version was 43.5 % / 1.457.
V2_WORKCELLS' 41.6 % → 47.1 %, measured at `--stride 2` with
charge-to-first-stage accounting, is the same movement. **The staging now throws
away none of the redundancy the atlas offers**, which is what makes a faulted
arm's ink re-queueable rather than lost.

### The one real defect in the recommended pattern — FIXED, 2026-09-11

**This section is kept as the record of how the hole was found and closed.**
Under the eight-stage pattern the zigzag leaves **0.00 % of the certified block
with no drawer**, every gap count in §3 and §4 is zero, and the block coverage
figures below all read 100.00 %. What follows is the six-stage measurement that
prompted stages 6 and 7.

The six-stage zigzag left **2.31 % of the certified block with no drawer at
all**, and it was not spread around the rim as V2_WORKCELLS §4 suggests. It was
a single strip:

```
   SEAM0  y ∈ [1.010, 1.410]   100.00 % covered
   SEAM1  y ∈ [2.220, 2.620]    79.00 % covered   <-- all of the loss
   ROW0 / ROW1 / ROW2            100.00 % each
```

SEAM0 is offered to arms 13, 17 (row 0, reaching up to y = 1.34) **and** 31, 71
(row 1, reaching down to y = 1.08), so the two sides cover it between them.
**SEAM1 is offered only to arms 97 and 2** — both in row 2, both reaching down to
y ≈ 2.28 — and is never offered to 31 or 71, who reach up to y = 2.56. The strip
`y ∈ [2.220, 2.40]` is therefore below everyone the pattern invites. For the
CSAIL logo, which sits at `y ∈ [0.78, 2.65]`, that costs **0.868 m of ink (5.2 %)
and stroke 17 entirely** — 0.429 m that no stage can draw.

Adding two more seam stages that offer SEAM1 to arms 31 and 71 takes the block
from 97.69 % to **100.00 %** and the CSAIL logo from 94.84 % to 100 % (at 54
pieces instead of 50). **That is now the shipped pattern.**
`scripts/workcell_envelopes.py` answered the clearance question it raised
(V2_WORKCELLS §4b): 13 on SEAM0 against 31 on SEAM1 is **+194.3 mm** and 17
against 71 is **+201.1 mm**, both comfortably past the 50 mm gate; the tempting
alternative of crossing 31 and 71 over the two seams to stay at six stages is
−163.1 / −201.5 mm, because a transverse pair is unseparable in y as well as in
x. `zigzag_pattern()` has returned the eight-stage pattern since commit
`19bd490`'s successor, and every number in this document is that pattern's.

## 6. Open questions

**How should load balance enter — tie-break or weight?** Measured, and the
answer is **tie-break**, comfortably. The DP's value is a lexicographic pair,
`(pieces + balance_w × imbalance, imbalance)`; with the default `balance_w = 0`
the first component *is* the piece count and the second breaks its ties toward
the least-loaded state. Free balance, no pieces given up:

| set | pattern | no balance | **tie-break (default)** | weight w = 100 |
|---|---|---|---|---|
| strokes | single | 1 254 pcs, spread 29.4 % | **1 254 pcs, spread 10.0 %** | 1 255 pcs, spread 9.8 % |
| scribble | single | 1 024 pcs, spread 79.1 % | **1 024 pcs, spread 7.5 %** | 1 024 pcs, spread 7.5 % |
| hatch | single | 1 869 pcs, spread 20.4 % | **1 869 pcs, spread 21.1 %** | 2 011 pcs, spread 13.0 % |
| hatch | zigzag | 2 170 pcs, spread 24.1 % | **2 170 pcs, spread 15.5 %** | 2 224 pcs, spread 11.5 % |

(The zigzag row is the eight-stage pattern, re-run 2026-09-11; the six-stage
figures were 2 129 pcs at 32.4 / 32.4 / 29.1 %. The correction is worth 16.9
points of free balance on the staged hatch, because the two extra stages give
the middle arms somewhere to put ink that the free tie-break can then use.)

(spread = max − min arm load, as a percentage of the mean.) Where the capability
sets leave slack — scribbles and scattered strokes, where many atoms genuinely
have a choice — the free tie-break takes **all** of it: 79 % spread down to 7.5 %
at zero cost. Where they do not — the hatch, where every line crosses the whole
paper and the split is forced — the tie-break buys nothing and a real weight buys
balance at a bad exchange rate: 142 extra pieces for 18.6 m of spread, about
**one piece per 13 cm**. A piece is a pen-up, a transit and a seam; 13 cm of
imbalance is not worth one. **Recommendation: leave `balance_w = 0` and keep the
tie-break.** The lever exists if a stage ever turns out to be makespan-bound
rather than piece-bound, which is a thing `sequence.py` can measure and this
module cannot.

The cross-line coupling is otherwise one line: lines are visited **longest
first**, so the biggest commitments are made while the load table is still flat.
That is a heuristic, it is the only one in the module, and it is worth someone
checking against a proper global assignment before anyone quotes the load table
as a plan.

**How do the pieces feed `sequence.py`?** A piece is `(stage, arm, polyline)`,
which is exactly the shape `sequence.cost_matrix` / `solve` want — one tour per
(stage, arm) bucket, `n` pieces, the same `segs` contract `allocate` hands it
today. Three things are not answered:

1. **Ordering across stages is a barrier, not a tour.** Each (stage, arm) bucket
   is an independent TSP, and the barrier between stages serialises them. A
   piece that could be drawn in stage 0 *or* stage 2 by the same arm is a
   scheduling degree of freedom the DP currently spends on "fewest pieces" and
   nothing else — see the 301 same-arm hand-overs in the strokes set.
2. **Pen-up legs between pieces are not priced here.** The DP minimises pieces,
   not transit; two pieces of one line that are far apart in the tour cost more
   than the piece count says. `sequence.price_crossings` is the thing that
   knows, and it runs after this module, not inside it.
3. **`plan_stroke` still has to accept each piece.** Every number on this page is
   what the 2 cm atlas permits. A piece the planner refuses splits, and the
   honest way to close the loop is to run the pieces through `plan_stroke` and
   re-derive the capability map from the refusals.

**Should the capability map be eroded?** A continuous stroke sample rounds to its
nearest 2 cm cell and can therefore sit up to 14 mm outside the region that cell
certifies — which is why `dead_spans.go_cells` erodes by one cell. `traces` can
(`--erode`) and by default does not, matching `certified_area` and
`workcell_envelopes`. Eroding makes every number here more conservative and none
of them wrong; whether a hand-over should be *claimed* on an un-eroded map is a
question for whoever signs off the seam.

**Does the piece count survive the permissive gate?** It survives everything
tried. On the shipped gated-63 atlas under `allocate.atlas_cells`' reading at
**zero** permitted lean (`--gate flat --tilt-max-deg 0`) the logo goes to 45
pieces unstaged and 55 staged with *less* coverage (97.0 % / 92.7 %) — that gate
refuses cells `strict_go` certifies *with* a lean, and at `tilt_max_deg = 0` it
cannot take them back. Restore v19's own `tilt_max_deg = 15` and it returns to
40 / 50. The two gates are not ordered, and the honest reading is that the piece
count is set by the *shape* of the coverage boundary, not by where exactly the
boundary sits. Strict is quoted above because it is the gate V2_WORKCELLS'
clearances were measured under.

## 7. What is tested

`tests/test_traces.py`, 31 tests, no environment variables and no atlas — every
capability map is rasterised from rectangles through the same `Coverage` object
the atlas produces, so a re-sweep can never turn one red for a reason that is
not a bug.

- **DP optimality against brute force** on every capability chain a small
  alphabet admits: 2–4 states × 4–6 atoms, 15⁴ … 15⁶ chains each, including
  chains with undrawable atoms. The result must equal the exhaustive minimum
  *and* be a legal assignment.
- A line fully inside one arm's cell is **one piece**, one atom, no seam — both
  on a toy map and on a real row band.
- A crossing in an overlap zone is **exactly one hand-over**, the seam is at the
  middle of the overlap, each side laps δ, and the lap never leaves the zone (a
  6 mm overlap yields a 6 mm lap).
- A hard-edge crossing is **two pieces meeting exactly**, at the transition point
  to 2 × 10⁻⁵ m and not at the 0.5 mm sample, with no ink added or lost.
- A line through a dead band with no main-stage drawer is **deferred to a seam
  stage**: five pieces, stages ≥ 2 in the dead bands and ≤ 1 either side, each
  dead-band piece 0.40 m long and every crossing a hard edge.
- Absorption never raises the piece count (400 random chains) and keeps the
  assignment legal; a short piece no neighbour can draw is kept.
- The summary adds up: loads sum to drawn metres, drawn = ink + 2δ per overlap
  hand-over, and staging can only cost pieces, never save them.
