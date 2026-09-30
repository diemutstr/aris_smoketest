# Can sliding arm 2 along its pole kill the dead zone?

**Short answer: not by sliding. The plate is already at the bottom stop.**

Arm 2 (side, green) is the only arm in the final rig with a free mounting
parameter: it hangs off a vertical 2 × 3″ T-slot pole and could in principle be
re-clamped anywhere along it. This note sweeps that parameter and reports what
the paper gains.

The finding in one line: **the coverage gain is all in the DOWN direction, and
down is exactly the direction where the pole has no material left** — the base
plate's top edge is flush with the pole's own lower end. Down travel is
0.00–2.70 cm. Up travel is 65 cm and every centimetre of it is worse.

Sources: `scripts/arm2_height_sweep.py` (the sweep), `out/arm2_height_sweep.png`
(the figure), `out/arm2_height_sweep.json` (every number below),
`out/arm2_sweep/dz*/` (the per-height atlases). Baseline is
`out/atlas_final/coverage.npz` — 2 cm grid, tilt ≤ 15°, pen 0.110, strict-GO =
`margin ≥ 0.30 ∧ σ_min ≥ 0.14`, frame boxes active, 7826 sheet cells.

## 1. What is actually dead near arm 2

Of the sheet's 7826 cells, **2455 (31.4 %, 0.982 m²) are strict-GO by nobody**.
Partitioning them by nearest arm base, arm 2's region owns **1121 of them
(0.448 m², 45.7 % of all dead paper)** — the largest share of the three.

Two named features carry almost all of it:

| feature | cells | area | what it is |
|---|---|---|---|
| **right rod strip**, x ≥ 1.66 | 578 | 0.231 m² | 84 % of that strip is dead. Guide rods (canvas x 1.68–1.94) + the paper web rising 2.9 cm over them (`paper_curl`, x ≥ 1.80). **Not a reach problem** — it is clearance against the transport. |
| **arm-2 under-shoulder hole** | 412 | 0.165 m² | a disc r ≤ 0.34 m around the J2 axis. |
| *together* | **990** | **0.396 m²** | **40.3 % of all dead paper** |

The hole is **not under the base plate**. Arm 2's J1 axis is horizontal along
−X, so its shoulder (J2 axis) sits 0.333 m in −X of the plate, at canvas
**(1.2768, 1.2572)** — the plate is at (1.6098, 1.2572). Everything below is
measured from the shoulder.

Radial profile of arm-2 strict-GO around that shoulder, as drawn:

| ring r (m) | 0.0–0.1 | 0.1–0.2 | 0.2–0.3 | 0.3–0.4 | 0.4–0.5 | 0.5–0.6 | 0.6–0.7 | ≥ 0.7 |
|---|---|---|---|---|---|---|---|---|
| % strict-GO | 76 | **24** | 43 | 74 | 58 | 40 | 12 | **0** |

So there are *two* failures, with different causes:

- an **inner annulus at r ≈ 0.10–0.22** where the arm must fold up under
  itself and the margin/σ gates bite — a wrist-fold problem;
- an **outer cliff at r ≈ 0.64** where the arm simply runs out of stretch,
  because 0.776 m of the 0.855 m reach is spent going straight down before any
  of it goes sideways.

Sliding along z can only ever move the second one. It cannot move the first,
because sliding the base along z **does not move the shoulder in xy at all**.

## 2. The feasible slide range, and where it comes from

Re-derived independently from the binary DXF (ezdxf + ACIS decode of all 646
3DSOLIDs; world transform derived from scratch, floor at exactly z = 0). The
result reproduces `rig_final.FRAME_BOXES_W_CM` to the digit and adds the piece
the box model does not carry: **which part grips the pole.**

W-frame cm, X 187.31–194.95, Y 144.83–160.07 — the pole is 2 × 3″×3″ alu
profile stacked in Y (the drawing's own MTEXT: *"1 robot hanging on its side on
a boom pole (2 x 3x3")"*):

| copy | pole z | length |
|---|---|---|
| FRONT (= what the PDF prints) | 155.0675 → 226.0675 | 71.00 cm |
| TOP (= the same part as the four arm-31 boom beams) | 152.3675 → 226.0275 | 73.66 cm = **29.00 in** |

These 4 solids are **the only geometry in the whole drawing where the two model
copies disagree** (313 of 315 top-copy solids match the front copy to < 0.05
cm). This is FINAL_RIG Flags #1, now with evidence both ways: the top-copy
reading makes the pole an exact 29-inch part identical to the arm-31 booms; the
front-copy reading is what the printed front view actually draws (verified by
pixel-measuring the PDF: the pole's edge line is present at z 155.5 and gone at
z 154.5) but is a round *metric* 71.00 cm in an all-imperial rig.

The arm's stack, identical in both copies:

| piece | z (W cm) | grips the pole? |
|---|---|---|
| base plate, 1.27 thick | 132.4852 → 155.0675 | no — sits at x 182.23–183.50, inboard of the pole |
| lower spacer block, 3.81 | 132.4852 → 139.6165 | **no — hangs in air, 13 cm below the pole's end** |
| upper spacer block, 3.81 | 147.9362 → 155.0675 | only on the top-copy reading, and then by 2.70 cm |
| **bracket, 2 halves** | **155.0675 → 160.8338** | **yes — this is the whole joint, 5.77 cm of it** |

**This is the drawing's own warning made geometric.** MTEXT: *"Arm 2 side
position: conection base plate to boom pole is not stiff and stable."* The arm
hangs off the *end* of the pole on one 5.77 cm bracket, with two spacer blocks
bolted to nothing and the plate top exactly flush with where the pole stops.

Taking "the bracket must sit fully on the pole" as the criterion:

| | down | up |
|---|---|---|
| front-copy pole | **0.00 cm** | +65.19 cm |
| top-copy pole | **+2.70 cm** | +67.89 cm |

Up is genuinely open — the −X mounting face was swept in the DXF over
z 160.84 → 226.03 across the full plate footprint and hits nothing; both
gussets are on the +X face. The binding constraint upward is the central double
top beam underside at z 226.0275. **The sweep below assumes the top-copy
bottom (152.3675), i.e. the union already in `rig_final`, because it is the
more generous of the two.**

One more measurement matters for the recommendation: the volume the pole
*would* occupy if lengthened downward — X 187.33–194.95, Y 144.83–160.07,
z 63.67 → 155.07 — is **completely empty, all the way to the tabletop.**

## 3. The sweep

`scripts/arm2_height_sweep.py --neighbours 999`. Per height it builds a
modified `ArmSpec` (explicit z shifted; xy and R untouched) **and slides arm 2's
own clamped hardware with it** — `side_plate`, `side_clamps`, `side_bracket` —
so the other two arms keep seeing honest obstacles. The pole (`side_boom`) and
its top brace (`side_gusset`) are structure and stay put. Then it re-runs the
arm-2 atlas at 2 cm with the current gates and frame obstacles active.

dz > 0 is up the pole. Baseline is dz = 0, union 68.63 %.

| dz (cm) | canvas z | extra pole needed | arm-2 GO | % sheet | dead cells recovered | union GO % | ≥2 arms % | newly dead |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| −26 | 0.516 | +23.3 | 2939 | 37.55 | 468 | 73.13 | 18.94 | 116 |
| −24 | 0.536 | +21.3 | 2984 | 38.13 | 524 | 73.95 | 18.71 | 108 |
| −22 | 0.556 | +19.3 | 3011 | 38.47 | 558 | 74.55 | 18.45 | 95 |
| **−20** | **0.576** | **+17.3** | **3035** | **38.78** | **587** | **75.12** | **18.23** | **79** |
| −18 | 0.596 | +15.3 | 2945 | 37.63 | 553 | 74.67 | 17.56 | 80 |
| −16 | 0.616 | +13.3 | 2813 | 35.94 | 513 | 73.97 | 16.61 | 95 |
| −14 | 0.636 | +11.3 | 2653 | 33.90 | 459 | 73.18 | 15.36 | 103 |
| −12 | 0.656 | +9.3 | 2465 | 31.50 | 404 | 72.48 | 13.69 | 103 |
| −10 | 0.676 | +7.3 | 2286 | 29.21 | 343 | 71.56 | 12.36 | 114 |
| −8 | 0.696 | +5.3 | 2112 | 26.99 | 269 | 70.71 | 11.00 | 106 |
| −6 | 0.716 | +3.3 | 1938 | 24.76 | 197 | 69.96 | 9.55 | 93 |
| −4 | 0.736 | +1.3 | 1734 | 22.16 | 146 | 69.47 | 7.44 | 80 |
| −3 | 0.746 | +0.3 | 1640 | 20.96 | 116 | 69.32 | 6.39 | 62 |
| **−2** | **0.756** | **— (fits)** | 1565 | 20.00 | 84 | **69.15** | 5.60 | 43 |
| −1 | 0.766 | — | 1471 | 18.80 | 45 | 68.91 | 4.64 | 23 |
| **0** | **0.776** | — *(as drawn)* | **1408** | **17.99** | 0 | **68.63** | **4.11** | 0 |
| +1 | 0.786 | — | 1336 | 17.07 | 17 | 68.21 | 3.62 | 50 |
| +2 | 0.796 | — | 1306 | 16.69 | 31 | 67.91 | 3.53 | 87 |
| +4 | 0.816 | — | 1282 | 16.38 | 70 | 67.34 | 3.80 | 171 |
| +6 | 0.836 | — | 1212 | 15.49 | 85 | 66.06 | 4.18 | 286 |
| +8 | 0.856 | — | 1092 | 13.95 | 104 | 64.58 | 4.13 | 421 |
| +12 | 0.896 | — | 747 | 9.55 | 162 | 61.23 | 3.07 | 741 |

Monotone: every centimetre down helps until −20, every centimetre up hurts.
(Up does "recover" a handful of dead cells — it pushes the outer GO edge
outward on the far side — but it loses far more than it wins, which is why
`newly dead` explodes: 741 cells at +12.)

Union % here holds arms 13 and 31 at their baseline masks; §5 confirms that is
exact.

## 4. Recommendation

**Do not slide. Lengthen the pole by 20 cm and re-clamp at canvas z = 0.576.**

- **Best without touching the pole: dz = −2 cm** (canvas z 0.756). Union
  68.63 → **69.15 %**; arm-2 GO 1408 → 1565 cells; 84 dead cells recovered,
  43 newly dead — a net 41 cells = 0.016 m². That is inside the noise of a
  mount the drawing itself calls not stiff, and it spends the last 2.7 cm of
  bracket engagement to get it. **Not worth doing.** On the front-copy reading
  of the pole it is not even available.
- **Best overall: dz = −20 cm** (canvas z **0.576**, J1 axis z_W **121.268**).
  Union strict-GO **68.63 → 75.12 %** (+6.5 points, 5371 → 5879 cells).
  Arm-2 strict-GO **17.99 → 38.78 %** of the sheet (1408 → 3035 cells,
  0.563 → 1.214 m²). Cells reachable by **≥ 2 arms 4.11 → 18.23 %** — a 4.4×
  increase in handoff/parallelism capacity, which is the bigger operational
  prize. **587 currently-dead cells (0.235 m²) recovered**, 79 newly dead
  (0.032 m²), net −508 dead cells; total dead paper 31.4 → 24.9 %.
  The peak is a broad plateau — −18 gives 74.67 %, −22 gives 74.55 % — so
  ±2 cm of build tolerance costs under half a point.

Cost: the pole must reach down to z_W 135.07. That is **17.3 cm below the
top-copy bottom, 20.0 cm below the printed one — order 20 cm of extra
profile**, into a volume verified empty for another 71 cm below that.

New pose if adopted (nothing in `rig_final.py` has been changed):

| | value |
|---|---|
| J1 axis, W cm | (182.230, 152.471, **121.268**) |
| base, canvas m | (1.60984, 1.25723, **0.576**) |
| base plate, W z | 112.485 → 135.068 |
| bracket, W z | 135.068 → 140.834 |
| pole, W z | 152.3675 → 226.0275 must become **≈ 132 → 226** |

**The stiffness argument points the same way, and is the real reason to do
this.** Today the joint is one 5.77 cm bracket on the end of the pole, with two
spacer blocks bolted to air. Extend the pole past the whole stack and all three
pieces land on live profile — roughly 28 cm of engagement instead of 5.8 —
which is the direct answer to the drawing's *"not stiff and stable"*. Moving
the arm is the moment to fix the clamp; the two changes are the same job.
Until it is stiffened **and** re-surveyed under load, keep
`rig_final.CALIB_STATIC` at 0.03 for this arm regardless of height (Flags #2).

## 5. What still does not work, and why

At dz = −20 the residual dead paper is 24.9 % (1947 cells). Accounting,
as drawn → at the optimum:

| | as drawn | at z 0.576 | Δ |
|---|---:|---:|---:|
| left strip x ≤ 0.12 (feed roll + arm-31 boom) | 569 | 569 | 0 |
| **right strip x ≥ 1.66 (guide rods + paper curl)** | 578 | **493** | −85 |
| **arm-2 under-shoulder hole** | 412 | **181** | **−231** |
| arm-31 under-shoulder hole | 397 | 397 | 0 |
| arm-13 under-shoulder arc | 160 | 160 | 0 |
| mid-sheet, between the lobes | 306 | 133 | −173 |
| sheet edges | 33 | 14 | −19 |

Two things survive on purpose:

- **The right rod strip stays 493 cells dead.** It is not a reach failure and
  never was — it is `guide_rods` + `paper_curl` clearance. No base height
  fixes it. Only moving the transport, or accepting a narrower drawable web,
  would.
- **The under-shoulder hole shrinks but does not close.** The radial profile
  at z 0.576 reads 71 / 32 / 100 / 99 / 92 / 81 / 84 / 56 / 6 % for rings
  0.0–0.1 … 0.8–0.9. The outer cliff moves from r ≈ 0.64 to r ≈ 0.82 (the
  20 cm of drop converts almost 1:1 into sideways reach, exactly as expected),
  and the mid-annulus fills in. But **the inner annulus at r ≈ 0.10–0.16 is
  still there**, because it is a wrist-fold/σ problem at a shoulder that
  sliding along z never moves. Killing it needs a different lever: relaxing
  the tilt cone, moving the plate in **xy**, or letting arm 13/31 cover it.

Note the direction of the trade: 587 recovered vs 3035 − 1408 = 1627 extra
arm-2 GO cells. Most of the new arm-2 capability lands on paper somebody
already covered — which is precisely why the ≥2-arm number quadruples.

## 6. Neighbour check, and how far to trust this

**Arms 13 and 31 are unaffected.** Re-running both atlases at dz = −20 with
arm 2's slid hardware in their obstacle sets: arm 13 strict-GO **2276 → 2276**
(0 lost, 0 gained), arm 31 **2009 → 2009** (0 lost, 0 gained). This is
structural, not luck: the sliding boxes sit at canvas x 1.607–1.673,
y 1.16–1.33, which is 1.56 m from arm 13's base and 1.36 m from arm 31's —
both well beyond a Franka's 0.855 m + pen. Union figures in §3 are therefore
exact, not an approximation.

Checks run alongside:

- **The box patch really does gate the sweep.** Inflating `side_plate` to a
  full-table slab at canvas z 0.063–0.263 takes arm 13 from 629 reachable /
  575 strict-GO (4 cm grid) to **0 / 0**. The obstacle list each arm receives
  was also printed directly: arms 13 and 31 see the slid boxes at their new z;
  arm 2 sees none of the `side_*` boxes, because they are all tagged
  `mount:side` and excluded from its own check (`fleet.ArmSpec.static_obstacles`).
- **Arm 2 never approaches its own pole**, which that exclusion would hide.
  Recomputing FK from every stored strict-GO pose and measuring the chain
  against the `side_boom` box: min clearance **0.309 m** as drawn and
  **0.376 m** at z 0.576 (`STATIC_MARGIN` is 0.05). Lowering the arm moves it
  *away* from the pole.

Limits of this study:

1. It is a **per-cell static IK/gate atlas**, not a trajectory study. A cell
   turning strict-GO means a good pose exists there, not that a stroke can be
   drawn through it. The planner/scheduler must be re-run before any of this
   is promised to a drawing.
2. **Inter-arm proximity goes up a lot.** 18.2 % of the sheet becomes dual-arm
   (from 4.1 %), and arm 2's lobe now reaches to canvas x ≈ 0.46, deep into
   arm 31's territory. That is the point — but `coordination.py` and the
   concurrency diagnostic have not been re-run at this height, and they
   should be before adopting it.
3. The **pole-bottom ambiguity (Flags #1) is still open** and is worth
   resolving with the fabricator anyway, since it is the difference between
   0.00 and 2.70 cm of free travel. It does not change the recommendation:
   both readings say the same thing, that the arm is at the bottom stop.
4. `rig_final.py` is **unchanged**. Adopting this means editing
   `ARM_MOUNTS_W["side"]["p_w_cm"]` z, the `side_plate` / `side_clamps` /
   `side_bracket` boxes, and lengthening `side_boom` — then regenerating the
   URDF and re-running `scripts/run_atlas.py`.

## ADOPTED, 2026-08-21 — and re-measured, because this study did not model the pole

The user took the recommendation for **both** side arms (2 in unit A, 97 in
unit B by mirror symmetry). It lives as `rig_final6.FLEET_FINAL6_OPT` +
`rig_final6.FRAME_BOXES6_OPT_W_CM`, installed by `ARIS_RIG=final6_opt` — and
`rig_final.py`, the drawing, is still not modified.

**The numbers in §3 are not the numbers of the adopted build.** This sweep slid
the three clamped boxes and left `side_boom` at its drawn length, so the bracket
hung 20 cm past the end of a pole that was not there — which is exactly the
thing that cannot be built. The adopted build lengthens `side_boom` downward by
20 cm, which is 20 cm of new obstacle for every other arm, and it was re-swept
from scratch on the merged six-arm canvas (`scripts/run_atlas6.py`,
`docs/MERGED_CANVAS.md`):

- the extra profile costs the other four arms **exactly zero cells** (13, 31,
  17 and 71 are cell-for-cell identical with the pole and without it), which
  extends §6's neighbour result to the longer pole;
- on the continuous 1.8034 × 3.63064 m canvas the move takes union strict-GO
  **69.13 → 75.93 %**, cells reachable by ≥ 2 arms **7.86 → 25.47 %**, and each
  side arm **9.89 → ~22.4 %** of the whole canvas (17.9 → 38.8 % of its own web,
  which reproduces this study's 18.0 → 38.8 % on the single web);
- and it does the thing a single web could not be asked about at all: the seam
  strip goes **77.47 → 88.00 %** GO with cross-unit coverage **41.30 → 74.91 %**.

Also new, and needed: `frames.Q_READY_WALL_LOW`. The drawn ready pose puts the
pen **0.100 m below the paper** once the base drops 0.20 m.

## Reproduce

```
scripts/arm2_height_sweep.py --neighbours 999    # ~2.5 min on 32 cores
scripts/arm2_height_sweep.py --replot            # redraw from cached atlases
```
