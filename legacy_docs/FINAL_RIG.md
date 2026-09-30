# FINAL RIG — the 3-arm installation, extracted from the authoritative drawing

Everything in this document is traceable to the two files the user named as THE
final setup (`raw_slack_file_dump/`):

- **PDF** `Drawing installation, 3 arms, 1 up, 1 side, 1 down, sizes cm.pdf` —
  1 page, Vectorworks export (author "Stefan Strauss"), front view + top view +
  text block. Dimensions in **cm**.
- **DXF** `D.I., 3 arms, 1 up, 1 side, 1 down, sizes cm.dxf` — AC1032,
  `$INSUNITS=5` (centimeters, verified against the 218,4 dimension),
  `$DIMLFAC=1`. The modelspace holds **two complete 3D copies** of the
  installation (a front copy and a rotated top copy); the front copy is the
  authoritative geometry, the top copy cross-checks it (agreement < 0.02 cm
  everywhere except one flagged item, see Flags #1).

Extraction: visual PDF read page-by-page + programmatic DXF parse (ezdxf;
layers `0` / `Design Layer-3` / `Defpoints`; 166 top-level 3DSOLID + 87 block
inserts with 175 more solids, 11 MTEXT, 11 DIMENSION — none text-overridden,
so displayed values ARE measured values rounded to one decimal). All 11
dimension annotations reconcile with the model geometry to ≤ 0.05 cm.

The machine-readable form of every number below is
**`aris_sixarm/rig_final.py`** — the single source the URDF generator, the
fleet registry and the collision model all read. This file is the provenance
record; that file is the data.

## The arms are named in the drawing

MTEXT: *"Three robot arms: 13 floor, 31 left - upside down, 2 right side
position"* — the same physical arms as the legacy six-arm registry. The final
fleet therefore reuses ids **13** (upright), **31** (inverted), **2** (side).
Notably, the legacy registry always listed arm 2 as a wall-plate mount ("
revisit if arm 2 goes active as a wall arm" — it has).

## Frames

- **W** ("world", the drawing): origin = outer front-left corner of the
  installation at floor level (underside of the leveling feet), +X right,
  +Y back, +Z up. Units cm. Envelope: X 0→218.44, Y 0→208.28, Z 0→233.65.
- **C** ("canvas", the planning frame): origin = front-left corner of the
  **paper top surface**, axes parallel to W, units m, z=0 the paper plane —
  the identical convention every planner module already used.
  `C = (W − (21.246, 26.748, 63.668)) / 100`.

## Dimension inventory (PDF annotation ↔ DXF measurement)

| Annotation | DXF measured | What it measures (W frame) |
|---|---|---|
| 218,4 (both views) | 218.440 | outer width, X 0→218.44 |
| 208,3 ("entire table") | 208.280 | outer depth, Y 0→208.28 |
| 233,7 height construction | 233.700 (dim) / 233.648 (structure) | floor→top beams; the dim's lower defpoint sits 0.052 below the floor |
| 63,5 table height | 63.468 | floor→tabletop top |
| 91,6 "backside base plate to surface (drawing paper)" | 91.600 | arm-2 plate TOP edge (z=155.068) → **tabletop** (63.468); to the paper top it is 91.40 — see Flags #3 |
| 45,7 | 45.737 | arm-31 J1 axis → left outer face |
| 55,8 (arm 31) | 55.854 | arm-31 J1 axis → back outer face |
| 55,8 (top view / arm 2) | 55.829/55.809 | back outer face → arm-2 boom centerline / plate center Y |
| 34,9 (both views) | 34.940 | arm-2 plate backside (x=183.500) → right outer face |
| 22,6 × 19,0 | 22.582 × 19.000 × 1.27 thick | all three base plates |
| 13,8 × 9,5 | 13.806 / 9.500 | J1 axis offset from plate edges (9.5 = centered on the 19 side; 13.8/8.78 = NOT centered on the 22.6 side) |
| 3″×3″ = 7,62 | 7.620 | every beam / post / leg cross-section |

Other exact plate thicknesses: tabletop 2.54 (1″), gusset/brace plates 3.81
(1.5″), gussets 20.32×20.32 (8″), device plates 24.13 (9.5″).

## Drawing surface

A continuous **paper web** feeding from a roll at the left edge, lying flat on
the tabletop, curling up at the right edge to guide rods and a winder:

- flat drawable region **180.34 × 170.00 cm**, W: X 21.246→201.586,
  Y 26.748→196.748, top face **z = 63.668** (paper drawn 0.2 thick on the
  63.468 tabletop; the 2 mm is modeler convenience — the top face is the datum)
- canvas frame: sheet `SHEET_FINAL = (1.8034, 1.700) m`, z = 0, paper center
  C (0.9017, 0.8500, 0), W (111.42, 111.75, 63.668) cm
- provenance: paper solid 40 (front copy) exact; roll/curl solids 41–47

## The three arm mounts

All three base plates are 22.582 × 19.0 × 1.27 cm; the J1 axis sits 13.8 cm
from one 19-edge (8.78 from the other) and centered (9.5/9.5) on the 22.6
sides. Base "front" (+X of link0) = the 8.78-offset side — an inference from
the Franka base asymmetry plus all three depicted poses (Flags #6).

| arm | mount | J1 axis, W cm | mounting plane | base orientation (quat xyzw, W) | canvas pose, m |
|---|---|---|---|---|---|
| **13** | upright on the tabletop, front strip, in front of the paper | (109.289, 14.068) | z = 64.738 (plate top), axis +Z | (0, 0, 0.70711, 0.70711) = rotz(π/2); front faces +Y, toward the paper | (0.88043, −0.12680, 0.01070) |
| **31** | inverted, hanging under the central double top beam | (45.737, 152.426) | z = 155.868 (plate BOTTOM), axis −Z | (1, 0, 0, 0) = rotx(π); front faces +X, toward table center | (0.24491, 1.25678, 0.92200) |
| **2** | side: vertical plate clamped to the right boom, J1 axis horizontal | Y 152.471, z 141.268 | x = 182.230 (plate left face), axis −X | (0.70711, 0, −0.70711, 0) = roty(−π/2)·rotz(π); front faces −Z, straight down | (1.60984, 1.25723, 0.77600) |

Derived quantities the planner cares about:

- arm 31 mounting plane is **92.20 cm above the paper** (92.40 above the
  tabletop) — within 2 mm of the legacy rig's measured `h_inv = 0.924`.
- arm 2 J1 axis is **77.60 cm above the paper**, pointing at the table center.
- arm 13's plate top is 1.07 cm above the paper plane; the paper's front edge
  is 12.68 cm in +Y from its J1 axis.
- arms 31 and 2 both hang on the central-double-beam centerline Y = 152.45
  (axes at Y 152.426 / 152.471, within 0.03).

Provenance per arm: arm 31 — dims 45,7 + 55,8, plate solid 83, axis marker
solid 86, ring solid Groep-57. Arm 2 — dims 34,9 (×2), 55,8, 91,6, plate solid
Groep-72/143. Arm 13 — **no dimension annotation references it**; position is
model geometry only (plate solid Groep-66/137 + 13.8 marker, identical in both
DXF copies) — Flags #5.

## Structure / collision model

`rig_final.FRAME_BOXES_W_CM` — 35 conservative AABBs that provably enclose
every drawn member. Key modeling choices:

- **table_block**: everything below the tabletop (legs, feet, top+bottom rail
  levels, 45° corner braces, tabletop plate) as ONE enclosing box
  (218.44 × 208.28 × 63.468). Planning drops it (`zmin=0.0`): the
  z ≥ `Z_PAPER` plane gate already forbids the whole below-paper half-space,
  and a slab 2 mm under the pen would veto every legitimate drawing pose.
- **top_slab**: all top beams (perimeter + central double beam at
  Y 144.83–160.07) + connector plates as one slab z 226.03→233.65
  (canvas z 1.624→1.700 — above any reachable pose, kept anyway).
- **arm-31 boom**: two vertical beam-pair boxes flanking the plate
  (X 23.53–31.15 and 55.28–62.90, canvas x 0.023–0.099 / 0.340–0.417,
  z down to 152.37 = 3.5 cm BELOW the mounting plane) + gussets + clamp
  stack. These replace the legacy `BOOM_R=0.12` cylinder proxy for this arm.
- **arm-2 boom**: vertical beam pair X 187.31–194.93 (canvas x 1.661–1.737,
  directly behind the mounting plate) + clamp blocks straddling the axis
  height + gussets. Bottom z taken as the **union** of the two DXF copies
  (Flags #1).
- **paper transport**: the feed roll (Ø≈20.2 + R 7.0 end flanges, canvas
  x −0.214→+0.001, z −0.012→0.194) sits **flush against the paper's left
  edge** — it, not reach, bounds the drawable strip there. Guide rods /
  winder / crank sit ≥ 0.09 m beyond the right edge; the **rising paper web
  itself** (up to 2.9 cm over the rods) is boxed as `paper_curl`.
- **8 hung devices** (cameras or lights — purpose not stated) under the top
  beams, canvas z ≥ 1.53.

Every box carries its source string in `rig_final.py`. Below-paper members
are enclosed but excluded from planning; everything above the paper plane is
in exactly one box (verified independently, see Verification).

## Pen holder / tool

Source: `Pen holder cad(1).zip` — all native SolidWorks (no neutral formats;
the SW-2024/25 container was reverse-engineered to tessellations + assembly
transforms; meters confirmed by the pencil's own 174.6 mm and the base
plate's 226×190 drawing callouts).

**The holder is CLAMPED BETWEEN THE FRANKA HAND'S FINGERS** — the hand is
present and stock (flange→hand chain unchanged, so `TCP_D` stays the solver
convention), the fingertips are custom/drilled, finger half-width **28.5 mm**
(URDF fingers fixed there). The zip holds TWO rig generations:

| build | status | flange→tip (panda_hand frame) | pen axis |
|---|---|---|---|
| **10° "natural hold"** (assembly complete, Mar 2026) | fully determined | **(−8.0, 0.0, +148.7) mm** = TCP + (−8.0, 0, +45.3) mm | tilted **10.0°** about y_hand |
| **23° clutch** (newest parts, no assembly file) | protrusion ADJUSTABLE — tip **not determined by CAD** | at the 10° build's 21 mm protrusion: (−18.0, 0, 145.7) mm; to reach the upstream 0.209 m: ~90 mm protrusion → **(−44.9, 0, 209.0) mm, 45 mm off-axis** | tilted 23.0° (file says "22 deg" — flagged) |

**Neither build reproduces the scalar tool model** (`PEN_EXT` = 0.110 below
TCP = 213.4 mm from the flange, on-axis). The INLINE planning default REMAINS
the gate-validated real-touchdown value — a measurement outranks a CAD whose
deployed configuration is unconfirmed — and the CAD's own numbers are pinned
alongside it in `frames.py` (`TIP_HAND_HOLDER10`, `PEN_TILT_HOLDER10`,
`PEN_EXT_HOLDER10`).  The LATERAL holder's own pair moved on 2026-09-03 to
`PEN_EXT_HOLDER` / `PEN_LAT_HOLDER` = 0.0460262 / 0.0860369 m from a photo of
the real gripper — grip at the far end of the Fat blades, bore at the
housing's own 23°, 20 mm of graphite past the cap.  That pair never was
gate-validated (docs/SYSTEM_MODEL.md §7e).

Collision: the final-rig pen capsule and the URDF tool cylinder use the
**union envelope of both builds** — r = 0.05 m, z −0.033…+0.210 in the hand
frame (`rig_final.PEN_R_FINAL`, `rig_final.TOOL`) — which covers the 10°
housing (max radius 30 mm), the clutch extended to 0.209 m (45 mm off-axis +
pencil), and the sharpened-pencil butt behind the flange plane. Visual: the
CAD-extracted mesh `assets/final_rig/meshes/penholder_rig10_panda_hand_frame.stl`
on each hand.

Tool flags (also in Flags below): which build ships is UNCONFIRMED (newest
files say 23° clutch + "Fat Franka Finger", whose cradle geometry is not
fully resolved); clutch protrusion is a per-pencil setting AND wears with the
consumable Conté à Paris pencil; a full-length pencil interferes with the
wrist (butt 23 mm behind the flange plane) — pencils must be sharpened
shorter.

**Amended 2026-09-02** — the assembly was re-read from scratch for
`docs/SYSTEM_MODEL.md` §7a, and two of the rows above need correcting:

- **There is no cradle.** `Fat Franka Finger v250904` is an 18.4 × 90 × 50 mm
  blade that replaces the whole stock finger, not a cradle; and in the 10°
  assembly the fingertips are stock `Franka_Finger_FR3` tips, each drilled for
  one `93514A130` brass insert, seating **7.000 mm inside the mount post's own
  18 × 18 mm end sockets**. The jaw gap is **36.0008 mm**, not 57 — 57 adds
  the 3.5 mm of tip left outside each socket instead of subtracting the 7 mm
  inside it, and is 7 mm wider than the post is long. Nothing between holder
  and hand can absorb a rotation, so the housing's clocking **is** the pen's
  lean: 10.0000° here, 23.00° on the 23° build.
- **flange→tip is confirmed, to a tenth of a millimetre.** Re-derived from the
  assembly's own transforms, the pencil's sharp point sits **46.096 mm** from
  the grip centre at **(7.93, 0, 45.41) mm** — this table's TCP + (−8.0, 0,
  +45.3), with the sign of x a mounting choice rather than a disagreement (the
  post is square and seats either way up). The 46.1 is the cap end at 25.361
  plus **20.7 mm** of pencil past it, i.e. the "21 mm protrusion" above,
  measured. `PEN_TILT_HOLDER10` = 10° is exact: **10.0000°**.

**The 23° model in `rig_final.PENHOLDER22` had the housing end-for-end, and
that is FIXED (2026-09-03).** This table's 25 mm grip-to-cap-end was right and
`PENHOLDER22["nose_x"] = 0.0` was not: the pen leaves through the **cap**, and
x = 0 is the far end. The 17.0 mm land there is the **spring's** stop — 17.0
will not pass a 19.05 mm spring, which is exactly what it is cut to do — the
assembly's parts run cap → sleeve → spring → tail face along its bore, and the
preview shows the sharpened point through the cap with 72.5 mm of blunt pencil
tail out the other end. `penholder22_T_hand` now points the housing's +X at
the tip.

One book-keeping note, because two of this page's own numbers look like they
disagree with §7c and do not. The **20.7 mm** above is measured to the
*housing's* end face; this build's cap stands **3.734 mm** proud of it, so
past the cap's own outer face the pencil shows **17.000 mm** — which is what
the assembly's bounding boxes give directly, and it puts that grip 29.096 mm
from the cap face against the 23° build's 30.001.

`assets/final_rig/` is unaffected (it carries the CAD-extracted mesh, already
in hand coordinates). See `docs/SYSTEM_MODEL.md` §7c for what the correction
cost — no gate on the certified programme flipped, but the r = 0.050 tool
capsules no longer contain the holder.

The collision union is unaffected: it was sized on the housing's 30 mm max
radius and a 0.209 m reach, both of which still bound these numbers.

## Verification

An independent agent re-derived the geometry from the DXF/PDF with its OWN
SAB/ACIS parser (exact vertices + NURBS control-point hulls + rational
circles; scripts kept separate from the extraction's), then diffed
`rig_final.py`. Position tolerance 0.05 cm; box containment tolerance
0.01 cm, conservative direction only.

| quantity | independent derivation | rig_final.py | Δ (cm) | verdict |
|---|---|---|---|---|
| world-frame offset | DXF + (241.282, 148.140, 155.275) | same | 0 | PASS |
| arm 13 plate + J1 pose | (109.2886, 14.068, 64.7378), +Z, front +Y | (109.289, 14.068, 64.738) | ≤0.001 | PASS |
| arm 31 plate + J1 pose | dim 45.7377 / marker 152.429 / plate bottom 155.8678, −Z, front +X | (45.737, 152.426, 155.868) | ≤0.003 | PASS |
| arm 2 plate + J1 pose | face 182.2301 / 152.4711 / 141.2678 (ring 141.2786), −X, front −Z | (182.230, 152.471, 141.268) | ≤0.011 | PASS |
| all three base rotations | consistent with plate asymmetry + depicted chains | ARM_R_W | — | PASS |
| paper origin/size/top | (21.2463, 26.748, 63.6678); 180.340 × 170.000; tabletop 63.4678 | same | ≤0.0004 | PASS |
| arm IDs | MTEXT "13 floor, 31 left upside down, 2 right side" | 13/31/2 | — | PASS |
| two-copy boom discrepancy | side boom z 155.0678 (front) vs 152.3677 (top copy); unique to the side boom | union 152.37–226.07 | — | CONFIRMED |
| 11 dimension entities | all consistent with the model ≤0.02 | — | — | PASS |

**Containment findings (all folded back into the boxes, conservative
direction):** the 8 hung-device boxes sat ~4 cm outboard of the drawn devices
(knob/lens details protrude 4.9 cm inboard) → extended inboard; the
guide-rod box missed the Ø2 couplers (X −1.23, z +0.5) and two Y-segments
toward crank/winder → extended; the feed-roll END FLANGES (exact R = 7.0
circles) reach the outer wall X = 0.0 → box X-lo moved to −0.15; six
0.02–0.04 cm beam-pair/rail nicks (down_boom_E, down_gusset_W/E, side_boom,
side_gusset, top_slab) → padded 0.05; the levelling-foot pads overhang the
table block by 0.33 → widened; and the PAPER WEB ITSELF rises 2.9 cm over
the guide rods (solid 5D2) → new `paper_curl` box. After the fixes every
drawn member above the paper plane is inside a box (the deliberately unboxed
items are the decorative robot depictions and the J1-axis marker solids,
which sit where the real robots go); the only below-plane geometry outside a
box was the 0.33 cm foot-pad overhang, now enclosed.

The URDF is generated from `rig_final.py` and separately checked:
`tests/test_final_rig.py` pins every URDF box/weld to the module at 2e-6 m,
and `scripts/check_final_rig_urdf.py` (station venv) loads both URDFs in
pydrake, FK-checks one dimension-anchored quantity per arm, and holds
drake's FK of the vendored URDF against `frames.fk` to 6e-12.

## What the final rig can draw (atlas, 2 cm grid, tilt ≤ 15°, pen 0.110)

`scripts/run_atlas.py --out out/atlas_final`; map:
`out/atlas_final/final_dead_zones.png`; masks: `out/atlas_final/coverage.npz`.

| quantity | value |
|---|---|
| sheet cells (1.8034 × 1.700 m) | 7826 |
| union REACHABLE | 84.9 % |
| union STRICT-GO (margin ≥ 0.30, σ ≥ 0.14, frame boxes active) | **68.6 %** |
| arm 13 (up) strict-GO | 29.1 % — the whole front half-disc |
| arm 31 (down) strict-GO | 25.7 % — back-left annulus |
| arm 2 (side) strict-GO | 18.0 % — back-right lobe, with a reach-without-GO ring under its mount |
| cells strict-GO by ≥ 2 arms | 4.1 % — thin handoff bands where the three lobes meet (~(0.7, 0.75)) |

**Dead zones** (nobody strict-GO): the left strip x ≲ 0.12 m (feed roll +
wrist margin + arm-31 boom), the right strip x ≳ 1.70 m (reach + guide-rod
corridor), all four corners, the under-base holes of arms 31 and 2, and the
top-center sliver between the 31/2 lobes. Left/right thirds are ~40 % dead
each; the middle third only 14 %.

## Appendix — arm 2's mount height (docs/ARM2_HEIGHT.md)

Arm 2 is the only arm with a free mounting parameter: it hangs on a vertical
2 × 3″ pole and could be re-clamped anywhere along it. Asked whether sliding
it along z kills the dead zone under it — **it cannot, because the plate is
already at the bottom stop.** The full study is `docs/ARM2_HEIGHT.md`;
`scripts/arm2_height_sweep.py` → `out/arm2_height_sweep.{png,json}`.

- **Arm-2-attributable dead paper today: 990 cells = 0.396 m² = 40.3 % of all
  dead paper** — the right rod strip (578 cells, guide rods + `paper_curl`
  clearance, *not* reach) and an under-shoulder hole (412 cells, a disc
  r ≤ 0.34 m about canvas (1.2768, 1.2572); arm 2's shoulder is 0.333 m in −X
  of its plate because its J1 axis is horizontal).
- **Slide range, DXF-re-derived**: the *only* piece gripping the pole is the
  bracket at z 155.0675–160.8338; the base plate's top is flush with the
  pole's own lower end and the two 3.81 spacer blocks hang in air. Travel
  **down 0.00 cm (front copy) / 2.70 cm (top copy)**; **up 65.19 cm**, clear
  the whole way to the top beam at 226.0275. This is Flags #1 and Flags #2
  meeting: the mount is "not stiff and stable" *because* the arm hangs off the
  end of the pole.
- **Every centimetre down helps, every centimetre up hurts**, monotonically.
  Sliding up 12 cm costs 7.4 points of union coverage.
- Best reachable without touching the pole (−2 cm): union 68.63 → 69.15 %.
  Noise; not worth the last of the bracket engagement.
- **Optimum −20 cm (canvas z 0.576)**: union strict-GO **68.63 → 75.12 %**,
  arm 2 **18.0 → 38.8 %**, cells covered by ≥ 2 arms **4.1 → 18.2 %**, 587
  dead cells (0.235 m²) recovered against 79 newly dead. Needs ≈ 20 cm more
  pole, into a volume verified empty for 71 cm below that — and that same
  change is what would finally make the joint stiff (≈ 28 cm of engagement
  instead of 5.8). Arms 13 and 31 are provably unaffected (2276 → 2276,
  2009 → 2009).
- **Still dead afterwards**: the right rod strip (493 cells — transport
  clearance, no base height fixes it) and an inner annulus at r ≈ 0.10–0.16
  about the shoulder (181 cells — a wrist-fold/σ failure at a shoulder that
  sliding along z never moves in xy).

Nothing in `rig_final.py` was changed; adopting this means editing
`ARM_MOUNTS_W["side"]`, the three `side_*` mount boxes, `side_boom`'s length,
and regenerating the URDF + atlas.

**Adopted 2026-08-21, for BOTH units, without touching this file.** The user
took the −20 cm recommendation for arm 2 and (by mirror symmetry) arm 97. It
lives as `rig_final6.FLEET_FINAL6_OPT` — the same 35 boxes with the clamped
stack slid and `side_boom` lengthened, handed to
`rig_final.frame_boxes_canvas(boxes=…)` — and is installed by
`ARIS_RIG=final6_opt`. `rig_final.py` stays the drawing. The re-swept numbers
(and what the longer pole costs the other arms: nothing) are in
`docs/MERGED_CANVAS.md`.

## Flags / ambiguities / risks

1. **Arm-2 boom bottom disagrees between the two DXF model copies**: front
   copy z 155.07→226.07, top copy z 152.37→226.03. Collision model uses the
   union (152.37→226.07). These 4 solids are the ONLY geometry in the drawing
   where the copies disagree (313 of 315 match to < 0.05 cm). Evidence both
   ways: the top-copy length is 73.66 cm = **29.00 in exactly**, the same part
   as the four arm-31 boom beams, and its top sits flush on the beam
   underside; the front-copy version is what the PDF actually prints
   (pixel-verified) but is a round *metric* 71.00 cm in an all-imperial rig
   and over-runs 0.04 cm into the top beam. Ask which is as-built — it is the
   difference between 0.00 and 2.70 cm of free travel for the arm-2 plate
   (docs/ARM2_HEIGHT.md).
2. **The drawing itself warns about arm 2's mount**: MTEXT *"Arm 2 side
   position: conection base plate to boom pole is not stiff and stable."*
   A compliant mount means the surveyed base pose may not hold under load —
   the calibration margin term for arm 2 should NOT be reduced below 30 mm
   until the mount is stiffened or surveyed under load. Surface this to the
   fabricator. **The geometry says exactly why**: the only piece gripping the
   pole is a 5.77 cm bracket (z 155.0675–160.8338) at the pole's very bottom
   end; the base plate's top edge is flush with where the pole stops and the
   two 3.81 spacer blocks are bolted to nothing. Lengthening the pole ~20 cm
   downward fixes the stiffness AND is worth +6.5 points of union coverage —
   see the appendix above.
3. The 91,6 annotation says "to surface (drawing paper)" but geometrically
   measures to the **tabletop** (91.40 to the paper top). We anchor arm 2 by
   the plate solids, not this dim, so nothing depends on the reading — but
   the wording should be clarified with the author.
4. Rounding: "63,5" is exactly 63.468; "233,7" structure is 233.648; "22,6"
   plate is 22.582. The model geometry (3 decimals) wins over the rounded
   annotations everywhere.
5. **Arm 13 has no dimension chain** — its pose rests on the plate solid
   alone (both copies agree exactly). It also has **no fasteners drawn**
   (arms 31/2 have clamp blocks; 13's plate just rests on the tabletop).
   Confirm how it will be fixed and whether its position is final.
6. Base **yaw convention**: "front = the 8.78-offset plate side" is inferred
   from the base asymmetry + the three depicted (decorative) poses. A 180°
   yaw error per arm is possible until checked against a real plate. The
   depicted poses all lean toward the inferred front, consistently.
7. Curved bodies (leveling feet, paper roll, decorative robot shells) have
   point-sampled extents; all structural box members are exact. The roll box
   is padded outward and flagged as approximate.
8. The DXF text offers an option: *"possibly 2 tables 208,3 × 109,2"* (split
   along X). Not modeled; ask if the split is happening.
9. Arm-31 axis X has three sources spanning 45.720–45.741 (marker / dim /
   ring); the dim value 45.737 is used. Arm-2 axis z: 141.268 (13.8 rule) vs
   141.279 (ring solid), 0.011 apart — 13.8 rule used.
10. The two DXF copies were cross-checked wherever both draw the same member;
    agreement < 0.02 cm except Flags #1.

## Open questions for the user

- Which reading of the arm-2 boom bottom is as-built (Flags #1)?
- Arm 13 fastening + final position (Flags #5)?
- Confirm each base plate's yaw (Flags #6) — a photo of each plate suffices.
- Is the two-table split (Flags #8) happening?
- What are the 8 hung devices (cameras/lights)? They are modeled as obstacles
  only.
