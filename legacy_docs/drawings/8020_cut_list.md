# 80/20 cut list — ARIS six-arm cage

**Issued 2026-09-16 · design mount plane h = 970.0 mm above the paper · all dimensions mm.**

> **REV A — SUPERSEDES THE 1435 / 1525 DROP POSTS of out/ceiling_8020_layout.*.**  Those posts came from reading the original drawing's 233,7 cm as a paper-referenced dimension when it is floor-referenced.  They are 716.4 mm too long.  Do not cut them.

## Open items — READ BEFORE CUTTING

5 of these 7 block a cut on this sheet.  Nothing below is a guess about them; each says what it blocks.

| # | item | blocks | how it closes |
|---|---|---|---|
| 1 | **PLATE OFFSET DIRECTION** | BLOCKS the drop clusters — the post x positions depend on it. | Measure the real plate, or open the post pitch to 381.0 (posts at the axis +/- 190.5), which fits either way with 14.4 mm each side. |
| 2 | **BASE CABLE PASS-THROUGH** | BLOCKS the plate and the clamp stack. | Measure the connector envelope on a real FR3 base and cut the plate and the clamp stack around it. |
| 3 | **CAGE LEGS** | BLOCKS the leg cut (item E qty) and any FURTHER mid-span member. | A structural check of the unsupported spans, then a leg count and position — and a check that no mid-span leg lands in a certified flight path.  The seam posts do land in one: see open item 7. |
| 4 | **GUSSET PART AND ATTACHMENT** | BLOCKS item F. | The fabricator's own bracket selection, against the 76.2 post face and the runway's y face. |
| 5 | **ROOM SURVEY** | BLOCKS nothing that is cut to the corrected datum, but it is the reason the datum has to be stated on every sheet. | Survey the room: floor flatness, clear height, and door/route width for a 4.01 m frame. |
| 6 | **RE-CERTIFICATION OF THE DROP CLUSTER** | BLOCKS final fabrication of the clusters and gussets. | Send the steel design back for re-certification against the certified poses (minutes, not days) — system_model.reconciliation(). |
| 7 | **SEAM FRAME — AS DIRECTED 2026-09-14 — one representative bar per side** | CLOSED 2026-09-14.  It blocked the middle row's parks, which were re-searched against the bar and moved — a seam bar stands 114.3 mm outboard of the canvas edge over the full 1651 mm, straight through the band a middle-row arm's links sweep.  docs/DECISIONS.md. | NOTHING OUTSTANDING.  The bar is what was asked for, and it is in the planner's certified static set (mounts.obstacles_for) as of 2026-09-14 — so the parks and the certified area already account for it.  A builder who finds something else at the seam edits one table, mounts.SEAM_BARS_MM. |

**1. PLATE OFFSET DIRECTION** — Does the base plate's 25.15 mm offset from the J1 axis run in canvas +x or -x?  The drawing gives it for an arm whose front faces +X; every arm here is clocked the other way (R_world_base = Ry(180)), so the offset flips — and that flip is an INFERENCE.  7.79 mm rides on it: the plate nests in a 241.4 mm slot, and offset the wrong way it does not fit at all.  *Closes by:* Measure the real plate, or open the post pitch to 381.0 (posts at the axis +/- 190.5), which fits either way with 14.4 mm each side.  *BLOCKS the drop clusters — the post x positions depend on it.*

**2. BASE CABLE PASS-THROUGH** — The plate and the clamp stack are solid across the arm's own base cable.  The manufacturer's link0 visual carries the connector and cable stub 230.7 mm past the base flange and 177 mm radially; every arm here is INVERTED, so that 230.7 mm points straight UP — through the 12.7 mm plate, through the 95.7 mm clamp stack, and on into the cluster slot.  THE MODEL FOUND THIS, not a person: the collision shell stops dead at the flange, so no clearance check in this repo has ever seen it.  *Closes by:* Measure the connector envelope on a real FR3 base and cut the plate and the clamp stack around it.  *BLOCKS the plate and the clamp stack.*

**3. CAGE LEGS** — How many legs does a 4.01 m frame need, and where?  Correcting the ceiling datum turned this cage from something hanging off a room ceiling into something standing on the floor.  Four corner legs at 1651.0 mm are drawn.  PARTLY ANSWERED 2026-09-14: the seam frame (item I, open item 7) puts FOUR more posts at the paper's mid-length, so the clear runway span is now 1853.42 mm corner-to-seam, not the full frame — but those posts are REPORTED, NOT PHOTOGRAPHED, and nothing else has changed.  *Closes by:* A structural check of the unsupported spans, then a leg count and position — and a check that no mid-span leg lands in a certified flight path.  The seam posts do land in one: see open item 7.  *BLOCKS the leg cut (item E qty) and any FURTHER mid-span member.*

**4. GUSSET PART AND ATTACHMENT** — The drawing has no part number for a gusset, and the 203.2 x 203.2 x 38.1 figure is its envelope, not a detail.  The rotation onto the runway's outboard y faces resolves a hard interference and leaves 89.2 mm inside a transverse pair.  *Closes by:* The fabricator's own bracket selection, against the 76.2 post face and the runway's y face.  *BLOCKS item F.*

**5. ROOM SURVEY** — The real room has NEVER been measured.  Every ceiling number in this repo descends from the original drawing's 233,7 cm, which is floor-to-top-of-cage — the drawing defines no room ceiling at all.  The cage as drawn stands 2336.5 mm tall.  *Closes by:* Survey the room: floor flatness, clear height, and door/route width for a 4.01 m frame.  *BLOCKS nothing that is cut to the corrected datum, but it is the reason the datum has to be stated on every sheet.*

**6. RE-CERTIFICATION OF THE DROP CLUSTER** — The certified obstacle model is a single 200 x 200 mm column on the base axis.  The real steel is a 393.8 x 152.4 mm cluster offset 25.15 mm off it, plus 203.2 mm gussets.  Neither contains the other: all 60 pieces of mount hardware fall outside the certified envelope, worst 185.55 mm.  *Closes by:* Send the steel design back for re-certification against the certified poses (minutes, not days) — system_model.reconciliation().  *BLOCKS final fabrication of the clusters and gussets.*

**7. SEAM FRAME — AS DIRECTED 2026-09-14 — one representative bar per side** — Pete Werner, 2026-09-14, on the real hardware: "there are a few bars on the real hardware that are not in our model.  they are supports in the middle ... the real thing is essentially the two halves next to each other."  The rig is TWO of the original 2184.4 x 2082.8 half-cages butted along the paper, and two butted halves are 4165.6 against this frame's 4011.64 — a difference of 153.96 mm, one doubled 3 in end frame to within 1.56 mm.  Laid flush with this frame's own ends, each half's seam-side END RAIL lands within 0.78 mm of the MIDDLE RUNWAY already drawn: the runway IS the two butted end rails and is NOT cut twice.  What was missing is what holds them up — items I and J, 2 bars and 2 brace clusters straddling y = 1815.32.  The bars are model bodies; the braces are drawn DASHED because a box containing a face-bolted brace also contains its post and system_model will not carry interpenetrating geometry.  WHAT TO BUILD THERE IS SETTLED, not photographed: Pete Werner, the same day — "just put a representative bar in the middle that is as wide as two of the corner struts" — so this sheet asks for one 76.2 x 152.4 bar per side instead of a pair of inferred posts, and nothing about the seam is waiting on a photograph.  *Closes by:* NOTHING OUTSTANDING.  The bar is what was asked for, and it is in the planner's certified static set (mounts.obstacles_for) as of 2026-09-14 — so the parks and the certified area already account for it.  A builder who finds something else at the seam edits one table, mounts.SEAM_BARS_MM.  *CLOSED 2026-09-14.  It blocked the middle row's parks, which were re-searched against the bar and moved — a seam bar stands 114.3 mm outboard of the canvas edge over the full 1651 mm, straight through the band a middle-row arm's links sweep.  docs/DECISIONS.md.*

## The datum, in one sentence

**z = 0 is the top surface of the paper as laid on the table; the floor is -636.68 mm and the runway beams' underside — what a drop post hangs from — is 1623.62 mm, so the cage stands 2336.5 mm floor to top of steel, which is the original drawing's own 233,7 cm.**

x runs across the short side of the canvas (0 -> 1803.4), y along the long side (0 -> 3630.64), z up.  The canvas reference corner is (0, 0) and every plan dimension on the drawings is measured from it.

## Drop post — the one height-dependent cut

```
post cut length = (runway underside - h) + post over-run
                = (1623.62 - h) + 34.98
```

`34.98` is the drawing's own over-run of the post past the plate underside (`down_plate` lo z minus `down_boom_W` lo z), not a round 35.  24 off, whichever height is chosen.

| mount plane h | post cut length | total 3-in extrusion | what h is |
|---:|---:|---:|---|
| **970** | **688.6** | 47.41 m | **the design height of this sheet AND `layout.LAYOUT_PROPOSED['h']`** — adopted 2026-09-10; what the software plans against today, and the only height at which the canvas has no enclosed dead cells |
| 940 | 718.6 | 48.13 m | what the software planned against 2026-08-26 .. 2026-09-09; superseded |
| 850 | 808.6 | 50.29 m | what the hardware is built at now (verticals trimmable) |

At the sheet's own h = 970 the post is **688.6 mm**.  Cut all 24 to one length and keep the six mount planes mutually coplanar within +/-3 mm.

## Members

| item | qty | profile | cut length | what it is |
|---|---:|---|---:|---|
| **A** perimeter side rail | 2 | 3" x 3" T-slot | **4011.64** | runs in y, full length. The frame outside sits 190.5 (7.5 in) clear of the canvas on all four sides. |
| **B** perimeter end rail | 2 | 3" x 3" T-slot | **2032.00** | runs in x, butts between the side rails. 2032.0 = 80.00 in, the original frame's own cut. |
| **C** runway beam | 6 | 3" x 3" T-slot | **2032.00** | two laid side by side per arm row (152.4 overall) = the drawing's CENTRAL DOUBLE BEAM. Seam ON the row line, so each arm's J1 axis lies on it. 3 rows. |
| **D** drop post | 24 | 3" x 3" T-slot | **688.60** | four per arm in a 2x2 cluster, pitch 317.6 (x) x 76.2 (y). THE HEIGHT-DEPENDENT CUT — see the table. (The original rig's own is 736.9 = 29.00 in.) |
| **E** corner leg | 4 | 3" x 3" T-slot | **1651.00** | floor to the perimeter rail's UNDERSIDE. ASSUMED — the cage is self-supporting and floor-standing, and a 4.01 m frame on four legs has no precedent (the original spans 2.08 m). MID-SPAN LEGS ARE ALMOST CERTAINLY REQUIRED: see open item 3. |
| **F** top gusset | 24 | 8" x 8" x 1.5" gusset | **203.2 x 203.2 x 38.1** | 203.2 x 203.2 x 38.1, four per arm, top flush with the grid, ROTATED onto the runway's outboard y faces. The drawing's inboard orientation needs 406.4 mm across a transverse pair and only 216.2 exists; rotated, the pair clears by 89.2. No part number: see open item 4. |
| **I** seam support bar | 2 | 3" (x) x 6" (y) | **203.2 x 203.2 x 38.1** | AS DIRECTED 2026-09-14 — one representative bar per side. Pete Werner: "just put a representative bar in the middle that is as wide as two of the corner struts." ONE PER SIDE, tabletop (-27.38) to runway underside (1623.62), standing in the same x bands as the corner legs but at the seam, y = 1815.32, 152.4 wide in y and centred on it. Two 3 in posts side by side build the same thing — the bar is exactly their union — and that is what the original drawing's own post_BL / post_BR do. These are the mid-span legs open item 3 said were almost certainly required. Same cut as item E. |
| **J** seam corner brace | 4 | 8" x 8" x 1.5" gusset | **203.2 x 203.2 x 38.1** | AS DIRECTED 2026-09-14 — one representative bar per side. 203.2 x 203.2 x 38.1, the drawing's brace_BL / brace_BR — two plates on each seam post's two INBOARD faces, running 203.2 into that post's OWN half-cage and 203.2 down from the runway underside. Four clusters, eight plates. Same envelope as item F and no part number either. DRAWN DASHED — these are NOT in the collision model: a box that contains a face-bolted brace also contains its post, and system_model refuses geometry that interpenetrates. Nothing rides on that, they sit 450 mm above the mount plane. |

**Total 3-in T-slot extrusion at h = 970: 47.41 m.**  (Items A-E and I; the gussets F and the seam braces J are bought brackets.)

> **Items I and J are the SEAM FRAME — AS DIRECTED 2026-09-14 — one representative bar per side.**  They are the steel that holds up the two butted half-cage end rails at y = 1815.32, read at run time from `system_model.seam_bodies()` (2 boxes).  The END RAILS THEMSELVES ARE NOT A NEW CUT: they are item C's middle runway, which this model already builds within 0.78 mm of where the two halves put them.  Open item 7.

## Plate and fabricated parts

| item | qty | size | what it is |
|---|---:|---|---|
| **G** robot base plate | 6 | 225.82 x 190.0 x 12.7 | the drawing's own 22,6 x 19,0 plate at 12.7 (0.5 in). The arm bolts to its UNDERSIDE, and that underside IS the mount plane h. Nests in the 241.4 mm slot between the post pairs with 7.79 mm clear each side. NEEDS A HOLE for the base cable — open item 2. |
| **H** plate clamp stack | 6 | 226.0 x 152.4 x 95.7 | clamps the plate between the post pairs, sitting on top of it. ENVELOPE ONLY — the drawing carries no part number. NEEDS A HOLE for the base cable — open item 2. |

## Fasteners

**Per the original drawing** — T-slot corner brackets, end fasteners and gusset hardware to the fabricator's own standard for 3-in profile.  The drawing carries no fastener schedule and neither does this sheet; nothing in the certified model depends on one.

## The z ladder at this height

| level | mm above the paper |
|---|---:|
| grid top | 1699.82 |
| grid underside | 1623.62 |
| gusset bottom | 1496.62 |
| clamp top | 1078.40 |
| plate top | 982.70 |
| mount plane | 970.00 |
| post bottom | 935.02 |
| paper top | 0.00 |
| table top | -2.00 |
| leg bottom | -27.38 |
| floor | -636.68 |

## Plan set-out

Frame outside **2184.4 x 4011.64** (86.00 x 157.94 in), 190.5 mm clear of the canvas on all four sides.  Columns x = 596.7 / 1206.7 (pitch 610.0); rows y = 605.11 / 1815.32 / 3025.53 (pitch 1210.21).  Base positions are to the **J1 axis** (the centre of the base bolt circle), not to a plate edge; tolerance +/-10 mm per base.

Every arm is clocked identically — `R_world_base = Ry(180)`, the arm's front toward the canvas x = 0 edge, so **all six connector panels face the x = 1803.4 edge**.  The plate centre sits 25.15 mm from the J1 axis toward that same edge — DIRECTION INFERRED, open item 1.

The certified drawing area at this height is **1500 x 3640 mm** (x 160..1640, y 0..3620), 5.460 m², from `out/certified_area_h0970.json` — drawn dashed on the plan.  It is what this spacing buys, and it is not the whole canvas.

## Provenance

Generated by `scripts/draw_8020.py`, which reads `aris_sixarm.system_model` (dimensions and provenance), `aris_sixarm.layout` (base positions and h), `aris_sixarm.mounts` (the certified keep-out) and `out/certified_area_h####.json`.  It defines no dimension of its own.  **Nothing here is a survey.**
