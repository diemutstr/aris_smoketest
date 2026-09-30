# The system model — `assets/system_model/`

A dimensioned, provenance-annotated 3D model of the whole ARIS installation:
the 80/20 cage, the table and paper it stands over, six FR3 arms at
`FLEET_PROPOSED`, and a pen holder on every hand. Visual **and** collision.

Built 2026-09-01. Generator `scripts/gen_system_model.py`, truth
`aris_sixarm/system_model.py`, validator `scripts/check_system_model.py`,
tests `tests/test_system_model.py`.

```
python3 scripts/gen_system_model.py all           # meshes + URDFs + manifest
python3 scripts/gen_system_model.py urdf --fingers stock|fat|both
<station venv>/bin/python scripts/check_system_model.py
<station venv>/bin/python scripts/render_system_model.py
```

`<station venv>` = `/home/franka/git/franka_manipulation_station/.venv`. The
repo's own venv (numpy, scipy, trimesh, pytest) is `.venv` — see the README.

From python, so nothing hardcodes the layout:

```python
from aris_sixarm import system_model as SM
SM.urdf_path()                      # installation.urdf, manufacturer shells
SM.urdf_path("capsule")             # installation_capsules.urdf
SM.urdf_path(fingers="fat")         # installation_fatfingers.urdf  (§7d)
SM.urdf_path(with_arms=False)       # environment.urdf
SM.manifest_path()                  # model_manifest.json
SM.report()                         # the dimensions, as text
```

---

## 1. The headline: the ceiling datum was wrong, and this model says so

**`mounts.MOUNTS.ceiling_z = 2.34 m` is a provenance bug.** It has not been
changed — see §6 — but the model carries the truth.

The original drawing has one overall height dimension, **233,7 cm**, and it is
**floor to top-of-construction**. The drawing describes a self-supporting cage
standing on the floor; it defines no room ceiling anywhere. The paper sits
**636.68 mm above that floor**. So above the *paper*, the same steel reaches:

| datum | mm above the paper | how |
|---|---:|---|
| top of construction | **1699.82** | `top_slab` hi, 233.65 cm − 63.668 cm |
| beam underside (what a post hangs from) | **1623.62** | `top_slab` lo |
| cage total height above the **floor** | **2336.50** | = the drawing's 233,7 cm |

`ceiling_z = 2.34 m` is that 233,7 re-datumed to the paper — a floor-
referenced number used as a paper-referenced one. It puts the beam underside
at **2340.0** above the paper against the drawing's 1623.62, and the top of the cage at **2416.2** against 1699.82 — **716.4 mm**
too high, both of them. Measured from the floor, where the drawing measures,
the code datum stands the cage **3052.88 mm** tall against the 2336.5 drawn.

Being too tall is **conservative for collision** — a longer obstacle never
certifies a pose a shorter one refuses — which is why nothing has broken and
why no certified number is in question. It is **not** conservative for a
fabricator:

| | drop post length at h = 970 |
|---|---:|
| the buggy datum | 1404.98 mm |
| **the corrected datum** | **688.60 mm** |
| the original rig's own | 736.90 mm |

The corrected post is 48.3 mm shorter than the one that was actually built,
and the buggy one is 716.4 mm too long — an error of a different order. (At
the 940 this table used to be written for, the corrected post was 718.60 mm,
within 18 mm of the built one, and the sheet's own UNKNOWN 3 had spotted the
same thing from the other end: "a grid at the ORIGINAL's own beam height gives
a 718.6 post".)

### Corroborated straight off the drawing, not only through the extraction

Everything above descends from the DXF solids via
`rig_final.FRAME_BOXES_W_CM`. The PDF also carries a **text block written by
the person who drew it**, and it says the same things independently:

> Table height 63,5 cm, Height construction **233,7 cm**
> Entire cage construction built with T-slotted aluminum profile:
> 3" x 3" (**7,62 x 7,62 cm**) or 1,5" x 3"
> Base plate robot arms: **22,6 x 19,0 cm**
> Position of center of rotation of the arm: **13,8** x 9,5 cm on the base plate
> **Note: the axis of rotation of the robot arms is not in the center of the base plates!**

And the front view **dimensions the 233,7 from the floor**, with 63,5 (the
table) stacked underneath it — which is the datum correction, in the author's
own drafting. `test_the_model_reproduces_the_drawings_own_TEXT_block` holds
the model to those figures at the text's own rounding.

One text-versus-solid disagreement is worth knowing about and changes nothing
here: *"Distance backside base plate to surface ( drawing paper: 91,6 cm"*
says 916 mm where the `down_plate` solid sits at **922.0**. That is the
*original* rig's mount plane; the height in force is
`layout.LAYOUT_PROPOSED["h"] = 940`, so nothing in this model depends on it.
Pinned so nobody re-derives 922 from the text and concludes the model is
wrong.

**Nothing here is a survey. The real room has never been measured.**

## 2. The z ladder in force

All mm, canvas frame, `z = 0` at the **top surface of the paper**.

```
  1699.82   grid top  = top of construction  (2336.50 above the floor)
  1623.62   grid underside — the runway beams' soffit, where posts hang
  1496.62   gusset bottom
  1078.40   clamp stack top
   982.70   plate top
   970.00   MOUNT PLANE — the arm bolts to the plate's underside
   935.02   post bottom (34.98 of post over-runs past the plate)
     0.00   paper top
    -2.00   table top / paper underside
   -27.38   cage leg bottom
  -636.68   floor
```

## 3. What is modelled

**77 static bodies**, every one an axis-aligned box with a named source.

| | count | what |
|---|---:|---|
| cage | 68 | 4 perimeter rails, 4 corner legs (1651.0 long, stopping under the rails they carry), 6 runway beams (3 rows × 2), 24 drop posts (6 arms × 4), 24 gussets, 6 clamp stacks |
| mount | 6 | the robot base plates, 225.82 × 190 × 12.7 |
| table | 2 | the table, and the floor plane |
| canvas | 1 | the paper, 1803.4 × 3630.64 × 2.0 |

Plus, per arm × 6:

- **visual** — the manufacturer's glTF meshes, **with their real textures**
  (§4)
- **collision** — the manufacturer's own collision shells (`installation.urdf`)
  or the audited capsule set (`installation_capsules.urdf`). §5.
- **pen holder** — housing + cap, re-decimated from the raw CAD, with the
  proven 3-cylinder envelope as collision, plus the pencil tail's own fourth
  cylinder. §7.
- **cable dress** — two translucent conduit loops, **ESTIMATED**, visual only.

### Cage details worth knowing

- **3″ × 3″ (76.2 mm) T-slot throughout**, the drawing's own MTEXT.
- Perimeter frame **2184.4 × 4011.64** outside — the 2184.4 is exactly
  86.00 in, the original frame's own cut. Margin 190.5 mm (7.5 in) clear of
  the canvas on all four sides.
- **Three double-beam runways**, 2 × 76.2 side by side (152.4 overall),
  span 2032.0 = 80.00 in, **seam on the row line** so each arm's J1 axis lies
  on it.
- **Per arm a 2 × 2 drop-post cluster**, pitch 317.6 (x) × 76.2 (y), plan
  footprint 393.8 × 152.4.
- The **plate nests between the post pairs**: 241.4 mm slot, 225.82 mm plate,
  **7.79 mm clear each side** (the drawing's own slightly-fat 77.2 booms
  leave 240.5 and 7.34). Its centre sits 25.15 mm in canvas **+x** from
  the J1 axis — and that direction is an inference. See §8.
- **Gussets rotated onto the runway's outboard y faces.** In the drawing's
  own orientation two 203.2 mm gussets face each other across a transverse
  pair and need 406.4 mm where only **216.20 mm** exists. Rotated, each plate
  is centred on its own post and the pair clears by **89.20 mm**.

### 3b. The seam support — the two half-cages, and the bar where they meet

> **Pete Werner, 2026-09-14:** *"we need to be a bit careful with the parking
> poses — there are a few bars on the real hardware that are not in our model.
> they are supports in the middle, I think the half-table drawing had them. the
> real thing is essentially the two halves next to each other."*
>
> ...and, the same day, settling what to model rather than photographing it:
> *"just put a representative bar in the middle that is as wide as two of the
> corner struts."*

**Two new bodies, `system_model.seam_bodies()`: `seam_bar_W` and
`seam_bar_E`, one per side of the frame, centred on y = 1815.32 mm.** They
take the model from 77 static bodies to 79. Provenance **DRAWING** — the bar
is the two half-cage end-frame corner posts (`post_BL` / `post_BR`) as one
member. **Their corner brace plates are not geometry**: bolted on a post's
faces they overlap it in any AABB, the model's own four corner legs carry none
either, and they sit at z 1420…1624 — 450 mm above anything a certified pose
reaches.

**They are in the certified static set** (`mounts.obstacles_for`), since the
re-certification of 2026-09-14. `ARIS_SEAM_POSTS=0` takes them back out.

#### The arithmetic that says where the seam is

The drawing describes ONE self-supporting cage, **218.44 × 208.28 cm**.
`rig_final6` already reads the installation as two of those abutting
back-to-back (`MIRROR_PLANE_CANVAS_Y` = 1.81532 m). This module's cage was
never built that way: it is one long frame, `FR_L` = 4011.64 mm, derived from
the canvas. **Those turn out to be the same statement**, and that is the whole
corroboration:

| | mm |
|---|---:|
| two half-cages butted, outer face to outer face | 2 × 2082.8 = **4165.60** |
| this model's frame | **4011.64** |
| difference | **153.96** = one doubled 3″ end frame (152.4) **+ 1.56** |

Lay each half flush with this model's own outer frame end and its seam-side
**end rail** lands on this model's own **middle runway**:

| | y band, mm |
|---|---|
| half A (south), y ∈ [−190.5, 1892.30] — its N end rail | 1816.10 … 1892.30 |
| `runway_r1_N` | 1815.32 … 1891.52 |
| half B (north), y ∈ [1738.34, 3821.14] — its S end rail | 1738.34 … 1814.54 |
| `runway_r1_S` | 1739.12 … 1815.32 |

**0.78 mm, both of them, in opposite directions** (`SEAM_RAIL_RESIDUAL_MM`).

So **the middle runway *is* the two butted end rails**. That steel is already
in the model and must not be counted twice — `seam_bodies()` builds no rail,
and `tests/test_system_model.py` refuses one. The middle arm row's J1 axes lie
on the seam plane because that is where two half-cages put a beam; the seam
plane, the paper's mid-length, the frame's mid-length, the middle row and
`rig_final6.MIRROR_PLANE_CANVAS_Y` are one number, pinned four ways.

#### What was missing: what holds those rails up

| body | x, mm | y, mm | z, mm |
|---|---|---|---|
| `seam_bar_W` | −190.5 … −114.3 | 1739.12 … 1891.52 | −27.38 … 1623.62 |
| `seam_bar_E` | 1917.7 … 1993.9 | 1739.12 … 1891.52 | −27.38 … 1623.62 |

3″ in x on the frame's own corner-leg line, **6″ in y — "two corner struts" —
centred on the seam plane**, standing from the tabletop to the runway
underside. Cut length **1651.0 mm, the same cut as a corner leg.** These are
the mid-span legs `OPEN_QUESTIONS['cage_legs']` said were *"almost certainly
required — the original spans 2.08 m"*.

**THE TABLE IS `mounts.SEAM_BARS_MM`** and it is the only place a hardware
change touches. It lives in `mounts` and not here for an import-graph reason
spelled out in both modules: `system_model` reads `layout.FLEET_PROPOSED` at
import and `layout` builds that fleet through `mounts.obstacles_for`, so
`mounts` cannot import `system_model`.
`test_the_seam_bar_table_is_the_drawings_own_arithmetic` pins the table
against this section's own derivation.

#### One bar, or the four posts it replaces — the same steel

The bar was four inferred posts for four hours on 2026-09-14, and the swap
cost nothing measurable, by arithmetic rather than luck. The two posts at one
x occupied y ∈ [SEAM_Y − 76.2, SEAM_Y] and [SEAM_Y, SEAM_Y + 76.2]; **their
union is exactly this bar**, and a union of two AABBs that share a face is an
AABB. Every clearance in this repo is a distance to that occupied volume, so
the bar reproduces every number the posts decided — measured, not assumed:
park clearances 50.9 / −35.3 mm, certified area 16 184 → 16 019, v19 at
−59.4 mm, all identical.

#### What the seam support is NOT

- **Nothing below the paper.** The nine 3 × 3 blobs in the drawing's top view
  are the **table's** legs and levelling feet: the table has a mid-width leg
  and a mid-depth cross rail, both under the tabletop, both already inside
  this model's solid `table` body.
- **No diagonals above the table.** The X-bracing in the front view is all in
  the 63.5 cm table frame. Above it there are corner gussets only.

#### The corner braces, at the seam and at the frame's own corners

`brace_FL`…`brace_BR` exist in the drawing and in
`rig_final.FRAME_BOXES_W_CM`, and this model has never built them anywhere. A
pre-existing omission, now flagged rather than fixed — z 1420…1624 is 450 mm
above anything a certified pose reaches.

#### What it cost — `scripts/seam_impact.py`, docs/DECISIONS.md

Until 2026-09-14 `StudySpec.static_obstacles` returned the neighbours' mount
boxes and base columns **and nothing else**: every certified number on the
proposed rig was earned against a fleet standing in an empty room with no cage
around it. The bars are in that set now, which re-decided every certified cell
— the middle row's parks moved, the certified area lost 165 of 16 184 cells
with the hole-free block untouched, and the shipped v19 programme fails on
steel. The numbers are in **DECISIONS.md**, and `scripts/seam_impact.py`
re-measures them against any artefact still on disk from before.

## 4. The textures — solved by vendoring, not by stripping

The vendored glTFs referenced **65 image files that were never copied
across**. Every renderer fell back to flat white and printed a warning per
image per mesh (`Meshcat could not get data for the named uri 'hand_color.png'`).

**All 54 the arm's own meshes name were found** — 27 PNG maps and their 27
`.ktx2` twins — in the directory the meshes themselves came from,
`~/git/franka_manipulation_station/assets/franka_description/meshes/visual`.
The **27 PNGs** (3 maps × link0–7 + hand: colour, normal,
occlusion-roughness-metallic, 2048² each) are vendored **byte-identically**,
with a sha256 per file in the manifest. No resampling: a resized texture is a
texture whose provenance is "the tool resized it".

The `.ktx2` twins are **not** copied. VTK says so itself — *"glTF extension
KHR_texture_basisu is used in this model, but not supported by this loader"* —
so they were 0.8 MB of dead weight and one warning per mesh. The extension
declaration and the `.ktx2` image entries are removed and the texture table
re-indexed onto the PNGs that remain.

The `.bin` geometry buffers come too (5.95 MB, the ten meshes this model
mounts, not the 10 MB `cobot_pump` nothing here uses). Pointing the buffer at
`../../../franka_description` works in Drake and VTK but **trimesh refuses
it** — its resolver will not follow a URI out of the glTF's own directory. An
asset package only some loaders can open is not an asset package.

Result: `assets/system_model/` is **self-contained, 18.1 MB**, and opens in
Drake, VTK, trimesh, or anything else that reads glTF 2.0, with zero dangling
references. `scripts/check_system_model.py` §8 and
`test_no_gltf_asks_for_a_texture_that_is_not_there` both pin it.

## 5. Collision geometry — the unaudited spheres are gone

`assets/proposed_rig/installation.urdf` carries the vendored Panda's **66
collision spheres per arm**. Nothing in this repo has ever audited them, and
they are not the model any certified number was earned against — the
generator's own header says so. **They are not in this model.**

Two geometries replace them, and both are backed by measurement:

**`installation.urdf` — the manufacturer's collision shells.** Ten OBJs from
`vamp/resources/panda/meshes/collision`, vendored byte-identically.
`scripts/collision_audit.py --validate` compares each shell's AABB with the
FR3's own collision box in the station's `fr3_franka_hand.urdf`: eight of nine
agree to **≤ 0.5 µm** and link6 to **0.344 mm**, i.e. the Panda and FR3
collision shells are the same object. Drake reports their convex hulls at the
same volume to 0.1 % on nine of ten (link6 4.2 %) — they are **already convex
shells**, so nothing is lost by using them as URDF collision, and Drake's
signed-distance queries work on them directly.

**`installation_capsules.urdf` — the audited capsule set.**
`selfcoll.BODY_CAPSULES`, 31 per arm, emitted as `drake:capsule`. Every radius
was fitted to those same manufacturer meshes by
`scripts/self_collision_audit.py`, and each row in the source records the
exact mesh maximum it rounds up from.

Two opinions on the same question, from the same measurement, in the same
frame. Read a disagreement between them as a real result.

**They are not interchangeable at the fingertips.** `selfcoll.BODY_CAPSULES`
covers link0–7 and the hand; it has no row for `panda_link8` (a pure frame) or
for either finger, because the finger is gripped shut around the holder and
the holder's own envelope is what the planner watches there. So the capsule
variant gives those three links **no collision geometry at all**, where the
mesh variant gives the fingers their shells. If you are asking a question
about the fingertips, ask it of `installation.urdf`.

The tool is a cylinder set in both (3 envelope + 1 graphite). The cage is
boxes. **No sphere carries a collision role anywhere in either file.**

One shell needed care. `finger.obj` is a single mesh used for both fingers and
it lies **entirely on one side** — y ∈ [−0.0001, 0.0264] — so the right finger
has to carry the same `Rz(180)` mirror its *visual* carries, or its collision
geometry sits up to **52.8 mm** from the finger. The generator derives it from
the vendored URDF's own visual origin rather than hardcoding it, and
`test_every_collision_shell_lands_on_its_own_link` holds every shell's AABB to
its link's visual (link0 exempted, because its disagreement *is* the base-cable
finding of §8.2).

## 6. Reconciliation with `mounts.py` — the re-cert queue

`aris_sixarm/mounts.py` is **not changed by this work**. It carries the
schematic keep-out envelope every certified number was earned against.
`system_model.reconciliation()` lists every place the two disagree, machine-
readably, in `model_manifest.json`:

| item | Δ | which way it cuts | action |
|---|---:|---|---|
| **ceiling datum** | 716.38 mm | code is **conservative** (booms ~700 mm too long) | survey the room; the fabricator's post is 688.6, not 1435.0 |
| **drop cluster vs boom column** | 193.80 mm | **neither contains the other** — 393.8 × 152.4 real vs a Ø200 column; measured per body, all 60 pieces escape, worst **185.55 mm** | **RE-CERT REQUIRED before fabrication** |
| mount plate thickness | 37.30 mm | conservative **in thickness only** — the modelled plate is centred on the J1 axis and the real one sits 25.15 mm off it, so in plan it escapes by **25.06 mm** | re-certify at the true offset |
| steel below the mount plane | 34.98 mm | model has steel the code does not; 60 mm of the 95 mm chain gap still spare | confirm at re-certification |
| **base cable pass-through** | 230.70 mm | **neither model has it, and the steel to be cut is not drawn** | §8 |
| mount height | 0.00 mm | **CLOSED 2026-09-10** — `docs/BUILD_SHEET.md` re-issued at the 970.0 in force, at the corrected datum | none unless `h` moves again |
| cage legs | 1651.00 mm | model has steel the code does not; stands 190 mm clear of the canvas | confirm once count and position are decided |

**The two that block fabrication are the drop cluster and the cable
pass-through.**

### RE-CERT-PENDING, measured per body

The label is computed from the geometry, not assigned by hand:
`system_model.recert_escape_mm` asks how far each body reaches outside
`mounts.arm_mount_boxes` — the schematic envelope every certified number was
earned against — and the manifest carries the answer on every body.

| hardware | count | escape past the modelled keep-out |
|---|---:|---|
| gussets | 24 | 135.25 – **185.55 mm** |
| drop posts | 24 | 71.75 – 122.05 mm |
| clamp stacks | 6 | 38.15 mm |
| **base plates** | 6 | **25.06 mm** |

**All 60 pieces are outside it, the plate included** — and the plate is worth
calling out, because the re-issued layout sheet reads it as *inside*. It
compared thicknesses (12.7 real against 50 modelled) and that is true; in
**plan** it escapes by 25.06 mm, because the modelled plate is centred on the
J1 axis and the real one is offset 25.15 mm off it. Nothing here is inside
what was certified.

## 7. The pen holder — RED FLAG

The placement is **inferred, not read**. The 2026.08.19 CAD delivery is eight
printed parts as STL + SLDPRT pairs with **no assembly file**.
`rig_final.penholder22_T_hand` puts the grip centre on the hand TCP and aims
the bore along the **planner's** TCP-to-tip ray.

Two things that used to be wrong here are now right.

**The housing was mounted end-for-end** until 2026-09-03 — the pen left the
model at the tail land instead of through the cap. **§7c is that correction,
and its re-certification.** The grip centre, the post axis, the bore direction
and the pen tip are all unchanged by it; the housing body, the cap, the
graphite and the collision cylinders all moved.

**And the tip was in the wrong place twice.** A photo of the real gripper
arrived on 2026-09-03 and put the pen tip about **5 cm below the bottom edge
of the Fat Franka Finger blades' contact plates**, not 15 cm below the hand;
read again on 2026-09-04 it put the whole holder at the plates' **far end**,
not on the finger centreline. **§7e is both corrections.** The tool transform
is now `PEN_EXT_HOLDER = 0.0588421` and `PEN_LAT_HOLDER = 0.1253421`, with the
grip centre at `panda_hand` (0.0665, 0, 0.1034).

**And the 22° is gone.** The housing's own machined flats clock at **23.0°**
(measured off the STL) and, since 2026-09-07, so does the bore this model
draws. That disagreement had been the biggest open question in this file since
2026-08-25 — a 22° discrepancy with no mechanism to live in — and it closed
from the direction nobody was checking: the **square block** the fingers pinch
is flush with the blades only when the bore is at 23°, and the photograph
shows it flush (§7e). `extract_penholder22_meshes.py` has printed that
disagreement on every run for a fortnight; it now prints **−0.00°**. The model
was drawing the bore along a planner ray that was only ever an assumption.

### 7a. The assembly was found, and it changes the answer

The lead this section used to end on has been followed.
`raw_slack_file_dump/Pen holder cad(1).zip` nests
`Natural hold assembly - closed.zip`, and inside it is
`Natural hold assembly - closed.SLDASM` — the **complete 10° "natural hold"
build**: housing, sleeve, cap, Conté pencil, Mil-Spec spring and **two FR3
fingertips, mated to the housing**. (The files are the SolidWorks 2024/25
container: nibble-swapped stream names, raw-deflate payloads. The component
transforms come out of `swXmlContents/COMPINSTANCETREE`, the per-part
bounding boxes out of `Contents/DisplayLists`, and both cross-check against
the STLs of the parts this delivery does ship. `scripts/read_solidworks.py`
is that decode, so none of what follows has to be taken on trust:

```
# -j, and ignore the "stripped absolute path spec" warning: the outer zip has
# a stray "/" entry, and everything else extracts
unzip -j '../raw_slack_file_dump/Pen holder cad(1).zip' -d /tmp/holder
unzip -j '/tmp/holder/Natural hold assembly - closed.zip' -d /tmp/holder
python3 scripts/read_solidworks.py asm \
    '/tmp/holder/Natural hold assembly - closed.SLDASM'
```
)

Resolved, that assembly says four things, all of them exact:

| what | value |
|---|---|
| post axis vs finger travel | **0.0000°** — they are the same axis |
| bore vs finger travel | **90.0000°** |
| bore vs the hand's approach axis | **10.0000°** — the housing's own file name |
| grip centre from `panda_hand` | **103.26 mm**, against the stock TCP's 103.4 |
| jaw gap at the seated fingertips | **36.0008 mm** |

**There is no cradle.** That was the escape hatch and it is gone. The
fingertips are stock FR3 tips — `Franka_Finger_FR3 Fingertip only.SLDPRT`,
drilled for one `93514A130` brass insert and nothing else — and they seat
**7.000 mm inside the mount post's own 18 × 18 mm end sockets**. Nothing
between holder and hand can absorb a rotation.

So on the build that *has* an assembly, the angle in the housing's file name
**is** the pen's lean out of the hand's approach axis, and the grip centre is
the TCP to 0.14 mm. This delivery's housing is named `22 deg` and its flats
measure **23.00** (independently confirmed here off the STL: the post socket's
half-width along the bore direction is 9.78 mm = 9.0 / cos 23.03°). Same
naming convention, same sockets, same square-seated tips.

**That was the verdict — "the pen leans 23°, not 45°" — and 2026-09-03
overturned it.** Not because the CAD is wrong: it is right about *that* build.
It is overturned because **that build is not what is mounted**, and because
the lean it implies does not physically fit under the hand.

- **The photo of the real gripper shows no fingertips at all.** The Fat
  blades' bare plates clamp the post's **end faces**. Nothing is seated in a
  socket, so nothing transmits the housing's 23° clocking to the hand: the
  clocking is free and the lean is whatever the assembler's hands set.
- **And the hand sets a floor on it.** 55.1 mm of barrel stands behind the
  grip; there are 37.4 mm between the grip and the hand's underside. Against
  the manufacturer's own hand collision shell the raw housing STL is inside
  the hand at any lean **below 35.17°** (−11.90 mm at 23°) and clears by
  **6.16 mm at 45°**. A 23° build would pass 12 mm of printed barrel through
  the gripper's casting.

So the lean stays **45°**, now for a reason rather than by inheritance. What
moved instead is the *depth*: at the tip the photo shows,

| | old (0.110 / 0.110) | **now (§7e)** |
|---|---:|---:|
| lateral offset from TCP | 0.110 m | **0.0588421 m** |
| axial depth from TCP | 0.110 m | **0.0588421 m** |
| reach from the grip centre | 155.563 mm | **83.215 mm** |
| graphite past the cap's outer face (§7c) | 125.6 mm | **53.2 mm** |
| …as the photo's own foreshortened view reads it | 110.0 mm | **37.6 mm** |

**72.3 mm of tip position.** Note also that the **sign** is a mounting choice,
not a CAD fact — the post is square, so the holder seats either way up and
the lean is ±45°.

One thing the old text asserted and should not have: that
`frames.PEN_LAT_HOLDER` was *gate-validated against a real touchdown*. It
never was. `frames.py` has always said what it is — **USER-SPECIFIED**,
2026-08-25, from the estimate "tip ~15 cm below the bottom of the gripper's
white housing" — and only the **inline** pen's axial `PEN_EXT = 0.110` carries
a touchdown (gate B, MZ 0.924). That wording is corrected wherever it appeared
(here, §7c, §7d, `system_model.OPEN_QUESTIONS`, `rig_final`, README).

**What is still open: which build ships — and the running robot says it is not
the one that was assembled.** `Aris_Kindt/franka_control_gui.py` closes on the
holder with `width = 0.0432`, `epsilon_inner = 0.0`, `epsilon_outer = 0.08`,
and libfranka calls a grasp successful only when the measured opening exceeds
`width − epsilon_inner`. So **43.2 mm is a lower bound on the real jaw gap**,
and the 36.0 mm this CAD gives would report failure every time. 50 mm does
not — and 50 mm is the post's *bare ends*, which is what the newest finger
part clamps: `Fat Franka Finger v250904` is an 18.4 × 90 × 50 mm blade that
replaces the whole stock finger, and 18.4 mm cannot enter an 18.0 mm socket.

So the deployed configuration is **Fat blades on the post's bare ends**, not
stock tips seated in the sockets at 36 mm — which the photo now confirms
directly. It is why the model's 0.018 is the *assembly's* number and not the
rig's. What it does **not** support is the guess "at ~50 mm": §7d's arithmetic
rules that out, and §7e says what the reading should be instead.

One measurement still closes the transform: the perpendicular distance from
the mounted pen's tip to the gripper's approach axis. The model now says
**86.0 mm** (66.5 of it is the grip's own offset along the
blades, §7e); §7e turns any reading into a tip.

### 7b. The stack inside the bore

The internals are no longer omitted. `rig_final.penholder22_stack` places
them, VISUAL ONLY, in this order — which is the 10° assembly's order,
interface by interface re-measured on this delivery's own STLs:

```
x=0.00000  TAIL face, 17.00 mm land — the spring's stop
x=0.00334  tail shoulder — a flat 21.0 face lands here   [TAIL STOP]
           SPRING 9657K26, ⌀19.05/⌀14.97, free 50.81 → squeezed to 39.675
x=0.04301  [optional shim: 5.1 or 10.1 mm, ⌀21.0]
           SLEEVE "pen holder for clutches v1.00", 40.10, ⌀21.0
x=0.04811  CLUTCH "Creatcolor monolith graphite v1.01", 35.00, inside it
x=0.08311  the cap's 16.00 mm shoulder                    [FRONT STOP]
x=0.08510  the cap's outer face — AND THIS IS WHERE THE PEN LEAVES (§7c)
```

Why each interface is what it is:

- **Tail stop, and it is the spring's.** The bore necks to a 17.00 mm land at
  x = 0, and **17.00 will not pass the 19.05 mm spring** — that land exists to
  stop it, which is also how we know x = 0 is the tail and not the nose (§7c).
  Nothing 21 mm gets past x = 3.34 either. In the 10° assembly the spring's
  end coil lands 3.302 mm from that face; this housing's shoulder is at 3.34.
- **The spring bears on the discs.** ⌀19.05 over ⌀14.97 clears the 21.148 bore
  by 1.05 mm and lands on the 21 mm parts' end annulus, which the sleeve's and
  the spacers' 15.0 mm bores are cut to match.
- **Front stop — the cap, and the pen goes out through it (§7c).** The cap is
  a threaded **collar**, not a lid — 36 mm flange, 11.4 mm long, open right through, 21.51 mm counterbore stepping to a
  **16.00 mm** shoulder. 16.00 is under 21.0, so the shoulder retains the
  stack. And the counterbore is not incidental: **21.51 on a 21.00 sleeve** is
  a clearance fit, 1.9 mm deep, and the sleeve's front 3.0 mm is exactly what
  crosses the housing's end face at 80.1 to reach the shoulder at 83.115. The
  cap is cut to receive this part.
- **Preload.** 83.115 − 3.340 = 79.775 mm of space, less the 40.100 sleeve,
  squeezes the spring to 39.675: **11.135 mm of preload**, with 11.175 mm left
  before coil bind. **That is the pen's compliance** — the spring pushes the
  sleeve toward the cap, and paper force on the graphite pushes it back and
  compresses the spring — and the two spacers are a
  **shim set** that sets it — one of {none, 5, 10} mm, giving 11.1 / 16.2 /
  21.2 mm of preload against 22.3 mm to solid. Both at once asks 26.3 and
  binds, which is why there are two spacers and not a stack of them.
- **The clutch is a split collet and the sleeve is its taper.** The sleeve's
  bore is a true cone — 15.030 growing to 16.290 mm at 0.01747 mm/mm, a
  surface of revolution to 1 µm — and the clutch's nose cone grows at
  0.01750 mm/mm. Matched tapers wedge. The clutch's free 17.066 mm does not
  enter a 16.290 mm hole, which is the point: it is slit, and going in closes
  it onto the 7.0 mm graphite. Drawing load pushes it deeper, i.e. tighter.
- **The 13.0 mm land** at the taper's small end — and in both spacers — is the
  bench extractor's guide: that tool is a 30 mm head on a **12.0 × 40 mm**
  pusher rod, 12.0 goes down 13.0, and 40 traverses the 40.1 mm sleeve. It
  stays omitted; it is a bench tool, not part of the mounted holder, so which
  end it is fed from is a workshop question and not a claim made here.

One free check on all of that: the render shows the spring through the
barrel — and it should. The housing has **two windows through the wall**,
spanning x ≈ 10…27 mm and roughly ±50…130° about the bore, which the outer
surface's own angular coverage measures directly (no material at all in those
sectors, at four sampled stations, and a closed wall at x = 8 and x = 30). The
spring is 3.34…43.0 mm along the bore, so it is exactly what you would see
through them.

**Collision: one cylinder more since §7c, and re-proved.** Every body *in the
bore* is inside the 21.148 mm bore, so the three coaxial cylinders
`PENHOLDER22["env_cylinders"]` that the housing and cap were fitted to still
cover them. The graphite does not stop at the bore, though — it runs right
through and out the back — so `tail_cylinder` joins them and the shipped hull
(`rig_final.penholder22_hull`) is **four**.
`rig_final.penholder22_internals_escape()` measures every drawn body against
that hull on every generator run and the generator refuses to write a URDF if
it is not **0.0 m**. It is 0.0 m, for all three shim settings.

### 7c. The housing was modelled end-for-end — FIXED, 2026-09-03

`PENHOLDER22["nose_x"] = 0.0` said the pen left the housing at x = 0. **It
leaves through the cap at the other end.** The model now says so, and this
section is the re-certification rather than the report.

Four independent things fix the sense, and every one of them is re-derived
here from `Natural hold assembly - closed.SLDASM` itself rather than from the
prose that used to stand in this section (`scripts/read_solidworks.py asm`,
the commands in §7a):

1. **The parts' own order along the bore.** Resolved, the assembly places, in
   its own millimetres: cap 1017.121 … 1028.558, sleeve 1020.121 … 1060.221,
   **spring 1060.121 … 1098.919**, housing 1020.855 … 1102.221. The spring is
   at the end **away from the cap**, so it pushes the pen assembly **toward**
   the cap and paper force compresses it. **That is the tool's compliance.**
   Modelled the other way round the spring pushes the pen into a rigid cap and
   there is none.
2. **The pencil, and the preview.** The Conté pencil spans 1000.121 …
   1174.735 — 174.614 mm of it, in an 85.100 mm barrel. It goes straight
   through: **17.000 mm of sharpened point past the cap's outer face** and
   **72.514 mm of blunt, flat-cut tail past the housing's tail face**. The
   assembly's own render shows exactly that (`read_solidworks.py dump …
   PreviewPNG`).
3. **The 17.0 mm land is the SPRING's stop, not the pen's.** 17.00 will not
   pass a 19.05 mm spring — that is the only thing in the holder it stops, and
   it is cut 2.15 mm long to do exactly that. A ⌀7 graphite or a ⌀8.5 pencil
   walks straight through it without touching it.
4. **The grip is 25.001 mm from the housing's cap end and 55.099 mm from its
   tail.** A robot pen grips close to its tip, and `docs/FINAL_RIG.md`'s
   independent extraction read the same 25 mm — which is why its 46.0 mm
   flange-to-tip reproduces the assembly's own 46.096 exactly, and the old
   55.1 mm grip-to-nose reproduced nothing.

**What changed.** `penholder22_T_hand` set `Xh = -u`, pointing the housing's
+X *away* from the tip; it now sets `Xh = u`. That is a **180° rotation about
the post axis and nothing else**: the post axis is still hand y, the grip
centre is still the hand TCP, the bore is still the planner's 45° ray, and
**the pen tip does not move by a picometre** (`check_system_model.py` reports
the same worst 5.259e-12 m over 25 configs × 6 arms as before the change (it
is 4.779e-12 today, because §7e then moved the tip on purpose), and
`test_the_flip_did_not_move_the_pen_tip` pins it).

| | end-for-end | **corrected** |
|---|---:|---:|
| grip → the cap's outer face, where the pen leaves | −30.001 mm (behind) | **+30.001 mm** |
| grip → the housing's own end face | −25.001 mm | **+25.001 mm** |
| grip → the tail face | +55.099 mm | **−55.099 mm** (behind) |
| graphite past the exit, at the tip of the day (155.563 mm) | 100.464 mm | **125.562 mm** |
| the pencil tail, past the tail face | not modelled | **72.514 mm, at the wrist** |

Note which face the graphite numbers are measured to. §7a and the old §7c
quoted **130.5 / 94.5 mm**; those are measured to the **housing's own end
face** at x = 80.100. The cap stands **5.000 mm** proud of it and the pen
leaves through the cap's outer face at x = 85.100, so past the real exit the
numbers are **125.562 mm** (at the 155.563 mm tip of the day) and **89.499 mm**
(the CAD's 23° lean at the same axial depth). Both are still true statements;
this section now says which is which.

**And the tip moved on 2026-09-03 (§7e), so the live number is neither.** At
`PEN_EXT_HOLDER` / `PEN_LAT_HOLDER` = 0.0588421 the reach is 83.215 mm and the
graphite past the cap is **53.214 mm** — 37.6 mm as a photo along the hand's
x axis reads it, against the 20–40 mm that photo shows. Nothing else in this
section moves with it: the housing's placement depends on the ray's
*direction* and on the grip centre, and both are unchanged, so the two housing
meshes are byte-identical across the change and every escape measured below
still stands.

The 10° assembly's own answer is **17.000 mm** past its cap's outer face,
measured above — and `docs/FINAL_RIG.md`'s **20.7 mm** is the same measurement
to the *housing's* end face, since that build's cap stands 3.734 mm proud
(46.096 − 25.361 = 20.735, and 46.096 − 17.000 = 29.096 puts its cap face
29.1 mm from the grip against this build's 30.0). The two numbers agree; the
face they are measured to does not. Whatever is really on the arms, it is not
this housing with a short stick.

**The pencil tail is now a body.** It was in the assembly and in the preview
and nowhere in this model. `rig_final.penholder22_tail()` places it: a ⌀7
cylinder (this build's clutch bore, not the Conté's ⌀9.74) from the housing's
tail face back **72.514 mm toward the wrist**, with its own envelope cylinder
`tail_cylinder` fitted the way `env_cylinders` are — radius `lead_r_coll`,
1 mm of axial lead-in, escape proved at 0.0 m on every generator run by
`penholder22_internals_escape`. The holder's collision model is therefore
**four** cylinders now, not three.

What is MEASURED about the tail is the 72.514 mm overhang. What is **ASSUMED**
is that this build shows the same one — and **§7e made the assumption cheap**.
At the corrected tip a stick showing 72.514 mm of tail is
72.514 + 85.100 + 53.214 = **210.8 mm** long, which a ⌀7 graphite stick can
be; a 175 mm Cretacolor Monolith in the same holder shows **36.7 mm** of tail.
At the old 155.563 mm tip the same arithmetic asked for **283.176 mm** and no
stick was that long, while the assembly's own 174.614 mm pencil pushed out to
that tip ended **36.049 mm inside** the barrel with no tail at all. That the
two readings could not be reconciled was one of the several things wrong with
that tip.

**And the tail is what nearly touches the hand, which is what the photo
shows.** At the 45° lean the tail's closest approach to the manufacturer's
hand shell is **18.36 mm** — at its *start*, by the housing's tail face, not
at its far end, which swings out in hand x and clears by 59.04 mm. The
housing's own tail face sits **1.52 mm** below the hand's underside plane. In
a photo taken along the hand's x axis the whole tail is foreshortened onto the
hand and reads as "almost touching", which is exactly what the photo says.

#### What the re-certification cost

**No gate on the certified programme flipped, and no constant moved.** Both
statements need the same paragraph, because the second is why the first is
worth reading twice.

`scene_check`, `validate`, `coordination` and `selfcoll` do not plan against
the holder's CAD at all: the tool is the two-capsule L of
`rig_final.STATIC_CAPSULES_LAT` — TCP → bracket corner → tip, both at
r = 0.050 — and that is unchanged. Re-run on the 100 % programme
`out/csail_schedule_h094_v14.*`, `scene_check` returns **exactly** what it
returned before the flip:

| | before | after |
|---|---:|---:|
| min inter-arm clearance (margin 80 mm) | 80.73 mm, arms 13↔17 | **80.73 mm, arms 13↔17** |
| self-collision (margin 20 mm) | 26.1 mm | **26.1 mm** |
| frame clearance (margin 50 mm) | 54.1 mm | **54.1 mm** |
| neighbour base column (margin 80 mm) | 136.4 mm | **136.4 mm** |
| verdict | PASS | **PASS** |

**But the L-capsules no longer contain the holder, and that is the bill.**
`scripts/collision_audit.py --part tool` measures the raw CAD against them:

| | worst escape | bracket radius it would need |
|---|---:|---:|
| housing + cap, mounted end-for-end | −1.720 mm (contained) | 0.0483 |
| housing + cap, **mounted correctly** | **+6.546 mm** | **0.0565** |
| **+ the pencil tail** | **+77.661 mm** | **0.1277** |

The tool capsule was CAD-validated as containing the holder in 2026-08-26
(`rig_final.STATIC_CAPSULES_LAT`, `coordination.py`); it was validated against
a holder that was on backwards. Grow the bracket capsule to the radius that
would contain the corrected body and re-run the same programme, and the gate
does flip:

| bracket r | min inter-arm | verdict |
|---|---:|---|
| 0.0500 (shipped) | 80.73 mm, arms 13↔17 | PASS |
| 0.0565 (housing + cap) | **79.12 mm**, arms 2↔31 | **FAIL** by 0.88 mm |
| 0.1277 (with the tail) | **−18.21 mm**, arms 31↔97 | **FAIL**, and the 50 mm frame gate fails too (13.4 mm) |

Six of the fifteen arm pairs sit within 3 mm of the 80 mm margin, so there was
never much room: the programme was conducted at 80.7 mm against an 80 mm bar.
**Nothing here widens a capsule.** `BRACKET_R_LAT` and `PEN_R_LAT` are what
the certified programme was gated at, and re-deriving them is a
re-certification with its own gate — the same rule that keeps the 45° planning
transform in place two sections up. The escape is reported, in the manifest
(`tool.end_for_end_fixed`), by the audit on every run, and here.

**And it is the tail that makes it expensive.** Say it precisely: the housing
alone breaks the 80 mm inter-arm gate by 0.88 mm; the tail breaks it by
98.2 mm. If the tail is real, the tool capsule is not a 5 cm sausage and the
programme has to be re-conducted, not re-labelled. That is the strongest
argument yet for taking §7a's ruler measurement — and for one more: **how much
stick is actually loaded, and does any of it stand out of the back.**

#### Self-collision at the parks

Both bodies are welded to the hand, so tool-vs-hand and tool-vs-link7 are
**pose-invariant** — the same at a park pose as anywhere else. Measured
(scratch geometry, the same capsule table `selfcoll` ships):

| | housing, before | housing, after | **the tail** |
|---|---:|---:|---:|
| nearest `hand.*` capsule (r 0.040–0.050) | inside | inside | **+3.36 mm** |
| nearest `link7.*` capsule, six parks | +50.22 mm | +34.02 mm | **+12.83 mm** |
| manufacturer's hand shell | 69.16 mm | ~~49.21 mm~~ **6.16 mm** | ~~43.02 mm~~ **18.36 mm** |
| manufacturer's link7 shell | 79.75 mm | 55.35 mm | **45.88 mm** |
| stock fingers at `FINGER_FIX` | 1.23 mm | 0.08 mm (the grip) | 32.56 mm |

**Two of those were wrong and are corrected here (2026-09-03).** The
hand-shell row for the corrected housing and for the tail could not be
reproduced. Re-measured, in the way the reader can repeat: the raw housing STL
placed by `rig_final.penholder22_T_hand`, against the convex hull of
`assets/system_model/meshes/collision/hand.obj` in the `panda_hand` frame (the
shell is convex to 0.01 % by volume, so the hull is the shell), gives
**6.155 mm**; the shipped 8000-face housing mesh gives **6.239 mm**; the ⌀7
tail gives **18.36 mm**, at its *start* by the tail face rather than at its
far end (59.04 mm). Those are the numbers §7e's lean argument rests on, and
they are the reason the photo's "the tail almost touches the hand" is a
statement this model agrees with rather than one it contradicts.

**No gate flips, and the reason is structural rather than lucky.**
`selfcoll.SELF_PAIRS` watches a pair only when the two bodies are at least
`WATCH_CHAIN_D = 4` joints apart, and the tool and the hand are 0 apart and
the tool and link7 are 1 — pairs the module deliberately does not watch,
because "the hand is a fixed flange on link7 … what keeps them out of each
other is the FR3's own joint limits and the casting geometry". So the
+3.36 mm and +12.83 mm are **under** `SELF_PLAN_MARGIN` (23 mm) and nothing
asks about them. Against the metal itself there is no
interference: the tail misses the hand shell by **18.4 mm**, link7's by
45.9 mm and the fingers by 32.6 mm, and the housing clears the hand shell by
**6.2 mm**. Those are real clearances and they are small — which is the point
§7e makes about the lean.

**And the fingers moved.** `gen_system_model.FINGER_FIX` was 0.0285; it is now
**0.018**. 50 mm of post less 2 × 7.000 mm of socket is 36.000, and the
assembly puts the two fingertip grip faces 36.0008 mm apart. 0.0285 is the
fingertip's **back** face — 3.5 mm proud of the post's end — and a finger
parked there holds nothing, because 57 mm is 7 mm wider than the post is long.
At 0.018 the URDF's `finger.gltf` (which includes its own tip) lands where the
assembly puts it in all three axes: the finger's distal 18.1 mm covers
`panda_hand` z 94.2…112.3 mm, and so does the CAD block.

Meshes are re-decimated from the raw STLs, weld-then-decimate, 8000 faces per
part:

| part | raw faces | out | extent lost |
|---|---:|---:|---:|
| housing | 174 772 | 8 000 | **0.1122 mm** |
| cap | 30 770 | 8 000 | **0.0048 mm** |

The 3-cylinder housing envelope is re-proved against the result on every run —
the housing's own three, never the tail's fourth: worst escape **+0.109 µm**,
against a 10 µm bar. (Not zero: the envelope's radii
were fitted to a coarser 5000-face decimation where every vertex sat inside;
a finer mesh follows the true barrel more closely and pokes a tenth of a
printer layer proud of one band. It is reported, not hidden —
`meshes.holder_decimation.envelope_worst_escape_m` in the manifest.)

### 7d. The Fat Franka Finger — the mesh arrived, and it moves the argument

`Fat Franka Finger v250904.STL` landed on 2026-09-02 (8234 faces, watertight,
sha256 `b1a79369…`, 18.4339 × 90.0003 × 50.000 mm). §7a had it from the SLDPRT's
bounding box alone and called it "a blade replacing the whole stock finger".
That is right, and the mesh says a good deal more.

**THE MOUNTING SENSE IS CONFIRMED BY THE PHOTO, 2026-09-03 — the model already
had it, and there is no other way to bolt the part on.** The photo of the real
gripper shows each blade's mounting **foot** outboard against the carriage,
the slanted **web** running down and inward from it, the flat **contact
plate** inboard of the foot, and the two blades converging toward the paper.
That is this placement exactly. In `panda_hand`, at `FAT_FINGER_FIX`:

| band | link y | `panda_hand` z | gap between the two blades |
|---|---|---:|---:|
| mounting foot | 18.500 … 26.500 | 62.24 … 76.24 | **70.867 mm** |
| rib crest | 8.066 | 93.75 … 94.24 | **50.000 mm** |
| contact plate | 10.650 … 14.500 | 94.24 … 112.24 | **55.168 mm** |

— a V that closes 15.7 mm over 50 mm of finger, feet apart at the carriages
and plates together at the paper. It reads as "outboard" only against the
*stock grip plane*, which is a different datum: the blade is 15.85 mm thick
from carriage face to contact face where the stock finger is 26.4 mm from
carriage face to grip plane, and that 10.65 mm of difference is exactly why
the plates sit further apart than stock fingers would at the same joint value.

**And the mirror is not merely unphotographed — it is impossible.** The only
other way to put a foot on the carriage flat is on the foot's *inner* face,
which runs the web outward and lands the contact plate at link y = **34.350**
instead of 10.650: two plates **68.700 mm apart at q = 0**, so closing them on
a 50 mm post would need q = **−9.350 mm**. Mounted that way round the blade
cannot grip this holder at any joint value. The rigid placement the photo
demands is the one the model already had, and it is the only one there is.

What the photo *cannot* settle is the 8 mm question one step in from that:
whether the foot's outer face lands on the finger's own back face (this model,
and the fit lands there to **0.097 mm** without being asked to) or 8 mm
further in on some other carriage flat. That is worth **16 mm of `width`**,
and `width` is the measurement that settles it — see (c) below and §7e.

**It is drawn in the FR3 fingertip's own frame, and that fixes the transform
with no free parameter.** The same zip carries
`Franka_Finger_FR3 Fingertip only.SLDPRT` — the stock tip the 10° assembly
seats in the post sockets — and the two parts share one coordinate system:

| | |
|---|---|
| the fingertip's `93514A130` brass-insert axis (from the `Rodgers fingertip` sub-assembly) | (y, z) = **(0.0004, 55.000)** |
| this part's contact-plate hole | (y, z) = **(0.0002, 54.9998)** |
| the fingertip block | x 68.0578 … 78.5578, an 18.1156 mm square |
| this part's plate | occupies exactly that z band, 0.1502 mm outboard of the tip's back face |

One caveat that is not a rounding error: **the STL and the SLDPRT do not share
a datum.** The STL is exported **10.5000 mm along +y** off the part origin (x
and z agree to 0.0002 mm; only y moves), and it is the SLDPRT's datum that puts
the plate hole on the finger centreline. `rig_final.FATFINGER["stl_y_shift"]`
carries it.

So the placement is fixed by placing the *fingertip*, which §7a already did:

```
link x =  cad y                    across the hand — the tip is centred
link y =  0.0785578 - cad x        the jaw axis; cad x 78.5578 IS the grip plane
link z =  cad z - 0.0101579        the finger length; the tip's distal face is
                                   the finger mesh's own tip
```

a proper rotation (det +1, `Rz(−90°)`) and a translation. **Residuals against
the manufacturer's own finger: 0.155 mm worst** — foot outer face vs the
finger's back face +0.155 (visual) / +0.097 (collision), plate face vs the
fingertip's back face +0.150, grip plane vs the finger's inner face +0.084 /
+0.133, tip z 0.000 / +0.051. The two manufacturer meshes disagree with *each
other* by 0.051 mm, so 0.155 is about as tight as this can be held.
Independently: the plate band's centre lands at `panda_hand` z = **103.242 mm**
against the 10° assembly's own grip centre of 103.26 and the stock TCP's 103.4.

**In the finger link frame, then:**

| feature | link x | link y | link z |
|---|---|---|---|
| mounting foot (×2) | −10.500 … 9.500 / 59.500 … 79.500 | 18.500 … 26.500 | 3.842 … 17.842 |
| its two M4 holes (Ø4.296 waist, Ø7.293 counterbore **both** faces) | ±6.000 | along y | 11.842 |
| slanted web | full | 26.500 → 8.066 | 17.842 … 35.842 |
| **rib** (the web's own top, flat-cut) | full | **8.066** | 35.351 … 35.842 |
| **contact plate** | −9.000 … 79.500 | **10.650** … 14.500 | 35.842 … 53.842 |
| its two Ø6.000 holes | 0.000 and 69.000 | along y | 44.842 |

**Why there are two of everything.** The part is its own mirror image about
y = 34.5 — 0.44 mm over every mating feature, 99.05 % by volume, the two ends
differing only in the plate's outer edge (+79.500 one end, −9.000 the other,
1.5045 mm) and one R5 corner. Bolt the near foot down and the blade reaches
+69 mm along the finger's x; bolt the far foot down and it reaches −69. Those
are the two placements a LEFT and a RIGHT finger need if both blades are to
reach **the same way in the hand** — and no single 180° rotation gets you
there, because the map that swaps the feet is improper. Only the part's own
mirror symmetry makes it realisable. One printed part, two fingers.

**The rib is the thing nobody had.** Along the plate's proximal (hand-side)
edge the web's outer face runs *past* the contact plane and is flat-cut at
cad x = 70.4915: **2.5839 mm proud, for the full 90 mm, at every station
sampled.** It is the innermost feature on the part, so the jaw gap between two
of these is

```
2 q + 21.3004 mm   at the plates      (q = the finger joint; 2 q is libfranka's `width`)
2 q + 16.1326 mm   at the ribs
```

#### The deployed-grasp question

**(a) Nothing on this finger locates the post.** The contact face is one flat
plane — 1516 mm², the part's largest — broken only by the two Ø6.000 holes and
the R5 corners. The holes are 69.000 mm apart where the post is 26 mm square,
so at most one can ever face it; they are mirror twins of a single feature, not
a two-point pattern. And the post has nothing to receive a pin: rays down the
22° housing's post axis hit **solid material at z = 5.426 and 44.574** — the
socket floor is a chamfered cone, not a bore. The Ø6.000 hole is the
installation hole for the same `93514A130` flanged barbed insert the Rodgers
fingertip carries, on the same axis to 0.0002 mm; it is a fastener hole, and
there is nothing on the post for a fastener to reach.

Two readings of the plate follow, and **the photo chooses A**: no fingertip is
fitted on the real gripper, so B is out as a description of what is on the
arms — though the four measurements behind it are what fixed the transform in
the first place, and they stay written down.

- **A — the plate grips. THIS IS WHAT IS MOUNTED.** Then it never reaches the
  post. Closing on the bare
  50 mm ends, the **rib** lands first, at q = 25.000 − 8.0663 = **16.9337 mm**,
  leaving the plates 2.5839 mm off. Measured, not argued: bisecting the blade
  against the committed holder meshes returns the same 16.9337 mm and names the
  rib crest as the touching vertex.
- **B — the plate carries the stock fingertip — RULED OUT BY THE PHOTO**, but
  four measurements point at the plate being a *seat* for one. The flat band between the rib and the plate's far edge is **18.000 mm**
  and the FR3 fingertip is an **18.1156 mm** square. The Ø6.000 hole is centred
  in that band on the tip's own insert axis. The plate face is 0.1502 mm
  outboard of the tip's back face, i.e. exactly a seat. And with the tips
  seated in the post's sockets at the assembly's own 36.0008 mm, the rib clears
  the post by **0.916 mm**.

**(b) The clocking is free, and the lean is set by hand — and then by the
hand.** Under reading A two flat plates on a square post's ends leave the
rotation about the jaw axis unconstrained. Under reading B it is constrained
*only* if a fingertip is fitted **and** seated in the housing's own 18 × 18 mm
socket. **The photo shows no fingertip fitted**, so reading A is what is on the
arms and the clocking is free.

Free about the jaw axis is not free in the room, though. Whatever the
assembler set, the housing's 55.1 mm of barrel behind the grip has to fit in
the 37.4 mm between the grip and the hand's underside, and that puts a floor
under the lean: **35.17°**, measured by sweeping the raw housing STL against
the manufacturer's own hand collision shell. 23° is 11.90 mm inside the hand;
45° clears by 6.16 mm. So the two hypotheses are no longer symmetric —
**45° is the one that fits and 23° is ruled out** — and §7e is where that
lands.

**(c) The jaw opening, and the 43.2 mm does not fit.** `width` is 2 q, the
*stock* grip plane's opening; each build adds back how far its real contact
face sits outboard of it. **The mounting sense does not change one number in
this table — the photo confirms the sense the model already had (see above), so
these are the same six they were:**

| build | libfranka `width` | photo | vs the GUI's 0.0432 |
|---|---:|---|---|
| **fat plates on the bare post ends** | **0.0287** | possible | FAIL |
| **fat ribs on the bare post ends** (the plates cannot reach) | **0.0339** | **what the model draws** | FAIL |
| fat plate + fingertip, seated in the sockets | 0.0357 | ruled out — no tip fitted | FAIL |
| stock fingertips seated in the sockets (the 10° assembly) | 0.0360 | ruled out | FAIL |
| fat plate + fingertip, flat on the bare post ends | 0.0497 | ruled out | pass |
| stock finger faces flat on the bare post ends | 0.0500 | ruled out — not stock fingers | pass |

**So the reading to expect on the robot today is 0.0339 m, or 0.0287 m if the
post is seated distal of the rib and the plates land flat.** The photo has cut
six rows to two and both of them FAIL the GUI's 43.2 mm — which is not a
contradiction (see the fallback below) but it is the open item.

The fat blade's reachable gap is 21.300 … 101.300 mm, so it can *reach* 50 mm —
at `width` 0.0287, which `grasp(0.0432, epsilon_inner=0.0)` would report as a
failure every time. **§7a's guess — "Fat fingers flat on the post ends at
~50 mm" — is arithmetically ruled out by the plate offset the mesh now gives.**

**Read it as a ruler instead.** `width` measures where the contact face is:
a reading of *w* puts it (50 − 1000·*w*)/2 mm outboard of the stock grip plane.

| `width` read | implied contact offset | what it would mean |
|---:|---:|---|
| 0.0287 | 10.650 mm | the model's plate offset exactly |
| 0.0339 | 8.050 mm | the model's **rib** — what it draws |
| 0.0432 | 3.400 mm | the blade sits 7.2 mm further in than modelled |
| 0.0497 | 0.150 mm | a fingertip is on the plate after all |
| 0.0500 | 0.000 mm | the contact face IS the stock grip plane |

A reading near 0.0432 or 0.0500 would say the foot is bolted to a carriage
flat 7–11 mm inboard of the stock finger's back face, i.e. the 8 mm question
the photo cannot settle. A reading of 0.0339 says the model is right as it
stands.

One honest weakening of the 43.2 mm evidence while we are here: the GUI's menu
path falls back to `open_gripper(width=0.001)` when the grasp reports failure,
so a failing `grasp` still ends up holding the pen and nobody would notice. The
constant is best read as "somebody tuned this until arm 13 reported success",
which is evidence, not proof.

**(d) Where the post sits along the plate, and where that puts the grip.**
Under reading A — bare plates, no fingertip, which is what the photo shows —
the post is a **26 mm** square closing on an **18.000 mm** flat band, with the
**2.5839 mm** rib standing proud along the band's proximal edge. So the post
cannot be centred on the band and lie flat: 4 mm of it would overhang onto the
rib, and the rib would hold the plates off. There are exactly two seatings:

| seating | post spans (`panda_hand` z) | grip centre | what touches | `width` |
|---|---|---:|---|---:|
| **post centred on the plate band** | 90.24 … 116.24 | **0.103242** | the **rib crest**, 2.58 mm short of the plate | 0.0339 |
| **post pushed distal, butted against the rib** | 94.24 … 120.24 | **0.107242** | the **plates**, flat, over 18 of the post's 26 mm | 0.0287 |

The second is the assembly move a pair of hands would make — drop the holder
in until its post edge catches the rib's ledge, then close — and it leaves
8.0 mm of post hanging past the plate's far edge, unsupported but clear. The
first is what the model draws (`FAT_FINGER_FIX = 0.0169337`, the tightest a
bare blade closes on this holder), and it is the conservative one. Either way
the plate band spans `panda_hand` z **94.242 … 112.242 mm** and its **bottom
edge is at 112.242 mm**, which is the datum §7e's tip is measured from.

The model's own holder keeps the grip **height** on the hand TCP,
z = 0.1034 — 0.158 mm out from the centred seating and 3.842 mm in from the
butted one. Nothing here is worth moving the TCP convention for.

**Along the plate is a different question, and 2026-09-04 answered it.** The
plate is 90 mm long and the post is 26 mm, so where the post sits along hand x
is free as far as the CAD is concerned — and the photograph is not: the holder
is clamped at the plates' **far end**, the post's outer face flush with the
plate's own far edge at link x 79.500, so the grip is at **x = 66.500 mm**.
See §7e. (The plate's far corners carry an R5, so the last ~1.8 mm of the
post's corner at the plate's distal edge overhangs a rounded edge rather than
flat material. The post is held off by the rib there anyway.)

| | `panda_hand` |
|---|---|
| grip centre (as drawn) | (**0.0665**, 0, **0.1034**) — the plates' far end, at the TCP's height |
| post axis | hand **y**, by construction |
| bore | ⊥ the post, so it lies in the hand's x–z plane |
| lean out of hand z | **23.00°** — the post's own clocking, which is the only lean that puts the block square to the hand (§7e) |

Pen tip, at the axial depth the photo gives (§7e):

| lean | tip, `panda_hand` | housing vs the hand shell |
|---|---|---:|
| **23° — the housing's flats, and what the model uses since 2026-09-07** | **(0.0860369, 0, 0.1494262)** | **+13.84 mm** |
| 45° (what it drew for three days) | (0.1253421, 0, 0.1622421) | +7.78 mm |

The 63.3 mm of §7a is gone, and not because the CAD changed: there is no
mechanism that *could* carry the 22° — not a cradle (§7a), not the fingertip
sockets (§7a, and the photo shows no tip fitted), not this blade. **The
"no room for the barrel at 23°" half of that argument is withdrawn**: it was
true on the finger centreline and is not true at the placement the photograph
shows (§7e).

**The measurements that close it — still TWO, and one of them is now urgent.**

1. **The perpendicular distance from the mounted pen's tip to the gripper's
   approach axis.** The model says **86.0 mm** — 66.5 mm of grip offset
   along the blades plus 19.5 mm of bore (§7e); §7e turns any reading into a
   tip, at either lean.
2. **The gripper's own `width` while the pen is held** — read it off
   `franka::GripperState`, or caliper the jaw. **Expect 0.0339, or 0.0287.**
   Anything near 0.0432 or 0.0500 says the blade's foot is bolted 7–11 mm
   further inboard than modelled, which is the one thing about the blade the
   photo could not settle.

#### In the model

`installation_fatfingers.urdf` — the same scene as `installation.urdf` with the
blade in place of the stock finger. `python3 scripts/gen_system_model.py urdf
--fingers stock|fat|both`; `SM.urdf_path(fingers="fat")`. The three original
URDFs are **byte-identical**; `model_manifest.json` and `meshes/MESH_SOURCES.json`
grow by one record each.

- **Mesh** `meshes/fatfinger/fatfinger_leftfinger.obj`, in the LEFT finger's
  link frame, weld-then-decimate 8234 → 8000 faces, **0.0000 mm of extent
  lost**. The right finger uses the same file under `rpy = (0, 0, π)`,
  `xyz = (0.069, 0, 0)` — which *is* the other foot bolted down.
- **Collision: four axis-aligned boxes**, measured off the vendored mesh on
  every run (three contiguous z bands at the CAD's own steps, the foot band
  split at the part's mirror plane) and re-proved against its vertices, face
  centroids and edge midpoints. **Worst escape 0.000000 mm.** A convex hull was
  rejected: it swallows the Z's concavity whole.
- **Joint value** `FAT_FINGER_FIX = 0.0169337` — reading A's number, the
  tightest the bare blade closes on this holder. It is what the URDF *draws*,
  not a claim about the arms.

#### Does the blade escape the envelopes the planner trusts? — REPORTED, NOT FIXED

Worst over the whole finger-joint range (q ∈ [0, 0.040]), both fingers, against
the vendored mesh:

| envelope | stock finger | **Fat blade** |
|---|---:|---:|
| `selfcoll.BODY_CAPSULES` hand rows (r 0.040–0.050) | contained, 3.14 mm spare | **escapes by 49.93 mm** |
| `coordination.HAND_R = 0.104` | contained, 36.79 mm spare | contained, **0.51 mm** spare |

**The self-collision capsule set does not contain this finger.** That is not a
surprise and it is not a bug in the capsules: §5 already says
`BODY_CAPSULES` has **no finger row at all** — the finger is gripped shut
around the holder and the holder's own envelope is what the guard watches
there. A 90 mm blade reaching 69 mm sideways out of the hand is a different
object, and the capsule set was never asked about it. At the modelled grasp
(q = 16.93 mm) the escape is 41.17 mm.

`HAND_R` still contains it — but by **0.51 mm at full open**, which is a margin
and not a clearance. Nothing here is changed: no capsule radius,
`frames.PEN_LAT_HOLDER`, `FINGER_FIX`, the gate constants or the layout. If the
Fat fingers are what ship, the self-collision guard needs a finger row and
`HAND_R` needs re-deriving, and both are re-certifications with their own gate.

### 7e. What the photo says, and what one ruler reading would set

A photo of the real gripper arrived on 2026-09-03. It is the first evidence
about the *mounted* tool that is not CAD, and it settles three things, leaves
one open, and moves a constant. It was read twice, and the second reading —
2026-09-04 — moved a second constant and withdrew an argument. That correction
comes first, because it changes what the rest of this section is about.

#### The block is square to the hand, and the pen is short — 2026-09-07

Pete, reviewing the side view down the jaw axis, gave two corrections. The
second is a **measurement in disguise**.

> *"shorten the shaft of the pen and bring the tip closer to the cylindrical
> pen holder, I think it only juts out 3–4 cm max"* — and, once the
> orientation below was corrected and confirmed (*"looks great now"*),
> **about 2 cm**.
>
> *"in the picture the square part of the holder is flush with the metal part,
> so the entire pen-holder plastic bit that is being pinched is about 20
> degrees more upright"*

**"Square to the hand" IS "bore at 23°", and the housing's own STL says so to
0.00°.** The mount post is a 26 mm square section whose four flats are clocked
`PENHOLDER22["post_clock"]` = **23.00°** about the post axis relative to the
bore — measured, not assumed: the four large flats' normals sit at
**−113.00 / −23.00 / +67.00 / +157.00°** off the housing's own +X, which *is*
the bore. The post axis is hand y, so those normals live in the hand's x–z
plane. Then:

| bore lean | post-flat normals, off hand +z | the block |
|---|---|---|
| 45° (what the model drew) | −158 / −68 / **+22** / +112 | askew, by 22° |
| **23°** | **−180 / −90 / 0 / +90** | **square to the hand** |

So the block is flush with the blades' plate faces and edges at 23° and at no
other angle, and the 22° it was out is exactly the *"about 20 degrees more
upright"* Pete asked for. **The housing's file-name angle is the mounted lean
after all** — which is what §7a concluded on 2026-09-02, what the casing
argument of 2026-09-03 briefly overturned, and what the withdrawal of that
argument on 2026-09-04 left standing unopposed.

**And the tip now comes off the holder, not off the blades.** 20 mm of
graphite past the cap's outer face, which is 30.001 mm from the grip:

```
PEN_LEAN_HOLDER     = 23°                    the housing's own clocking
PEN_GRAPHITE_HOLDER = 0.020 m                "about 2 cm"
u                   = (sin 23, 0, cos 23)  = (0.390731, 0, 0.920505)
tip = grip + (0.030001 + 0.020) * u        = (0.0860369, 0, 0.1494262)
PEN_EXT_HOLDER      = 0.1494262 - 0.1034   = 0.0460262 m
PEN_LAT_HOLDER      = 0.0860369 m
```

**This SUPERSEDES the "tip 50 mm below the bottom edge of the blades" rule** of
2026-09-03 — and, at 20 mm, contradicts it: the derived tip sits **37.18 mm**
below the plate edge, not 50. The blades' edge was a plausible datum for a tip
nobody had measured; the holder is a better one, because the protrusion is the
thing a person can actually see and adjust. One corroboration that is not an
argument but is worth writing down: 20 mm of graphite + the 85.100 mm barrel +
the assembly's 72.514 mm tail is a **177.6 mm** stick, and a Cretacolor
Monolith is **175 mm**.

| | 2026-09-04 | **2026-09-07 (shipped)** |
|---|---:|---:|
| `PEN_LEAN_HOLDER` — the bore, at the grip | 45.00° | **23.00°** |
| `PEN_GRAPHITE_HOLDER` | — (a consequence) | **0.020 m, an INPUT** |
| `PEN_EXT_HOLDER` | 0.0588421 | **0.0460262** |
| `PEN_LAT_HOLDER` | 0.1253421 | **0.0860369** |
| tip, `panda_hand` | (0.1253, 0, 0.1622) | **(0.0860369, 0, 0.1494262)** |
| graphite past the cap | 53.214 mm | **20.000 mm** |
| reach from the grip | 83.215 mm | **50.001 mm** |
| the TCP → tip ray | 64.85° | **61.86°** |
| tip below the blades' plate edge | 50.000 mm | **37.184 mm** |
| grip centre | (0.0665, 0, 0.1034) | **unchanged** |

**And the §7 red flag closes.** `scripts/extract_penholder22_meshes.py` has
printed the disagreement between the drawn bore and the housing's own clocking
on every run since 2026-08-25. It now prints **−0.00°**. The 22° that had no
mechanism to live in — not a cradle, not the fingertip sockets, not the blade —
turns out never to have needed one: the model was drawing the bore along a
planner ray that was itself only ever an assumption.

Casing clearance stays irrelevant by instruction and is reported only: at this
lean the housing clears the hand shell by **+13.84 mm**, the cap by **+59.90**,
and the 72.5 mm pencil tail runs **−10.61 mm** into it.

#### The holder is clamped at the FAR END of the blades — 2026-09-04

Pete, on the first render: *"you need to move the pen to the front of the
aluminum extension … you essentially need to slide it forward in x by a few cm
so the gripping point is at the tip of the finger extension."*

He is right, and the model had it on the finger centreline because that is
where the plate's own Ø6.000 hole is — the one feature on the blade that looks
like it locates something. It does not (§7d(a)): it is an insert hole for a
fingertip nobody fitted. **The blade's contact plate is 90 mm long and runs
link x −9.000 … +79.500**, so a post on the centreline leaves 66.5 mm of
aluminium hanging past the tool, which is what every render before this showed
and what the photograph does not.

Clamped at the far end instead, a 26 mm post with its **outer face flush with
the plate's own far edge** has its axis at

```
grip_hand_x = 79.500 − 26.000/2 = 66.500 mm     rig_final.PENHOLDER22
```

**The sign is the side the pen leans toward, and that is load-bearing.** Both
blades reach the same way in the hand (one part, mirrored — §7d), the reach is
+x, and the bore leans +x, so the tool is cantilevered the way the photograph
shows. Clamped at the far end with the pen leaning back over the wrist it
would be a different and much worse tool.

**Nothing inside the holder moves.** Sliding the grip along hand x translates
the whole assembly and nothing else: the bore still leans 45.00° off the
approach axis, the grip is still 30.001 mm from the cap's outer face and
55.099 mm from the tail face, and the graphite past the cap is the same
53.214 mm. The **grip height is unchanged** (the plate band's own centre, §7d
(d)) and so is the post-between-the-plates seating.

**What it does change is the tip, and one word in the vocabulary.**

| | 2026-09-03 | 2026-09-04 | **2026-09-07 (shipped)** |
|---|---:|---:|---:|
| grip centre, `panda_hand` | (0, 0, 0.1034) | **(0.0665, 0, 0.1034)** | (0.0665, 0, 0.1034) |
| `PEN_EXT_HOLDER` | 0.0588421 | 0.0588421 | **0.0460262** |
| `PEN_LAT_HOLDER` | 0.0588421 | 0.1253421 | **0.0860369** |
| tip, `panda_hand` | (0.0588, 0, 0.1622) | (0.1253, 0, 0.1622) | **(0.0860369, 0, 0.1494262)** |
| `PEN_LEAN_HOLDER` — the **bore's** lean, at the grip | 45.00° | 45.00° | **23.00°** |
| the **TCP → tip ray**, which is no longer the bore | 45.00° | 64.85° | **61.86°** |

`PEN_LEAN_HOLDER` is the angle a protractor on the real barrel would read. It
stopped being the angle of the TCP → tip ray the moment the grip left the TCP,
and `penholder22_T_hand` now aims the bore at the **grip → tip** ray. Two
generators used to rebuild the graphite from `arctan2(pen_lat, pen_ext)` and
would have drawn it 20° off the barrel; `rig_final.penholder22_lead` owns that
arithmetic now, in one place.

**AND THIS WITHDRAWS AN ARGUMENT MADE BELOW.** With the grip on the centreline
the gripper's own casing ruled out any lean under 35.17°, and that was the
reason given here for 45°. At the far-end placement it does not:

| | grip on the centreline | **grip at the plate end** |
|---|---:|---:|
| housing vs the hand shell, 23° | −11.90 mm | **+13.84 mm** |
| housing vs the hand shell, 45° | +6.15 mm | **+7.78 mm** |
| cap, 45° | +40.44 mm | **+81.87 mm** |
| the 72.5 mm pencil tail, 45° | +18.36 mm | **−24.32 mm** |

At this placement **23° clears the casing better than 45° does**, so the
hardware no longer forces the lean. 45° stood, for three days, because it was
the user's standing rule and the incumbent — and on 2026-09-07 the housing's
own block geometry replaced it with 23° anyway (above). The casing was never
the reason for either.

And on the user's instruction (2026-09-04) the casing numbers do not constrain
the placement at all any more: *"the pen's simulated length is a random number
at this point so it does not matter if it intersects with the gripper
casing."* The modelled pencil is a ⌀7 stick of arbitrary length until somebody
measures the real protrusion, so the **−24.32 mm** above — 24 mm of tail
drawn through the casting — is a drawing artefact, reported and not designed
around. The barrel itself is clear.

#### What the 2026-09-03 reading said, and still says

**What it settles.**

1. **The blades are mounted the way this model has them** — foot outboard at
   the carriage, web down and inward, plates inboard and converging toward the
   paper. §7d has the numbers, and also the proof that the mirror is
   impossible: mounted the other way round two plates sit 68.700 mm apart at
   q = 0 and could never close on a 50 mm post.
2. **No fingertips are fitted, and the bare plates clamp the post's end
   faces.** That deletes four of §7d's six `width` rows and — because a seated
   fingertip in the housing's own socket was the only thing in the system that
   could fix the clocking — it makes the lean a decision made by hand.
3. **The pen leaves through the cap, toward the paper, and the pencil's tail
   stands out of the back of the housing at the wrist.** Both are what §7c
   already fixed. The photo puts the tail's push-cap end within a centimetre or
   two of the hand's underside, and the model agrees: the tail's closest
   approach to the manufacturer's hand shell is **18.36 mm**, and the housing's
   own tail face sits **1.52 mm** below the hand's underside plane.

**What it forces — SUPERSEDED 2026-09-04, see above.** This argument was made
with the grip on the finger centreline and does not survive the move to the
plate's far end: at the real placement 23° clears the casing better than 45°
does. It is kept because the measurement is sound and the sweep is worth
having; it is no longer a reason for anything. The photo cannot read the lean —
the pen leans in the plane perpendicular to the jaw axis, which is toward or
away from the camera. The post sits 55.099 mm from the
housing's tail face, so **55.1 mm of barrel stands behind the grip**, and there
are **37.4 mm** between the grip and the hand's underside. Sweeping the raw
housing STL (placed by `rig_final.penholder22_T_hand`) against the convex hull
of the manufacturer's own `hand.obj` collision shell:

| lean | housing vs the hand shell | cap | the ⌀7 pencil tail |
|---:|---:|---:|---:|
| 10° | **−18.79 mm** | +53.8 | **−17.40 mm** |
| 20° | **−14.82 mm** | +50.4 | **−4.75 mm** |
| **23° (the housing's own flats)** | **−11.90 mm** | +49.3 | **−1.86 mm** |
| 30° | **−5.05 mm** | +46.6 | +4.80 |
| 35° | **−0.16 mm** | +44.6 | +9.46 |
| **35.17° (the floor)** | **0.00 mm** | +44.6 | +9.61 |
| 40° | +2.99 | +42.6 | +13.99 |
| 45° (what it drew, 2026-09-03/04) | **+6.16 mm** | +40.4 | **+18.36 mm** |
| 50° | +9.37 | +38.3 | +22.55 |

**On the centreline the lean could not be less than 35.17°.** At 23° twelve
millimetres of printed barrel would have been inside the gripper's casting.
That was, for one day, the first argument for the 45° ray that was about the
*hardware* rather than about the planner — and moving the grip 66.5 mm out
along the plates dissolved it. The sweep above is the centreline's; the
far-end numbers are in the 2026-09-04 table.

**What it moves.** Pete's reading of the photo: *"just have the pen protruding
out the bottom of those finger extensions by ca 5 cm."* This is the rule that
has held through both corrections — it fixes the tip's HEIGHT, and neither the
grip move nor anything else has touched it. The Fat blades' contact plates end
at `panda_hand` **z = 0.1122421** (§7d), so the tip is at **z = 0.1622421**,
and from the hand TCP at 0.1034:

```
PEN_LEAN_HOLDER = 45°            the BORE's lean, at the grip
PEN_EXT_HOLDER  = 0.1622421 - 0.1034                        = 0.0588421 m
PEN_LAT_HOLDER  = grip_hand_x + PEN_EXT_HOLDER * tan 45 deg = 0.1253421 m
                = 0.066500 + 0.0588421          (grip_hand_x since 2026-09-04)
```

| | old | **2026-09-03** | **2026-09-04 (shipped)** |
|---|---:|---:|---:|
| tip, `panda_hand` | (0.110, 0, 0.2134) | (0.0588421, 0, 0.1622421) | **(0.1253421, 0, 0.1622421)** |
| reach from the grip centre | 155.563 mm | 83.215 mm | **83.215 mm** |
| graphite past the cap's outer face | 125.562 mm | 53.214 mm | **53.214 mm** |
| …as the photo's foreshortened view reads it | 110.0 mm | 37.6 mm | **37.6 mm** |
| tip below the blades' plate edge | 110.0 mm | 50.000 mm | **50.000 mm** |
| tip below the hand's underside | 147.4 mm | 96.3 mm | **96.3 mm** |
| implied ⌀7 stick, with 72.514 mm of tail | 283.2 mm | 210.8 mm | **210.8 mm** |

Every row but the first is a length **along the holder**, which is why the
2026-09-04 grip move leaves them all alone: it translates the tool, it does
not stretch it.

The photo shows roughly 20–40 mm of graphite below the cap, foreshortened;
37.6 mm is what this transform puts there. **The holder body does not move by
a picometre**: its placement is fixed by the ray's *direction* and by the grip
centre, both unchanged, so the two housing meshes are byte-identical across
the change and every escape §7c measured still stands.

**And it is still USER-SPECIFIED, not measured.** `frames.PEN_EXT_HOLDER` and
`PEN_LAT_HOLDER` are a reading of a photograph, refined by a constraint. Only
the **inline** pen's `PEN_EXT = 0.110` carries a real touchdown (gate B,
MZ 0.924). Several places in this repo said the holder's pair was
"gate-validated against a real touchdown"; they were wrong and they are
corrected.

#### One ruler reading would set it

The protrusion is adjustable — that is the whole point of a clutch pencil — so
what follows is not a prediction but a conversion table. **Forward**: given the
graphite standing past the cap's outer face, where the tip is.

| lean | graphite past the cap | reach from grip | tip x | tip z | below the plates | vs the shipped tip |
|---:|---:|---:|---:|---:|---:|---:|
| 23° † | 20 mm | 50.001 | 19.537 | 149.426 | 37.2 | 41.3 |
| 23° † | 30 mm | 60.001 | 23.444 | 158.631 | 46.4 | 35.6 |
| 23° † | 40 mm | 70.001 | 27.352 | 167.836 | 55.6 | 32.0 |
| 23° † | 90 mm | 120.001 | 46.888 | 213.862 | 101.6 | 53.0 |
| 23° † | 126 mm | 156.001 | 60.954 | 247.000 | 134.8 | 84.8 |
| **45°** | 20 mm | 50.001 | 35.356 | 138.756 | 26.5 | 33.2 |
| **45°** | 30 mm | 60.001 | 42.427 | 145.827 | 33.6 | 23.2 |
| **45°** | 40 mm | 70.001 | 49.498 | 152.898 | 40.7 | 13.2 |
| **45°** | **53.214 mm** | **83.215** | **58.842** | **162.242** | **50.0** | **0.0 — shipped** |
| **45°** | 90 mm | 120.001 | 84.854 | 188.254 | 76.0 | 36.8 |
| **45°** | 126 mm | 156.001 | 110.309 | 213.709 | 101.5 | 72.8 |

**Inverse**: given a measured perpendicular distance from the tip to the
gripper's approach axis — the ruler measurement §7a has been asking for since
2026-09-02 — everything else.

**Both tables are in the BORE's own lateral offset**, i.e. measured from the
GRIP, not from the wrist axis. Since 2026-09-04 the grip is 66.500 mm out
along hand x, so a ruler laid against the real gripper reads
`66.5 + (this column)`: **subtract 66.5 mm from a ruler reading before
entering the inverse table, and add it to the forward table's tip x.** The
shipped row is **19.5 mm** here (50.001 mm of reach at 23°) and **86.0 mm**
on the ruler. The 45° rows below are kept as arithmetic; the shipped lean is
23°.

| lean | measured tip↔axis | reach from grip | axial from TCP | tip z | below the plates | graphite past the cap | vs the shipped tip |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 23° † | 30 mm | 76.78 | 70.68 | 174.08 | 61.8 | 46.78 | 31.2 |
| 23° † | 47 mm | 120.29 | 110.73 | 214.13 | 101.9 | 90.29 | 53.2 |
| 23° † | 58.8 mm | 150.59 | 138.62 | 242.02 | 129.8 | 120.59 | 79.8 |
| 23° † | 70 mm | 179.15 | 164.91 | 268.31 | 156.1 | 149.15 | 106.7 |
| 23° † | 110 mm | 281.52 | 259.14 | 362.54 | 250.3 | 251.52 | 206.7 |
| **45°** | 30 mm | 42.43 | 30.00 | 133.40 | 21.2 | 12.43 | 40.8 |
| **45°** | 47 mm | 66.47 | 47.00 | 150.40 | 38.2 | 36.47 | 16.7 |
| **45°** | **58.8 mm** | **83.22** | **58.84** | **162.24** | **50.0** | **53.21** | **0.0 — shipped** |
| **45°** | 70 mm | 98.99 | 70.00 | 173.40 | 61.2 | 68.99 | 15.8 |
| **45°** | 110 mm | 155.56 | 110.00 | 213.40 | 101.2 | 125.56 | 72.3 |

All lengths in millimetres, `panda_hand` frame, grip centre on the TCP at
z = 103.4. **† the 23° rows are arithmetic, not candidates**: at that lean the
housing is 11.9 mm inside the hand (above). They are kept so the table answers
the question as it was asked, and so that a future build that moves the post
along the barrel can be checked against them.

**What would still change the answer.** A `width` reading near 0.0432 or
0.0500 (§7d(c)) would say the blade's foot is bolted 7–11 mm further inboard
than modelled. That moves the plates' bottom edge nowhere — the blade's own
geometry is unchanged — but it would move the grip centre along the jaw axis,
not along z, so **the 50 mm and this whole table survive it**. What would not
survive it is §7d's grip-width arithmetic.


## 8. Open physical questions — for Pete

All seven are in `system_model.OPEN_QUESTIONS` and in the manifest, each with
what rides on it and what would answer it.

1. **Plate offset direction.** Does the plate's 25.15 mm offset run in canvas
   +x or −x? The drawing gives it for an arm whose front faces +X; this rig
   clocks every arm the other way, so it flips — and *which edge of a Franka
   base plate is its front* is itself an inference. **7.79 mm rides on it:**
   offset the wrong way the plate does not fit between the posts at all. Fix
   either by measuring the real plate, or by opening the post gap to 304.8 mm
   (posts at axis ± 190.5), which fits either way with 14.4 mm each side.
   *Blocks: fabrication of the drop clusters.*

2. **Base cable pass-through.** *Found by this model, by nothing else.* The
   manufacturer's link0 visual carries the connector and cable stub **230.7 mm
   past the base flange** (677 of 40 142 vertices). Every arm here is
   inverted, so that 230.7 mm points straight **up**: through the 12.7 mm
   plate, through the 95.7 mm clamp stack, and 122 mm on into the drop
   cluster. The collision shell stops dead at the flange, so no clearance
   check in this repo has ever seen it. Vertically there is room (the posts
   run 905.0 → 1623.6, the cable tops out at 1170.7) but **the plate and the
   clamp stack are solid across it today**. *Blocks: fabrication of the plate
   and the clamp stack.*

3. **Ceiling survey.** How high is the real room above the paper? The drawing
   defines no room ceiling; every ceiling number in this repo descends from a
   floor-referenced dimension. 716.4 mm of modelled boom rides on it, and so
   does whether the cage fits the room at all. *Blocks: cutting the grid.*

4. **Pen holder cradle.** §7, and now §7d: the Fat finger's mesh has arrived
   and it carries no cradle either — the plate is flat, and the only thing in
   the system that would fix the lean is a fingertip seated in the housing's
   own socket. **Two** measurements close it now, and one of them is a number
   the robot already knows: its own grasp `width`. *Blocks: pen-tip
   calibration.*

5. **Cage legs.** Correcting the datum turns the cage from something hanging
   off a room ceiling into something standing on the floor — and the re-issued
   cut list has no legs in it, because it assumed the former. Four corner legs
   are modelled, mirroring the original's exactly, but a 4.01 m frame on four
   legs has **no precedent**: the original spans 2.08 m. The unsupported
   perimeter span between runways is 1210.2 mm. **Partly answered
   2026-09-14:** the seam bars (§3b) are mid-span supports at exactly the
   mid-length this entry said would need them, so the longest unsupported run
   is now corner leg to seam bar and not end to end. *Blocks: fabrication.*

6. **Gusset attachment.** The rotation onto the y faces resolves a hard
   interference and buys 89.2 mm, but the drawing has no gusset part number
   and the attachment is an envelope, not a detail. *Blocks: fabrication.*

7. **Cable dress.** Nothing about it has been measured. The 30 mm loops here
   are the middle of the collision audit's 20–40 mm estimate, **visual only
   and off in collision** — turning them on is a re-certification, not a
   switch. *Blocks: nothing yet.*

## 9. Provenance classes

Every body names exactly one:

| class | meaning | static bodies |
|---|---|---:|
| **DRAWING** | lifted from the original drawing, through `rig_final.FRAME_BOXES_W_CM` / `ARM_MOUNTS_W`. The entity is named. | 47 (61.0 %) |
| **CODE** | a constant this repo plans against | 1 (1.3 %) |
| **AUDIT** | measured by a script here, off CAD or off the manufacturer's meshes | 0 (0.0 %) |
| **ASSUMED** | chosen here; every one is also an open question | 29 (37.7 %) |

That mix covers the **static** bodies only — the cage, table and canvas. The
29 ASSUMED are the 24 rotated gussets, the 4 cage legs and the table
footprint. The arms, their collision geometry and the tool are not boxes and
carry their own class: **arm visual, collision shells and capsules are all
AUDIT**; the tool **geometry** is AUDIT and its **placement** is ASSUMED.

## 10. Validation

| check | result |
|---|---|
| pydrake parse, all three URDFs | clean; 186 bodies / 42 DOF, environment 78 bodies / 0 DOF |
| pen tip vs `frames.fk`, 25 configs × 6 arms | **worst 5.26 pm** (bar 10 pm) |
| pen tip orientation | worst 1.33e-11 on a matrix entry (bar 1e-10) |
| every cage body vs `system_model` | exact |
| collision spheres in either file | **0** |
| arm collision bodies | 66 manufacturer shells / 186 audited capsules |
| dangling glTF references | **0** |
| URDF regeneration | byte-stable, twice — see the note below |
| **clean-room rebuild** | `gen_system_model.py all` into an empty directory reproduces textures, glTFs, shells, the re-decimated holder, the Fat finger, the URDFs and the manifest |
| fat-finger variant (§7d) | parses, 186 bodies / 42 DOF, joints at the derived value, 4 measured boxes per finger, envelope escape **0.000000 mm** |
| holder envelope after the §7c flip | 4 cylinders; internals escape **0.0 m** at every shim, housing meshes escape the housing's own three by **+0.109 µm** |
| **the pen tip after the §7c flip** | **unmoved: the same worst 5.259e-12 m**, digit for digit, as before it |
| **the pen tip after the §7e re-specification** | it MOVED, on purpose: 72.348 mm, to (0.0588421, 0, 0.1622421). Worst URDF-vs-`frames` error **4.779e-12 m** over 25 configs × 6 arms |
| the holder BODY after §7e | **byte-identical** — the placement is fixed by the ray's direction and the grip centre, and neither moved |

**A note on "equal to what is committed".** From 2026-09-02 the committed
`installation.urdf` and `installation_capsules.urdf` did **not** reproduce byte
for byte on this station, and it was not a code change: 48 and 150 lines
respectively differed, every one of them a rotation written by `_rpy_checked`,
and the worst difference was **4.44e-16** — two ULP on π, i.e. sub-attometre at
the tip. Both this repo's venv (numpy 2.5.2) and the station's (numpy 2.2.6)
produced the *same* new bytes, so the committed files had been written by a
third environment whose matmul rounded one bit the other way. They were left
as committed rather than churned for two ULP.

**RESOLVED 2026-09-03, as a side effect and not by a hand edit.** §7c re-wrote
both files anyway. Before regenerating, the pre-existing churn was re-measured
in this environment against the old code — **48 and 150 lines exactly**, so it
reproduces and nothing was slipped into the larger diff. The flip's own share
is 126 changed lines plus 6 added per file (one collision cylinder and one
visual per arm). `test_regeneration_is_byte_stable` is green again.
| **arm inside the steel at a park pose** | **none**, in either collision variant |
| **static bodies interpenetrating** | **none** — all 77 checked pairwise |

### What the corrected cage leaves around the certified programme

The question this model was built to answer, measured at the six certified
park poses (`Q_PARK_PROPOSED`) — the poses the programme actually holds for
whole phases:

| | manufacturer shells | audited capsules |
|---|---:|---:|
| closest arm to another **arm** | 403.2 mm | **264.0 mm** |
| closest arm to **steel** | 349.3 mm | **316.1 mm** |
| closest arm to the **paper** | 196.5 mm | 196.5 mm |
| closest arm to the **table** | 198.5 mm | 198.5 mm |

**Nothing interpenetrates.** The overall minimum, 196.5 mm, is the pen's own
graphite over the paper — that is the park hover, and it is meant to be small.

**The §7c flip changed none of these four numbers.** It moved the holder body
and lengthened `pen_lead` at the far end from the paper, so what the closest
approaches are made of did not move: all four minima are the same to 0.1 mm
before and after. The only difference in the printed table is which arm's
`pen_lead` is *named* on the table row — arm 13 before, arm 97 after, at the
identical 198.5 mm, because six arms park at the same hover and the tie is
broken by iteration order.

**The tightest steel approach in the whole rig is an arm to its neighbour's
drop post** (arm 71's link2 to arm 31's post, 316.1 mm under the audited
capsules). That is worth reading twice: the thing an arm comes closest to is
precisely the structure the certified envelope models as a Ø200 column and
which the real steel escapes by up to 185.55 mm. It clears today, by a wide
margin — but it is the reason the drop-cluster re-certification is the item
that matters, rather than a formality.

The capsules read tighter than the shells because they are a conservative
envelope of the same meshes, which is what a conservative envelope is for.

This is the **park poses only**. The drawing poses are gated by the atlas
against `mounts.py`'s schematic column, and re-gating them against this cage
is exactly the re-certification item in §6 — the corrected cage is *lower*
than the modelled column but *wider*, so neither result carries over.

The generator writes every number as an **exact float64 round trip**
(`repr`), so `float(text) == value` and there is no truncation error to pick a
tolerance for. The proposed rig's 1e-9 writer costs 1.06 nm at the far row,
where `y = 3.0255333333333336` truncates to `3.025533333`; that is why the bar
here is picometres rather than nanometres.

## 11. Renders

`scripts/render_system_model.py` → `out/` (gitignored):
`system_model_three_quarter.png`, `_elevation.png`, `_plan.png`,
`_drop_cluster.png`, `_holder.png`, `_holder_side.png`, `_photo.png`,
`_holder_jaw.png` (2400 px, PBR, shadows) and `system_model.html` (static
meshcat, 32.6 MB). Arms are posed at `Q_PARK_PROPOSED`.

**`_holder_jaw` is the view the tool gets judged from** (added 2026-09-07):
straight down the **jaw** axis, aimed at the holder's own middle, `up = −z_hand`.
The bore's lean and the graphite's protrusion both lie *in* the image plane
here, so neither is foreshortened — which is what makes it the shot to hold
against a photograph. `_holder_side` looks down hand **x** and shows the V of
the blades; this one looks down hand **y** and shows the **angle**. The 18 mm
plate band hides the middle of the block from this camera; the barrel above it
and the cap below are on the same line and carry the angle.

**`_holder_side` is the shot that says which way round the housing is** (§7c),
and it was added because the error was visible in `_holder.png` for a day
before anybody read it. The camera looks straight down the hand's own **x**
axis, from the side the pen leans toward, with `up = -z_hand`: finger travel
lies across the image, the approach axis runs down it, and the bore — which
lives in the hand's x–z plane — is foreshortened by cos 45° and nothing else.
So the barrel and the pencil tail read as the part **above** the fingers and
the cap, the graphite and the tip as the part **below** them. Mounted
end-for-end the same shot puts the fat end below the fingers, which is why it
is worth its own camera rather than a note.

**`_photo` is the photograph's own viewpoint** (§7e), added 2026-09-03: an
oblique from slightly *below* the hand — further along `+z_hand`, which is
toward the paper — looking roughly down the hand's x axis, so the jaw axis
lies across the frame. It is the shot in which the Fat blades read as the
**V** the photograph shows (feet apart at the carriages, plates together at
the paper) and in which the pencil's tail is seen standing out of the back of
the housing at the wrist. Rendering the model from the place the photograph
was taken is how "does the model agree with the photograph" stops being a
matter of opinion.

`--urdf`, `--tag` and `--views` render a second scene without overwriting the
first — the fat-finger close-ups of §7d and §7e are

```
<station venv>/bin/python scripts/render_system_model.py \
    --urdf assets/system_model/installation_fatfingers.urdf \
    --tag fatfingers_ --views holder,holder_side,photo,holder_jaw --no-html
```

→ `out/system_model_fatfingers_holder.png`,
`out/system_model_fatfingers_holder_side.png`,
`out/system_model_fatfingers_photo.png`.

## 12. Relationship to `assets/proposed_rig/`

`assets/proposed_rig/` is **still current** for what it is: the asset every
certified number was checked against, and the one `scripts/collision_audit.py`
reads. This model is an addition to it, not a replacement. See
`assets/proposed_rig/README.md` for which to reach for.

It is no longer *untouched*, and the two exceptions are worth naming. On
2026-09-03 the §7c housing correction re-baked its two hand-frame holder
meshes and moved the holder's collision cylinders in its `installation.urdf`,
because `rig_final.penholder22_T_hand` is shared and leaving a body in the
certified tree that is known to be on backwards is worse than the churn. Then
§7e moved the pen **tip**, which changes `pen_bracket`'s and `pen_body`'s
lengths, the `pen_tip` weld and `pen_lead` — and leaves the two holder meshes
**byte-identical**, because the placement depends on the ray's direction and
the grip centre and neither moved. **Nothing else in that tree moved**: no
capsule radius, no arm collision geometry, no base pose, no layout number.
§7c has the measured cost of the first; §7e has the second, and the
consequence that every programme and atlas in `out/` was planned at the old
tip and is now stale.
