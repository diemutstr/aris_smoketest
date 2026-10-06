# rig — what the installation looks like

**Job.** The one place that knows the table, the paper, the six slots the arms hang in, the
steel around them, the pen that is in and the clearances. It is also the one place that turns
table-frame things into one arm's base frame. Every planner below the system planner sees only
base-frame geometry, and all of it comes from here.

**In.** `config/rig.json`, plain data with a source note on every block, and optionally
`config/calibration/<slot>.json` per slot. Nothing else in the package reads `config/` (the
checker has its own reader, on purpose). The rig does not read `site/`: which robot hangs in
which slot is the robot side's business.

**Out.** `Rig.load(config_dir)` gives a `Rig` with these calls. `a` is always a slot, a string
like `"2R"`.

| call | gives |
|---|---|
| `arm_ids`, `slot_names` | the mounted slots; every slot of the frame, mounted or not (both in `rig.json` order) |
| `T_table_base(a)`, `T_base_table(a)`, `park_q(a)` | the arm's pose (calibrated when its base part applies) and its park |
| `arm(a)` | the arm model with this slot's tool: the measured pen tip when the pen part applies, else the current pen's nominal length |
| `calibration_status(a)`, `calibrated(a)` | `{"base": ..., "pen": ...}`, each `"none"`, `"applied: ..."` or `"<part> part not applied: <why>"`; whether both parts applied |
| `pen()` | the pen that is in: its `pens.table` entry plus `"name"`. The server copies it into every job header |
| `rules()` | the drawing rules (`DrawRules`), all from `rig.json`, with `gates()` inside; press and speed on the paper are the pen's |
| `to_base(a, line)`, `to_table(a, points)` | lines and points moved between the frames |
| `paper(a)` | the paper as a plane in `a`'s base frame, free side up, with three margins: links, lifted pen, rest of the tool |
| `wall_between(a, b)`, `wall_in_base(a, wall)` | a wall in the table frame, and as a plane for one arm |
| `phase(n)` | phase 1 or 2: the leaders move, the others stand parked, walls 1L-2R and 2R-3L (phase 1) or 1R-2L and 2L-3R (phase 2) |
| `obstacles_for(a, phase)` | `obstacles` for a moving arm, with the parked arms it can reach (always its row partner) and the walls it stands next to |
| `obstacles(a, parked=(), walls=(), for_planning=True)` | everything `a` must stay clear of |
| `gates()` | the planners' `Gates`: joint-limit margin 0.15 rad, smallest singular value 0.04, self margin 0.020 + 0.003 |
| `execution()` | what the executor checks before a motion: `start_tolerance`, 0.03 rad per joint (0.005 until 2026-10-06: a holding arm drifts more, measured on site) |
| `drawing_area_m`, `drawing_area_centre_m` | the admissible drawing area, x by y (1.56 x 3.56 on the full rig), and its centre in the table frame ([0, 0] when `rig.json` has none). Written by the system planner from its maps; the area is None if absent |
| `leaders(phase)`, `row_partner(a)` | (1L, 2R, 3L) in phase 1, (1R, 2L, 3R) in phase 2; rows 1L-1R, 2L-2R, 3L-3R |

`aris.rig.is_slot(name)` says whether a name is a slot (`1L` to `3R`). A call with anything that
is not a mounted slot, for instance an old robot id like `31`, raises with a hint.

## Slots

An arm is named by its place on the frame, not by the robot that hangs there (decided
2026-10-02). Row 1 is at the −y end, row 3 at the +y end; L is at −x, R at +x. The load checks
that the names in `rig.json` are well formed, used once, and agree with the axes: L at negative
x, R at positive x, rows numbered in the order of y.

Which robot is in which slot is in `site/aris_2026-10.json`, read only by the robot side.
Confirmed on site 2026-10-06: 1L = robot 2, 1R = 31, 2L = 97 and 2R = 71 (the live pair), 3L =
13, a floor-mounted upright arm that this rig never drives (`controlled: "never"`), and 3R empty
(robot 17 is off the table, `absent: true`).

A slot entry in `rig.json` may say `"hanger": false`: no hanger steel there today. 3L and 3R say
so. It counts only for a slot that is not mounted (a mounted arm always hangs from its hanger;
`config/rig.json` with all six mounted stays the design rig), so it changes the two-arm rig and
not the six-arm one.

## Frames

The table frame has its origin at the table centre, on the paper. x runs across the table,
y along it, z up. The old code put its origin at a canvas corner: table = old − (0.9017, 1.81532, 0).
Each arm's base frame is the arm model's own origin, on the underside of its mounting plate. The
arms hang upside down, so base z points down and base x points to table −x.

```
            y
            ^        table 2.188 x 4.1656, paper 1.8034 x 3.6306, both centred
  +1.2102   |   3L o-----o 3R
            |      |  \  |        o = a slot's first-joint axis, 0.970 above the paper
      0     |   2L o-----o 2R     \ = the phase 1 walls, 1L-2R and 2R-3L
            |      |  /  |
  -1.2102   |   1L o-----o 1R
            +------------------> x
               -0.305  +0.305
```

| slot | old id | axis x | axis y | row partner | leader in phase |
|---|---|---|---|---|---|
| 1L | 13 | −0.305 | −1.2102 | 1R | 1 |
| 1R | 17 | +0.305 | −1.2102 | 1L | 2 |
| 2L | 31 | −0.305 | 0 | 2R | 2 |
| 2R | 71 | +0.305 | 0 | 2L | 1 |
| 3L | 2 | −0.305 | +1.2102 | 3R | 1 |
| 3R | 97 | +0.305 | +1.2102 | 3L | 2 |

The rows are exactly a sixth of the old 3.63064 m canvas apart (1.2102133 m), because that is
where the old code put them. With those numbers the base poses match the old code to 2e-16.

## Walls and fences

A wall is a vertical plane halfway between two arms' axes and square to the line joining them.
The wall between 1L and 2R passes through (0, −0.6051), 0.6776 m from both axes, turned 26.75°
from x. The phase 2 wall between 1R and 2L passes through the same point, so the two phases'
walls cross there. As a plane for one arm, the free side is the side that arm's axis is on.

The wall clearance is **0.040 m** until the arms' x and y are calibrated (they are 1 to 2 cm off
today; 2026-10-02). Then it goes back to 0.025, half the arm-to-arm clearance. Walls are
planning conventions, so they use the **nominal** axes from `rig.json`: a calibration does not
move them.

A **fence** is a wall that holds in every phase: every controlled arm's whole body stays on its
free side, at the wall clearance. `tools/mounted_rig.py` writes one between every row with no
controlled arm and the nearest row with one, halfway (`fence_row_...`). In a row with one
controlled arm, the switched-off arm beside it is fenced off by a plane halfway between the two
columns, at x = 0, normal toward the controlled arm (`fence_col_minus_x` or `fence_col_plus_x`);
the same plane from several rows is written once.

## Steel

All steel is axis-aligned boxes in the table frame. Every box has a source note.

| part | boxes | numbers from |
|---|---|---|
| hanging struts | 2 per slot, 0.0762 x 0.1524 section (long side along y) | the technical drawing (Pete, 2026-09-30): `docs/drawings/plan_centre_datum.pdf`, sheet 3, panel D. Turned 180° (below): for every arm, axis to outside face 0.22205 on −x and 0.17175 on +x (393.8 mm outside to outside, 241.4 mm clear between). Top at the runway underside (old model). Bottom 65 mm below the plate underside (0.905): the old model's 35 mm plus 30 mm, on Pete's instruction (2026-09-30), to be conservative because nobody will measure it. |
| mounting plate | 1 per slot, 0.2258 x 0.190 x 0.0127 | old model (drawing). Centre 25.15 mm toward table −x of the axis, for every arm (sheet 3, panel D, turned). |
| clamp stack | 1 per slot, on the plate | old model (drawing) |
| runways | 3, one double beam per row at 1.624 to 1.700 | old model (drawing) |
| seam bars | 2, beside the table at y = 0, from the table top to the runway | old model, "representative, not measured" (Pete, 2026-09-14) |
| perimeter rails and corner legs | 4 + 4 | old model (rails from the drawing, legs assumed) |

**The frame is the drawing turned 180°** (site, 2026-10-06: confirmed with drawn crosses, and
with the drawing's offsets 1L planned through its own inner strut). The slot positions are
symmetric and do not change; the hanger numbers that are not do: the −x and +x outside faces
swap (222.05 mm on −x, 171.75 mm on +x) and the plate centre moves to 25.15 mm toward table −x
of the axis, so the J1 axis sits 25.15 mm toward table +x of its plate.

**Hangers follow the calibration.** Every slot has its hanger, whether or not an arm hangs
from it today (the row 1 hangers are steel only on the two-arm rig; 3L and 3R have none, see "Slots"). A mounted arm is bolted to its
plate, so its hanger is placed from the arm's *calibrated* axis: the x and y of its calibrated
`T_table_base` when the base part of its calibration applies, the nominal axis otherwise. Seen
from the arm, its own hanger therefore never moves; seen from the table and from the
neighbours, it moves with the arm. Heights stay as `rig.json` gives them. (Walls do not move:
they are planning conventions, see "Walls and fences".)

An arm's own struts, plate and clamp are obstacles for it like any other steel. Its base (the
arm model's link0 capsules) sits among them. Link 0 is marked fixed, and the collision check leaves it out.
Link 1, which only turns about the base axis, passes just under the strut ends. It is checked
per pose like every other link, except against its own arm's struts, plate and clamp. Those
boxes carry `exempt = ("link1",)` for that arm only (`hanger.exempt_links` in `rig.json`), and
the rig checks that pair once (see "Link 1"). A neighbour's hanger exempts nothing. An arm skips any box further from its shoulder than
**1.20 m + the steel clearance**: 1.20 m is how far any part of the arm, hand, holder and pen can
get from the shoulder. That is a bound from the link lengths, checked by sampling: 1.079 m with
the old body model and 1.028 m with the new one. Each arm keeps 16 to 23 boxes.

**Compared with the old `spec.static_obstacles()`.** The seam bars are identical. The old set had
each neighbour's plate as a 0.226 x 0.190 x 0.050 block centred on the axis. It is now the
drawing's 12.7 mm plate, 25.15 mm off the axis (toward −x since the frame was found turned). The old set had each neighbour's "boom" as one 0.2 m square
column up to 2.34 m. Those are replaced by the two struts of the drawing, and the runway is at the
drawing's 1.62 m. The old set also had 4 "body column" boxes per neighbour (20 per arm). Those
are arms, not steel. They now arrive as a parked arm's capsules or stay behind a wall. The old set
had no runways, rails or legs, and no clamps.

## Parked arms

`obstacles(a, parked=("1R",))` adds 1R's whole collision body, standing at its park
configuration, as capsules in `a`'s base frame. The park configurations are the old code's home
parks (`Q_PARK_PROPOSED`, h = 0.970, lateral holder).

Each park against its steel, the paper and itself. The first four columns come from the old code's
body model and are plain distances (demanded: paper 0.020, lifted pen 0.020, self 0.020, steel 0.050). The
last column comes from the new kernel and is measured beyond each obstacle's demanded margin:

| slot | paper (body) | pen tip height | self | steel, old body model (tape struts, 35 mm) | everything, new kernel, beyond the margin (frame turned, 2026-10-06) |
|---|---|---|---|---|---|
| 1L | 0.250 | 0.300 | 0.123 | 0.179 (own strut) | +0.136 (link 2, own +x strut) |
| 1R | 0.250 | 0.300 | 0.153 | 0.179 (own strut) | +0.127 (link 2, own −x strut) |
| 2L | 0.300 | 0.350 | 0.126 | 0.080 (seam bar W) | +0.079 (link 6, seam bar W) |
| 2R | 0.150 | 0.200 | 0.133 | 0.179 (own strut) | +0.137 (link 2, own +x strut) |
| 3L | 0.250 | 0.300 | 0.124 | 0.179 (own strut) | +0.115 (link 2, own +x strut) |
| 3R | 0.250 | 0.300 | 0.114 | 0.179 (own strut) | +0.144 (link 2, own −x strut) |

Every park keeps the demanded clearances, links 0 and 1 not counted (see "Link 1").

## Link 1

Link 1 turns about the base axis only, so a sweep over all of joint 1 is everything it can ever
do. Against its own hanger it is exempt from the per-pose check, and `tests/test_rig.py` checks
it once here, over all of joint 1. Against everything else it is checked per pose; the sweep
below shows it is far from all of that anyway.

Against the arm's own hanger steel (struts, plate, clamp), link 1 must keep 0.020, not 0.050
(`link1_to_own_mount_m`, orchestrator, 2026-09-30). Link 1 and that steel are bolted to the same
plate. The 0.050 steel gate pays for not knowing where separate parts stand relative to each
other, so it does not apply between them; the self clearance does. Against everything else,
link 1 must keep the usual clearances.

The worst clearance beyond the required margin, for every slot:

| own hanger (0.020) | other steel | walls, both phases (0.040) | other arms parked | paper |
|---|---|---|---|---|
| +0.031 | +0.274 or more | +0.189 | +0.216 or more | +0.488 |

With 2L's base calibrated 20 mm off in x and y (its hanger moving with it) the numbers are
+0.031, +0.304, +0.180, +0.206, +0.488: the own-hanger clearance does not change, because the
two are bolted together.

With the struts where the drawing puts them and 30 mm longer, link 1 passes the end of the near strut (+x since the turn)
with 50.8 to 62.4 mm of room. That is even 0.8 mm beyond the 0.050 steel clearance. But it is not
beyond 0.050 plus the 0.003 planning allowance: that fails on a third of the joint 1 range
(−2.73 to −0.92 rad). So the exemption stays: without it, every planner would refuse those joint 1
angles.

**Link 0**, the fixed base, is never checked against obstacles. As modelled, it is seven round
bands about the axis, radius 0.171 to 0.177 near the mounting face. It overlaps its own steel
(distances; negative is overlap):

| link 0 band (base z, m) | near (+x) strut | far (−x) strut | plate |
|---|---|---|---|
| link0.0, −0.238 to −0.075: the cable connector stub, above the mounting face | −0.081 | −0.031 | −0.115 |
| link0.1, −0.075 to 0 | −0.080 | −0.030 | −0.176 |
| link0.2, 0 to 0.035 | −0.076 | −0.025 | −0.171 |
| link0.3, 0.035 to 0.070 | −0.065 | −0.014 | −0.125 |
| link0.4, 0.070 to 0.100 | −0.016 | +0.034 | −0.042 |
| link0.5, 0.100 to 0.120 | +0.026 | +0.074 | +0.024 |
| link0.6, 0.120 to 0.144 | +0.032 | +0.078 | +0.042 |

The overlap is the round envelope, not the casting: the real base is bolted into the 241.4 mm gap,
and the bands are circles 0.34 to 0.35 m across. It says nothing about a collision, and the model
does not use it.

## Clearances

These are demanded by the checker and kept as the old gates. The planning allowance is a
separate number, added when `for_planning=True`, so that the checker is a second opinion and not
a coin toss. The old code's allowance was 0.013, but that paid for the old checker's sampling
error. The new distances are exact, and both the planner and the checker bound the motion between
samples to better than 1 mm, so 0.003 is enough (orchestrator, 2026-09-29).

| against | demanded | planning allowance |
|---|---|---|
| steel | 0.050 | 0.003 |
| another arm (parked) | 0.050 | 0.003 (a parked arm is as still as steel) |
| a wall or fence | 0.040 until x and y are calibrated (2026-10-02), then 0.025 (half of arm-to-arm) | 0.0015 (because both arms pay it) |
| paper, arm body | 0.020 | 0 |
| paper, lifted pen | 0.020: 20 mm until the calibration routine is proven on the rig, then 3 mm (Pete, 2026-09-30). The sequencer lifts 2 mm more than this | 0 |
| paper, rest of the tool (gripper, blades, holder) | 0.0: must not touch (Pete, 2026-09-30) | 0 |
| itself | 0.020 | 0.003 (`rig.self_margin()`, `rig.gates()`) |
| link 1 to its own hanger steel | 0.020 (same plate; see "Link 1") | checked once, not planned |

## Gates and drawing rules

These are facts about the installation, like the clearances. They sit in `rig.json` under `gates`
and `drawing`, and `rig.gates()` and `rig.rules()` are the one source for them.

| | value | note |
|---|---|---|
| joint-limit margin | 0.15 rad | |
| smallest singular value of the tip Jacobian | 0.04 | Pete, 2026-09-30, lowered from 0.08. It was the only gate that limited reach at the paper, costing 2 cm at full stretch, and self-collision passes at the rim |
| pen lean | 15° | |
| speed on the paper | 0.015 m/s | the pen's (below) |
| press | 0.0021 m | the pen's (below) |
| landing speed | 0.003 m/s | 0.010 until 2026-10-06 (site) |
| lift height | 0.025 m | to be replaced by the pen's clearance to the paper + 2 mm |
| shortest piece | 0.010 m | |
| speed fraction | 0.30 | of the joint speed limits |

## Pens

`rig.json` has a `pens` block: `current` names the pen that is in, `table` holds every pen by
name. `graphite_4h` (2 mm 4H graphite in the lateral holder) is in:

| | value | used by |
|---|---|---|
| `press_m` | 0.0021 (what drew on 2L/2R, site 2026-10-06; 0.0035 before) | `rules().press`: the system planner puts the drawing's points this far below the paper (position control presses by planning there) |
| `speed_m_per_s` | 0.015 | `rules().draw_speed`, the speed of the tip along a line |
| `tip_length_nominal_m` | 0.020 | the tool model: how far the graphite stands past the cap's outer face. The kernel's tool is built for 0.020 (`MODEL_TIP_LENGTH` in `rig.py`); another length moves the tip along the pen axis by the difference |
| `capsule_radius_m` | 0.005 | the pen capsule's radius in the tool model |
| `force_band_n`, `force_levels`, `force_cap_n`, `force_ramp_m`, `lift_ramp_s`, `servo_ki_per_s`, `servo_trim_max_n` | 0.7-1.0 N, 9, 3.5 N, 0.002 m, 0.2 s, 1.0 /s, 1.0 N | mode B only (the impedance controller): the operator PC reads them from the job header |

The second pen, `gel_g2`, presses 0.0025 at 0.015 m/s with the same length, capsule and force
block, and carries `"drag_only": true`: a gel pen in the lateral holder skids when pushed and
draws when pulled, and the planner uses the flag to draw it only pulled.

`pen()` returns the current entry (notes and source left out) plus `"name"`; the server copies it
into every job header. The drawing file stays pen-agnostic.

The old `drawing.draw_speed_m_per_s` is gone from `rig.json`. It is still read as a fallback for
a pen that has no `speed_m_per_s` (the checker's reader does the same); a pen with neither is a
broken install and stops the load.

## Calibration file

`config/calibration/<slot>.json` has two parts on two clocks (DESIGN 4c):

```
{"slot": "2R",
 "base": {"passed": true, "date": "...", "method": "plane",
          "T_table_base": [[...4x4...]], "residuals": {...}, "why": ""},
 "pen":  {"passed": true, "date": "...", "pen": "graphite_4h", "tip_hand_m": [x, y, z],
          "reference_touch": {"xy_table_m": [x, y], "q": [...7...]}, "why": ""}}
```

- `base` (the plane job now, the dimples later; redone when an arm or the frame moves) replaces
  the slot's nominal pose when it passed. The paper plane, the parked body and the hanger move
  with it; walls do not.
- `pen` (a one-touch touch-off; redone after every pen switch or handling of the pencil) replaces
  the tip when it passed **and** names the pen that is in (`pens.current`). The pen capsule moves
  onto the line through the new tip and ends exactly at it (5.6e-17 m in the test). Otherwise the
  tip comes from the pen's nominal length.
- Each part is applied on its own; either may be missing. `calibration_status(a)` reports both,
  for instance `{"base": "applied: 2R.json base (2026-10-02, plane)", "pen": "pen part not
  applied: it was measured for pen 'gel_06', the pen in is 'graphite_4h'"}`. `calibrated(a)` is
  true when both applied.
- A file for another slot, a pose that is not a rigid transform, or a tip that is not three
  numbers stops the load with an error. The old one-part format is not read.

The tests move 2L by (3, −2, 1) mm and tilt it 0.5°: that changes 2L's pose, its paper plane and
its own hanger, and nothing else (walls, other steel, other arms). Moving 2L 20 mm in x and y
moves its four hanger boxes by exactly that much.

## Calibration marks

`rig.json` `marks` lists the ten calibration spots of DESIGN section 6 ("Steps 2 and 3 as
built", `docs/figures/marks_six_slots.png`), each shared by exactly two neighbouring slots:

| mark | nominal (x, y) m | shared by |
|---|---|---|
| A, B | (0, −0.40), (0, +0.40) | 2L, 2R |
| R1a, R1b | (0, −1.61), (0, −0.81) | 1L, 1R |
| R3a, R3b | (0, +0.81), (0, +1.61) | 3L, 3R |
| S12L, S12R | (−0.30, −0.605), (+0.30, −0.605) | 1L+2L, 1R+2R |
| S23L, S23R | (−0.30, +0.605), (+0.30, +0.605) | 2L+3L, 2R+3R |

Every slot shares at least two spots (its yaw). The frame is anchored on the arms' nominal
mountings (a rigid fit of the solved slot positions onto them); the marks are solved wherever
they really are, 2–5 cm from the nominal spots by hand. `groups` names the subsets
`aris mark --group` takes: `all`, `row2`, `rows12`, `rows23`.

`Rig.marks` (name → nominal xy and the two sharers), `Rig.mark_groups`, and
`rig.marks_for(slots)`, the marks whose sharers are all in `slots`. `config/calibration/marks.json`,
when present, gives per mark `xy_m`, `state` (nominal or solved), `date` and `residual_mm`
(either flat or under a `"marks"` key); `rig.mark_xy(name)` is the solved position when solved,
else the nominal one, and `rig.mark_state(name)` says which. Entries for marks this rig does not
have are ignored, because the calibration folder is shared with the rigs of fewer arms.

## The two-arm rig

`tools/mounted_rig.py --arms 2L,2R --area 1.72 0.9 --centre 0 0 --out config/two_arms` writes
`config/two_arms/rig.json`: `config/rig.json` with only 2L and 2R mounted, a drawing area of
1.72 x 0.9 m about the table centre, fences toward rows 1 and 3 (1L and 1R hang there switched
off; 3L is the floor arm, whose body stands at the row 3 end, beyond the y = +0.605 fence, as
long as it stays folded in its own row; 3R is empty), the marks both of whose sharers are controlled (A and B) and the mark groups made
of controlled slots only (`row2`). Every hanger stays except where a slot has none (3L, 3R). In a row with one controlled arm, the
switched-off arm beside it would get a fence at x = 0 (`fence_col_minus_x` or
`fence_col_plus_x`, written once for several rows). The tool prints the area and its centre and
reminds you that the area must lie inside the area the drawable maps give about that same
centre (the system planner computes it; the server refuses to start otherwise).
`--check config/two_arms` says whether the derived file is still in step with `config/rig.json`.

## Assumed, not measured

- The tool's clearance to the paper is 0, which means it must not touch. Pete chose this on
  2026-09-30. There will be no measured holder height, so the CAD and pen models are used as they
  are. It is also the limit on how far a pen may wear.
- **The struts are placed from the drawing, and Pete's tape disagrees with it.** On 2026-09-16 the
  tape read 156 and 240 mm from the axis to the two outside faces. The drawing says 171.75 and
  222.05 (now −x 222.05 and +x 171.75, the frame being the drawing turned), a 42 mm offset
  against the drawing's 25.15. Pete decided on 2026-09-30 to use
  the drawing (sheet 3, panel C lists the disagreement).
- The strut heights, the plate, the clamp, the runways, rails and legs are the old drawing-based
  model. The seam bars are "representative", and the corner legs were assumed in the old model.
- **How far the hanging struts reach below the mounting plate is not measured, and will not be.**
  The old model says 35 mm. On Pete's instruction the struts are 30 mm longer, 65 mm below the
  plate, to be conservative. Link 1 then passes the near strut's end (+x since the turn) with 50.8 to 62.4 mm of room,
  against the 0.020 it must keep to its own mount (see "Link 1").
- The park configurations are the old home parks. They were not searched for the new phase
  scheme.

## Tests

`tests/test_rig.py`: 50 tests, about 5 s in all. They cover the poses
against the old code, the walls, the steel, the parks, link 1, the pens, the slot checks, the
two calibration parts, hangers following the calibration, and the two-arm rig with its fences
(for the planner and the checker), and the calibration marks.
