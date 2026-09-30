# rig — what the installation looks like

**Job.** The one place that knows the table, the paper, where the six arms hang, the steel around
them and the clearances. It is also the one place that turns table-frame things into one arm's
base frame. Every planner below the system planner sees only base-frame geometry, and all of it
comes from here.

**In.** `config/rig.json`, plain data with a source note on every block, and optionally
`config/calibration/<arm_id>.json` per arm. Nothing else in the package reads `config/`.

**Out.** `Rig.load(config_dir)` gives a `Rig` with these calls:

| call | gives |
|---|---|
| `arm_ids`, `T_table_base(a)`, `T_base_table(a)`, `park_q(a)` | the arms and their poses |
| `arm(a)` | the arm model with this arm's tool (the calibrated pen tip if there is one) |
| `to_base(a, line)`, `to_table(a, points)` | lines and points moved between the frames |
| `paper(a)` | the paper as a plane in `a`'s base frame, free side up, with three margins: links, lifted pen, rest of the tool |
| `wall_between(a, b)`, `wall_in_base(a, wall)` | a wall in the table frame, and as a plane for one arm |
| `phase(n)` | phase 1 or 2: the leaders move, the other three stand parked, walls 13-71 and 71-2 (phase 1) or 17-31 and 31-97 (phase 2) |
| `obstacles_for(a, phase)` | `obstacles` for a moving arm, with the parked arms it can reach (always its row partner) and the walls it stands next to |
| `obstacles(a, parked=(), walls=(), for_planning=True)` | everything `a` must stay clear of |
| `gates()` | the planners' `Gates`, all from `rig.json`: joint-limit margin 0.15 rad, smallest singular value 0.04, self margin 0.020 + 0.003 |
| `rules()` | the drawing rules (`DrawRules`), all from `rig.json`, with `gates()` inside. Every planner takes them from here, not from the type defaults |
| `leaders(phase)`, `row_partner(a)` | (13, 71, 2) in phase 1, (17, 31, 97) in phase 2; 13-17, 31-71, 2-97 |

## Frames

The table frame has its origin at the table centre, on the paper. x runs across the table,
y along it, z up. The old code put its origin at a canvas corner: table = old − (0.9017, 1.81532, 0).
Each arm's base frame is the arm model's own origin, on the underside of its mounting plate. The
arms hang upside down, so base z points down and base x points to table −x.

```
            y
            ^        table 2.188 x 4.1656, paper 1.8034 x 3.6306, both centred
  +1.2102   |    2 o-----o 97
            |      |  \  |        o = an arm's first-joint axis, 0.970 above the paper
      0     |   31 o-----o 71     \ = the phase 1 wall between 13 and 71, and between 71 and 2
            |      |  /  |
  -1.2102   |   13 o-----o 17
            +------------------> x
               -0.305  +0.305
```

| arm | axis x | axis y | row partner | leader in phase |
|---|---|---|---|---|
| 13 | −0.305 | −1.2102 | 17 | 1 |
| 17 | +0.305 | −1.2102 | 13 | 2 |
| 31 | −0.305 | 0 | 71 | 2 |
| 71 | +0.305 | 0 | 31 | 1 |
| 2 | −0.305 | +1.2102 | 97 | 1 |
| 97 | +0.305 | +1.2102 | 2 | 2 |

The rows are exactly a sixth of the old 3.63064 m canvas apart (1.2102133 m), because that is
where the old code put them. With those numbers the base poses match the old code to 2e-16.

## Walls

A wall is a vertical plane halfway between two arms' axes and square to the line joining them.
The wall between 13 and 71 passes through (0, −0.6051), 0.6776 m from both axes, turned 26.75°
from x. The phase 2 wall between 17 and 31 passes through the same point, so the two phases'
walls cross there. As a plane for one arm, the free side is the side that arm's axis is on, and
the margin is 0.025 m, half the arm-to-arm clearance. Walls use the nominal axis positions, so a
calibration does not move them.

## Steel

All steel is axis-aligned boxes in the table frame. Every box has a source note.

| part | boxes | numbers from |
|---|---|---|
| hanging struts | 2 per arm, 0.0762 x 0.1524 section (long side along y) | Pete's tape, 2026-09-16: outside face to axis 0.240 on one side and 0.156 on the other. Top at the runway underside (old model). Bottom 65 mm below the plate underside (0.905): the old model's 35 mm plus 30 mm, on Pete's instruction (2026-09-30), to be conservative because nobody will measure it. |
| mounting plate | 1 per arm, 0.2258 x 0.190 x 0.0127 | old model (drawing). Centred between Pete's struts, so 0.042 toward the wide side. |
| clamp stack | 1 per arm, on the plate | old model (drawing) |
| runways | 3, one double beam per row at 1.624 to 1.700 | old model (drawing) |
| seam bars | 2, beside the table at y = 0, from the table top to the runway | old model, "representative, not measured" (Pete, 2026-09-14) |
| perimeter rails and corner legs | 4 + 4 | old model (rails from the drawing, legs assumed) |

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
drawing's 12.7 mm plate, moved 42 mm to sit between the struts. It could not sit centred: Pete's
narrow strut is 80 mm from the axis. The old set had each neighbour's "boom" as one 0.2 m square
column up to 2.34 m. Those are replaced by the two measured struts, and the runway is at the
drawing's 1.62 m. The old set also had 4 "body column" boxes per neighbour (20 per arm). Those
are arms, not steel. They now arrive as a parked arm's capsules or stay behind a wall. The old set
had no runways, rails or legs, and no clamps.

## Parked arms

`obstacles(a, parked=(17,))` adds arm 17's whole collision body, standing at its park
configuration, as capsules in `a`'s base frame. The park configurations are the old code's home
parks (`Q_PARK_PROPOSED`, h = 0.970, lateral holder).

Each park against its steel, the paper and itself. The first four columns come from the old code's
body model and are plain distances (demanded: paper 0.020, pen 0.003, self 0.020, steel 0.050). The
last column comes from the new kernel and is measured beyond each obstacle's demanded margin:

| arm | paper (body) | pen tip height | self | new steel, old body model (35 mm struts) | everything, new kernel, beyond the margin |
|---|---|---|---|---|---|
| 13 | 0.250 | 0.300 | 0.123 | 0.179 (own strut) | +0.130 (link 2, own strut) |
| 17 | 0.250 | 0.300 | 0.153 | 0.179 (own strut) | +0.131 (link 2, own strut) |
| 31 | 0.300 | 0.350 | 0.126 | 0.080 (seam bar W) | +0.079 (link 6, seam bar W) |
| 71 | 0.150 | 0.200 | 0.133 | 0.179 (own strut) | +0.135 (link 2, own strut) |
| 2 | 0.250 | 0.300 | 0.124 | 0.179 (own strut) | +0.112 (link 2, own strut) |
| 97 | 0.250 | 0.300 | 0.114 | 0.179 (own strut) | +0.150 (link 2, own strut) |

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

The worst clearance beyond the required margin, for every arm:

| own hanger (0.020) | other steel | walls, both phases | other arms parked | paper |
|---|---|---|---|---|
| +0.019 | +0.257 or more | +0.204 | +0.216 or more | +0.488 |

With the struts 30 mm longer, link 1 passes under their ends with 39 to 50 mm of room, so
19 to 30 mm beyond the 0.020.

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
| a wall | 0.025 | 0.0015 (half, because both arms pay it) |
| paper, arm body | 0.020 | 0 |
| paper, lifted pen | 0.003 | 0 |
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
| draw speed | 0.02 m/s | |
| lift height | 0.025 m | to be replaced by the pen's clearance to the paper + 2 mm |
| shortest piece | 0.010 m | |
| speed fraction | 0.30 | of the joint speed limits |

## Calibration file

`config/calibration/<arm_id>.json` is written by the calibration job:

```
{"arm_id": 31, "date": "2026-10-01", "passed": true,
 "T_table_base": [[...], [...], [...], [0, 0, 0, 1]],   measured base pose, table frame
 "tip_hand_m": [x, y, z],                              optional: measured pen tip, hand frame
 "source": "what produced it"}
```

If `passed` is true, the file replaces that arm's nominal base pose and pen tip. The pen
capsules move with the tip. If it is false,
the file is ignored, and `calibration_status(a)` says so. A pose that is not a rigid transform
stops the load with an error. The test file moves arm 31 by (3, −2, 1) mm and tilts it 0.5°. That
changes arm 31's pose and its paper plane, and nothing else (walls, steel, other arms).

## Assumed, not measured

- The tool's clearance to the paper is 0, which means it must not touch. Pete chose this on
  2026-09-30. There will be no measured holder height, so the CAD and pen models are used as they
  are. It is also the limit on how far a pen may wear.
- **The 0.240 side of every arm's struts is toward −x.** This is not confirmed; it is one setting
  per arm, `strut_wide_side`. The old model inferred the opposite, with its wider side toward +x.
- The strut heights, the plate, the clamp, the runways, rails and legs are the old drawing-based
  model. The seam bars are "representative", and the corner legs were assumed in the old model.
- The plate sits centred between the struts. That is inferred from its size, not seen.
- **How far the hanging struts reach below the mounting plate is not measured, and will not be.**
  The old model says 35 mm. On Pete's instruction the struts are 30 mm longer, 65 mm below the
  plate, to be conservative. Link 1 then passes under them with 39 to 50 mm of room, against the
  0.020 it must keep to its own mount (see "Link 1").
- The park configurations are the old home parks. They were not searched for the new phase
  scheme.
