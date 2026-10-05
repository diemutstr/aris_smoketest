# calib: the paper under each arm, where each arm hangs, and the pen's length

**Job.** Measure two things with each arm's own joint sensors and write them to the slot's
calibration file, `config/calibration/<slot>.json` (slots `1L` .. `3R`):

- **base**: how the paper lies under the arm, its tilt and height (the plane job, DESIGN.md
  section 6 step 1). Re-done when an arm or the frame moves.
  The mark job (steps 2 and 3) then adds x, y and the turn about the vertical, and rewrites
  the same part with `method: "marks"`.
- **pen**: where the pen tip really is in the hand (the touch-off, DESIGN 4c and section 6
  step 4, or the mark job's pivot). Re-done after every pen switch or handling of the pencil.
  With position control and a geometric press, the pen length sets the tone.

The mark job also writes `config/calibration/marks.json`: where each mark really is.

Each part has its own date and pass flag. `aris/rig.py` reads the file when it loads. Nothing
else applies a calibration.

## Calls

| call | does |
|---|---|
| `fit_plane(points)` | least-squares plane: unit normal (pointing toward the frame's origin, the arm's base), offset, signed distances |
| `calibrate_plane(rig, slot, contacts_q)` | the plane job's solver → `PlaneCalibration` |
| `calibration_from_events(rig, slot, rows)` | the same, from the executor's event rows (`{"event": "contact", "arm": slot, "q": [...]}`); other slots and events are skipped |
| `touchoff(rig, slot, contact_q, reference_xy_table, pen_name)` | the touch-off solver → `PenCalibration` |
| `files.write_base(result, config_dir)`, `files.write_pen(result, config_dir)` | each rewrites only its own part of the slot's file and keeps the other one. They create the file when it is missing and write it whole, so a reader never sees half a file |
| `pivot(rig, slot, Q)` | 3 or more touches with the tip seated on one mark → `Pivot`: the tip in the hand frame, the mark in the base frame |
| `touch_point(rig, slot, q, tip)` | one touch with a known tip → the point in the base frame |
| `solve_marks(rig, touches, known=None, base_tips=None)` | the mark job's solver → `MarkSolution`: every slot's x, y, yaw and every mark's xy |
| `files.write_mark_solution(rig, solution, config_dir)` | the mark job's one writer: every slot's base part (`method: "marks"`), its pen part (the pivot's tip) and `marks.json`. Also callable one by one: `write_base(slot_fit, ...)`, `write_pen(...)`, `write_marks(solution, ...)` |
| `files.base_tips(config_dir, slots)` | the tip each slot's current base height was measured with, for `base_tips` |
| `simulate.simulate_touches(rig_true, rig_nominal, slots, marks, noise)` | what a person guiding the arms would register on a true rig; for tests and the simulated driver |
| `files.read(config_dir, slot)`, `files.listing(config_dir)` | one file as written; every file with its digest and each part's passed / date / why |

The solvers never raise on bad data: they return `passed` False with a one-line `why`.

## The plane job (base)

The arm touches the paper on a grid of about 25 points, with the pen upright and the hand turned
the same way at every point.

1. Each touch's joints go through forward kinematics with the tip the rig uses now
   (`rig.arm(slot)`), which gives the pen tip in the base frame.
2. A least-squares plane is fitted through the tips.
3. The base is turned about a horizontal table axis through its origin. The turn is the
   smallest one that makes the table's up direction, seen from the base, equal to the fitted
   normal. Then the base is moved along table z until the plane sits at the paper height. x, y
   and the turn about the vertical are not changed. **Convention** (also in the file):
   - new rotation = R(v) · rotation before, with v = (roll, pitch, 0), a rotation vector in
     the table frame.
   - "Before" is the pose the rig had when the touches were made: rig.json's, or an earlier
     base part.
4. The pass rule. All limits are at the top of `plane.py`:

| limit | value | catches |
|---|---|---|
| touches | at least 9 | too few to tell a tilt from one bad touch |
| spread | 20 mm | touches on a line or at one spot: no plane |
| RMS distance from the plane | 0.5 mm | a paper that is not one flat surface, or touches from two set-ups mixed together |
| largest distance | 1.5 mm | one slipped or early touch; the reason names that touch |
| tilt | 3 deg | the wrong arm's touches, or an arm not mounted as rig.json says |
| height change | 30 mm | a plane far from nominal: the wrong pen, the wrong base height, or the descent went past the paper |

5. The height map is every touch in the corrected table frame. Its z is that touch's distance
   from the plane, which is the unevenness of the table.

**The height includes the pen.** With one hand orientation, an error in the pen length moves
every touch alike, so it lands in the height. The base part records which pen and which tip the
touches were made with (`measured_with`).

**The first touch-off after a plane job therefore finds no change.** By construction, the tip it
measures is the tip the plane was measured with. Later touch-offs measure how much the pen has
changed since then. This also keeps the two parts consistent: a pen part written before a new
plane job still fits after it, as long as the rig used that pen part during the plane job. The
rig does, when the pen in matches the part.

## The touch-off (pen)

The pen touches the paper once, near a reference point.

1. The tip the rig uses now is put at the touch's joints, and its height above the measured paper
   (the base part, applied by the rig) is found.
2. The pen really ends on the paper, so the tip is moved along the pen axis in the hand frame
   until it touches. The pen leans 23 deg in the holder, so 1 mm of extra length moves the tip
   0.92 mm down and 0.39 mm sideways. Moving along the axis corrects both.
3. The new `tip_hand_m` is written together with the reference touch (its xy and q). Later
   touch-offs reuse the same point.

| refusal | why |
|---|---|
| no passing base part | the paper must be measured first |
| another pen than the rig's `pens.current` | the result would be for the wrong pen |
| tip moved more than ±5 mm from the pen's nominal length | the wrong pen, or a slipped or broken lead |
| touch more than 3 cm from the reference point | the arm touched somewhere other than planned |
| pen more than 60 deg off the paper's normal | a flat pen says little about its length |

The rig applies the pen part only when it passed and names the pen that is in. Otherwise the
status says, for example, `pen part not applied: it was measured for pen 'graphite_4h', the pen in
is 'gel_07'`.

## The mark job (base x, y, yaw; and a pen tip)

There are ten marks, small dimpled plates on the table. Each is shared by two neighbouring
slots: the row pairs share two marks on the centre line, the column pairs one seam mark each
(rig.json `marks`). A person guides the pen into each mark.

- At its **first mark** a slot is guided through 3 or 4 hand orientations, with the pen kept
  seated. This is the **pivot**. Each touch says "hand pose k times tip = mark", which gives
  6 unknowns and 3 equations per touch. Least squares gives the tip in the hand (its true
  length included) and the mark in the base frame.
- At **every other mark** it touches once, read with that tip.

**The solve.** Every touch, seen from the base origin in table axes, is a horizontal vector e.
The mark is where Rz(yaw)·e + (x, y) puts it. Unknowns: each slot's (x, y, yaw) and each mark's
(x, y).
- **The frame** is a convention. Mark A stays at its nominal position and A→B points along +y.
  If at least two of the touched marks are already known (solved earlier), those fix the frame
  instead.
- **Two passes.** First a closed form: each slot is fitted rigidly onto the marks' current
  estimates, then each mark is averaged from its slots, three rounds. Then one least-squares
  refinement over everything.
- **Kept from before.** z, roll and pitch stay as the rig had them (the plane job). When
  `base_tips` names the tip the plane was measured with, z moves by the hand-z part of (pivot
  tip − that tip). This is exact for the plane job's upright touches, where the hand's z axis
  is the paper's normal. It keeps the new tip and the old height consistent. Without it, a pen
  1.3 mm longer than nominal would draw 1.2 mm too high.
- **Convention** (also in the file): new rotation = Rz(yaw) · rotation before; x and y solved.

**Subsets.** `known` holds the marks solved earlier. A subset (`rows12`) or a single slot uses
them as fixed points. Only the marks this solve placed itself go to `marks.json`; known marks
and untouched marks keep their entries.

**Refusals and flags.** All limits are at the top of `marks.py`.

| rule | value | catches |
|---|---|---|
| pivot touches | at least 3 | too few to separate the tip from the mark |
| pivot spread | the pen axes at least 15 deg apart | the tip's length barely seen |
| pivot condition | at least 0.1 | the hand only turned about one axis: the tip's part along that axis is free. Tilt, do not only spin |
| pivot residual | 1.0 mm | a touch where the pen left the dimple; the reason names it |
| anchors | A and B touched, or two known marks | otherwise: "needs two anchors" |
| partners | each slot needs two marks that something else pins (known, the frame, or touched by another slot) | otherwise: "<slot> needs a partner: only 1 shared mark" |
| pair distance | two slots measuring the same two marks agree within 1.5 mm | a mark moved between them, or a touch is off; the reason names the pair |
| pose | within 30 mm of rig.json's axis, and within 3 deg of yaw of the pose before | "refused: 2R 0.045 m … — wrong slot or wrong robot?" |
| RMS per touch | 1.0 mm | touches that do not fit one rigid layout |
| rigidity | full rank | a group of slots hanging on the rest by one mark only |

A mark touched by one slot only, and not known, is solved but flagged "determined by one arm":
it constrains nothing.

**Pen part from the pivot.** The tip goes into the slot's pen part with `method: "pivot"` and
`reference_touch` null. A mark is no place for a touch-off, because the pen would sit in its
dimple, so the next touch-off picks its own reference point. The marks base part keeps the
plane job's numbers under `plane`.

## What it cannot do

- The plane job cannot see x, y or the turn about the vertical: a flat paper looks the same
  when the arm slides over it. The mark job provides them.
- The mark job's accuracy is limited by the person and the joints, not by the solver. The
  numbers below show it. The weakest link is the seam between rows: two seam marks only 0.4 m
  apart tie each end row to the middle one, so an end row's turn is known several times
  worse than the middle row's.
- A wrong joint zero bends the measured surface. A small one shows up as tilt and height, a
  large one as scatter and a refusal. It cannot say which joint.
- The touch-off is one touch and is not averaged: its error is the joint noise of that touch.
- It does not plan or fly the touches. That is the server's job (`aris calibrate`,
  `aris touchoff`, `aris mark`).

## Measured numbers

All tests use synthetic touches, with the kernel's forward kinematics as the truth.

**Plane job.**
- Set-up: true planes tilted up to 2 deg in roll and pitch and raised up to 20 mm, a 5 × 5 grid
  0.4 m wide around the axis. The IK puts the tips exactly on the plane (pen upright, one spin).
  Slots 2R and 3R come from `config/two_arms`, 1L from `config`.
- Results over 10 seeds for slot 2R, tilted 1.5 / −2.0 deg and raised 20 mm:

| joint noise | tilt error, worst | height error, worst | fit RMS, mean | passes |
|---|---|---|---|---|
| 0.5 mrad | 0.024 deg | 0.055 mm | 0.14 mm | 20 of 20 |
| 2 mrad | 0.097 deg | 0.22 mm | 0.56 mm | 7 of 20 (RMS or largest-distance rule) |

  At 1 mrad all runs pass, with a largest distance of 0.95 mm.

**Touch-off.**
- A pen 1.3 mm longer than nominal is found to 1e-9 mm without noise. The tip moves along
  the pen axis only.
- An 8 mm error is refused.
- With 0.5 mrad of joint noise, over 10 seeds, the length error is at most 0.31 mm (mean 0.12 mm).

**Mark job.**
- Truth: every slot is moved 1–2 cm in x and y, 3–8 mrad in yaw, 1 deg in roll and pitch and
  10 mm in z. The pen is 1.3 mm longer than nominal, and the marks are taped up to 5 mm off
  nominal (A exactly nominal, B straight along +y from it).
- The rig the solver sees is what the plane job left: the true tilt, the height read with the
  nominal pen, nominal x, y and yaw.
- Touches: the pivot uses 4 orientations, spread from turns of 30 deg and tilts of 30 deg. The
  guiding error is one offset per slot and mark, the pen seated off the dimple's centre. Joint
  noise is added on every touch.
- "Seam" is the worst distance between where two neighbours put the pen when both aim at the
  same point of their seam: (0, 0) for 2L/2R, the row seams and the seam marks for the others.
  This is what shows in a drawing.

Without noise every case is exact: below 1e-6 mm, and below 1e-6 mrad for yaw.

| case | noise (guiding, joints) | x, y | yaw | tip | z | seam |
|---|---|---|---|---|---|---|
| (a) 2L+2R, A and B | 0.3 mm, 0.3 mrad | 0.58 mm | 1.06 mrad | 0.49 mm | 0.41 mm | 1.29 mm |
| (b) six slots, ten marks | 0.3 mm, 0.3 mrad | 3.9 mm | 5.2 mrad | 0.88 mm | 0.75 mm | 1.18 mm |
| (c) rows12 after (b) | 0.3 mm, 0.3 mrad | 3.8 mm | 4.9 mrad | 1.33 mm | 1.09 mm | 1.23 mm |
| (d) 2R alone after (b) | 0.3 mm, 0.3 mrad | 0.69 mm | 0.24 mrad | 0.48 mm | 0.35 mm | — |
| (a) | 0.1 mm, 0.05 mrad | 0.13 mm | 0.23 mrad | 0.10 mm | 0.09 mm | 0.30 mm |
| (b) | 0.1 mm, 0.05 mrad | 0.72 mm | 0.99 mrad | 0.16 mm | 0.15 mm | 0.26 mm |
| (c) | 0.1 mm, 0.05 mrad | 0.73 mm | 0.95 mrad | 0.16 mm | 0.12 mm | 0.28 mm |
| (d) | 0.1 mm, 0.05 mrad | 0.14 mm | 0.11 mrad | 0.10 mm | 0.08 mm | — |

Each row is the worst of 3 seeds.
- The large x, y and yaw errors in (b) and (c) are the end rows. They are tied to the middle
  row by two seam marks 0.4 m apart, and the position error grows with the distance from the
  frame's anchor A. The seams, which are what a drawing shows, stay near 1.2 mm.
- With a single pivot of four touches, 0.3 mrad of joint noise alone limits the tip to about
  0.5 mm.

Tests: `tests/test_calib.py`, 25 tests. 24 quick ones in about 5 s; the noise table is
`slow` and takes about 10 s.
