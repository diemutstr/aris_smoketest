# calib: the paper under each arm, and the pen's length

**Job.** Measure two things with each arm's own joint sensors and write them to the slot's
calibration file, `config/calibration/<slot>.json` (slots `1L` .. `3R`):

- **base**: how the paper lies under the arm, its tilt and height (the plane job, DESIGN.md
  section 6 step 1). Re-done when an arm or the frame moves.
- **pen**: where the pen tip really is in the hand (the touch-off, DESIGN 4c and section 6
  step 4). Re-done after every pen switch or handling of the pencil. With position control
  and a geometric press, the pen length sets the tone.

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

## What it cannot do

- It cannot see x, y or the turn about the vertical: a flat paper looks the same when the arm
  slides over it. These wait for the dimples (steps 2 and 3).
- A wrong joint zero bends the measured surface. A small one shows up as tilt and height, a
  large one as scatter and a refusal. It cannot say which joint.
- The touch-off is one touch and is not averaged: its error is the joint noise of that touch.
- It does not plan or fly the touches. That is the server's job (`aris calibrate`,
  `aris touchoff`).

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

Tests: `tests/test_calib.py`, 20 tests, about 2 s.
