# calib — the paper plane under each arm (calibration step 1)

**Job.** Find how the paper really lies under one arm: its tilt and its height, measured with the
arm's own joint sensors. The arm has touched the paper at a grid of about 25 points with its pen
upright and the hand turned the same way at every point. This module turns those touches into the
arm's corrected pose on the table and writes it to `config/calibration/<arm_id>.json`, which
`aris/rig.py` reads on the next start. Nothing else applies a calibration: the rig reads the file.
This is step 1 of DESIGN.md section 6; the dimples (steps 2 and 3) come later and add on top.

**In.** The rig (the arm's nominal pose and nominal pen tip, the paper at table z = 0) and the
joints of every touch, in order. They come either as an array (N x 7) or as the event rows the
executor writes (`{"event": "contact", "arm": id, "q": [...], ...}`); rows of other arms and
other events are skipped.

**Out.** A `PlaneCalibration`: pass or fail with a one-line reason, the corrected 4 x 4
`T_table_base`, the tilt found (roll and pitch), the height change, how far the touches lie from
the fitted plane (RMS and largest, and which touch), and every touch in the table frame (the
height map). `write_calibration(result, path)` writes the file.

| call | does |
|---|---|
| `fit_plane(points)` | least-squares plane through the points: unit normal (pointing toward the frame's origin, the arm's base), offset, signed distances |
| `calibrate_plane(rig, arm_id, contacts_q)` | everything above, never an exception on bad data |
| `calibration_from_events(rig, arm_id, rows)` | the same, from an event list |
| `write_calibration(result, path, date=None)` | the one writer |

## How it works

1. Each touch's joints go through the arm model's forward kinematics with the pen tip the
   planners use for this arm (`rig.arm(arm_id).tip(q)`): the pen tip in the arm's base frame.
2. A plane is fitted through those tips by least squares. Its normal points from the paper toward
   the arm.
3. The corrected pose. Start from the nominal pose. Turn the base about a horizontal table axis
   through the base's origin, by the smallest turn that makes the table's up direction (as seen
   from the base) equal to the fitted normal. Then move the base up or down along table z until
   the fitted plane sits at the paper height. x, y and the turn about the vertical are not
   changed. **Convention**, also written in the file: new rotation = R(v) times nominal rotation,
   where v = (roll, pitch, 0) is a rotation vector in the table frame; roll is about table x,
   pitch about table y.
4. The pass rule (all limits in one place at the top of `aris/calib/plane.py`):

| limit | value | catches |
|---|---|---|
| touches | at least 9 | too few to tell a tilt from one bad touch |
| spread | 20 mm across | touches on a line, or all at one spot: no plane |
| RMS distance from the plane | below 0.5 mm | a paper that is not one flat surface (loose, bent table), or touches from two set-ups mixed |
| largest distance | below 1.5 mm | one slipped or early touch; the reason names it |
| tilt | below 3 deg | the wrong arm's touches, or an arm not mounted as `rig.json` says |
| height change | below 30 mm | a plane far from nominal: wrong pen, wrong base height, or the descent went past the paper |

   Every limit that fails is named in `why`, joined in one line. A failed file is still written
   (with `why`); the rig does not apply it and says "not applied".
5. The height map: every touch in the corrected table frame. Its z is that touch's distance from
   the fitted plane: the unevenness of the table, kept for the person and for later.

## The file

What the rig reads: `arm_id`, `date`, `passed`, `T_table_base`. What a person reads: `rms_mm`,
`max_residual_mm`, `worst_point`, `roll_deg`, `pitch_deg`, `tilt_deg`, `height_change_mm`,
`n_points`, `height_map_table_m` (x, y, z per touch), `T_table_base_nominal`, `convention`, and
`why` when it failed.

**No `tip_hand_m`.** The pen tip stays the rig's nominal one. With the hand turned the same way at
every touch, an error in the pen's length moves every touch alike, so it cannot be told apart from
the paper height: it lands in the height. That is the height we want, where this pen meets the
paper. The pen's real length comes from the dimple (step 2) or the drawn check.

## What it cannot do

- It cannot see x, y or the turn about the vertical: a flat paper looks the same when the arm
  slides over it. Those stay nominal until steps 2 and 3.
- It cannot see the pen length (above).
- A wrong joint zero (a joint off by a fixed angle) bends the measured surface; small ones show
  up as tilt and height, large ones as scatter and a refusal. It cannot say which joint.
- It does not plan or fly the touches; that is the calibration job (`aris calibrate <arm>`), not
  built yet.

## Measured numbers

Synthetic touches, the kernel's forward kinematics as the truth: true planes tilted up to 2 deg in
roll and pitch and raised up to 20 mm, a 5 x 5 grid of 0.4 m around the arm's axis, tips put
exactly on the plane by the IK (pen upright, one spin), arms 31 and 71 (`config/two_arms`) and 13
(`config`), then joint noise on every joint. Over 10 seeds (arm 31, 1.5/-2.0 deg, +20 mm):

| joint noise | tilt error, worst | height error, worst | fit RMS, mean | passes |
|---|---|---|---|---|
| 0.5 mrad | 0.024 deg | 0.055 mm | 0.14 mm | 20 of 20 |
| 2 mrad | 0.097 deg | 0.22 mm | 0.56 mm | 7 of 20 (RMS or largest-distance rule) |

At 1 mrad all 20 pass (largest distance 0.95 mm). The limits suit encoder-level noise; a
real fit RMS near 0.5 mm means something other than noise.

A 40 mm plane is refused for its height, a 3.5 deg tilt for its lean, one touch 5 mm above the
paper by the largest-distance rule naming that touch. Under a millisecond per arm. Tests:
`tests/test_calib.py` (14 tests, under a second).
