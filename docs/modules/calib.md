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
| touches | at least 8 | too few to tell a tilt from one bad touch |
| spread | 20 mm | touches on a line or at one spot: no plane |
| RMS distance from the plane | 1.0 mm | a paper that is not one flat surface, or touches from two set-ups mixed together |
| largest distance | 2.5 mm | one slipped or early touch; the reason names that touch (a touch 5 mm off is still refused) |

The first limits were 9 points, 0.5 mm RMS and 1.5 mm largest. On 2026-10-06 they refused
every real plane on the wood: RMS 0.87–0.92 mm and largest 1.69–1.83 mm with 8–12 points,
from the table's flatness and the touch noise. The values above are the ones the site passed
them with.
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

There are ten marks: sharpie cross-hairs drawn on the wood by eye, 2–5 cm from their nominal
places. Each is shared by two neighbouring slots: the row pairs share two marks on the centre
line, and the column pairs one seam mark each (rig.json `marks`). A person guides the pen onto
each mark. Every touch is a fresh seating: the arm lifts and returns to its hover in between.

- **The pivot.** At its first mark a slot is guided through six hand orientations, tilted and
  turned, with the tip on the mark.
  - Each touch says "hand pose k times tip = mark": 6 unknowns, 3 equations per touch.
  - Least squares gives the tip in the hand (its true length included) and the mark in the
    base frame.
  - The tilts matter. Turning the hand only about the vertical leaves the tip's part along
    that axis free, and the solver refuses such a pivot.
- **Every other mark** is touched once and read with that tip.

**The solve** (`planar.py`). Every touch, seen from the base origin in table axes, is a
horizontal vector e; the mark is where Rz(yaw)·e + (x, y) puts it. The unknowns are each slot's
(x, y, yaw) and each mark's (x, y).
- **Two passes.** A closed form places the slots one by one: each by a rigid fit onto two marks
  already placed, or onto one mark with its turn as before. One least-squares refinement over
  everything follows.
- **The frame.** The touches fix everything up to one planar rigid motion of the whole layout.
  The marks are centimetres off and cannot fix it. Anchoring on one mark would shift the steel,
  the fences and the drawing area by as much against reality.
  - The solved layout is moved as a whole onto the nominal mountings (`rig.nominal_pose`): the
    mean yaw error over the slots in the solve is zero, and the mean axis lands on the mean
    nominal axis.
  - The turn comes from the yaws, not from a point fit of the axes. Two arms 0.61 m apart,
    each 1–2 cm off, would tilt a point fit by up to 4 deg; each arm's own yaw is off by mrad.
  - When two or more of the touched marks were solved before (`known`), they hold the frame
    instead (subsets). A single known mark cannot hold it: it is solved again, with a note.
- **Kept from before.** z, roll and pitch stay as the rig had them (the plane job). When
  `base_tips` names the tip the plane was measured with, z moves by the hand-z part of (pivot
  tip − that tip). This is exact for the plane job's upright touches, where the hand's z axis
  is the paper's normal. It keeps the new tip and the old height consistent. Without it, a pen
  1.3 mm longer than nominal would draw 1.2 mm too high.
- **Convention** (also in the file): new rotation = Rz(yaw) · rotation before; x and y solved.
  The frame used is written to `marks.json` (`frame`).

**Subsets.** `known` holds the marks solved earlier. A subset (`rows12`) or a single slot
uses them as fixed points. Only the marks this solve placed itself go to `marks.json`, each with
its offset from nominal (information only: a mark may lie anywhere). Known marks and untouched
marks keep their entries.

**Refusals and flags.** All limits are at the top of `marks.py`. The residual limits are set for
cross-hairs placed by eye; dimples, which centre the pen, would allow half.

| rule | value | catches |
|---|---|---|
| input | some touches; one slot alone only with two of its marks known | "no touches"; "2L alone needs marks solved before" (the site's `aris mark 2L` crash, 2026-10-06) |
| pivot touches | at least 3 (six planned) | too few to separate the tip from the mark |
| pivot spread | the pen axes at least 15 deg apart | the tip's length barely seen |
| pivot condition | at least 0.1 | the hand only turned about one axis: tilt, do not only spin |
| pivot residual | 1.5 mm | a touch where the pen slipped off the mark; the reason names it |
| partners | each slot needs two marks pinned by something else (known, or touched by another slot) | otherwise: "<slot> needs a partner: only 1 shared mark" |
| pair distance | two slots measuring the same two marks agree within 2.5 mm | a mark moved between them, or a touch is off; the reason names the pair |
| pose, after the frame fit | within 30 mm and 3 deg of the nominal mounting | "refused: 2R 0.050 m … — wrong slot or wrong robot?" |
| RMS per touch | 1.0 mm | touches that do not fit one rigid layout (an RMS, so below the single-touch limit) |
| rigidity | full rank | a group of slots hanging on the rest by one mark only |

A mark touched by one slot only, and not known, is solved but flagged "determined by one arm":
it constrains nothing.

**Pen part from the pivot.** The tip goes into the slot's pen part with `method: "pivot"` and
`reference_touch` null. A mark is no place for a touch-off, so the next touch-off picks its own
reference point. The protocol runs a touch-off after the mark job, which sets the pen length
against the paper again. The marks base part keeps the plane job's numbers under `plane`.

## The meetings (base x, y, yaw of every arm; 2026-10-08)

The site's FR3s can be hand-guided only in Desk's programming mode, which the person switches
herself. Pairs of arms that share a spot are guided until their pen tips touch in the air, and
the software reads both arms' joints at standstill. The pairs are the 7 that rig.json's marks
define: the three row pairs, at one or two of their spots, and the four column pairs at the
seam spots.
- `aris.server.meetings.calibrate_from_meetings(config_dir, meetings)` takes
  `meetings = [(slot_a, q_a, slot_b, q_b, spot), ...]`.
- When it passes, it writes every solved slot's base part through `files.write_mark_solution`,
  with method "meetings" and `yaw_from` per slot.
- No pen part and no `marks.json` are written. The crosses job stays as a visual check only.

**The model** (`aris/calib/meetings.py`, `solve_meetings`).
- Each tip comes from forward kinematics with the slot's tool as the rig has it: the
  touch-off's tip when the pen part applies, else the nominal one.
- A meeting is one physical point: T_a p_a = T_b p_b. Horizontally,
  Rz(ψ_a) e_a + t̂_a + δ_a − Rz(ψ_b) e_b − t̂_b − δ_b = 0 (2 equations), with e = (R̂ p)_xy.
- The unknowns are x, y (δ) and yaw (ψ) for every slot that appears. They are solved by
  Gauss-Newton with exact rotations (scipy least squares, analytic Jacobian).
- **The frame.** Moving or turning everything together changes no meeting.
  - A reference slot is held exactly at its nominal pose: rig.json `marks.reference_slot`,
    "2L".
  - If it is not among the solved slots, the first solved slot in rig order is held, and the
    result says so.
  - For a row pair this gives the same poses as the earlier pair solve, up to that choice of
    frame.
- **Which yaws are solved** (`yaw_from` per slot: "reference", "meetings" or "nominal"):
  - A slot whose meetings are less than 0.3 m apart keeps its nominal yaw: "1R's yaw is
    nominal: it met at one point only; a second meeting (--yaw) determines it".
  - While the Jacobian is short of full rank, the yaw with the largest share in the
    undetermined direction is held nominal and named. Example: the 7 pairs met once each give
    14 equations for 15 unknowns, so one yaw is held.
- Heights are not solved. How far the two tips disagree in height is reported as a note.

| refusal | limit | message |
|---|---|---|
| a slot that met no one (when `slots` is given) | — | "3L met no other arm" |
| groups not linked | — | "1L/1R are not linked to 2L by any meeting" |
| residual | 2 mm | names the meeting; the tips were not touching, or an arm moved |
| pose | 30 mm, 3 deg from nominal | wrong slot or wrong robot |
| a shift left free | — | the meetings do not hold every slot in place |
| input | (slot_a, q_a, slot_b, q_b, spot), 7 joints each, two different mounted slots | — |

**Measured** (synthetic truth: every slot 0.5–1 cm and 0.1–0.4 deg off, then the layout moved
so that 2L is exactly nominal; meetings 6 cm above each spot; worst of 5 seeds):

| meetings | no noise | 0.3 mrad joint noise: x, y | yaw | seam |
|---|---|---|---|---|
| row pair 2L/2R at A and B | exact (1e-7) | — | — | — |
| 7 pairs + row 2 twice | exact, every yaw solved | 1.74 mm | 3.54 mrad | 0.73 mm |
| 7 pairs + every row twice | exact | 0.94 mm | 1.58 mrad | 0.79 mm |
| 7 pairs once | one yaw held nominal and named; the rest exact when that yaw is nominal | — | — | — |

- The pose errors grow with the distance from the reference 2L, through the short baselines
  (0.36 m) of the end rows' meetings.
- What a drawing shows is the seam between two neighbours. That is the gap between where both
  put the pen at their shared spot, and it stays below 0.8 mm.
- The brief's 0.5 mm / 1 mrad for every arm is not reached at 0.3 mrad of joint noise.

## The paper surface (the height map)

On site, a fixed 2.1 mm press gave no ink in places and reflex trips in others, because the
paper varies by ±1.8 mm. `aris/calib/paper.py` turns the plane jobs' touches into one surface
over the table.

- **In.** Every slot's plane-job touches, as each `base` part already stores them:
  `height_map_table_m` holds x, y and the measured z in the table frame. For a marks base part
  they are taken from its `plane` entry and moved from the pose of that time to the pose the
  marks job found. Failed base parts give nothing.
- **Fit.** One method with one setting: a thin-plate spline with a small smoothing term
  (`SMOOTHING` = 1e-4 m², added to the kernel's diagonal). Why a thin-plate spline:
  - It is the least-bending surface through scattered points and needs no grid.
  - It behaves the same for 8 touches under one arm and 300 under two.
  - It is evaluated with numpy alone from its centres, weights and three affine numbers, so the
    checker reads the same file with its own reader.
  - The smoothing lets the fit miss each touch by 0.01–0.02 mm instead of bending through it,
    and keeps the system solvable when two touches nearly coincide. 1e-3 would already flatten
    30 cm bumps by 0.1 mm.
- **Out.** `Surface.z(x, y)` (vectorised) is the paper height. Inside the touches' convex hull
  it is the spline; outside, the bump fades linearly to the flat paper (z = paper_z) over
  10 cm; beyond that it is flat. With no touches it is flat. `range`, `n_points`, `date` (the
  newest plane job) and `residual_mm` (fit minus touches, RMS).
- **File.** `write_paper(surface, config_dir)` writes `calibration/paper.json`: kind, centres,
  weights, affine, hull, taper, the points, date and residual. It also carries the evaluation
  formula in words (`evaluate`). `surface(config_dir)` reads it back, or gives the flat paper
  when there is no file. The server rebuilds it after every plane job
  (`build_surface` → `write_paper`). The system planner puts the drawing on
  `surface.z(x, y) − press`.

**How dense the touches must be.** A 30 cm bump of ±2 mm (the test surface) needs touches about
every 5 cm to be followed within 0.3 mm. Measured inside the hull of two slots' grids:

| touch spacing | worst error | RMS error |
|---|---|---|
| 10 cm | 0.95 mm | 0.37 mm |
| 7.5 cm | 0.31 mm | 0.14 mm |
| 5 cm | 0.13 mm | 0.03 mm |

Today's plane job (8–12 touches per arm, about 10–20 cm apart) cannot see bumps that
short; it sees the slow ones. A gap between two slots' grids that is wider than half a
wavelength is guessed, not measured.

## What it cannot do

- The plane job cannot see x, y or the turn about the vertical: a flat paper looks the same
  when the arm slides over it. The mark job provides them.
- The mark job's accuracy is limited by the person and the joints, not by the solver; the
  numbers below show it. The weakest link is the seam between rows: two seam marks tie each end
  row to the middle one, so an end row's turn is known several times worse than the middle
  row's.
- The frame is a convention: the solved layout sits where the mountings say on average. A
  frame that has turned or moved as a whole cannot be seen.
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
| 2 mrad | 0.097 deg | 0.22 mm | 0.56 mm | 20 of 20 (7 of 20 under the first limits) |

  At 1 mrad all runs pass, with a largest distance of 0.95 mm.
- On the hardware (2026-10-06): real planes on the wood came out at RMS 0.87–0.92 mm and
  largest 1.69–1.83 mm with 8–12 points.

**Touch-off.**
- **On the hardware (2026-10-06), the first measured number:** the touch-off repeated to
  0.04 mm, with a correction of +1.17 mm.
- A pen 1.3 mm longer than nominal is found to 1e-9 mm without noise. The tip moves along
  the pen axis only.
- An 8 mm error is refused.
- With 0.5 mrad of joint noise, over 10 seeds, the length error is at most 0.31 mm (mean 0.12 mm).

**Mark job.**
- Truth:
  - Every slot is moved 1–2 cm in x and y, 3–8 mrad in yaw, 1 deg in roll and pitch, and
    10 mm in z.
  - Then the whole layout is moved onto the nominal mountings, the solver's frame.
  - The pen is 1.3 mm longer than nominal, and every mark is 2–5 cm off nominal.
- The rig the solver sees is what the plane job left: the true tilt, the height read with the
  nominal pen, and nominal x, y and yaw.
- Touches: pivot orientations are drawn from turns of 30 deg and tilts of ±30 deg. Joint noise
  is 0.3 mrad on every touch.
- "Seam" is the worst distance between where two neighbours put the pen when both aim at the
  same point of their seam: (0, 0) for 2L/2R, the row seams and the seam marks for the others.
  This is what shows in a drawing.
- "Marks" is the worst error of a recovered mark position.

Without noise every case is exact: below 1e-6 mm, and below 1e-6 mrad for yaw.

**Cross-hairs** (the protocol as planned): the guiding error is independent per touch, in a
random direction in the plane. Worst of 5 seeds over the seeds that passed; the last column
counts refused seeds.

| case | guiding per touch | pivot touches | x, y | yaw | tip | z | seam | marks | refused |
|---|---|---|---|---|---|---|---|---|---|
| (a) 2L+2R | 0.3 mm | 6 | 0.22 mm | 0.54 mrad | 0.57 mm | 0.46 mm | 0.62 mm | 0.32 mm | 0 |
| (a) | 0.3 mm | 4 | 0.21 mm | 0.29 mrad | 1.12 mm | 1.00 mm | 0.93 mm | 0.73 mm | 0 |
| (a) | 0.5 mm | 6 | 0.36 mm | 0.56 mrad | 0.64 mm | 0.54 mm | 0.92 mm | 0.43 mm | 0 |
| (a) | 0.5 mm | 4 | 0.27 mm | 0.52 mrad | 1.54 mm | 1.18 mm | 1.20 mm | 0.81 mm | 0 |
| (b) six slots | 0.3 mm | 6 | 1.07 mm | 2.00 mrad | 0.76 mm | 0.75 mm | 1.07 mm | 1.38 mm | 0 |
| (b) | 0.3 mm | 4 | 1.27 mm | 2.24 mrad | 1.23 mm | 1.12 mm | 1.95 mm | 1.44 mm | 1 |
| (b) | 0.5 mm | 6 | 1.21 mm | 3.90 mrad | 1.27 mm | 1.11 mm | 1.20 mm | 2.41 mm | 0 |
| (b) | 0.5 mm | 4 | 1.57 mm | 2.29 mrad | 1.92 mm | 1.80 mm | 1.69 mm | 2.23 mm | 2 |

Findings:
- Six pivot touches beat four. The tip and the seam improve most: in (a) at 0.3 mm, from 1.12
  to 0.57 mm and from 0.93 to 0.62 mm.
- The 4-touch refusals were pivots whose reachable orientations at a mark 2–5 cm off were all
  turns about the vertical, with no tilt (condition 0.000). The planned orientations must
  include tilts.
- In (b) the end rows carry most of the x, y and yaw error. The seams stay near 1–1.2 mm.
- The pen-tip error lands mostly in z through the base-tip correction. The touch-off after the
  mark job sets it against the paper again.

**Dimple model, kept for comparison** (one guiding offset per slot and mark, the pen seated
off the dimple's centre; six-touch pivot; worst of 3 seeds; (c) rows12 and (d) 2R alone, both
after (b) with its marks known):

| case | noise (guiding, joints) | x, y | yaw | tip | z | seam |
|---|---|---|---|---|---|---|
| (a) | 0.3 mm, 0.3 mrad | 0.32 mm | 0.47 mrad | 0.35 mm | 0.33 mm | 0.62 mm |
| (b) | 0.3 mm, 0.3 mrad | 1.03 mm | 1.90 mrad | 1.08 mm | 1.05 mm | 1.21 mm |
| (c) | 0.3 mm, 0.3 mrad | 0.90 mm | 1.90 mrad | 0.72 mm | 0.70 mm | 0.99 mm |
| (d) | 0.3 mm, 0.3 mrad | 0.54 mm | 1.51 mrad | 0.29 mm | 0.19 mm | — |
| (a) | 0.1 mm, 0.05 mrad | 0.10 mm | 0.16 mrad | 0.07 mm | 0.06 mm | 0.19 mm |
| (b) | 0.1 mm, 0.05 mrad | 0.22 mm | 0.53 mrad | 0.14 mm | 0.12 mm | 0.22 mm |
| (c) | 0.1 mm, 0.05 mrad | 0.25 mm | 0.50 mrad | 0.12 mm | 0.12 mm | 0.30 mm |
| (d) | 0.1 mm, 0.05 mrad | 0.19 mm | 0.46 mrad | 0.07 mm | 0.06 mm | — |

**Paper surface.** Two slots' 13 × 13 grids, 5 cm apart, on the ±2 mm / 30 cm surface with
0.05 mm of touch noise: worst error inside the hull 0.25 mm, RMS 0.05 mm, fit residual 0.01 mm.
Beyond the hull plus 10 cm the surface is exactly flat.

Tests: `tests/test_calib.py`, 33 tests. 32 quick ones in about 5 s; the noise table is
`slow` and takes about 10 s.
