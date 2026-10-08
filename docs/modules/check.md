# check — the independent checker

**Job.** Every motion passes through here before it may reach a robot. The checker looks at the
motion exactly as the arm will fly it and says pass or fail, with every number it measured. It
shares no code with the planners, so the two can only agree by both being right. In the old code
every serious bug was found because an independent check disagreed with the planner.

**In.** `check(config_dir, slot, motion, phase, q_before=None, standing=None, surface_z=None)`: the rig's
`config/` folder, which arm by its slot on the frame (`"2R"`; the old robot ids mean nothing
here), one `Motion` (draw, free, lower, lift, touch or retreat), the `Phase` it runs in (who moves, who
stands parked, which walls, all by slot), where the arm is before it starts, and the arms that
stand still away from their park (`{slot: 7 joints}`). `surface_z` (table frame, metres)
replaces the drawing surface (paper less the pen's press) as the height every drawing tip, lower
end and lift start must sit on: the air run (`aris draw --air 30`) flies the whole plan 30 mm
above the paper and passes paper + 0.030. The pen-depth floor moves with it; links, tool, lifted
pen and touches still answer to the real paper.

The checker's own numerical allowances (how finely it judges, never a relaxed limit) are the
`checker` block of rig.json, read into a `Tolerances`; its defaults apply when the block is
missing. Tests may pass `tolerances=Tolerances(...)` instead, the one keyword beyond the contract.

| rig.json `checker` | value | what it catches |
|---|---|---|
| `step_m` | 0.001 | most any capsule point moves between two clearance samples: a contact between samples |
| `clearance_tol_m` | 0.00025 | how far under the truth a reported clearance may lie |
| `rate_tol` | 0.05 | 1 kHz vs 4 kHz readings: a corner in the trajectory |
| `tip_height_tol_m` | 0.0005 | tip off the drawing surface (or the paper end of a lower, lift, touch): a pen that floats or digs |
| `line_tol_m` | 0.0002 | tip off the planned line |
| `back_tol_m` | 0.00001 | numerical allowance on "never backwards" |
| `speed_tol` | 0.03 | tip faster than the pen's speed by more than this share |
| `stop_speed_m_per_s` | 0.00025 | slower than this mid-line is a stop |

**Out.** A `Verdict`: `passed`, the list of measurements (name, value, limit, pass or fail, and
where it happened), `tightest` (the measurement that uses most of its allowance, or the worst
failing one), and `min_clearance`, the smallest clearance beyond the demanded one over every
obstacle, in metres, and `notes`: how the rig was read, i.e. every part that fell back to its
nominal value and why (a slot without a passing calibration, a pen part for another pen, a pen
without a speed). `print(verdict)` gives a short table with the notes under it. Bad input (an empty motion, NaN, an
arm that does not move in this phase, an unreadable rig) is a failed verdict with the reason,
never an exception.

## What is checked

"The motion as it will be flown" means: between two samples the joints follow the cubic that
matches position and velocity at both. Nothing is judged on the samples alone.

| # | measurement | limit |
|---|---|---|
| 1 | `well formed`, `moves`: at least two samples, times increasing, no NaN; some joint turns | more than 1e-5 rad |
| 2 | `starts at q_before`; `at rest at start`, `at rest at end` | 1e-6 rad; 1e-6 rad/s |
| 3 | `joint positions` on the 4 kHz samples and every clearance sample | inside the limits (1e-7 rad, the driver's tolerance) |
| 3 | `velocity`, `acceleration`, `jerk` at 1 kHz, by finite differences as the driver reads them, with three samples of standing still at both ends | the FR3 limits (2.62/5.26/4.18 rad/s; 10 rad/s²; 5000 rad/s³) |
| 3 | `1 kHz vs 4 kHz`: the same three readings at 4 kHz. A smooth trajectory reads the same; a corner reads higher the finer you look | differ by at most 5 % (readings under a tenth of the limit are not compared) |
| 4 | `clearance steel`: every box of rig.json and every arm's struts, plate and clamp (its own included) | 0.050 m |
| 4 | `clearance paper (links)`: the arm's moving links | 0.020 m (`body_to_paper_m`) |
| 4 | `clearance paper (tool)`: everything bolted to the flange except the pen (gripper, blades, holder, pencil tail) | `tool_to_paper_m`; if rig.json lacks it, `body_to_paper_m`, and the verdict says so |
| 4 | `clearance walls`: the phase's walls that have this arm on one side, and the fences of rig.json (every phase) | `wall_m`, 0.040 m until x and y are calibrated |
| 4 | `clearance parked arms`: every arm parked in the phase, at its park configuration, and every arm in `standing` ({slot: 7 joints}: arms standing still somewhere other than their park, in park and calibrate jobs), at those joints; bases included. The closest is named `parked2R:…` or `standing2R:…`; a slot in both stands where `standing` says | 0.050 m |
| 5 | `clearance self`: the capsule pairs at least four joints apart | 0.020 m |
| 6 | all of 4 and 5 hold between the samples too (below) | |
| 7 | drawing: `tip on paper`: the tip against the **drawing surface**, which lies the current pen's press (`pens.table.<pen>.press_m`; 2.1 mm for the 4H graphite today) below the paper; the planned points lie there, and the press is the same number the planners get from `rig.rules().press` | 0.5 mm |
| 7 | drawing: `tip on line`: distance from the planned tips (`motion.tip_base`) and the line through them, at 1 kHz | 0.2 mm |
| 7 | drawing: `never backwards`: progress along the line (below) never falls back (a numerical allowance) | 0.01 mm |
| 7 | drawing: `never stops`: slowest speed along the line between the moment the pen first reaches a quarter of its top speed and the moment it last drops below it | 0.25 mm/s, whatever the drawing speed (a sharp corner slows the pen to about 1 mm/s at any drawing speed; a real stop reads 1e-7 m/s) |
| 7 | drawing: `tip speed` | the current pen's speed on the paper (`speed_m_per_s`, 15 mm/s; the number the planners read; a pen without one falls back to `drawing.draw_speed_m_per_s`, with a note) + 3 % (the timing step overshoots by up to 2.9 % where it speeds up or slows down) |
| 8 | free: `clearance paper (pen)`: the pen capsule, whose surface ends exactly at the tip, against the real paper | `pen_lifted_to_paper_m` (0.020 m) |
| 8 | lower, lift (setting the pen down, taking it up): `pen depth (lower, lift)`: the pen may reach the drawing surface at one end, where its round end reads up to 1.3 mm below the tip; it may never go 2 mm deeper than the surface. Everything else as for a free motion | 2 mm below the drawing surface: -0.0055 m against the paper today |
| 8 | lower, lift: `tip on surface (lower, lift)`: a lower ends, a lift starts, with the tip on the drawing surface (a lower that stops on the paper itself fails: the pen would not press) | 0.5 mm |
| 9 | `hold: clearance at the end`: the last configuration, standing, against everything | at the demanded clearances |
| 11 | retreat (flown from a pose already inside the arm-to-arm clearance, after an interrupted pen-tip meeting): instead of `clearance parked arms`, one row `retreat approaches <slot>` per other arm (parked, or standing where `standing` says): how far the distance to it ever falls back below the best so far, on samples no more than `step` of capsule travel apart, with the bound between samples. Then `retreat ends clear` (the distance to the nearest other arm at the end, against the arm-to-arm clearance), or `retreat length` when the tip travels at most 0.25 m. A joint that starts closer to a limit than the planners' gate (rig.json `gates.limit_margin_rad`, 0.15 rad), or past the limit itself by at most 0.1 rad (the arm is physically there), is left out of `joint positions` and gets three rows instead: `retreat starts near the limit of joint N` (at most 0.1 rad past), `retreat approaches the limit of joint N` (its distance from the limit never falls back, on the 4 kHz samples) and `retreat clears the limit of joint N` (it ends at the gate or further in). Every other joint stays inside its limits as usual. Everything else (steel, paper, walls, fences, velocity and the rest, self, hold) as for a free move | 1 mm; 0.050 m or 0.25 m; 0.1 rad, 1e-5 rad, 0.15 rad |
| 10 | touch (the calibration's probe for the real paper): the motion is split at its bottom (the sample furthest, in joints, from the first); the descent is checked as a lower and the climb as a lift, against the **paper itself** (no press: the touch looks for the paper, it does not draw). `tip on paper`: the descent ends, the climb starts, with the tip on the paper. The extra depth the arm may go on for in reality is not part of the planned path and is not checked. One verdict: each row from the half where it is tighter, its detail saying which | 0.5 mm; pen depth -0.002 m |

The clearances are the **demanded** ones of rig.json (`clearances`), not the planning allowance
on top. The links, the tool and the lifted pen clear the **real paper**; only the pen's own
rows (drawing, setting down, taking up) know the press. Capsules bolted to the base (link 0) are not checked against obstacles (they hang inside
the mount), only against the arm itself. One more exception is data in rig.json
(`hanger.exempt_links`, today `["link1"]`): an arm's link 1 is not checked against its **own**
struts, plate and clamp, because it only turns about the base axis and a rig test settles that
clearance once over the whole turn of joint 1 (`clearances.link1_to_own_mount_m`, 0.020). Against
every other box, a neighbour's hanger included, link 1 is checked like any link. During a drawing motion the pen is not checked against the
paper.

**Progress along the line.** Between two samples of a drawing motion the flown tip goes from
its position at one sample to its position at the next, and those samples stand for known arc
lengths of the planned line. Where the tip is along that chord gives the arc length in between.
This parameter is exactly the planned arc length at every sample and continuous in between;
the nearest point of the line is not used, because at a sharp corner it jumps between the two
legs and briefly runs backwards on a motion that is fine. `never stops` and `never backwards`
are read on this parameter at 1 kHz.

## Before the next phase: `check_phase_end`

`check_phase_end(config_dir, phase, q_by_slot, standing=None)` takes where every arm stands, by slot (active arms at the
end of their queues; a parked arm left out stands at its park configuration; an active arm left
out fails). Everything stands still, so it checks one configuration per arm: every one of the 15
pairs of arms against each other, whole bodies with their bases, at `arm_to_arm_m` (one row per
pair, `arms 2L and 2R`), and every arm against the steel, the paper (links, tool, pen) and itself
(one row per arm, the tightest of those). No walls: the pairs are measured directly. The
coordinator calls it before it starts the next phase.

`standing` names the arms the phase did not move that stand away from their park (pens are
lifted one arm per phase, and after a stop mid-drawing several arms may stand with pens down).
Such an arm is accepted wherever it stands, with no row of its own: it is where it was, and every
motion of the phase was checked against that pose. It still counts in all 15 pairs. A slot the
phase moves cannot be standing. Measured: 2L's pen lifted (2L to park) with 2R's pen still on the
paper passes with 2R standing; without `standing` it fails `arm 2R: paper (pen)`, and a phase
that leaves the arm it moved at the paper fails `arm 2L: paper (pen)`.

## How the motion between samples is covered

1. The checker picks its own sample times. It never uses the ones it was handed, so the same
   motion handed over at 100 Hz or 4 kHz is checked the same way.
2. On each piece of the cubic, each joint's largest speed is known exactly, so how far each
   joint can turn between two times is bounded. A joint turning by an angle moves a point at most
   by that angle times the point's distance from the joint's axis. That distance is measured
   where the arm actually is, at both ends of the interval, plus what the joints further down the
   chain can change it by in between. Samples are placed so that no capsule point can move more
   than `step` (1 mm) between two of them.
3. Every obstacle class is measured exactly at every sample.
4. Clearance cannot change faster than the capsules move, so on each interval the clearance is at
   least where the two falling lines from both ends cross. That bound is subtracted.
5. Where it could hide something more than `tol` (0.25 mm) below the tightest value found, the
   interval is halved and measured again.

The tightest clearance is therefore reported to within 0.25 mm below the truth, never above it.
The other classes are reported as true lower bounds; one that is far (more than 20 mm) above the
tightest class is only computed to a lower bound, marked "at least". Far pairs are skipped only
when a cheap bound proves they cannot be the closest: a ball around the whole arm over 16
consecutive samples, then a ball per capsule, then the exact distance.

## Independence

- `aris.check` imports `aris.types` and nothing else from the package; a test reads the imports
  of every file and fails otherwise. It does not import Drake either (the tests do).
- Kinematics (`model.py`): written from the FR3 URDF joints (a shift and roll-pitch-yaw per
  joint, then a turn about z). The planners use Denavit-Hartenberg parameters. The tests compare
  both with a Drake `MultibodyPlant` built from the URDF in `assets/system_model`.
- Distances (`geometry.py`): segment to segment by the clamped parametrisation (the planners take
  the smallest of five candidates); segment to box as the smallest of the end points against the
  box and the segment against the box's twelve edges, zero if the segment enters the box (the
  planners walk the piecewise-quadratic distance along the segment). The obstacles stay in the
  table frame, where the steel is axis-aligned (the planners work in each arm's base frame).
- The rig (`config.py`): its own reader of rig.json and of the calibration files. Every
  number the checker takes from the rig, the press and the drawing speed included, comes from
  there, never from a default in `aris.types`, so the checker and the planners read the same
  numbers. The struts, plate and clamp of each slot are placed from the words in rig.json, not
  from `rig.py`.
- Slots: `rig.json slots.list`, one entry per slot (`"slot": "2R"`, axis, height, turn, park
  configuration, `mounted`, `hanger`). A slot whose arm is not mounted keeps its hanger steel and
  nothing else, unless it has none (`"hanger": false`: a floor arm or an empty slot). Parked arms appear in the verdict as `parked2R:link3.0`, hanger boxes as `strut2R_plus_x`,
  `plate2R`, `clamp2R`.
- The calibration file `config/calibration/<slot>.json` has two parts, each applied on its own:
  `base` (the slot's pose) when it passed; `pen` (the measured tip in the hand frame) when it
  passed **and** names the pen that is in (`pens.current`). Otherwise the nominal value, and a
  note in the verdict. A file written for another slot, or a pose that is not rigid, is a broken
  install: every verdict fails "well formed".
- The pen: without a measured tip the model's tip moves along the pen axis by the current pen's
  `tip_length_nominal_m` against the length the model was built for (`fr3.json`
  `tool.pen_length_m`, 0.020); the pen capsule takes the pen's `capsule_radius_m`. A moved pen
  capsule keeps its direction: it lies on the line through the tip along the pen axis and its
  surface ends exactly at the tip (the same rule as the planners' `with_tip`; a test compares
  the capsule ends with the planners' rig on the same calibration files).
- The paper height map (`config/calibration/paper.json`, written by the calibration): a
  thin-plate spline stored as centres, weights and three affine numbers, fading to the plane over
  `taper_m` outside the touches' hull. `paper.py` evaluates it with numpy alone from the formula
  the file states (agrees with the writer to 1e-12 m). Where there is one, the drawing surface is
  the map less the press, read at the tip's x, y: `tip on paper` (draw) and `tip on surface`
  (lower end, lift start) answer to it, and the pen-depth floor of a lower or lift is lowered by
  how far the map's lowest touch lies below the plane. The links, the tool, the lifted pen and a
  touch still answer to the plane. No file (or a "flat" one): the plane. Every draw, lower and
  lift verdict says in its notes which surface was used. `surface_z` (the air run) replaces the
  map too.
- The hanger follows the calibrated axis: the arm is bolted to its plate, so a slot's struts,
  plate and clamp move across the table with the calibrated base position (x and y; the heights
  are the frame's). They are shifted, not turned: the boxes stay axis aligned, and a few
  milliradians of turn move a plate corner well under a millimetre.
- Data is shared, code is not: the capsules, limits, tool and self-collision rule are a copy in
  `aris/check/fr3.json`.

**Keeping the capsule table in step.** When `aris/kernel/fr3.py` or `aris/kernel/tool.py`
changes (a capsule, a radius, a limit, the tip, the pair rule), `test_capsule_table_matches_the_
planners` fails and lists every entry that differs. Edit `aris/check/fr3.json` by hand to match,
checking each number against its source (the mesh fit, the datasheet), and run the tests again.
Which capsules are the tool is `tool.tool_bodies` in the same file; when the planners' `Body`
carries `is_tool`, the test compares it too. The pen capsule's far end is written as `"tip"`: it is always the tip moved back by the radius,
so it follows a calibrated tip.

## Files

`motion.py` (the call), `phase.py` (`check_phase_end`), `sweep.py` (clearance along the motion), `scene.py` (obstacles and their
distances), `model.py` (kinematics), `geometry.py` (distances), `timing.py` (the flown curve and
the driver's readings), `drawing.py` (the pen on the paper), `retreat.py` (the retreat rule), `config.py` (rig reader), `paper.py` (paper height map),
`verdict.py` (the answer), `fr3.json` (the data copy).

## Measured (tests/test_check.py)

| test | result |
|---|---|
| kinematics against Drake (URDF from assets), 1000 configurations, 9 frames | 2e-11 (the URDF writes pi/2 as 1.57079632679) |
| agreement with the planners, 10 000 random configurations per arm, six arms | capsule end points 9e-16 m; steel 4e-16, paper 6e-16, pen 5e-16, walls 6e-16, parked 5e-16, self 6e-16 m. Nothing above 1e-6 m to explain |
| distances against dense sampling, parallel and point cases | never above, within the sampling error |
| pen tip over the 4 687 frames of the old hover run of arm 71 (slot 2R) against the old code | 5e-16 m |
| faults caught, one test each | strut, pen 1 mm into the paper (reads -1.5 mm), wall, parked neighbour, self, corner (jerk 2.2 at 1 kHz, 9.0 at 4 kHz, of the limit), velocity 1 % over (reads 1.010; 0.99 passes), tip off the line by 0.5 mm (reads 0.51), stop halfway, empty motion, motion that does not move, start not at q_before, end not at rest, arm not moving in the phase |
| same motion at 100 Hz, 1 kHz, 4 kHz and as retimed | same verdict; tightest clearance within 0.5 mm |
| a drawn V turning 150 degrees | passes; slowest along the line 1.6 mm/s at the corner |
| the sequencer's word at 80 mm/s, corner turning 130 + 18 degrees (`tests/data/arm_stop_31_word_0.npz`) | the pen really slows there: 1.03 mm/s along the line at 1 kHz, 0.83 mm/s tip speed at 10 kHz; not a stop. It failed the old limit (5 % of 80 mm/s = 4 mm/s) and passes the absolute 0.25 mm/s; a real stop halfway reads 7e-8 m/s |
| drawing 1.5 % over its timing / 4 % over | tip speed 20.33 mm/s passes / 20.83 fails (limit 20.6) |
| measured with a press of 3.5 mm (the tests read the press from rig.json): lower (10 mm above the paper to the drawing surface, 3.5 mm below it) and the same reversed as lift | pass; pen 1.4 mm inside its 2 mm below the surface. As a free motion it fails the pen's clearance; lowered 3 mm below the surface it fails pen depth (1.6 mm too deep); a lower that ends on the paper itself (and the lift that starts there) fails `tip on surface` (3.5 mm) |
| a touch down to the paper and back, press 3.5 mm / the same touch planned to the drawing surface | passes / fails `tip on paper` (3.5 mm) and pen depth |
| air run: a line drawn 30 mm above the paper and the lower onto it, with `surface_z` = paper + 30 mm / without | pass (tip 0.03 mm and 0.00 mm off) / fail their tip rows (33.5 mm) |
| retreat of 2L from tools 44 mm apart (2R standing): straight up 60 mm, then 100 mm away / the same with a 3 mm dip toward 2R / the first as a free move | passes (gap 44 to 140 mm) / fails `retreat approaches 2R` (3.5 mm: the dip plus up to half a millimetre of the bound between samples) / fails `clearance parked arms` (44.1 mm) |
| retreat of 2R with joint 6 0.047 rad past its limit (the site, 2026-10-08), joint 6 turned straight back 0.25 rad / the same as a free move / with a wiggle back 0.05 rad / starting 0.12 rad past | passes (ends 0.203 rad inside) / fails `joint positions` (-0.047) / fails `retreat approaches the limit of joint 6` (0.05 rad) / fails `retreat starts near the limit of joint 6` |
| a line drawn on a 1.5 mm bump of a paper height map (90 touches), with the map / on the plane | passes (tip 0.015 mm off) / fails `tip on paper` by the bump on the line (1.40 mm) |
| a drawn line on the drawing surface (3.5 mm below the paper), checked with the press of 3.5 mm and with a press of 0 | passes (tip 0.01 mm from the surface) / fails `tip on paper` (3.51 mm); the tool and link clearances are the same in both |
| the two calibration parts on slot 2R: base only, pen part for another pen, pen part only (base failed), both | each part applied exactly when it should; the notes name what stayed nominal and why; capsule ends against the planners' rig on the same files 5e-16 m |
| hanger of a calibrated slot (base moved 8 mm, -6 mm) | its struts, plate and clamp move by the same x and y, as in the planners' rig; nobody else's steel moves; link 1 to its own strut 61.9 to 60.4 mm |
| drawing speed from the pen: 15 mm/s; pen set to 10 mm/s; pen without a speed (falls back to `drawing.draw_speed_m_per_s`, 10 mm/s) | limit 15.45 mm/s, passes / 10.30, fails / 10.30, fails, with a note |
| a failing motion, and the same with a sample between every two | same failures; -81.242 and -81.243 mm |
| reported against the truth (planners' kernel on a 20 kHz sampling) | never above, within 0.25 mm (78.35 reported, 78.60 true) |
| good free motions (slots 1L, 2L, 3R) | pass |
| good free motion of 2L with 2R parked / 2R standing at a configuration reaching into it / 2R standing at its own park | passes (156.3 mm) / fails `clearance parked arms` (-28.1 mm, `standing2R:link7.1`) / reads exactly as parked |
| phase end: all arms at park | pass; tightest slot 2L, link 6 against the west seam bar, 128.6 mm (demanded 50) |
| own-hanger exemption, slot 2R at park | without it the closest steel is link1.0, 61.9 mm from its own minus-x strut; with it link2.2, 201.4 mm from the plus-x strut |
| phase end: pair clearance against the planners' kernel, 40 configurations | 1.3e-16 m |
| phase end: slot 2R into parked slot 2L; an active arm missing | caught |
| speed, CPU time, one core, 62 capsules | 6.8 s free motion 191 ms; the same 1.2 m circle, now 81 s at 15 mm/s, 878 ms (at 20 mm/s, 60 s: 635 ms) |

Where the time goes (60 s drawing motion): about 10 000 clearance samples, then the 1 kHz and
4 kHz readings (240 000 samples of the cubic) and the pen at 1 kHz. Drake is not used at run
time, so it is not the bottleneck; numpy's per-array overhead on small arrays is. Faster would
be a compiled version of this module's own distance loop (written separately from the planners'
native code), or a tighter bound on how far a capsule moves.

## What it cannot do, and what it found

- **The tool and the paper.** The tool keeps its own clearance to the paper (`tool_to_paper_m`,
  provisionally 0: it must not touch). With the refitted tool capsules (62 capsules in all,
  2026-09-29) a straight line drawn by slot 2L passes: tool 14.5 mm above the paper with the hand
  square, 8.1 mm at the worst 15 degree lean; links 112.5 mm (a lower bound) and 84.2 mm.
  (The hanging struts follow the technical drawing since 2026-09-30: outer faces 171.75 mm to
  table -x and 222.05 mm to +x of each axis, plate 25.15 mm toward +x, and 30 mm longer than
  the old model, ending 65 mm below the plate.)
- The old hover run of arm 71, now slot 2R (lesson L44) fails here: acceleration 218 rad/s² at 1 kHz on the
  flown curve (the old code read 89 at 48 Hz), jerk and the 1 kHz/4 kHz comparison, and link 6
  passes 8.9 mm above the paper (the old gate measured joint centres, not capsule surfaces).
- The hanger follows a calibrated base by shifting; a turn of the base is not applied to the
  (axis-aligned) boxes.
- Arms standing still are checked where they stand: parked ones at their park configurations
  from rig.json, the others at the joints given in `standing`. Arms moving at the
  same time are kept apart by the walls; the checker does not compare two moving arms.
- A class far above the tightest one is reported as "at least" that value.
- Drawing: "on the line" is judged against the planned tips as a polyline, locally (the planned
  points around the current sample), so a line that crosses itself is fine.
