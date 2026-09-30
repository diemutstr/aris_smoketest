# check — the independent checker

**Job.** Every motion passes through here before it may reach a robot. The checker looks at the
motion exactly as the arm will fly it and says pass or fail, with every number it measured. It
shares no code with the planners, so the two can only agree by both being right. In the old code
every serious bug was found because an independent check disagreed with the planner.

**In.** `check(config_dir, arm_id, motion, phase, q_before=None)`: the rig's `config/` folder,
which arm, one `Motion` (drawing or free), the `Phase` it runs in (who moves, who stands parked,
which walls), and where the arm is before it starts. Optional keywords set the tolerances in the
table below.

**Out.** A `Verdict`: `passed`, the list of measurements (name, value, limit, pass or fail, and
where it happened), `tightest` (the measurement that uses most of its allowance, or the worst
failing one), and `min_clearance`, the smallest clearance beyond the demanded one over every
obstacle, in metres. `print(verdict)` gives a short table. Bad input (an empty motion, NaN, an
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
| 4 | `clearance walls`: the phase's walls that have this arm on one side | 0.025 m |
| 4 | `clearance parked arms`: every arm parked in the phase, at its park configuration, base included | 0.050 m |
| 5 | `clearance self`: the capsule pairs at least four joints apart | 0.020 m |
| 6 | all of 4 and 5 hold between the samples too (below) | |
| 7 | drawing: `tip on paper` (before the controller presses) | 0.5 mm |
| 7 | drawing: `tip on line`: distance from the planned tips (`motion.tip_base`) and the line through them, at 1 kHz | 0.2 mm |
| 7 | drawing: `never backwards` along the line (a numerical allowance) | 0.01 mm |
| 7 | drawing: `never stops`: slowest speed along the line between the moment the pen first reaches a quarter of its top speed and the moment it last drops below it | 5 % of the drawing speed (1 mm/s) |
| 7 | drawing: `tip speed` | drawing speed + 2 % |
| 8 | free: `clearance paper (pen)`: the pen capsule, whose surface ends exactly at the tip | 0.003 m (`pen_lifted_to_paper_m`) |
| 9 | `hold: clearance at the end`: the last configuration, standing, against everything | at the demanded clearances |

The clearances are the **demanded** ones of rig.json (`clearances`), not the planning allowance
on top. Capsules bolted to the base (link 0) are not checked against obstacles (they hang inside
the mount), only against the arm itself. One more exception is data in rig.json
(`hanger.exempt_links`, today `["link1"]`): an arm's link 1 is not checked against its **own**
struts, plate and clamp, because it only turns about the base axis and a rig test settles that
clearance once over the whole turn of joint 1 (`clearances.link1_to_own_mount_m`, 0.020). Against
every other box, a neighbour's hanger included, link 1 is checked like any link. During a drawing motion the pen is not checked against the
paper.

## Before the next phase: `check_phase_end`

`check_phase_end(config_dir, phase, q_by_arm)` takes where every arm stands (active arms at the
end of their queues; a parked arm left out stands at its park configuration; an active arm left
out fails). Everything stands still, so it checks one configuration per arm: every one of the 15
pairs of arms against each other, whole bodies with their bases, at `arm_to_arm_m` (one row per
pair, `arms 31 and 71`), and every arm against the steel, the paper (links, tool, pen) and itself
(one row per arm, the tightest of those). No walls: the pairs are measured directly. The
coordinator calls it before it starts the next phase.

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
- The rig (`config.py`): its own reader of rig.json and of passing calibration files. The
  struts, plate and clamp of each arm are placed from the words in rig.json, not from `rig.py`.
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
the driver's readings), `drawing.py` (the pen on the paper), `config.py` (rig reader),
`verdict.py` (the answer), `fr3.json` (the data copy).

## Measured (tests/test_check.py)

| test | result |
|---|---|
| kinematics against Drake (URDF from assets), 1000 configurations, 9 frames | 2e-11 (the URDF writes pi/2 as 1.57079632679) |
| agreement with the planners, 10 000 random configurations per arm, six arms | capsule end points 9e-16 m; steel 4e-16, paper 6e-16, pen 5e-16, walls 6e-16, parked 5e-16, self 6e-16 m. Nothing above 1e-6 m to explain |
| distances against dense sampling, parallel and point cases | never above, within the sampling error |
| pen tip over the 4 687 frames of the old hover run of arm 71 against the old code | 5e-16 m |
| faults caught, one test each | strut, pen 1 mm into the paper (reads -1.5 mm), wall, parked neighbour, self, corner (jerk 2.2 at 1 kHz, 9.0 at 4 kHz, of the limit), velocity 1 % over (reads 1.010; 0.99 passes), tip off the line by 0.5 mm (reads 0.51), stop halfway, empty motion, motion that does not move, start not at q_before, end not at rest, arm not moving in the phase |
| same motion at 100 Hz, 1 kHz, 4 kHz and as retimed | same verdict; tightest clearance within 0.5 mm |
| a failing motion, and the same with a sample between every two | same failures; -81.242 and -81.243 mm |
| reported against the truth (planners' kernel on a 20 kHz sampling) | never above, within 0.25 mm (78.35 reported, 78.60 true) |
| good free motions (arms 13, 31, 97) | pass |
| phase end: all arms at park | pass; tightest arm 31, link 6 against the west seam bar, 128.6 mm (demanded 50) |
| own-hanger exemption, arm 71 at park | without it link1.0 reads 39.5 mm from its own strut (under 50); with it the tightest steel is link2.2, 184.7 mm |
| phase end: pair clearance against the planners' kernel, 40 configurations | 1.3e-16 m |
| phase end: arm 71 into parked arm 31; an active arm missing | caught |
| speed, CPU time, one core, 62 capsules | 6.8 s free motion 200 ms; 60 s drawing motion 635 ms (with the first 40-capsule model: 145 and 650 ms) |

Where the time goes (60 s drawing motion): about 10 000 clearance samples, then the 1 kHz and
4 kHz readings (240 000 samples of the cubic) and the pen at 1 kHz. Drake is not used at run
time, so it is not the bottleneck; numpy's per-array overhead on small arrays is. Faster would
be a compiled version of this module's own distance loop (written separately from the planners'
native code), or a tighter bound on how far a capsule moves.

## What it cannot do, and what it found

- **The tool and the paper.** The tool keeps its own clearance to the paper (`tool_to_paper_m`,
  provisionally 0: it must not touch). With the refitted tool capsules (62 capsules in all,
  2026-09-29) a straight line drawn by arm 31 passes: tool 14.5 mm above the paper with the hand
  square, 8.1 mm at the worst 15 degree lean; links 112.5 mm (a lower bound) and 84.2 mm.
  (The hanging struts are 30 mm longer since 2026-09-30; they end 65 mm below the plate.)
- The old hover run of arm 71 (lesson L44) fails here: acceleration 218 rad/s² at 1 kHz on the
  flown curve (the old code read 89 at 48 Hz), jerk and the 1 kHz/4 kHz comparison, and link 6
  passes 8.9 mm above the paper (the old gate measured joint centres, not capsule surfaces).
- Parked arms are checked at their park configurations from rig.json only. Arms moving at the
  same time are kept apart by the walls; the checker does not compare two moving arms.
- A class far above the tightest one is reported as "at least" that value.
- Drawing: "on the line" is judged against the planned tips as a polyline, locally (the planned
  points around the current sample), so a line that crosses itself is fine.
