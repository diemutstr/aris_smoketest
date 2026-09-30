# Rebuild plan: fast modular kernel, calibration, fleet execution

Status: PROPOSAL for audit, 2026-09-29. Nothing here is implemented. Written by Claude from the
conversation with Pete on 2026-09-29; sections marked "Pete" are his requirements in his words or
close to them. Claims marked (unverified) have not been checked against code or hardware.

## 1. Requirements (Pete)

1. Planning must be fast. An RRT plan for a single arm should take 0.1 s tops. "We are never
   planning in the full space of the system."
2. The arms should be almost moving immediately after a drawing is submitted (time to first
   motion in seconds, not minutes).
3. The drawing must be efficient: long continuous strokes, few lifts, few reconfigurations.
4. An absolutely bulletproof calibration routine: the arms are not mounted perfectly (8020 allows
   a small tilt), the pen geometry was eyeballed and changes as the graphite wears. Today arms
   sometimes run into the table, or hover and do not draw, and stitched drawings do not align.
5. Nothing may be drawing-specific. The planner must work for any drawing out of the box. The
   acceptance test is a corpus of drawings, never one picture.
6. Robust, clean, modular code. Mono repo even though we deploy on several machines.
7. A clean GUI with few features: a clear visualisation of what is going on.
8. Planning strategy (Pete): leader arms that are almost guaranteed never to intersect in
   workspace are planned independently and start moving at once (the 1-2-1 pattern: in a 2x3 grid
   pick alternating columns per row). The opposite arms in each column (2-1-2) then draw what they
   can. Then reverse the roles. Possibly virtual walls so the two groups can draw fully
   independently (how is open). Whatever is left is filled in at the end by independent single-arm
   plans. For a single arm: the local planner produces a joint path per stroke, a cheap heuristic
   (flood fill from one side, greedy, or similar) sequences the strokes, RRT plans the transits
   between them. Joint multi-arm planning is almost never needed; interleaving arms comes later.

## 2. Where we are

| item | value |
|---|---|
| planner package `aris_sixarm/` | 47 800 lines, 41 modules; staged.py 5 350, allocate.py 4 870 |
| `scripts/` | 42 100 lines, 90 files |
| tests | 24 500 lines |
| single-arm word "unknown" plan (0.55 m wide) | 20 to 51 s wall |
| six-arm CSAIL logo (staged planner, 99.988 % flown, certified) | 1 406 s planning, 403 s drawing |
| time to first motion (logo) | 0.19 s in the schedule, but only after the full 1 406 s plan |
| bench corpus (staged planner) | scatter 99 %, starburst 97 %, duotone 97 %, hatch 83 %, spiral 84 % certified |

Why it is slow (diagnosis, not yet profiled): every collision check is Python over capsule pairs
one configuration at a time, and every layer above it multiplies that count: the IK yaw sweep, the
ladder search over the stroke, the RRT, the certification pass. There is no compiled kernel and no
batching. The staged planner also grew through a long series of feature-driven fixes (exact rooms,
held-pose barriers, partner standoff, pen-up fixes) that each added a layer.

Facts that carry over unchanged: mount layout (x = +-305 mm, y = 0 / +-1210.2 mm from the table
centre; h = 0.970 m paper to plate underside; table 2188 x 4165.6 mm; canvas 1803.4 x 3630.6 mm),
tool tip (0.0860, 0, 0.1494) m in the hand frame (photo-derived, eyeballed), gate constants
(PAIR_MARGIN 50 mm, SELF 23, STATIC 50, FRAME_FLOOR 63, MARGIN_GATE 0.15), the pathway CSV contract
of the executor, the Hershey text assets, the drawings under docs/drawings, the test corpus.

## 3. Decision: rebuild, do not clean in place

The old package becomes `legacy/`, frozen, importable as a reference oracle for tests. A new
package is built around one compiled batch collision kernel. Everything above the kernel is small
because the kernel makes checks cheap. Legacy is deleted when the corpus passes on the new core.

## 4. Repository structure (mono repo, one distribution, extras per machine role)

```
aris/
  pyproject.toml          one package "aris"; extras: [robot] [gui] [dev]
  config/
    site.json             arm ids, addresses, DDS domains, nominal mounts
    calibration/          per-arm base pose + tool tip, dated, written only by calib
    gates.json            margins and limits
  aris/
    core/                 pure numpy, no ROS, no GUI
      world/              table, frame, mounts, tool, pen; loads config + calibration
      kin/                batch FK, IK, Jacobians (vendored analytical IK)
      collide/            compiled kernel + benchmark
      plan/               stroke tracking, transits, ordering, fleet schedule
      certify/            gates, pairwise clearance under a timing tolerance
      planfile/           THE contract: timed joint plans + strokes + hashes
    calib/                touch grid, fiducials, pivot; fits; writes config/calibration
    robot/                the only ROS code
      arm/                the four verbs, one process per arm
      fleet/              fan-out with common start, barriers, reporting
      controllers/        colcon workspace: impedance controller, franka_ros2 patches
      launch/             per-arm launch driven by site.json
    gui/                  FastAPI + web; imports core; reaches robot via one bridge
    cli.py                the one front door: aris plan | certify | calib | run | site
  third_party/            pinned vendored deps
  tests/
    core/                 no ROS needed
    corpus/               random drawings, no-drop invariant
    sil/                  loopback controller sim, exercises robot/arm and fleet
  docs/
  legacy/                 today's aris_sixarm + scripts, frozen
```

Rules:
- Imports flow one way: config, then core, then calib / robot / gui side by side, then cli. A
  lint test fails if core imports anything above it or if any file outside config contains an
  address.
- The plan file is the only artefact that crosses machines. It records the git hash and the
  calibration hash it was made with; the operator refuses a plan whose hashes do not match.
- Roles by install extras, not by code. Dev machine: core, calib, gui, dev. Operator: core,
  robot, plus the colcon build. Same commit on both.
- One CLI. No loose scripts.
- Config is data. Site facts, calibration and gates live in config and are never inlined.
- Tests run without hardware. Robot tests run against the loopback simulation.

Machines: dev machine (no ROS) plans, certifies, fits calibration, serves the GUI. Operator PC
(ROS 2 Jazzy, RT kernel, 192.168.50.0/24) runs one ros2_control process per arm, the arm clients,
the fleet coordinator, executes plan files and returns calibration touch data.

## 5. Hardware interface

Bottom-up on the operator PC (verified 2026-09-16/29 from the vendored franka_ros2 and the
operator patches; see aris-control-stack-briefing, hardware-day1 notes):

1. One ros2_control process per arm, namespaced `arm_<id>`, DDS domain == arm id. The operator
   patch to moveit.launch.py already builds the inverted-mount robot model with mount tilt as
   launch arguments (mount_to_world, mroll, mpitch, myaw, mz; no x/y).
2. Controllers: `fr3_arm_controller` (joint_trajectory_controller, position) and the live
   `cartesian_impedance_controller` (effort; NOT in git, 378 lines on the operator). Only one owns
   the joints at a time; switching is one controller_manager service call.
3. The move interface is `control_msgs/action/FollowJointTrajectory` on the trajectory
   controller: joint positions with time stamps, optional velocities. Every position move ends up
   as that message, MoveIt or not. (unverified) The trajectory header stamp is honoured as a start
   time by the JTC, so goals to several arms with the same future stamp start together within a
   few ms on one PC.
4. MoveIt is an optional client on top and is NOT used as the planner. It has no force/contact
   notion. It stays useful for RViz and the robot description only.
5. Drawing goes through the impedance controller. TODAY its executor (`rtff_pathway_exec.py`)
   paces by arc length at 0.02 m/s and IGNORES q1..q7 and t_s, latching its own nullspace. So two
   arms drawing concurrently cannot yet be certified against each other. Stage 1 of the controller
   redesign (joint-reference subscriber; executor publishes pose + q at the same s) is a
   prerequisite for concurrent drawing. Transits in position mode are fine today.

The robot layer exposes four verbs per arm, each a small rclpy client with no planning inside:
move (timed joint trajectory to the position controller, with start time), draw (a stroke to the
impedance controller; today the pathway CSV, later the joint-referenced format), switch
(controller ownership), state (joint angles, estimated external wrench). One client process per
arm, a fleet coordinator above that fans out goals with a common start stamp, waits, reports.
Certification accepts a timing tolerance (each arm may be early or late by a bounded amount) and
uses hold poses as barriers where a hard guarantee is wanted. Adding arms changes the fleet list
only.

## 6. Kernel and planner

- Collision: one compiled batch checker for one arm against the static world (table, frame,
  struts, seam bars) plus the other arms as frozen or swept obstacles. Candidate: VAMP (the
  franka-station-sim repo has a working FR3 example; fr3drivers carries a `vamp_bimanual` binary).
  Alternatives: cuRobo (GPU) or a small numba/C++ capsule kernel. Benchmark in configurations per
  second; target: single-arm RRT <= 0.1 s, stroke tracking ~1 ms per stroke.
- Kinematics: batch FK/Jacobian; vendored analytical IK (third_party/franka_analytical_ik) with
  yaw/q7 sweep as today.
- Stroke tracking (the local planner): redundancy resolution along a Cartesian stroke with the
  tip on the paper, tilt allowed, maximin manipulability, joint velocity limits at a fixed
  fraction of QD_MAX (30 % today). One stroke = one continuous joint path; a stroke the arm cannot
  finish is split at the cell boundary, not abandoned.
- Sequencing per arm: cheap heuristic (greedy nearest / flood fill from one side), RRT transits
  between strokes, hover height for pen-up moves.
- Fleet strategy (Pete, section 1 item 8): leaders (1-2-1) planned independently and dispatched at
  once; followers (2-1-2) draw what is safe; roles reversed; virtual walls if needed; gap fill by
  independent single-arm plans at the end. Time to first motion = time to plan the first stroke and
  transit of each leader, so planning must be incremental: the plan file / channel must allow
  chunks to be appended while earlier chunks execute.
- Drawing-agnostic input: a list of polylines with ink id in the table frame (mm). Downstream code
  never sees "the drawing". CI runs a corpus of random and structured drawings and checks the
  no-drop invariant (every stroke either drawn once or listed with a reason).

## 7. Calibration

Unknowns per arm: base pose in the table frame (6 DoF; tilt from the 8020 mount matters most: 1
degree at 0.97 m is 17 mm at the paper), tool tip in the hand frame (3 DoF, eyeballed today,
graphite wears), paper plane (height and tilt, possibly per location).

Procedure (proposal): the arm is the probe, under force/position control with the existing
position-lag contact detector.
1. Touch the paper at ~9 points spread over the arm's cell: plane fit gives base tilt (2) and
   height (1) relative to the paper; residual reports paper flatness.
2. Touch 3 fiducials: a small jig with a cone the pen tip drops into, placed at tape-measured
   table positions; gives x, y, yaw (3). Every arm registers to the same fiducials, which is what
   makes stitching consistent.
3. Tool tip: pivot into the cone from ~5 hand orientations, solve the tip offset (3) once; before
   every job a touch-off on a reference block updates the current pen length.
4. Longer term the controller finds the paper by contact (contact descent exists), so the pen
   length only needs to be roughly right.
Bulletproof means: one command per arm, minutes, a printed residual with a reject threshold, one
file written (config/calibration), a drawn test pattern that verifies the result, and detection of
the arm's own kinematic error (a 9 mrad joint-5 offset was seen on 2026-09-10).

## 8. GUI

Few features: load a drawing, show the plan (what each arm draws, in what order, what is left and
why), show live arm state during execution, run the calibration routine and show its residuals.
One bridge to the operator. No planning logic in the GUI.

## 9. Migration and milestones

1. Skeleton: package, config, lint rules, CLI, legacy frozen. Kernel with benchmark.
2. Calibration procedure end to end on one arm (data collection through the four verbs, fit,
   verification pattern).
3. Single-arm planner on the corpus: stroke tracking, sequencing, transits, certify, planfile.
4. Fleet: leaders/followers, incremental dispatch, timing tolerance, barriers; SIL first.
5. GUI. Then delete legacy.

## 10. Open questions

- Which collision backend; whether VAMP's FR3 model, attachments (pen) and obstacle primitives
  cover our needs, and whether it can express swept obstacles for time-indexed arms.
- How to define the virtual walls for the leader/follower groups so each group is provably
  independent; how much of the canvas each group can then cover.
- Streaming plans versus "the plan file is the only artefact": incremental dispatch needs a live
  channel or an append-only file the operator tails.
- Whether the base pose and tip offset are separately identifiable from touches alone, and how
  many touches are needed for a given accuracy.
- How the stage-1 controller redesign (joint reference) is sequenced relative to this rebuild.
