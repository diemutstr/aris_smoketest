# kernel.arm — the arm model

**Job.** Everything the planners need to know about one Franka FR3 carrying the pen holder:
where the hand and the pen tip are for given joint angles, which joint angles put the hand at a
given pose, the arm's joint limits, and the capsules that make up its collision body. All six
arms are identical, so one `Arm` object serves all of them. It works in the arm's own base
frame and knows nothing about the table, the paper or the other arms.

Files: `aris/kernel/arm.py` (the `Arm` class), `aris/kernel/fr3.py` (the robot's numbers),
`aris/kernel/tool.py` (the pen holder, `default_tool()`).

## Calls

| call | in | out |
|---|---|---|
| `Arm(tool)` | a `Tool` (use `default_tool()`) | the arm |
| `limits` | | joint position, velocity, acceleration, jerk limits |
| `fk(Q)` | joints (N,7) | hand pose `T_base_hand` (N,4,4) |
| `tip(Q)`, `pen_axis(Q)` | joints | pen tip (N,3); unit vector along the pen, out of the tip (N,3) |
| `link_frames(Q)` | joints | all ten frames (N,10,4,4) |
| `body(Q)` | joints | `Body`: 40 capsules per configuration |
| `self_pairs`, `self_margin` | | which capsule pairs to check against each other (366 pairs), and 0.023 m |
| `ik(T_base_hand, q7)` | poses (M,4,4), joint-7 angle (M,) | joints (M,4,7) and valid (M,4) |
| `hand_pose(tip, normal, spin, lean)` | tips (M,3), paper normal (3,), spin (M,), lean (M,2) | hand poses (M,4,4) |
| `sigma_min(Q)`, `tip_jacobian(Q)` | joints | smallest singular value of the tip Jacobian (N,); the Jacobian (N,3,7) |
| `limit_margin(Q)` | joints | radians to the nearest joint limit (N,) |

## Frames

The base frame is the robot's own link0. Frames 1 to 7 are the joints' link frames (the
Franka URDF frames, standard modified Denavit-Hartenberg). The flange is 0.107 m past link7,
and the **hand frame** is the flange turned -45 degrees about its axis, the way the stock
Franka hand is mounted. Every `T_base_hand` in the new code is this frame. The old code often
used the "hand TCP", 0.1034 m further along the hand's z axis; the IK solver still works in
that point internally, and `ik` converts.

## The tool

The pen holder is clamped at the far end of the Fat Franka Finger blades. In the hand frame
the pen tip is at (0.0860369, 0, 0.1494262) m and the pen leans 23 degrees from the hand's z
axis toward +x. These are the old code's numbers for the lateral holder, copied digit for digit;
the forward kinematics of the tip agrees with the old code to 4e-16 m.

`hand_pose` puts the tip on a point. With no lean, the hand's z axis points straight into the
paper; `spin` turns the hand about the paper normal, starting from the base x axis laid into the
paper plane. `lean = (tx, ty)` then tilts the hand by that rotation vector written in the hand's
own x-y plane, which lies in the paper plane, by |lean| radians. This is exactly the old
lateral-holder convention (checked to 2e-16 on 2000 random poses). Note that the pen itself is
always 23 degrees off the hand axis, so "no lean" means the hand is square to the paper, not the
pen.

## The collision body

40 capsules. The seven link0 capsules are **fixed** (`is_fixed`): they do not move with the
joints, and the collision check does not test them against obstacles (they sit inside the
arm's own mount). All others move.

| body | capsules | radii (mm) | where the numbers come from |
|---|---|---|---|
| link0 (base) | link0.0 - link0.6, **fixed** | 177, 176, 171, 160, 112, 76, 78 | old `selfcoll.py` table: seven slices about the base axis; the lowest holds the cable connector stub behind the mounting face |
| link1 - link7 | three each, link1.0 - link7.2 | 63 68 76 / 63 69 75 / 62 75 59 / 62 77 64 / 63 67 61 / 51 56 49 / 47 44 38 | old `selfcoll.py` table, fitted to each link's own metal |
| hand | hand.0 - hand.2 | 39, 36, 31 | new fit |
| Fat finger blades | finger_left.0-1, finger_right.0-1 | 24, 16 / 16, 24 | new fit |
| pen holder | holder.0 - holder.2 | 16, 21, 25 | new fit on the housing and cap meshes |
| pencil tail | pen_tail | 5 | the 72.5 mm of pencil behind the holder, as the old model draws it (never measured) |
| pen | pen (the only `is_pen` capsule) | 5 | the 20 mm of graphite past the cap; the capsule's surface ends exactly at the tip |

Why this model. The old code had four: fat sausages about the lines between joint origins
(`coordination.py`, radii 90 to 130 mm, up to 131 mm of air), vendor sphere sets (shown by the
2026-09 audit not to contain the arm, worst 235 mm outside on link0), the base column bands, and
the self-collision table in `selfcoll.py`, which fits three capsules to each link's own metal
with every radius the exact largest distance of any mesh vertex, rounded up to the millimetre.
The last is the tightest one that contains the metal, so it is used for links 0 to 7 as is. Its
hand capsules were fitted to the stock fingers, but the rig has Fat finger blades reaching 80 mm
further, and its tool was a 50 mm "L" that the 2026-09-03 holder correction showed no longer
contains the holder. So the hand, the blades and the holder were refitted with the same method
(`tests/oracle/fit_arm_capsules.py`) on the meshes of the real build in
`assets/system_model/meshes`.

Self pairs: two bodies are checked against each other only if they are at least four joints
apart (the hand, blades and tool count as one body at the end of the chain). Closer bodies meet
at a joint and their capsules overlap in every configuration; the old code used the same rule.

Not modelled: the estimated cable service loops on the forearm and wrist (visual-only guesses in
the old URDF).

## Limits

Positions: FR3 datasheet (old `frames.py`). Velocities 2.62 (joints 1-4), 5.26, 4.18, 5.26 rad/s:
FR3 URDF, confirmed by libfranka's rate limiter. Acceleration 10 rad/s^2 and jerk 5000 rad/s^3
on every joint: libfranka `rate_limiting.h`; 10 rad/s^2 is also the driver's gate.

## Inverse kinematics

The vendored analytic solver (`third_party/franka_analytical_ik`, He et al.) returns four
branches per pose and joint-7 angle. Slot b is always the solver's branch b. Each answer is
kept only if it is inside the FR3 limits and our own forward kinematics puts the hand on the
requested pose to 1e-9 m; the solver clamps at the edge of the workspace and returns answers
that miss by centimetres, and those are rejected.

**What it cannot do.** The solver has the Panda's limits built in and computes only one of the
two elbow angles, so it never returns:
- joint 6 above 3.7525 rad (the FR3 allows up to 4.5169): a fifth of joint 6's range
- joint 7 beyond ±2.8973, joint 2 beyond ±1.7628, joint 3 beyond ±2.8973 (FR3: ±3.0159, ±1.7837, ±2.9007)
- an almost straight elbow, joint 4 above -0.467 rad (the second root)
- joint 2 within about 0.045 rad of zero (it treats that as the shoulder singularity)

Measured: of random configurations inside the FR3 limits, only 66 % are found again from their
own hand pose, and 21 % of those poses get no answer at all. Of configurations that hold a pen
on a paper 0.97 m below the base (hand within 15 degrees of vertical, 0.15 rad from every limit),
58 % are found again and 26 % of the poses get no answer; nine in ten of the misses have joint 6
above 3.7525. The old planner had the same blind spots. Fixing them means changing the solver.

About 0.1 % of genuine answers, near singularities, reproduce the pose only to between 1e-9 and
1e-6 and are dropped by the 1e-9 check (the old code did the same); clamped answers miss by
1e-4 or more.

## Measured (tests/test_kernel_arm.py)

| test | result |
|---|---|
| hand, tip, all link frames vs old code, 10 000 random configurations | worst 4.4e-16 m |
| IK on 10 000 reachable poses | 19 865 solutions, same count per pose as the old code, same solutions in the same order; worst pose error 9.9e-10 m; all inside the limits |
| IK on 10 000 random poses in a 2 m cube | 971 solutions, count per pose equal to the old code |
| `hand_pose` -> `ik` -> `tip`, lean up to 15 degrees | worst tip error 1.4e-11 m over 9 062 solutions |
| `sigma_min`, `limit_margin` vs old code | 4.7e-16, exact |
| mesh vertices outside their capsules (worst, mm; negative is inside) | link0 -0.14, link1 -0.88, link2 -0.34, link3 -0.35, link4 -3.31, link5 -0.00, link6 -0.14, link7 -1.42, hand -0.73, blades -0.22 / -0.44, holder -0.09, pen 0.00 (the tip, by design), pencil tail -1.50 |
| self pairs | 366; none is overlapping in every configuration; 86 % of random configurations clear 23 mm |
| speed, one core, batch 10 000 | fk 1.06 M/s, tip 1.13 M/s, body 0.32 M/s, ik 0.21 M poses/s, sigma_min 0.31 M/s |

Regenerate the reference data with
`ARIS_RIG=proposed ARIS_TOOL=lateral ../.venv/bin/python tests/oracle/make_arm_reference.py`.
The containment test reads the meshes directly and needs `trimesh` (present in the venv).
