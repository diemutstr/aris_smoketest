# kernel.arm — the arm model

**Job.** Everything the planners need to know about one Franka FR3 carrying the pen holder:
where the hand and the pen tip are for given joint angles, which joint angles put the hand at a
given pose, the arm's joint limits, and the capsules that make up its collision body. All six
arms are identical, so one `Arm` object serves all of them. It works in the arm's own base
frame and knows nothing about the table, the paper or the other arms.

Files: `aris/kernel/arm.py` (the `Arm` class), `aris/kernel/fr3.py` (the robot's numbers),
`aris/kernel/tool.py` (the pen holder, `default_tool()`), and the IK solver
`native/fr3_ik/` (C++, installed with `../.venv/bin/pip install ./native/fr3_ik`).

## Calls

| call | in | out |
|---|---|---|
| `Arm(tool)` | a `Tool` (use `default_tool()`) | the arm |
| `limits` | | joint position, velocity, acceleration, jerk limits |
| `fk(Q)` | joints (N,7) | hand pose `T_base_hand` (N,4,4) |
| `tip(Q)`, `pen_axis(Q)` | joints | pen tip (N,3); unit vector along the pen, out of the tip (N,3) |
| `link_frames(Q)` | joints | all ten frames (N,10,4,4) |
| `body(Q)` | joints | `Body`: 62 capsules per configuration, with `is_pen`, `is_fixed`, `is_tool` |
| `self_pairs` | | which capsule pairs to check against each other (784 pairs); the margin is the rig's (`Gates.self_margin`) |
| `reach` | | (7,62): how far each capsule can be from each joint's axis, for bounding motion between samples |
| `capsule_table()` | | the 62 capsules as data: frame index, two ends in that frame, radius, is_pen, is_fixed, is_tool, name |
| `tool.with_tip(tool, tip_hand)` | a tool, a calibrated tip in the hand frame | the same tool with the tip there; the pen capsule keeps its direction and radius and ends exactly at the new tip |
| `chain_table()` | | the chain as data: the 7 DH rows, then the fixed flange and hand frames (parent, rotation, translation); with `capsule_table()` enough to rebuild `body` exactly (tested to 1e-12) |
| `ik(T_base_hand, q7, with_flags=False)` | poses (M,4,4), joint-7 angle (M,) | joints (M,8,7), valid (M,8), and flags if asked |
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

62 capsules. The seven link0 capsules are **fixed** (`is_fixed`): they do not move with the
joints, and the collision check does not test them against obstacles (they sit inside the
arm's own mount). All others move, link1 included, and are checked per pose. The 33 capsules
of the hand, blades, holder and pencil tail are marked `is_tool` (bolted to the flange); the pen is `is_pen` only.

| body | capsules | radii (mm) | where the numbers come from |
|---|---|---|---|
| link0 (base) | link0.0 - link0.6, **fixed** | 177, 176, 171, 160, 112, 76, 78 | old `selfcoll.py` table: seven slices about the base axis; the lowest holds the cable connector stub behind the mounting face |
| link1 - link7 | three each, link1.0 - link7.2 | 63 68 76 / 63 69 75 / 62 75 59 / 62 77 64 / 63 67 61 / 51 56 49 / 47 44 38 | old `selfcoll.py` table, fitted to each link's own metal |
| hand (gripper body) | hand.0 - hand.13 | 26-31 above, 3-6.5 along the bottom edges | new fit, paper-facing (below) |
| Fat finger blades | finger_left.0-3, finger_right.0-3 | 25, and three of 2 along the edges | new fit, paper-facing |
| pen holder | holder.0 - holder.8 | 25, 25, and seven of 2 | new fit on the housing and cap meshes, paper-facing |
| pencil tail | pen_tail.0-1 | 4.5 | the 72.5 mm of pencil behind the holder, as the old model draws it (never measured) |
| pen | pen (the only `is_pen` capsule) | 5 | the 20 mm of graphite past the cap; the capsule's surface ends exactly at the tip |

Why this model. The old code had four: fat sausages about the lines between joint origins
(`coordination.py`, radii 90 to 130 mm, up to 131 mm of air), vendor sphere sets (shown by the
2026-09 audit not to contain the arm, worst 235 mm outside on link0), the base column bands, and
the self-collision table in `selfcoll.py`, which fits three capsules to each link's own metal
with every radius the exact largest distance of any mesh vertex, rounded up to the millimetre.
The last is the tightest one that contains the metal, so it is used for links 0 to 7 as is. Its
hand capsules were fitted to the stock fingers, but the rig has Fat finger blades reaching 80 mm
further, and its tool was a 50 mm "L" that the 2026-09-03 holder correction showed no longer
contains the holder. So the hand, the blades, the holder and the tail were refitted
(`tests/oracle/fit_arm_capsules.py`) on the meshes of the real build in
`assets/system_model/meshes`, with a second demand beside containment: the capsules must not
reach further toward the paper than the metal does. A first fit (3 + 2 + 2 + 3 capsules) put the
round end of a capsule 14.7 mm below the holder's cap with the hand square to the paper, which
left no drawing pose any clearance. The refit places each capsule so that the mesh points it
covers near the paper lie on its underside, and adds small capsules along the edges nearest
the paper. Measured heights above the paper with the pen tip on it (lowest point, mm):

| part | square: mesh | square: capsules | leaned up to 15 deg: mesh | capsules | worst gap over the cone |
|---|---|---|---|---|---|
| holder (housing and cap) | 14.73 | 14.73 | 9.41 | 8.28 | 1.20 |
| blade, left | 37.18 | 37.18 | 12.44 | 11.24 | 1.21 |
| blade, right | 37.18 | 37.18 | 12.05 | 10.85 | 1.21 |
| gripper body | 83.41 | 82.37 | 45.79 | 44.65 | 1.20 |
| pencil tail | 95.38 | 94.75 | 80.67 | 80.34 | 1.20 |

Before the refit the capsule column read 0.03, 7.73, 8.23, 48.53, 91.75 square and -7.99, -5.59,
-9.83, 10.01, 77.82 leaned (below zero means through the paper), with gaps up to 37 mm. "Leaned"
is the worst over 0, 5, 10, 15 degrees in 8 directions; the gap is checked on those and 300 more
random leans in the cone. The pen itself touches the paper, of course; the closest thing after
it is the holder's cap edge, 9.4 mm up at a 15 degree lean.

`reach[j, k]` bounds, over all configurations, the distance of any point of capsule k (radius
included) from joint j's axis; turning the joints by dq moves the capsule by at most
sum_j reach[j, k] |dq_j|. Exact for capsules fixed to the frame the joint turns; the triangle
inequality along the chain further down.

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

With joint 7 fixed, the hand pose fixes the wrist centre. The distance from the shoulder to the
wrist fixes the elbow angle (two roots); the forearm must then lie on a cone around the
shoulder-to-wrist line and square to joint 6's axis (two roots); and the upper arm can be
reached two ways from the base (two roots). So there are up to 8 answers, returned in 8 slots;
slot b always means the same root, so the same slot is the same arm shape from pose to pose.
Joint limits are passed in. Every answer is checked by our own forward kinematics and kept
only if it puts the hand on the pose to 1e-7 m (answers land at 1e-15; none has fallen between
1e-9 and 1e-7). Where the pose is truly singular the answer is flagged instead of dropped: flag 1
means joint 2 is exactly 0, where joints 1 and 3 turn about the same axis and only their sum is
fixed; one member of that family is returned. Very close to that point (joint 2 about 1e-8) joints
1 and 3 are ill-conditioned and come back to about 1e-5 rad, their sum exact.

**What was wrong with the old solver** (the vendored `franka_analytical_ik`, which the old code
keeps using): it had the Panda's limits built in, so it never returned joint 6 above 3.7525 (a
fifth of the FR3's joint-6 range), |joint 7| above 2.8973, |joint 2| above 1.7628 or |joint 3| above
2.8973; it computed only one of the two elbow roots, so it never returned an almost straight
elbow (joint 4 above -0.467); and it replaced every configuration with joint 2 within 0.045 rad of
zero by a made-up one that misses the pose. Of random configurations only 66 % came back from
their own pose; of drawing configurations only 57 %, and 26 % of drawing poses got no answer at all.

**What that buys on the paper.** Tips on a 2 cm grid on the paper 0.97 m below the base out to
0.95 m, 8 spins, no lean, joint 7 on the old 16-value grid, gates 0.15 rad and sigma_min 0.08:
tips with at least one usable answer 4724 old, 4786 new (+1.3 %, new ones both right under the
base and at the rim); (tip, spin) pairs 28 082 against 29 973 (+6.7 %); usable answers 125 816
against 204 605 (+63 %). Reach barely grows; what grows is the choice at each point, which is
what the local planner searches.

## Reach at the paper and what limits it

Measured by `tests/reach_cases.py` (4 s): paper 0.97 m below the base, square to the base axis;
tip positions along a ray from the axis at 5 mm steps; 16 hand spins; leans 0 to 35 degrees in
5 degree rings, 8 directions each; q7 on 64 values; all 8 IK slots. A radius counts if at least
one configuration passes the gates of that row. Obstacles and self-collision are ignored for
the rim; the last column checks the arm's self-collision gate (23 mm) at the rim.

| gates | rim (m) | self-collision at the rim |
|---|---|---|
| 1 today: margin 0.15 rad, sigma_min 0.08, lean 15 deg | 0.785 | passes, all 14 passing configurations |
| 2 margin 0.10 | 0.785 | passes (19 of 19) |
| 3 margin 0.05 | 0.785 | passes (28 of 28) |
| 4 margin 0 (the hard limits) | 0.785 | passes (33 of 33) |
| 5 sigma_min 0.04, margin 0.15 | 0.800 | passes (22 of 22) |
| 6 sigma_min 0, margin 0.15 | 0.805 | passes (12 of 12) |
| 7 lean 25 deg, margin 0.15, sigma 0.08 | 0.785 | passes (26 of 26) |
| 8 lean 35 deg | 0.785 | passes (43 of 43) |
| 9 loosest: margin 0, sigma 0, lean 35 | 0.805 | passes (98 of 98) |

What ends it. The arm itself ends at 0.805 m: at 0.810 there is no configuration at all, even
with the joint limits removed, so that is the stretched arm, not a limit. Inside that, the only
gate that bites is sigma_min: the last 2 cm before full stretch are near the elbow
singularity. Joint-limit margin and lean change nothing at the rim. One centimetre beyond
today's rim (0.795 m, lean up to 15 deg) there are 173 configurations; 90 of them pass the
margin and none passes sigma_min 0.08. The best has sigma_min 0.065 and a margin of 0.140 rad
(joint 3 at -2.761), so it fails first on sigma_min. Relaxing sigma_min to 0.04 buys 15 mm
of radius; nothing else buys any.

## Measured (tests/test_kernel_arm.py)

| test | result |
|---|---|
| hand, tip, all link frames vs old code, 10 000 random configurations | worst 4.4e-16 m |
| IK round trip, 100 000 random configurations | original found for 100.000 % (old solver 66.4 %); 3.02 answers per pose (old 1.97); worst pose error 2.4e-15 |
| IK round trip, 2 000 drawing configurations | 100 % (old 56.8 %); poses with no answer 0 % (old 26.3 %) |
| every old answer among the new ones (20 000 reference poses) | yes, to 1.3e-6 rad (the old solver's own precision near singularities) |
| `hand_pose` -> `ik` -> `tip`, lean up to 15 degrees | worst tip error 5.9e-16 m over 12 327 solutions |
| `reach` bound on capsule travel, 10 000 random moves | never exceeded; travel / bound median 0.25, worst 0.75 (small moves 0.29, 0.76) |
| `sigma_min`, `limit_margin` vs old code | 4.7e-16, exact |
| mesh vertices outside their capsules (worst, mm; negative is inside) | link0 -0.14, link1 -0.88, link2 -0.34, link3 -0.35, link4 -3.31, link5 -0.00, link6 -0.14, link7 -1.42, hand, blades, holder, pencil tail -0.00 (the refit is tight by construction), pen 0.00 (the tip, by design) |
| self pairs | 784; none is overlapping in every configuration; 86 % of random configurations clear 23 mm |
| speed, one core, batch 10 000 | on a quiet machine (wall time, 40 capsules): fk 1.4 M/s, body 0.40 M/s, IK solver alone new 1.13 M poses/s against old 0.75 M. CPU time under load average 70 (62 capsules): fk 0.10 M/s, body 0.09 M/s, sigma_min 0.11 M/s, IK solver alone new 0.80 M against old 0.43 M poses/s |

Regenerate the reference data with
`ARIS_RIG=proposed ARIS_TOOL=lateral ../.venv/bin/python tests/oracle/make_arm_reference.py`.
The containment test reads the meshes directly and needs `trimesh` (present in the venv).
