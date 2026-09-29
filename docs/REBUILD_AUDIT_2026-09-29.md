# Audit of REBUILD_PLAN_2026-09-29

Adversarial review, 2026-09-29. Everything below was checked against code and notes on this
machine; where I could not check something I say so. Numbers I measured myself are marked
*(measured)*. No file in the repo was modified except this one. No robot was contacted.

---

## 1. Executive summary — top findings, ranked

1. **The diagnosis in §2 is wrong, and I profiled it.** *(measured)* The single-arm word plan is
   66.8 s: `arm_program` (pen-up routing/transits) **58.6 %**, `scene_check` **36.9 %**, the stroke
   planner **2.9 %**, the ladder DP **0.1 %**, IK **8 %**, the raw C++ IK solver **0.3 %**. 80 % of
   the run is collision checking, but it is **capsule-vs-32-static-boxes**, not capsule pairs, and
   ~28 % of the whole run is a 36-iteration ternary search that has a closed form. **Change** the
   diagnosis and re-aim the plan at pen-up routing and certification; ~64 % of the run is recoverable
   with no new dependency at all.
2. **§3's "rebuild, do not clean in place" is not supported by the code census. Refactor.** About
   **48 %** of the package is kernel that survives essentially verbatim (kinematics, gates,
   certification, stroke planning, transit routing, atlas, planfile/export); the genuinely
   pattern-specific part is ~16 %, and **`staged.py` is a leaf that nothing imports** — it can be
   deleted without touching anything below it. `allocate.py` and `writing.py` hold single-arm lessons
   that are not staged-specific. **Change** §3 to "delete the leaf, keep the kernel", which preserves
   144 recorded defect-fixes for free instead of re-earning them.
3. **Pete's leader/follower strategy is already built and measured, and the plan does not say so.**
   `traces.leader_follower_pattern` is leaders 13/71/2 → followers 17/31/97 → all-six pass:
   **99.988 % flown, certified, makespan 402.7 s against the joint conductor's 209.9 s, TTFM 0.19 s,
   planning 1 406 s**, with **followers keeping 11.1 % of offered ink (stage A: 0 %)**. **Change** the
   plan to "improve the measured pipeline", every claim an A/B against those numbers.
4. **The geometry vindicates the 1-2-1 choice and answers the virtual-walls question — with
   numbers.** *(measured)* Leader diagonals clear the 50 mm gate on **99.2-99.3 %** of ~1 M certified
   pose pairs and on **100 %** once the pens are 0.6 m apart; transverse (leader↔follower) pairs clear
   on only **81-82 %**. **Agree** with the pattern. Row bands at table y = ±605 mm with a 25 mm
   setback make the three leaders **provably independent at +66 mm** and keep the leader group
   **56.7 %** of the canvas (69.1 % unwalled); walling all six into their own blocks makes every pair
   clear at **+55.9 mm** over **57.4 %** of the canvas with *no pairwise certificate at all*. Two
   older claims **do not reproduce at h = 0.970** and should be dropped: there is **no inner
   elbow-fold hole** (`r_GO` = [0.01, 0.748] m — the lateral pen reaches under the base), and a
   **parked partner blocks 0 of 5 906 drawing cells**; the 2 % that is blocked is all pen-up hover.
5. **"Fill the gaps at the end" is the main event, not a remainder.** The final all-six pass drew
   **81 %** of the picture, and stage C cost **3 075 s of planning wall for 126.6 s of motion**.
   The recorded verdict is blunt: *"a pattern that defers most of the picture to the conductor has
   reinvented v19 with a slower planner."* **Reject** the framing.
6. **Both of §5's synchronisation claims are wrong, and the pacing contract has a hole.** The JTC
   does **not** wait at a future header stamp — it interpolates from the current state to point 0
   across the pre-stamp window (`trajectory.cpp:94-119`), so arms sync on *arrival* and an arm whose
   `points[0]` is 0.64 m away starts an uncertified drift the instant the goal lands; the mechanism
   has never been used anywhere in this codebase. Timing-tolerance certification is **unsound for
   drawing**: measured on this rig it PASSes 0-0.75 s, **FAILs 1.00-1.25 s**, PASSes 1.75-10 s,
   symmetric window **0**, and the executor has no clock and never reads `t_s`. And the plan's
   "timed joint plans" are not executable by an acceleration-limited driver: the same trajectory
   reads **37.7 rad/s² at 48 Hz and 1 422 rad/s² resampled to 1 kHz** against a **10 rad/s²** gate,
   and time-scaling cannot fix it because a corner is impulsive however slowly you fly it.
   **Change** claim 3, **reject** the timing tolerance (certify in s with a path governor), and put
   acceleration and jerk continuity into the plan-file contract.
7. **Calibration attacks the wrong error first and overpromises the accuracy.** Nothing measures
   where the paper is, **by design**: the ladder gate is off by default and **returns success without
   measuring for inverted arms** (`draw_rtff_supervised.sh:590-593`), and `config/site.json` ships
   `RTFF_CONTACT_DESCEND: "0"` — all six arms are inverted, which is a complete account of "hits the
   table / hovers and does not draw". And "bulletproof sub-mm from touches" is not available: base
   x, y and yaw are exactly unobservable from plane touches, and the measured joint-5 residual
   (9.3 ± 1.0 mrad) is **2.3-2.8 mm at the tip** and is a *control* steady-state error that differs
   between the position law you calibrate in and the impedance law you draw in. **Change**: runtime
   contact-finding first; accept by **drawn mark measured externally** (< 1 mm inter-arm); and note
   that a tip found *deeper* than modelled immediately eats the **0.5 mm** of paper-chain headroom the
   shipped programmes hold, so every certified plan is invalidated by the calibration that fixes it.
8. **VAMP is the wrong kernel here.** There is no `vamp.fr3`; the sphere cover and the arm count are
   compile-time; `vamp.panda`'s joint 4 box is wider than the FR3's; `Environment` has no time axis,
   so a partner's swept trajectory is inexpressible — and sphere-ising it was measured to cost a
   **median 195 mm** of clearance. Every vendored Franka sphere model here was measured **optimistic
   on every body (worst +235.2 mm on link0)**, and the project's own rule is that flipping the
   collision model is **a re-certification, not a commit**. **Change** to your own capsule kernel over
   the *measured* geometry, and keep a second, independent checker.
9. **The corpus's lost ink is mostly the canvas rim, and §2's table overstates the baseline.**
   `traces.BLOCK = (0.16, 0.00, 1.64, 3.62)` withholds a 0.16 m rim from every arm and is **the
   single largest source of lost ink in the corpus (6.41 m)** — and it is *right*, because at
   h = 0.970 the rim genuinely is unreachable (union 97.72 %; the corner is 850 mm from the nearest
   axis against a 748.7 mm certified radius). The rest is `no_drawer` from **single-cell offers**, so
   **change** the split rule to `s*` rather than the cell boundary. Also: **only `scatter` of the five
   bench pictures is certified**, and the quoted percentages are at **tilt 15°, which is not the
   default** (tilt is worth 95.63 → 100.00 % on CSAIL and 85.42 → 97.23 % on duotone).
10. **Incremental dispatch is unsafe as specified, and the milestone order cannot be executed.**
    Milestone 2 depends on 3 and 4. And the recorded lesson that kills the naive version is
    `staged.py:2946-2969`: *"it happens first" is not a defence against a claim quantified over all
    pairs* — a clear-out routed without the rooms its destination was chosen in dipped to **−53.1 mm**
    0.7 s after leaving, from a pose pair **7.00 s apart on the two clocks**. **Change**: add a lease
    protocol plus the rule that every dispatched chunk ends in a pose certified holdable
    indefinitely; one fleet-wide re-time rate, never per arm; reorder the milestones and put honest
    durations on them (~4-5 months for one engineer, of which the kernel is ~3 weeks).

---

**What the plan gets right, so it is not lost below.** The one-way import rule with a lint test;
config as data with no addresses in code; the plan file as a hashed cross-machine contract; a
drawing-agnostic input (polylines with ink id) with a no-drop invariant and corpus CI; the 1-2-1
leader choice; incremental planning as the route to time-to-first-motion; "one stroke = one
continuous joint path"; and the recognition that stage 1 of the controller redesign is a prerequisite
for concurrent drawing. Four of those already exist in the repo and should be adopted rather than
re-invented. The problem with the plan is not its direction; it is that its central factual claims
are unmeasured and three of them are wrong.

## 2. Section-by-section

## A. Architecture and code structure (section 4)

**Verdict: mostly sound as a target shape, wrong in three specifics that will bite on day one.**

What is right and should stay: one-way import rule with a lint test; config as data with no
addresses in code; tests without hardware; a single front door. Note that three of these already
exist and work — `config/site.json` is exactly the proposed `config/site.json`, is read by
`scripts/day1.py` and the GUI, and has a test that forbids hard-coded addresses elsewhere;
`atlas.model_signature()` already stamps every artefact with the model it was made under; the
pathway CSV + `*.manifest.json` pair in `docs/ARIS2_CONTRACTS.md` §1 is already a
version-stamped cross-machine contract. The rebuild should adopt these, not re-invent them.

**A1. `robot/controllers/` as a colcon workspace inside the pip package will not work.**
`[tool.setuptools.packages.find] include = ["aris*"]` will sweep the ROS package's python, and
`colcon build` drops `build/`, `install/`, `log/` inside your distribution tree. Worse, `pip
install -e .` does not build a ROS package and `colcon` does not install a wheel, so
`pip install aris[robot]` on the operator gives you python that imports but no controller.
*Fix:* put the colcon workspace at the top level as a sibling (`ros2_ws/src/…`), not under
`aris/`, and make `aris run` require an already-sourced overlay and fail loudly if it is missing.

**A2. rclpy is not pip-installable; "one distribution, extras per role" cannot hold on Jazzy.**
rclpy on Jazzy is built against the system CPython and ships in `/opt/ros/jazzy`. Either the
operator install is a venv created with `--system-site-packages` (and then your numpy/scipy pins
fight the system ones), or `aris[robot]` is installed into the system interpreter. Two further
constraints already recorded in this repo: `pyproject.toml:21-25` says *"the robot PC is
offline"*, so any extra needs a local wheelhouse; and `third_party/franka_analytical_ik` is a
compiled pybind extension, so "same commit on both machines" also means "same Python ABI on both
machines". *Fix:* split the distribution in two — a pure-python `aris-core` wheel (kinematics,
gates, planfile, certify) with no ROS, and a ROS package `aris_robot` that `ament` installs and
that depends on the wheel. One repo, two artefacts. The plan's "one distribution" is the part
that breaks, not the mono repo.

**A3. Hash matching as specified will block work at the worst moment and still not be sound.**
A git hash is not a function of what actually ran (editable installs, dirty trees, uncommitted
patches — this repo's own history has "the pathway CSV writer is UNCOMMITTED" as a shipping
blocker). And a single hash conflates things that should fail differently. *Fix:* hash the
semantic inputs separately — `site geometry`, `calibration`, `gates`, `collision geometry`,
`planner version` — put all five in the manifest, and let the operator **refuse** on calibration
and geometry mismatch and **warn** on planner version. Add a `dirty` flag. This is what
`atlas.model_signature()` already does for one of the five.

**A4. "Legacy as oracle" cannot be an in-process import.** `frames` carries process-global,
env-var-selected state (`ARIS_TOOL`, `ARIS_RIG`, `frames.PEN_LAT`), and the repo already records
that mutating it *"leaked into 28 tests"* and that importing `scripts/day1.py` sets `ARIS_TOOL`
for the whole interpreter. Two copies of that in one process is a bug factory. *Fix:* the oracle
is a subprocess that writes an artefact; the test compares artefacts (plan files), not objects.
That also forces the plan file to be the real contract, which is what you want anyway.

**A5. `gates.json` as flat data loses the derivations that make the gates safe.** The gate
constants are not independent numbers: `rig_final.STATIC_PLAN_MARGIN = 50 + 13 mm`, where the 13
is derived from the checker's own sampling constants and pinned by a test that re-derives it;
`paper.ROOM_FLOOR = FRAME_FLOOR`; `coordination.PAIR_MARGIN = 0.050` is
`SAFETY_M 0.020 + CALIB_M 0.030` (`coordination.py:205-207`) — i.e. **30 of the 50 mm inter-arm
gate is an allowance for not having surveyed the bases**. Put the *inputs* in config and keep the
*derivation* in code with the test that proves producer-tighter-than-checker. And record in the
config that the 30 mm is retired by calibration, because that is a throughput lever (see G).

**A6. The plan file is not the only artefact that crosses machines, and the plan says so itself
two sections later.** Section 6 needs appendable chunks; section 7 needs touch data coming back
from the operator; the GUI needs live state. That is three channels, not one. Say so: (i) plan
file dev→op, immutable, hashed; (ii) observation file op→dev (touches, forces, tracking),
immutable, hashed; (iii) a live control/telemetry channel that is explicitly **not** a source of
truth for anything that gets certified.

**A7. Missing from the tree.** (a) measured collision geometry as versioned data with provenance
— the 2026-08-26 collision audit measured per-link radii and a 3-band base column, and found
capsules that *under*-covered the arm by up to 170 mm; that table is a safety artefact and must
not live as literals in a new kernel; (b) a run store (force logs, tracking residuals, executor
logs) — today they are in `/tmp` on the operator; (c) atlases and their signatures; (d) a
migration path for calibration files.

**A7b. Four of the boxes in the proposed tree already have contents.** `aris_sixarm/gui/` is 2 668
lines with an operator layer (ssh dispatch, health gating, typed-confirmation run button, "identify
arms", site-setup form); `aris_sixarm/export/pathway.py` is the versioned cross-machine contract;
`aris_sixarm/sil/` is a Drake loopback of the impedance law with the live YAML; `aris_sixarm/execute/`
is 1 414 lines of program/runner/backends. The plan lists `gui/`, `planfile/`, `tests/sil/` and
`robot/` as new work. Decide deliberately which of these four is re-written and why, because
re-writing the SIL in particular means re-deriving the impedance replica from a controller source
that **is not in git** (B4).

**A8. "One CLI, no loose scripts" is right for the operator and wrong for research.** 90 scripts
exist because 90 experiments were run, and the audit trail in `docs/` is the project's main asset.
Rule it as: the CLI is the only thing anyone runs *on the operator*; research entry points live in
`tools/`, are not installed, and may not be imported by `aris/`.

**A9. One thing the plan does not say and should: nothing that is certified may depend on wall
clock.** `sequence.TIME_BUDGET` is a wall-clock budget above 16 segments per arm, and
`docs/BENCH.md` records that this makes `hatch` and `duotone` irreproducible across machines and
across concurrency. A hashed plan file whose content depends on how busy the machine was is not a
contract. Budget in node expansions, not seconds.

---


## B. Hardware interface (section 5)

**Verdict: of the five claims in §5, one is confirmed, two are half right and two are wrong; the
four verbs are missing at least seven; and the timing-tolerance idea is unsound for drawing.**

Caveat that applies to all of it: **there is no ROS on this machine.** `/opt/ros` does not exist,
`import rclpy` fails, `dpkg -l | grep -c ros-` is 0, and the vendored
`Aris_Kindt/franka_ros2_ws/src/joint_trajectory_controller/` is **source only, never built here**.
Its `package.xml:4` says version **3.3.0** (ros2_controllers 3.x, Iron-era) inside a **Humble** tree
(`franka_ros2_ws/src/Dockerfile:1` `FROM ros:humble`), while the rig runs **Jazzy** (every dispatch
script sources `/opt/ros/jazzy/setup.bash`, e.g.
`worktrees/aris2-rtff/dispatch_arm13_0730_circles.sh:4`) with `franka_ros2` branch `jazzy @ 3e48835`.
So the JTC code quoted below is *indicative of the family*, not the running binary. Verifying it on
the operator is a 10-minute job and is on the list in section 4 below.

### B1. The future-stamp claim is wrong in the way that matters

Claim: *"the trajectory header stamp is honoured as a start time by the JTC, so goals to several
arms with the same future stamp start together within a few ms."*

The stamp **is** honoured as a reference time (`trajectory.cpp:30,56,95`;
`joint_trajectory_controller.cpp:1164-1168`), but **the arm does not wait.**
`joint_trajectory_controller/src/trajectory.cpp:94-119`:

```
 98:  if (sample_time < first_point_timestamp)
101:    if (interpolation_method == ...::NONE)
103:      output_state = state_before_traj_msg_;
105:    else
112:      interpolate_between_points(
113:        time_before_traj_msg_, state_before_traj_msg_, first_point_timestamp,
114:        first_point_in_msg, sample_time, output_state);
```

With the default `interpolation_method: "splines"`
(`joint_trajectory_controller_parameters.yaml:69-76`), the controller interpolates from the state
captured at goal receipt (`joint_trajectory_controller.cpp:171-182`) toward **point 0** across the
whole pre-stamp window. So the arms **start drifting the instant their goal lands** and merely
*arrive* at point 0 together.

Two consequences:
- **Synchronisation:** it works only if `points[0]` equals the arm's current measured pose. That is
  a precondition the plan must state and the sender must check.
- **Safety, which is worse:** this is the mechanism behind the already-recorded hazard *"FIRST MOVE
  = uncertified straight ramp from the measured pose to row 0"*. The exported day-1 CSVs start
  **0.64 m / 4.16 rad from the park**, so a goal stamped 2 s in the future would have the arm
  crossing that gap, uncertified, over those 2 s. `day1.py send --from-q` already refuses > 0.05 rad
  / 10 mm for exactly this reason; the rebuilt `move` verb must keep that refusal.
- If a true hold is wanted, `interpolation_method: "none"` gives it (`trajectory.cpp:101-104`) — at
  the price of no interpolation at all.

Also relevant to any synchronisation budget: the quantisation floor is one controller_manager
period, and the two configs on this machine disagree — `update_rate: 1000`
(`franka_bringup/config/controllers.yaml`, `franka_fr3_moveit_config/.../fr3_ros_controllers.yaml:3`)
versus **`update_rate: 60`** (`my_ros2_ws/src/fr3_moveit_config/config/ros2_controllers.yaml:3`),
i.e. **16.7 ms**, in the file whose `fr3_arm_controller` is the position one that matches the rig.
Settle which ships.

**And the mechanism has never been used.** All six `FollowJointTrajectory` senders in the operator
tree leave `header.stamp` unassigned (`arm2_go_hover.py:84-85,90`;
`arm2_draw_circle_ika.py:199-203`; `arm2_draw_spiral_ika.py:196-200`;
`arm2_draw_pathway_ika.py:154-155,193`; `ee_edge_run.sh:53-55`). Timing is carried entirely by
`time_from_start`. The plan describes a proposal as if it were existing behaviour.

### B2. Tolerances are all off, so the JTC will not abort on a tracking failure

`constraints.goal_time` defaults to **0.0** and every per-joint `trajectory`/`goal` tolerance
defaults to **0.0** — *"Defaults to zero (ie. the tolerance is not enforced)"*
(`include/joint_trajectory_controller/tolerances.hpp:46-49,74-89`) — and **none is configured in any
yaml on this machine**. There is also no start-state check inside the JTC:
`validate_trajectory_msg` (`joint_trajectory_controller.cpp:1143-1246`) never compares `points[0]`
to the current state. The check people remember is MoveIt's `allowed_start_tolerance: 0.01`
(`my_ros2_ws/src/fr3_moveit_config/config/moveit_controllers.yaml:5`) and it applies only when
MoveIt executes. **Configure path and goal tolerances, per joint, in the new controllers.yaml.** It
is the cheapest safety net on the list.

### B3. Goal chaining: replace, not append — but it does work

`goal_accepted_callback` (`joint_trajectory_controller.cpp:993-1009`) calls `preempt_active_goal()`
then `add_new_trajectory_msg()`; both write the same realtime buffer
(`:1251` `traj_msg_external_point_ptr_.writeFromNonRT`), both before the next `update()`, so the
hold-position write is overwritten and never read. Net effect is a clean replace, and
`Trajectory::update()` (`trajectory.cpp:53-58`) re-captures `state_before_traj_msg_` from the
current state, which bridges consecutive goals. **So chunk-by-chunk dispatch on the JTC is possible
without a stop, provided each new trajectory's first point continues from where the arm actually
is.** Note the topic interface is inconsistent (`topic_callback:932-945` replaces without
preempting) — use the action, not the topic.

### B4. What the impedance controller and the executor actually accept today

- The tracked controller copy
  (`worktrees/aris2-rtff/aris_kindt_dwatkins_ztouch_control/ros2_ws/src/cartesian_impedance_controller/src/cartesian_impedance_controller.cpp`)
  is **350 lines and IS in git** (commit `196ac13`). The 378-line, not-in-git controller is a
  **different, off-machine file** on the operator. So the plan's parenthetical is right about the
  live one and wrong about what is available here.
- **It has exactly one subscription** (`:286-290`, `geometry_msgs/PoseStamped` on
  `/cartesian_impedance/equilibrium_pose`), zero publishers, and **no joint-reference / nullspace-q
  subscriber of any kind**.
- **No runtime parameters.** Everything is read once in `on_configure` (`:224-277`) and there is
  **no `add_on_set_parameters_callback` anywhere in the package** — yet the executor sets stiffness
  per phase via `set_parameters` (`rtff_pathway_exec.py:212-213,457-465`). On this copy those calls
  succeed and do nothing. If per-phase K works on the rig, the live 378-line controller has a
  parameter callback this copy lacks. **That is a load-bearing divergence and a reason to get the
  live source into git before designing against it.**
- At activation it latches the current tip pose as the equilibrium **and the current posture as the
  nullspace target, overriding the configured `q_nullspace`** (`:301-331`, and the yaml says so at
  `config/cartesian_impedance_controller.yaml:41-45`). Anything that assumes a configured posture is
  wrong.
- `rtff_pathway_exec.py` (2478 lines) reads `kind, x_m, y_m, z_m, qx..qw, intensity, q1..q7`
  (`:328-353`). **`t_s` is not a column it reads at all** — nor are `stroke_idx`/`wp_idx`. `q1..q7`
  are parsed but dead unless `--joint-ref` / `RTFF_QREF=1` (`:226-227`, `:369`). Pacing is arc length
  on an open-loop wall clock at 50 Hz (`:1847`, `:2201`, `dt = 0.02` at `:185`). Defaults:
  draw **0.02 m/s** (`:2371`), **travel 0.08 m/s** (`:2372`) — the plan's "travels at 0.02" is the
  value `config/site.json` passes to the *supervisor* (`RTFF_TRAVEL_SPEED`), not the executor
  default.

  **This is a safety finding, not a nitpick.** Because the executor ignores `t_s` and paces by arc
  length at its own speed, **the plan's joint-velocity certificate does not transfer to the
  hardware.** The day-1 exports were paced and certified at `QD_FRAC = 0.30` of `frames.QD_MAX`
  (verified at CSV spacing *and* at 1 kHz, with over-limit treated as a refusal) at a realised draw
  speed of 0.0867 m/s; the executor will fly the same rows at whatever *it* is configured to. A
  dispatch that forgets `RTFF_TRAVEL_SPEED=0.02` runs the transits at the executor's 0.08 m/s
  default — **4× the speed the rows were certified at, i.e. ~120 % of the joint-velocity limits.**
  Either (a) the executor must honour `t_s`, or (b) the plan must be paced at *the executor's*
  speeds and the exporter must refuse a file whose manifest speeds differ from the dispatch
  environment. Today neither holds, and nothing checks it. Note also that `docs/ARIS2_CONTRACTS.md`
  §1 already reserves `speeds` in the manifest for exactly this — make the dispatcher compare them.
- **The joint-reference patch exists and is uncommitted** (`git status` ` M rtff_pathway_exec.py`,
  `+55 −2`; publisher at `:232`, publish at `:374-377`). With no consumer, turning it on publishes
  into the void. Also a latent bug: `rim_tilt_waypoints` rewrites `wp["q"]`/`wp["xyz"]`
  (`:167,171`) and never re-solves `wp["qj"]`, so `RTFF_RIM_TILT=1` **plus** `--joint-ref` publishes
  a joint reference inconsistent with the pose it publishes.

**So stage 1 of the redesign needs three things, not one:** (i) commit the live 378-line controller;
(ii) add the `joint_reference` subscriber **with the mandatory slew-rate guard** already specified in
`docs/ARIS2_CONTRACTS.md:55-74` (a multi-radian step in the nullspace target drives the arm into
joint limits); (iii) fix the rim-tilt/qj inconsistency. Until (i) exists in git, the SIL is
validating a replica of a file nobody can read.

### B5. The four verbs miss at least seven things

`move / draw / switch / state` omits:

1. **stop** — immediate, controller-independent, and distinct from hold. Today there is no software
   stop: the abort procedure of record is *"physical e-stop"*.
2. **hold / resume** — exists today as a `/tmp/rtff_control` file token with verbs
   `stop|fault|remeasure|pause|raise` (`rtff_pathway_exec.py:760-772`), and `pause` lifts 30 mm and
   keeps republishing every 20 ms so the controller is not starved (`:774-815`). The new verb must
   preserve that "keep the reference alive while held" property or the impedance controller will
   fault.
3. **recover** — franka error recovery. The executor has **no** error handling: zero hits for
   `error_recovery`/`estop`; its state callback reads only `o_t_ee.pose` and `o_f_ext_hat_k`
   (`:312-326`) and never `current_errors`, `robot_mode` or `last_motion_errors`. All recovery lives
   in `draw_rtff_supervised.sh` (`:187,204,682,936,962`) and escalates to `systemctl restart`.
4. **brakes / enable / robot mode** — the health gate that exists (`aris_hold.sh:120-149`) requires
   ≥1 active hardware component, ≥3 active controllers and `robot_mode: 2`. That is the real
   readiness predicate and the `state` verb must return it.
5. **reflex / collision-behaviour configuration** — the executor has its own sentinels
   (`--f-max 3.5 N` with a 12-tick sustain at `:2355,2330`; press clamp `--d-max 0.008 m` at
   `:567,2348`; LOST-TOUCH `:1966-1992`, FLOAT `:2049-2161`, OVER-PRESS `:2065-2119`; stale-state
   watchdog 2.0 s → `EXIT_REFLEX` at `:756-758,1699-1733`). Note `d_max` is 8 mm here against the
   12 mm in the briefing — pick one. libfranka's own collision thresholds are not set anywhere in
   code.
6. **tool / payload** — `setEE`/`setLoad` appear in no code (fr3drivers audit says the same); today
   they are Desk-side. Since `docs/ARIS2_CONTRACTS.md:96-98` makes `setEE` = the nominal pen tip,
   the tool is part of the interface and must be verified, not assumed.
7. **identify** — which physical arm answers on this domain. `config/site.json` still records
   `"mounted": null` and *"which arm ids are mounted in the two middle positions is not known"*.

**And the ladder/plane gate.** It is not in the executor; it is in the supervisor, it is **off by
default**, and it **returns success without measuring** for inverted and wall arms —
`draw_rtff_supervised.sh:590-593`:

```
591:  [ "${RTFF_LADDER_GATE:-0}" = "1" ] || return 0
592:  [ "${ARM_INVERSE:-0}" = "1" ] && return 0
593:  [ "${ARM_MOUNT:-}" = "wall" ] && return 0
```

Every arm in the final installation is inverted, so **as built, nothing measures where the paper is
before a stroke.** Combined with `RTFF_CONTACT_DESCEND: "0"` in `config/site.json`, that is a
complete mechanical explanation of "arms sometimes run into the table, or hover and do not draw" and
it is a *runtime* defect, not a calibration one. Replacing this gate is the single highest-value
item in section 5 and the plan does not mention it.

### B6. MoveIt is not optional today

Claim 4 is **refuted as stated.** MoveIt is not the drawing planner (correct), but it has three live
runtime roles: position-mode motion executor via `/move_action` (`go_start_pos.py:46`,
`ladder_touch.py:171,187`, `touchdown.py:88-90`, `goto_xy.py:34`, `draw_position.py:124`,
`preposition_pose.py:46`); IK/collision service via `/compute_ik` (`ladder_touch.py:168,199-225`,
`reach_validate.py:24-25`); and an offline Cartesian path planner (`moveit_plan.py:2-21,29,91`).
Beyond that, `operator_franka_patches/moveit.launch.py:152-162` sets
**`moveit_manage_controllers: True`**, the MoveIt bringup *is* the ros2_control bringup, and
`pgrep -x move_group` is the stack's liveness signal (`draw_rtff_supervised.sh:175`). So "MoveIt
stays useful for RViz and the robot description only" understates a dependency you have to
deliberately remove — including every touch-pose move the calibration routine needs. Plan that
removal explicitly, because the calibration milestone depends on it.

### B7. Namespaces, domains, and one process with several domains

- `namespaced arm_<id>` is **refuted**: `arm_env.sh:4-5`, the 2026-09-09 briefing and
  `docs/ARIS2_CONTRACTS.md:93` all say **root namespace, isolation purely by `ROS_DOMAIN_ID`**, and
  every topic in `rtff_pathway_exec.py` is hard-coded absolute root (`:83-84,239,249-253`). The
  namespace machinery *is* wired in `operator_franka_patches/moveit.launch.py` (`:179,266,285,303`)
  but `default_value=''` (`:313-315`) and no caller passes one. Built and deliberately unused.
- `domain == arm id` is **confirmed** (`arm_env.sh:22-31,36`).
- **Multi-domain rclpy in one process: unverifiable here and unattempted in this codebase** — no
  rclpy, and **zero occurrences of `domain_id`** anywhere under `/home/franka/aris_project`; all 20+
  `rclpy.init()` calls are bare. rclpy does support per-context domain ids
  (`Context.init(domain_id=…)` → `rcl_init_options_set_domain_id`), but every `Node` and `Executor`
  must take `context=` explicitly, each context is a separate DDS participant, and shutdown and
  signals are per-context.
  **Recommendation: do not do it.** Either (a) keep one process per arm and have the fleet
  coordinator talk to those processes over non-DDS IPC (a Unix socket or local HTTP), which sidesteps
  the question entirely, or (b) put all six arms on **one** domain with real namespaces — the
  machinery already exists — which is also the only way to get a common ROS clock and is therefore
  the prerequisite for any synchronised start. Pick (b) if you want a fleet clock; pick (a) if you
  do not.
- **There is a DDS split-brain that the plan should clean up while it is in there.** Three
  incompatible worlds coexist: fastrtps + domain = arm id (live RTff, `arm_env.sh:36-37`);
  **cyclonedds + domain 0** (legacy z-touch: `recover_franka.sh:13`, `stop_franka.sh:18`,
  `probe_surface.sh:35`, `start_franka.sh:116`); fastrtps + domain 42 (`operator_scripts/
  franka_recovery.sh:16,43`). **`recover_franka.sh` and `stop_franka.sh` cannot reach a live RTff
  arm** — they speak the wrong middleware on the wrong domain. Any runbook that lists them as
  recovery tooling is wrong. `Aris_Kindt/aris_kindt_dds.xml` is orphaned; the live profile is
  `/home/diemut/aris_kindt/operator_scripts/dds_profile.xml` (`arm_env.sh:38`).

### B8. Controller switching semantics

`pen_switch.sh:16-19` calls `/controller_manager/switch_controller` with **BEST_EFFORT**
strictness — no `--strict` at any of ~24 call sites, and `fix_stack.sh:69` sets `strictness=1`
explicitly. STRICT(2) is used nowhere. A healthy switch takes ~1-4 s (tightest budget `timeout 4`,
`start_moveit_impedance.sh:24`; settling sleep 1-2 s with the intent stated at `touchdown.py:74`);
a wedged one escalates to **120-285 s** (`draw_rtff_supervised.sh:736-741`). The three command
interfaces are mutually exclusive at the hardware
(`franka_hardware/src/franka_hardware_interface.cpp:44-52,91-100`).
Design consequences the plan must absorb: **use STRICT**, treat a switch as a 1-4 s operation that
can fail, never switch while an arm is holding a pose it must keep, and remember that leaving an
arm in impedance mode makes every MoveGroup execution fail with `CONTROL_FAILED`
(`touchdown.py:55-57`). Also note `aris_hold.sh` performs **no** controller switch — hold is a
control-file token plus systemd, with a 25 s wait for proof of pen-up (`:85-92`).

### B9. Is the timing-tolerance idea sound for concurrent arms? No, not for drawing

Three independent reasons:

1. **It was measured, and the safe set is not an interval.** `scripts/timing_tolerance.py` exists
   and was run on the two-arm word: shifting arm 71's clock gave PASS at 0…+0.75 s, **FAIL at
   +1.00 and +1.25 s**, marginal at +1.5, PASS again from +1.75 to +10 s, and **any** early start
   collided. `certified_window_s` (largest |Δt| with every smaller shift passing) came out **0**.
   The script's own docstring says it: *"clearance is not monotone in Δt."* A scheme that certifies
   "±Δt" is therefore certifying a set it has not checked.
2. **N arms need N−1 relative offsets.** A tolerance box in 5 dimensions is not a grid you can
   sweep. The only tractable formulation is the one already in the repo: pairwise images
   `C_ij(s_i, s_j)` over the *whole* product of path parameters, from which a timing band is a
   restriction near the diagonal. If you want a timing tolerance, derive it from the pair image;
   do not measure it by sampling shifts.
3. **The drawing executor has no clock to be tolerant about.** It paces by arc length, open loop,
   at 50 Hz, and the force sentinels can stall or slow it. `t_s` is not even a column it reads. So
   the *time* at which arm A reaches a point is not a controlled quantity; only its *arc length* is.
   Certification for concurrent drawing must therefore be in the **s-domain, monotone in s, with a
   path governor** — which is exactly Pete's own stage 4 — not in the t-domain.

Corollary: the current standing constraint is right and the plan should restate it rather than
plan around it — *"assume NO synchronised multi-arm motion"* until a fleet clock and an s-governor
exist. Concurrent **position-mode transits** are a different and easier case, and those the JTC can
carry once B1's precondition is enforced.


## C. Speed (section 6)

**Verdict: the targets are achievable but they are aimed at 3 % of the run. The diagnosis in
section 2 is wrong in three specifics, and I measured it rather than argued about it.**

### C1. The measurement

Profiled on this machine (Ryzen 9 7950X3D, 32 threads, CPython 3.12, single-threaded run):

```
ARIS_RIG=proposed ARIS_TOOL=lateral .venv/bin/python -m cProfile -o day1_word.prof \
    scripts/day1.py word --arm 31 --hover --width 0.55 --height 0.061 --dy -0.10
```

66.79 s unprofiled (76.17 s under cProfile), 13 strokes, 1.318 m of ink, PASS.
**Import/startup is 0.13 s** — essentially all of the wall clock is planning.

The script's own stage timers, unprofiled run:

| stage | seconds | share |
|---|---:|---:|
| `plan_strokes` — 13 certified strokes, ladder DP and all | **1.92** | **2.9 %** |
| `cost_matrix` + `held_karp` — the sequencing | 0.52 | 0.8 % |
| **`arm_program`** — pen-up routing, transits, hover solving | **39.12** | **58.6 %** |
| **`scene_check`** — the independent timeline certificate | **24.63** | **36.9 %** |
| export | 0.15 | 0.2 % |

The output says the same thing: 3 940 CSV rows = **675 draw + 3 265 travel**. The drawing is 3 % of
the planning and 17 % of the rows.

Attributing every numpy frame's self time back to the project code that caused it (this sums to the
76.17 s exactly):

| category | seconds | share |
|---|---:|---:|
| **all collision / clearance** | **61.3** | **80.4 %** |
| — arm capsules vs the 32 static frame AABBs (`rig_final.segment_box_clearance` / `_point_box_d`) | 43.8 | 57.4 % |
| — of which the 36-iteration ternary search inside it | ~21.7 | **~28 %** |
| — `scene_check` timeline certificate (arm-vs-arm, static lower bound, self) | 11.9 | 15.6 % |
| — arm vs itself (`selfcoll`) | 3.6 | 4.7 % |
| — `paper.py` route/screen own arithmetic | 1.9 | 2.5 % |
| IK (of which the raw C++ solver is **0.26 s = 0.3 %**) | 6.1 | 8.0 % |
| FK — **all of it the scalar path**, which is 79× slower per config than the batch path | 5.3 | 7.0 % |
| certification (`validate`) | 0.9 | 1.2 % |
| sequencing / Held-Karp | 0.49 | 0.6 % |
| RRT structure excluding its collision gate | 0.32 | 0.4 % |
| **ladder / DP (`pwl.py`)** — 33 calls at 185 µs | **0.08** | **0.1 %** |
| I/O, export, imports | 0.26 | 0.3 % |

Counts: **153 702 IK solves**, ~**172 000 FK evaluations**, and **≈473 000 configuration-level
clearance queries** (261 240 static-gate rows + 151 425 arm-pair rows + 60 570 self rows) — an
aggregate **7 100 clearance queries per second**.

### C2. The plan's diagnosis, corrected

> *"every collision check is Python over capsule pairs one configuration at a time, and every layer
> above it multiplies that count: the IK yaw sweep, the ladder search over the stroke, the RRT, the
> certification pass."*

- **"all Python": TRUE.** There is no compiled collision kernel anywhere; the only C++ in the loop
  is the IK extension and numpy's ufuncs.
- **"capsule pairs": FALSE for the hot path.** The expensive check is **capsule vs axis-aligned
  box** — 7 arm capsules against **32 static frame boxes** — at 43.8 s (57.4 %). Arm-vs-arm capsule
  pairs are 5.0 s (6.6 %).
- **"one configuration at a time": FALSE for the certificate, partly false for the gate.**
  Instrumenting the kernels over a whole word plan: `pair_clearance` and `self_clearance` are called
  with N = 10 095 (the entire timeline) **every time**; the static gate runs at **mean N = 43.6**,
  with N = 1 in only 6 % of calls and 0.1 % of rows. The code is already broadcast; it is asked in
  batches that are too small, which is a different defect with a different fix.
- **"the IK yaw sweep … the ladder search over the stroke": FALSE.** IK is 8 % and the raw solver is
  0.3 %; the ladder DP is **0.1 %**. Making both free saves 6 s of 67 s.
- **"the RRT": FALSE as written, TRUE via its gate.** `transit.py`'s own bookkeeping is 0.4 %, but
  `transit.plan`'s cumulative cost is 20.2 s and ~98 % of that is its collision gate calling
  `chain_static_clearance` one small batch at a time.

The honest one-line version: **the planner spends 80 % of its life asking, in Python, whether an arm
capsule clears one of 32 axis-aligned boxes, and it asks ~1.8 M times per word through a 36-step
ternary search that has a closed-form answer.**

This matters for the rebuild because the plan's organising claim — *"everything above the kernel is
small because the kernel makes checks cheap"* — is only two thirds right. The measured hot spot is
not the stroke planner the plan wants to make fast; it is the **pen-up routing and the certificate**,
both of which are *layers*, and both of which the plan proposes to rewrite from scratch anyway.

### C3. Is ≤ 0.1 s single-arm RRT realistic? Yes.

Measured primitive costs on this machine, 32-obstacle scene, single thread:

| | configs/s | vs today |
|---|---:|---:|
| `paper.chain_static`, N = 1 — what the RRT gate actually does | **330** | 1× |
| achieved in the real word plan (mean N = 43.6) | 7 800 | 24× |
| `chain_static_clearance`, N = 1 000 — the pure-numpy ceiling, no new code | **23 846** | 72× |
| `vamp.panda.validate`, 59 spheres vs 32 cuboids, through a Python loop | **523 504** | **1 586×** |

And VAMP's planner, measured here (Python 3.10 venv, 64 cuboids, 100 random free start/goal pairs):
**95/100 solved, median 7.6 ms, mean 23.9 ms, p90 80.3 ms, max 113.6 ms**, simplify 1.6 ms.

So: ≤ 0.1 s is met at the median by a wide margin and is **at the limit at the tail** — p90 80 ms,
max 114 ms, and 5 % unsolved. State the target as a percentile, not a maximum, and define what
happens on the 5 %: an unsolved transit must become a priced `inf` edge that leaves the search
space, which is exactly what `sequence.cost_matrix` already does with a refused `paper.route`.

### C4. Is ~1 ms stroke tracking realistic? No, and it does not matter.

Measured: 13 strokes in 1.92 s = **148 ms per stroke** today. With a VAMP-class checker, the
collision part of that mostly disappears, but the floor is IK: the fiber for the lateral tool is
`s × phi × q7 × branch`, and at ~150 k solves per word the raw solver alone is 0.26 s. A realistic
post-kernel target is **10-30 ms per stroke**, i.e. 5-15× better than today, and **1 ms is off by
about two orders of magnitude.** More to the point, `plan_strokes` is 2.9 % of the run: driving it to
1 ms saves under 2 s of 67 s. Either restate the target as *per waypoint* (in which case it is
nearly met already) or delete it and put the target where the time is.

### C5. What the kernel is actually worth, and three cheaper things

**Ceiling for "replace the collision kernel and change nothing else":** the 473 000 clearance queries
at VAMP-class 523 k cfg/s is ~0.9 s instead of 61.3 s, so the word plan goes **66.8 s → ≈ 6 s, about
11× end to end.** Batch the FK and IK too and the floor is ~2 s. That is a real prize and worth
having. But it is **11×, not 100×**, and Amdahl is the reason.

Three levers measured on the existing code that need **no new dependency, no GPU, and no compiled
binary**:

1. **Replace the 36-iteration ternary search with closed-form segment-to-AABB distance.** Removes
   ~96 % of 2.98 M `_point_box_d` calls: **≈ 21.7 s of 76.2 s (28 %)**. It also makes the gate
   *exact by formula* instead of exact-by-argument-about-a-search, which is a safety improvement.
2. **Ask the static gate in batches.** N ≈ 1 000 instead of N ≈ 44 takes the static half from
   7 800 to 23 846 rows/s: **33.5 s → 11.0 s (a further ~30 %)**. This is an architectural change to
   the RRT gate and the router (collect candidate edges, check them together), not a kernel change.
3. **Stop calling scalar `frames.fk` from `scene_check._chain`, and stop building the chain twice.**
   Batch FK is 79× faster per config, and `check_timeline` builds the chain exactly 2× per
   (arm, sample) — 121 136 calls where 60 570 are needed. Together **≈ 4.5 s (6 %)**.

Those three are **~64 % of the run** and could be done in days. They should be done first, because
they also tell you what is left, and they do not commit the project to a binary dependency.

### C6. VAMP: what it can and cannot do here

**Can:**
- Collision-check a 7-DoF Franka at **523 k cfg/s** against 32 cuboids, and plan RRT-Connect in
  milliseconds.
- Obstacles: `add_sphere, add_capsule, add_cuboid, add_heightfield, add_pointcloud`. No meshes, no
  half-spaces.
- **Attachments are supported and already used** — `vamp.Attachment` with
  `add_sphere(s)/posed_spheres/relative_frame/set_ee_pose`, plus `Environment.attach/detach`. The
  `franka-station-sim` code builds one from the hand's Drake collision geometry
  (`station_sim/vamp_env.py:141`) and *measures* the EE transform rather than typing it. **A lateral
  pen holder goes in the same way and needs no recompile.** This is the best news in the section.

**Cannot, and these are the ones that decide it:**
- **There is no `vamp.fr3`.** The shipped robots are `sphere, ur5, panda, fetch, baxter`
  (verified: `/home/franka/git/vamp/src/impl/vamp/robots/` contains exactly those five headers).
  `franka-station-sim/station_sim/vamp_env.py:67` uses `ROBOT = "panda"`, and its `docs/planning.md`
  says so outright.
- **The Panda joint box is not the FR3's** — measured live: `vamp.panda` joint 4 upper `+0.0873`
  against the FR3's `-0.1518`. VAMP samples inside its own box, so RRT-Connect can legally return a
  path the FR3 driver refuses. `vamp_env.plan()` handles it by **rejecting and retrying**, never
  clipping (a clipped waypoint is no longer collision-checked). Any use here must do the same and
  must report the rejection rate.
- **The sphere model is compile-time.** `panda.hh`: `n_spheres = 59`, `resolution = 32`,
  `min_radius`/`max_radius` as `constexpr`. A different link set, a different cover, a changed
  margin — recompile.
- **No time-indexed or swept obstacles.** `Environment` is a static snapshot with no time axis and
  no sweep. `franka-station-sim/docs/planning.md` states the consequence: *"it cannot find a motion
  that needs both arms to move out of each other's way together"* — each leg is planned with the
  other arm frozen. For this rig that means the other five arms enter as **frozen configurations**,
  which is exactly the leader/follower assumption, so it is compatible — but it also means VAMP can
  never certify concurrent motion, and the swept-partner room has to stay in your own code. Note
  what that costs if you try to express a partner's *trajectory* as VAMP spheres: the repo already
  measured that reducing a leader's swept capsules to a 0.075 m sphere grid costs a **median 195 mm
  of clearance**, taking a follower from 55.9 % of poses clear to **0 %** (`aris_sixarm/exact_room.py`
  docstring; `docs/V2_STAGED.md` §22.3). Sphere-ising the partner is not a conservative refinement,
  it is a different and much fatter obstacle.
- **Packaging is the hardest part and there is direct precedent for it going badly.** VAMP is not
  in the planner venv and cannot be: every `vamp_planner` artefact on this machine is **cp310**, the
  planner venv is **cp312**, and no cp312 wheel exists — so VAMP means compiling and maintaining it.
  A 6-arm composite robot would be a **42-DoF hand-built robot with the booth geometry frozen into
  a binary**, and the station has already been bitten by exactly this: the fr3drivers audit records
  `bimanualpanda70180` as *"the compiled robot with this station's 0.70 m offset baked in … a
  different arm spacing needs a recompiled VAMP robot, not a config edit — the hardest
  station-specific dependency in the repo"*, and `third_party/vamp_bimanual` as a **binary-only
  copy taken from another workstation's venv that cannot be recreated without it.** This project
  changes `h`, the pitch and the yaws regularly; freezing them into a binary is the wrong trade.
  (There is evidence an FR3 robot has been built before — `bimanualfr3finray70`, 14-DoF, 178
  spheres, FR3-shaped joint box, exists as a cp310 binary in a retired venv — so it is *possible*,
  just not reproducible.)
- One practical trap for an inverted mount: VAMP's Panda carries `panda_link0` spheres reaching
  **0.03 m below the mounting plane**, which `franka-station-sim` works around with
  `BASE_SINK_M = 0.03`. With the base above the work, that sink is in the wrong direction.

**cuRobo:** not installed anywhere — only comparison-harness source in a retired repo — and there is
no `torch`, `cupy` or `warp` in any venv here. The GPU is present and capable (RTX 3090 24 GB,
driver 580, nvcc 12.0), so cuRobo is a from-scratch install plus a CUDA toolchain, and it buys
throughput this workload does not need: 473 k queries per word is a CPU-sized problem.

### C7. Recommendation on the kernel

**Do not adopt VAMP as the kernel. Write the capsule/sphere kernel yourself, in C++ or numba,
against the measured geometry, and keep the model in data.**

Reasons, in order: (i) the geometry must be *your* measured capsules and 3-band base columns, not a
compile-time sphere cover you did not choose — the 2026-08-26 collision audit found the old capsules
**failed to contain the arm by up to 170 mm** and that up to 78 mm of an 80 mm margin was model
optimism, so the collision model is a safety artefact that must be versioned data; (ii) a 6-arm
2×3 booth with per-arm yaw, per-arm height and a lateral tool cannot be frozen into a binary when
`h` and the pitch are still moving; (iii) the ceiling either way is ~11× end-to-end, and 64 % of
that is available without any dependency at all (C5); (iv) VAMP's obstacle model cannot express the
one thing this application is actually about — a partner's swept trajectory — without a 195 mm
clearance loss that has already been measured.

Use VAMP, if you use it, as a **second opinion in tests**: an independent checker built from a
different sphere cover is a genuinely useful oracle for a gate, and the project's own rule is that
the checker must share no code with the checked.

Benchmark acceptance for whatever kernel is chosen, stated as numbers so it can pass or fail:
≥ 300 k configuration-clearance queries/s single-threaded at N = 1 against 32 static boxes;
bit-identical verdicts to the current gate on a pinned corpus of configurations; a containment
proof against the manufacturer collision meshes; and a documented batch-size curve, because the
current code's problem is the curve, not the peak.


## D. Pete's leader/follower strategy

**Verdict: the strategy is geometrically sound at h = 0.970 — much more so than the older notes
suggest — and it has already been built and measured. Keep it. But it costs ~1.9× the makespan of
joint conducting, the followers keep almost nothing, and the "final pass to fill the gaps" draws most
of the picture. The plan must start from those numbers rather than propose the strategy as new.**

All geometry below is computed on this machine from `assets/site/h0970/atlas_proposed_h0970_lat0860/`
using the repo's own pairwise clearance (`coordination.clearance_matrix`, measured capsule table
`CAPSULES_LAT`), cross-validated against the independent `scene_check.pair_clearance` on 20 pose
pairs (identical to 0.1 mm) and against `docs/V2_WORKCELLS.md`'s pinned case (arm 31 @ (0.44,1.40) vs
arm 71 @ (1.24,1.40) = −160.8 mm, reproduced exactly). *(measured)*

### D1. What already exists

`aris_sixarm/traces.py:leader_follower_pattern` implements it: stage A leaders **13, 71, 2**
(priority 0-2) with followers **17, 31, 97** (3-5); stage B roles swapped; stage C all six under
`idle.conduct` with `scene_check.check_timeline`. The leaders are precisely Pete's 1-2-1 — one arm
per row, columns alternating — and `docs/V2_STAGED.md:1120-1160` quotes his specification verbatim.
The design insight the plan's §1.8 rests on is already in the code, already right, and already
measured.

### D2. The measured results

| | v19 joint conductor | zigzag | **leader/follower (first run)** | **leader/follower (final, lf10)** |
|---|---:|---:|---:|---:|
| CSAIL makespan | **209.9 s** | 432.6 s | 261.0 s | **402.7 s** |
| coverage | — | 94.2 % | 95.6 % | **99.988 %, certified** |
| time to first motion | 3 712 s | 0.280 s | **0.183 s** | **0.19 s** |
| planning wall | — | 769 s | 912 s | **1 406 s** |
| ink in the leader/follower stages | — | — | **2.97 m of 16.08 m** | — |
| ink in the final all-six pass | — | — | **13.10 m (81 %)** | — |
| follower ink that fitted | — | — | **11.1 %** (stage A **0 %**) | 66.9 % of offered, after standoff |

The pattern delivers what it was built for — TTFM 0.19 s against 3 712 s — and that is a real
success. It also costs **about 1.9× the makespan** at equal coverage. The plan asks for "arms almost
moving immediately" *and* "efficient drawing"; on the measurements these pull in opposite directions,
and the plan should say which wins when they conflict.

### D3. Are the 1-2-1 leaders really independent? On paper almost entirely; in the air 99.2 %.

**Paper footprints** *(measured, strict-GO = margin ≥ 0.30 rad and σ_min ≥ 0.14)*. Per arm
**1.53-1.62 m²**, i.e. **23.1-24.4 % of the canvas each**. Pairwise GO overlap, as a fraction of the
smaller arm:

| pair class | separation | overlap |
|---|---:|---:|
| same row (transverse, e.g. 31-71) | 610 mm | **47.3-49.2 %** |
| same column, adjacent rows (13-31) | 1 210.2 mm | 11.0-11.2 % |
| **leader diagonals (13-71, 71-2)** | **1 355.3 mm** | **3.8-3.9 %** |
| leaders two rows apart (13-2) | 2 420.4 mm | **0.0 %** |

So the leaders barely share paper: 0.058 m² and 0.060 m² of lens between the diagonal pairs, nothing
at all between 13 and 2.

**In the air**, over 0.92-1.04 M certified pose pairs per pair (atlas pose per cell, stride 2):

| pair | class | clears 50 mm | worst |
|---|---|---:|---:|
| **13-71** | leader diagonal | **99.29 %** | −208.0 mm |
| **71-2** | leader diagonal | **99.20 %** | −203.7 mm |
| **13-2** | leaders, two rows | **100.00 %** | ≥ +250 mm |
| 13-31 / 17-71 / 31-2 / 71-97 | same column, adjacent rows | 97.6-97.7 % | −236 to −261 mm |
| **13-17 / 31-71 / 2-97** | **transverse (leader↔follower)** | **81.4-82.1 %** | **−262.0 mm** |

All three leaders drawing at once: **98.48 %** of pose triples clear all three pairs (the
independent product is 98.49 %, so the constraints are essentially uncorrelated).

**So the answer to "are they really independent" is: 99 % yes, and the 1 % is localised.** The
13-71 conflict is a **±0.28 m band about the row boundary y = 1.2102** — 0.269 m² of arm 13's cells
and 0.259 m² of arm 71's — and **leader diagonals are 100 % clear once the two pen tips are 0.6 m
apart**. A transverse pair, by contrast, is still at −182.8 mm with its pens **1.0-1.2 m apart**.
That is the quantitative justification for the 1-2-1 choice, and it is worth putting in the plan.

Adding the pen-up hover to both arms' envelopes drops the leader diagonals to 99.1 % and the
transverse pairs to **72.3-72.7 %** — the hover layer is where the loss is, consistent with the
recorded history.

**Two older claims do not reproduce at h = 0.970 and the plan should stop carrying them.**
*(measured)*
- **There is no inner hole.** `r_GO` runs **[≈0.01, 0.748] m** and radial occupancy is 1.000 from the
  60-80 mm band outward for every arm — the 86 mm lateral pen offset lets the wrist stand off the
  point it is drawing, so each arm reaches directly under its own base. The elbow-fold hole is an
  inline-pen artefact.
- **A parked partner blocks nothing.** Worst park-vs-ink over all 30 ordered pairs is **+117.6 mm**,
  and **0 of 5 906** drawing cells are blocked fleet-wide. The "a partner standing at park kills
  26.2 % of fleet drawing poses" figure was h = 0.850. The residual at 0.970 is entirely in the
  **pen-up hover**: worst +1.4 mm, **2.22 %** of draw+hover cells under 50 mm. Same conclusion as
  before — the problem is the lift layer, not the park — but the number is 2 %, not 26 %.

### D4. Reach: the canvas is not fully covered, and the usable width is 1.50 m not 1.80 m

*(measured)* Union of all six arms' strict-GO = **97.72 %** of the canvas; **0.151 m² (2.28 %) is
reachable by no arm, and all of it is at the rim** — 194 cells at x < 0.16 m, 184 at x > 1.64 m, plus
corner bites. Cause: the canvas corner (0, 0) is **850.0 mm** from arm 13's axis against a certified
radius of 748.7 mm, i.e. **101 mm short**. The largest all-live rectangle is
**x ∈ [0.16, 1.64] × full length = 1.50 × 3.64 m = 5.46 m²**. (With the *uncertified* reachable
radius of 0.806 m the union would be 99.54 %, so this is a gate boundary, not a hard kinematic one.)

Redundancy over the canvas: **1 arm 57.28 %, 2 arms 37.55 %, 3 arms 2.22 %, 4 arms 0.67 %, none
2.28 %.** Note what that means for the plan's §1.3 ("few reconfigurations") and for any handoff
scheme: **over half the canvas has exactly one arm that can draw it.** There is no allocation freedom
there, and a refusal in that region cannot be reassigned — which is exactly the `no_drawer` failure
mode of H2.

**Action for the plan:** state the placement constraint (drawings must sit inside 1.50 × 3.64 m to be
fully live), and stop describing the rim as a coverage bug to be fixed in software.

### D5. Virtual walls: they work, here is where and what they cost

*(measured)* Method: for every certified cell, build the world chain and require **every capsule's
whole extent** — metal, not just the pen — on the arm's own side of the plane; a setback `s` per side
guarantees a `2s` pair gap. Verified by re-measuring the pair minima with the walls enforced.

First, the hard constraint: an arm's metal reaches **0.713 m past its own base in +x and 0.404 m
behind it**, so **a vertical wall must lie in x ∈ [0.774, 1.030] m** — outside that window one whole
column has *zero* compliant poses (measured: at x = 0.752 the left column keeps 0 cells).

| scheme | setback | fleet union | leader-group union | worst pair among leaders | worst pair overall | dead strips |
|---|---:|---:|---:|---:|---:|---|
| **B: horizontal planes at y = 1.2103 / 2.4203** (table y = ±605 mm) | 25 mm | **84.48 %** | **56.73 %** (3.758 m²) | **+66.0 mm** | −262.0 mm (same-row pairs share a band) | 160 mm + 140 mm |
| A: vertical plane at x = 0.9017 | 25 mm | 68.42 % | 35.91 % (2.379 m²) | +62.9 mm | −261.0 mm (same-column pairs untouched) | 140 mm |
| C: both walls (each arm in its own block) | 25 mm | **57.36 %** | — | — | **+55.9 mm — all six simultaneously independent** | 140 + 160 + 140 mm |
| (no walls) | — | 97.72 % | 69.10 % | −208.0 mm | −262.0 mm | — |

Read off three answers to Pete's open question:

1. **Scheme B is the wall for the leader/follower staging.** Row bands at ±605 mm with a 25 mm
   setback make the three leaders **provably independent at +66.0 mm** and leave the leader group
   **56.73 % of the canvas** — 82 % of the 69.10 % they have unconstrained. Scheme A keeps only 52 %
   of it. At zero setback the leader triple is only +6.3 mm, so **the 25 mm setback is what buys the
   margin** and it costs ~2 pp of area: cheap, and it should be a config number.
2. **Full asynchrony for all six is available and costs 40 pp.** Scheme C gives every arm its own
   Voronoi block, all fifteen pairs clear at **+55.9 mm**, and the six arms can then run completely
   independently over **57.36 %** of the canvas with no coordination of any kind. The remaining
   42.6 % needs a coordinated pass. That is a real, quantified design option the plan does not have:
   *phase 1 = six arms fully asynchronous inside walls over 57 % of the picture (no pairwise
   certificate at all, TTFM = one stroke plan), phase 2 = conducted remainder.*
3. **The cost lands on the middle row.** Confining arms 31 and 71 to y = 1.815 ± 0.605 costs **27 %**
   of their combined footprint (3.238 → 2.365 m²), and below ±0.40 m they have essentially nothing
   (0.690 m²). The middle row is where the walls hurt, which is also where every recorded failure
   was.

Two honest flags on these numbers, both from the method: they are **lower bounds**, because the atlas
stores one pose per cell (best joint margin) and a cell rejected as non-compliant may have a
compliant alternative pose at a different q7/lean/branch — re-solving under a half-space constraint
would raise every wall-compliant area by an unmeasured amount. And they are **static pose pairs**:
no pen-up leg, no interpolation; `scene_check.check_timeline` remains what certifies motion.

### D6. Why the followers keep nothing — and it is not what the plan assumes

The plan assumes the follower's problem is ink: *"the opposite arms then draw what they can."*
Measured, it is not. Running the per-piece ink gate as a measurement (`--no-follower-gate`) produced
a stage A **identical piece for piece and metre for metre**. The follower's bucket produces **no
timeline at all**: `writing.arm_program` cannot route the pen-up legs out of the held pose, into the
first stroke and between pieces with the leaders' rooms in the way. The blocker is **pen-up routing**
— which is also where 58.6 % of the planning time goes *(measured, section C)* and where the
transverse clear-fraction drops from 82 % to 72 %.

The chain of causes, each measured and each fixed in turn:

1. **The room's shape.** A leader's trajectory reduced to `cluster_capsules` spheres (0.075 m grid)
   costs a **median 195 mm** of clearance against the capsules it contains: against the spheres **0 %**
   of follower arm 31's ink samples cleared the 50 mm gate, against the exact swept capsules **55.9 %**
   did (`aris_sixarm/exact_room.py` docstring). `exact_room.py` keeps the spheres as a broad phase and
   answers exactly at 0.05 ms/pose.
2. **The leader's tour, not the geometry.** Even with exact rooms, every pose follower 31 could hold
   stood **+53.7 mm** from leader 71's room — legal at the 50 mm pair gate, unroutable at the 63 mm
   router floor — because the *leader's* tour hugged the follower's shoulder. Fix:
   `frozen.set_standoff` / `--partner-standoff`, the leader owing S extra clearance to the follower's
   **pose-invariant** links (base column plus links 0 and 1). At S = 90 mm the follower flew **66.9 %**
   of offered ink and the leader lost **zero**; at S = 130 mm the row closes up and it fails at −8.3 mm.
3. **The shoulder cannot be moved.** Park search over ~490 certified poses per arm found **0** poses
   that clear in the failing stages (*"the parked shoulder is pose-invariant to 4 mm"*), and the
   x-erosion alternative needs 1.04 m of erosion for 50 mm, costing 61 % of coverage. Recorded as
   **"NO PARK CAN MOVE A SHOULDER."**
4. **The last blocker was the other row.** Arm 31's 1.9 m did not fly because **arm 2 — a different
   row — was holding its stage-C entry hover over the sheet**; with arm 2 parked, all 8 pieces fly. No
   in-group ordering could ever fix it. Fix: `CONDUCT_HOME_FIRST`.

**The lesson the rebuild must absorb:** in a leader/follower scheme the binding constraint is not who
may draw where — the paper footprints barely overlap and a parked arm blocks nothing. It is **where
the arms that are not drawing are allowed to stand, and how anything gets from one stroke to the next
past them.** The plan's §6 has one sentence on transits and none on held poses.

### D7. What the plan is ignoring from the staged experience

Each was a defect found by measurement, and each would have to be rediscovered:

| lesson | what breaks without it |
|---|---|
| Pen-up transits must be certified against the paper | arms flew **253.6 mm below the canvas**, one *held* there for 0.75 s by a scheduled pause; cause = two hover poses on **different IK branches** with a straight joint-space line between them |
| The Cartesian via-walk must own its own last hover | handing the final hover to a pose solved near the target puts it on the target's branch and re-creates the flip the walk exists to avoid |
| Park poses must be searched with a bearing | centre-aimed parks **interpenetrate by 95.6 mm**; outward-only saturates at 75 mm and needs a bearing dimension to reach 97.3 mm |
| Ready/park poses must be checked for the **pen**, not just the chain | the inverted ready pose puts a 200 mm pen **16 mm below the paper** and a 300 mm pen **113 mm** below; conductor v1 parked four arms there three times a run and no gate looked |
| Gates must auto-refine until the verdict stops moving | a frame gate read 49.1 mm at sub=2 and 53.5 mm at sub=8 and refused 8 valid configurations |
| Producers must be tighter than checkers by a **derived** slack | `STATIC_PLAN_MARGIN = 50 + 13 mm`, the 13 derived from the checker's own sampling constants; without it the router proposes what the judge refuses, forever |
| The allocator must price feasibility, not metres | it handed an arm an ink-certified bag with **no flyable tour**, discovered three stages later |
| One cost model shared by allocator and sequencer | the balancer balanced loads the sequencer then moved; 84.8 → 77.8 s when fixed |
| Pace charges are per **binding interval**, not per move | pairing each move's fastest point with its tightest point asked for **5 476 s against 67 s** |
| Hover poses must be *scored* against frozen partners, not gated | raising `HOVER_HOLD_MARGIN` broke a pinned test and the PASS was **withdrawn**; the real fix was a clearance **term in the score**, opt-in, capped by the min of two caps, asked of one pose only so it *can refuse nothing* |
| A clear-out/tuck must be routed in the same rooms its destination was chosen in | a follower's clear-out dipped to **−53.1 mm** 0.7 s after leaving, and the offending pose pair was **7.00 s apart on the two clocks** (time-aligned: +169.5 mm) — a bug class a rebuild hits the *moment* it dispatches leaders early |
| Simultaneous mutual rooms are circular | arm 71's bucket flew in pass 1 and stopped flying in pass 2 — *"the scheme asks every arm to yield to a path the other has already abandoned"*; the priority sweep turns the cycle into a DAG and bounds a re-plan's blast radius |
| `freeze_sets` clears the keep-bands flag, so every caller must restate it | the retry silently routes in the relaxed room it exists to leave behind |
| The no-drop invariant is a `raise`, not a convention | `staged.py:2382-2398` refuses to finish a run whose coverage account does not close |
| Measured capsule radii and a banded base column | the schematic capsules **did not contain the arm** (link0 with connector and cable by **+170 mm**) and up to **78 of an 80 mm margin** was model optimism |
| Row separation was measured with **one** arm per row in the air | *"Rows are separated for drawing; they are not separated for four arms leaving four held poses simultaneously, and nothing in this build ever certified that"* — the row-conductor merge failed cross-row at **−191.9 mm** |

### D8. Is the gap-fill step where all the time and all the failures go? **Yes, already measured.**

- It drew **81 %** of the picture in the first leader/follower run — the main event, not a remainder.
- Its planning dominates the wall: the serialised **[31, 71] slot alone was 1 151 s of the 1 406 s.**
- It is where the certificates failed: stage C **FAIL −108.6 mm** in one integrated run, cross-row
  **−191.9 mm** in the row-composition run, and a *stationary* arm was in 3 of the 5 failing pairs.
- And its cost is structural: planning a row group against a **parked** fleet is **2.4-5.4× cheaper**
  than against four arms holding hovers over the sheet. The gap-fill pass is expensive *precisely
  because* the earlier stages leave arms holding poses over the paper. The two halves of the plan are
  coupled through the held poses, and the plan does not model that coupling.

**Recommendation for §6.** Keep the 1-2-1 leaders — the geometry says they are 99.2 % independent and
100 % independent beyond 0.6 m of pen separation, which is a strong result. Make first-class the three
things measured to matter: the **partner standoff** to the follower's pose-invariant links, **exact
swept rooms** with a sphere broad phase, and **home-first** (an arm that has finished goes home rather
than holding over the sheet, unless holding is measured cheaper). Adopt **row bands at ±605 mm with a
25 mm setback** as the virtual walls, and evaluate the **scheme-C option**: all six arms fully
asynchronous inside their own blocks over 57 % of the canvas with no pairwise certificate at all, then
one conducted pass for the remaining 43 %. Finally, state the target against 99.988 % / 402.7 s, and
be explicit that beating 209.9 s means going back to joint conducting.


## E. Time to first motion and incremental dispatch

**Verdict: the plan names the requirement and has no mechanism for it. This is the section that
needs the most new design.**

The measured facts: under the already-built leader/follower pattern, time to first motion is
**0.18-0.34 s** once a stage is planned, and the plan wall is **1 406 s** for the logo. So TTFM is
not a planning-speed problem at all; it is a *commit* problem — nothing may move until the thing
that certifies it exists.

**E1. What actually delivers "moving in seconds" is a lease protocol, not faster planning.**
Give each arm an exclusive, geometrically-defined volume (a "lease") that no other arm's lease
overlaps by more than the pair margin. An arm may be dispatched into its lease the moment its
own chunk is planned, with no knowledge of any other arm's plan, because the lease is what makes
the pairwise certificate unnecessary. Leases are granted and revoked by the coordinator; a grant
is checked against the other arms' *current poses and held leases*, which is cheap. This is the
only architecture I know that gives seconds-to-first-motion with a hard safety argument and no
joint planning. The plan's "virtual walls" is this idea without the protocol; make it the
protocol.

**E2. The safety condition the plan is missing: every dispatched chunk must end in a certified
holdable pose.** If an arm is committed to a motion whose continuation is unplanned, it can
arrive somewhere from which every continuation is refused. The conductor already has this as the
"rest-suffix rule", and `docs/CONCURRENCY.md` measures what it costs: 116.6 s of the 148 s of
pause is arms held out of a finished pose. So: every chunk ends at a pose certified to be
holdable indefinitely against every other lease; the arm may only be handed the next chunk if it
starts from that pose. Without this rule, incremental dispatch is not safe; with it, it is, and
you have priced it.

**E3. Goal chaining on the JTC works, but only for transits — and not for drawing.** *(verified in
source, B3.)* A new `FollowJointTrajectory` goal **replaces** the active one rather than appending,
but the replacement re-captures the current state and interpolates from it to the new point 0, so
consecutive goals bridge without a stop **provided each new trajectory's first point continues from
where the arm actually is**. That is a usable chunked-dispatch primitive for **position-mode
transits**. It is no use for drawing, because drawing does not go through the JTC at all: it goes
through the impedance controller, whose reference is a pose stream paced by arc length. So build
both: chunked JTC goals for transits, and the streaming path governor of Pete's stage 4 — `s`
advancing at the planned rate, slowed by tracking error, never stopping mid-stroke — for ink.
Note the switch between them costs 1-4 s and can wedge for minutes (B8), so chunk boundaries should
not straddle a controller switch.

**E4. The plan-file contract must become an append-only journal.** Concretely: a header
(hashes, arm list, leases), then a sequence of signed chunks, each `{arm, lease id, q(s) samples,
kind, entry pose, exit pose, holdable-exit certificate}`. The operator tails it. The dev machine
may append; it may never rewrite. That is a small change to `docs/ARIS2_CONTRACTS.md` §1, not a
new format — the CSV is already row-oriented with `q1..q7` on every row. Note what exists today:
`aris_sixarm/program_schema.py` is `SCHEMA_VERSION = 1`, a whole-programme artefact in which
**unknown keys raise** — good strictness, but it means the journal is a version bump with an explicit
forward-compatibility rule, not an added field.

**E5. What to tell the user while it warms up.** The honest UX is: arms start on the first
lease within a few seconds, and the coverage number grows while they draw. The GUI must show the
gap list *provisionally* and update it — see J.

---

## F. Long continuous strokes

**Verdict: the local planner is right; the two sentences around it are wrong.**

**F1. "Split at the cell boundary" is the wrong split rule, and it is the measured cause of the
corpus's worst rows.** The natural split is `s*`, the certified extent of the band — the planner
already computes it (`stroke_api.plan_stroke` returns `{split, s*, reason}`; the memory records
*"s_star now = certified extent not optimistic band reach"*). Splitting at a cell boundary cuts
strokes that did not need cutting and fails to cut where the band actually dies. The staged
planner's residual on `hatch` (83.1 %) and `spiral` (84.4 %) is recorded as *"the pattern's
SINGLE-CELL OFFERS / placement ceiling"* — i.e. exactly the cell-boundary decomposition. If the
new design keeps it, it inherits the 83/84 %.

**F2. What physically limits stroke length.** In order of how often they bind, from the recorded
runs: (i) the certified band going empty in `(s, q7, branch)` — joint margin < 0.15 rad or
σ_min < 0.10 — at h = 0.970 with the lateral tool this is the **outer rim**, since there is no inner
hole (D3: `r_GO` = [0.01, 0.748] m); (ii) the reach radius itself, 748 mm certified against 806 mm
reachable; (iii) with the **lateral** tool the yaw≡q7 degeneracy is broken, so there is a
real extra DOF (`phi`) and the fiber is `s × phi × q7 × branch` — this is why `lateral.py` exists
and why per-stroke cost went to 80-280 ms; (iv) self-collision and the frame. Note the recorded
trap: with a *vertical inline* pen, `(yaw, q7)` aliases and a ±1-index DP window produces **false
disconnects** with period 3. Any new lattice must not re-introduce it.

**F3. "Few lifts" is not won by the stroke planner, it is won by the sequencer, and the plan's
"cheap heuristic" gives back the win.** Measured: pen-up time was **33.5 % branch flips, 49.6 %
tall legs, 16.9 % honest** travel; arm 71's stage-A pen-up went 70.5 s → 22.0 s and flips 4 → 0
when three things were fixed — `hover_solve` choosing the *nearest* comfortable pose rather than
the first (a 5 rad branch flip where a 0.26 rad pose existed), a Viterbi over IK sheets across
the whole tour (`allocate.chain_sheets`) instead of per-stroke sheet choice, and a router that
tries the 8 cm rung before the 42 cm depot via. A greedy/flood-fill sequencer over paper distance
reproduces the "before" column. *Fix:* sequence over `(stroke, direction, entry-fiber variant)`
with real transit **time** as the cost, and pick the IK sheet with a chain-wide Viterbi. That is
the minimum that keeps "long continuous strokes" true.

**F3b. An RRT transit is not executable by today's drawing path, and that is a sequencing
dependency the plan hides.** The impedance executor walks consecutive non-draw rows as a **Cartesian**
path with orientation held per row; the day-1 record states that *"large joint reconfiguration in a
transit is where the deployed law fails in SIL ⇒ single-branch plans"*. A joint-space RRT path that
changes IK branch cannot be followed by that law at all. So either transits run in position mode on
the JTC — which means a controller switch per transit at 1-4 s each — or stage 1 of the controller
redesign (joint reference + slew guard) lands first. The plan lists stage 1 as a prerequisite for
*concurrent drawing*; it is also a prerequisite for **RRT transits**, which is a much earlier
milestone. Say so, and price the switch-per-transit alternative: on the day-1 word there are 12
inter-stroke transits, so switching costs 24-96 s of the run.

**F4. How to measure the sequencing heuristic.** Four numbers per drawing, already implemented:
pen-up seconds per ink metre; number of sheet/branch flips; Σ‖q_exit − q_entry,next‖∞; makespan.
Run them on the corpus, not on one picture; and price the sequencer with the *same* cost model
the allocator uses (`allocate.cost_model` exists precisely because the two used to disagree and
the balancer balanced loads the sequencer then moved).

---

## G. Calibration (section 7)

**Verdict: the identifiability story is right, the accuracy claim is not supported, and the
procedure attacks the wrong error first.**

### G1. What is and is not observable

Plane touches give one scalar per touch. Base **x, y and yaw are exactly unobservable** from
plane touches — the plane is invariant under in-plane translation and rotation — so of the 6
base DOF, only roll, pitch and height come out. The plan says this and it is correct.

The tip offset from plane touches is a linear problem: the tip's height is affine in the tip
vector `p`, and the design matrix's rows are the paper normal expressed in the **hand** frame.
Consequences the plan does not state: touches at a **constant tool orientation give rank 1** —
twenty of them are one equation; **rotating about the plane normal contributes nothing** (a
lateral offset then moves in-plane, which the plane cannot see); only **leaning off vertical**
separates the 86 mm lateral component from the 46 mm axial one, and it does so with sensitivity
`sin(lean)`. At 15° that is 0.259, so 0.3 mm of touch noise is 1.16 mm of lateral tip error per
touch. This is not hypothetical — `scripts/touchdown_calibrate.py` already implements exactly
this fit with a condition-number refusal (`MIN_COND = 20`), and its docstring records that the
current tool numbers (`PEN_LAT_HOLDER = 0.0860369`, `PEN_EXT_HOLDER = 0.0460262`) are
*"USER-SPECIFIED, from a photograph … never measured against a robot."* Reuse it.

Cone fiducials are a much better instrument: each touch is **3 equations, not 1**, and they also
supply the x, y, yaw the plane cannot.

### G2. The accuracy budget, and why "sub-mm" is not available from touches alone

| term | size | can calibration remove it? |
|---|---|---|
| FR3 repeatability | ~0.1 mm | no (noise floor) |
| joint-5 static offset, measured 9.3 ± 1.0 mrad on lab FR3s | **2.3-2.8 mm at the tip** (lever ≈ 0.25-0.30 m) | only as a joint-offset parameter, and see below |
| FR3 absolute kinematic error, rest of the chain | ~0.5-1 mm | partly, with joint-offset ID |
| contact-detection bias (position-lag latch) | 1-3 mm systematic, 0.2-0.5 mm repeatable | yes, if the detector is characterised |
| paper compliance + waviness | 0.5-3 mm | only by mapping, not by a plane fit |
| fiducial jig placement by tape over 4.17 m | 2-3 mm absolute | **irrelevant, see G4** |
| pen tip geometry (photo-derived today) | unknown, ≥ several mm | yes |

The joint-5 number is the one that decides the section. It is recorded as a **steady-state error
under the position law**, *absent in SIL* — i.e. it is a control/friction/gravity effect, not a
kinematic constant. So it is **different under the impedance law**, and a calibration taken in
position mode does not transfer to drawing in impedance mode. That is a systematic, pose-dependent
2-3 mm offset between where you calibrate and where you draw, and no amount of touching fixes it.

**Conclusion: a touch-only calibration cannot be "bulletproof" to sub-mm. The only measurement
that closes every loop — kinematics, joint offsets, controller steady-state error, tool geometry,
pen compliance, paper — is a mark the arm actually draws, measured externally.**

### G3. Recommended routine (replaces §7 steps 1-4)

Per arm, ~12 minutes; all six arms plus measurement in about 1.5 h.

1. **Rigid probe, not the pen.** Put a hardened pin of the same nominal geometry in the holder
   for the geometric work. Then the pen's own protrusion is one scalar measured against a
   reference block, not a 3-vector re-identified every time it wears.
   *(Pen compliance is not currently a problem and it is worth knowing why: with
   `frames.PEN_GRAPHITE_HOLDER = 0.020 m` of unsupported graphite, lateral stiffness
   `3EI/L³` is ~6.6×10⁵ N/m at 7 mm diameter and ~4.4×10³ N/m at 2 mm — both rigid at 0.3 N of
   drag. It becomes fatal if the free length grows: at 2 mm × 100 mm it is ~35 N/m, i.e. 8.6 mm
   of deflection at 0.3 N. Put the free length in the config and gate on it.)*
2. **Point fiducials, touched from many orientations.** Three (better four) cone or ball
   fiducials rigidly fixed to the table, each touched from ≥ 5 arm configurations spanning ≥ 25°
   of lean and both elbow branches. 3 fiducials × 5 poses × 3 equations = 45 equations for base
   pose (6) + tip (3) + the identifiable joint offsets (joints 2-6; joint 1 aliases base yaw and
   joint 7 aliases tool rotation). **Solve all six arms jointly**, with the fiducial positions as
   shared unknowns — see G4.
3. **Paper map, not a paper plane.** 25-40 touches per arm cell on a grid, at **two press
   forces**, and extrapolate to zero force. This gets you three things a 9-point plane fit does
   not: the contact bias cancelled (it is `F/k` with a known `F`), the local compliance measured,
   and the paper's waviness mapped. Budget: ~4 s per touch → 2-3 minutes per arm. Feed the map
   into the stroke z, not a single plane. *Why it matters quantitatively:* the press band is
   0.7-1.0 N and the depth clamp DMAX is 12 mm, so a 3 mm wave spends a quarter of the depth
   budget and moves the tone across its whole band.
4. **Verification by drawing, and that is the acceptance test.** Each arm draws a fixed test
   pattern: a box + diagonals at its cell edges, plus a registration cross at each shared
   location in every overlap band with a neighbour. Photograph each overlap patch (a 100 mm field
   at 0.05 mm/px is a phone photo with a scale bar in frame) and measure the offset between the
   two arms' crosses. **That number — inter-arm mark-to-mark offset — is the one Pete is
   complaining about and the only one worth a threshold.**
5. **Residual fit.** If step 4 shows a systematic pattern, fit a per-arm 2-D correction in the
   paper plane (affine first, then quadratic) on top of the kinematic calibration. This is where
   the joint-5/impedance-mode error goes, and it is the difference between 2-3 mm and 0.3 mm.

### G4. Fiducial positions do not need to be measured accurately, and the plan should say so

What the installation needs is that the six arms agree with **each other**, not with the table.
If all six register to the same physical fiducials, the fiducial coordinates *define* the table
frame; a tape-measure error in them is a global similarity transform of the whole drawing, which
nobody can see. So: solve the fiducial positions as free parameters in a joint bundle adjustment
over all six arms, fix the gauge by convention (fiducial 1 at the origin, fiducial 2 on the x
axis), and use the tape numbers only as a starting guess and a blunder check. Then put the
fiducials **in the overlap bands between neighbouring arms**, where the stitching error is, not
in the middle of each arm's own cell.

### G5. Wear: measure it before designing around it

The plan treats graphite wear as a headline risk. Order of magnitude: length loss =
deposited volume / tip cross-section. A 2 mm wide, 5 µm thick deposit is ~0.01 mm³ per metre; over
a 7 mm stick (38.5 mm²) that is 2.6×10⁻⁴ mm/m — 0.08 mm over 300 m of line, negligible. Over a
sharpened ~1 mm-radius tip (3.1 mm²) it is ~3×10⁻³ mm/m, ~1 mm over 300 m, plus a larger blunting
transient. **This is a ten-minute experiment: draw 100 m, re-touch-off, read the difference.**
Do it before building a wear-management feature.

**What the tolerance on pen length actually is, which the plan never states.** Tone is force, and
force is depth × stiffness. The recorded numbers are: band **0.7-1.0 N**, one intensity step
**0.04 N**, effective stiffness **0.40 × commanded** with commanded `K_z = 800` → **320 N/m**, and
*"press ~10 mm for 0.7-1.0 N"*. Those last two are inconsistent by about 4× (320 N/m × 10 mm = 3.2 N,
not 0.85 N), so bracket it:

| | at 320 N/m | at the ~85 N/m the 10 mm press implies |
|---|---:|---:|
| depth for the whole tone band (0.3 N) | **0.94 mm** | 3.5 mm |
| depth per intensity step (0.04 N) | **0.125 mm** | 0.47 mm |
| tone error from 1 mm of pen-length or plane error | **0.32 N — the whole band** | 0.085 N ≈ 2 steps |

So **pen length and paper height must be right to a few tenths of a millimetre (at 320 N/m) or about
a millimetre (at 85 N/m) for tone to mean anything**, and one quasi-static push-and-measure per arm
settles which. That is the number the calibration section should be designed against, and it is a
far tighter requirement than the geometric stitching one.

What is certain either way is that the *runtime* answer is the right one and it is currently
switched off: `config/site.json` ships `RTFF_CONTACT_DESCEND: "0"`, i.e. the executor flies the
baked z instead of feeling for the paper, and the executor's ladder gate is **skipped for
inverted arms** — which is a complete explanation for "arms sometimes run into the table, or
hover and do not draw". **Turning contact descent on for the inverted arms is a bigger win than
any offline calibration, and it should be item 1 of section 7, not item 4.**

### G6. What "bulletproof" must mean operationally

- One command per arm; ≤ 15 minutes; writes exactly one dated file; never writes a constant.
- Printed residuals with hard reject thresholds: fiducial fit RMS < 0.4 mm and max < 1.0 mm;
  design-matrix condition number reported and refused below a floor; paper map RMS < 0.5 mm and
  peak-to-peak reported; **inter-arm mark offset < 1.0 mm** in every overlap band.
- Refuse to draw if: any threshold is missed; the calibration is older than N jobs or M hours;
  the pre-job pen touch-off differs from the stored length by more than 2 mm (something moved or
  broke); the measured paper height differs from the map by more than 2 mm; the arm identity in
  `site.json` is unconfirmed (today it is: *"which arm ids are mounted in the two middle positions
  is not known"*).
- And record the payoff so calibration is not treated as pure overhead: **30 mm of the 50 mm
  inter-arm gate is `CALIB_M`, an admitted allowance for nobody having measured where the bases are**
  (`coordination.py:205-207`; `mounts.MOUNTS.calib = 0.03` inflates every neighbour's body column for
  the same reason). Be precise about what retiring it buys, because the easy version of this claim is
  out of date: the pair gate has *already* been cut from 80 mm to 50 mm (2026-09-09), and the
  measured margin/makespan curve **flattens below 50 mm** (phase-1 makespan 47.7 s at 65 mm, 47.3 s
  at 50, 47.1 s at 35). So a survey is **not** a makespan lever any more. What it buys is
  **feasibility and robustness**: at 80 mm only **3 of 24** priority orders were feasible at all, and
  at h = 0.970 the remaining 2.22 % of draw+hover cells that sit under 50 mm against a parked partner
  — the binding one at **+1.4 mm** — are exactly the cells a 30 mm survey allowance is paying for.
  Thirty millimetres off five neighbours' body columns is also the largest single thing a survey does
  for reach (`scripts/asbuilt_layout.py` docstring says so, and `feasible_workspace.rig(..., calib=)`
  already takes the number). Do the survey for the feasibility, not for the clock.

### G7. Two interface facts the calibration section must handle and does not

- `docs/ARIS2_CONTRACTS.md` §4: the robot is configured with `setEE` = the **nominal** tip, so
  `o_t_ee` *is* the pen tip and the impedance controller runs with `tool_tip_offset = [0,0,0]`
  and a nominal-tip Jacobian. A recalibrated tip therefore has to be pushed into the robot
  (`setEE`, a Desk-side change per the fr3drivers audit) **or** compensated in the plan — and if
  the two disagree, the controller's stiffness axes and depth clamp are wrong even when the
  geometry is right. Decide which one owns the tip, write it down, and gate on the two agreeing.
- The tool has a lean (`frames.PEN_LEAN_HOLDER = 0.401 rad = 23°`), so "press along the pen axis"
  and "descend in world z" differ by ~8 %. Small, but it must appear in the touch model.

---

## H. Drawing-agnostic guarantee and the corpus

**Verdict: the invariant set in the plan is one line where it needs to be seven, and the corpus
is the wrong shape.**

**H1. The invariants that must be enforced, all of which exist today and were each bought with a
defect:**
1. *Conservation:* every millimetre of every input line ends in exactly one of {drawn, listed in
   the gap list with a reason code and the refusing gate}; `flown + missing == picture` asserted
   in the top-level run. (Bought by: a certified programme silently dropping 1.9 m.)
2. *Coverage counts pen-down only.* (Bought by: a conduct log printing the allocator's 84.2 %
   while the schedule JSON said 1.23 %.)
3. *Certified-or-split:* a stroke planner call returns a validated plan or a structured
   `{split s*, reason}`; it never raises and never returns garbage.
4. *No uncertified motion of any class.* Pen-up transits, entry/exit corridors, parking, holds
   and go-home are motions. The paper was not an obstacle for pen-up moves and arms flew 253 mm
   **under** the canvas, held there for 0.75 s by a scheduled pause.
5. *Gate consistency:* every producer is strictly tighter than the corresponding checker by a
   slack **derived** from the checker's own sampling constants, with a test that re-derives it;
   and the checker shares no code with the producer.
6. *Sampling-rate independence:* every gate auto-refines until the verdict stops moving.
   (A frame gate that read 49.1 mm at sub=2 and 53.5 mm at sub=8 refused 8 valid configurations.)
7. *Determinism:* identical inputs give a bit-identical plan file, and replay reproduces it.
   No wall-clock budgets anywhere in the certified path.

**H2. What the 83 % / 84 % rows actually say — and a correction to §2's table.** The final bench
run (`docs/V2_STAGED.md:3505-3542`, `--split-m 0.15 --partner-standoff 0.09 --row-compose serial
--tilt-max-deg 15`):

| picture | flown | missing, by reason | makespan | `all_ok` |
|---|---:|---|---:|---|
| hatch | 83.14 % | `no_drawer` **5.074 m** | 380.2 s | **no** |
| scatter | 98.96 % | `plan_refused` 0.087 m | 156.7 s | **YES** |
| starburst | 96.65 % | `no_drawer` 0.570, `plan_refused` 0.063 | 356.2 s | **no** |
| spiral | 84.40 % | `no_drawer` **2.174 m** | 275.5 s | **no** |
| duotone | 97.35 % | `no_drawer` 0.340 m | 238.0 s | **no** |

So the plan's §2 line *"bench corpus … scatter 99 %, starburst 97 %, duotone 97 %, hatch 83 %,
spiral 84 % **certified**"* is wrong: **only scatter is certified.** `starburst` fails on the hold
gate (a barrier pose at +141.4 mm against `MARGIN_GATE` 150 mm), not on ink. Fix the baseline table
before anything is compared against it.

`no_drawer` is defined in `staged.py:4552-4553` as *"the DP left the span UNCOVERED: after the
refusal loop's bans, no (stage, arm) cell could take it"*, and the mechanism is named at
`docs/V2_STAGED.md:3295-3299`: *"A stretch offered to ONE (stage, arm) cell has no alternative when
that cell refuses: the leader/follower region split at `--split-m 0.15` is what narrows the offer,
and v19's conductor had all six arms with no stage or region partition at all."* `spiral` is a
single 13.9 m stroke; `hatch` is 43 long parallel strokes of 30.1 m. **Both are failure modes of the
stage × region decomposition, not of reach**, and the new plan proposes the same decomposition with
a *worse* cut rule (F1). Designing them out means: offer a stretch to the union of cells that can
take it, split at `s*` per arm, and treat the handoff as normal control flow — which is also what
restores the fallback the partition removes.

**H3. Corpus composition.** The five bench drawings are all "pictures". Add generators, because
the acceptance test must be adversarial, not representative:
- random polylines at three densities (exists);
- one stroke longer than the canvas diagonal (spiral) and a family of long parallel strokes at
  0°/30°/45°/90° (hatch), **deliberately crossing every cell and row boundary**;
- strokes that pass directly under a base, along the seam, and across the row boundaries at
  y = ±605 mm (the wall lines and the leader-diagonal conflict band);
- strokes at the canvas rim and in the corners;
- sub-minimum-length fragments and closed loops;
- multi-ink drawings (the staged model still has no ink dimension — `duotone`'s two colours went
  through as one);
- the same drawing at three scales and three placements.
Each run asserts H1 and reports coverage + gap-by-reason. Pin the numbers; a regression is a
failing test, not a conversation.

---

## I. Milestones (section 9)

**Verdict: wrong order, and the durations are missing. As listed, milestone 2 cannot be done.**

Milestone 2 ("calibration end to end on one arm") needs: a way to move the arm to each touch pose
(position mode, i.e. the JTC path and the controller switch — milestone 4's territory), a contact
detector, an e-stop/hold story, and certified transits between touch poses (milestone 3's
territory). It is listed second and depends on the two after it.

**Recommended order.**

0. **Measure before rebuilding (3-5 days).** Profile the real workload; benchmark the candidate
   kernels on *this* machine against *these* gates; decide the kernel on numbers. The plan's
   diagnosis is currently unprofiled and the project's own records contradict it (see C).
1. **Robot layer on the two mounted arms (2-3 weeks).** The verbs plus the ones the plan omits
   (B): hold, stop, recover, home, tool/limits, identity. Bring up position-mode motion through
   the JTC with an explicit start-pose check, and the loopback SIL. **Confirm which physical arm
   is in which slot** — `config/site.json` still says `"mounted": null`.
2. **Calibration on one arm, verified by a drawn mark (2-3 weeks incl. two hardware sessions).**
   Deliverable is a number: inter-arm mark offset on the two mounted arms.
3. **Kernel + single-arm stroke planner + sequencer behind the EXISTING plan-file contract
   (3-4 weeks).** A/B every corpus drawing against legacy as a subprocess oracle. Ship nothing
   that loses coverage.
4. **Two arms concurrent, with leases and holdable exits (3-4 weeks), SIL first.**
5. **Six arms + GUI (3-4 weeks).** Delete legacy only when the corpus numbers are ≥ the recorded
   ones *and* the hardware has drawn with the new stack.

Honest total for one implementing engineer/agent with hardware access: **4-5 months**, of which
perhaps 3 weeks is the kernel the plan is organised around. The rest is re-earning the defect
list in `docs/`.

**And that is the number to attack, not accept.** Under R0 — delete `staged.py`, keep the 48 % that is
kernel, keep `scene_check` as the independent judge — milestone 3 stops being "write a single-arm
planner" and becomes "put the new kernel under the existing one and prove bit-identical verdicts",
which is weeks rather than months. On that route the honest total is **6-8 weeks**, and the whole
difference is whether the 144 recorded defect-fixes are inherited or re-earned. Two bounded jobs
should come before either route, because they move the baseline the rebuild will be measured against
(2b.7): the `starburst` stage-B hold (+141.4 mm against a 150 mm gate — the only thing between four of
the five corpus pictures and a certificate) and the reverse-order search that was closed in code and
never measured.

**Smallest thing that proves the concept on the two mounted arms** (do this first, it is about a
week): one arm, one long stroke, planned by the new kernel + local planner, calibrated by the new
routine, drawn on paper; then the *same* stroke drawn by the second arm from the other side with a
50 mm overlap; measure the joint. If the two halves line up to under a millimetre and the ink is
continuous, the kernel, the local planner, the calibration and the contract are all proven at
once, and if they do not, the failure is localised.

---

## J. GUI

**Verdict: the feature list is right for a demo and missing everything safety-relevant.**

Keep: load a drawing; show the plan; show live arm state; run calibration and show residuals.

Add, in order of importance:
1. **Stop and hold.** A big stop, and per-arm hold/resume, reachable in one click from every
   screen, that works when the plan view is broken. Today's hold is a control-file token plus a
   25 s wait for proof of pen-up; that latency must be shown, not hidden.
2. **Arm identity and mode.** Which physical arm is in which slot, whether that is *confirmed*,
   its controller, its robot mode, its error/reflex state, and whether it has been recovered.
   Arm identity is genuinely unknown today and a mis-identified arm is a crash.
3. **The certificate of what is about to run.** Min inter-arm / frame / paper / self clearance,
   which gate was tightest, the calibration hash and its age, the geometry hash. Refuse the run
   button when any of them is stale, and say which.
4. **The gap list before you start.** What will not be drawn and why, by reason code, on the
   drawing. This is the honesty mechanism from H1 made visible, and it is what stops the "why are
   there so many gaps in the lines?" conversation happening after the ink is on the paper.
5. **Pen and paper state.** Measured paper height vs the map, current press force and tone, last
   touch-off, DMAX headroom remaining.
6. **What is actually happening now**: per-arm current stroke, progress, leases held, and for a
   waiting arm, *who it is waiting for*. The conductor already attributes blocking; show it.
7. A single "what changed since the plan was made" banner.

No planning in the GUI is right. One bridge to the operator is right. Add: the GUI must be able
to render a plan file with no operator connection at all, so it is usable for review off-site.


---

## 2b. Addendum after reading STAGED_LESSONS_2026-09-29.md

`docs/STAGED_LESSONS_2026-09-29.md` (144 items, L1-L144, each with a citation) landed after sections
A-J were written. It corroborates them and changes or sharpens seven things. Nothing above is
retracted; the items below supersede where they conflict.

### 2b.1 The biggest change: the rebuild decision itself (question A, plan §3)

§4 of the lessons file is a census: **~48 % of the package is kernel that survives essentially
verbatim** — kinematics and rig model ~14 % (`frames ik rig_final mounts fleet layout link_spheres
envelope metrics system_model`), collision/gates/certification ~8.5 % (`coordination scene_check
exact_room frozen selfcoll validate`), single-arm stroke planning ~10 % (`planner pwl smooth pacing
stroke_api menu tilt lateral`), pen-up routing ~6 % (`paper transit`), atlas ~2 %, planfile/export
~7.5 %. The pattern-specific part is **~16 %** (`staged traces idle progress`), and
**`staged.py` is a leaf that nothing imports**.

So the choice in plan §3 is a false one. The available move is not "rebuild versus clean in place";
it is **delete the leaf and keep the kernel**: freeze `staged.py`, lift the reusable parts out of
`allocate.py` / `writing.py` (`arm_program`, `hover_solve`, `held_pose_ok`, `hold_candidates`,
`self_pace_beat`, `sequence_arm`, `chain_sheets` are not staged-specific), and build the new
orchestration — leases, incremental dispatch, walls — on top of what already certifies. That keeps
144 defect-fixes and their tests instead of re-earning them, and it is the difference between a
three-week job and a four-month one.

**This also settles "legacy as oracle" (A4) more cheaply than I proposed.** L131/L132 are the load
bearing pair: `scene_check` is *an independent second derivation that shares no code with the
planner*, with capsule radii, column bands and pen radii deliberately restated and pinned together by
a test — and **every bug in the 144-item list was caught because the judge disagreed with the
producer**. The oracle you need is not frozen legacy; it is the second derivation, and it must be
kept, not rebuilt alongside the first.

### 2b.2 New and serious: acceleration (question B/F, plan §6 and the planfile contract)

L44 is the most dangerous item in the file and it is absent from the plan. *Acceleration depends on
the rate you measure it at*: the same trajectory reads **37.70 rad/s² at 48 Hz and 1 422 rad/s²
resampled to 1 kHz**, against a driver gate of **10 rad/s²**. `pacing.py` bounds velocity and nothing
else. And time-scaling does not fix it — **at a corner the true acceleration is impulsive however
slowly you fly it.**

Consequences for the rebuild:
- The plan-file contract must carry **C² (or at least acceleration- and jerk-bounded) joint paths**,
  not piecewise-linear waypoints. The `franka-station-sim` pipeline already shows the shape of the
  answer (TOPPRA retime → Bezier wire → gate validator), and its driver gate *refuses* the quintic
  and accepts the cubic — so this is a live constraint, not a theoretical one.
- L46: **one re-time rate for the whole fleet** (`execute.Governor`, one scalar). Per-arm retiming
  voids every pairwise certificate. Add this to the verb list; it is a coordination invariant.
- L45 restates my pacing finding exactly and more sharply: **the velocity certificate holds at `t_s`
  and no other timing**, and the pacer hits `QD_FRAC = 0.30` exactly, so *"anything near 100 % is a
  bug upstream"*.

### 2b.3 Corpus: the lost ink is mostly the rim, deliberately (question H)

L143: `traces.BLOCK = (0.16, 0.00, 1.64, 3.62)` withholds a **0.16 m rim** of the canvas from every
arm and is **the single largest source of lost ink in the corpus — 6.41 m** — and it is *"not in the
rebuild plan's carry-over list"*. That corroborates D4 from the other direction: the rim is genuinely
unreachable at h = 0.970, and `BLOCK` is the model of it.

So the H2 attribution needs splitting: of the corpus `no_drawer` metres, the larger part is **ink
placed in a rim no arm can reach** (a *placement* problem, fixed by R8c), and the smaller part is
**single-cell offers** (L72 puts spiral's share at 1.5 m and calls it *"the real planner ceiling"*).
Both must be reported separately in the gap account, or the planner gets blamed for the canvas.

Two more corpus items: **L76 — the 15° tilt cone is the single biggest coverage lever and it is not
the default** (CSAIL 95.63 → 100.00 %, duotone 85.42 → 97.23 %); the quoted bench percentages are at
tilt 15 while the CLI default is 0, so the corpus and the shipped default disagree. And **L119 — the
staged model has no ink dimension**, so a two-colour picture is *wrong rather than slow*; an ink is a
constraint on which arm may draw a stroke at all and belongs in the capability map. That is a
correctness hole, not a feature gap, and it should be in the plan's §6 input contract.

### 2b.4 Incremental dispatch: the lesson that kills the naive version (question E)

L112 (`staged.py:2946-2969`) is the one to put in front of whoever implements §6: **"it happens
first" is not a defence against a claim quantified over all pairs.** Measured on `bench/scatter`: a
follower's clear-out was *routed* with `rooms=None` while its *destination* was chosen with the
leaders' rooms installed; the leg dips to **−53.1 mm 0.7 s after leaving**, and the offending pose
pair is **7.00 s apart on the two clocks** (time-aligned they are +169.5 mm apart). Any scheme that
dispatches a leader early and lets a follower reason about "where the leader will be by then" hits
this immediately.

L140 is the hardware half of the same point and it is stronger than I had it: **clearance is not
monotone in the skew** (+1.00/+1.25 s bottoms at 35.8 mm), so *"two seconds is safe; one is not"*,
and **a cross-process fleet clock must exist before six arms run concurrently.**

L44-L46 add the third half: a chunked dispatch that re-times anything per arm voids the certificate
it was dispatched under.

So the lease protocol of E1 is not optional polish — it is the only formulation in which "dispatch
before the whole plan exists" has a proof, because a lease is a claim quantified over all pairs *by
construction* rather than over a schedule nobody has yet written.

### 2b.5 Calibration: one consequence I under-weighted (question G)

L141: the tool tip is **user-specified, never gate-validated, and has been re-specified three times**;
the thinnest margin in the shipped programme is **paper-chain 20.5 mm against a 20 mm gate**, and
*"a touchdown calibration that lowers the tip further eats this first."*

That inverts the order of operations in plan §7. A successful calibration that finds the tip deeper
than modelled **invalidates every certified programme by 0.5 mm of headroom**, and the pipeline has
to be able to say so and re-plan. So: the tip is part of the geometry hash (A3), the paper-chain
headroom must be reported next to the calibration residual, and the acceptance test for calibration
includes *re-certifying* an existing programme at the new tip, not just measuring the tip.

L135/L136 confirm the G6 rewrite and are worth quoting to Pete directly: gate constants do not move,
`CALIB_M` stays at 30 mm because it is *"a real uncertainty about where the bases ARE"*, and **the
inter-arm minimum is set by the gate, not the geometry** (v19 lands at 50.32 mm against a 50 mm gate),
so *"a base survey is what buys discretionary air back."*

### 2b.6 Things that credit the plan, and one that answers an open question

- **L5 vindicates the plan's RRT emphasis with a number.** A C-space RRT tier is *mandatory*: "the
  arm can FLY there" goes **84.05 % → 95.34 %** and feasible area **5.568 → 6.316 m²**, and of 66
  ladder-exhausted crossings the planner flies 65. It currently costs **1.90 s per plan** — which is
  exactly the ≤ 0.1 s target, 19× away, and the first place a fast kernel pays. That is the honest
  justification for the plan's §6 target, better than the one the plan gives.
- **L142 partly answers plan §10's "which collision backend".** `cc_experiment` was already evaluated
  and rejected (no Python bindings, ABI, Drake fork, no licence) and kept as an A/B oracle. Record the
  decision rather than reopening it.
- **L6, L7, L23, L33 are all "do not turn that knob" results** — lower hovers recover 0 of 8 cells;
  lifting can make clearance *worse* (63.7 → 21.9 mm for an outboard inverted arm); park search
  saturates on a 97.8-98.9 mm plateau that 7.5× the grid cannot move; and *no routing fixes a pose*.
  A rebuild without them will spend weeks on each.

### 2b.7 Two items the lessons file leaves open, which the plan should own

1. **L51 — quality and speed are the same knob, and the fix is unbuilt.** Tightening
   `writing.densify` from 0.762 mm to 0.2 mm of chord error took one stroke **66.9 → 552.0 s (8×)**,
   because that stretch is a near-null-space wrist reconfiguration. The named fix — let the stroke
   planner price the null-space reconfiguration it chooses — is exactly the local planner the rebuild
   is writing, so it should be a requirement on it, not a backlog item.
2. **L60/L61 and `starburst`'s stage-B hold (+141.4 mm against a 150 mm gate)** are the only thing
   between four of the five corpus pictures and a certificate, and the reverse-order search closed in
   code at `V2_STAGED.md:3302` was **never measured**. Two small, bounded jobs that would move the
   baseline the rebuild is measured against; do them before the rebuild, not after.

## 3. Revised plan — the changes to make to the plan document

Ordered by how much they change the outcome.

**R0. Replace §3's decision.** Not "rebuild, do not clean in place" but **"delete the leaf, keep the
kernel"**: `staged.py` is a leaf nothing imports and ~48 % of the package is kernel that survives
verbatim (2b.1). Freeze/delete `staged.py`, lift the non-staged-specific parts out of `allocate.py`
and `writing.py`, keep `scene_check` as the independent judge it already is, and build the new
orchestration on top. This is the single change that most reduces the cost and risk of everything
else in the plan.

**R0b. Put acceleration and jerk in the plan-file contract.** *(2b.2)* The same trajectory reads
37.7 rad/s² at 48 Hz and **1 422 rad/s² at 1 kHz** against a **10 rad/s²** driver gate, and
time-scaling cannot fix a corner. The contract must carry acceleration-and-jerk-bounded joint paths
(TOPPRA → spline → gate-validate, as `franka-station-sim` already does), and re-timing must be
**one fleet-wide scalar**, never per arm, or every pairwise certificate is void.

**R1. Delete "why it is slow (diagnosis, not yet profiled)" and replace it with the measurement.**
Section 2's diagnosis is contradicted by this project's own latency record: the recorded hot spots
were allocation balance (93.4 % of one allocation: 931 s of 997 s), conductor image building
(187 s → 1.8 s by broad phase + memo + pooling, in numpy) and `scene_check` (38 % of the conduct),
and *"the stage the log blamed was already 1 % of the conduct"*. Collision checking is already
batched and broadcast over whole timelines. A compiled kernel is worth having, but organising the
whole rebuild around it is organising it around the wrong number.

**R2. Say that Pete's leader/follower pattern is already built and measured, and state its
numbers.** `traces.leader_follower_pattern` implements leaders 13/71/2, followers 17/31/97, roles
swapped, then an all-six pass; `docs/V2_STAGED.md` §22-§31 is its record. Final measured result
(lf10): **CSAIL 99.988 % flown, certified, makespan 402.7 s against the joint conductor's
209.9 s, TTFM 0.19 s, planning 1 406 s.** First measurement: **followers kept 11.1 % of the ink
offered to them (stage A: 0 %), and the final all-six pass drew 81 % of the picture.** After the
standoff fix, followers reached 66.9 % of offered ink in stage A. The plan must be written as
*"improve the measured leader/follower pipeline"*, and every claim it makes must be an A/B against
these numbers.

**R3. Rewrite section 5's claim 3 and add the precondition.** The JTC does not hold at a future
stamp; it interpolates from the current state to point 0 across the pre-stamp window. Therefore:
`move` must refuse a goal whose `points[0]` is further than a threshold from the measured state
(this already exists as `day1.py send --from-q`); and configure path/goal tolerances, which are all
zero today.

**R3b. Close the pacing hole.** The executor paces by arc length at its own speed and never reads
`t_s`, so the plan's joint-velocity certificate does not transfer, and a dispatch that omits
`RTFF_TRAVEL_SPEED=0.02` runs transits at 4× the certified speed. Either make the executor honour
`t_s`, or pace the plan at the executor's speeds and have the dispatcher refuse when the manifest's
`speeds` disagree with the environment. This is the cheapest genuine safety fix on the list.

**R4. Replace "certification accepts a timing tolerance" with s-domain certification.** Timing
tolerance was measured on this rig and the safe set is not an interval (PASS 0-0.75 s, FAIL
1.0-1.25 s, PASS 1.75-10 s, symmetric window 0). Derive any timing band from the pairwise
`C_ij(s_i, s_j)` image, and certify concurrent drawing in s with a monotone path governor. Keep
"no synchronised multi-arm motion" as the standing constraint until the governor and a fleet clock
exist.

**R5. Promote runtime plane-finding to the top of section 7.** The supervisor's ladder gate
returns success without measuring for inverted arms (`draw_rtff_supervised.sh:590-593`) and
`config/site.json` ships `RTFF_CONTACT_DESCEND: "0"`. Every arm in the installation is inverted.
This is the actual cause of "runs into the table / hovers and does not draw", and it is a runtime
defect. Offline calibration is necessary but it is second.

**R6. Change the calibration procedure:** rigid probe for the geometric fit; point fiducials
touched from ≥5 orientations spanning ≥25° of lean, solved **jointly across all six arms with the
fiducial positions as free parameters** (the tape measure then only fixes an invisible global
transform); identifiable joint offsets (joints 2-6) in the parameter vector; a paper **height map**
at two press forces rather than a 9-point plane; and **acceptance by a drawn mark measured
externally** — inter-arm mark-to-mark offset in each overlap band, < 1 mm. Add the note that the
joint-5 residual is a *control* steady-state error, different under the impedance law, so nothing
measured in position mode transfers without the drawn check.

**R7. Fix the split rule and the sequencer.** Split at the certified extent `s*`, never at a cell
boundary — the cell-boundary decomposition is the recorded cause of `hatch` 83.1 % and `spiral`
84.4 %. Sequence over `(stroke, direction, entry-fiber variant)` on real transit **time**, with a
chain-wide Viterbi over IK sheets; a greedy nearest/flood-fill over paper distance reproduces the
measured "before" (pen-up 33.5 % branch flips, 49.6 % tall legs).

**R8. Add the lease protocol to section 6 and make the plan file a journal.** Incremental dispatch
needs exclusive spatial leases, disjoint by construction, plus the rule that **every dispatched
chunk ends in a pose certified holdable indefinitely against every other lease.** Without that
rule, dispatching before full certification can drive an arm into a state with no legal
continuation. The plan file becomes header + append-only signed chunks; the operator tails it.

**R8b. Answer the virtual-walls open question with the numbers, and add the scheme-C option.**
*(measured)* Adopt **row bands at table y = ±605 mm with a 25 mm setback** as the walls: the three
leaders are then provably independent at **+66.0 mm** and keep **56.7 %** of the canvas (against
69.1 % unwalled; the setback is what buys the margin — at 0 mm the triple is +6.3 mm). A vertical
wall is much worse (leader group 35.9 %) and must in any case lie in **x ∈ [0.774, 1.030] m** or a
whole column has zero compliant poses. And evaluate the option the plan does not have: **wall all six
arms into their own blocks** — every pair clears at **+55.9 mm** over **57.4 %** of the canvas, so
phase 1 becomes six fully asynchronous arms with *no pairwise certificate at all* and a
time-to-first-motion of one stroke plan, and only the remaining 43 % needs a conducted pass. Cost
lands on the middle row: confining 31 and 71 to ±605 mm costs **27 %** of their combined footprint.
Both figures are lower bounds (one atlas pose per cell).

**R8c. State the placement constraint.** *(measured)* The six-arm strict-GO union at h = 0.970 is
**97.72 %**; the missing 2.28 % (0.151 m²) is **all rim** — the canvas corner is 850.0 mm from the
nearest arm axis against a 748.7 mm certified radius. The largest all-live rectangle is
**1.50 × 3.64 m**. Also: **57.3 % of the canvas has exactly one arm that can draw it**, so over half
the picture has no allocation freedom and a refusal there cannot be reassigned. Put both in the plan;
they bound what any planner can promise.

**R9. Split the distribution, not the repo.** `aris-core` pure-python wheel + `aris_robot` ament
package; colcon workspace as a top-level sibling, not inside the python package; operator install
from a local wheelhouse (the robot PC is offline); hash the five semantic inputs separately and
refuse only on calibration/geometry.

**R10. Carry the measured collision geometry across, as data.** The 2026-08-26 collision audit
found capsules that did **not contain the arm** (link0 with connector and cable by +170 mm) and
that up to 78 of the then-80 mm margin was consumed by model optimism; the fix was per-link
measured radii and a 3-band base column. Whatever kernel is chosen must be loaded with **those**
numbers, and a containment test against the manufacturer meshes must be part of the kernel
acceptance. Changing kernels silently changes the safety envelope.

**R11. Make the invariants explicit (H1) and make the corpus adversarial (H3).** Seven invariants,
each already bought with a defect; corpus generators rather than five pictures, including strokes
that deliberately cross every cell and row boundary, pass under a base, and sit at the rim.

**R12. Reorder the milestones** (see I) and put honest durations on them. Milestone 2 as written
cannot start before milestones 3 and 4 exist.

**R11b. Three input-contract items the plan is missing.** *(2b.3)* (i) An **ink dimension**: an ink
is a constraint on which arm may draw a stroke at all and belongs in the capability map — without it a
two-colour picture is *wrong*, not slow. (ii) The **tilt cone** (15°) is the single biggest coverage
lever and is not the default; make the default match the configuration the corpus numbers were taken
at, or restate the numbers. (iii) The gap account must **separate rim-blocked ink from `no_drawer`**,
because `traces.BLOCK` is 6.41 m of the corpus loss and it is the canvas, not the planner.

**R12b. Put the tool tip in the geometry hash, and re-certify on calibration.** *(2b.5)* The shipped
programme holds **20.5 mm of paper-chain clearance against a 20 mm gate**, so a calibration that finds
the tip deeper invalidates every certified plan by 0.5 mm. Report the paper-chain headroom next to the
calibration residual, and make "re-certify an existing programme at the new tip" part of the
calibration acceptance test.

**R13. Correct §2's table.** Only `scatter` of the five bench drawings is certified (H2). The module
count is 71 `.py` files, not 41 *(measured)* — the 47 799 / 42 126 / 24 539 line counts are right.
And add one row that is missing and is the most important number in the table: **planning wall for a
single-arm word = 66.8 s, of which 58.6 % is pen-up routing and 36.9 % is certification.**

**R14. Two things to delete from the plan.** *"stroke tracking ~1 ms per stroke"* — pick a target
that is physically available (see C) and state whether it is per stroke or per waypoint.
And *"legacy is deleted when the corpus passes on the new core"* — keep the docs forever; they are
the defect list, and they are worth more than the code.

---

## 4. What I could not verify from this machine, and what would settle it

1. **The running JTC.** No ROS here; the vendored tree is Humble-era JTC 3.3.0 and the rig is
   Jazzy with 4.x. *Settle it:* on the operator, `ros2 pkg xml joint_trajectory_controller`, then a
   two-arm test — send both arms a single-point goal at `points[0] = measured q`, stamped 2 s
   ahead, and log `/joint_states` to see whether they move before the stamp. 20 minutes.
2. **The live 378-line `cartesian_impedance_controller`.** Not on this machine. Specifically
   unknown: whether it has a parameter callback (the tracked 350-line copy does not, yet the
   executor sets K per phase), and whether it will accept a joint-reference subscriber without
   further change. *Settle it:* scp the file into git. This blocks stage 1 and it blocks the SIL
   being meaningful.
3. **Whether one rclpy process can hold six domains on Jazzy.** Nothing in this codebase has ever
   tried; `domain_id` appears nowhere. *Settle it:* a 30-line script on the operator. But prefer
   not needing the answer (B7).
4. **Which physical arm is in which slot, and which way the pen holder is mounted.**
   `config/site.json` says `"mounted": null`; the holder can sit either way round in the jaw and the
   86 mm lateral tip flips sign with it. *Settle it:* the GUI's "Identify arms" poll plus a visual
   check against the posed tool model, before any motion.
5. **The as-built base poses, heights and yaws.** Unsurveyed; 30 mm of the 50 mm inter-arm gate is
   the allowance for that. `scripts/asbuilt_layout.py` is the thing that consumes a survey and it
   has never been fed one.
6. **Paper flatness and the real paper height per cell.** Never measured. Hover passes plus a ruler
   are the only data.
7. **Wear rate of the actual pen.** No measurement exists. Ten minutes of drawing and a re-touch-off
   settles it.
8. **The stiffness/press-depth inconsistency.** The briefing records both effective stiffness
   0.40 × commanded (→ 320 N/m in z) and *"press ~10 mm for 0.7-1.0 N"*, which imply stiffnesses
   differing by ~4×; and `d_max` is 8 mm in the executor against 12 mm in the briefing. One
   quasi-static push-and-measure per arm settles both and it is needed for tone.
9. **Measured draw speed 0.0867 m/s versus the executor's 0.02 m/s default.** The notes record both
   and do not reconcile them. Whatever paces the plan must match what the arm does.
10. **VAMP/cuRobo on this hardware** — see C for what I could and could not establish.
11. **Whether `update_rate` is 1000 or 60 on the operator.** Two configs on this machine disagree;
    it changes any synchronisation budget by 17×.
12. **The as-built arm-axis position inside the strut pair.** Pete's tape gives the axis **42 mm**
    off the post-pair centre where the model has 25.15 mm — a ~17 mm shift of **both** axis lines,
    still recorded as unresolved and still not applied to `system_model`. Every geometry number in
    section D moves by that amount. *Settle it:* one straight-edge measurement, then re-run the atlas.
13. **Whether the wall-compliant areas in D5 are as small as measured.** They are lower bounds
    because the atlas stores one pose per cell; re-solving each cell under a half-space constraint
    (the q7 window, the lean ladder, the IK branch) would raise them by an unmeasured amount. If the
    scheme-C option looks attractive, that re-solve is the measurement to take next and it is a
    day's work.
14. **Anything about behaviour under the impedance law.** Every clearance number in this audit is a
    property of *planned* configurations. The controller's steady-state error (the joint-5 residual)
    means the arm is not exactly there, and nothing on this machine measures by how much while
    drawing.

