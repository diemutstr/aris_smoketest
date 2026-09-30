# ARIS2 contracts — piping planner joint trajectories into the deployed RTff stack

Written 2026-09-09. These are the shared interfaces every aris2 work item builds against.
Change them here first, then in code. Background: `/home/franka/aris_project/briefings/
CONTROL_STACK_line_to_joint_torques_2026-09-09.md` (the operator stack), `docker/jazzy/README.md`
(the Jazzy harness), `docs/HARDWARE_LADDER.md` §0 is SUPERSEDED on the executor choice: the
installation keeps its Cartesian impedance controller; we replace its REFERENCE, not its law.

## 0. Ground rules

- Minimal change to the deployed stack. Every deployed-side change is flag-gated and byte-identical
  in behaviour when the flag is off or the new data is absent.
- Deployed code lives on `Aris_Kindt`. Dev branches: `aris2-rtff` (worktree
  `/home/franka/aris_project/worktrees/aris2-rtff`, off `diemut-operator-rtff`) and `aris2-gui`
  (off `diemut`). Never commit on `diemut*`. Nothing is pushed without Pete's explicit approval.
- Planner-side code lives in this repo (`aris_sixarm`, branch `aris2`, venv `.venv`).
- Anything importing `rclpy` or building C++ runs in the Jazzy container (`docker/jazzy/`), never on
  the host. No ROS on the host. Nothing under `/home/franka/git` is read or run.

## 1. Pathway CSV v2 (planner -> operator)

Same file the deployed GUI produces and the executor `rtff_pathway_exec.py` loads, plus seven
optional columns.

```
stroke_idx,wp_idx,kind,x_m,y_m,z_m,qx,qy,qz,qw,intensity,q1,q2,q3,q4,q5,q6,q7
```

- Frame: the ARM's own `fr3_link0`, metres, unit quaternion (x,y,z,w). One row per waypoint.
- `kind`: `draw` = pen on paper; anything else (`travel`, `lift`, `lift_start`, `lift_end`) is
  pen-up. The executor tests `kind == "draw"` and nothing else.
- Pose = the EE frame the robot is configured with (`setEE` = NOMINAL pen tip, see §4).
  Quaternion convention is the deployed one: `(1,0,0,0)` = pen straight down on a floor arm; the
  pen axis is EE +Z.
- `z_m` = the paper plane in base frame as baked by the generator; the executor overrides it with its
  live plane logic (ladder gate / baked z for inverted arms). Record the value used in the manifest.
- `intensity` 0..1 = tone. It is the ONLY pressure channel. It must survive.
- `q1..q7` (rad): the planner's joint solution for that row. FK(q) through the same EE frame MUST
  equal (x,y,z,quat) of the row to < 0.5 mm / < 0.5 deg (the exporter asserts this). Columns may
  be absent or empty; then the executor behaves exactly as v1.
- **Transits are part of the file (finding 2026-09-09).** The planner reconfigures the arm between
  strokes (measured on the v18 CSAIL schedule, arm 31: 4.4 rad on j3, 4.7 rad on j5, 3.1 rad on j3 at
  stroke boundaries) and realises that inside its certified pen-up transits. The executor walks
  consecutive non-draw rows as a Cartesian path at `--travel-speed`, so the transit frames MUST be
  emitted as `travel` rows, in order, with q on every row. A file that drops them is not executable:
  in the SIL the arm hit joint limits at the first such boundary and never recovered. Every row
  carries q1..q7 (pen-up rows too); no empty joint cells.
- Sidecar `<name>.manifest.json` (written by the exporter, read by nobody on the deployed side):
  `arm_id`, `rig`, `tool` (name + tip offset in hand-TCP frame), `T_world_base` (4x4, row-major),
  `paper_z_base_m`, `joint_columns` (bool), `generator` (module + git sha), `source` (planner job /
  schedule file), `speeds` (draw/travel m/s the plan was paced at), `created`.

## 2. Joint reference topic (executor -> controller), stage 1 of the redesign

- Topic `/cartesian_impedance/joint_reference`, type `sensor_msgs/msg/JointState`.
- `name` = `<arm_prefix><robot_type>_joint{1..7}` (the operator's yaml says `fr3v2`, the briefing
  says `fr3`; therefore CONSUMERS MATCH BY INDEX, 7 positions, and only warn on a name mismatch).
- `position` = q_ref (7, rad). `velocity` = qdot_ref (7, rad/s; zeros until stage 2 friction
  feedforward exists). `effort` empty. `header.stamp` = publish time.
- Executor: published on EVERY equilibrium-pose publish when enabled (`--joint-ref` CLI flag or env
  `RTFF_QREF=1`), from the same interpolation fraction as the pose: linear in q between the two
  waypoints of the current segment, held at a waypoint otherwise. If a waypoint has no q, nothing is
  published for that segment. Disabled (default) = no new publisher is even created.
- Controller (later, on the LIVE 378-line source): parameter `joint_reference_topic` (empty =
  disabled = today's behaviour), `joint_reference_timeout_s` (default 0.5). A fresh reference
  replaces the nullspace target `q_nullspace_` (low-passed with `filter_alpha`); stale or absent
  -> the posture latched at activation, i.e. today's behaviour.

- **Continuity (finding 2026-09-09).** The reference is a posture stream, not a set of targets: a
  step of several radians at a stroke boundary drives the arm into joint limits. The executor only
  produces steps if the file has them (see §1), and the controller MUST still guard: slew the
  effective nullspace target toward the received reference at a bounded rate (proposed 1.0 rad/s,
  parameter `joint_reference_rate_limit`) and count/log ticks where the raw reference is further than
  0.5 rad from the effective target. The SIL exposes the same guard (`--qref-rate-limit`).

## 3. Robot-state interface (what the sim must present to the executor)

The executor reads, in order of preference:

1. `/franka_robot_state_broadcaster/robot_state` (`franka_msgs/msg/FrankaRobotState`):
   `o_t_ee.pose.position/orientation` (EE = NOMINAL tip, base frame) and
   `o_f_ext_hat_k.wrench.force` (estimated external wrench at the EE, base frame). Published at
   100 Hz. Silence > 0.5 s switches the executor to the fallback pair.
2. Fallback: `/franka_robot_state_broadcaster/current_pose` (`geometry_msgs/PoseStamped`) and
   `/franka_robot_state_broadcaster/external_wrench_in_base_frame` (`geometry_msgs/WrenchStamped`).

The sim publishes all three. Wrench sign: report the force the ENVIRONMENT exerts on the tool,
expressed in base; the executor's `--force-sign` absorbs the robot's convention and the sim's
sign is a calibration item to confirm on the rig (the real signal also carries a ~2 N pose-dependent
bias the sim does not have).

The sim subscribes to `/cartesian_impedance/equilibrium_pose` (`PoseStamped`, base frame, EE = tip)
and `/cartesian_impedance/joint_reference` (§2). `ROS_DOMAIN_ID` = arm id, root namespace.

## 4. Pen tip model

- The robot believes EE = NOMINAL tip (`setEE`), so `o_t_ee` IS the nominal tip and the controller
  runs with `tool_tip_offset = [0,0,0]`. The controller's Jacobian is the nominal-tip Jacobian.
- Nominal tip in the hand-TCP frame comes from `aris_sixarm.frames` (`tool_offset(...)`: inline
  pen `PEN_EXT = 0.110` axial; lateral holder `PEN_LAT_HOLDER`, `PEN_EXT_HOLDER`). The tool name
  is a parameter; the manifest records the numbers used.
- ACTUAL tip = nominal tip + `tip_error` along the pen axis (EE +Z). SIL default
  `tip_error = -0.010` m: the physical tip is 1 cm CLOSER to the hand than the robot believes.
- The sim reports `o_t_ee` at the NOMINAL tip (what the robot would report), never the actual one.

## 5. Impedance law replica (sim side)

Structure verbatim from the live controller (briefing §7.2): setpoint low-pass (`filter_alpha` at
1 kHz, slerp for orientation), pose-error clamp (`max_pose_error_pos/rot`), `F = K e - D J dq`,
`tau = J^T F + N (Kn (q_null - q) - Dn dq) + coriolis`, per-joint magnitude + slew saturation.
Gravity is added by the "hardware layer" (libfranka), so the sim adds `tau_g` outside the law.
Parameters come from a YAML in the controller's own format:
`aris_sixarm/sil/config/cartesian_impedance_controller.yaml` = the LIVE operator values (briefing
§7.1, with the DRAW-phase K the supervisor commands, §8.4: `2800 2800 800 600 600 200`), provenance
in the header; `--controller-yaml` points at the real file when it arrives. Nullspace target =
posture at activation, or the §2 reference when present.

## 6. Where new code lives

| piece | location | tests |
|---|---|---|
| exporter (schedule/program -> CSV v2 + manifest) | `aris_sixarm/export/pathway.py` | `tests/test_export_pathway.py` (host venv) |
| executor joint reference | worktree `rtff_pathway_exec.py` (flag-gated) | `docker/jazzy/test/` (container) |
| SIL: Drake plant + law replica + setpoint sources + metrics | `aris_sixarm/sil/` (`python -m aris_sixarm.sil`) | `tests/test_sil_*.py` (host venv, extra `sil`) |
| SIL ROS wrapper (closes the loop with the real executor) | `aris_sixarm/sil/ros_node.py` (rclpy imported lazily) | container |
| web console operator layer | `aris_sixarm/gui/` | host venv, fake operator |
