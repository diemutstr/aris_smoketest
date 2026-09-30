# robot: the operator PC side

**Job.** Run the checked motions on the real arms. The planning PC plans, checks and queues;
the operator PC (ROS 2 Jazzy, the Franka driver) copies the queues as they are written, runs
them with the same executor and coordinator as the simulated arms, and sends back every event.
This is the only place that knows about ROS. Folder: `robot/`. Set-up steps:
`robot/README.md`.

## The two machines

| planning PC | operator PC |
|---|---|
| drawing server, planners, checker | one ROS launch per arm (namespace `arm_<id>`, DDS domain = arm id) |
| writes the job: header, phases, one queue per phase and arm | `aris-robot run --job <id>`: copies the phases and the queues byte for byte, runs them, posts the events back |

The copy follows the server's files as they grow and resumes from its own length after a
lost link, so a dropped network stops nothing already copied. The runner refuses a job planned
for a different rig or calibration (digests in the job header), a job it has already run, and
stops before any phase that needs an arm not marked `mounted` in `robot/site.json`. A stop on
the server stops the arms within about 0.2 s (the answer to the next event post).

**What the server has to add** (for its owner; the stand-in in `robot/tests/fake_server.py`
implements all four):

| endpoint | does |
|---|---|
| `GET /jobs/{id}/header` | `job.json` as JSON |
| `GET /jobs/{id}/phases?offset=B` | `phases.jsonl` from byte B on, kept open and sent as it grows, closed after the end line |
| `GET /jobs/{id}/queues/{phase}/{arm}?offset=B` | that queue file from byte B on, the same way, closed after the end marker; 404 until the file exists |
| `POST /jobs/{id}/events` | body `{"source": "robot", "rows": [{"seq": n, ...event...}]}`; append rows in `seq` order, each once; answer `{"accepted", "next_seq", "stop"}`, `stop` true once the job was stopped on the server |

The server must also stop running its own simulated arms for such a job: with real arms, the
executors run on the operator PC.

## How the arm follows a motion

Two controllers, one at a time, switched by the driver:

- **Trajectory controller** (`fr3_arm_controller`) for free motions, parks and jogs. It gets
  the trajectory exactly as planned: every knot with its velocity and time. It joins them with
  the same cubic the planner uses. It never re-times.
- **`aris_joint_impedance_controller`** (new, C++) for lower, draw and lift. At 1 kHz:

  torque = K (q_d − q) + D (qd_d − qd) + Jᵀ f + coriolis − (normal spring term, pen down)

  The robot adds gravity itself. q_d and qd_d are the planned trajectory, sent by the driver
  as one sample per millisecond, a tenth of a second ahead. Between two samples the controller
  uses the cubic through both, which is the trajectory's own definition. f is the pen force,
  pushed along the paper normal at the pen tip. J is the Jacobian of the tip, from the robot's
  model and the pen tip on the flange (written per arm from the rig).

| parameter | default | |
|---|---|---|
| `k_gains` | 300 300 250 250 40 40 15 Nm/rad | about 200 N/m at the pen tip in its softest direction |
| `d_gains` | 30 30 25 20 4 4 1.5 Nm·s/rad | not tuned on an arm |
| `max_force` | 5 N | largest pen force |
| `max_torques`, `max_torque_rate` | 75×4, 11×3 Nm; 990 Nm/s | libfranka refuses 1000 Nm/s |
| `max_tracking_error` | 0.05 rad | any joint further than this from the reference: hold |
| `starve_timeout` | 20 ms | the stream ran dry: hold |
| `start_tolerance` | 0.01 rad | a new stream must start where the arm holds |
| `idle_force_timeout` | 2 s | nobody streaming: the pen force is taken away over 1 s |
| `k_normal` | 100 N/m | the pen's spring along the paper normal, pen down |
| `d_normal` | critically damped | for the arm's own mass along the normal, from the model each tick |

**Holding** means the controller stands still with zero force. A hold is started by `~/hold`,
by a stream that ran dry, or by a tracking error. It lasts until `~/resume` or until the
controller is switched on again, and while it lasts every new stream is refused. `~/status`
(250 Hz) gives the reference, the tracking error, the fed force, the robot's own estimate of
the outside force, and why the arm holds.

**How stiff the pen is.** One stiffness per joint does not make the pen equally stiff in
every direction. With the joint springs alone, at 672 drawing poses of the six arms (pen
upright, inside the gates), the stiffness along the paper normal was 361 / 848 / 1968 N/m
(min / median / max). A paper 1 mm higher than planned pressed 0.85 N harder, which is the
whole force band. So while the pen is down (lower, draw, lift: the samples carry the paper
normal, which may be tilted after calibration) the controller replaces the tip's stiffness
and damping along the normal by a soft spring:

  extra torque = − Jᵀ (B_k J e + B_d J ė), with B_k = K_t − P K_t P − k_n n nᵀ

Here K_t = (J K⁻¹ Jᵀ)⁻¹ is the stiffness the joint springs give at the tip, n is the normal,
P = I − n nᵀ, and B_d is the same with the damping. The stiffness at the tip then becomes
exactly P K_t P + k_n n nᵀ. That is k_n along the normal, the paper-plane part unchanged, and
no coupling between the two, so a paper-height error pushes the pen neither harder than
k_n allows nor sideways. It is symmetric, and the joint stiffness stays positive definite, so
the arm stays passive. It costs two 3×3 inverses per tick. The damping along the normal is
critical for the arm's own mass there, from the mass matrix each tick. (This differs from
the formula first proposed, −Jᵀ n nᵀ F_imp + k_n …, only in also removing the coupling term
K_t n: without it the law is not symmetric, and passivity is not guaranteed.)

Measured through the compiled law at the same 672 poses, k_n = 100 N/m:

| | min / median / max, N/m |
|---|---|
| along the normal, pen free to slide | 100.0000 / 100.0000 / 100.0000 |
| along the normal, pen held sideways | 100.0000 / 100.0000 / 100.0000 |
| softest direction in the paper plane, before | 169 / 218 / 306 |
| the same, with the normal spring | 169 / 218 / 306 (in-plane block changed by at most 3e-15, relative) |

A paper 3 mm off now changes the press by 0.3 N, and the force servo trims that away within
a few seconds.

## The driver verbs (`aris_robot/driver.py`)

| verb | what the real arm does |
|---|---|
| `state()` | joints and speeds; robot mode and errors; the impedance controller's hold. Able to move only in mode idle or move, with no error, not stopped |
| `move(traj)` | checks the arm is at the first knot (5 mrad), then the trajectory controller flies it. Answers done, or failed with the controller's own error text |
| `draw(motion)` | lower, draw and lift go through the impedance controller with the pen force; a free motion goes to `move` |
| `hold()` | nothing to do: both controllers hold where the last motion ended |
| `stop()` | at once: the impedance controller holds, a trajectory goal is cancelled; the arm refuses to move until `recover` |
| `recover()` | Franka error recovery, then the trajectory controller takes the arm again. Refused while the arm is in user stop or guiding |
| `switch(name)` | "trajectory" or "impedance" takes the arm |

The executor sends lower and lift to `move` with the trajectory only, so the runner puts a
small router in front of each driver. The router finds each trajectory in the arm's own queue
and sends lower and lift to `draw`. **Contract change requested:** the executor should call
`draw` for lower, draw and lift; then the router goes.

## The pen force (`aris_robot/force.py`, no ROS)

- **Air zero (tare).** The robot's force estimate is not zero in the air, and it changes with
  the pose (2.5 N on arm 17 in the old stack). Before each landing the arm stands still for
  0.2 s and the mean reading becomes the zero. It is refused if it is over 8 N, or if it moves
  by more than 0.6 N.
- **Contact.** "The moment the force lifts off the air zero is the table" (Diemut). Contact is
  when the force above the zero stays over 0.25 N for three readings in a row. A single spike
  is ignored.
- **How hard.** Intensity 0 to 1 maps onto the band (0.7 to 1.0 N for graphite, in 9 steps,
  from `site.json`).
- **When.** Zero while lowering. From zero up to the setpoint over the first 2 mm of the line
  (Diemut's slide-in). Back to zero over the first 0.2 s of the lift.
- **Guard.** More than 3.5 N above the zero for 12 readings in a row: the arm holds and the
  motion fails. This is a check that the numbers make sense. It does not protect the paper.
- **Servo.** A slow correction of the fed force toward the setpoint, from the force
  estimate: a 1 s time constant (`servo_ki` 1/s), bounded to ±1 N, only in contact. On by
  default.
- **Touch** (`aris-robot touch`). A straight descent at 5 mm/s, at most 60 mm, with zero force.
  It stops at the first contact and reports the joints, the pen tip and the air zero. This is
  the step the calibration job will use.

## Tested here (no ROS), 2026-09-30

`robot/tests`: 42 tests, 38 in the quick set (11 s); the 4 slow ones compile the controller
core (6 to 15 s under load).

- **Sampling.** The trajectory sampled at 1 kHz matches `aris.kernel.retime.sample` to
  8.9e-16 rad and 1.1e-13 rad/s, between the samples too. Four arms, random timed trajectories.
- **Controller core.** The law, the reference and the holds, compiled with g++ from the same
  header the controller uses. Ticking at 1 ms, the reference equals the planned trajectory to
  9e-16 rad. Ticking at 0.7 ms, so between samples, it is within 3.9e-10 rad and 1.1e-6 rad/s.
  It lands exactly on the last knot. Also checked: holds after 20 ms without samples; refuses
  a stream while holding or away from the reference; a tracking error latches a hold where the
  arm is, with zero force; the force is capped at 5 N; the torque is rate-limited; the force
  is taken away when idle.
- **Pen force.** Synthetic traces (no recorded force log exists on this machine). The tare
  finds a 2.3 N air zero and refuses 9 N or a moving reading. Contact is found 2 readings after
  onset and ignores a 3 N spike. The guard ignores 11 readings over the cap and acts on 12. The
  servo brings a plant that feels 70 % of the fed force to the setpoint and stays within its
  bound.
- **Driver logic.** Against a fake arm node (ROS stubbed out): lower, draw and lift with no
  force while lowering. Contact found within 50 ms of the paper. The force ramps in to the
  setpoint and out to zero. An over-force holds the arm. A starved stream fails the motion. An
  implausible air zero refuses the landing. Stop, then recover. A stream refused while holding.
- **Launch generation.** All six arms: address, domain and namespace; the hanging base
  reproduces `T_table_base` to 1e-12; the pen tip in the flange frame reproduces the kernel's
  tip to 1e-12; the fake-hardware variant. The launch file reads only keys that are written.
- **Runner.** Real HTTP against the stand-in server with simulated arms. The job is written
  while it runs (free, lower, draw, lift): done, the arm back at its park to 1e-9, the copied
  queue byte-identical, every event on the server in order (4 motions at 50x: 0.3 s). Also
  checked: a link that hangs up every 2000 bytes still gives a byte-identical copy; a phase
  needing an unmounted arm fails the job, with the reason posted; a stop on the server stops
  the arms, also while the runner is still waiting for the plan; a wrong rig, an unknown job
  and an unreachable server are refused.
- **Normal spring.** For 50 random Jacobians and normals, the joint stiffness the law
  applies is symmetric and positive definite. It gives exactly P K_t P + k_n n nᵀ at the tip,
  and damping 2√(k_n m) along the normal, with m from a mass matrix when one is given.
  Pen up, it is the plain joint impedance. Measured at the 672 drawing poses: see the table
  above.
- **C++ syntax.** The ROS controller compiles only against stub headers written here, which
  catches mistakes in its own code, not in the Jazzy API.

## Only on the operator PC

- Building against Jazzy's real headers: `get_optional()`, `RealtimePublisher`,
  `FrankaRobotModel`, and the `/**/node` wildcards in the controllers file.
- Anything with an arm: gains and damping (the normal spring's damping uses the model's mass
  matrix), the sign and drift of the force estimate, whether
  1 kHz samples reach the controller without starving it (a tenth of a second of lead), the
  switch between controllers while standing, error recovery.
- Whether fake hardware offers a position command interface (the fake setup assumes it).
- `aris-robot identify` reports the address, the domain, the mode, the robot's IP in its launch
  and the nearest park. It gives no serial number: FCI and ROS do not report one.

## What it cannot do (yet)

- No certified park from far away without the server's park job. `aris-robot park` moves
  straight only from within 0.05 rad. `jog` moves one joint by at most 0.1 rad. `touch`
  descends at most 60 mm. None of these is checked for collisions.
- No re-plan after a failure. A failed arm holds, and the job ends after its phase (as with the
  simulated arms).
- No depth limit along the pen inside the controller (the old stack's DMAX). The tracking-error
  hold (0.05 rad) and the force guard are the only limits.
- The force estimate is the robot's model estimate at its end effector. Its drift against the
  0.3 N band is not known for this controller, and the servo, on by default, follows it.
