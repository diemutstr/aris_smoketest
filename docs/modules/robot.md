# robot: the operator PC side

**Job.** Run the checked motions on the real arms. The planning PC plans, checks and queues;
the operator PC (ROS 2 Jazzy, the Franka driver) copies the queues as they are written, runs
them with the same executor and coordinator as the simulated arms, and sends back every event.
This is the only place that knows about ROS. Folder: `robot/`. Set-up steps:
`robot/README.md`.

## The two machines

| planning PC | operator PC |
|---|---|
| drawing server, planners, checker | one ROS launch per slot (namespace `arm_<slot>`, the DDS domain from the site table) |
| writes the job: header, phases, one queue per phase and arm | `aris-robot serve`, on the server's "run": copies the phases and the queues byte for byte, runs them, posts the events back |

**Slots and robots** (DESIGN 4c). Arms are slots, `1L 1R 2L 2R 3L 3R`, strings everywhere
here (rows keep the key `"arm"`: `"arm": "2R"`). Which robot hangs in which slot, its address
and its DDS domain are in the site table `site/aris_2026-10.json` (the domain must be an
integer; today it is the old robot id). `robot/site.json` names that table and keeps only what
is about this PC: the server, `mounted` and `force_sign` per slot, the tare, contact, touch and
collision settings. Every run and serve's first row report, per slot, the robot the table
names and its identity: "verified", "mismatch: ..." (the job is refused), or "unverified" (no
serial on either side). FCI and ROS report no serial, so it is "unverified" until one can be
read.

**Tracking** (DESIGN 4c). The job header's `tracking` decides how the arms follow:
`position` (mode A, the default) sends every kind of motion through the stock trajectory
controller, and the press is the plan's (the pen's `press_m` below the paper; recorded in
the first row, nothing to apply). The collision thresholds are set to site.json
`collision.job` (40 N) by a service call to franka_hardware's
`set_full_collision_behavior` at job start, before anything moves, and back to
`collision.normal` after the job, also after a refusal. A service call per job is cleaner
than a launch parameter: the thresholds belong to the job's mode, not to the stack, and the
launch has no such parameter. If the robot refuses them, the job is refused. `impedance`
(mode B) is the impedance controller with the pen force, below, unchanged.

**Where the arms stand.** The server plans from what the runner reports; it cannot see the
arms. The first row of a run, "runner started", carries `where`, the 7 joints of every
mounted arm, read before anything moves, and the job id. Every row about one arm (motion
started, motion done, holding, failed, stopped, finished) carries `q`, read from that arm
when the row is written. The last row, "runner finished", carries the result and `where`
again. A stale position cannot move an arm: the executor refuses any motion whose start is
more than the rig's start tolerance (0.005 rad per joint) from the arm's joints. It writes a
"failed" row that names the joint and the distance, and the arm holds. A park job (`aris park`)
is a job like any other, one arm per phase, and runs the same way: nothing in the runner
depends on what is drawn.

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

## The resident process (`aris-robot serve`)

Pete's requirement: one process on the hardware computer that nobody touches; every command,
calibration included, comes over the Ethernet cable. `aris-robot serve` is started by systemd
at boot (`robot/aris-robot.service`, restart always) and:

- **Keeps the ROS stacks up.** One `ros2 launch` per mounted arm, each in its own process
  group. One that exits is restarted after 1, 2, 4, ... s, up to a minute, and every start
  and death is a row. It supervises child processes rather than using a systemd unit per arm,
  because the stacks follow site.json without installing units, their deaths reach the server
  like every other row, and there is one unit to enable.
- **Pulls its work.** It asks `GET /operator/next?wait=30` (long-poll), acknowledges with
  `POST /operator/ack`, and gets `run <job>`, `recover <arm>` or `report`. A run is exactly
  `runner.run_job`, in the same process, with drivers kept for the process's life. This
  PC opens no port.
- **Fetches the calibration** from the server (`GET /calibration`, `GET /calibration/{arm}`)
  into its config before every job, and removes local files the server does not have. The
  drivers take the new pen and paper. The job's digests then agree, unless rig.json itself
  differs (another commit), which is still refused.
- **Says where the arms are.** A "where" row every 10 s while idle, besides the rows of every
  run (above).
- **Needs no terminal.** Everything goes to the server as rows (`POST /operator/rows`) and to
  `out/operator/rows.jsonl` and `serve.log`. A command that fails is a row; the process goes
  on.

## How the arm follows a motion in mode B (`tracking: impedance`)

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
| `touch(motion)` | the calibration touch, below |
| `hold()` | nothing to do: both controllers hold where the last motion ended |
| `stop()` | at once: the impedance controller holds, a trajectory goal is cancelled; the arm refuses to move until `recover` |
| `recover()` | Franka error recovery, then the trajectory controller takes the arm again. Refused while the arm is in user stop or guiding |
| `switch(name)` | "trajectory" or "impedance" takes the arm |

The executor sends every kind to its verb itself (free to `move`, touch to `touch`, lower,
draw and lift to `draw`).

## The pen force (`aris_robot/force.py`, no ROS)

Where the numbers come from: the band, levels, cap, ramps and servo are facts of pen and
paper, in `config/rig.json` (`pen`). The server copies that block into every job header, and
the runner applies the header's values to every arm of that job. Both machines therefore use
the same numbers, and the "runner started" row says which were used. A header without them
(an older job) runs with the operator PC's rig file. `robot/site.json` keeps what is a fact of
the site or the arm: each arm's force sign, the tare limits, the contact thresholds, the
touch settings.

- **Air zero (tare).** The robot's force estimate is not zero in the air, and it changes with
  the pose (2.5 N on arm 17 in the old stack). Before each landing the arm stands still for
  0.2 s and the mean reading becomes the zero. It is refused if it is over 8 N, or if it moves
  by more than 0.6 N.
- **Contact.** "The moment the force lifts off the air zero is the table" (Diemut). Contact is
  when the force above the zero stays over 0.25 N for three readings in a row. A single spike
  is ignored.
- **How hard.** Intensity 0 to 1 maps onto the band (0.7 to 1.0 N for graphite, in 9 steps,
  from the job's `pen`).
- **When.** Zero while lowering. From zero up to the setpoint over the first 2 mm of the line
  (Diemut's slide-in). Back to zero over the first 0.2 s of the lift.
- **Guard.** More than 3.5 N above the zero for 12 readings in a row: the arm holds and the
  motion fails. This is a check that the numbers make sense. It does not protect the paper.
- **Servo.** A slow correction of the fed force toward the setpoint, from the force
  estimate: a 1 s time constant (`servo_ki` 1/s), bounded to ±1 N, only in contact. On by
  default.
- **Touch** (`touch.py`, the calibration's `touch` motions; DESIGN 6 step 1). Under
  position control, with the stock trajectory controller and not the impedance one: the
  encoders say where the paper is, the force only when. The air zero is taken standing at
  the hover. The arm flies the descent half of the motion at its own slow timing. At the
  first onset (1.0 N over the zero, 3 readings in a row) the trajectory is cancelled, and the
  joints of the first of those readings are the answer. If the planned end comes without
  contact, it goes straight on in the same direction for the motion's `extra_depth` (at most
  30 mm), at 2 mm/s, the hand keeping its orientation (IK, as the planner made the descent);
  then it gives up with "no contact within … mm". In both cases it flies back to the hover
  along the path flown. Over 3 N at any reading it stops and holds where it is, flies no way
  back, and the arm refuses to move until it is recovered. The executor logs a "contact" row
  with the joints. The first `aris touchoff <slot>` from the planning PC also checks the force sign.
  On fake hardware a fake paper stands in for the force estimate.

## Tested here (no ROS), 2026-09-30

`robot/tests`: 60 tests, 56 in the quick set; the 4 slow ones compile the controller
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
- **Touch.** Simulated position control and a fake paper (5000 N/m, an air reading of 2.3 N
  with noise). Paper 5 mm high, at the plan and 4 mm low: contact read within 0.4 mm of the
  surface, back at the hover to 1e-9. Paper 12 mm low: the extension goes on straight (within
  20 µm) at no more than 2 mm/s and finds it 12 mm past the end. Paper 50 mm low: it gives up
  after exactly the planned descent plus 20 mm and comes back. Steel instead of paper: the
  cap stops and holds, with no way back flown. Refused before moving: an extra depth over the
  cap, or a 9 N air reading. Through the executor, the "contact" row carries the joints at the
  paper.
- **Slots and modes.** The site is read from the site table and site.json (robot, address,
  domain per slot; a duplicate domain and an unknown slot are refused). Identity is
  verified, mismatched or unverified. Through the stand-in server: the header's tracking
  (none, position, impedance) reaches every driver. Position raises the collision thresholds
  and restores them after; impedance leaves them alone. The first row carries the mode, the
  pen with its press and the robots. Refused before anything moves: an unknown mode, a robot
  mismatch, thresholds the robot refuses (restored even then). On the fake ROS node, mode A
  sends lower, draw and lift to the trajectory controller as planned, nothing to the
  impedance controller, and the thresholds calls carry the site's values.
- **Pen rules from the job.** The runner hands the header's `pen` to each driver (a gel-pen
  band and cap in the test), or the rig file's when the header has none, and the first row
  says which.
- **serve.** Against the stand-in server: report, two runs (a drawing-like job and a
  calibration job with a touch), recover, and recover of an arm that is not there. Every
  command is acknowledged in order. The server's calibration file arrives and the stale local
  one is removed, so the calibrated job's digests agree and it runs. The touch's contact row
  sits on the calibrated fake paper. "where" rows come while idle. A job planned for another
  rig gives a "run refused" row, and the process goes on. The stack keeper restarts a dying
  child after 0.05, 0.1, 0.2 s and stops a running one with SIGINT in under 3 s.
- **Runner.** Real HTTP against the stand-in server with simulated arms. The job is written
  while it runs (free, lower, draw, lift): done, the arm back at its park to 1e-9, the copied
  queue byte-identical, every event on the server in order (4 motions at 50x: 0.3 s). Also
  checked: the first and last rows carry `where` and every row about the arm carries `q`,
  equal to the simulated arm's joints (the start and end of each motion to 1e-9); an arm
  20 mrad off on joint 5 is refused before anything moves, with a row naming joint 5 and the
  tolerance, and it holds where it stands; a park job (two arms, one per phase, from 0.03 rad
  off, a header without drawing or digests) ends with both arms at their parks and reports
  it; a link that hangs up every 2000 bytes still gives a byte-identical copy; a phase
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
- The touch on a real arm: cancelling a trajectory goal mid-descent (the controller then holds
  where it is), the force estimate from the robot state broadcaster at 100 Hz, and the
  systemd unit (user, paths, real-time limits).
- `aris-robot identify` reports the address, the domain, the mode, the robot's IP in its launch
  and the nearest park. It gives no serial number: FCI and ROS do not report one.

## What it cannot do (yet)

- Nothing moves an arm except a job of the server (`aris-robot` has only `serve` and the
  read-only `identify`, Pete 2026-10-02).
- No re-plan after a failure. A failed arm holds, and the job ends after its phase (as with the
  simulated arms).
- No depth limit along the pen inside the controller (the old stack's DMAX). The tracking-error
  hold (0.05 rad) and the force guard are the only limits.
- The force estimate is the robot's model estimate at its end effector. Its drift against the
  0.3 N band is not known for this controller, and the servo, on by default, follows it.
