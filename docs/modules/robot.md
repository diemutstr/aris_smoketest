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
is about this PC: the server, `mounted`, `force_sign` and `rt_core` per slot, the touch and the
collision settings. Every run and serve's first row report, per slot, the robot the table
names and its identity: "verified", "mismatch: ..." (the job is refused), or "unverified" (no
serial on either side). FCI and ROS report no serial, so it is "unverified" until one can be
read.

**Tracking.** One mode: joint position control (Pete, 2026-10-07). Every kind of motion goes
through the stock trajectory controller, and the press is the plan's: the pen's `press_m`
below the paper, recorded in the first row, with nothing to apply. A header asking for another
mode is refused. For every job, the collision thresholds are set to site.json
`collision.job` (40 N) by a service call to franka_hardware's `set_full_collision_behavior`
at job start, before anything moves, and back to `collision.normal` after the job. A service
call per job is cleaner than a launch parameter: the thresholds belong to the job, and the
launch has no such parameter. If the robot will not take them, the first row says so and the
job runs at the normal thresholds.

**Same code on both machines.** serve sends its `aris.version.code_version()` with every
long-poll and acknowledgement and in its first row. The runner refuses a job whose header's
`code.digest` differs from its own, or that has no `code` ("planned by unknown"), before
anything moves.

**The gripper** (`gripper.py`): `franka_gripper_node` per stack (root namespace of the arm's
domain). A `grip` job (header `kind: "grip"`, `slot`, `verb`, `params`, no phases) runs home,
open or close with no plan, no motion and no threshold call. It posts "grip started", then
"grip done" (`width_before_m`, `width_after_m`, `grasped`) or "failed", then "job done" or
"job failed". Each command is idempotent, and jaws that did not move fail even when the
gripper said success.

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
- **Says where the arms are.** A "where" row every 10 s, read from the arms, also while a job
  runs, besides the rows of every run (above).
- **Needs no terminal.** Everything goes to the server as rows (`POST /operator/rows`) and to
  `out/operator/rows.jsonl` and `serve.log`. A command that fails is a row; the process goes
  on.

## The mark calibration

One guided meeting per arm pair (Pete, 2026-10-08). The person does the Desk mode switches in
the browser; the software never touches Desk. Both arms of the pair are active in the meet
phase, each with one `guide` motion at its hover, and each arm's watch runs on its own.
1. At the hover the arm's `fr3_arm_controller` is deactivated. An `instruction` row goes on
   the job and the operator rows, with the text from site.json `guide.instruction`: "your
   turn: in Desk switch BOTH arms to programming mode; bring the two pen tips together until
   they touch; let go of both; both back to execution mode, FCI on".
2. Programming mode switches FCI off, so the joint states go stale. This is expected:
   - the arm's stack is restarted every `guide.restart_every_s` (15 s) while they are stale;
   - once they are fresh again, the hardware component is reactivated, error recovery runs,
     and the trajectory controller stays OFF;
   - the sample is the joints once the arm has been still (`still_rad`) for `settle_s` (2 s).
   A sample within `moved_rad` (0.02 rad) of the hover gets the row "nobody moved <slot>;
   waiting" and the watch keeps waiting. After `timeout_s` (600 s) the guide fails.
3. Then the controller is activated (it holds where the arm is). The arm retreats: the pen
   goes 3 cm straight up, then the tip moves on a straight horizontal line to above the hover
   (away from the partner's tip, since the hover is on the arm's own side), then straight
   down (or up) to the hover. Both legs are IK-tracked at the free speed and end at the
   hover's joints. Both arms retreat at once and only ever move apart. The result is done, why "check", with the sampled
   joints.
The sample is never discarded: "guide: registered" (with the joints) is posted the moment it
is taken, before any retreat. If the lift or the glide fails, the trajectory controller holds
where the arm stands, the row "registered; stayed at the meeting pose (glide failed: <why>)"
is posted, and the guide still succeeds ("check", `q` the sample, `q_end` where the arm
stands): the server plans the retreat from there.
Every stage is a "guide: ..." row on the job and the operator rows.

The Desk/panda-py calibration driver was removed on 2026-10-07 after three days of token
failures; it lives at 7d93a14. The guide under FCI (enabling buttons with the controller off) lives at b26aecd; it does not work on these FR3s.

**From the arms (2026-10-06).**
- **No zeros.** An arm without a reading (stack down, stale joints) is `"q": null` with a
  reason in every row and in `where` (`where_missing`).
- **Start tolerance.** It comes from the header or the rig (0.03 rad). The trajectory goal is
  0.03 rad and 3 s.
- **Settling.** After a move the arm must reach |qd| ≤ 0.005 rad/s within 1.5 s.
- **Link drops.** A link drop is auto-recovered once per 2 minutes per arm.
- **Force sign.** It is −1 on the hung arms.

**Recovery (2026-10-07).**
- `recover` is hardware component inactive → active → error recovery → the trajectory
  controller and broadcasters active → the joint states fresh (stamp advancing over 1 s) → if
  not, serve restarts the arm's stack and checks again. Every step runs and is a row.
- serve restarts the stack of an arm with no fresh joint states for 30 s (at most once per
  90 s), after the link-drop auto-recover. A stack started while FCI was off comes up once
  FCI is on.
- A reading older than 2 s is stale: `q` null, with "stale joint states (last x s ago)".

**2026-10-08.**
- **Touch by position lag** (7e2572c, rows12: plane fits of 1L/1R at RMS 15-22 mm; the force
  estimate of arms 31 and 2 is worthless at 3 N). Contact is where the tip stopped: the lag of
  the actual tip behind the commanded one along the descent, less its median over the first
  0.3 s of the descent at speed (in the air), over
  0.3 mm for 3 readings; the touch is the actual joints of the first. The force (8 N over the
  hover zero) is only the safety stop, and also a contact at that reading ("stopped by the
  force cap"). The 3 N threshold and the arming are gone. A row per touch: rule, lag and
  force at contact.
- **Stacks start one at a time:** the next arm's once the previous one's joint states are fresh,
  or after 40 s (the Dell froze launching four).
- **Row 3** hangs inverted (robots 13, 17): `force_sign` −1, cores 28 and 29; all six mounted
  since 2026-10-09.
- **Old services.** `aris-session@*`, the orchestrators and the keep-runners must be disabled
  on the Dell before serve (they hold the spawner lock): README section 5.

**2026-10-07, second round** (the touch arming here is replaced by 2026-10-08).
- **Touch.** The touch detector arms at constant descent speed: the acceleration ramp plus
  0.1 s, from the planned timing. Its zero is the mean of 20 readings taken then; contact is
  3 N above it over 15 readings. A trip before arming (the descent's own jolt) is counted, not
  taken. The cap (6 N over the hover's air zero) counts from the first reading. An extension
  past the planned end re-arms after its own ramp and keeps the first zero.
- **Cores.** Stacks run as `taskset -c <rt_core>` (16-19; row 3: 28 and 29; every core must be in the Dell's isolated set 8-19, 28-39, which serve checks). Real-time priority goes on the
  control-loop threads only, set by the site's helper. The whole tree at FIFO 95 froze the PC.
- **No silent hangs.** serve watches a job until it has started: no progress for 10 s
  fails it with a row naming the step, also in the job's log, and the late start is blocked.
- **Fewer refusals.** The touch refuses only when there is no force signal at all.
  Collision thresholds the robot will not take are noted and the job runs at the normal
  thresholds. Recovery from guiding mode is allowed; only a user stop is refused.

## The driver verbs (`aris_robot/driver.py`)

| verb | what the real arm does |
|---|---|
| `state()` | joints and speeds; robot mode and errors. A reading older than 2 s is none. Able to move only in mode idle or move, with no error, not stopped |
| `move(traj)` | checks the arm is at the first knot (the start tolerance), then the trajectory controller flies it, at its own timing. Answers done once the arm stands still, or failed with the controller's own error text |
| `draw(motion)` | lower, draw and lift: the same as `move` |
| `touch(motion)` | the calibration touch, below |
| `hold()` | nothing to do: the trajectory controller holds where the last motion ended |
| `stop()` | at once: the running goal is cancelled; the arm refuses to move until `recover` |
| `recover()` | the sequence above. Refused only in user stop (only the person at the arm can release it) |

The executor sends every kind to its verb itself (free to `move`, touch to `touch`, lower,
draw and lift to `draw`).

## The touch (`aris_robot/touch.py`, no ROS)

The calibration's `touch` motions (DESIGN 6 step 1), under position control with the stock
trajectory controller: the encoders say where the paper is. The arm flies the descent half of
the motion at its own slow timing. At every reading the tip of the commanded joints (the
controller's own `controller_state` reference when it publishes one; else the planned sample
at the reading's time, counted from when the goal was sent) and the tip of the actual joints
give the lag along the descent. Its baseline is the median lag over the first 0.3 s of the
descent at speed (in the air: every descent starts 20 mm or more above the paper), so a
controller that trails while moving is no contact; the lag rule waits for it, the force cap
does not.
Contact is a lag over 0.3 mm for 3 readings in a row: the trajectory is cancelled, and the
ACTUAL joints of the first of those readings are the answer. The force estimate is only the
safety stop: over 8 N above the hover zero (20 readings standing at the hover) it stops, and
that reading's actual joints are the contact ("stopped by the force cap"). If the planned end
comes without contact, the arm goes straight on in the same direction for the motion's
`extra_depth` (at most 30 mm), at 2 mm/s, the hand keeping its orientation, under the same
rules; then it gives up with "no contact within … mm" (and the largest lag seen). In every case
it flies back to the hover along the path flown. With no force readings at all, the touch is
refused before moving. The driver posts a row per touch (rule, lag and force at contact, lag at
the hover); the executor logs a "contact" row with the joints. On fake hardware the actual
joints follow the commanded ones exactly, so only the fake paper's force cap finds contact.

## Tested here (no ROS), 2026-10-07

`robot/tests`: 55 tests, about 30 s.

- **Driver.** Against a fake arm node (ROS stubbed out):
  - free, lower, draw and lift all go to the trajectory controller exactly as planned, and it
    stays active;
  - a trajectory 0.1 rad from the arm is refused;
  - a reflex → recover → park, with the recovery steps in order;
  - a stalled stack is reported stale and restarted by recovery.
- **Launch generation.** All six arms: address, domain and namespace; the hanging base
  reproduces `T_table_base` to 1e-12. The controllers file has the trajectory controller and
  the two broadcasters only. The fake-hardware variant. The launch file reads only keys that
  are written. No stack is written for a robot the site table says this rig never drives.
- **Touch.** Simulated position control and a fake paper (5000 N/m, an air reading of 2.3 N
  with noise).
  - The simulated tip stops at the paper while the commanded joints go on. Paper 5 mm high, at
    the plan, and 4 mm low: contact by the lag at the paper within 0.3 mm; back at the hover.
  - Paper 12 mm low: the extension goes on straight (within 20 µm) at no more than 2 mm/s and
    finds it.
  - Paper 50 mm low: it gives up after exactly the planned descent plus 20 mm and comes back.
  - A force spike (6 N) with no lag is no contact; a force over the cap (10 N) is a contact
    at that reading, and the way back is flown.
  - Refused before moving: an extra depth over the cap, or no force readings at all. A 9 N
    air reading is no reason to stop.
  - Through the executor, the "contact" row carries the joints at the paper.
- **Slots.** The site is read from the site table and site.json (robot, address, domain per
  slot). A duplicate domain, an unknown slot, or a mounted slot whose robot is never driven
  is refused. Identity is verified, mismatched or unverified.
- **Jobs and thresholds** (through the stand-in server):
  - a job raises the collision thresholds and restores them after;
  - thresholds the robot will not take are noted in the first row, and the job runs;
  - the first row carries the pen with its press and the robots;
  - refused before anything moves: a mode other than position, and a robot mismatch.
- **The pen** of the header is recorded in the first row, or the rig file's when the header
  has none, and the row says which.
- **serve.** Against the stand-in server: report, two runs (a drawing-like job and a
  calibration job with a touch), recover, and recover of an arm that is not there. Every
  command is acknowledged in order. The server's calibration file arrives and the stale local
  one is removed, so the calibrated job's digests agree and it runs. The touch's contact row
  sits on the calibrated fake paper. "where" rows come while idle. A job planned for another
  rig gives a "run refused" row, and the process goes on. The stack keeper restarts a dying
  child after 0.05, 0.1, 0.2 s and stops a running one with SIGINT in under 3 s. Stacks are
  launched as `taskset -c <core>` (no chrt). A park after a stack restart runs. A job whose
  start hangs in a call that never answers fails within the timeout, with a row naming the
  step, and never starts late.
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

## Only on the operator PC

- The `/**/node` wildcards in the controllers file on Jazzy.
- Anything with an arm: the sign and drift of the force estimate, the arming of the touch on
  a real descent, error recovery and the hardware component coming back, `taskset` on
  the stacks.
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
- No force control: the press is the plan's depth below the measured paper.

Mode B (joint impedance with the pen force) was removed on 2026-10-07; it lives at commit
cb6e958.
