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
mode is refused. For every job except a mark job, the collision thresholds are set to site.json
`collision.job` (40 N) by a service call to franka_hardware's `set_full_collision_behavior`
at job start, before anything moves, and back to `collision.normal` after the job. A service
call per job is cleaner than a launch parameter: the thresholds belong to the job, and the
launch has no such parameter. If the robot will not take them, the first row says so and the
job runs at the normal thresholds.

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

## The mark calibration: its own driver (`calib.py`, `desk.py`, `handover.py`)

Pete, 2026-10-05: the most mature software for this, its own driver so that a mode switch
never reaches a running ROS controller, and nobody looks at a computer during it.

- **`CalibArm`** implements the same verbs on libfranka through panda-py (optional extra
  `calib`). `move` streams our cubic q(t), qd(t) at 1 kHz into panda-py's joint position
  controller. It does not use `move_to_joint_position`, which times the motion itself; the
  checker's verdict holds at our timing only. `draw` and `touch` are refused ("not this
  driver"). `recover` is libfranka's error recovery, refused only in user stop.
- **`guide`**: FCI off, Desk to programming (the light goes white), then wait for a pilot
  button: ✓ check or ○ circle end it; ✗ cross means "I am redoing this seat", so the arm
  stays with the person and the wait goes on (row "guide: button cross, waiting"). Then Desk
  to execution and FCI on, and the connection is
  made again. The joints are read until two reads 0.5 s apart agree within 1e-4 rad: that is
  the sample. Then the pen goes straight up 3 cm (2 or 1 cm where it cannot reach), and a
  straight joint move goes back to the hover (refused beyond 0.5 rad), so the queue's next
  motion starts where it was planned. Done with q = the sample and why = the button; the
  executor writes the "registered" row (q, button, mark). No button in 10 minutes: failed,
  the arm in execution mode, holding.
- **`Desk`** is a four-call interface: `mode`, `buttons`, `unlock`/`lock`, `fci`. `PandaDesk`
  uses panda-py's Desk; `SimDesk` is scripted for the tests. The operating-mode request
  (panda-py has none) goes through panda-py's own request helper to `robot/site.json`
  `desk.mode_endpoint` (method, path, bodies; today a guess), and every Desk call is a row
  "desk: <call>" with its HTTP status, so a wrong endpoint shows at once and is fixed in the
  config. Credentials come from
  `robot/secrets.json` (gitignored).
- **The hand-over in serve**: a job whose header has `"kind": "mark"` runs with a `Switch`
  per arm. Before an arm's phase, serve pauses that arm's stack: SIGINT to its process group,
  waits until it has exited, and does not restart it. Only then does it connect the
  calibration driver. After the phase it disconnects, resumes the stack, and waits until the
  stack reports the joints. One FCI connection per robot at a time holds because serve is the
  only process that starts either, does the two steps in that order, one job at a time, and
  gives up the phase rather than connect when a stack does not exit. libfranka refuses a
  second connection anyway.
- **Rows**: "calibration driver: stack stopped / connected / disconnected / stack back",
  "guide: handed over", "guide: button x", "guide: taken back" (or "guide: no button"), and
  the executor's "registered".

**Field report applied (2026-10-06).** No mark touch was registered in 15 jobs because of
the Desk/FCI hand-over. The rules now:
- **Desk control**, once per arm turn. If the browser or an old token holds it, the driver
  waits up to 60 s for a circle press, with a row saying so. Control is always released at
  the end of the turn and on every failure.
- **FCI order**: control → FCI on → libfranka connects. Per guide: programming (FCI goes
  off) → button → execution → FCI on → reconnect → 1.5 s settle → standstill.
- **Mode switch**: `POST /desk/api/operating-mode/<mode>`, no body, `X-Control-Token`
  header (site.json `desk.mode_endpoint`). The buttons are check, cross, circle, left,
  right, up, down; `listen` is ended with `stop_listen`.
- **No zeros**: an arm without a reading (FCI off, stack down) is `"q": null` with a reason
  in every row and in `where` (`where_missing`). The calibration driver is the source of
  joints for its slot during its turn.
- **Mark jobs** set no collision thresholds.
- **Tolerances**: the start tolerance comes from the header or rig (0.03 rad); goal 0.03 rad
  and 3 s.
- **Standstill and recovery**: after a move, |qd| ≤ 0.005 rad/s within 1.5 s. Recovery is
  reflex first, then the hardware component inactive → active. A link drop is
  auto-recovered once per 2 minutes per arm.
- **Touch**: 3 N over 15 readings, cap 6 N; force sign −1 on the hung arms.
- **Cores**: each stack is pinned to its own isolated core (`taskset -c rt_core`).

**Recovery (2026-10-07).**
- `recover` is error recovery → hardware component inactive → active → the trajectory
  controller and broadcasters active → the joint states fresh (stamp advancing over 1 s) → if
  not within 10 s, serve restarts the arm's stack and checks again. Every step is a row.
- A reading older than 2 s is stale: `q` null, with "stale joint states (last x s ago)".

**2026-10-07, second round.**
- **Touch.** The touch detector arms at constant descent speed: the acceleration ramp plus
  0.1 s, from the planned timing. Its zero is the mean of 20 readings taken then; contact is
  3 N above it over 15 readings. A trip before arming (the descent's own jolt) is counted, not
  taken. The cap (6 N over the hover's air zero) counts from the first reading. An extension
  past the planned end re-arms after its own ramp and keeps the first zero.
- **Cores.** Stacks run as `chrt -f 95 taskset -c <rt_core>` (16-19).
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
trajectory controller: the encoders say where the paper is, the force only when. The air zero
is taken standing at the hover. The arm flies the descent half of the motion at its own slow
timing. The detector arms only once the descent runs at constant speed (its acceleration ramp
plus 0.1 s). It then takes its own zero, the mean of 20 readings. Contact is 3 N above that
zero over 15 readings in a row. The trajectory is then cancelled, and the joints of the first of
those readings are the answer. A trip before arming is the descent's own jolt: it is counted,
and the descent goes on. If the planned end comes without contact, the arm goes straight on in
the same direction for the motion's `extra_depth` (at most 30 mm), at 2 mm/s, the hand keeping
its orientation. This flight arms after its own ramp and keeps the first zero. Then it gives up
with "no contact within … mm". In both cases it flies back to the hover along the path flown.
Over 6 N above the hover's air zero, at any reading, it stops and holds where it is, flies no
way back, and the arm refuses to move until it is recovered. With no force readings at all, the
touch is refused before moving. The executor logs a "contact" row with the joints. The first
`aris touchoff <slot>` from the planning PC also checks the force sign. On fake hardware a fake
paper stands in for the force estimate.

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
  - Paper 5 mm high, at the plan, and 4 mm low: contact read where 3 N lies (0.6 mm in, within
    0.25 mm); back at the hover to 1e-9.
  - Paper 12 mm low: the extension goes on straight (within 20 µm) at no more than 2 mm/s and
    finds it.
  - Paper 50 mm low: it gives up after exactly the planned descent plus 20 mm and comes back.
  - Steel instead of paper: the cap stops and holds, with no way back flown.
  - A 5 N jolt at the start of every flight is not taken for the paper; the contact is found
    where it is.
  - Refused before moving: an extra depth over the cap, or no force readings at all. A 9 N
    air reading is no reason to stop.
  - Through the executor, the "contact" row carries the joints at the paper.
- **Slots.** The site is read from the site table and site.json (robot, address, domain per
  slot). A duplicate domain, an unknown slot, or a mounted slot whose robot is never driven
  is refused. Identity is verified, mismatched or unverified.
- **Jobs and thresholds** (through the stand-in server):
  - a job raises the collision thresholds and restores them after; a mark job does not;
  - thresholds the robot will not take are noted in the first row, and the job runs;
  - the first row carries the pen with its press and the robots;
  - refused before anything moves: a mode other than position, and a robot mismatch.
- **Calibration driver** (a fake panda-py FCI and `SimDesk`): a guide round trip with each
  button. The sample is the person's pose, the Desk calls in order (FCI off, programming,
  buttons, execution, FCI on), the connection made again, a 3 cm lift then the hover, and the
  three guide rows. The standstill check waits out a settling arm and fails on a restless
  one, which then holds. No button: failed, in execution mode, no move. Draw and touch
  refused; move, recover, and the user-stop refusal. Through the executor: the "registered"
  row with button, mark and q. Serve with a mark job on two arms: pause 2R → calibration →
  resume 2R → pause 3R → … in that order and in time, every arm registered, back at its
  park. A mark job without the calibration driver is refused. The stack keeper pauses a
  running stack, keeps it down, and resumes it.
- **The pen** of the header is recorded in the first row, or the rig file's when the header
  has none, and the row says which.
- **serve.** Against the stand-in server: report, two runs (a drawing-like job and a
  calibration job with a touch), recover, and recover of an arm that is not there. Every
  command is acknowledged in order. The server's calibration file arrives and the stale local
  one is removed, so the calibrated job's digests agree and it runs. The touch's contact row
  sits on the calibrated fake paper. "where" rows come while idle. A job planned for another
  rig gives a "run refused" row, and the process goes on. The stack keeper restarts a dying
  child after 0.05, 0.1, 0.2 s and stops a running one with SIGINT in under 3 s. Stacks are
  launched as `chrt -f 95 taskset -c <core>`. A park after a stack restart runs. A job whose
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
  a real descent, error recovery and the hardware component coming back, `chrt`/`taskset` on
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
