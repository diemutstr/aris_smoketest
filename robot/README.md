# robot/ — the operator PC side

This folder is what runs next to the arms, on the operator PC (ROS 2 Jazzy, the vendored
franka_ros2). The planning PC runs the drawing server and writes the queues of motions; this
side fetches them, runs them on the arms, and posts back what happened. Same repository, same
commit on both machines. `docs/modules/robot.md` explains how it works; this file is only the
steps.

```
robot/
  site.json                    this PC: the server, which slots are mounted, each slot's
                               force sign and real-time core, the touch, the thresholds
  aris_robot/                  the Python package (the command `aris-robot`)
  ros2_ws/src/aris_bringup     the launch file of one arm and the controller settings
  tests/                       everything that runs without ROS
```

**One process runs here, `aris-robot serve` (section 5), started at boot by systemd. Nobody
touches this PC after that: every command (draw, park, calibrate, touch-off, recover, report)
is given on the planning PC with `aris ...`, and the only thing on this side is the e-stop
(and, for the mark calibration, the arm's light and its pilot buttons: section 6).**
The first run is: build (sections 1 to 3), fill in the site (section 4), install and start
serve (section 5), then everything from the planning PC.

Nothing below has been run on the operator PC yet. Where a step can fail for a reason we could
not check here, it says so.

## 0. Before anything moves

- One person's only job is the physical e-stop. Nothing in this software is the abort.
- The arms must be unlocked and in FCI mode in Desk. The pen holder is clamped by the gripper
  fingers: **never start the franka gripper node** (homing opens the fingers). The launch file
  here does not start it.
- The collision thresholds in force are whatever the arm was last given (the old stack raised
  them from 20 N to 40 N before every pass). Write down which are in force.

`robot/generated/` (the launch arguments serve writes) and `out/robot_jobs/` (the local copies of
jobs) are gitignored, and so is `out/operator/`, serve's log directory.

## 1. Get the code

```
cd ~ && git clone git@github.com:wernerpe/aris_smoketest.git aris3 && cd aris3 && git checkout aris3
```

(Use the same commit as the planning PC: `git log -1` on both.)

## 2. Build the ROS workspace

The workspace builds on top of the franka_ros2 workspace already on the operator PC
(`~/ros2_ws`, with libfranka and the operator patches deployed). Check the patches first: the
arms hang upside down, and the launch passes the mount to the robot description.

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
grep -c mount_to_world $(ros2 pkg prefix franka_description)/share/franka_description/robots/fr3/fr3.urdf.xacro
#   0 means the operator patch is missing: copy Aris_Kindt/operator_franka_patches/fr3.urdf.xacro
#   into franka_description (src and install), as its README says
cd ~/aris3/robot/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DFranka_DIR=<libfranka build dir>     # on the Dell: where libfranka 0.21 was built
source install/setup.bash
```

The workspace holds one package, `aris_bringup`: the launch file of one arm and its
controller settings (the stock joint trajectory controller and the two broadcasters).

## 3. The Python side

rclpy comes from the ROS install, so the virtual environment sees the system packages. Source
ROS and the workspace first, in every terminal that runs `aris-robot`.

```
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
source ~/aris3/robot/ros2_ws/install/setup.bash
cd ~/aris3
python3 -m venv --system-site-packages .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime
pip install -e . && pip install -e robot
python -c "import rclpy, aris_msgs.msg, aris, aris_robot; print('ok')"
python -m pytest robot/tests -q -m "not slow"      # needs: pip install pytest fastapi uvicorn httpx
```

## 4. Slots, the site table and the site file

Arms are named by their slot on the frame: `1L 1R 2L 2R 3L 3R` (row 1 at the −y end, L at −x;
DESIGN 4c). Every command, row and file here uses the slot (rows with
`"arm": "2R"`, namespace `arm_2R`). The old robot ids (13, 17, 31, 71, 2, 97) live on only in
the site table.

- **The site table**, `site/aris_2026-10.json` at the repository root, says which robot hangs
  in which slot today: per slot the robot (`fr3-71`), its control-box IP, its DDS domain and,
  once known, its serial. Edit it when a robot is moved. The DDS domain must be an integer,
  so it is not derived from the slot; it is taken from this table. Today it is the old robot
  id, as the live stacks use it.
- **`robot/site.json`** is about this PC. It holds the server's address (`server_url`, the
  planning PC, port 8420) and the site table it uses (`site_table`). Per slot it holds
  `mounted` (true only for a slot whose arm hangs there and answers), `force_sign` and
  `rt_core`. It also holds the touch (`touch`) and the collision thresholds (`collision`).
  One source for each fact: nothing in the site table is repeated here.

How hard the pen presses is the plan's: it runs the pen's `press_m` (config/rig.json `pens`)
below the paper. The job header carries the pen, and the first row records it.

**Which robot is it?** `aris-robot identify` and serve's first row report, per slot, the
robot the table names and what answers at its address (the robot mode, the address the stack
was launched with, which park it stands nearest). A job for a slot whose robot is not the one
the table names is refused. The check needs a serial, and FCI and ROS report none. Until a
serial can be read, every slot is reported as "unverified". Identify by the address and by
which park the arm stands at.

**Tracking.** One mode: joint position control (Pete, 2026-10-07). Every motion, the
drawing ones included, flies through the stock trajectory controller exactly as planned.
The press is geometric: the plan runs the pen's `press_m` (3.5 mm for 4H graphite) below the
paper. A job header asking for another mode is refused. For each job except a mark job, the
collision thresholds are raised to site.json `collision.job` (40 N, as the old stack did
before every pass), with a service call at job start. After the job they go back to
`collision.normal`. If the robot will not take them, that is noted in the first row, and the
job runs at the normal thresholds.

## 5. The resident process: `aris-robot serve`

Install it once and leave this PC alone:

```
sudo cp ~/aris3/robot/aris-robot.service /etc/systemd/system/
sudo nano /etc/systemd/system/aris-robot.service     # User= and the paths, if not operator/~/aris3
sudo systemctl daemon-reload && sudo systemctl enable --now aris-robot
systemctl status aris-robot
```

What the unit needs: the operator's user (it owns `~/aris3`, `~/ros2_ws` and the venv);
ROS 2 Jazzy, the franka_ros2 workspace and `robot/ros2_ws` sourced (the unit does it, in
that order); the venv's `aris-robot`; real-time limits for the control loop (`LimitRTPRIO`,
`LimitMEMLOCK`, set in the unit). `Restart=always` brings it back after a crash and at every
boot.

What it does, on its own:
- writes the launch arguments (`robot/generated/arm_<slot>.json` and `_controllers.yaml`) and
  starts one ROS stack per mounted slot (`ros2 launch aris_bringup arm.launch.py`, namespace
  `arm_<slot>`, the slot's DDS domain from the site table), each in its own process group, with
  its output in `out/operator/stack_arm<slot>.log`. A stack that dies is
  started again after 1 s, then 2, 4, ... up to a minute, and every death is reported.
- asks the drawing server for work (`GET /operator/next`, waiting 30 s at a time), so this PC
  opens no port. "run" runs a job (a drawing, a park, a calibration or a touch-off: the
  server plans parks from the positions serve reports); "recover" recovers one arm after a
  person has looked (never in user stop or guiding); "report" reports every arm.
- before each job, fetches the calibration files from the server into `config/calibration/`
  (the server owns them; a local file the server does not have is removed).
- every 10 s while no job runs, reports where every arm stands.
- says everything as rows to the server and to `out/operator/rows.jsonl`; its own log is
  `out/operator/serve.log`. It needs no terminal.

The config directory must hold the same rig file as the server's. Add
`--config config/two_arms` before `serve` in the `ExecStart` line when the server runs with
that one.

**Fake hardware**: `aris-robot --fake serve` (add `--fake` to the unit's command line). The
stacks start on fake hardware, with the trajectory controller on the position interface
(fake hardware ignores torques). A touch works as on a real arm: the fake hardware has no force estimate, so a fake paper stands
in for it, at `--fake-paper-mm` above the nominal paper (default 0: the touch meets it at the
planned end of its descent). `aris-robot serve --sim-speed inf` runs without ROS at all, on
simulated arms that also touch a fake paper. It is useful for trying the server's commands on
any PC.

Not checked here: that franka_description's fake hardware offers a position command interface
(with fake hardware the trajectory controller is set to positions).

**The first jobs**, all from the planning PC: `aris touchoff <slot>` first (its report shows
the sign of the force onset: it must match the slot's `force_sign` in site.json), then a
drawing in the air (planned 30 mm above the paper), then on paper. A stale position cannot
move an arm: a motion that does not start within 0.005 rad of the arm's actual joints is
refused before it moves, with a row saying which joint is how far off, and the arm holds.

**If serve is down** (or the server is): `aris-robot identify` is a read-only check. Per slot
it reports the robot the site table names, whether its address and DDS domain answer, the
robot mode, the address the stack was launched with, and which park the arm stands nearest.
It needs the stacks running, and it moves nothing. `systemctl status aris-robot` and
`out/operator/serve.log` say why serve stopped. To look at one arm with the ROS tools, set
its domain: `ROS_DOMAIN_ID=71 ros2 control list_controllers -c /arm_2R/controller_manager`.

## 6. Calibration on the operator PC

The mark calibration (`aris mark` on the planning PC, DESIGN 6) has a person seat the pen on
taped spots. Its arms are flown by a second driver (`aris_robot/calib.py`) on libfranka
directly, through panda-py, with Franka Desk for the modes and the pilot buttons. For each
arm's turn serve stops that arm's ROS stack, so a mode switch never reaches a running ROS
controller. When the arm's turn is over, the stack is started again; the other arms keep
theirs, parked.

**Install** (once, in the same venv). The panda-py wheel is built against one libfranka
version, and it must match the robots. In Desk, Settings → System shows the robot's system
version; Franka's compatibility table names the libfranka for it. On site (2026-10-06): FR3
system 5.9 → libfranka 0.21.x → the release wheel `panda_python-1.1.1+libfranka.0.21.3`
(for Python 3.12) from https://github.com/JeanElsner/panda-py/releases, not PyPI's default
build:

```
pip install panda_python-1.1.1+libfranka.0.21.3-cp312-cp312-manylinux_2_17_x86_64.whl
pip install -e "robot[calib]"           # the pin panda-python==1.1.1 accepts that wheel
```

Then restart serve. Without panda-py, serve refuses mark jobs with a row saying so.

**Desk control.** Only one holder can control Desk at a time. A browser with Desk open, or a
token left by an earlier attempt, holds it. At the start of an arm's turn the calibration
driver takes control. If someone else holds it, a row says "press circle on the pilot", and
the driver waits 60 s for that press. It keeps control for the whole turn and always releases
it at the end of the turn, including after a failure. Then FCI is switched on, and only then
does libfranka connect. Close Desk in the browser once FCI is on.

**The secrets file**: `robot/secrets.json` (gitignored, never committed), Desk's login per
robot, or one entry for all:

```
{"default": {"username": "...", "password": "..."},
 "fr3-71":  {"username": "...", "password": "..."}}
```

**What the person does**: nothing at a computer. When an arm reaches a spot it holds, and
its light turns white. Pinch the enabling buttons, seat the pen tip on the spot, let go (the
arm holds), and press a pilot button: ✓ registered, ○ skip this spot. ✗ means "I want to redo this": the
arm stays with you (light white), so seat it again and press ✓ or ○. The arm takes itself
back (light blue). Once it stands still, it reads its joints, lifts the pen straight up a few
centimetres, and returns to its hover. No button within 10 minutes: that arm stops there, and
the job says why on the planning PC.

Found on the arms (2026-10-06):
- The mode switch is `POST /desk/api/operating-mode/programming` (or `/execution`), with an
  empty body, the header `X-Control-Token`, and a 200 answer. This is `robot/site.json`
  `desk.mode_endpoint`. Every Desk call is a row with its HTTP status, so a change shows at
  once.
- The pilot buttons are check, cross, circle, left, right, up and down.
- Leaving execution mode or releasing control switches FCI off.

While an arm is handed over, or its stack is down, it has no reading. Rows say `"q": null`
with the reason, and never zeros. A mark job sets no collision thresholds, because its arm's
stack is down.

**Not checked without a robot** (only on the fakes here): the whole turn on this firmware
with panda-py 1.1.1: take control, FCI on, connect, guide, release; `listen` and
`stop_listen`; that the joints read the same standing still after the hand-back.

**Real-time cores.** Each mounted slot's stack runs on its own isolated core at SCHED_FIFO
95: serve launches it as `chrt -f 95 taskset -c <rt_core> ros2 launch ...`. The cores are
site.json `rt_core`: 2L → 16, 2R → 17, 1L → 18, 1R → 19, and the priority is `ros.rt_priority`.
3L (the floor arm) and 3R (empty) get no stack at all: the site table marks them
`controlled: never` / `absent`, and site.json refuses to mount them. Link drops ended on
2026-10-07 once each arm's loop had its own core and the network card's interrupts were kept
off those cores. The Dell's isolated cores are 8-19 and 28-39.

**One-time host step: the network card's interrupts on cores 12, 13, 32, 33** (as root, once;
again after a kernel or NIC change):

```
sudo systemctl disable --now irqbalance          # it would move them back
grep -E 'enp|eno|eth' /proc/interrupts           # the NIC's IRQ numbers, first column
# give each of the NIC's IRQs one of the four cores, in turn, e.g. for IRQs 140..143:
echo 12 | sudo tee /proc/irq/140/smp_affinity_list
echo 13 | sudo tee /proc/irq/141/smp_affinity_list
echo 32 | sudo tee /proc/irq/142/smp_affinity_list
echo 33 | sudo tee /proc/irq/143/smp_affinity_list
cat /proc/irq/140/smp_affinity_list              # check
```

`/proc/irq` settings do not survive a reboot. Put the `echo` lines into
`/etc/rc.local`, or a oneshot systemd unit that runs before `aris-robot.service`, so they
are applied at every boot.

## Defaults worth knowing

- On the arms (2026-10-06):
  - Start tolerance 0.03 rad, from the job header or rig.json.
  - Trajectory goal tolerance 0.03 rad, `goal_time` 3 s.
  - After a move, the arm is standing when every |qd| ≤ 0.005 rad/s; the joints are read
    after waiting at most 1.5 s for that.
  - Recovery (`aris recover <slot>` on the planning PC; 2026-10-07) runs these steps, each
    reported in a row:
    1. Franka error recovery.
    2. The hardware component inactive, then active.
    3. The trajectory controller and both broadcasters active.
    4. The joint states must be fresh: their stamp advances over 1 s.
    5. If they are not fresh within 10 s, serve restarts that arm's stack and checks again.

    After that a park (`aris park`) flies without restarting anything. A joint reading older
    than 2 s is no reading: rows say `"q": null` with "stale joint states (last 7.3 s ago)".
  - After a job the trajectory controller stays active and holds.
  - A link drop (`communication_constraints_violation`) is recovered by itself, at most once
    per 2 minutes per arm (site.json `execution.auto_recover`).
  - Touch: onset at 3 N over 15 readings, cap 6 N. The detector arms only once the descent
    runs at constant speed (its acceleration ramp over, plus 0.1 s), and takes its zero
    then. A trip during the ramp is the descent's own jolt, not the paper; it tripped 60 mm
    in the air on 2026-10-07, and the descent now carries on.
  - A job starts within 10 s or fails with a row naming the step it was stuck in. A stuck
    start never moves an arm later.
  - `force_sign` is −1 on the hung arms.
