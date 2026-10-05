# robot/ — the operator PC side

This folder is what runs next to the arms, on the operator PC (ROS 2 Jazzy, the vendored
franka_ros2). The planning PC runs the drawing server and writes the queues of motions; this
side fetches them, runs them on the arms, and posts back what happened. Same repository, same
commit on both machines. `docs/modules/robot.md` explains how it works; this file is only the
steps.

```
robot/
  site.json                    this PC: the server, which slots are mounted, each slot's
                               force sign, the tare and contact thresholds, the touch
  aris_robot/                  the Python package (the command `aris-robot`)
  ros2_ws/src/aris_msgs        the reference and status messages
  ros2_ws/src/aris_controllers the controller aris_joint_impedance_controller (C++)
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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select aris_controllers && colcon test-result --verbose
source install/setup.bash
```

`colcon test` runs `core_test`: the controller's law, reference and holds without the robot
(the same test ran on the planning PC). If the build fails in
`joint_impedance_controller.cpp`, the likely places are the Jazzy API calls that could not be
compiled here: `get_optional()` on state interfaces, `RealtimePublisher::trylock/msg_/
unlockAndPublish`, `FrankaRobotModel`. The live Cartesian controller on this PC uses the same
calls; compare with it.

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
  `mounted` (true only for a slot whose arm hangs there and answers) and `force_sign`. It also
  holds the tare and contact thresholds (`force`), the touch (`touch`), and the collision
  thresholds for position tracking (`collision`). One source for each fact: nothing in the
  site table is repeated here.

How hard the pen presses (the force band, levels, cap, ramps, the servo, the press depth) is
a fact of pen and paper. It lives in `config/rig.json` (`pens`) on the planning PC, comes
with every job, and the runner applies the job's values. A job without them (an older one)
runs with this PC's rig file.

**Which robot is it?** `aris-robot identify` and serve's first row report, per slot, the
robot the table names and what answers at its address (the robot mode, the address the stack
was launched with, which park it stands nearest). A job for a slot whose robot is not the one
the table names is refused. The check needs a serial, and FCI and ROS report none. Until a
serial can be read, every slot is reported as "unverified". Identify by the address and by
which park the arm stands at.

**Tracking.** The job header says how the arms follow (DESIGN 4c):
- `position` (mode A, the default): every motion, the drawing ones included, flies through
  the stock trajectory controller exactly as planned. The press is geometric: the plan runs
  the pen's `press_m` (3.5 mm for 4H graphite) below the paper. For the job the collision
  thresholds are raised to site.json `collision.job` (40 N, as the old stack did before every
  pass), with a service call at job start; after the job they go back to `collision.normal`.
- `impedance` (mode B): lower, draw and lift under `aris_joint_impedance_controller` with the
  pen force (DESIGN 4b). It is chosen on the planning PC (`--tracking impedance` when the job
  is submitted); nothing changes here.

The calibration touch is under position control in both modes.

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
stacks start on fake hardware, which ignores torques. A drawing motion therefore goes through
the trajectory controller, without pen force, and the impedance controller is never used. A
touch works as on a real arm: the fake hardware has no force estimate, so a fake paper stands
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

**Install** (once, in the same venv): `pip install -e "robot[calib]"`. The panda-py wheel is
built against one libfranka version, and it must match the robots. In Desk, Settings →
System shows the robot's system version, and Franka's compatibility table names the libfranka
for it. If that is not the libfranka of PyPI's `panda-python` (the version pinned in
`robot/pyproject.toml`), install the wheel for it from
https://github.com/JeanElsner/panda-py/releases (its file name carries the libfranka version,
e.g. `panda_python-<version>+libfranka.<x.y.z>-cp312-...whl`) with `pip install <file>`.
Then restart serve. Without panda-py, serve refuses mark jobs with a row saying so.

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

**Not checked without a robot**:
- Desk's operating-mode request on this firmware. Its method, path and bodies are in
  `robot/site.json` (`desk.mode_endpoint`, today a guess): on day one switch the mode by hand
  in Desk with the browser's developer tools open, and copy the request there. Every Desk call
  is a row with its HTTP status ("desk: mode programming", 404 = wrong path);
- the pilot buttons' event names;
- whether panda-py's FCI connection must be made again after a mode change (the driver does
  so anyway);
- that the joints read the same standing still after the hand-back;
- the wheel's libfranka version against the robots'.

## Defaults worth knowing

- Joint stiffness `[300 300 250 250 40 40 15]` Nm/rad: 170 to 300 N/m at the pen tip in the
  paper plane. While the pen is down, the stiffness along the paper normal is replaced by a
  soft spring, 100 N/m, critically damped (`k_normal`, `d_normal`).
- Pen force 0.7 to 1.0 N (graphite), cap 3.5 N, ramped in over the first 2 mm of a line; the
  force servo trims it from the force estimate with a 1 s time constant (on by default).
- The controller holds and reports when the stream runs dry for 20 ms, when a joint is 0.05 rad
  off its reference, or on `~/hold`.
