# robot/ — the operator PC side

This folder is what runs next to the arms, on the operator PC (ROS 2 Jazzy, the vendored
franka_ros2). The planning PC runs the drawing server and writes the queues of motions; this
side fetches them, runs them on the arms, and posts back what happened. Same repository, same
commit on both machines. `docs/modules/robot.md` explains how it works; this file is only the
steps.

```
robot/
  site.json                    addresses, domains, which arms are mounted, the server, the pen force
  aris_robot/                  the Python package (the command `aris-robot`)
  ros2_ws/src/aris_msgs        the reference and status messages
  ros2_ws/src/aris_controllers the controller aris_joint_impedance_controller (C++)
  ros2_ws/src/aris_bringup     the launch file of one arm and the controller settings
  tests/                       everything that runs without ROS
```

**In normal operation one process runs here, `aris-robot serve` (section 8), started at boot
by systemd. Nobody touches this PC after that: every command (draw, park, calibrate, recover,
report) is given on the planning PC with `aris ...`, and the only thing on this side is the
e-stop.** Sections 5 to 7 are the hardware-day steps that come before it. The tools they use
(`aris-robot run, park, jog, touch, identify, switch, recover`) stay for that, but they are
not needed once serve runs. Stop serve (`sudo systemctl stop aris-robot`) before using any of
them on the same arm.

Nothing below has been run on the operator PC yet. Where a step can fail for a reason we could
not check here, it says so.

## 0. Before anything moves

- One person's only job is the physical e-stop. Nothing in this software is the abort.
- The arms must be unlocked and in FCI mode in Desk. The pen holder is clamped by the gripper
  fingers: **never start the franka gripper node** (homing opens the fingers). The launch file
  here does not start it.
- The collision thresholds in force are whatever the arm was last given (the old stack raised
  them from 20 N to 40 N before every pass). Write down which are in force.

`robot/generated/` (from `aris-robot bringup`) and `out/robot_jobs/` (the local copies of
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

## 4. The site file

Edit `robot/site.json`: the server's address (`server_url`, the planning PC, port 8420), and
per arm its control-box IP, its DDS domain (the arm id), and `mounted`: true only for the arms
bolted in their places and switched on. The IPs in the file are from the 2026-09-09 briefing;
arm 71 had no IP on 2026-09-15. Nothing else about this site is typed anywhere else.

## 5. Launch files

```
aris-robot bringup                 # writes robot/generated/arm_<id>.json and _controllers.yaml
aris-robot --fake bringup          # the same, for fake hardware
```

It prints one `ros2 launch` line per arm. Each arm runs in its own terminal, in namespace
`arm_<id>` on DDS domain `<id>`:

```
ros2 launch aris_bringup arm.launch.py args:=$HOME/aris3/robot/generated/arm_31.json
```

The launch refuses an arm that is not mounted (unless the files are for fake hardware). To look
at one arm with the ROS tools, set its domain: `ROS_DOMAIN_ID=31 ros2 control list_controllers
-c /arm_31/controller_manager` should show `fr3_arm_controller` and the broadcasters active and
`aris_joint_impedance_controller` inactive.

## 6. First run: fake hardware, one arm

```
aris-robot --fake bringup
ros2 launch aris_bringup arm.launch.py args:=$HOME/aris3/robot/generated/arm_31.json
aris-robot --fake identify
aris-robot --fake switch 31 impedance && aris-robot --fake switch 31 trajectory
aris-robot --fake jog 31 --joint 7 --delta 0.05
```

Not checked here: that franka_description's fake hardware offers a position command interface
(the fake arm ignores torques, so with fake hardware the trajectory controller is set to
positions and drawing motions go through it, without pen force).

Then a job. The server on the planning PC must offer the four endpoints of
`aris_robot/remote.py` (not built there yet, see `docs/modules/robot.md`). Start a job there
(`aris draw ...`), take its id, then:

```
aris-robot run --job <id> --sim-speed inf     # no ROS at all: simulated arms, at once
aris-robot --fake run --job <id>              # fake hardware, real time
```

Every event appears in the job's log on the server (`aris status`), and a local copy of the job
is kept under `out/robot_jobs/<id>/`. A job runs once; a second run of the same id is refused.

## 7. A real arm

```
aris-robot bringup
ros2 launch aris_bringup arm.launch.py args:=$HOME/aris3/robot/generated/arm_31.json
aris-robot identify                 # address, domain, mode, which park the arm stands nearest
aris-robot jog 31 --joint 7 --delta 0.05
aris-robot park 31                  # only from within 0.05 rad of the park, straight
```

A park from further away is a planned job, planned from where the arms really stand. The
server knows that only from this PC: every run reports every mounted arm's joints at its start,
on every row about an arm, and at its end, and serve reports them every 10 s while idle. With
serve running, `aris park` on the planning PC is all it takes. Without serve:

1. Run anything with `aris-robot run` first (the last run's report is what the server plans
   from). If the arms were moved by hand since, the server's positions are stale.
2. `aris park` on the planning PC: it plans a park job from those positions.
3. `aris-robot run --job <that id>` here. It runs like any job, one arm per phase.

A stale position cannot move an arm: a motion that does not start within 0.005 rad of the
arm's actual joints (on every joint) is refused before it moves. The job then fails with a
"failed" row saying which joint is how far off, and the arm holds. That refused run reports
the true positions, so run `aris park` again and then the new job.

The first contact with paper is the touch, with the pen a few centimetres above the paper:

```
aris-robot touch 31 --depth 0.04 --extra 0.01
```

It goes straight down at 5 mm/s under position control (the trajectory controller), and on
for at most 10 mm more at 2 mm/s if it has not met the paper. It stops at the force onset
(1.0 N over the air reading), prints the joints, the pen tip and the air reading, and goes
back up the same way. Over 3 N it stops and holds where it is. This is the same touch the
calibration job uses (`aris calibrate <arm>` on the planning PC, with serve running). Check
the sign of the force (site.json `force.sign`) here: the reading must rise when the pen meets
the paper. Then the first job, in the air first (a drawing planned 30 mm above the paper), then on
paper.

If an arm faults or was stopped: look at it, clear the cause, then `aris-robot recover 31`
(franka error recovery, then the trajectory controller takes the arm again). The runner never
does this by itself: a failed arm holds, and the next job refuses an arm that is not able to
move. Never recover an arm in user stop (mode "user stopped") or guiding.

## 8. The resident process: `aris-robot serve`

Once the steps above work for every mounted arm, install serve and leave this PC alone:

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
- writes the launch files and starts one ROS stack per mounted arm (site.json), each in its own
  process group, with its output in `out/operator/stack_arm<id>.log`. A stack that dies is
  started again after 1 s, then 2, 4, ... up to a minute, and every death is reported.
- asks the drawing server for work (`GET /operator/next`, waiting 30 s at a time), so this PC
  opens no port. "run" runs a job exactly as `aris-robot run --job` does (a drawing, a park or
  a calibration); "recover" recovers one arm; "report" reports every arm.
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

## Defaults worth knowing

- Joint stiffness `[300 300 250 250 40 40 15]` Nm/rad: 170 to 300 N/m at the pen tip in the
  paper plane. While the pen is down, the stiffness along the paper normal is replaced by a
  soft spring, 100 N/m, critically damped (`k_normal`, `d_normal`).
- Pen force 0.7 to 1.0 N (graphite), cap 3.5 N, ramped in over the first 2 mm of a line; the
  force servo trims it from the force estimate with a 1 s time constant (on by default).
- The controller holds and reports when the stream runs dry for 20 ms, when a joint is 0.05 rad
  off its reference, or on `~/hold`.
