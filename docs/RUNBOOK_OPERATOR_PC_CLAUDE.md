# Runbook for the Claude on the robot PC (the Dell next to the arms)

You are setting up and verifying the operator side of the Aris drawing rig, together with
Diemut, who is at the machine; Pete is not on site and answers messages. Work through the steps
in order. After every step, write the report in the form given at the end (what you ran, the
exact output, pass or fail) so Diemut can send it to Pete. Stop at the first failure you cannot
explain from the text here and report it; do not improvise a fix. Where a step needs hands at
the robot (pilot buttons, enabling buttons, Desk in the browser), Diemut does it, with the
emergency stop in her other hand; you tell her exactly what to do and wait.

## Ground rules — read twice

- **Nothing you do may move an arm.** The only things that move arms are `aris …` commands typed
  on the planning laptop by a person. You never run `ros2 control`, `ros2 topic pub`, libfranka,
  panda-py motion calls, or Desk actions that move or unlock an arm. Desk in the browser is
  for unlocking the arms and switching FCI on, done by Diemut, as before.
- The old repositories and workspaces on this machine — `~/RTff`, `~/motion_ws`,
  `~/motion_ws_runner`, `~/aris_orchestrator`, `~/ika_arm31`, `~/impedance_helpers`,
  `~/impedance_ws`, anything named `Aris_Kindt`, `rtff`, `pathway`, `ladder`, `posdraw` — are
  **not part of this system**. Do not read them for guidance, do not run anything from them, do
  not edit them. Any memory or earlier context you have about them does not apply. The one
  exception is step 3, which checks that a vendor patch they installed is present.
- Work only inside `~/aris-clean/aris`. Edit only the files this runbook names:
  `robot/site.json`, `site/aris_2026-10.json`, and the copied systemd unit under
  `/etc/systemd/system/`.
- Never commit, push, or change git branches.
- If a step's output differs from what is written here, that is a finding, not a problem to
  solve: report it.

## Step 1 — a clean copy of the code

```
mkdir -p ~/aris-clean && cd ~/aris-clean
git clone git@github.com:wernerpe/aris_smoketest.git aris && cd aris
git checkout aris3 && git log -1 --oneline
```

Report the commit line. (The planning laptop must be on the same commit; Pete compares.)

## Step 2 — the ROS environment that is already here

```
source /opt/ros/jazzy/setup.bash && echo ROS_DISTRO=$ROS_DISTRO
ls ~/ros2_ws/install/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 pkg prefix franka_description && ros2 pkg prefix franka_hardware
```

Expected: `ROS_DISTRO=jazzy`, both packages found. If `~/ros2_ws` does not exist or lacks the
franka packages, stop and report: the franka_ros2 workspace is a precondition Pete installs.

## Step 3 — the inverted-mount patch is present

```
grep -c mount_to_world $(ros2 pkg prefix franka_description)/share/franka_description/robots/fr3/fr3.urdf.xacro
```

Expected: a number greater than 0. `0` means the patch that lets the arms hang upside down is
missing from this workspace: stop and report. Do not copy anything from the old repositories
yourself.

## Step 4 — build the robot packages

```
cd ~/aris-clean/aris/robot/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=<libfranka build dir> 2>&1 | tail -40
source install/setup.bash
```

Expected: `Summary: 1 package finished` (the bringup package; there is no controller of our own
any more — the arms run the stock trajectory controller). A build error: report the first 40
lines verbatim and stop. Do not patch it.

## Step 5 — the Python side

```
cd ~/aris-clean/aris
python3 -m venv --system-site-packages .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime
pip install -e . && pip install -e robot && pip install pytest fastapi uvicorn httpx
python -c "import rclpy, aris, aris_robot; print('ok')"
python -m pytest robot/tests -q -m "not slow"
```

Expected: `ok`, then `N passed` (around 55) and no `failed`. Every later step assumes this
terminal has run `source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash;
source ~/aris-clean/aris/robot/ros2_ws/install/setup.bash; . ~/aris-clean/aris/.venv/bin/activate`.

## Step 6 — the site files

Two files. Change only what is named.

1. `robot/site.json`: set `"server_url"` to `http://<planning laptop address>:8420` (Pete gives
   the address). Leave everything else.
2. `site/aris_2026-10.json`: the rows `2L` and `2R` name the two live robots and their
   control-box addresses. Check each address answers and which robot it is:
   ```
   ping -c 2 192.168.50.14; ping -c 2 192.168.50.15
   curl -sk https://192.168.50.14/ -o /dev/null -w "%{http_code}\n"
   curl -sk https://192.168.50.15/ -o /dev/null -w "%{http_code}\n"
   ```
   Expected: both ping, both answer `200` (that is Desk). Which robot hangs in slot 2L (the L
   side of the table — Pete says which side) and which in 2R **cannot be known from this PC**:
   report what answers at each address and let Pete confirm the assignment; change the two
   rows only if he says so. Set `"sure": true` on a row only when Pete has confirmed it.

## Step 7 — the one hardware fact to verify (Diemut at the e-stop, nothing moves by itself)

The calibration has Diemut hand-guide an arm while our stack keeps reading its joints. That
works when the arm is idle under FCI with no controller active. Check it on ONE arm, with
serve stopped and that arm's stack started by hand (see the README for the launch line), Desk:
unlocked, FCI on:

```
ros2 control switch_controllers --deactivate fr3_arm_controller
ros2 topic echo /joint_states --field position
```

Diemut pinches the enabling buttons on that arm and moves it a little, then lets go. Expected:
the numbers follow her hand while she guides and stand still when she lets go. Report the
first and last lines you saw. If the numbers freeze during guiding, stop and report: the
calibration design depends on this. Then `ros2 control switch_controllers --activate
fr3_arm_controller` and stop the stack.

## Step 8 — removed

Desk is not used by this system any more (no login, no control token, no mode switching).
`robot/secrets.json` is not needed. Unlocking the arms and switching FCI on stay manual, in
the browser, as before.

## Step 9 — fake hardware, end to end, nothing real moves

On the planning laptop Diemut starts `aris serve --config config/two_arms --driver robot
--host 0.0.0.0 --uncalibrated` (section 4 of her document). Here:

```
cd ~/aris-clean/aris && aris-robot --fake serve --config config/two_arms
```

Leave it running. From the laptop Diemut runs `aris arms` (both slots must appear) and then
`aris draw tests/data/server_small.json --air 30` (a drawing flown in the air on fake
hardware). Report the last 30 lines of `out/operator/serve.log` and whether the laptop's report
ended in PASS. Then stop serve with Ctrl-C.

## Step 10 — install the resident process

```
sudo cp ~/aris-clean/aris/robot/aris-robot.service /etc/systemd/system/
sudo nano /etc/systemd/system/aris-robot.service
```
Set `User=` to this PC's ROS user, replace every `~/aris3` by `/home/<user>/aris-clean/aris`,
and add `--config config/two_arms` before `serve` in `ExecStart`. Save. Then:
```
sudo systemctl daemon-reload && sudo systemctl enable --now aris-robot
sleep 20 && systemctl status aris-robot --no-pager | head -20
tail -30 ~/aris-clean/aris/out/operator/serve.log
```
Expected: `active (running)`; the log shows one stack started per live slot and "operator
started". From the laptop, `aris arms` must list `2L` and `2R` with joint angles. Report all
three outputs.

**Your job ends here.** The first real motions (`aris park`, `aris touchoff`, `aris draw
--air`, then `aris mark` and `aris draw`) are typed on the laptop by Diemut, in that order, by
Stage 0 of her document, with the emergency stop in her hand. If she asks you to watch, the
places to look are `out/operator/serve.log` and `out/operator/rows.jsonl`; `aris-robot
identify` is the one read-only command you may run while serve is stopped.

## How to report each step

```
Step N — <name>: PASS | FAIL | FINDING
ran: <the commands>
output: <verbatim, trimmed to the relevant lines>
note: <one sentence, only if something differs from the runbook>
```

Write the report after every step, not at the end; Diemut sends it to Pete.
