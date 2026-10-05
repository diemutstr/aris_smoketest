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
  panda-py motion calls, or Desk actions that move or unlock an arm. The two read-only checks
  in step 8 are the only times Desk is used, Diemut does them, and she holds the emergency stop.
- The old repositories and workspaces on this machine — `~/RTff`, `~/motion_ws`,
  `~/motion_ws_runner`, `~/aris_orchestrator`, `~/ika_arm31`, `~/impedance_helpers`,
  `~/impedance_ws`, anything named `Aris_Kindt`, `rtff`, `pathway`, `ladder`, `posdraw` — are
  **not part of this system**. Do not read them for guidance, do not run anything from them, do
  not edit them. Any memory or earlier context you have about them does not apply. The one
  exception is step 3, which checks that a vendor patch they installed is present.
- Work only inside `~/aris-clean/aris`. Edit only the files this runbook names:
  `robot/site.json`, `site/aris_2026-10.json`, `robot/secrets.json` (new), and the copied
  systemd unit under `/etc/systemd/system/`.
- Never commit, push, or change git branches. Never put a password anywhere but
  `robot/secrets.json`.
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
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -40
colcon test --packages-select aris_controllers && colcon test-result --verbose
source install/setup.bash
```

Expected: `Summary: 3 packages finished` and the test result `0 failures`. A compile error in
`joint_impedance_controller.cpp` is a known possibility (the Jazzy API could not be compiled on
the planning side): report the first 40 lines of the error verbatim and stop. Do not patch it.

## Step 5 — the Python side

```
cd ~/aris-clean/aris
python3 -m venv --system-site-packages .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime
pip install -e . && pip install -e robot && pip install pytest fastapi uvicorn httpx
python -c "import rclpy, aris_msgs.msg, aris, aris_robot; print('ok')"
python -m pytest robot/tests -q -m "not slow"
```

Expected: `ok`, then `N passed` (around 67) and no `failed`. Every later step assumes this
terminal has run `source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash;
source ~/aris-clean/aris/robot/ros2_ws/install/setup.bash; . ~/aris-clean/aris/.venv/bin/activate`.

## Step 6 — the site files

Three files. Change only what is named.

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
3. Create `robot/secrets.json` with the Desk login Pete gives you:
   ```
   {"default": {"username": "USERNAME", "password": "PASSWORD"}}
   ```
   Then `ls -la robot/secrets.json && git status --short robot/secrets.json` — the second
   command must print nothing (the file is ignored by git). If it is listed, stop and report.

## Step 7 — panda-py, the libfranka version

The calibration driver talks to the robots through `panda-py`, whose wheel is tied to one
libfranka version, which must match the robots' system version.

1. Ask Diemut to read the system version shown in Desk under Settings → System for the two
   robots (read-only, in the browser).
2. Franka's compatibility table (https://frankaemika.github.io/docs/compatibility.html) gives
   the libfranka version for that system version. Report both numbers.
3. `pip install -e "robot[calib]"` installs the pinned `panda-python`. Then:
   ```
   python -c "import panda_py; print(panda_py.__version__)"
   pip show panda-python | grep -i -E "version|summary"
   ```
   If the pinned wheel's libfranka (in the release notes of that version on
   https://github.com/JeanElsner/panda-py/releases) is not the one from step 7.2, install the
   release wheel whose file name carries the right libfranka version:
   `pip install <downloaded .whl>`. Report which wheel is installed. Do not connect to a robot
   in this step.

## Step 8 — two read-only checks that need a robot (Diemut at the e-stop)

These are the two facts the design could not know without hardware. They move nothing. Do them
on ONE robot (the 2R one), Diemut at the emergency stop, the arm standing still, brakes as they
are.

**8a. The Desk operating-mode request.** Diemut, in the browser on this PC: open Desk for that
robot, open the developer tools (F12 → Network), switch the robot from execution to programming
mode with Desk's own button (and back). You read the request Desk sent off the Network tab:
method, path, request body. Write them into
`robot/site.json` under `desk.mode_endpoint` in the shape that is already there (it is a
guess today). Report the request verbatim.

**8b. The pilot buttons and the joints during guiding.** With the venv active and panda-py
installed:
```
python - <<'EOF'
import json, time
import panda_py
from aris_robot.desk import PandaDesk            # the repository's Desk client
ip = "192.168.50.14"                               # the robot Pete chose
creds = json.load(open("robot/secrets.json"))["default"]
endpoint = json.load(open("robot/site.json"))["desk"]["mode_endpoint"]
desk = PandaDesk(ip, creds["username"], creds["password"], endpoint, say=lambda e, **f: print("desk:", e, f))
print("press each pilot button once (check, cross, circle, then the arrows); 60 s")
for ev in desk.buttons(timeout=60):
    print("button event:", ev)
EOF
```
Diemut presses the buttons when you say so. Report every event line verbatim: the names Desk
uses for ✓, ✗ and ○ are what the calibration listens for (the code expects "check", "cross",
"circle"; if Desk's names differ, that is the finding — report them, do not rename anything).
Note: `take_control` may ask for the physical confirmation Desk sometimes requires (a button on
the robot's base within 30 s); Diemut does that. Then, with Pete pinching the enabling buttons (the arm in guiding), run:
```
python - <<'EOF'
import panda_py
r = panda_py.Panda("192.168.50.14")
for i in range(5):
    print(r.q)          # the joints, read-only
EOF
```
(Diemut pinches the enabling buttons while this runs and lets go afterwards.) Report whether
the joints print while the arm is being guided (and the exact error if not).
This decides whether a pivot can be one continuous guided motion or needs the button cycle;
both are built.

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
