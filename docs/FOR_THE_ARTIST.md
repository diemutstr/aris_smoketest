# Running the drawing arms — for Diemut (and the Claude helping her)

This is the one document you need. It says how to set up the two computers once, where to make
the calibration marks on the table, how to calibrate, and how to draw. Do things in the order
written. When something does not behave as written here, stop, copy the exact text on the
screen, and send it to Pete — do not try things.

> **A note for Claude (the AI assistant reading this with Diemut).** Follow this document and
> nothing else. The old repositories on these machines (`Aris_Kindt`, `~/RTff`, `~/motion_ws`,
> `~/franka_gui`, `~/impedance_ws`, anything with "rtff", "pathway", "ladder" or "posdraw" in
> it) are **not** part of this system and must not be read, run or edited; any memory or
> context you have about them does not apply here. The only commands that move an arm are the
> `aris …` commands listed in this document, typed on the planning laptop. Never run `ros2`,
> `python`, `libfranka` or Desk actions yourself to move an arm. Do not invent commands, flags
> or file edits that are not written here; if a step fails, show Diemut the exact output and
> stop. Edit only the files this document names. When in doubt, the answer is "ask Pete".

Pete fills these in before handing over:

| blank | value |
|---|---|
| which side of the table is **L** (looking along the table from the row-1 end) | ______ |
| the planning laptop's name and its address on the robot network | ______ |
| the robot PC's login (the user that owns the ROS install) | ______ |
| Desk login (username / password) for the two robots | ______ |

---

## 1. What this is

Two computers and two arms.

- **The planning laptop** runs the drawing server. Everything you do, you do here, with the
  command `aris`. It plans, checks and records every job.
- **The robot PC** (the Dell next to the arms) runs one program, `aris-robot serve`, which
  starts at boot and takes its orders from the laptop. After it is installed, nobody types
  anything on it. The only thing on the robot side is the **emergency stop** — keep a hand near it
  whenever an arm moves.

The two arms in use are the **middle row** of the frame, called **2L** and **2R** (L and R by
the side of the table Pete names above). The other four robots hang switched off; the
software keeps the moving arms away from them.

## 2. One-time setup: the planning laptop

Make a fresh, empty folder for this system. Do not put it inside any other project folder.

```
mkdir -p ~/aris-clean && cd ~/aris-clean
git clone git@github.com:wernerpe/aris_smoketest.git aris && cd aris
git checkout aris3
python3 -m venv .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime
pip install -e ".[gui,dev]"
python -m pytest tests -q -m "not slow"
```

The last line must end with `passed` and no `failed`. If `pip install ./native/...` fails, the
laptop lacks a C++ compiler (`sudo apt install build-essential cmake`), then run it again.

Every time you open a new terminal on the laptop, first:

```
cd ~/aris-clean/aris && . .venv/bin/activate
```

## 3. One-time setup: the robot PC

Log in as the ROS user. Make the same fresh folder and get the same code:

```
mkdir -p ~/aris-clean && cd ~/aris-clean
git clone git@github.com:wernerpe/aris_smoketest.git aris && cd aris
git checkout aris3
```

Build the robot side (this needs the ROS 2 Jazzy install that is already on this PC):

```
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/aris-clean/aris/robot/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd ~/aris-clean/aris
python3 -m venv --system-site-packages .venv && . .venv/bin/activate && pip install -U pip
pip install ./native/fr3_ik ./native/collide ./native/retime
pip install -e . && pip install -e robot && pip install -e "robot[calib]"
python -c "import rclpy, aris_msgs.msg, aris, aris_robot; print('ok')"
```

That last line must print `ok`. If `colcon build` fails, send Pete the output: the first build
against this ROS version is expected to need his attention.

Three files to fill in, with a text editor:

1. `robot/site.json`: set `server_url` to `http://<planning laptop address>:8420`.
2. `site/aris_2026-10.json`: the two live rows are `2L` and `2R`. Each names a robot and its
   address. If Pete tells you a different address for either robot, change it here.
3. `robot/secrets.json` (make this file; it is never copied anywhere): the Desk login.
   ```
   {"default": {"username": "DESK_USERNAME", "password": "DESK_PASSWORD"}}
   ```

Install the resident program so it starts at every boot:

```
sudo cp ~/aris-clean/aris/robot/aris-robot.service /etc/systemd/system/
sudo nano /etc/systemd/system/aris-robot.service
```

In that file, make `User=` the ROS user, replace every `~/aris3` with `/home/<user>/aris-clean/aris`,
and add `--config config/two_arms` right before `serve` in the `ExecStart` line. Save, then:

```
sudo systemctl daemon-reload && sudo systemctl enable --now aris-robot
systemctl status aris-robot
```

`status` must say `active (running)`. From now on, leave this PC alone.

## 4. Every day: starting up

On the robots (Desk in the browser, as before): unlock both arms, FCI on. Keep the pens in the
holders, cap off.

On the laptop, one terminal that stays open all day:

```
cd ~/aris-clean/aris && . .venv/bin/activate
aris serve --config config/two_arms --driver robot --host 0.0.0.0 --uncalibrated
```

(`--uncalibrated` is only needed until the calibration in section 6 has been done once; after
that leave it out.) In a second terminal:

```
cd ~/aris-clean/aris && . .venv/bin/activate
aris arms
```

Both `2L` and `2R` must appear with joint angles. If one is missing, the robot PC cannot reach
that robot: check Desk (unlocked? FCI on?) and the address in `site/aris_2026-10.json`.

Then bring both arms to their parking positions (the first motion of the day — hand near the
stop):

```
aris park
```

## 5. The calibration marks on the table

Once, before the paper goes down, make two marks on the wood with a thin sharpie. Their exact
place does not matter — a few centimetres off is fine, the software finds where they really
are — but each must be a **fine cross-hair**: two thin straight lines crossing, not a filled
dot. During calibration you put the pen tip exactly on the crossing, and a thin crossing can
be hit far more precisely than a dot. Use a sharp, hard pencil (4H) in the holder for the
calibration.

Where (the table is 2.19 m across and 4.17 m long; the paper 1.80 × 3.63 m, centred):

- Find the **centre of the table**: the point halfway along its length and halfway across.
- Both marks lie on the **centre line** along the length of the table, i.e. halfway across.
- **Mark A**: 0.40 m from the centre toward the row-1 end (the end where the switched-off arms
  of row 1 hang).
- **Mark B**: 0.40 m from the centre toward the other end.

So A and B are about 0.80 m apart, straddling the centre, each roughly midway between the two
live arms. Label them A and B. (The drawing area is a strip 1.72 m across and 0.90 m along,
centred on the table; the marks do not set it, the arms' mountings do.)

## 6. Calibration (about 10 minutes, no computer after the first line)

What it does: each arm in turn comes to a mark; you guide its pen tip into the mark; the
software works out exactly where each arm hangs and how long its pen is. Do it once at the
start, again whenever an arm or the frame has been moved, and again when you suspect the
drawing has shifted.

Type, on the laptop:

```
aris mark
```

Then stand at the table. Everything from here is the arm's base light and the three buttons on
the pilot (the control pad on the arm):

1. The arm flies to a mark and stops above it. Its light is **blue**: stay clear.
2. The light turns **white**: your turn. Pinch the enabling buttons on the arm, move it so the
   pen tip sits exactly on the crossing of the mark, and let go. The arm holds where you leave it. **Keep the arm's
   tilt roughly as it arrived** — the software asked for that tilt on purpose; only move the
   tip onto the mark.
3. Press **✓ (check)** on the pilot. The arm takes itself back (blue), lifts the pen, and flies
   to the next position — sometimes the same mark with a different tilt, sometimes the other mark.
4. If the tip was not properly in the mark, press **✗ (cross)** instead: the arm stays with you
   (white) for another try. If a mark is unreachable or damaged, press **○ (circle)** to skip it.
5. When an arm is done it parks itself and the other arm starts. **Both arms parked = done.**
   An arm standing still above a mark with its light blue for more than a minute means that
   arm failed: go to the laptop, where the reason is written.

Per arm: four touches at its first mark (the four tilts), one touch at the second mark. Ten
touches in all.

After every pen change, or whenever you have handled a pencil, one touch per arm (seconds):

```
aris touchoff 2L
aris touchoff 2R
```

That measures the pen's length; without it the pen presses too hard or not at all.

## 7. Drawing

A drawing is a file of lines, in millimetres, measured from the centre of the table (x across,
y along). You get it from Pete or make it with his converter; the format is:

```
{"units": "mm", "frame": "table",
 "lines": [{"id": "a", "points": [[-100, 0], [100, 0]]},
           {"id": "b", "points": [[0, -100], [0, 100]]}]}
```

First time with a new drawing, fly it in the air (nothing touches the paper):

```
aris draw mydrawing.json --air 30
```

Then on paper:

```
aris draw mydrawing.json --note "4H graphite on 120 g paper"
```

The command prints a line whenever something changes and, at the end, what was drawn and what
was not (and why). Other commands, any time:

| command | what it does |
|---|---|
| `aris status` | the current or last job |
| `aris stop` | every arm stops at once and holds; the job is finished |
| `aris park` | every arm back to its park |
| `aris draw --rest-of <job id>` | draw what a stopped job left |
| `aris recover 2L` | after a fault, once a person has looked at the arm |
| `aris rig` | what the server runs, calibration state |

**Never** push an arm by hand while a job runs. If something looks wrong: emergency stop first,
`aris stop` second.

## 8. When something goes wrong

| what you see | what to do |
|---|---|
| `aris arms` does not list an arm | Desk: unlocked, FCI on? Then `sudo systemctl restart aris-robot` on the robot PC (the one exception to "leave it alone") |
| a job fails before any motion | read the reason in the output; usually "not calibrated", "arm not parked" (`aris park`), or the drawing is larger than the area (it is scaled down automatically unless that would halve it) |
| an arm holds mid-job | look at it, clear the cause, `aris recover 2L` (or 2R), then `aris draw --rest-of <job id>` |
| the arm's light is yellow or red | a robot fault: Desk, as before; then `aris recover` |
| calibration says "wrong slot or wrong robot?" | the robots' addresses in `site/aris_2026-10.json` are swapped; send Pete the output |
| anything else | send Pete: the exact output, the job id, and the folder `out/jobs/<job id>/` from the laptop |

## 9. For Pete, before handing over

- The first run under real ROS has not happened yet: build, fake hardware, identify, touch-off,
  air drawing, paper — in that order, with Pete present.
- Day-one checks on the hardware that nothing here can test: the Desk mode request on this
  firmware (`robot/site.json` `desk.mode_endpoint`), the pilot button names, whether the arm's
  joints can be read during hand-guiding, the panda-py wheel's libfranka version against the
  robots' system version.
- Fill the table at the top.
