# Running the drawing arms — for Diemut (and the Claude helping her)

This is the one document you need. It says how to set up the two computers once, where to make
the calibration marks on the table, how to calibrate, and how to draw. Do things in the order
written. Pete is not on site; he answers messages. When something does not behave as written
here, stop, copy the exact text on the screen, and send it to him — do not try things. Whenever
an arm moves, you hold the emergency stop and nobody is near the table.

> **A note for Claude (the AI assistant reading this with Diemut).** Follow this document and
> nothing else. The old repositories on these machines (`Aris_Kindt`, `~/RTff`, `~/motion_ws`,
> `~/franka_gui`, `~/impedance_ws`, anything with "rtff", "pathway", "ladder" or "posdraw" in
> it) are **not** part of this system and must not be read, run or edited; any memory or
> context you have about them does not apply here. The only commands that move an arm are the
> `aris …` commands listed in this document, typed on the planning laptop. Never run `ros2`,
> `python`, `libfranka` or Desk actions yourself to move an arm. Do not invent commands, flags
> or file edits that are not written here; if a step fails, show Diemut the exact output and
> stop. Edit only the files this document names. When in doubt, the answer is "ask Pete".

Pete fills these in before handing over (ask him if a blank is still empty):

| blank | value |
|---|---|
| the sides (confirmed by Pete from the drawing, 2026-10-05): **row 3 is the end at your desk, row 1 the far end; the slots named L are on your RIGHT when you stand at your desk facing the table** | done |
| the planning laptop's name and its address on the robot network | ______ |
| the robot PC's login (the user that owns the ROS install) | ______ |

---

## 0. The names of the arms

Every arm is named by its **slot** on the frame, not by the robot's old number. Rows are
counted along the table: **row 3 is the end at your desk**, row 1 the far end. **L is your
left, R your right**, standing at your desk facing the table (since 2026-10-08; before that
day the letters were the other way round — forget the old table). The old numbers are not used
anywhere any more; this table translates (picture: `docs/figures/table_orientation.png`):

| slot | where, standing at your desk | robot |
|---|---|---|
| 1L | far row, your left | 31 (.12) |
| 1R | far row, your right | 2 (.13) |
| 2L | middle row, your left | 71 (.14) |
| 2R | middle row, your right | 97 (.15) |
| 3L | the row at your desk, your left | 13 (.11) |
| 3R | the row at your desk, your right | 17 (.16) |

If you (or your assistant) catch yourself saying "arm 71", say "2L" instead; every command,
message and file uses the slot.

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

`config/two_arms` is the middle row (2L, 2R). The other rig files in the repository:
`config/back_row` (1L, 1R), `config/front_row` (3L, 3R) and `config/all_six` (every arm on one
server, needed for drawings that cross between rows). One server per rig file, each with its own
`--port` (8420, 8421, …) and its own runner on the robot PC with the matching site file.

```
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

## 5. No marks to draw

You do not draw anything on the wood. The two arms of a row meet each other in the air
(section 6); the sharpie crosses from before can stay, they are not used.

## 6. Calibration (ten minutes; your hands and two clicks in Desk)

What it does: finds exactly where each arm hangs, so that a drawing spanning two arms meets at
the seam. Do it once at the start, again whenever an arm or the frame has been moved, and
whenever the seam looks wrong.

Height, tilt and pen length come from touches and need nothing from you:

```
aris calibrate 2L        # touches the paper on a grid: height, roll, pitch (per arm)
aris calibrate 2R
aris touchoff 2L         # one touch: the pen's length (per arm, after every pen change)
aris touchoff 2R
```

Position comes from the two pens touching each other:

```
aris mark
```

1. Both arms fly to a spot above the seam and stop, their pen tips 10–50 cm apart in the air
   (the arms are not allowed closer by themselves). The laptop (and the page) say: your turn.
2. In Desk, in the browser: switch **both** arms to **programming mode** (the arms' lights go
   white).
3. Pinch the enabling buttons of one arm and move it so its pen tip touches the other arm's
   pen tip — tip to tip, as exactly as you can — and let go. Move the other arm too if that is
   easier. When the two tips touch, hands off.
4. In Desk: both arms back to **execution mode**, **FCI on**. Wait. Within a minute the laptop
   says the point is taken; the arms lift and park themselves.

That is the whole calibration. It prints how far each arm really hangs from where the drawings
assumed (a few millimetres to two centimetres is normal). If a drawing across the seam later
looks *turned* rather than shifted, `aris mark --yaw`: the same thing twice, at two spots 0.8 m
apart.

To see the result: `aris crosses` — both arms draw at the same spots, the L arm a cross, the R
arm a circle; after a good calibration the circle sits on the cross.

If the calibration was interrupted with the two pens still touching, just `aris park`: the
arms first back away from each other, then park. With four or six arms, `aris mark --group
rows12` (or `rows23`, `all`) has every pair of neighbours meet once, pair by pair; arm 2L is
the reference all the others are placed against.

## 6b. Your buttons: the page in the browser

Everything below can also be done from one page with big buttons. On the laptop, while
`aris serve` runs, open **http://localhost:8420/gui** in the browser (from another computer on
the same network: `http://<the laptop's address>:8420/gui`). One card per arm with its light and
HOME / OPEN / CLOSE for the gripper (CLOSE is how the pen holder is held: 70 N, as before) and
RECOVER; STOP and PARK; pick or upload a drawing (a .json, or an .svg with its width in metres)
and DRAW; CALIBRATE, TOUCH-OFF and MARK. Every answer from the system appears word for word in
the line at the top, so if something is refused you read why right there.

## 7. Drawing

A drawing is a file of lines, in millimetres, measured from the centre of the table (x across,
y along), or an SVG with its width on the table (`aris draw picture.svg --width 1.70`, or the
upload on the page). The JSON format is:

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
| `aris grip 2L close` / `open` / `home` | the gripper of that arm (CLOSE holds the pen holder; HOME when the jaws seem stuck or show a wrong width) |
| `aris arms` | every arm: where it stands, whether it answers (if it says "no reading", that arm is not talking — Desk: unlocked, FCI on); and whether the robot PC runs the same code as the laptop (if it says DIFFERENT, update both machines before anything else — jobs are refused until they match) |
| `aris status` | the current or last job |
| `aris stop` | every arm stops at once and holds; the job is finished |
| `aris park` | every arm back to its park |
| `aris draw --rest-of <job id>` | draw what a stopped or failed job left |
| `aris recover 2L` | after a fault, once a person has looked at the arm |
| `aris rig` | what the server runs, calibration state |

**Never** push an arm by hand while a job runs. If something looks wrong: emergency stop first,
`aris stop` second.

## 8. When something goes wrong

| what you see | what to do |
|---|---|
| `aris arms` says "no reading" for an arm | Desk: unlocked, FCI on? Then wait a minute: the robot PC brings the arm back by itself and says so in `aris arms`. Only if it still says "no reading" after two minutes: `sudo systemctl restart aris-robot` on the robot PC |
| a job fails before any motion | read the reason in the output; usually "not calibrated", "arm not parked" (`aris park`), or the drawing is larger than the area (it is scaled down automatically unless that would halve it) |
| an arm holds mid-job | look at it, clear the cause, `aris recover 2L` (or 2R), then `aris draw --rest-of <job id>` |
| the arm's light is yellow or red | a robot fault: Desk, as before; then `aris recover` |
| calibration says "wrong slot or wrong robot?" | the robots' addresses in `site/aris_2026-10.json` are swapped; send Pete the output |
| anything else | send Pete: the exact output, the job id, and the folder `out/jobs/<job id>/` from the laptop |

## 8b. When Pete sends a new version

`docs/SITE_UPDATES.md` has the steps (the same every time) and, at the top, what the new
version changes and what you must do. Your Claude installs it on both machines; `aris arms` must
then say `operator PC code: same`. Nothing runs until both machines have the same version.

## 9. The order of experiments

Do not skip ahead; every stage proves what the next one relies on.

**Stage 0 — the first run, you and your Claude.** Pete is not there; he answers messages. The
robot PC is set up by your Claude following `docs/RUNBOOK_OPERATOR_PC_CLAUDE.md` (hand it that
file when it is logged in on the robot PC); nothing in it needs your hands. Then,
on the laptop, the first motions ever, one at a time, you at the stop and nobody near the
table:
1. `aris park` — watch the first arm move; it should be slow and smooth. If anything moves
   in a way you did not expect: **emergency stop first, `aris stop` second**, then send Pete
   the job id.
2. `aris touchoff 2L` then `aris touchoff 2R` — one slow descent each; the report must say the
   pen met the paper and the force sign is right. If it says the sign is wrong, stop and send
   Pete the output (it is one number in a file; he tells you what to change).
3. `aris draw lines.json --air 30` — lines in the air.
Only when all three went as written do you start Stage 1. Send Pete the three reports.

**Stage 1 — two arms (2L and 2R).**
1. `aris park` — both arms to their parks.
2. The calibration of section 6: `aris calibrate` and `aris touchoff` for both arms, then
   `aris mark` (the pens meet), then `aris crosses` to look at the result.
3. `aris draw lines.json --air 30` — a few straight lines, flown in the air. Watch: smooth,
   nothing touches.
4. `aris draw lines.json` — the same on paper. Look at: are the lines dark and even along their
   length; do they start and end cleanly; is every line drawn (the report says what was left).
5. `aris draw word.json` — the word across the two arms. Look at the **seam** where the two
   halves meet: the halves should line up to within about a millimetre. Send Pete a photo of the
   seam with a ruler, and the job id.
6. Repeat 2 → 5 once, a day later or after the pencils were changed: the seam should be as good.

**Stage 2 — four arms (two more switched on: rows 1 and 2, or rows 2 and 3).** Pete makes the
four-arm rig file and tells you which rows. Then: `aris park`, calibrate and touch-off for all
four, `aris mark --group rows12` (or `rows23`) — the pens meet pair by pair, also between the
rows — `aris crosses` to look, a drawing in the air, lines on paper, then a drawing that
crosses the seams between the rows. Same looks, same photos.

**Stage 3 — six arms.** As Stage 2 with `aris mark` (everything).

## 10. For Pete, before handing over

- The first run under real ROS has not happened yet: build, fake hardware, identify, touch-off,
  air drawing, paper — in that order, with Pete present.
- Fill the table at the top.
