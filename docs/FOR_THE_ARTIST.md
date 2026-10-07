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
counted along the table: **row 3 is the end at your desk**, row 1 the far end. Careful with the
letters: **the slots named L are on your RIGHT, the slots named R on your LEFT**, when you stand
at your desk facing the table (the names come from the technical drawing, which looks at the
table from the other end; see `docs/figures/table_orientation.png`). The old numbers (13, 17,
31, 71, 2, 97) are not used anywhere any more; this table translates (picture:
`docs/figures/table_orientation.png`):

| slot | where, standing at your desk | old number (confirmed on day one by which address answers) |
|---|---|---|
| 2L | middle row, on your RIGHT — **in use** | probably 71 (it drew the half on your right) |
| 2R | middle row, on your LEFT — **in use** | probably 97 (it drew the half on your left) |
| 3L, 3R | the row nearest your desk — switched off | 2, 97 or 31 |
| 1L, 1R | the far row — switched off | 13, 17 |

If you (or your assistant) catch yourself saying "arm 71", say "2R" instead; every command,
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

**The picture to work from: `docs/figures/marks_for_diemut.png`** (your desk at the bottom,
measurements from the table's edges). In words:

- Both marks lie on the **centre line** along the length of the table: 1.09 m from either long
  edge (the table is 2.19 m across and 4.17 m long).
- **Mark A**: on the centre line, 1.68 m from the FAR short edge (the end away from your desk).
- **Mark B**: on the centre line, 1.68 m from the short edge at your desk.
- A and B come out 0.80 m apart, straddling the middle of the table, each between the two arms
  in use.

So A and B are about 0.80 m apart, straddling the centre, each roughly midway between the two
live arms. Label them A and B. (The drawing area is a strip 1.72 m across and 0.90 m along,
centred on the table; the marks do not set it, the arms' mountings do.)

**For the back row (arms 1L = robot 2 and 1R = robot 31), two more marks, also on the centre
line** (`docs/figures/marks_six_slots.png` shows all of them; measured from the FAR short edge,
the end away from your desk):

- **Mark R1a**: on the centre line, 0.48 m from the far short edge.
- **Mark R1b**: on the centre line, 1.28 m from the far short edge.

`aris mark` on the back row's server uses R1a and R1b the way the middle row uses A and B. Two
more marks, **S12L** and **S12R**, tie the back row and the middle row together; they are needed
only when both rows are calibrated in one go on one server (`aris mark --group rows12`), which is
what makes the two rows' drawings line up on one sheet. Draw them now so they are there when
that day comes: both 1.48 m from the far short edge, each 0.30 m off the centre line — S12L
toward the long edge on the L arms' side (your right when you sit at your desk), S12R toward
the other long edge.

## 6. Calibration (about 15 minutes, no computer after the first line)

What it does: each arm in turn comes to a mark; you guide its pen tip into the mark; the
software works out exactly where each arm hangs and how long its pen is. Do it once at the
start, again whenever an arm or the frame has been moved, and again when you suspect the
drawing has shifted.

Type, on the laptop:

```
aris mark
```

Then stand at the table. Nothing else is needed — no Desk, no buttons, no light to watch:

1. The arm flies to a mark and stops a few centimetres above it, then goes soft: your turn.
2. Pinch the enabling buttons on the arm, move it so the pen tip sits exactly on the crossing
   of the mark, and **let go**. **Keep the arm's tilt roughly as it arrived** — the software asked
   for that tilt on purpose; only move the tip onto the mark.
3. Keep your hands off for two seconds. The arm takes that as "done", lifts the pen and flies
   to the next position — sometimes the same mark with a different tilt, sometimes the other
   mark. The flight is your confirmation.
4. Not right yet? Pinch again and move it before the two seconds are up; the clock starts
   over. A mark you cannot reach: pinch briefly without moving the arm, let go — that skips it.
5. When an arm is done it parks itself and the other arm starts. **Both arms parked = done.**
   An arm hanging above a mark for more than three minutes without anyone guiding it gives up
   on that touch; the reason is on the laptop.

Per arm: six touches at its first mark (upright and five tilts), one touch at the second
mark. Fourteen touches in all. When it is done the laptop prints, for each arm, how far it
really hangs from where the drawings say (a few millimetres is normal) and where the marks
really are (a few centimetres from where you aimed is normal).

**Right after the calibration, and again after every pen change** or whenever you have handled
a pencil, one touch per arm (seconds, nothing for you to do):

```
aris touchoff 2L
aris touchoff 2R
```

That measures the pen's length exactly; without it the pen presses too hard or not at all.

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
file when it is logged in on the robot PC); you do the one thing in it that needs hands (pinch
the enabling buttons and guide an arm by hand) with the emergency stop in your other hand. Then,
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
2. `aris mark` — the calibration of section 6. Then `aris touchoff 2L`, `aris touchoff 2R`.
3. `aris draw lines.json --air 30` — a few straight lines, flown in the air. Watch: smooth,
   nothing touches.
4. `aris draw lines.json` — the same on paper. Look at: are the lines dark and even along their
   length; do they start and end cleanly; is every line drawn (the report says what was left).
5. `aris draw word.json` — the word across the two arms. Look at the **seam** where the two
   halves meet: the halves should line up to within about a millimetre. Send Pete a photo of the
   seam with a ruler, and the job id.
6. Repeat 2 → 5 once, a day later or after the pencils were changed: the seam should be as good.

**Stage 2 — four arms (two more switched on: rows 1 and 2, or rows 2 and 3).** Pete makes the
four-arm rig file and tells you which rows. On the wood, two more marks per added row (he gives
their rough places, like A and B). Then: `aris park`, `aris mark --group rows12` (or
`rows23`), touch-offs for all four, a drawing in the air, lines on paper, then a drawing that
crosses the seams between the rows. Same looks, same photos.

**Stage 3 — six arms.** As Stage 2 with `aris mark` (everything), ten marks on the wood.

## 10. For Pete, before handing over

- The first run under real ROS has not happened yet: build, fake hardware, identify, touch-off,
  air drawing, paper — in that order, with Pete present.
- The one hardware fact nothing here can test: with the trajectory controller deactivated,
  the enabling button hand-guides the arm and the joint states keep streaming (runbook step 7).
- Fill the table at the top.
