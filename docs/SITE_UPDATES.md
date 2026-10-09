# Updates for the site — how to install one, and what each one changes for you

Read the newest entry first; it says what to do today. The procedure is the same every time.

## How to install an update (both machines, same commit)

On the planning laptop (the Thinker) and on the robot PC (the Dell), in `~/aris-clean/aris`:

```
git pull                                  # if this machine reaches GitHub, or:
git fetch /path/to/aris3-<commit>.bundle aris3:aris3-new && git checkout aris3-new
. .venv/bin/activate && pip install -e . && pip install -e robot
```

On the robot PC, additionally, **only when the entry below says "ROS packages changed"**:

```
cd robot/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=<libfranka build dir> && source install/setup.bash
```

Then, on the robot PC: Desk — every arm unlocked, FCI on; `sudo systemctl restart aris-robot`.
On the laptop: restart `aris serve`, then `aris arms`. It must list every arm with joints and say
`operator PC code: same (<commit>)`. If it says DIFFERENT, one machine did not get the update:
do the steps again there. Nothing else is needed; jobs are refused until the two match.

## 2026-10-08, night — contact by the encoders, not by force

What changed: the plane fit failed on 1L/1R (contacts ±4 cm) because the force estimate on
those arms is useless at 3 N. The touch now finds the paper by POSITION LAG: the controller keeps
commanding the pen down, and the moment the real tip stops following (0.3 mm), that is the
contact — encoders, not force; the force only stops the arm as a safety cap (8 N) and then also
counts as the contact. Same for every pen and arm. Also: a slot with no height calibration makes
its first touch from 60 mm above the nominal paper and plans the rest from that contact (the
free move that hit the table in job 1594 cannot happen again); and the robot PC starts the
stacks one at a time (four at once froze the Dell).

What to do: install, then `aris calibrate` on each arm again (the earlier fits are not written,
nothing to remove), `aris touchoff`, `aris mark --group rows12`, `aris crosses --group rows12`.
Per touch, the job's rows say which rule found the contact (lag or cap), the lag and the force.

## 2026-10-08, late night — a pen per arm

What changed: each arm has its own pen (gel in the back row, pencils in the middle row, on one
server). `aris pen` lists what is in; `aris pen 1L gel_g2` puts a pen in a slot (also on the
page, per arm card). Press, speed and the pull-only rule follow the pen in that arm; the job
header and the report say which pen each arm had. `aris draw --pen` is gone. The pen that is in
is kept in `config/<rig>/pens.json` next to the calibration files (per rig, like them).

What to do: install; on the rows12 server `aris pen 1L gel_g2`, `aris pen 1R gel_g2`,
`aris pen 2L graphite_4h`, `aris pen 2R graphite_4h` (or whatever is in); then `aris touchoff`
for each arm (the pen part is per pen: a touch-off made with the other pen is ignored and the
status says so); then as before.

## 2026-10-08, night — one server for rows 1 and 2 (`config/rows12`)

`aris mark --group rows12` needs the four arms on ONE server. The config is `config/rows12`
(1L, 1R, 2L, 2R; drawing area 1.56 × 2.25 m about (0, −0.63)). Run it instead of the two
row servers (stop those first — one runner per arm, an arm can only be in one stack):

On the Dell, a site file with the four slots mounted — copy `robot/site.json` to
`robot/site_rows12.json` and set `"mounted": true` on 1L, 1R, 2L, 2R (cores 16–19 are already
assigned), `server_url` to the laptop's port 8420 — then run the one runner with it:
```
sudo systemctl stop aris-robot      # or stop your two runners
aris-robot --site robot/site_rows12.json serve --config config/rows12
```
(`--site` goes before `serve`; or put both in the systemd unit's `ExecStart`.)

On the laptop, one server: `aris serve --config config/rows12 --driver robot --host 0.0.0.0
--site site/aris_2026-10.json`. Then `aris arms` must list all four with joints; copy the four
calibration files (`1L.json 1R.json 2L.json 2R.json`, already swapped as below) into
`config/rows12/calibration/`; `aris park`; `aris mark --group rows12` (four meetings, pair by
pair: 1L–1R, 1L–2L, 1R–2R, 2L–2R); `aris crosses --group rows12`.

## 2026-10-08, evening — touches fixed, the meeting's point is kept, L/R as seen from the desk, row 3

What changed:
- **Touches stopped at the force cap** (0 contacts): the detector took its zero after the pen
  was already on the paper. Now the zero is taken at the hover, and a cap trip counts as the
  contact (dated back to where the force started rising). `aris calibrate` and `aris touchoff`
  work again.
- **The meeting's point is kept** even when the lift afterwards fails; the arm then just holds
  where it is and the next phase plans its way out (a retreat, then park).
- **L and R are now as seen from Diemut's desk**: L = her left, R = her right. Every file was
  renamed: rig, site table, configs, figures, documents. New table: 1L = 31, 1R = 2, 2L = 71,
  2R = 97, 3L = 13, 3R = 17.
- **Row 3 hangs** (13 and 17 inverted): `config/front_row` and `config/all_six` exist;
  `config/back_row` is in the repository now too.

What to do, in this order:
1. Your own `config/back_row` would block the update: `mv config/back_row config/back_row.local`
   on both machines before pulling.
2. Install (**`colcon build` on the robot PC**: the gripper node is new since your build).
3. **Swap your calibration files** — they are named by slot, and the slots were renamed, so
   each file now names the other arm. In every config you calibrated (`config/two_arms/
   calibration/`, your back-row copy):
   `mv 2L.json x && mv 2R.json 2L.json && mv x 2R.json` (and the same for 1L/1R). The server
   refuses a file measured on another robot, so a mistake here is caught, not flown.
4. `robot/site.json` on the Dell: your `mounted` flags are per slot name — swap them too if
   only one arm of a row was mounted. Row 3: `3L`/`3R` `mounted: true` when you run it, cores
   20 and 21 must be isolated like 16–19 (README).
5. `aris arms` (names and robots must match the table above), `aris park`, then the
   calibration: `aris calibrate`, `aris touchoff`, `aris mark`, `aris crosses`.

## 2026-10-08, afternoon — an arm past a joint limit frees itself

What changed: `aris park` failed on 2R with "joint 6 is -0.0469 rad from its limit". Now any
job first moves such a joint back inside its range on its own (a "retreat" the checker allows
only inward), then plans as usual. More than 0.1 rad past the limit still needs a person
(programming mode, turn the joint back by hand).

What to do: install, `aris park`, then the calibration as below.

## 2026-10-08, later — the same, plus: arms back away when too close; groups; 2L the reference

What changed: after a meeting the arms retreat on their own (up, then away); if the job was
interrupted with the tips touching, `aris park` first moves them apart (a "retreat" motion
that only ever increases the distance), so nothing gets stuck. `aris mark --group rows12|
rows23|all` has every pair of neighbours meet once (row pairs at their first spot, column pairs
at the seam spot), solved together, with 2L held as the reference. The tips start 10–50 cm
apart, depending on the spot.

What to do: nothing new; install and run `aris mark` as in the entry below.

## 2026-10-08 — calibration: the two pens meet in the air, Diemut switches the modes

What changed: hand-guiding under FCI did not work (the pilot's enabling button only works in
Desk's programming mode), so the software never touches Desk again and the person does the two
clicks. `aris mark`: both arms of a row fly to a spot above the seam, tips 10–25 cm apart, and
their controllers switch off. In Desk (browser) Diemut puts BOTH arms in programming mode,
brings the two pen tips together until they touch, lets go, and puts both back in execution
mode with FCI on. The robot PC waits for the arms to come back, reads both at standstill (one
point seen by two arms → their relative position), lifts and parks them. `aris mark --yaw` does
it at a second spot 0.8 m away for the turn. No marks on the wood, no ruler. `aris crosses`
afterwards draws a cross (L) and a circle (R) at the spots: they should sit on each other.
Section 6 of her document.

What to do: install (no `colcon build` beyond the gripper one). `aris calibrate` and
`aris touchoff` for both arms if not done; then `aris mark`; then `aris crosses` and look.
If the arms do not come back within a minute after FCI on, `aris arms` says what it sees.

## 2026-10-07, late — calibration without Desk

What changed: `aris mark` no longer touches Desk at all — no login, no control token, no
programming/execution switch, no pilot buttons, no light. The arm flies to the hover, its
controller is switched off (the arm is idle under FCI, as after a launch), Diemut pinches the
enabling button, puts the pen on the cross and lets go; two seconds still = registered, the arm
takes over and flies on. Pinch again before that to redo; a brief pinch without moving skips.
`robot/secrets.json` and the panda-py install are not needed any more. Section 6 of her
document; the design in `docs/DESIGN.md` §6.

What to do: install (no `colcon build` needed beyond the gripper one below). Before the first
`aris mark`, the one-minute check in `docs/RUNBOOK_OPERATOR_PC_CLAUDE.md` step 7: deactivate
the controller on one arm and watch `/joint_states` follow Diemut's hand. If it does, `aris
mark` on each row. If the numbers freeze while she guides, report that — nothing else to try
that night.

## 2026-10-07, night — the gripper and the page with the buttons

What changed:
- **The gripper is back.** Our stack now starts the gripper node for each arm (**ROS packages
  changed: `colcon build` on the robot PC**). `aris grip 2L close|open|home` and the page's
  HOME / OPEN / CLOSE buttons use Diemut's old working settings (grasp at 70 N, open to 70 mm)
  and check that the jaws really moved; if they did not, it says so and asks for HOME.
- **A page with buttons**, in the browser on the laptop: `http://localhost:8420/gui` — arms,
  grippers, STOP, PARK, pick or upload a drawing (SVG too) and DRAW, CALIBRATE / TOUCH-OFF /
  MARK, the live job and its report. Section 6b of `docs/FOR_THE_ARTIST.md`.

What to do: install as above **with `colcon build`**, restart `aris-robot`, start `aris serve`,
open the page, press HOME on an arm whose jaws look wrong, then CLOSE on the pen holder.

## 2026-10-07, commit bf84cc6 — the machines bring the arms up themselves

What changed:
- Our launch no longer sets real-time priority on the whole ROS stack (`chrt`). It pinned the
  whole tree to FIFO 95 on top of your helper's pinning, which is what froze the Dell twice.
  Your helper (FIFO 95 on the control-loop threads, cores 16–19) is the right one — run it as on
  10-07. `rt_priority` is gone from `robot/site.json`; your copy still loads.
- `aris recover` reactivates the hardware first, then runs every step and restarts the arm's
  stack if the joints stay stale. Serve does the same by itself after 30 s without joint states
  (for example when the stacks started with FCI off): switch FCI on and wait a minute.
  "Restart the runner" is no longer the remedy.
- A runner refusal (wrong rig, wrong code) always appears as a row on the job.
- The two machines compare their code (`aris arms`).
- Graphite press 1.6 mm (2R ripped the paper at 2.1).

What to do: install as above (ROS packages did NOT change: no `colcon build`). FCI on, your
helper on, restart `aris-robot`, `aris arms`. Guide 2R's joint 6 back inside its range by hand
first (the park pose is fine; the arm was left beyond the limit). Then draw.

## 2026-10-07, commit b38ffb5 — one tracking mode, the paper height map, the 10-07 report

What changed:
- One tracking mode: joint position control with a geometric press. The impedance controller
  is gone (**ROS packages changed**: `colcon build` once; `aris_controllers` and `aris_msgs` no
  longer exist — one package builds).
- Every `aris calibrate <slot>` adds its touches to one height map of the table; drawings are
  planned on it minus the press. `aris rig` shows the map.
- Pens lift one arm per phase (park with two back pens down works). Recover brings controllers
  back after a link drop or guiding. The touch detector arms only once the descent runs at
  constant speed (no trips in the air). `aris draw pic.svg --width 1.70` draws an SVG.
  `aris arms` shows every arm. A job hung at start is failed with a row after 10 s.
- The back row's marks (R1a, R1b) are in `docs/FOR_THE_ARTIST.md`, section 5.

What to do: install (with `colcon build`), then `aris mark` on each row for x/y.
