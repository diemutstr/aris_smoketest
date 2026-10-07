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
