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
