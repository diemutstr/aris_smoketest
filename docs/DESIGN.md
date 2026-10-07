# Planner design

Agreed with Pete, 2026-09-29. Clean rebuild. Start simple; speed-ups are collected in
`OPTIMIZATION_NOTES.md` and only built when a measurement asks for them.

Three levels. Each level has one job, calls only the level below it, and returns two things:
what it planned, and what it could not plan and why.

```
drawing server      takes a whole drawing, runs it on the rig, reports progress
  system planner      which arm draws which line, in which phase, behind which walls
    arm planner       one arm: a tour of drawing and free-space motions
      sequencer         order, direction, which alternative
      local planner     one line -> a bunch of alternative drawing plans
      free-space planner  one configuration to another, pen up
```

## 1. Local planner

Same code for every arm. Knows nothing about the table, the canvas or the other arms.

**In**
- lines, as polylines of pen-tip positions in the arm's base frame
- obstacles, as geometry in the arm's base frame (this is also how walls arrive)
- the tool (tip offset) and the arm limits
- drawing rules for the timing step (draw speed, no stops mid-line, press, hand turns only at the paper)

**Out, per line**
- a list of pieces (one piece if the line can be drawn in one go, more if it needs a lift)
- per piece a bunch of alternative plans; each has a start configuration, an end configuration,
  the joint path, a score and a draw time; each can be drawn in either direction
- or "cannot draw", with the stretch and the reason

**How**
- Graph: one layer per step along the line. A node is one choice of spin, lean, elbow value and
  IK branch, which is one joint configuration. Edges go only to the next layer and only between
  neighbours, with a cap on the joint step (the cap keeps the arm on one branch).
- Lift edges: stay at the same point of the line and jump to any other node, at a high fixed cost.
  A line drawn in one go is the case with zero lift edges.
- Objective: stay above the gates (joint limits, singular value, clearance), minimise joint motion.
- Search: one sweep over the layers.
- Then three steps behind one call: search, smooth, retime. The final path is re-solved exactly
  (never interpolated in joint space) and validated.

## 1b. Free-space planner

Same code for every arm, same rules as the local planner: base frame in, base frame out, knows
nothing about the table, the canvas or the other arms.

**In:** start configuration, goal configuration, obstacles in the base frame, the tool, the limits.
**Out:** a collision-free joint path, shortened and timed; or "no path", with the reason.

**How**
- try the straight joint-space move first; most moves between nearby lift-off configurations are one
- otherwise bidirectional RRT (one tree from each end, each growing toward the other)
- shorten the result, re-checking every shortcut
- fixed random seed derived from the inputs, so the same question gives the same path
- the pen is part of the arm's collision body; the paper is an obstacle
- both ends sit a few millimetres above the paper, which is the hardest place for an RRT to start
  from; so each end is first raised straight up to a comfortable height, and the RRT connects the
  two raised configurations

**How we find out how good it is.** A fixed test set per arm: a thousand pairs of lift-off
configurations taken from the local planner's bunches, near and far, same branch and different
branch. Reported: how many are solved, the time per plan, and the path length against the straight
joint-space distance. This is the acceptance test of the free-space planner.

## Shared kernel

Both planners are built from the same four parts, and nothing else:

| part | job |
|---|---|
| arm model | kinematics, IK, limits, collision body (links + tool + pen) |
| obstacles | geometry in the base frame: paper, frame, walls, parked arms |
| collision check | distance between the arm model and the obstacles, in batches |
| retime | turn a joint path into a timed trajectory inside the limits |

What differs between arms (mount position, calibration) lives in one place: the transform from
the table frame to that arm's base frame. It is applied once, by the arm planner, on the way in.

## 2. Sequencer

Wraps the local planner and the free-space planner. Together they are the **arm planner**.

- picks one alternative from each bunch, the order of the pieces and the direction of each
- simple heuristic first (greedy)
- each piece starts and ends at a lift-off configuration: the same arm shape, raised a short
  distance off the paper, with a tiny straight motion to set the pen down
- free-space motions between lift-off configurations come from the free-space planner
- if a free-space motion cannot be found: try another alternative, else hand the piece back
- emits motions as it goes, so the first motion is available before the tour is finished

**Arm planner in:** lines, obstacles and walls in the base frame, the start configuration.
**Arm planner out:** a sequence of drawing motions and free-space motions, plus the leftovers.

## 3. System planner

Only ever calls the arm planner, with different lines and different walls.

Pete's scheme (2026-09-29): leaders get as much space as they need; followers get started with
what they can do without interfering with the leaders; then the roles swap; then a fill phase.

**Build order.**
- Step 1: leaders only. The three leaders of a phase draw; the three followers stay parked the
  whole phase and are plain obstacles. Then the roles swap. No footprints needed.
- Step 2: followers draw next to the leaders, against the leader's footprint, as described below.

**Leaders can reach each other.** The diagonal pairs (13-71 and 71-2) hang 1.36 m apart and each
arm reaches about 0.9 m, so they can touch; the audit found 0.7 % of their pose pairs closer than
the margin. The two outer leaders of one column (2.42 m apart) cannot.

**Diagonal walls between the leaders** (Pete, 2026-09-29; figure: `figures/leader_walls.png`).
A wall is a vertical plane in the table frame. An arm keeps its whole body, pen included, on its
own side and half the arm-to-arm margin away, so two arms on opposite sides are clear of each
other by construction. The wall between two leaders is the plane halfway between their bases and
square to the line joining them:
- it passes through the table centre line at y = -605 mm and y = +605 mm, turned 26.7 degrees from
  the row direction
- it is 678 mm from both bases, further than a row wall would be (605 mm), so each leader loses less
- each leader's region is the part of the canvas nearest to it
- in phase 2 the walls are the mirror image, so they cross the phase 1 walls at one point each.
  A line that crosses a wall in phase 1 is, almost always, whole inside one region in phase 2.
  With row walls the two phases would share the same walls and the swap would gain nothing.

The independent checker confirms the separation afterwards by measuring the distance between the
leaders' footprints.

**Parked followers.** Each parked follower stands inside the region of the leader of its own row,
403 mm from the nearest wall, and is a plain obstacle for that leader.

**Step 2: followers.** The rig is three rows of two arms; inside a row the leader has priority.

**Inside a row, per phase**
1. The leader plans first. Its only obstacles are the walls, the frame, the paper and the
   follower standing parked. It takes the space it needs.
2. The leader's **footprint** is everywhere its body goes during the phase, drawing and free-space
   moves together. It is handed to the follower as one more obstacle.
3. The follower plans against that footprint and draws what it can. Because it avoids everywhere
   the leader will ever be in this phase, the two can run at any relative timing. No
   synchronisation.
4. What the follower could not draw is a leftover.

**Phases**

| phase | leaders | followers |
|---|---|---|
| 1 | 13, 71, 2 (1-2-1) | 17, 31, 97 |
| 2 | 17, 31, 97 (2-1-2) | 13, 71, 2 |
| 3 fill | what is left: lines that cross a wall in both phases, and lines neither turn could draw |

Every arm gets one turn with priority, so every line is offered once to an arm that is not
restricted by its partner.

**Allocation.** Each line goes to the arm in its row that reaches it best; where both reach it
equally, to the leader. A line that crosses a wall waits for the next phase. Lines are not
cut unless no arm can draw them whole; a cut gets an overlap at the joint.

**After one alternation, what is left** (expected, to be measured):
- lines that cross a wall in both phases; in particular any line longer than one arm's reach
- two small patches where the phase 1 and phase 2 walls cross, at the centre line 605 mm either
  side of the table centre: no arm may bring its body that close to a wall in either phase
- the rim no arm reaches (2.5 % of the canvas), which is never drawn

**A phase is just (active arms, walls).** The system planner runs a list of them: 1-2-1, 2-1-2,
then fill layouts with the walls somewhere else or a single arm alone, until nothing is left or
a layout draws nothing new.

**Running a phase.** The three leaders plan in parallel and start moving as soon as their first
motion exists. Each follower starts once its leader's plan for the phase is complete. When a row
is done, both arms park and confirm. The next phase starts when all rows have confirmed.

**Out:** per phase and arm, the arm planner's motion sequence; and the list of what was not drawn,
with reasons.

**The system planner only ever calls the arm planner.** A wall, a parked arm and a leader's
footprint are all just obstacles in the arm's base frame.

Known from the old planner and the audit (to be re-measured): leaders in different rows clear each
other on 99 % of pose pairs even without walls; two arms of the same row hang only 610 mm apart,
and the old followers kept 11 % of what they were offered at first and 67 % after fixes; a parked
partner blocks none of the drawing area at this height; 57 % of the canvas can be drawn by exactly
one arm; 2.5 % (the rim) by none.

## 4. From plan to robot

PROPOSAL (2026-09-29). Planning and execution never call each other. Between them sits one thing:
a queue of motions per arm. Data flows one way.

```
arm planner -> checker -> queue (one per arm) -> executor (one per arm) -> arm
                                                  coordinator: starts phases, waits for "all parked"
```

**Motion.** The only thing that crosses from planning to execution. One drawing motion, one
free-space motion, or the short lower or lift at the ends of a piece: a timed joint trajectory, its start configuration, its end configuration, its
kind. Every motion starts where the previous one ended and ends in a configuration the arm can
hold for as long as it likes, inside its own region.

**Arm planner.** Produces motions one at a time, in order. It never talks to a robot. Collecting
everything before running and running each motion as it appears are the same code; the only
difference is when the executor is started.

**Checker.** A motion enters the queue only after the independent check has passed. What is in
the queue is safe to run. The check happens inside the arm planner's loop, before the planner
builds on the motion (decided 2026-09-30, after a 10 000-line run lost 13 % of its ink to one
refusal): the server hands the arm planner a `verify` callable that runs the checker; a piece
is drawn only if every motion of its group (the move to it, lower, draw, lift) passes, else the
piece is left over as `failed_check` with the checker's word, and the arm plans on from where
it was. A motion carries the checker's numbers (`Motion.checked`); the queue refuses one
without them. Nothing planned on top of an unchecked motion ever exists.

**Queue.** Append-only, one per arm and phase, stored as a file. It is also the record of what
was planned, and what the GUI shows.

**Executor.** One per arm. Takes the next motion, confirms the arm is at its start configuration,
runs it, reports it done. Empty queue: the arm holds. It never plans and knows nothing about other
arms. It talks to the arm through a handful of verbs (move, draw, switch controller, read state,
stop); a simulated arm offers the same verbs, so everything above runs without hardware.

**Coordinator.** The only part that knows about phases at run time: start a phase, wait until
every arm reports parked, start the next.

**When a motion fails.** That arm stops and holds; the rest of its queue is dropped; the arm
planner is called again with the lines still to draw and the configuration the arm is actually
in. The other arms are not affected, because the walls make them independent.

**The operator PC runs one resident process (Pete, 2026-10-01).** `aris-robot serve` is
started once (at boot) and never touched again: it brings up and supervises the per-arm ROS
stacks, and it asks the drawing server what to do — run this job (a drawing, a park, a
calibration), recover this arm, report — over the one Ethernet connection, pulling, so the
operator PC needs no open port and the server stays the only front door. It fetches the
calibration files from the server, so both machines always hold the same ones. Everything a
person does, they do on the planning PC with `aris …`; the e-stop is the only thing on the
operator side.

**Followers, removed 2026-10-02.** Step 2 (a follower drawing against its leader's swept
footprint, with the footprint as a distance-field obstacle for the planners and the checker)
was built, measured (the leaders took 98–100 % of the ink under the five laws, so the followers
drew nothing at 3–25× the planning cost) and removed from the code once mode A was decided, to
keep one way of doing things. It lives at commit 85d3ac9 (`aris/system/followers.py`,
`aris/kernel/footprint.py`, `types.Field`, the checker's `fields`). It comes back, with the
balance law (split a row's lines between leader and follower before the leader plans), as soon
as the simple thing draws on the hardware.

## 4c. Decided 2026-10-02, after the first word drawn on the hardware

**Slots and robots.** An arm is named by its slot on the frame, `1L 1R 2L 2R 3L 3R` (row 1 at
the −y end, L at −x). `config/rig.json` describes slots; one table in `site/` says which robot
(serial, address) hangs in which slot today, and the runner refuses a robot that is not the one
the table names. The old prime-number ids live on only in that table. Nothing in the planning
code knows a robot.

**Tracking.** The word "unknown" was drawn on 2026-10-01 under plain joint position control:
certified joint trajectories through the stock trajectory controller, at most 15 mm/s on the
paper, every touchdown from standstill, and a purely geometric press — the plan runs `press`
below the measured paper (2.1 mm for 2 mm 4H graphite, measured on site 2026-10-06). It is the
only mode (4b).

**The paper height map (2026-10-07).** A fixed press on paper that varies by millimetres gives
no ink in places and a reflex in others (a gel pen's window is about 1 mm). Every touch of the
plane jobs goes into one height field for the table (`config/calibration/paper.json`, a
thin-plate spline, flat beyond the touched hull); the system planner puts the drawing on
`surface(x, y) − press` and the checker judges the tip against the same file. Following 30 cm
bumps to a quarter millimetre needs touches about 5 cm apart.

**The drawing surface.** The press is a property of the surface the drawing's points lie on:
the system planner gives the points `z = paper − press` and the planners below put the tip where
the points are; the real paper stays the plane the holder and links must clear; the checker's
"tip on paper" rule reads the same press. No planner knows that pens exist.

**Pens are rig data.** `rig.json` has a `pens` table (name, nominal length, capsule, press,
speed on paper, whether it draws only pulled) and one line naming the pen that is in.
The job header names it; the report records it; the drawing file stays pen-agnostic.

**Calibration in two parts, two clocks.** Per slot, one file with a `base` part (x, y, z, roll,
pitch, yaw; the plane job now, the dimples later; re-done when an arm or the frame moves) and a
`pen` part (the tip offset from a one-touch touch-off at a reference point on the paper, against
the measured plane; re-done after every pen switch or handling of the pencil — with a
geometric press the pen length is the tone). Each part has its own date and pass flag.

**One model from the sources.** Sources: `config/rig.json`, `config/calibration/<slot>.json`,
`site/`, the vendor meshes. One builder, `Rig.load`, turns them into the model — slot poses
with calibration applied, hanger steel placed from the *calibrated* axis (the arm is bolted to
the plate), parked bodies at calibrated poses, the pen capsule from the measured length,
fences, phases. Nothing else constructs geometry; the checker re-reads the same files with its
own reader; caches (maps, kinematic table) are keyed by the sources' digests; every job header
pins those digests. Until x/y are calibrated (1–2 cm off today) the wall clearance is 40 mm.

## 5. Drawing server

The top of the hierarchy and the one front door. The GUI and the command line talk only to it.
It is the only part that touches both planning and hardware; the planners stay pure.

**In:** a drawing, as a list of lines in the table frame.
**Out:** progress while it runs, and at the end what was drawn and what was not, with reasons.

**What it does**
- loads the rig description and the calibration; refuses to start without a valid calibration
- reads where the arms are and which are available
- calls the system planner phase by phase
- passes each motion through the checker into the arm's queue
- owns the executors and the coordinator (section 4)
- stop, pause, resume
- keeps the state of the job: waiting, planning, drawing, paused, done, failed

One job at a time. Calibration and "park all arms" are other kinds of job run by the same server
through the same executors.

**Where it runs.** One repository, two roles, the same commit on both machines. The planning PC
runs the server and the planners and writes the queues; it never imports ROS. The operator PC
runs `robot/`: it fetches the queue records from the server as they are written, runs the
executors and the coordinator next to the arm drivers, and posts events back. A lost link stops
nothing that is already queued; an arm whose queue runs dry holds.

## 4b. How the arm follows a motion (decided 2026-09-30; settled 2026-10-07)

The arm follows the certified joint trajectory under plain joint position control (the stock
trajectory controller), and the press is geometric: the drawing surface lies `press` below the
measured paper, re-measured by touch. That is what drew Diemut's installation for years (position
control, z by touch, re-measured every 45 minutes) and every line since 2026-10-01. The
alternative designed here on 2026-09-30 — a joint impedance controller soft along the pen with
the press as a force — was built as "mode B", never ran on the hardware, and was removed on
2026-10-07 to keep one way of doing things (it lives at commit cb6e958). What position control
cannot do is follow paper it has not measured: a rigid pen on paper that varies by millimetres
needs the paper height map (4c) measured densely enough, or compliance in the pen holder.

## 6. Calibration

PROPOSAL (2026-09-29, second version after Pete's input). A job of the drawing server. The arms
measure everything themselves, with their own joint sensors; no camera, no tape measure. A person
is needed once, to guide each arm to the shared spots. Starting point: each arm's pose is known to
a few millimetres, at worst a centimetre, and its orientation may be off.

**Hardware needed** (Pete builds this, 2026-09-29; the software side is ours)
- four small plates with a cone-shaped dimple, taped to the table on the centre line, at 605 mm and
  1815 mm either side of the table centre. A dimple instead of a drawn dot, because the tip centres
  itself in it and nobody has to judge by eye.
- one rigid pin with a rounded tip, the size of a pen, for the holder. Graphite would wear and
  break in a dimple.

**Why those four spots.** Each is 678 mm from the arms around it. The two inner spots are reached
by four arms each, the two outer spots by the two end arms. Every arm reaches exactly two spots,
1.21 m apart.

| spot (mm from table centre, on the centre line) | slots |
|---|---|
| -1815 | 1L, 1R |
| -605 | 1L, 1R, 2L, 2R |
| +605 | 2L, 2R, 3L, 3R |
| +1815 | 3L, 3R |

**Step 1. Paper height and tilt, automatic — built first, on its own (Pete, 2026-10-01).** Each
arm touches the paper on a grid of about 25 points with its own pen, pen upright, the same hand
spin at every point. A plane through the touches gives the arm's roll, pitch and height over the
paper; the rest is the unevenness of the table, kept as a height map. With one orientation an
unknown pen length shifts every point alike: the tilt is exact and the length lands in the
height, which is the height we want (where the pen meets the paper). x, y and yaw stay nominal
until the dimples (steps 2 and 3) are done; they add on top without changing this step.
How: a job like park (`aris calibrate <arm>`): hover poses 60 mm above the nominal paper, the
free-space planner between them, a `touch` motion at each (the planned descent to the nominal
paper, checked like a lower; the arm flies it slowly under position control — the stock
joint-trajectory controller, not the drawing controller — stops at the first force onset, reads
its encoders, retreats; it may go 20 mm past the planned end before giving up, the declared
uncertainty of the paper). Encoders say where the paper is; force only says when. The contact
joints are event rows; the solver (`aris/calib/`) fits the plane and writes the file.
The pen length is not observable from a flat paper (a 15-degree lean changes the height by 3 %
of the error): it comes from the dimple pivot later, or from the drawn check (the same line with
the hand turned 180 degrees: a gap of 0.78 times the length error, since the pen sits at 23
degrees in the holder).

**Step 2. Shared spots, hand-guided, about a minute per arm and spot.** Guide the pin into the
dimple and, keeping it seated, tilt and turn the hand through a range of orientations. The arm
records its joints the whole time. One fit gives two things at once:
- where the tip is relative to the hand (its length included)
- where the dimple is as seen by that arm
The fit error is shown immediately; if the pin slipped, it is large and the step is repeated.

**Step 3. Solve, automatic.** All arms that touched the same dimple must agree on where it is.
Solving that for all six arms together gives each arm's position and turn on the table. The
positions of the dimples do not have to be measured: they are solved too. Because every arm has
two spots 1.21 m apart, its turn about the vertical comes out as well and does not have to be
assumed zero.

**Step 4. Pen length, automatic.** Swap the pin for the pen and touch the paper once. The arm's
height is known by now, so the touch gives how much longer or shorter the pen is than the pin.
The pen leans 23 degrees in the holder: 1 mm more length lowers the tip by 0.9 mm and moves it
0.4 mm sideways, and both are corrected.

**Step 5. Check by drawing.** Neighbouring arms each draw a cross at the same place. They should
coincide to within a millimetre. This is looked at, not measured.

**How often**
- steps 1 to 3: once, and again when an arm or the frame has been moved
- step 4: before every job, a few seconds per arm; it also catches wear
- while drawing: the controller finds the paper by contact at each pen-down
- later, once the calibration is good, the arms can revisit the dimples by themselves to confirm
  nothing has moved

**Output.** One dated file per arm: pose, tip, height map, fit errors, pass or fail. The drawing
server refuses to draw without a passing calibration.

**Steps 2 and 3 as built (decided 2026-10-05): marks, pivots, pairs.** Checked against the
drawable maps, the single four-arm spot of the first design does not work (12 % of the pen
orientations there, and one point shared across two rows leaves the end rows' yaw free). The
layout is ten spots, each shared by exactly two neighbouring arms (`docs/figures/marks_six_slots.png`):
the row pairs share two spots on the centre line (A/B at (0, ∓0.40) for the middle row, (0,
∓0.81) and (0, ∓1.61) for the end rows), the column pairs share one seam spot each at (±0.20,
±0.605). Every arm therefore has at least two shared spots (its yaw), and every seam is tied by
two points (the yaw between rows). The frame is anchored on the arms' mountings, not on the
marks: the solved layout is moved so that the mean of the solved slot positions equals the mean
of the nominal ones (the technical drawing, 1–2 cm) and the mean yaw error is zero — three
numbers, the frame's gauge. (Not a point fit of the positions: two slots 0.61 m apart, each
1–2 cm off, would turn such a fit by degrees.) The marks, drawn by hand 2–5 cm from their
nominal spots, are solved wherever they are. Anchoring on a mark would shift the
steel, the fences and the drawing area by the mark's error against the real table (decided
2026-10-05 after Pete's "expect 2–5 cm"). Per arm: one pivot (3–4 hand orientations at its first
spot, which gives the pen tip) and a single touch at every other spot; two arms ≈ 10 touches,
six ≈ 38. The protocol is `docs/figures/mark_protocol.png`: `aris mark` once; then the arm's
light (blue: stay clear, white: guide) and the pilot buttons (✓ registered, ✗ redo, ○ skip) are
the whole interface. The person's final position and orientation are the sample; the planned
hover is only the approach. Calibration is flown by its own driver (libfranka through
panda-py, with Desk for modes and buttons), with the arm's ROS stack stopped for the job — a
mode switch never reaches a running controller.

**Subsets.** `aris mark` does every controlled slot; `--group row2 | rows12 | rows23` or a list
of slots does fewer. x, y and yaw only exist relative to the spots, so a spot's position must be
known for an arm to be placed by it: a full or group calibration solves arms and spots
together; a later subset uses the spots solved before as known (`config/calibration/marks.json`
carries each spot's state, nominal or solved with its date); a single arm needs two already
solved spots, else the solver refuses with "needs a partner".

## Checker

An independent check of everything that goes to a robot. It shares no code with the planners.

## Open questions

- how much a follower can draw next to a leader's footprint, measured on the corpus
- how the footprint is represented (swept capsules or a voxel grid)
- what the fill phase looks like (walls shifted, or one arm at a time)
- how a line that crosses a wall in both phases is cut, and whether the two halves overlap
