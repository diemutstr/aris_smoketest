# Planner design

Agreed with Pete, 2026-09-29. Clean rebuild. Start simple; speed-ups are collected in
`OPTIMIZATION_NOTES.md` and only built when a measurement asks for them.

Three levels. Each level has one job, calls only the level below it, and returns two things:
what it planned, and what it could not plan and why.

```
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

**Rows.** The rig is three rows of two arms. A wall between neighbouring rows (a plane in the
table frame; every arm stays on its own side, half the arm-to-arm margin away) makes the three
rows independent of each other by construction. Each row is then a pair: one leader, one follower.

**Inside a row, per phase**
1. The leader plans first. Its only obstacles are the row walls, the frame, the paper and the
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
| 3 fill | what is left: lines that cross a row wall, and lines neither turn could draw |

Every arm gets one turn with priority, so every line is offered once to an arm that is not
restricted by its partner.

**Allocation.** Each line goes to the arm in its row that reaches it best; where both reach it
equally, to the leader. A line that crosses a row wall waits for the fill phase. Lines are not
cut unless no arm can draw them whole; a cut gets an overlap at the joint.

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

## Checker

An independent check of everything that goes to a robot. It shares no code with the planners.

## Open questions

- lift-off height: 5 to 10 mm is the target; it has to exceed the paper height error plus the
  press depth, so it depends on calibration and contact-finding
- how much a follower can draw next to a leader's footprint, measured on the corpus
- how the footprint is represented (swept capsules or a voxel grid)
- what the fill phase looks like (row walls shifted, or one arm at a time)
- how lines that cross a row wall are cut, and whether the two halves overlap
