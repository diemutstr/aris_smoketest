# Planner design

Agreed with Pete, 2026-09-29. Clean rebuild. Start simple; speed-ups are collected in
`OPTIMIZATION_NOTES.md` and only built when a measurement asks for them.

Three levels. Each level has one job, calls only the level below it, and returns two things:
what it planned, and what it could not plan and why.

```
system planner      which arm draws which line, in which phase, behind which walls
  arm planner       one arm: a tour of drawing and free-space motions
    sequencer         order, direction, which alternative; free-space moves between lines
    local planner     one line -> a bunch of alternative drawing plans
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

## 2. Sequencer

Wraps the local planner. Together they are the **arm planner**.

- picks one alternative from each bunch, the order of the pieces and the direction of each
- simple heuristic first (greedy)
- each piece starts and ends at a lift-off configuration: the same arm shape, raised a short
  distance off the paper, with a tiny straight motion to set the pen down
- free-space motions between lift-off configurations by RRT, with the paper, the frame, the walls
  and the pen itself as obstacles
- if a free-space motion cannot be found: try another alternative, else hand the piece back
- emits motions as it goes, so the first motion is available before the tour is finished

**Arm planner in:** lines, obstacles and walls in the base frame, the start configuration.
**Arm planner out:** a sequence of drawing motions and free-space motions, plus the leftovers.

## 3. System planner

Only ever calls the arm planner, with different lines and different walls.

- allocates lines to arms
- phases: 1-2-1 arms first, then 2-1-2, then a final phase for what is left in between
- sets up virtual walls per phase so that the arms in a phase can plan and execute independently
- between phases all arms stop in a safe holding configuration
- collects the leftovers of each phase and reassigns them in the next

## Checker

An independent check of everything that goes to a robot. It shares no code with the planners.

## Open questions

- lift-off height: 5 to 10 mm is the target; it has to exceed the paper height error plus the
  press depth, so it depends on calibration and contact-finding
- where the walls go, and how much of the canvas each phase then covers
- what the final phase looks like (one arm at a time, or non-neighbouring arms together)
- how lines that cross a wall are cut, and whether the two halves overlap
