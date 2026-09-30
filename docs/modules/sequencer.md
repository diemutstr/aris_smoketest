# sequencer — the tour of one arm

**Job.** The local planner says how each piece of the drawing can be drawn (a bunch of up to
four alternative plans per piece). The sequencer decides which alternative, in which order and
in which direction, and fills in everything between the pieces: the pen coming off the paper,
the move to the next piece, the pen going down again. Out comes the arm's whole job as a list
of timed motions, from where the arm stands back to where it should end.

Files: `aris/sequencer/tour.py` (the loop and the report), `lift.py` (lift-off and set-down),
`draw.py` (drawing motions), `guard.py` (what the sequencer checks itself).

## In and out

`tour(arm, bunches, q_start, obstacles, rules, q_end=None, free_options=None)`

- **In:** the arm, the local planner's bunches, where the arm is, the obstacles (base frame),
  the drawing rules (draw speed, lift height 25 mm, speed share, gates), where the arm should end
  (default: where it started), and settings for the free-space planner.
- **Out:** a generator. It hands out `Motion`s one at a time, in order, and at the end returns
  the pieces it could not draw (`Leftover`, reason and detail). `tour_all` collects everything;
  a `TourReport` passed in is filled as it goes (pieces, lifts, drawing time, pen-up time and
  share, longest free motion, joint travel of the moves, free-space calls and refusals, time
  to the first motion, CPU and wall time).

Per piece it hands out four motions:

| motion | kind | what |
|---|---|---|
| move | free | from where the arm is to the piece's first lift-off configuration (free-space planner) |
| set-down | lower | the pen straight down onto the paper: the lift-off flown backwards |
| drawing | draw | the piece, pen on the paper |
| lift-off | lift | the pen straight up 25 mm at the end of the piece |

and at the end one move to `q_end`. Every motion starts exactly where the one before ended
(to 1e-9 rad), starts and ends at rest, and ends in a configuration the arm can hold: before a
motion is handed out, its end is checked with the kernel (joint-limit margin, clearance to
every obstacle, the arm against itself; with the pen exempt from the paper where it is meant to
touch it). An empty drawing gives only the move to `q_end` (nothing, if the arm is already
there).

## How the next piece is chosen

Greedy. From where the arm is, every piece still to draw is priced in each of its alternatives
and both directions: the time the slowest joint needs to reach that alternative's first drawing
configuration at the allowed speed (joint distance divided by 0.3 of the joint speed limit).
That costs nothing, so no free-space move is planned just to compare candidates (lesson L75).
The cheapest is tried: its lift-offs and drawing motion are made, then the free-space planner
is asked for the move. If anything refuses, the next cheapest is tried. Ties go to the earlier
piece, so the same input always gives the same tour, also from a fresh process.

## The lift-off

The drawing configuration with the pen tip raised 25 mm straight up along the paper normal, by
a short move: the IK is solved every 2 mm of rise, each answer the one nearest the last, so
the arm keeps its shape (the same IK branch; a jump over 0.05 rad means it cannot). Every sample
must pass the gates (joint-limit margin, singular value), the timed move must be free as flown
(the pen is exempt from the paper, it is on its way to or from it), and the arm must be able to
stand at the top with the pen judged against the paper too. The set-down is the same timed
motion flown backwards. The lift-off is the nearest pen-up configuration to the drawing one,
never one picked for clearance somewhere else (lesson L53).

Holding the hand's orientation and joint 7 exactly sometimes fails: a joint-limit margin, a
change of shape, or a parked neighbour that the elbow swings toward as the arm rises (lesson
L7). Then joint 7 and the hand's turn about the paper normal are allowed to change a little on
the way up (up to 0.5 rad each, smallest first); the tip still goes straight up. On the fixed
cases this recovered 0.5 m of the word for arm 13 and several rim lines.

A piece whose alternatives all lack a lift-off (at either end, in either direction) becomes a
leftover `no_free_path`, "no lift-off at its start/end", with the reason of the first try.

## The drawing motion

The local planner's joint path is timed here in one call (`kernel.retime`, pen within 0.1 mm of
the planned pen path) and checked as flown against every obstacle: one piece, one drawing
motion. The timing step keeps the pen within 0.5 % of the draw speed (on two of 1 409
drawings it read 3.6 to 3.8 % over at 1 kHz; see "What the checker says").

The local planner times each plan only to report its duration (corner to corner) and hands
over no trajectory, so the sequencer times it again; the local planner's `draw_time` is not the
flown duration (left as is, orchestrator 2026-09-30).

A drawing that cannot be timed or flown makes the candidate unusable; if no alternative works
the piece is a leftover `unreachable`, "drawing: ...".

## When the free-space planner refuses

The refusal counts only from where the arm is; the next cheapest candidate is tried (another
alternative, another piece). A piece all of whose candidates were refused from here becomes a
leftover `no_free_path` with the refusals. After 12 refusals in one step the pieces refused so
far become leftovers and the step starts again with the rest. On the fixed cases below the
free-space planner never refused (0 of 382 calls). If the final move to `q_end` is refused, the
report says so (`end_refusal`); the last motion still ends holdable.

## What the checker says

Every motion of every case below went through the independent checker (branch aris3,
2026-09-30, the checker that knows `lower` and `lift`): **1 407 of 1 409 motions pass**; the
tightest clearance beyond the demanded one is 0.6 mm (arm 31, scatter). The two that fail are
drawings of arm 31 (random lines along the rim, `line:rim:190` and `line:rim:176`), on "tip
speed": 20.75 and 20.73 mm/s against the checker's 20.6 (3 % over 20). Saved for the timing
step's owner: `tests/data/arm_speed_31_lines_0.npz`, `_1.npz`.

The word at the old drawing speed (80 mm/s, arm 31): 52 of 53 pass. One drawing fails "never
stops": 1.03 mm/s at its slowest mid-way, against 4 mm/s (5 % of 80), at a sharp corner.
Saved for the checker's owner: `tests/data/arm_stop_31_word_0.npz` (trajectory, planned tips,
piece, q_before). At 20 mm/s the same corner passes.

The first round (2026-09-30, morning) failed every motion on two disagreements, since settled:
link 1 against its own struts, and the pen at the paper end of a set-down or lift-off (now the
kinds `lower` and `lift`).

## What it cannot do

- Greedy on the move to the next piece only; it does not look at where a piece leaves the arm.
  On the word, a better order of the same alternatives (moving one piece at a time, with the
  same price) prices the moves about 20 % lower (17.4 against 21.6 s), on a corpus drawing 6 %.
  The next lever: choose order, alternative and direction together along the tour
  (OPTIMIZATION_NOTES 13, lesson L54).
- The price is a lower bound on the move's time; the free-space moves took 1.1 to 3 times it,
  and a detour up to 9 times.
- A piece refused from where the arm is may be reachable from elsewhere; it is not offered
  again later.

## Measured (2026-09-30, branch aris3, machine load 4 to 15 on 32 cores, one process)

From park back to park; obstacles of the arm's leading phase (arm 31 phase 2, arm 13 phase 1);
kinematic table on; drawing speed 20 mm/s. Planning time includes the local planner. "Moves" is
the joint-space path length of the moves between pieces. Pen-up time is every motion that is not
drawing (moves, set-downs, lift-offs). No hatch lines lie within 0.80 m of arm 31.

| arm, case | lines | pieces drawn | planning CPU / wall s | first motion s | drawing s | pen up s | pen-up share | longest free s | moves rad | leftovers (m) | checker |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 31 word | 13 | 13 | 2.3 / 2.3 | 1.6 | 122.9 | 43.1 | 0.259 | 7.6 | 49.6 | unreachable 0.198 | 53 / 53 |
| 31 scatter | 27 | 22 | 4.1 / 4.1 | 2.8 | 140.8 | 83.0 | 0.371 | 13.2 | 94.9 | blocked 0.320, no path 0.177, unreachable 0.111, too short 0.002 | 89 / 89 |
| 31 starburst | 24 | 25 | 11.6 / 11.7 | 10.1 | 706.6 | 66.2 | 0.086 | 8.3 | 68.0 | blocked 0.908, no path 0.417, unreachable 0.429 | 101 / 101 |
| 31 spiral | 3 | 7 | 9.1 / 9.1 | 8.6 | 454.9 | 28.9 | 0.060 | 6.0 | 36.7 | blocked 1.135, unreachable 0.278 | 29 / 29 |
| 31 duotone | 5 | 5 | 3.2 / 3.2 | 2.5 | 207.5 | 35.3 | 0.145 | 9.4 | 41.1 | blocked 0.411, unreachable 0.208 | 21 / 21 |
| 31 lines | 100 | 103 | 37.9 / 38.3 | 31.5 | 2242.0 | 178.7 | 0.074 | 7.5 | 138.7 | blocked 2.325, no path 0.198, unreachable 0.015 | 411 / 413 |
| 13 word | 13 | 12 | 2.0 / 2.0 | 1.5 | 121.3 | 31.9 | 0.208 | 4.9 | 36.6 | unreachable 0.199, blocked 0.014 | 49 / 49 |
| 13 hatch | 43 | 43 | 17.9 / 18.2 | 14.8 | 1518.7 | 60.0 | 0.038 | 4.7 | 42.0 | none | 173 / 173 |
| 13 scatter | 24 | 23 | 2.8 / 2.8 | 2.1 | 142.6 | 57.6 | 0.288 | 5.9 | 61.3 | blocked 0.164, unreachable 0.011 | 93 / 93 |
| 13 starburst | 5 | 6 | 1.9 / 1.9 | 1.7 | 53.3 | 24.0 | 0.311 | 9.2 | 23.8 | blocked 0.489, unreachable 0.163 | 25 / 25 |
| 13 spiral | 3 | 2 | 1.6 / 1.6 | 1.6 | 73.7 | 6.5 | 0.081 | 2.9 | 5.7 | blocked 0.523, unreachable 0.147 | 9 / 9 |
| 13 duotone | 5 | 4 | 3.5 / 3.5 | 3.1 | 175.0 | 19.3 | 0.099 | 5.8 | 23.6 | blocked 0.286, unreachable 0.606 | 17 / 17 |
| 13 lines | 100 | 104 | 30.7 / 30.8 | 25.2 | 2214.7 | 172.2 | 0.072 | 5.5 | 130.1 | blocked 1.652, no path 0.250, unreachable 0.014 | 417 / 417 |

- Every lift and set-down takes 0.25 to 0.35 s; the moves between pieces 34 s of the word's 43 s
  pen-up time for arm 31.
- The sequencer's own CPU (lift-offs, timing the drawings, the moves, the checks) is 0.5 to
  0.7 s for the word, 5.6 to 6.4 s for 100 lines; the rest is the local planner.
- Free-space planner: 382 calls, 0 refusals. The leftovers "no path" (6 pieces, 1.04 m) are all
  pieces without a lift-off at one end: a joint-limit margin, a change of shape 2 mm up, or
  within 0.15 mm of the clearance.
- **Against the old planner, word, arm 31.** Old: planned in 66.8 s, its motion took 67.8 s (it
  drew at 80 mm/s). New: planned in 2.3 s of CPU (first motion after 1.6 s); at 20 mm/s the
  motion takes 166.0 s (122.9 drawing, 43.1 pen up); at the old 80 mm/s 82.7 s (39.6 drawing,
  43.1 pen up, 13 pieces, 0.198 m of the last "n" beyond the reach).

![the tour of the word for arm 31](figures/arm_word_31.png)

Tests: `tests/test_sequencer.py` (the price, the lift-off going straight up and the set-down
being the same backwards, one piece being one drawing motion, an empty drawing, a start nothing can leave, a
piece under a box that is left over while the rest is drawn).
