# kernel.retime: joint path to timed trajectory

**Job.** Take a joint path with no timing and say when the arm is where, so that the FR3 driver
accepts it: velocity, acceleration and jerk inside the limits when the driver samples it every
millisecond.

**In.**
- `path`: joint configurations, (N, 7), joined by straight pieces
- `limits`: joint position, velocity, acceleration and jerk limits
- `rules`: `speed_fraction` (share of the velocity limit that may be used, 0.3) and `draw_speed`
- `s` (drawing motions only): how far along the drawn line each sample is, in metres, increasing
- `tip_of` and `tip_budget_m` (optional): a function from joints to pen-tip positions, and how
  far the pen tip may stray from the input. For drawing, the budget defaults to 0.1 mm when
  `tip_of` is given.

**Out.** A `Trajectory`: times, configurations and joint velocities. Between two samples the motion
is the cubic that matches both. It starts and ends at rest, exactly at the first and last input
configuration. `retime_detailed` also returns the arc length at each sample (for drawing), how far
the flown path is from the input (joint space, and at the pen tip if asked), and the 1 kHz check.

If the path can't be timed, the result is a `Refusal` (from `aris.types`), never an exception. Its
reasons:

| reason | when |
|---|---|
| `too_few_samples` | fewer than two samples |
| `not_finite` | NaN or infinity in the path |
| `outside_limits` | a sample is outside the joint limits |
| `no_motion` | all samples are the same |
| `bad_arc_length` | `s` has the wrong length, is not finite, goes backwards, or stands still while the joints move |
| `bad_rules` | `speed_fraction` not in (0, 1], `draw_speed` not positive, or a tip budget without `tip_of` |
| `cannot_smooth` | the deviation budget can't be met (corners too sharp for the grid, or the pen-tip budget not met after 12 tightenings) |
| `leaves_limits` | the rounded path leaves the joint limits |

`sample(traj, t)` gives position, velocity and acceleration at any times. `check(traj, limits,
rate_hz)` samples at a rate, takes differences like the driver does, and reports the largest
velocity, acceleration and jerk per joint against the limits.

## Why the corners are rounded first

The old code timed straight pieces directly and bounded velocity only. Where two straight pieces
meet, the direction changes instantly. The acceleration at that point is an impulse, however
slowly it is flown. So the reading depends on how closely you look: the same trajectory read
38 rad/s² at 48 Hz and 1 422 rad/s² at 1 kHz, against a driver limit of 10. Slowing the clock
cannot fix that; only changing the path can.

So the path is smoothed before anything is timed. Each point is replaced by an average of its
neighbours along the path (three box averages in a row). A straight piece averages to itself, so
straight pieces are kept exactly and only the corners are rounded.

The window is as wide as the deviation budget allows at each place. First the narrowest width
that works everywhere is found. Then wider windows (4×, 16×, …) are blended in wherever they
also stay within budget. A weighted mix of paths that are each within budget is itself within
budget. So one sharp corner does not force tight, slow curves on the rest of the path.

The ends are mirrored through the end points, so they stay exactly where they were. Deviation is
measured at the same position along the path. The joint-space budget defaults to 0.15 mrad,
which keeps the pen tip within 0.2 mm: over the FR3's whole joint range, one milliradian of joint
motion moves the tip at most 1.28 mm (measured on 20 000 random configurations). With `tip_of`,
the pen tip is also compared with the input's tip line, and the joint budget is tightened until
the tip is within `tip_budget_m`.

## How the speed is chosen

1. **Fastest allowed speed.** One forward pass accelerates as hard as the joint limits allow; one
   backward pass brakes as hard as they allow. The speed is also capped at the velocity limit
   times `speed_fraction`, where bending would make acceleration or jerk too large, and at the
   draw speed for drawing.
2. **Turns take at least 20 ms.** A turn shorter than that would last only a few driver ticks,
   and the reading would depend on where the ticks fall. Turns too gentle to use more than 10%
   of the acceleration and jerk limits are exempt.
3. **Soften.** The passes switch from full acceleration to full braking instantly, which is
   infinite jerk. Progress along the path is averaged over 3 × 25 ms, which bounds the jerk and
   leaves constant-speed stretches unchanged. Before that, the speed cap is lowered around every
   slow spot, by the distance travelled in one averaging window, so the averaging can't carry
   speed into the slow spot.
4. **Write out and check.** Samples go at least every 5 ms, closer on turns (never closer than
   2 ms). Their velocities make acceleration continuous across samples, so jerk is finite
   everywhere. The result is checked at 1 kHz. If anything is over its target, the whole clock is
   slowed by the exact factor needed; in every test case so far that factor was 1.0005.

## What is guaranteed at 1 kHz

At the driver's 1 kHz, finite differences stay within these targets: velocity at most
`speed_fraction` × the limit, acceleration and jerk at most 0.9 × the limit. Positions stay inside
the joint limits, to the driver's own tolerance of 1e-7 rad. The flown path is within the
deviation budget of the input, and the pen tip within its budget if one was given. A drawing
motion never stops or reverses between its ends, and runs at `draw_speed` wherever no joint
limit is in the way.

## What it cannot do

- It doesn't know about obstacles. The rounded path cuts corners by up to the deviation budget,
  so whoever checks collisions must allow that much. A free-space motion can afford a larger
  budget, and should be given one.
- Many small kinks still cost time, because each one is a turn. A jittery 2 000-sample free path
  takes 13.7 s at 0.15 mrad, 6.9 s at 1 mrad and 4.2 s at 4 mrad, against 3.5 s if only the
  speed limit counted. Hand it a shortened path, with the budget your clearance allows.
- It times one motion. Keeping several arms on a shared clock is not its job.

## Measured (FR3 limits, speed fraction 0.3, targets 0.9 of acceleration and jerk)

| case | duration | velocity / (0.3 × limit) | accel / limit | jerk / limit | deviation |
|---|---|---|---|---|---|
| 90° corner | 2.06 s | 0.9995 | 0.854 | 0.050 | 0.108 mrad |
| 175° near-reversal | 2.63 s | 0.9995 | 0.854 | 0.050 | 0.109 mrad |
| random 9-point zigzags (3 seeds) | 10.4–11.4 s | 0.9995 | 0.854 | 0.050 | 0.108 mrad |
| corners on the joint limits | 9.85 s | 0.9995 | 0.854 | 0.050 | 0.107 mrad |

**Same answer at every rate** (90° corner, largest over joints; the old code grew about 30×
between 48 Hz and 1 kHz):

| rate | velocity | accel | jerk |
|---|---|---|---|
| 100 Hz | 0.786 | 8.541 | 238.2 |
| 1 kHz | 0.786 | 8.542 | 252.1 |
| 4 kHz | 0.786 | 8.543 | 252.1 |

At 100 Hz the jerk reads 6% low, because the speed-softening windows are shorter than a 100 Hz
difference spans. From 1 kHz up it doesn't change: 16 kHz gives 252.08 as well.

**Drawing through the real arm model.** Pen-tip shapes on the paper 0.97 m below the base: 10 cm
lines, 10 cm-radius half-circles and 6-segment zigzags, 1 mm apart, at random places and
directions. Each is made into a joint path with `Arm.hand_pose` and `Arm.ik`, following one IK
branch inside the gates. Of 36 shapes, 23 had such a branch. At 20 mm/s:
- pen speed error, away from the ends and zigzag corners: at most 0.68% (lines and arcs 0.04% or
  less)
- pen deviation from the input line: 0.007–0.040 mm with the joint budget alone, all inside the
  0.1 mm default
- a joint limit binds on none of the 23 shapes. The most any shape needs is 0.20 of the allowed
  joint speed.
- tightening works: asking for 5 µm at the pen gives 3.9 µm, and the zigzag takes 9.84 s instead
  of 9.30 s

**Drawing, synthetic arm** (1.5 rad per metre of pen motion): circle 0.05%, square 0.01%, zigzag
0.00% speed error. Where the joints must move 150 rad per metre of line, the pen slows to
10.6 mm/s there and stays at 20 mm/s elsewhere.

**Other results.**
- Ends: velocity exactly 0; configuration equal to the input's (the tests check to 1e-12).
  Acceleration at the very ends is below 0.01 rad/s²; the smooth spline doesn't force it to zero.
- Determinism: the same input gives a bit-identical output.
- Speed, CPU time of one process: a 2 000-sample drawing path takes 66 ms; a 2 000-sample
  jittery free path takes 242 ms. The long random zigzags (about 10 rad of joint travel with
  sharp corners, a smoothing grid of about 500 000 points) take 0.9–1.2 s each.
