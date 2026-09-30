# kernel.retime: joint path to timed trajectory

**Job.** Take a joint path with no timing and say when the arm is where, so that the FR3 driver
accepts it: velocity, acceleration and jerk inside the limits when the driver samples it every
millisecond.

**In.**
- `path`: joint configurations, (N, 7): the corners of a polyline (default), or, with
  `smooth=True`, samples of a smooth curve (see "Two kinds of path" below)
- `limits`: joint position, velocity, acceleration and jerk limits
- `rules`: `speed_fraction` (share of the velocity limit that may be used, 0.3) and `draw_speed`
- `s` (drawing motions only): how far along the drawn line each sample is, in metres, increasing
- `tip_of` and `tip_budget_m` (optional): a function from joints to pen-tip positions, and how
  far the pen may stray from where the input path puts it. For drawing, the budget defaults to
  0.1 mm when `tip_of` is given.
- `smooth` (default `False`): what the samples are. `False` gives exactly the numbers it gave
  before the option existed.

**Out.** A `Trajectory`: times, configurations and joint velocities. Between two samples the motion
is the cubic that matches both. It starts and ends at rest, exactly at the first and last input
configuration. `retime_detailed` also returns:
- the arc length at each sample (drawing only)
- how far the flown path is from the input, in joint space and at the pen if asked
- `width`: the narrowest corner window (with `smooth=True`: the smaller of the shortest gap
  between two input samples and the narrowest window at a corner of the drawn line)
- the report: the exact extremes of the cubic pieces against the limits

If the path can't be timed, the result is a `Refusal` (from `aris.types`), never an exception. Its
reasons:

| reason | when |
|---|---|
| `too_few_samples` | fewer than two samples |
| `not_finite` | NaN or infinity in the path |
| `outside_limits` | a sample is outside the joint limits |
| `no_motion` | all samples are the same |
| `bad_arc_length` | `s` has the wrong length, is not finite, goes backwards, or stands still while the joints move |
| `bad_rules` | `speed_fraction` not in (0, 1], `draw_speed` not positive, or a pen budget without `tip_of` |
| `cannot_smooth` | a deviation budget can't be met |
| `leaves_limits` | the rounded path (or, with `smooth=True`, the curve through the samples) leaves the joint limits |
| `too_slow` | the motion would take more than 600 s |

`sample(traj, t)` gives position, velocity and acceleration at any times. `check(traj, limits,
rate_hz)` samples at a rate, takes differences like the driver does, and reports the largest
velocity, acceleration and jerk per joint against the limits.

## Why the corners are rounded first

The old code timed straight pieces directly and bounded velocity only. Where two straight pieces
meet, the direction changes instantly. The acceleration at that point is an impulse, however
slowly it is flown. So the reading depends on how closely you look: the same trajectory read
38 rad/s² at 48 Hz and 1 422 rad/s² at 1 kHz, against a driver limit of 10. Slowing the clock
cannot fix that; only changing the path can.

Near each corner, the path is replaced by an average of its neighbours (three box averages in a
row, over a window of width w). A straight piece averages to itself, so only the corners change.
The rounded path is the input plus one small, exactly known bump per corner, and the bump reaches
only 1.5 w either side. Four consequences:
- The path is evaluated exactly, at any point, without a grid.
- Every corner gets its own window.
- A lone corner moves the path by 0.2 × w × (its change of direction), largest at the corner.
- Windows stay clear of the path's ends, so the ends are exact.

Each window is as wide as the budgets allow. Corners that break a budget are narrowed; the others
are left alone.
- **Joint budget:** 0.15 mrad by default. That keeps the pen within 0.2 mm, because over the
  FR3's whole joint range one milliradian of joint motion moves the pen at most 1.28 mm
  (measured on 20 000 random configurations).
- **Pen budget:** with `tip_of`, the pen is also compared with where the input path puts it,
  that is, the pen positions of the input joint path. Any sag of that path between its IK
  samples belongs to the planner that made it; the rounding adds at most the pen budget to it.

Deviation is always measured at the same position along the path.

## Sharp corners in a drawn line

A whole drawing path, zigzags and letters included, is timed in one call. At a corner of the
line, the pen rounds the corner inside the pen budget: it cuts the corner by at most 0.1 mm.
It slows down, more for sharper corners, because a turn must take at least 20 ms (see below).
Measured with a 1.5 rad-per-metre test arm at 20 mm/s:

| corner of the line | slowest pen speed there |
|---|---|
| 30° | 19.3 mm/s |
| 90° | 11.8 mm/s |
| 150° | 3.5 mm/s |
| 179° | 0.34 mm/s |
| 180° (the line doubles back) | stops for an instant |

The pen stops only at a true 180° cusp. There the joints reverse, so their velocity must pass
through zero. At every other corner the pen keeps moving. It never leaves the line by more than
the pen budget. The reported figure, 0.096–0.100 mm in these cases, is an upper bound: the
rounded path's measured pen deviation plus 1.3 m/rad times how far the written-out cubic strays
from it. Where that bound would exceed the budget, the pen is measured with `tip_of` at those
points.

## Two kinds of path: corners and smooth curves

The corner model above is right for a free-space path: the planner hands over a few waypoints
joined by straight moves, and the corners are real. It is wrong for a drawing motion, a lower or
a lift. There the planner solves the IK every 2 mm (down to 0.25 mm where it refines), so the
samples lie on a curve. Where a joint turns round while the others go on, consecutive samples
make "corners" of 90 to 130 degrees that are a fraction of a millimetre long. The corner model
rounds each of them inside a window of at most four sample gaps, and a turn must last 20 ms,
so it crawls through every one.

Measured on the case that showed it (line "big:8859" of the 10 000-line drawing, arm 13,
`tests/data/retime_smooth_arc.npz`): 109 samples, 7.6 s for 10.6 cm, and the pen down to
1.7 mm/s mid-line, with the joints at 1 to 3 % of their speed limits there. The joint path is
not quite smooth either, and it pays to know why. The line itself is a polyline of 12 vertices
9.7 mm apart that turns 10.7 degrees at each. Between 39 and 53 mm, joint 7 and the hand's spin
ramp up and down with a kink at each end. And at 48 mm the joints move 65 rad per metre of line
(near the singular-value gate). The 128-degree "corner" at 52.3 mm is where joint 7's ramp ends.

**`smooth=True`** says the samples lie on a smooth curve. The path is then the cubic spline
through the samples, twice differentiable, parametrised by `s` (else by the joint-space chord
length). It has no corners to round and no 20 ms rule, so the speed is set only by the
derivatives of the curve against the same targets as before, and by the draw speed. Deviation is
measured from the spline, and the pen from the polyline through the input's pen positions.

A drawn line can still have a real corner (a letter, a zigzag). A spline through a sharp corner
swings wide of the line. So, with a pen budget, the spline's pen is compared with the pen
polyline, three points per sample gap. Where it strays by more than half the budget, the spline
is cut at the nearby sample where the pen turns most. The pieces meet the cut with zero second
derivative, and the corner is rounded with the same bump as above, inside both budgets and with
the 20 ms rule. On the cases below it cuts only the word "unknown" (18 of 47 plans for arm 13,
20 of 48 for arm 31, at vertices where the pen turns 18 to 148 degrees, median 27); the random
straight lines and the arc's 10.7-degree vertices stay inside the spline. Without a pen budget (lifts) there are no
cuts.

One more thing changes with `smooth=True`: output samples are also placed where the path's own
acceleration changes (its speed-up and slow-down ramps). On a strongly bending curve the joints'
acceleration is mostly bending, the ramps got too few samples, and the cubic overshot the draw
speed where a ramp ends (+1.8 % measured on a rim line).

## How the speed is chosen

1. **Fastest allowed speed.** One forward pass accelerates as hard as the joint limits allow; one
   backward pass brakes as hard as they allow. This runs over about 500 points spread along the
   path, plus 12 across every narrow, sharp corner. The speed is also capped at the velocity
   limit times `speed_fraction`, where bending would make acceleration or jerk too large, and at
   the draw speed for drawing. This step lives in `kernel/speed.py`. The two passes are a small
   compiled function (`native/retime`). A numpy version gives the same numbers bit for bit, and
   is used when the compiled one isn't installed. `retime.backend()` says which one ran:
   `"native"` or `"numpy"`.
2. **Turns take at least 20 ms.** A turn shorter than that would last only a few driver ticks,
   and the reading would depend on where the ticks fall. Turns too gentle to use more than 10%
   of the acceleration and jerk limits are exempt. With `smooth=True` this applies only at the
   rounded corners of the drawn line, not to the curve's bends; there the speed-up nodes are
   also placed at every sample and halfway between, so no speed cell straddles a sample.
3. **Soften.** The passes switch from full acceleration to full braking instantly, which is
   infinite jerk. Progress along the path is averaged over 3 × 25 ms, which bounds the jerk and
   leaves constant-speed stretches unchanged. Before that, the speed cap is lowered around every
   slow spot, by the distance travelled in one averaging window, so the averaging can't carry
   speed into the slow spot.
4. **Write out and check.** Samples are placed where the path turns and where the joints' speed
   or acceleration change: at least every 50 ms, never closer than 6 ms, and every 6 ms in the
   first and last 30 ms. Ramps are sampled densely enough that even a copy resampled at 100 Hz
   reads the same at 1 and 4 kHz (the checker tests this). Their velocities make acceleration continuous across samples, so jerk is
   finite everywhere. The exact extremes of each cubic piece are then compared with the targets.
   If anything is over, the whole clock is slowed by the exact factor needed.

## What is guaranteed

The report holds the exact largest velocity, acceleration and jerk of the cubic pieces. A sampled
measurement at any rate, 1 kHz included, averages these and can only read less. Guaranteed:
- velocity at most `speed_fraction` × the limit
- acceleration and jerk at most 0.9 × the limit
- positions inside the joint limits, to the driver's tolerance of 1e-7 rad
- the flown path within the deviation budget of the input, and the pen within its budget if one
  was given
- a drawing motion never reverses; it stops only at a 180° cusp of the line; it runs at
  `draw_speed` wherever no joint limit is in the way
- with `tip_of`, the pen itself never runs more than 1% over `draw_speed`, read through the
  arm's tip at 1 kHz. Measured worst: +0.10% on the 23 test shapes, +0.00% on 9 letter shapes,
  -0.01% on two lines along the rim of arm 31's reach from the end-to-end run.

**Why the pen, not the path parameter.** Between the planner's IK samples, a straight move in
joint space need not keep the pen in step with `s`. Near a corner of a letter, the pen moved up to
1.24 mm per mm of `s`, and the pen ran up to 3.5% over the draw speed although ds/dt never did.
So, when `tip_of` is given, the speed choice reads how far the pen moves per unit `s` along the
rounded path. Wherever that is over 1, it lowers the path speed by the same factor. Where it is
under 1, the path speed is left alone: the pen is never sent faster than `s` asks. Without
`tip_of`, only ds/dt is held to `draw_speed`.

The output samples also follow the braking at the end of a drawing. Samples are added where a
joint's speed or acceleration changes by a tenth (a twentieth for acceleration) of the motion's
own top value, not of the joint limits. A drawing's joints move slowly, so measured against the
limits a whole braking ramp got almost no samples. The cubic then overshot there: +3.7% at the
pen on the two rim lines.

## What it cannot do

- It doesn't know about obstacles. The rounded path cuts corners by up to the deviation budget,
  so whoever checks collisions must allow that much. A free-space motion can afford a larger
  budget, and should be given one.
- Each corner of a free-space path is a near-stop at a tight budget, because the rounding is
  short. A larger budget makes corners faster.
- It times one motion. Keeping several arms on a shared clock is not its job.
- `smooth=True` trusts the caller: the spline through samples that are not on a smooth curve
  swings between them. With a pen budget, the swing is caught and the spline cut at the corner.
  Without one (lifts, lowers), nothing checks the swing: give it only IK samples of a smooth
  line. A spline through samples that sit on a joint limit can overshoot the limit (refused as
  `leaves_limits`).
- A sharp corner of a drawn line is still slow in both modes: turning the pen by 148 degrees
  within 0.1 mm takes it down to about 1.4 mm/s (the word's "w"). A larger pen budget would make
  it faster.

## Measured

Targets: speed fraction 0.3; acceleration and jerk held to 0.9 of the limit.

| case | duration | velocity / (0.3 × limit) | accel / limit | jerk / limit | deviation |
|---|---|---|---|---|---|
| 90° corner | 2.1 s | 1.0 | 0.86 | 0.05 | 0.135 mrad |
| 175° near-reversal, random 9-point zigzags, corners on the joint limits | 2.6–11 s | 1.0 | 0.86 | 0.05 | 0.135 mrad or less |

**Same answer at every rate** (90° corner, largest over joints; the old code grew about 30×
between 48 Hz and 1 kHz):

| rate | velocity | accel | jerk |
|---|---|---|---|
| 100 Hz | 0.786 | 8.56 | 234 |
| 1 kHz | 0.786 | 8.58 | 250 |
| 4 kHz | 0.786 | 8.58 | 250 |

**Drawing through the real arm model**, at 20 mm/s. The shapes were lines, half-circles and
zigzags at random places on the paper. On 23 shapes, the pen speed is within 0.25% of
20 mm/s away from ends and corners, and the pen stays within 0.052 mm of the input. No joint
limit binds. Asking for a 5 µm pen budget gives 4.99 µm, at 9.42 s instead of 9.28 s.

**Speed, CPU time on one core.** Old and new were run back to back on the same core. The
32-thread machine was at a load of 40 to 65 throughout, which inflates everything by up to
about 2×. Each figure is the median of 15 runs; for free space it is the worst of five random
paths.
- Free-space paths: waypoints 0.2 to 0.8 rad apart.
- Drawing paths: through the real arm model.
  - 250 samples: a 0.74 m letter-like line with sharp corners.
  - 2 000 samples: a 0.8 m wavy arc.

| case | old (load 63) | new (load 40) | target |
|---|---|---|---|
| free-space path, 3 waypoints | 357 ms | 2.6 ms | 5 ms |
| free-space path, 10 waypoints | 1 176 ms | 4.0 ms | 5 ms |
| free-space path, 30 waypoints | 14 066 ms | 5.4 ms | 5 ms |
| drawing, 250 samples | 3 407 ms | 10.0 ms | 10 ms |
| drawing, 2 000 samples | 789 ms | 21.6 ms | 40 ms |
| short, sharp drawing path | refused after 7 424 ms | timed in 2.6 ms | |
| impossible pen budget (1 nm): refused | | 20 ms | 50 ms |

The test suite's own least-disturbed runs, with the test arm: 3.7 ms for 30 waypoints, 4.9 ms
for 250 samples, 10.9 ms for 2 000 samples.

### `smooth=True` against the corner model (2026-09-30)

Tests: `tests/test_kernel_retime_smooth.py`. Pen speed "along the line" is measured as the
checker measures it: progress along the planned pen polyline, sampled at 1 kHz.

**The arc** ("big:8859", arm 13, 109 samples, 10.6 cm):

| | corners | smooth |
|---|---|---|
| duration | 7.63 s | 5.49 s |
| slowest along the line, once it has reached 15 mm/s | 1.75 mm/s (at 52.3 mm) | 14.75 mm/s (at 48.4 mm) |
| flown pen from the planned line | 0.028 mm | 0.040 mm |
| fastest pen | +0.08 % | -0.22 % |
| inside the limits at 1 and 4 kHz | yes | yes |

The one spot under 15 mm/s is where joint 5's acceleration binds: the 10.7-degree vertex at
48.4 mm falls where the joints move 65 rad per metre of line, and the spline takes that vertex
within one 0.22 mm sample gap. Joint 5 reads 0.54 of its acceleration limit there (the speed
choice plans a bend with 0.7 of the 0.9 target, 0.63 of the limit, and leaves the rest for
speeding up and slowing down). Everywhere else the pen is at 19.9 mm/s.

**The arm cases** (every alternative plan the local planner returns for the word "unknown" and
the 100 random lines, arms 13 and 31; slowest mid-line speed as the checker reads it, which
starts counting at a quarter of the fastest speed, so a clean start reads 5 mm/s):

| arm, set | plans | drawing time corners / smooth | slowest mid-line, corners: 1st / 5th / 50th percentile / minimum | smooth |
|---|---|---|---|---|
| 13 word | 47 | 437.5 / 437.1 s | 1.06 / 5.01 / 5.14 / 1.06 mm/s | 1.35 / 4.89 / 5.14 / 1.35 mm/s |
| 13 lines | 390 | 8 015 / 7 950 s | 2.41 / 4.99 / 5.08 / 1.82 | 4.99 / 4.99 / 5.08 / 4.99 |
| 31 word | 48 | 481.9 / 482.0 s | 0.99 / 1.14 / 5.13 / 0.99 | 1.50 / 1.69 / 5.14 / 1.50 |
| 31 lines | 370 | 7 852 / 7 720 s | 1.76 / 2.58 / 5.05 / 1.12 | 4.99 / 4.99 / 5.08 / 4.99 |

With `smooth=True` nothing is refused, everything is inside the limits, the flown pen stays
within 0.019 mm of the planned line (corners: 0.059 mm), and the fastest pen is -0.03 % over
the draw speed (corners: +3.06 % on a rim line of arm 31). The only plans under 2 mm/s are the
word's "w" (word:10, three 148-degree corners), 1.35 to 1.69 mm/s against 0.99 to 1.14 with
corners. Drawing time barely changes, because the draw speed, not the crawl, sets it: the crawl
costs seconds per line, a line takes tens.

**A lift** (11 samples 2 mm apart along the paper normal, no `s`): from the arc's end 0.278 s
both ways (acceleration-bound, 0.86 of the limit); from the arc's samples 45 and 53, where the
joint path bends, 0.480 -> 0.408 s and 0.588 -> 0.458 s, the peak joint speed rising from 0.70-0.75
to 0.96-0.97 of the target.

**CPU, one call** (least disturbed of seven): the arc 5.5 ms both ways; a 2 000-sample drawing
path (the synthetic wavy line of the speed test) 13.5 ms corners, 13.8 ms smooth (under load
up to 76 and 48 ms). The compiled sweeps needed no change: they work on any nodes, and the
smooth curve only adds nodes (two per sample gap). Compiled and numpy sweeps agree to 1e-12.
