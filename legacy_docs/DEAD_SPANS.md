# The 2.22 m nobody drew — is it unreachable?

Mostly no. Of the 13 empty spans in `out/csail_program_6arm.json` (2.222 m, 13.8 % of the 16.15 m traced), **1.137 m in 7 spans cannot be certified by any arm at any pen length**, and 0.869 m of that is a single failure mode: the under-base holes of inverted arms 2 and 97. The other 1.085 m is recoverable, and 0.429 m of it needs no hardware at all.

Two verdicts, because neither dominates. **Atlas** = pointwise at the strict gates (margin ≥ 0.30, σ ≥ 0.14, 1-cell eroded per `fuzz_planner._go_mask`) — conservative. **Planner** = each span handed to `allocate.probe_stroke`, which certifies at 0.15 / 0.10 but demands a *continuous* walk of the redundancy band — harder in a different direction. Recovered = ≥ 95 % covered. Rebuild both with `python3 scripts/dead_spans.py`.

| # | ink | m | at (x, y) | atlas tier | planner tier | arm | why still dead |
|---|---|---|---|---|---|---|---|
| 0 | grey | 0.139 | 1.699, 0.979 | — | — (0.71 at pen300) | — | waist |
| 1 | grey | 0.109 | 1.766, 0.983 | pen300 | — (0.73 at pen300) | — | waist |
| 2 | grey | 0.020 | 0.990, 0.994 | — | — (0.00 at any pen) | — | waist |
| 3 | grey | 0.086 | 0.988, 0.991 | — | **pen200** | 31 | waist |
| 4 | grey | 0.408 | 0.921, 0.710 | pen300 | **pen200** | 2 (wrong ink) | — |
| 5 | grey | 0.360 | 2.404, 0.837 | — | **repartition** | 97 (wrong ink) | under-base, 97 |
| 6 | orange | 0.039 | 1.443, 1.052 | pen300 | **repartition** | 31 (wrong ink) | — |
| 7 | orange | 0.030 | 1.485, 1.024 | today | **repartition** | 31 (wrong ink) | — |
| 8 | orange | 0.161 | 1.792, 0.931 | — | **pen300** | 97 | waist |
| 9 | orange | 0.339 | 1.131, 0.410 | — | — | — | **under-base, arm 2** |
| 10 | orange | 0.210 | 2.173, 0.334 | — | — | — | **under-base, arm 97** |
| 11 | orange | 0.170 | 2.430, 0.334 | — | — | — | **under-base, arm 97** |
| 12 | orange | 0.150 | 2.362, 0.331 | — | — | — | **under-base, arm 97** |

**Per tier (planner).** re-ink 0.429 m (3 spans) · 200 mm pen 0.494 m (2) · 300 mm pen 0.161 m (1) · **dead 1.137 m (7)**. Length-weighted, ignoring the 95 % rule, some arm certifies 0.767 m today / 1.053 m at 200 mm / 1.233 m at 300 mm — but **0.000 m today by an arm holding the right ink**. The allocator already enumerated all 62 colour partitions, so that 0.767 m is a real two-colour conflict, not an oversight: spans 6–7 want arm 31 orange while spans 4–5 want arms 2 and 97 grey. It needs a mid-piece pen change, or a seventh arm.

**Tilt buys nothing, at any budget.** The inverted arms' eroded strict-GO annulus is 0.27–0.70 m today; ≤ 15° takes the outer edge to 0.72 m, ≤ 30°/45° only open the *inner* radius to 0.01 m. No tilt condition recovers a single span, and the best of them adds 0.15 m length-weighted. It is also not implementable: `planner.sheet_fields` hard-codes `R_w = rotx(π)`. Pen length is the lever that works (200 mm → 0.26–0.82 m, 300 mm → 0.34–0.88 m), but a longer pen *widens* the under-base hole, which is why span 5 falls from 1.00 to 0.92.

**What stays dead, and where.** Spans 9–12 (0.869 m) sit 0.07–0.18 m from the base axis of arms 2 (1.303, 0.351) and 97 (2.304, 0.351): an inverted FR3 cannot draw underneath itself (boom keep-out r < 0.12 m, plus joint limits). Spans 0–2 (0.269 m) sit in the **waist** — the y ≈ 0.99 band midway between the front row (y = 1.631) and back row (y = 0.351), 0.70–0.88 m from every base, where four 0.70 m annuli fail to meet. Arms 13 and 17 are floor-mounted at x = −0.112 / 3.719 with 0.78 m of reach and contribute 0.00 m of the logo; the whole piece is drawn by the four inverted arms.

**Gate caveat, checked.** Atlas-strict over-calls dead, and in both directions. Probing the worst point of each dead span with `plan_stroke` on a 5 cm stroke: span 5 at (2.391, 0.604) — dead under *every* atlas — returns **ok** for arm 97 at 110 mm (margin 0.193, σ 0.238), so the 0.30 gate condemned it, not reach; span 0 at (1.719, 0.979) returns **ok** for arm 2 at 300 mm (margin 0.360). But spans 9 and 10 return `split / start_infeasible, s* = 0` for **all 18 arm × pen combinations** — zero IK solutions even at permissive gates. Those are genuinely unreachable at this base layout. Conversely the atlas is *optimistic* on span 1 (96 % of points GO at 300 mm, only 73 % walkable), so the two verdicts are complements, not a bound and a refinement.

Figure `out/dead_spans_analysis.png`, numbers `out/dead_spans_analysis.json`. Inherited caveats: `h_inv = 1.00` (the rig measured 0.924 — a lower mount shrinks every annulus), yaw-invariant per-arm disks, no arm-vs-arm collision reasoning.

---

## Resolution (2026-08-19): 1.297 m of CSAIL, 99.21 % of it certified

The 2.22 m was never one problem, and the three knobs this study named but did
not turn — a pen length **per arm**, a pen swap **between passes**, and a
placement search scored on **certified metres** instead of on the atlas — take
the piece from 86.3 % of 16.15 m to **99.2139 % of 11.567 m**: one 0.091 m span
left empty instead of thirteen totalling 2.222 m.

**The recipe.** Logo 1.2969 x 0.9908 m (53.8 % of the margin-limited size),
centred at (1.904, 1.031) m — offset (+0.100, +0.050) m from the sheet centre.
Two passes with a human pen swap between them. Pens: **arm 2 = 300 mm, arms 31 /
71 / 97 = 200 mm**, floor arms 13 / 17 = 110 mm and idle (they reach none of
this logo, at any pen). Phase 1 lays 4.064 m of grey, phase 2 lays 7.503 m of
orange, and all four inverted arms draw in both — which is not incidental, see
below.

| knob | what it was worth |
|---|---|
| pen swap between passes | the 0.767 m this study called "a real two-colour conflict" is gone. One pen per arm is a constraint WITHIN a pass, not across one; each phase is an independent single-colour problem with all six arms available. Single-pass tops out at **94.8 %** at any size we could certify — two passes are not a convenience here, they are the difference between 95 % and 99 % |
| per-arm pen length | arm 2 wants 300 mm (it works the waist), arms 31 / 71 / 97 want 200 mm. Uniform 110 mm loses ~1.5 pp, uniform 300 mm loses more: a long pen grows the under-base hole from 0.17 m to 0.26 m |
| placement | the under-base holes are 0.17–0.26 m across at the planner's own gates and the logo has whitespace. +100 mm right and +50 mm up is where the holes fall in it |

**The atlas was the wrong oracle, and by a lot.** `allocate.reach_fraction`
scored the old placements off the 2 cm reachability atlas; on candidates it
called 100 % reachable the allocator certified 67–86 %. A cell the pen can
stand on is not a stroke the redundancy band can be walked along. Replacing it
with a **planner-signed** field — a 5 cm test stroke at every cell of a 5 mm
grid, in three orientations, accepted only if `plan_stroke` returns `ok` —
costs 1.3 M plan calls (≈ 190 s on 30 cores) and predicts the certified answer
to about a centimetre. Measured annuli, per pen:

| mount | 110 mm | 200 mm | 300 mm |
|---|---|---|---|
| inverted (2, 31, 71, 97) | 0.17–0.72 m | 0.17–0.76 m | 0.26–0.79 m |
| floor (13, 17) | 0.19–0.80 m | 0.30–0.80 m | 0.25–0.79 m |

**What is still dead.** One span, 0.091 m of orange at ≈ (1.78, 0.96): the
centre of the four inverted bases, where four annuli of outer radius 0.72–0.79 m
fail to meet across a 1.001 x 1.280 m rectangle (the centre is 0.813 m from
every base). No pen closes it — 300 mm reaches 0.79 m — and no placement of a
logo this size keeps ink out of it. It is the same waist the study found, now
reduced from 0.269 m to 0.091 m and localised to a lens roughly 0.11 x 0.08 m.

**100 % is reachable and not runnable.** At 1.2656 m with arm 71 on a 300 mm
pen, allocation certifies **every metre** — 11.287 m, zero dropped. The
conductor refuses the result: arm 71's 300 mm pen sweeps within the 80 mm
margin of arm 97 standing at its ready pose, and no schedule fixes a conflict
with something that is not moving. So the last 0.79 % of ink costs 2.5 % of logo
width (5.0 % of its area) AND a timeline that will not run: 1.297 m at 99.21 %
is the better trade in both directions, not a compromise in one.

**A simpler rig is 0.18 pp behind.** At the same 1.2969 m and the same offset,
putting **all four inverted arms on one 200 mm pen** certifies 99.0387 % — still
over the bar, with one pen length instead of two among the arms that draw. It is
not the pick only because coverage outranks simplicity at equal size, and it has
not been through the conductor; if the rig would rather stock one pen length,
that is the run to certify next.

**The independent checker earned its keep twice.** Building this exposed two
conductor bugs that every previous run had hidden, both caught by
`scene_check` disagreeing with `coordination` rather than by inspection:
parked arms sorted last in the priority order and so appeared in *nobody's*
collision image, and the reachability DP validated an arm only until it
*arrived*, leaving its resting pose unchecked against arms still moving. Both
are fixed, both have regression tests, and the second is why phase 1 now pays
90.6 s of pauses where it used to pay 4.
