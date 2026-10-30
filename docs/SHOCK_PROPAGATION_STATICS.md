# Shock Propagation and Statics

Why shock propagation made energy, the exact statement of when it does, and what
`solver/shock_propagation/` does about it.

## Status

On branch `shock-statics` (2026-10-30). `StaticsGate` and `WarmStartRemap` run in
`ShockPropagationConditioner` after the contact graph. `physics_fuzz` over 500 seeds:
33 failing → 0, worst unexplained gain 6.45 → 0.16 J/kg; over 2000, 1 fails (see *The
last five seeds*, *The two after them* and *Open*).

## The problem

Heavy-on-light stacks sag, creep or topple without shock propagation; `mass_ratio_sweep`
shows that more iterations do not recover them. With it, dynamic collapses regained
energy: 33 of 500 fuzz seeds, worst 6.45 J/kg (a wall thrown 66 cm up by nothing).
Speed gates, scaling only in the position pass, per-pair ramps and a Box3D-style soft
step were all tried and rejected (`docs/BOX3D_SOLVER_COMPARISON.md`).

## Shock propagation is a change of basis

Take a chain: body `i` rests on body `i−1`, body 1 on the ground, contact `i` under body
`i`. Let `A` be the Delassus operator (relative normal velocity per unit impulse) and
`B_α` the operator the scaled solver actually applies: each contact's impulse reaches its
upper body in full and its lower body times `α`.

The scaled column of contact `j` acts as `+e_j − α·e_{j−1}` on the bodies. The physical
columns are `c_k = e_k − e_{k−1}`, and since the ground absorbs whatever is pushed into
it, `e_j = c_1 + … + c_j`. So

```text
e_j − α·e_{j−1} = c_j + (1−α)·Σ_{k<j} c_k
B_α = A · T_α,    T_α = I + (1−α)·(strictly upper-triangular ones)
```

`T_α` depends on the graph alone, never on masses or inertias (checked exactly with
rational arithmetic on random chains; the iteration matrix `I − B_α⁻¹A` is nilpotent with
mass-independent entries). Consequence: **the velocities the scaled solver produces are
exactly those the physical impulses `λ_phys = T_α·λ_α` would produce.** The momentum
shock propagation seems to make up is momentum carried down through every contact below
to the ground.

With depth differences the factor is `α^Δdepth` and the same telescoping holds along each
body's path to the ground. In 3D the withheld share is a wrench, and it can be carried
down only if the contacts below can produce it.

## Where the energy comes from

If the velocities are physical, energy can only come from `λ_phys` not being a contact
solution. A contact solution pushes (normal ≥ 0), stays within friction, and does no work
at a separating contact. `λ_phys` breaks that in three ways:

1. **Through an opening contact.** `λ_phys,k` includes `(1−α)·(load above k)` even when
   contact `k` is separating: it pushes while opening.
2. **Past friction.** A withheld share leaning off vertical has to be carried by the
   supports' friction; past their cone it cannot be.
3. **Past the support's edge.** A load whose centre of pressure lies outside the lower
   body's support polygon needs a support that pulls. A tipping body is scaled as if
   glued down.

So the statement the code relies on: **scaling is energy-free through any body in static
equilibrium on feasible impulses from the ground side.** It is exact for chains and
measured, not proven, in 3D.

## `StaticsGate`

A body's *carriers* are its met contacts with the ground or with bodies no higher than it
in the graph, other than driven bodies: the player pushes as its drive asks, so its push
is load, not support. Its *load* is its weight over the substep plus every warm-started push from
higher bodies, as a force and a torque about its centre of mass. A body is **grounded**
when:

- it has carriers, none counting if it separates faster than `OPENING_SPEED` (5 cm/s),
  and none closing on it that fast: a carrier striking the body brings an impulse last
  frame's do not hold;
- no driven body (one with a support drive: the player) is closing on it, through a met
  contact or one that has not met yet, for the same reason; and none touching it is
  being driven, travelling or turning as its drive asks, whose push can be anything;
- its load lies in the carriers' wrench cone (`ContactWrenchCone`: each contact's
  four-sided friction pyramid, as wrenches; membership by non-negative least squares to
  `CONE_TOLERANCE`, 2 %);
- every dynamic carrier lower than it is grounded.

Bodies are judged lowest first. A scaled pair whose lower body is not grounded is
unscaled. The carriers include contacts in any direction, so an arch's voussoir counts
its neighbours' thrust; a test of the support polygon under a block alone made voussoirs
flicker between scaled and unscaled and failed `a_knocked_arch_regains_no_energy`.

**Cost.** Three things keep it to a fraction of the frame:

- *Certificate.* Last frame's carrier impulses pushed and stayed within friction, so any
  positive multiple of them lies in the cone. If one balances the load, the body holds
  it, with no solve. (The multiple is needed because under scaling the carriers held the
  body's weight plus only part of the loads above.)
- *Verdicts.* A body's solved verdict is kept until its load or carriers drift 2 % (or
  1 mm per contact) from what it was *solved* for, not from last frame, so slow drift
  cannot creep past it.
- *Small solves.* A solution in six dimensions needs at most six columns (Carathéodory),
  so each least-squares step is a Cholesky solve of at most 8×8 on the stack.

In the fuzz sweep about five in six body checks skip the solve.

## `WarmStartRemap`

Warm starts solved under one set of scales are not a fixed point under another: a resting
body is held by `own + s·load_above`, and when `s` changes the old support impulse pops it
up. In a 1D model, scaling switched on under 100 kg resting on 8 cubes regained 1.07 J/kg;
remapped, 0.000.

Each frame, top-down, each body's vertical supports have their warm starts scaled together
so they carry its own load (unchanged by a change of scale) plus what this frame's scales
pass down to it. The gate changes scales from frame to frame, so it needs this: without the
remap, `a_tower_with_a_block_held_upright_regains_no_energy` regains 0.46 J/kg.

## Measurements

`physics_fuzz --seeds 500`:

| | failing seeds | 99th percentile | worst |
|---|---|---|---|
| main | 33 | 2.18 J/kg | 6.45 J/kg |
| remap alone | 28 | 2.43 | 6.18 |
| sideways contacts unscaled (|n·g| < 0.7) | 18 | 1.07 | 7.74 |
| gate + remap | 5 | 0.46 | 0.79 |
| + struck carriers, friction and CCD fixes | 0 | 0.20 | 0.48 |
| + driven strikes, 2 % cone tolerance | 0 | 0.20 | 0.39 |
| **+ drivers not carriers, frame-fixed lever arms** | **0** | **0.088** | **0.16** |

Seeds 500–1999, which the first three rows were not tuned on: gate + remap 18 of 1500
(99th percentile 0.52, worst 3.00); with struck carriers, friction and CCD 7 (0.25,
1.35); with driven strikes and 2 % tolerance 2 (0.20, 0.85); with drivers as load and
frame-fixed lever arms 1 (0.090, 0.53).
Those last two changes were made looking at seeds 120, 1742 and the arch, so the fresh
seeds are not fresh for them.

`physics_perf`, mean frame wall clock (conditioning stage), the same build with the gate
and remap switched off and on:

| | main | gate + remap |
|---|---|---|
| igloo blast, 88 bodies | 0.41 ms (0.009) | 0.52 ms (0.056) |
| 20-layer crate pyramid kept awake, 210 bodies | 9.9 ms (0.35) | 10.5 ms (0.70) |

At 2 % tolerance the certificate passes fewer bodies and more are solved: the pyramid's
conditioning went from 0.73 to 0.89 ms on a later run, the frame as a whole no slower.

The igloo's frame grows by more than its conditioning: the collapse it computes is a
different one, and the rest of the step follows it.

A naive version (SVD least squares, every body every frame) cost the pyramid 13.5 ms.

All `physics_scenarios` tests pass, including `mass_ratio_sweep` and the arch and tower
energy tests. `physics_fuzz::tests::a_honeycomb_wall_struck_low_by_a_steel_ball_regains_nothing`
is seed 495 replayed: 2.90 J/kg on main.
`a_honeycomb_wall_shoved_by_a_sprinting_walker_regains_nothing` is seed 253: 1.88 J/kg
without the struck-carrier rule. `a_jenga_tower_pushed_along_the_floor_regains_nothing`
is seed 1742, which fails with either driven strikes or the 2 % tolerance undone; it is a
sample, not a proof of either. `a_knocked_arch_regains_no_energy` fails at 5 % tolerance,
and `a_tower_sliding_along_the_floor_regains_no_energy` guards the slipping-carrier rule
below from coming back. `a_tall_jenga_tower_a_walker_leans_on_regains_nothing` is seed
636, 0.51 J/kg with drivers as carriers; `a_rolling_ball_does_not_speed_itself_up` doubles
its speed with lever arms measured from where bodies are now.

## The last five seeds

Each failed only with scaling on (`shock_alpha` 1 passed them all), but only one of the
causes was in the gate:

- **A carrier striking the body** (97, 200, and 253 once the others were fixed). The
  walker stands on the ground, so the wall block it runs into counts it as a carrier. Last
  frame's impulses held the block; this frame the walker drives it into the blocks above,
  which are scaled against it. Contacts inside the wall made 3.7 kJ in one frame. A carrier
  closing faster than `OPENING_SPEED` now ungrounds the body.
- **Friction that outlived its contact** (314, 424). A ball came off the tower spinning
  and landed hard. The next substep warm-started the landing's friction, the ball bounced
  clear, and `solve_friction_impulse` zeroed the unloaded contact's accumulator without
  taking back what it had applied: 3.5 kJ of spin in a frame. The row now takes it back,
  and so does the torsional row.
- **CCD treating a steep landing as a graze** (358). A 10 cm ball covering 21 cm a frame
  is CCD's, not the speculative band's, but it closed on the floor by 1.85 cm a substep,
  under the 2 cm margin `is_tunnelling_hit` asked for. It sank 11 cm before the next
  narrowphase. The approach is now asked of the frame, the margin over the frame's
  substeps.

Reading the fresh seeds and the arch test the struck-carrier rule broke turned up two
more:

- **Loads held only within 5 %.** A knocked arch left skewed out of its plane crept up,
  crown rising 2 cm/s for as long as it was watched, with the scales steady frame to frame.
  Unscaled from the start it settles and sleeps; at 2 % or 0.5 % tolerance it settles
  scaled. A load just outside the cone is a contact solution that pulls a little, every
  frame.
- **The player landing on a scaled tower** (120). Its contact had not met when the frame's
  scales were set, so the gate never saw it, and its impulse is in no warm start: the layers
  under it popped up 1 m/s. A driven body closing on a pair that has not met now ungrounds
  both. Only driven bodies: counting every closing pair fixed no more seeds and, in the
  build it was tried in, raised the first 500 seeds' 99th percentile from 0.13 to
  0.23 J/kg, by unscaling stacks blocks were merely falling onto.

## The two after them

- **The player leaning on a tower** (636). The walker stands on the ground, so it counted
  as a carrier of the block it leaned on, and held up by its drive rather than by its
  contacts it was never grounded itself: every layer above lost scaling, the eighteen
  layers sank 4 mm at each joint, and rising back out read as energy regained. A player
  leaning on a tall stack sank it. Driven bodies are no longer carriers. Counting their
  push as load let a block the player shoves along the floor pass as held, so a body
  touched by a driven body that is being driven (asking for travel or spin) is not
  grounded either: such a push is set by the drive, not by last frame.
- **A rolling ball speeding itself up** (1970, and 120 at 60 Hz). Not shock propagation.
  A contact's lever arm was its frame-start point less where the body had got to, so
  through a frame's substeps the point fell behind a rolling ball by up to a frame's
  travel, and the ground's push behind the centre spun it on: 0.33 to 1.11 m/s. Lever
  arms are now measured from where each body was when the frame's contacts were made
  (`LeverOrigins`), as position correction's always were; CCD, which makes contacts at
  bodies' current poses, measures from those.

The lever-arm fix took two tests down that had passed by its bug. A ball pressed against
a round pillar's upright edge got no contact once its centre was outside the pillar, and
the drifting lever arm had been nudging it free: a crease is cut into pieces, and each
left the joint between them to the other, so a centre level with a joint was found by
neither. The sphere's and capsule's crease tests now give a joint to the piece starting
there. And `breakable_fixed_joint_breaks_cleanly` had never seen its sphere break the
joint: the welded pair spins as one, which a 5 N·s limit carried, and the joint broke when
the pair hit the ground.

## Tried on the way

- **Remap alone**: scale changes on warm-started supports turned out rare (425 body-events
  in 275,760 fuzz frames); it fixes 5 seeds by itself.
- **Unscaling sideways contacts** (the old `horizontal_threshold` doc promised it; the code
  scaled every pair): 33 → 18 seeds, but the worst seed got worse. It is case 2 above, done
  crudely; the wrench cone covers it.
- **A separate friction-cone gate** on top of the wrench cone: redundant, and slightly worse.
- **Warm impulse as "loaded"**: a woken body has none, so a resting tower lost its scaling
  for a frame and sank 3.4 mm. Opening is a separating velocity, not a missing impulse.
- **Withheld shares as load.** Below a scaled contact the physical impulses carry the
  `(1−s)·λ` the scaled solve withheld, which in an arch leans further than the warm
  impulse. Passing those shares down top-down, as force at each lower carrier's contact
  points, is right by the theorem, but once the tolerance was 2 % it changed no measured
  result, so it is not in.
- **A slipping carrier ungrounds the body.** Meant for a tower pushed along the floor,
  whose bottom block slides. The withheld shares above it are weight, which its contact
  carries straight down however it slips; the rule only switched the tower's scales as it
  stopped, which regained 0.19 J/kg.
- **Driven bodies are not carriers**, on its own: over 2000 seeds it moved the 99th
  percentile by 0.01 and fixed no seed, until seed 636 showed what it is for; with it, a
  body a driver touches must be judged by whether the driver is being driven.
- **A slipping carrier under a driven push ungrounds the body**, for the block shoved
  along the floor: it fixed that and broke others; whether the driver is moving itself is
  the test that holds.

## Open

- Seed 1008 fails at 0.53 J/kg. Unread.
- The 3D statement is measured, not proven.
- The worst of the first 500 seeds is 0.16 J/kg; once 1008 is understood,
  `ENERGY_GAIN_LIMIT` can come down from 0.5.
