# Shock Propagation and Statics

Why shock propagation made energy, the exact statement of when it does, and what
`solver/shock_propagation/` does about it.

## Status

On branch `shock-statics` (2026-10-30). `StaticsGate` and `WarmStartRemap` run in
`ShockPropagationConditioner` after the contact graph. `physics_fuzz` over 500 seeds:
33 failing → 5, worst unexplained gain 6.45 → 0.79 J/kg.

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
in the graph. Its *load* is its weight over the substep plus every warm-started push from
higher bodies, as a force and a torque about its centre of mass. A body is **grounded**
when:

- it has carriers, none counting if it separates faster than `OPENING_SPEED` (5 cm/s);
- its load lies in the carriers' wrench cone (`ContactWrenchCone`: each contact's
  four-sided friction pyramid, as wrenches; membership by non-negative least squares to
  `CONE_TOLERANCE`, 5 %);
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
| **gate + remap** | **5** | **0.46** | **0.79** |

`physics_perf`, mean frame wall clock (conditioning stage), the same build with the gate
and remap switched off and on:

| | main | gate + remap |
|---|---|---|
| igloo blast, 88 bodies | 0.41 ms (0.009) | 0.52 ms (0.056) |
| 20-layer crate pyramid kept awake, 210 bodies | 9.9 ms (0.35) | 10.5 ms (0.70) |

The igloo's frame grows by more than its conditioning: the collapse it computes is a
different one, and the rest of the step follows it.

A naive version (SVD least squares, every body every frame) cost the pyramid 13.5 ms.

All `physics_scenarios` tests pass, including `mass_ratio_sweep` and the arch and tower
energy tests. `physics_fuzz::tests::a_honeycomb_wall_struck_low_by_a_steel_ball_regains_nothing`
is seed 495 replayed: 2.90 J/kg on main.

## Tried on the way

- **Remap alone**: scale changes on warm-started supports turned out rare (425 body-events
  in 275,760 fuzz frames); it fixes 5 seeds by itself.
- **Unscaling sideways contacts** (the old `horizontal_threshold` doc promised it; the code
  scaled every pair): 33 → 18 seeds, but the worst seed got worse. It is case 2 above, done
  crudely; the wrench cone covers it.
- **A separate friction-cone gate** on top of the wrench cone: redundant, and slightly worse.
- **Warm impulse as "loaded"**: a woken body has none, so a resting tower lost its scaling
  for a frame and sank 3.4 mm. Opening is a separating velocity, not a missing impulse.

## Open

- Five seeds still fail (97, 200, 314, 358, 424), all under 0.8 J/kg. Each should be checked
  for a body that only just passes the gate.
- The 3D statement is measured, not proven.
- Once the remaining seeds are understood, `ENERGY_GAIN_LIMIT` can come down from 0.5.
