# Box3D's Soft Step: A Comparison

Erin Catto's Box3D (June 2026) solves contacts with a soft step: soft normal
rows that push bodies apart, then a rigid pass that relaxes them, with no
shock propagation. This records what that solver does, why it looked like an
answer to our shock-propagation energy problem, and why it does not meet our
stacking requirements. It was designed as a second solver (2026-10-01) and
shelved before any code was written.

---

## 1. The problem it was meant to solve

Shock propagation scales the lower body's mass inside the velocity solve.
That is what lets a heavy body stand on light ones at three iterations. It is
also a source of momentum. Under the scaling, the ground holds a lower body
up with `m·g + α·J_above` instead of the full weight. In a collapse, the
shortfall comes out as energy. Every attempt to keep the benefit and lose
the cost failed (2026-10-01, `mass_ratio_sweep` and `physics_fuzz`):

| Attempt | Stacks | Collapses (fuzz, 500 seeds) |
|---|---|---|
| Today: scaled velocity solve, α 0.3 | 100:1 on 8 stands, sag 7 mm | 33 findings, worst 6.5 J/kg |
| No scaling (α 1.0) | 100:1 on 8 topples | 1 finding, worst 0.7 |
| Scaling only below a contact speed | as today | 5–16 findings; the arch and tower energy tests fail |
| Scaling only in the NGS position pass | 100:1 on 8 wobbles, creep 12 cm | 0 findings |
| No scaling, 24 iterations (8× cost) | 100:1 on 8 sags 21 mm, creeps 4 mm | not run |

The stacking benefit lives in the velocity solve, and so does the energy. A
solver that converges on heavy-on-light stacks without scaling masses would
remove the problem instead of trading against it. Box3D claims to stack
without any mass scaling.

## 2. What Box3D does

This section is read from `erincatto/box3d` (`src/solver.c`,
`src/contact_solver.c`, `src/solver.h`, `src/physics_world.c`, `src/types.c`).

- **One step with `n` sub-steps.** The default is `b3World_Step(world, 1/60, 4)`.
  Contacts are prepared once per step: anchors relative to each body, a base
  separation, and the effective masses. Every sub-step reuses them.
- **Each sub-step runs five stages:**
  1. *Integrate velocities* (gravity, damping).
  2. *Warm start* joints and contacts with the impulses accumulated so far.
  3. *Push*: one iteration. Joints get their bias. Contacts get **normal
     rows only**, soft and without friction.
  4. *Integrate positions.*
  5. *Relax*: one iteration. Joints run without bias. Contacts are rigid
     normal rows carrying only the speculative bias, followed by twist
     friction, rolling resistance and central manifold friction.
- **After the last sub-step:** two iterations of *restitution*. Then the
  impulses are stored for the next step's warm start.
- **The soft normal row (push pass).** The current separation comes from how
  far each body has moved and turned since the contact was prepared:
  `s = dot(Δp + dqB·rB − dqA·rA, n) + baseSeparation`.
  - **If `s > 0`** (speculative): `bias = s/h`, `massScale = 1`, `impulseScale = 0`.
  - **Otherwise:** `bias = max(massScale·biasRate·s, −contactSpeed)`, using
    the softness's `massScale` and `impulseScale`.
  - **Impulse:** `Δλ = −m_n(massScale·vn + bias) − impulseScale·λ`. It is
    clamped so the accumulated `λ ≥ 0`.
- **Softness** comes from `b3MakeSoft(hertz, ζ, h)`:
  - `ω = 2πf`, `a1 = 2ζ + hω`, `a2 = hωa1`, `a3 = 1/(1 + a2)`.
  - `biasRate = ω/a1`, `massScale = a2·a3`, `impulseScale = a3`.
  - All three are relative to the row's effective mass, so a contact is
    equally stiff whatever the masses.
- **Defaults:**
  - Contacts: `contactHertz` 30, `contactDampingRatio` 10, `contactSpeed` 3 m/s.
  - Contacts against static bodies use twice the hertz and half the damping ratio.
  - Hertz is clamped to `0.125 / h`, which is exactly 30 Hz at a 240 Hz sub-step.
  - Restitution: `restitutionThreshold` 1 m/s, `restitutionIterations` 2.
- **No shock propagation or mass scaling anywhere.**
- Graph colouring and the wide SIMD solver make the same algorithm faster.
  They have no bearing on the comparison.

## 3. Side by side

Our frame already has the same outer shape: contacts are generated once per
frame (`update_contacts`) and solved over 240 Hz sub-steps (`substep`). The
difference is what each sub-step does.

```text
ours, per substep                        Box3D, per substep
─────────────────                        ──────────────────
integrate forces, allowances             integrate velocities
prepare rows (fresh poses)               ─ (rows prepared once per step)
warm start (manifold cache)              warm start
velocity: 3+ PGS iterations, rigid,      push: 1 pass, soft normals only,
  shock-scaled, normal block + friction    true masses
NGS position pass, 3 iterations          ─
integrate positions, CCD                 integrate positions
─                                        relax: 1 pass, rigid normals +
                                           friction + twist + rolling
─                                        restitution (after the last substep)
```

## 4. Why it does not meet our requirements

**A soft contact holds its load by being compressed.** At rest, the warm
start supplies the contact's full impulse λ, and the velocity after it is
zero. For the push pass to leave λ unchanged, its terms must cancel:
`m_n·massScale·biasRate·s = impulseScale·λ`. With the softness above, that
reduces to

```text
s = λ / (m_n · h · ω²) = load / (m_eff · ω²)
```

The relax pass carries no bias for `s ≤ 0`, so it cannot move this fixed
point. Neither can more push iterations or more sub-steps, since the depth
does not depend on `h`.

- **Equal bodies:** `g/ω²` = 0.28 mm per contact at 30 Hz. Each contact
  further down carries more of the stack, so the depths add up.
- **Heavy on light:** the load is the heavy body's weight, but `m_eff` is
  close to the light body's mass. Each contact sinks by about the mass ratio
  times `g/ω²`: about 28 mm at 100:1.
- **Stiffening loaded contacts does not help.** Raising the hertz where the
  load is heavy brings back the lopsided Gauss-Seidel problem. The softness
  that makes a light body pass a heavy load on in one iteration is the
  softness that sags. In this scheme, convergence and sag are one trade-off.

A 1D column model of Box3D's update agrees with the algebra. The model ran
1 to 4 push iterations at 4 and 8 sub-steps, and the sag did not change
across them:

| Stack | 1D soft step (Box3D's exact bias) | Ours today | Required |
|---|---|---|---|
| 8 equal cubes | 16 mm | 1.2 mm | ≤ 2 mm |
| 100:1 cube on 3 | 147 mm | 6 mm | — |
| 100:1 cube on 8 | 439 mm | 7 mm | ≤ 10 mm |
| 10:1 cube on 8 | 58 mm | — | — |

The 3D solver adds angular terms to `m_eff`, which lowers it and deepens the
sag. Box3D's own demos stack bodies of similar mass, where centimetres of
sag in a tall stack go unnoticed. Our levels put heavy slabs and metal cubes
on light supports, and the player stands on stacks and reads their height.

### Smaller differences

- **Overlap comes out as velocity.** The push pass's bias resolves overlap
  by giving the bodies velocity, at up to `contactSpeed`. NGS also lifts
  bodies, but as a position change with no velocity. Under the soft step,
  `EnergyAudit` would see kinetic energy from every push-out, and the fuzz
  limits would need rethinking.
- **One reference pose.** Box3D measures `s` from the pose at which the
  contacts were prepared. Anchors and base separation must come from the
  same pose; mixing a frame-start pose with per-substep rows is wrong.
- **Restitution once per step** changes when bounces happen compared with
  our per-substep restitution.

## 5. What carries over

If a second solver is ever built, some of the scaffolding designed for this
one carries over:

- **A `relax` hook on `ConstraintSolver`**, called after position integration
  and CCD. Its default does nothing.
- **A solver choice** on `PhysicsConfig`, read by `PhysicsWorld::new`.
- **A `--solver` flag** on `physics_fuzz`, `physics_perf` and the game.
- **The ledgers** (`ContactWorkLedger`, `TractionLedger`, `ImpactLedger`)
  recording after the last pass that changes impulses.

The mass-ratio sweep is the first measure for any candidate solver. The sag
algebra above is a quick check worth doing before any build.

## 6. Where this leaves shock propagation

The energy comes from the scaling switching on and off, and from the warm
start carrying the old impulses across that switch.

**Ramping the scale per pair over time was tried and reverted (2026-10-01).**
Each pair's scale blended from 1 to `α^Δdepth` over N frames, with and
without the speed gate. A pair that changed which side was lower started
again from unscaled.

| Variant | Fuzz findings / worst | Arch / tower energy (limit 0.1 J/kg) | Nudged tower sinks | 100:1 on 8 |
|---|---|---|---|---|
| Today | 33 / 6.5 | pass | 0.5 mm | stands, sag 7 mm |
| Ramp 8 frames, no gate | 25 / 8.4 | 0.11 / 0.10 | 1.8 mm | stands |
| Ramp 30 frames, no gate | 10 / 6.4 | 0.28 / 0.12 | 2.8 mm | bounces, creep 48 mm |
| Gate 0.2 m/s, ramp 8 up | 12 / 6.8 | 0.31 / 0.20 | 1.8 mm | stands |
| Gate 0.2 m/s, ramp 30 up | 6 / 4.0 | 0.33 / 0.21 | 2.8 mm | stands |
| Gate 0.2 m/s, ramp 30 up / 8 down | 6 / 6.5 | 0.23 / 0.17 | 2.8 mm | ends 5 mm high |
| Gate 0.5 m/s, ramp 15 both ways | 14 / 3.8 | 0.21 / pass | 2.6 mm | creep 12 mm |

- **No variant passed the arch and tower energy tests.** Even the ungated
  ramp failed them, although it changes only pairs that are new or have
  changed sides. A slow transition makes as much energy as a sudden one:
  each frame's mismatch is smaller, but the ramp lasts longer.
- **Slow ramps put energy into resting stacks.** The heavy cube finished
  above where it was set.
- **A woken tower starts unscaled,** because its pairs had no state while
  asleep. It sinks before its scale ramps in.

Still open:

1. **Rescale the warm start at the transition.** The contact that needs it
   is the one below the pair that switched, so the fix is not local. The
   ramp result suggests that how much is carried across the switch matters,
   not how fast the switch happens, which is the part this would fix.
2. **Solve the normal rows directly along the support graph** (Baraff's
   linear-time solve on a tree). This is research. Contacts are
   inequalities, so it needs an active-set guess and a check afterwards.
   Each support edge is a 4×4 manifold block, which we already solve
   exactly. Jenga, arches and the temple have loops, so it does not apply
   to them.
