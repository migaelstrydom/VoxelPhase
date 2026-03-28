# Solver Constraint Refactor Plan

Move constraint expansion, warm-start caching, and position correction strategy inside the solver, where they belong.

---

## Problem

Solver-specific implementation details have leaked out of the `ConstraintSolver` trait and into shared types and the `PhysicsWorld` orchestration layer:

1. **Constraint expansion** (`expand_constraints`) runs in `PhysicsWorld`, converting `ConstraintKind` into `ConstraintRow`s — a PGS-specific format with scalar Jacobians, effective mass, and accumulated impulse. A different solver (XPBD, TGS, direct) would want a completely different internal representation.

2. **Baumgarte bias baked into rows.** The expansion step computes `bias = (beta/dt) * error` and embeds it in the row. This is a position correction strategy decision — it should be made by the solver, not by the expansion. The PgsNgsSolver uses NGS for contacts but gets Baumgarte for constraints because the bias arrives pre-baked. This causes energy injection that destabilises welded body structures.

3. **Warm-start cache on `Constraint`.** `Constraint::warm_impulses` stores "one per row the kind expands to" — the number, meaning, and existence of these values is PGS-specific. The persistent constraint definition should be solver-agnostic.

4. **`ConstraintRow` is a shared type.** It's defined in `src/physics/constraint/types.rs` and passed through the `ConstraintSolver` trait. It should be an internal detail of PGS-style solvers.

5. **Write-back and projection in `PhysicsWorld`.** `write_back_constraints()` and `project_angular_velocities()` are called directly from `PhysicsWorld::substep()`, outside the solver's control.

---

## Goal

A solver receives constraint definitions and body state, and owns the entire pipeline from there: expansion, warm-start caching, position correction, projection. `PhysicsWorld` passes constraints through but doesn't interpret them. Plugging in a new solver requires no changes outside the solver module.

---

## Design

### New `ConstraintSolver` trait

```rust
pub trait ConstraintSolver {
    /// Called once per frame after contact generation, before substeps.
    /// The solver can snapshot body state, pre-process constraints, etc.
    fn prepare(
        &mut self,
        bodies: &Arena<RigidBody>,
        constraints: &Arena<Constraint>,
    );

    /// Solve all constraints for one substep.
    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        constraints: &Arena<Constraint>,
        dt: f32,
    );

    /// Write solver state back to persistent constraints after each substep.
    /// The solver owns what gets cached (impulses, factorizations, nothing).
    fn write_back(&self, constraints: &mut Arena<Constraint>);

    /// Post-solve velocity projection (e.g., KeepUpright hard projection).
    /// Called after write_back, before position integration.
    fn project_velocities(
        &self,
        constraints: &Arena<Constraint>,
        bodies: &mut Arena<RigidBody>,
        dt: f32,
    );
}
```

Key changes:
- `solve` receives `&Arena<Constraint>` instead of `&mut [ConstraintRow]`.
- `write_back` replaces the free function `write_back_constraints`.
- `project_velocities` replaces the free function `project_angular_velocities`.
- `prepare` receives constraints for pre-processing.

### Simplified `Constraint`

```rust
pub struct Constraint {
    pub kind: ConstraintKind,
    pub active: bool,
    /// Opaque solver cache. The solver writes whatever it needs here
    /// (e.g., warm-start impulses for PGS, nothing for XPBD).
    /// Cleared when the constraint is created; the solver owns the format.
    pub solver_cache: SmallVec<[f32; 6]>,
}
```

The field is renamed from `warm_impulses` to `solver_cache` to reflect that its contents are solver-defined. The solver reads/writes it via `write_back` / `prepare`. `PhysicsWorld` never interprets it.

An alternative is to make it fully opaque (`Box<dyn Any>` or a trait), but `SmallVec<[f32; 6]>` is pragmatic — it covers PGS warm-start without allocation, and other solvers can use the same storage or ignore it.

### `ConstraintRow` becomes solver-private

`ConstraintRow` moves from `src/physics/constraint/types.rs` to `src/physics/solver/constraint_row.rs` (where it's already partially defined). It is no longer part of the public constraint API or the `ConstraintSolver` trait.

### Expansion moves into the solver

`expand_constraints` and the per-kind expansion functions (`weld::expand`, `follow_point::expand`, `keep_upright::expand`) move into the solver module. They become internal helpers of `PgsNgsSolver`. The expansion functions gain a `position_correction: bool` parameter (or similar) so the solver can control whether Baumgarte bias is included — PgsNgsSolver sets it to `false` and applies NGS instead.

Alternatively, the expansion functions stay where they are but produce rows with zero position-correction bias (raw error only), and the solver adds bias if it wants Baumgarte. This keeps the Jacobian math centralized for reuse by multiple solvers. **This is the preferred approach** — the Jacobian math (anchor computation, angular error extraction) is constraint-specific, not solver-specific. What's solver-specific is only the position correction strategy.

### NGS position correction for constraints

`PgsNgsSolver` gets constraint position correction integrated into the existing NGS loop in `apply_position_correction`. For weld constraints, this computes the current anchor separation and angular error from live body positions, then applies direct position/rotation corrections — no velocity injection.

Constraint corrections must be **interleaved with contact corrections** within the same NGS iteration loop, not run as a separate pass. If they run sequentially, they fight: contact NGS fixes a penetration, then constraint NGS moves the body to fix anchor drift, re-creating the penetration. By interleaving within each NGS iteration, both correction types see each other's adjustments and converge together.

The loop structure becomes:

```rust
for _ in 0..ngs_iterations {
    // Contact position corrections
    for manifold in manifolds {
        correct_contact_penetration(bodies, manifold, ...);
    }
    // Constraint position corrections
    for constraint in constraints {
        correct_constraint_drift(bodies, constraint, ...);
    }
}
```

Each iteration partially corrects both contact penetrations and constraint anchor errors. Over multiple iterations, the corrections converge to a state that satisfies both.

The constraint correction function (`correct_constraint_drift`) is specific to each constraint kind:
- **Weld**: read live anchor positions from body transforms, compute separation, apply linear + angular correction proportional to the error.
- **FollowPoint**: same as weld but may use compliance to soften the correction.
- **KeepUpright**: no NGS needed — handled by hard velocity projection, which is more effective for orientation constraints.

### Projection moves into the solver

`project_angular_velocities` becomes the implementation of `ConstraintSolver::project_velocities` for `PgsNgsSolver`. The function itself doesn't change; it just moves behind the trait.

---

## Phased approach

| Phase | What | Why |
|-------|------|-----|
| 1 | Change `ConstraintSolver` trait signature | Establishes the new contract. All downstream changes follow from this. |
| 2 | Move expansion + warm-start + write-back + projection inside `PgsNgsSolver`. Move `constraint_position_beta` from `PhysicsConfig` to `PgsNgsConfig`. | Eliminates the leaked abstractions from `PhysicsWorld`. `ConstraintRow` becomes solver-private. All solver parameters live on the solver. |
| 3 | Remove Baumgarte bias from constraint expansion, add interleaved NGS for constraints | Fixes the energy injection bug. Weld constraints use the same position correction strategy as contacts, interleaved in the same NGS loop. |
| 4 | Clean up `Constraint` struct | Rename `warm_impulses` → `solver_cache`, update doc comments. |
| 5 | Remove dead Baumgarte code path from `position_correction.rs` | The `apply_baumgarte_correction` branch and its config fields. Separate cleanup, not blocking. |

Each phase should compile and pass tests before proceeding. Phase 3 is what fixes the barricade bug.

---

## PhysicsWorld changes

After the refactor, `PhysicsWorld::substep` becomes:

```rust
fn substep(&mut self, dt: f32, ...) {
    // ... force integration ...

    self.solver.solve(
        &mut self.bodies,
        &mut self.cached_active_manifolds,
        &self.manifold_conditions,
        &self.constraints,  // was: &mut self.cached_constraint_rows
        dt,
    );

    self.solver.write_back(&mut self.constraints);

    self.solver.project_velocities(
        &self.constraints,
        &mut self.bodies,
        dt,
    );

    // ... position integration, CCD, sleep ...
}
```

And `update_contacts` no longer calls `expand_constraints`:

```rust
fn update_contacts(&mut self, dt: f32, ...) {
    // ... narrowphase, manifold cache, conditioning ...

    // No more expand_constraints() here.

    self.solver.prepare(&self.bodies, &self.constraints);
}
```

The `cached_constraint_rows` field is removed from `PhysicsWorld`. The `constraint_position_beta` config field moves into `PgsNgsConfig` (it's a solver parameter, not a world parameter).

---

## What stays in `src/physics/constraint/`

- `types.rs` — `ConstraintKind`, `Constraint` (simplified), `ConstraintHandle`.
- Per-kind modules (`weld.rs`, `follow_point.rs`, `keep_upright.rs`) — Jacobian math and geometric error computation, but **not** position correction strategy. These become helper functions that the solver calls, not a pipeline stage.
- `expand.rs` — may be absorbed into the solver or kept as a shared utility that produces rows without bias.
- `projection.rs` — moves behind the solver trait (called from `project_velocities`).
- `welded_bodies.rs` — **removed** (see cleanup section).

---

## Performance impact and mitigations

### Constraint expansion (regression risk: none)

Constraint rows (Jacobians, effective mass, warm-start) are expanded once per frame in `prepare`, same as today. They are reused across substeps — the velocity-phase PGS loop runs on the same rows each substep. This matches how contacts work: `SolverManifold`s are generated once and reused.

Fresh position errors for NGS are computed directly from live body transforms in the NGS pass, not by re-expanding rows. This is the same pattern as contact NGS, which reads current body positions and recomputes penetration depth on the fly without regenerating contact solver data.

The per-substep re-expansion added earlier in this session (the stale Baumgarte bias workaround) is removed — it was compensating for a problem that goes away entirely when Baumgarte is replaced by NGS.

**Work buffer reuse:** The solver owns a `Vec<ConstraintRow>` that is cleared-and-refilled once per frame in `prepare` (same pattern as `NarrowphaseWorkBuffer` and the current `cached_constraint_rows`). The `Vec` lives on `PgsNgsSolver` and persists across frames.

### Warm-start cache on `Constraint` (regression risk: low)

The current `SmallVec<[f32; 6]>` avoids heap allocation for constraints with up to 6 rows (which covers KeepUpright, Weld, and FollowPoint). Renaming it to `solver_cache` doesn't change the storage — same `SmallVec`, same inline threshold, same zero-allocation behaviour for all current constraint types.

If a future solver needs a different cache format, `SmallVec<[f32; 6]>` still works as a byte bag (6 floats = 24 bytes, enough for most compact representations). Only if a solver needs significantly more state would a heap allocation be necessary, and that would be a per-constraint one-time cost on creation, not a per-frame cost.

### NGS constraint position correction (new cost)

This is a new per-substep pass that iterates over active weld/follow-point constraints and applies direct position corrections. The cost is one body-pair position read + correction write per constraint per NGS iteration (typically 3 iterations).

**Mitigation:** This replaces the Baumgarte bias that was already being computed and applied in the PGS velocity loop. The total work is comparable — we're trading velocity-bias application for position correction. The NGS pass is also simpler (no effective mass computation, no accumulated impulse clamping) so the per-iteration cost is lower than a PGS row solve.

### Trait method overhead (regression risk: negligible)

The refactor adds `write_back` and `project_velocities` as separate trait method calls per substep. These are non-generic dynamic dispatch calls — one vtable lookup each per substep. At 240Hz × 4 substeps, that's ~960 vtable lookups/second, which is completely noise.

### Memory layout

`ConstraintRow` stays as a flat struct of scalars and `Vector3`s — cache-friendly sequential iteration in the PGS loop. Moving it inside the solver module doesn't change its memory layout or access pattern, just its visibility.

The solver's internal `Vec<ConstraintRow>` replaces `PhysicsWorld::cached_constraint_rows` — same allocation, same lifetime, same reuse pattern, just a different owner.

### Summary

| Change | Cost delta | Mitigation |
|--------|-----------|------------|
| Constraint expansion | Unchanged (once per frame) | Same work, same `Vec` reuse, different owner |
| Warm-start storage | Zero | Same `SmallVec<[f32; 6]>`, renamed |
| NGS constraint pass | New pass, ~= Baumgarte it replaces | Simpler per-iteration; fewer total iterations needed for convergence |
| Trait dispatch | +2 vtable calls per substep | Negligible at ~960/sec |
| Memory layout | Unchanged | Same struct, same `Vec`, different owner |

No regressions expected. The `box_grid_narrowphase_throughput` bench harness test (the most performance-sensitive test in the suite) should be unaffected since it tests contact throughput, not constraint solving.

---

## Testing strategy

- All existing bench harness tests must pass after each phase.
- The `barricade_drop_diagnostic` test (currently failing with filtering disabled) must pass after Phase 3.
- New test: weld constraint with `angular_compliance > 0` (soft weld) on ground, verify no energy injection over 3 seconds.

---

## Cleanup from investigation session

During the debugging session that produced this plan, several exploratory changes were made. These must be cleaned up before or during Phase 1:

| Change | Location | Action |
|--------|----------|--------|
| `WeldedBodies` struct | `src/physics/constraint/welded_bodies.rs` | **Remove entirely.** Contact filtering between welded bodies is unnecessary once NGS eliminates the energy injection. For rigid pre-authored structures (barricade, table), the correct architecture is Phase 4 of the destructible bodies plan (compound body until impact). Weld constraints are a transient post-fracture state where inter-body contacts are physically correct. |
| `WeldedBodies` field on `PhysicsWorld` | `src/physics/world.rs` | **Remove.** Also remove the `add_weld`/`rebuild` calls in `create_constraint`, `remove_constraint`, and `remove_body`. |
| `welded_bodies` parameter on `generate_dynamic_contacts` | `src/physics/narrowphase/dynamic_contacts.rs` | **Remove parameter** and the (currently commented-out) same-group check in the pair dispatch loop. |
| `WeldedBodies` re-export | `src/physics/constraint/mod.rs` | **Remove.** |
| Per-substep re-expansion | `src/physics/world.rs` `substep()` | **Remove** the `expand_constraints` call added inside `substep()`. It was a workaround for stale Baumgarte bias, which goes away entirely with NGS. Expansion returns to once per frame (and eventually moves into `prepare`). |
| Barricade `weld_gap` removal | `src/app/spawners/barricade.rs` | **Keep.** Planks fitting flush against posts is correct regardless of contact filtering. The weld anchors now coincide (zero initial error), which is better for solver convergence. |
| `barricade_drop_diagnostic` test | `src/physics/bench_harness/tests/constraint.rs` | **Keep.** Update the assertion after Phase 3 — it should pass once NGS replaces Baumgarte. This test directly reproduces the energy injection bug with contacts between welded bodies active (no filtering). |
| `welded_boxes_with_touching_faces_do_not_explode` test | `src/physics/bench_harness/tests/constraint.rs` | **Keep.** Validates that touching welded boxes don't explode. Will pass via NGS stability rather than contact filtering. |

---

## Constraint NGS position correction: math

The constraint NGS pass mirrors the contact NGS pass — it reads live body transforms and applies direct position/rotation corrections without injecting velocity. This section specifies the math for each constraint kind.

### Weld: linear correction

Given body A and body B with current positions and rotations, compute the world-space anchor points:

```
anchor_a = pos_a + rot_a * local_anchor_a
anchor_b = pos_b + rot_b * local_anchor_b
error = anchor_b - anchor_a
```

Compute the correction using the same effective-mass formula as contacts, but for two dynamic bodies:

```
inv_mass_sum = inv_mass_a + inv_mass_b
correction = (correction_factor / ngs_iterations) * error
delta_a = +correction * (inv_mass_a / inv_mass_sum)
delta_b = -correction * (inv_mass_b / inv_mass_sum)
```

Apply `delta_a` and `delta_b` directly to body positions. The `correction_factor` and iteration count match the contact NGS config. The mass-weighted split ensures lighter bodies move more, heavier bodies move less — same principle as contact NGS.

For zero-mass (static/kinematic) bodies, that body's delta is zero and the other body absorbs the full correction.

### Weld: angular correction

Compute the angular error as the rotation from the target orientation to the current orientation:

```
target_b_rot = rot_a * relative_orientation
error_q = rot_b * target_b_rot.inverse()
angular_error = error_q.xyz * 2.0 * sign(error_q.w)
```

This is the same axis-angle extraction already used in `weld::expand`. Apply a fraction of the correction to each body's rotation:

```
correction = (correction_factor / ngs_iterations) * angular_error
inv_inertia_sum = scalar effective inverse inertia along the error axis
rot_a = integrate_orientation(rot_a, +correction * share_a, 1.0)
rot_b = integrate_orientation(rot_b, -correction * share_b, 1.0)
```

Where `share_a` and `share_b` are the mass-weighted shares based on the world-space inverse inertia projected onto the correction axis. `integrate_orientation` is the existing function in `src/physics/math.rs` that applies an angular velocity delta to a quaternion.

### FollowPoint

Same math as weld, but the correction is scaled by compliance:

```
effective_correction = correction_factor / (1.0 + compliance * ngs_iterations)
```

With `compliance = 0`, this reduces to the weld case. With `compliance > 0`, the correction is softened — the bodies don't fully converge to the target, allowing controlled flex.

### KeepUpright

No NGS correction. KeepUpright uses hard velocity projection (`project_angular_velocities`), which directly strips forbidden angular velocity components. This is more effective than NGS for orientation-only constraints because the correction is exact regardless of iteration count.

---

## Interaction with other work

- **Destructible bodies Phase 3 (breakable welds)**: unaffected. Break detection checks accumulated impulse, which the solver writes to `solver_cache`.
- **Destructible bodies Phase 4 (compound-to-weld transition)**: this is the correct architecture for pre-authored destructible structures. Compound body until impact, then transition to separate welded bodies. The weld constraint becomes a transient post-fracture state.
- **Baumgarte removal** (Phase 5 of this plan): independent cleanup. Can be done anytime after Phase 3 confirms NGS works for constraints.

---

## Implementation notes

Phases 1, 2, 3, and 5 were implemented. Phase 4 (`warm_impulses` → `solver_cache` rename) was skipped — the current name is clear enough.

### What worked

- **Solver trait refactor (Phases 1+2):** Clean separation. `PhysicsWorld` no longer interprets constraints — the solver owns expansion, write-back, and projection. `ConstraintRow` is no longer part of the public API.
- **Baumgarte removal from FollowPoint:** NGS linear-only position correction is more stable than Baumgarte for FollowPoint. The old Baumgarte bias injected energy that caused oscillation in the grab system. NGS corrects position drift without affecting velocity, which is a better fit for soft positional constraints.
- **Baumgarte fallback removal (Phase 5):** NGS is strictly better than Baumgarte for contact position correction. The fallback code path was dead weight.
- **WeldedBodies removal:** The contact filtering struct was an exploratory workaround, not a solution. Removed cleanly.

### What didn't work

- **Angular NGS correction:** Applying angular corrections in the NGS pass (for both contacts and constraints) causes oscillation and energy growth. All NGS corrections are linear-only. Angular drift in constraints is handled by the PGS velocity rows and hard projection (KeepUpright).
- **Weld velocity projection:** Hard-projecting relative velocity between welded bodies after the PGS solve conflicted with contact impulses, creating a feedback loop. Reverted.
- **Aggressive constraint correction factors:** Higher `constraint_correction_factor` values (0.8+) caused oscillation between constraint and contact corrections. The factor should match the contact correction factor (0.2).

### Barricade / weld constraint stability

The plan assumed NGS would eliminate energy injection in welded structures, making contact filtering unnecessary. This turned out to be wrong — the instability is not caused by Baumgarte energy injection but by **bogus contacts between overlapping welded bodies**. The narrowphase sees two OBBs whose surfaces are touching and generates separation forces. The PGS solver converges to a state where both contacts and welds are satisfied, but the contact normals inject momentum into the structure as a whole. Increasing PGS iterations to 16 did not help — the solver converges to the wrong answer, not slowly to the right one.

The correct fix is **compound colliders** — one body with multiple colliders, so inter-body contacts don't exist. Contact filtering between welded bodies is functionally equivalent to a compound body and doesn't justify a separate mechanism. The upcoming destructible bodies work (compound-to-separate transition on impact) covers the barricade use case without needing rigid weld constraints at all.

Rigid weld constraints (compliance=0) don't have a practical use case once compound colliders + destructible breakage exist. Soft welds (compliance>0) could still be useful for flexible joints where the bodies aren't flush against each other.
