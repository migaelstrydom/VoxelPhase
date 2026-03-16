# Constraint System Plan (v3)

Design document for the generalized constraint system in the physics engine. This version documents the dual-mechanism architecture discovered during the KeepUpright implementation and provides guidance for future constraint types.

---

## Status

KeepUpright constraint implemented and working. The implementation revealed fundamental limitations of PGS-only constraint enforcement that led to a dual-mechanism design (PGS rows + post-solve projection). This document captures those lessons.

---

## Problem statement

The player character uses a capsule collider that falls over under contact forces. There is no mechanism to constrain a body's degrees of freedom — orientation locking, joint limits, or driven motion in constraint-space.

A per-body angular factor hack (scaling rows of the inverse inertia tensor) would solve the immediate problem but doesn't generalize. A proper constraint system participates in the solver loop alongside contacts, giving correct force interactions.

---

## Design principles

1. **Constraints are first-class solver citizens.** They are iterated in the same PGS loop as contacts, so constraints and contacts see each other's corrections within each iteration. This is critical for correct contact impulse computation.
2. **Hard constraints need post-solve projection.** PGS rows alone are insufficient for hard constraints (compliance=0) due to limited solver iterations and Jacobian degeneracy at large angles. A post-solve projection step provides exact enforcement.
3. **Pluggable, like everything else.** The constraint system adds a new data channel (constraint rows) through the existing solver trait, not a parallel solver. The solver, conditioner, stepper, and CCD traits remain independently swappable.
4. **Enum, not trait.** Constraint types are an enum to keep the solver loop monomorphic. New constraint types are added as variants.
5. **Zero per-frame allocation.** All work buffers are pre-allocated and reused.

---

## The dual-mechanism architecture

The constraint system uses two complementary enforcement mechanisms. This emerged from debugging the KeepUpright constraint and represents a load-bearing design decision.

### Why PGS rows alone don't work

Three problems converge to make PGS-only enforcement unreliable for hard constraints:

1. **Limited iterations.** The solver runs only 3 PGS iterations. With constraints solved alongside contacts (friction), 3 iterations aren't enough for the constraint and friction to converge — whichever is solved last "wins," leaving residual error from the other.

2. **Jacobian degeneracy at large angles.** The PGS constraint rows use linearized Jacobians (dot-product error measure). At small tilt angles, `sin(θ) ≈ θ` and the correction is approximately correct. At 90° tilt, the correction angular velocity becomes parallel to the tilt axis, producing zero corrective torque. The constraint becomes mathematically impotent at exactly the angles where correction is most needed.

3. **Stale expansion across substeps.** Constraint rows are expanded once per frame (in `update_contacts()`) but reused across all substeps. The bias term is frozen at the initial error. While contacts have the same staleness issue, their position correction (NGS) re-evaluates penetration each substep. Constraint rows don't have equivalent per-substep re-evaluation.

### Why post-solve projection alone doesn't work

Projection after the solver removes forbidden angular velocity and injects corrective velocity. But without PGS rows, the contact solver doesn't know the body is constrained:

- Friction computes impulses assuming the body will absorb torque as rotation
- The projection then prevents that rotation
- The energy the solver "budgeted" for rotation has nowhere to go
- This manifests as parasitic linear velocity (observed as upward drift when pushing blocks)

The PGS rows act as a **signal to the contact solver** that the body resists rotation, causing it to compute contact impulses that are consistent with the constrained state.

### The working architecture

Both mechanisms, working together:

```
PGS Solver (Phase 3):
  for each iteration:
    1. Contact normals + friction    ← contacts solved first
    2. Constraint rows               ← constraints solved last (higher priority)

Post-solve projection:
  3. For each hard constraint:
       Strip forbidden angular velocity components
       Inject corrective angular velocity using cross-product (no linearization)
```

- **PGS rows** tell the contact solver "this body resists tilt," producing correct contact impulses
- **Projection** provides exact enforcement regardless of iteration count or tilt angle
- **Solve order** (contacts first, constraints last) gives constraints higher effective priority in PGS

### The stale warm-start trade-off

The PGS rows produce warm-start impulses that are written back before the projection modifies angular velocity. This means warm-start reflects the PGS approximation, not the post-projection state. On the next substep, warm-start creates angular velocity the projection must correct.

This is accepted as a minor inefficiency, not a correctness problem:
- The projection runs every substep and overrides regardless
- The PGS rows still serve their primary purpose (coupling with contacts)
- Removing the PGS rows to avoid stale warm-start causes the contact coupling problem described above

---

## Constraint representation

Each constraint maps a pair of body velocities to a scalar constraint-space velocity, and the solver drives that scalar toward zero (or toward a target, for motors).

```
                ┌──────────────────────────────┐
                │        Constraint            │
                │                              │
   body_a ─────►  Jacobian J maps body vels    │
                │  to scalar constraint vel    ├──► scalar impulse λ
   body_b ─────►                               │     (clamped to bounds)
                │  effective_mass = 1/(J·M⁻¹·Jᵀ)│
                └──────────────────────────────┘
```

### ConstraintKind enum

Each variant is a user-facing constraint definition. The enum carries the parameters needed to expand into solver-ready rows.

```rust
pub enum ConstraintKind {
    /// Keep a body's world-space up-axis aligned with a target direction.
    /// 2 constraint rows (tilt around two horizontal axes).
    KeepUpright {
        body: RigidBodyHandle,
        target_up: UnitVector3<f32>,
        compliance: f32,  // 0 = hard (uses projection), >0 = soft (PGS only)
    },

    /// Fixed distance between two anchor points.
    /// 1 constraint row.
    Distance {
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        anchor_a: Point3<f32>,  // local-space
        anchor_b: Point3<f32>,  // local-space
        target_distance: f32,
        compliance: f32,
    },

    /// Hinge (revolute) joint — bodies share an anchor point, rotation
    /// constrained to one axis.
    /// 5 constraint rows (3 positional + 2 angular).
    Hinge { /* ... */ },

    // Future: BallJoint, Prismatic, Motor, etc.
}
```

### Constraint (stored definition)

The persistent object stored in the arena. Holds the kind plus warm-start state.

```rust
pub struct Constraint {
    pub kind: ConstraintKind,
    pub warm_impulses: SmallVec<[f32; 6]>,
    pub active: bool,
}
```

### ConstraintRow (solver-ready form)

Before the solve loop, each `Constraint` is expanded into one or more `ConstraintRow`s — the flat, uniform struct the solver actually iterates:

```rust
pub struct ConstraintRow {
    pub body_a: Option<RigidBodyHandle>,
    pub body_b: Option<RigidBodyHandle>,
    pub lin_jac_a: Vector3<f32>,
    pub ang_jac_a: Vector3<f32>,
    pub lin_jac_b: Vector3<f32>,
    pub ang_jac_b: Vector3<f32>,
    pub effective_mass_inv: f32,
    pub bias: f32,
    pub accumulated_impulse: f32,
    pub bounds: (f32, f32),
    pub constraint_index: usize,
    pub row_index: usize,
}
```

### ConstraintHandle

Follows the same pattern as `RigidBodyHandle` and `ColliderHandle` — a newtype around a generational arena index.

---

## Architecture integration

### Pipeline with constraints

```
PhysicsWorld orchestration (called by Stepper):

  1. update_contacts()
     ├── narrowphase contact generation
     ├── manifold cache merge (warm-start population)
     ├── conditioner.condition()  → reorder manifolds, compute shock scales
     ├── expand_constraints()     → constraints → ConstraintRow buffer
     └── solver.prepare()

  2. substep() × N
     ├── integrate_forces (gravity → velocities)
     ├── solver.solve(manifolds, conditions, constraint_rows, dt)
     ├── manifold_cache.write_back()
     ├── constraint_write_back()  → rows → Constraint::warm_impulses
     ├── project_angular_velocities()  ← hard constraint enforcement
     ├── integrate_bodies (velocities → positions)
     ├── CCD pass
     └── sleep state update
```

The projection runs after the PGS solver and write-back, but before position integration. This ensures the angular velocity that gets integrated into rotation is exactly what the constraint allows.

### ConstraintSolver trait (renamed from ContactSolver)

```rust
pub trait ConstraintSolver {
    fn prepare(&mut self, bodies: &Arena<RigidBody>);
    fn solve(
        &mut self,
        bodies: &mut Arena<RigidBody>,
        manifolds: &mut [SolverManifold],
        conditions: &ManifoldConditions,
        constraint_rows: &mut [ConstraintRow],
        dt: f32,
    );
}
```

### PhysicsWorld changes

```rust
pub struct PhysicsWorld {
    // ... existing fields ...
    constraints: Arena<Constraint>,
    cached_constraint_rows: Vec<ConstraintRow>,
    solver: Box<dyn ConstraintSolver + Send + Sync>,
}

impl PhysicsWorld {
    pub fn create_constraint(&mut self, kind: ConstraintKind) -> ConstraintHandle { ... }
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) { ... }
    pub fn constraint(&self, handle: ConstraintHandle) -> &Constraint { ... }
    pub fn constraint_mut(&mut self, handle: ConstraintHandle) -> &mut Constraint { ... }
}
```

### Body/constraint lifetime rules

`remove_body()` iterates the constraint arena and removes any constraint referencing the body. O(N) over the arena, acceptable since body removal is infrequent.

### Stepper, ManifoldConditioner, CCD — unchanged

These components are unaffected by the constraint system.

---

## PgsNgsSolver integration

### Phase 1: Pre-solve capture

No work needed for constraint rows. Warm-start impulses are already on the rows from expansion.

### Phase 2: Warm-start

Constraint rows warm-start after contacts:

```
for row in constraint_rows {
    warm_start_constraint_row(bodies, row, warm_start_scale);
}
```

### Phase 3: Iterative sequential-impulse

Contacts first, constraints last within each iteration:

```
for iteration in 0..solver_iterations {
    // Contacts first
    for manifold in manifolds {
        solve_normal_impulses(bodies, manifold, shock);
        solve_friction_impulses(bodies, manifold, shock);
    }
    // Constraints last — higher priority, gets last word in PGS
    for row in constraint_rows {
        solve_constraint_row(bodies, row);
    }
}
```

Constraint rows use **real masses** — no shock propagation scaling.

### Phase 4: Position correction (NGS)

Currently uses stale bias from expansion. Constraint rows don't participate in the NGS phase beyond the velocity bias already on the row. For hard constraints, the projection handles position correction via the cross-product formulation.

### Shock propagation interaction

Joint constraints use real masses, not shock-scaled masses. The keep-upright constraint is world-anchored (body_b = None), so there's no second body to shock-scale.

---

## Post-solve projection

### The projection step

After the PGS solver completes each substep, `project_angular_velocities()` enforces hard constraints exactly:

```rust
pub fn project_angular_velocities(
    constraints: &Arena<Constraint>,
    bodies: &mut Arena<RigidBody>,
    dt: f32,
    beta: f32,
) {
    for constraint in constraints {
        match constraint.kind {
            KeepUpright { body, target_up, compliance: 0.0 } => {
                let local_up = body.rotation() * Vector3::y();
                let up = target_up.into_inner();

                // Preserve only spin around target axis
                let spin = omega.dot(&up) * up;

                // Corrective velocity using cross product — works at all angles.
                // |local_up × target_up| = sin(θ), giving the correct rotation
                // axis and magnitude without linearization.
                let correction = local_up.cross(&up) * (beta / dt);

                body.set_angular_velocity(spin + correction);
            }
            // Soft constraints: no projection, PGS rows handle it
            _ => {}
        }
    }
}
```

### Why the cross product works at all angles

The PGS Jacobian uses `local_up · perp` (dot product) to measure error. At 90° tilt, this produces a correction angular velocity parallel to the tilt axis — zero corrective torque.

The projection uses `local_up × target_up` (cross product) instead. This gives:
- The correct rotation axis (perpendicular to both vectors)
- Magnitude `sin(θ)` — correct at all angles, maximum at 90°
- Smooth falloff to zero as the body approaches upright

This is the same geometric operation as "rotate vector A toward vector B" — it's not an approximation.

---

## Keep-upright constraint (implemented)

### How it works

The KeepUpright constraint uses both mechanisms:

1. **PGS rows** (2 angular rows, one per tilt axis perpendicular to `target_up`):
   - Signal to the contact solver that the body resists rotation
   - Ensure friction and normal impulses are computed correctly
   - Handle the soft constraint case (compliance > 0)

2. **Post-solve projection** (hard constraints only, compliance = 0):
   - Strips all angular velocity except spin around target axis
   - Injects corrective angular velocity via cross product
   - Runs every substep, before position integration

### Row expansion

```rust
fn expand_keep_upright(body, target_up, compliance, dt) -> [ConstraintRow; 2] {
    let local_up = body.rotation * Vector3::y();
    let (perp1, perp2) = perpendicular_basis(target_up);

    let error_1 = local_up.dot(&perp1);
    let error_2 = local_up.dot(&perp2);

    let inv_inertia_ws = body.world_inverse_inertia();
    let eff_mass_1 = 1.0 / ((inv_inertia_ws * perp1).dot(&perp1) + compliance / (dt * dt));
    let bias_1 = -(beta / dt) * error_1;

    // ... two rows with angular-only Jacobians, unbounded impulse ...
}
```

### Known limitations

- **Capsule sinking:** the capsule slowly sinks into terrain (y drifts from 0.75 to ~0.52 over seconds). This is a pre-existing capsule-static narrowphase issue, not a constraint problem.
- **Stale warm-start:** PGS warm-start values are computed before projection, so they don't perfectly reflect the post-projection state. Minor inefficiency.

---

## Guidance for future constraint types

### When you need projection

A constraint type needs post-solve projection when ALL of these apply:

1. **It fights high-frequency contact forces** — friction torques, collision impulses, or other solver-generated forces that create the DOFs the constraint forbids.
2. **It must be hard** — compliance=0, the constraint cannot be violated even briefly.
3. **The linearized Jacobian degrades at large errors** — e.g., angular constraints where the dot-product error measure fails at 90°.

### When PGS rows alone suffice

- **Soft constraints** (compliance > 0) — the solver naturally handles spring-like behavior.
- **Two-body positional constraints** (distance, ball-and-socket) — these don't fight friction torques in the same way. PGS convergence is typically adequate.
- **Constraints between dynamic bodies only** — less solver conflict than body-vs-world constraints that override friction from static geometry.

### Assessment by future type

| Type | PGS rows | Projection needed? | Difficulty | Notes |
|------|----------|--------------------|------------|-------|
| **Distance** | 1 row (linear) | Unlikely | Low | Two-body, linear Jacobian doesn't degenerate. Soft compliance natural. |
| **Ball-and-socket** | 3 rows (positional) | Unlikely | Low | Anchor point matching. Similar to distance but 3 DOF. |
| **Hinge** | 5 rows (3 pos + 2 angular) | Maybe for angular | Medium | The 2 angular rows restrict rotation to one axis — same Jacobian degeneracy risk as KeepUpright. May need angular projection if the hinge axis drifts under load. |
| **Prismatic** | 5 rows (2 linear + 3 angular) | Maybe for angular | Medium | Same angular concern as hinge. |
| **Motor** | 1-2 rows with target velocity | No | Low | Motors drive toward a target, not enforce a hard constraint. PGS handles this well. |

### Adding a new constraint type: checklist

1. Add a variant to `ConstraintKind` with parameters and `row_count()`.
2. Add expansion logic in a new file under `constraint/` (e.g., `distance.rs`).
3. Wire expansion into `expand_constraints()` in `expand.rs`.
4. **Test with PGS rows only first.** Many constraints work fine without projection.
5. **If the constraint fails under contact load** (residual drift, instability, energy artifacts), add a projection arm in `projection.rs`:
   - Define what "satisfied" means geometrically (which velocity components to keep/strip)
   - Use the cross-product or equivalent non-linearized formulation
   - Skip for soft constraints (compliance > 0)
6. Add tests: free-space validation, resting on ground, under contact load.
7. For two-body constraints: wire constraints into the island builder for sleep (see "What to defer").

### Tuning expectations

- **PGS-only constraints** (soft, positional): should work out of the box. The existing `constraint_position_beta` (0.2) and solver iterations (3) are adequate.
- **Hard angular constraints** (KeepUpright, hinge angular part): expect to need projection. The Jacobian degeneracy problem is inherent to linearized angular constraints in PGS.
- **Mixed constraints** (hinge = positional + angular): the positional part may work with PGS alone while the angular part needs projection. The projection function can selectively enforce only the angular DOFs.

---

## Performance

### Memory layout

`ConstraintRow` is a flat struct (~112 bytes). Rows are stored contiguously in a `Vec<ConstraintRow>`, giving linear memory access during the solve loop.

### Work buffers

All buffers are pre-allocated on `PhysicsWorld` and reused each frame:

| Buffer | Type | Lifecycle |
|--------|------|-----------|
| `cached_constraint_rows` | `Vec<ConstraintRow>` | Cleared + refilled in `update_contacts()`. Reused across substeps. |
| `Constraint::warm_impulses` | `SmallVec<[f32; 6]>` | Persistent on the arena entry. Written back after each substep. |

No per-frame heap allocation.

### Projection cost

`project_angular_velocities` iterates the constraint arena once per substep. Per KeepUpright constraint: 1 quaternion-vector multiply, 1 dot product, 1 cross product, 1 vector set. Negligible compared to the PGS solver.

---

## File layout

```
src/physics/
├── constraint/
│   ├── mod.rs              — re-exports
│   ├── types.rs            — Constraint, ConstraintKind, ConstraintRow, ConstraintHandle
│   ├── expand.rs           — expand_constraints(), write_back_constraints()
│   ├── keep_upright.rs     — KeepUpright PGS row expansion
│   ├── projection.rs       — post-solve angular velocity projection (hard constraints)
│   └── distance.rs         — (future) Distance expansion
├── solver/
│   ├── constraint_solver.rs — ConstraintSolver trait
│   ├── pgs_ngs.rs          — constraint rows in phases 2-3 (warm-start, SI loop)
│   ├── constraint_row.rs   — solve_constraint_row(), warm_start_constraint_row()
│   └── ...                 — other solver files unchanged
└── world.rs                — constraint arena + cached rows + projection call in substep()
```

---

## What to defer

- **Constraint islands for sleeping** — constraints should participate in island building so constrained assemblies sleep/wake as a unit. Safe to defer only because KeepUpright is world-anchored on the player, which never sleeps. **Must ship before any two-body constraint.**
- **Constraint-aware shock propagation** — if a constraint bridges two contact sub-graphs, the conditioner needs to see constraint edges. Not needed for world-anchored constraints.
- **Per-substep row re-expansion** — the initial implementation expands rows once per frame. If multi-substep drift becomes visible, re-expansion can be added.
- **Constraint breaking** (max force thresholds) — not needed initially.
- **Constraint debug visualization** — rendering constraint axes, limits, error vectors.
- **Hinge, ball-joint, prismatic, motor variants** — add as needed. See the readiness table above.
