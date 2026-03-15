# Constraint System Plan

Design document for adding a generalized constraint system to the physics engine. The immediate motivation is keeping the player capsule upright, but the system should be general enough to support hinges, distance constraints, motors, and other joint types.

---

## Status

Not started. Parked until higher-priority engine work is complete.

---

## Problem statement

The player character uses a capsule collider that falls over under contact forces. There is no mechanism to constrain a body's degrees of freedom — orientation locking, joint limits, or driven motion in constraint-space.

A per-body angular factor hack (scaling rows of the inverse inertia tensor) would solve the immediate problem but doesn't generalize. A proper constraint system participates in the solver loop alongside contacts, giving correct force interactions.

---

## Design principles

1. **Constraints are first-class solver citizens.** They are iterated in the same PGS loop as contacts, so constraints and contacts see each other's corrections within each iteration. This is critical for convergence with the engine's existing solver struggles.
2. **Enum, not trait.** Constraint types are an enum to keep the solver loop monomorphic. New constraint types are added as variants.
3. **Fits existing position correction.** Constraint drift is corrected through the same post-stabilization mechanism used for contact penetration, not a separate Baumgarte pass.

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

### Core data per constraint

- **body_a, body_b** — the constrained body pair (either may be None for world-anchored constraints)
- **Jacobian row** — 12 floats (linear + angular for each body), or stored implicitly per variant
- **effective_mass** — precomputed scalar `1 / (J · M⁻¹ · Jᵀ)`
- **accumulated_impulse** — for warm-starting and clamping
- **impulse_bounds** — `(min, max)` per constraint row. Contacts use `[0, +∞)`, motors use `[-max, +max]`, limits use `[0, +∞)` or `(-∞, 0]`
- **bias** — velocity bias for position correction or target velocity for motors
- **compliance** — inverse stiffness (0 = rigid, >0 = soft). Folded into effective mass as `effective_mass = 1 / (J·M⁻¹·Jᵀ + compliance/dt²)` (XPBD-style)

### Constraint enum

```rust
pub enum ConstraintKind {
    /// Keep a body's world-space up-axis aligned with a target direction.
    /// 2 constraint rows (tilt around two horizontal axes).
    KeepUpright {
        body: RigidBodyHandle,
        /// Target up direction (world-space, typically +Y).
        target_up: UnitVector3<f32>,
        /// Angular compliance (0 = perfectly rigid).
        compliance: f32,
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

### Solver-ready form

Before the solve loop, each `ConstraintKind` is expanded into one or more `ConstraintRow`s — the flat, uniform struct the solver actually iterates:

```rust
pub struct ConstraintRow {
    body_a: Option<RigidBodyHandle>,
    body_b: Option<RigidBodyHandle>,
    /// Linear Jacobian for body A (world-space).
    lin_jac_a: Vector3<f32>,
    /// Angular Jacobian for body A (world-space).
    ang_jac_a: Vector3<f32>,
    /// Linear Jacobian for body B (world-space).
    lin_jac_b: Vector3<f32>,
    /// Angular Jacobian for body B (world-space).
    ang_jac_b: Vector3<f32>,
    /// Precomputed 1 / (J·M⁻¹·Jᵀ + compliance/dt²).
    effective_mass_inv: f32,
    /// Velocity bias (position correction or motor target).
    bias: f32,
    /// Accumulated impulse for warm-start and clamping.
    accumulated_impulse: f32,
    /// Impulse bounds (min, max).
    bounds: (f32, f32),
}
```

---

## Solver integration

### Unified iteration loop

The solver's main PGS loop currently iterates over `SolverManifold`s. The loop should be extended to also iterate over `ConstraintRow`s within the same pass:

```
for iteration in 0..solver_iterations {
    for manifold in &mut manifolds {
        solve_manifold(bodies, manifold);
    }
    for row in &mut constraint_rows {
        solve_constraint_row(bodies, row);
    }
}
```

Constraints and contacts within the same iteration see each other's velocity corrections, which is the key benefit of unified solving.

### Solving a single constraint row

The per-row solve is the standard PGS step:

1. Compute current constraint velocity: `Cdot = J · v` (dot products of Jacobians with body velocities)
2. Compute impulse: `lambda = effective_mass_inv * -(Cdot + bias)`
3. Clamp: `new_accumulated = clamp(accumulated + lambda, bounds.0, bounds.1)`
4. Compute actual delta: `delta_lambda = new_accumulated - accumulated`
5. Apply: update body velocities using `delta_lambda * M⁻¹ * Jᵀ`

This is nearly identical to how contact normal impulses are solved, just with a generic Jacobian instead of a contact-specific one.

### Warm-starting

Constraint rows store `accumulated_impulse` across frames (like contact manifolds). On frame N+1, the cached impulse is applied scaled by the warm-start factor before the iteration loop begins. Constraints that didn't exist in the previous frame start with zero.

### Position correction

Constraint position error (drift) feeds into the same post-stabilization pass as contact penetration. Each constraint row computes a position-level error `C` (e.g., distance from target orientation for keep-upright). The bias term includes a position correction component: `bias = -(beta/dt) * C`, where `beta` is the stabilization factor from `PhysicsConfig`.

---

## Storage and API

### PhysicsWorld

```rust
pub struct PhysicsWorld {
    // ... existing fields ...
    constraints: Arena<Constraint>,
}

impl PhysicsWorld {
    pub fn create_constraint(&mut self, kind: ConstraintKind) -> ConstraintHandle { ... }
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) { ... }
}
```

### ConstraintHandle

Follows the same pattern as `RigidBodyHandle` and `ColliderHandle` — a newtype around a generational arena index.

---

## Keep-upright constraint (first implementation)

This is the driving use case. It constrains a body's local Y-axis to align with world +Y.

### Constraint rows

Two rows, one for each tilt axis:

- **Row 1:** angular Jacobian = world X-axis (prevents tilt around X)
- **Row 2:** angular Jacobian = world Z-axis (prevents tilt around Z)

Both rows have `lin_jac = zero` (pure angular constraint), `body_b = None` (anchored to world).

### Position error

The tilt angle around each axis is computed from the body's current rotation:

```
local_up = body.rotation * Vector3::y()
error_x = local_up.dot(world_z)   // sin of tilt around X
error_z = -local_up.dot(world_x)  // sin of tilt around Z
```

These feed into the bias term for drift correction.

### Bounds

`(-max_torque, +max_torque)` — practically `(-f32::MAX, f32::MAX)` for a rigid upright lock. Could be reduced for a "self-righting" behavior with limited strength.

---

## File layout

```
src/physics/
├── constraint/
│   ├── mod.rs              — re-exports
│   ├── types.rs            — ConstraintKind enum, ConstraintRow, Constraint
│   ├── keep_upright.rs     — row expansion + error computation for KeepUpright
│   └── distance.rs         — (future) row expansion for Distance
├── handle.rs               — add ConstraintHandle
├── pipeline/
│   └── solver.rs           — extend PGS loop with constraint rows
└── world.rs                — constraint arena + create/remove API
```

---

## What to defer

- Hinge, ball-joint, prismatic, motor variants — add as needed, the infrastructure supports them
- Constraint islands for sleeping — constraints should participate in island building, but this can be added after the basic system works
- Shock propagation interaction — constraints don't have a depth in the contact graph; they use real masses. Revisit if this causes issues
- Constraint breaking (max force thresholds) — not needed initially
