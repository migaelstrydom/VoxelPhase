# Constraint Solver

Pluggable constraint solver for the physics engine. Handles both contact constraints
(from collision detection) and joint constraints (user-defined).

## Architecture

```
ConstraintSolver (trait)
├── prepare()   — called once after contact generation + constraint expansion, before substeps
└── solve()     — called each substep with manifolds, conditions, constraint rows, dt

ManifoldConditioner (trait)
└── condition() — reorders manifolds + computes per-manifold metadata (shock scales)

PhysicsWorld orchestration:
  1. update_contacts()  → narrowphase + manifold cache
  2. conditioner.condition() → reorder manifolds, compute shock scales
  3. expand_constraints() → user constraints → ConstraintRow buffer
  4. solver.prepare()
  5. substep() × N:
     ├── solver.solve(manifolds, conditions, constraint_rows, dt)
     ├── write_back_constraints() → rows → Constraint::warm_impulses
     └── project_angular_velocities() → hard constraint enforcement (post-solve)

PgsNgsSolver (default solver)
├── PgsNgsConfig        — solver iterations, warm-start scale, position correction config
├── contact_generation_positions — body position snapshots for stale-depth correction
└── 4-phase solve pipeline:
    1. Capture pre-solve velocities + warm-start scales
    2. Warm-start from cached impulses (contacts: shock-scaled, joints: real masses)
    3. Iterative sequential-impulse (contacts first, then joints per iteration)
    4. Position correction (NGS or Baumgarte, real masses)

Post-solve projection (in PhysicsWorld, outside the solver):
  For hard constraints (compliance=0), directly enforces the constraint by
  stripping forbidden angular velocity and injecting corrective velocity.
  Uses cross-product formulation that works at all angles (no linearization).
  PGS rows still needed for correct contact coupling — without them, the
  contact solver computes wrong impulses (see CONSTRAINT_SYSTEM_PLAN.md).

ShockPropagationConditioner (default conditioner)
├── ShockPropagationConfig — shock_alpha, horizontal_threshold
└── ContactGraph (internal) — BFS depth from static geometry, reused allocations
```

## Files

| File | Purpose |
|------|---------|
| `constraint_solver.rs` | `ConstraintSolver` trait definition |
| `constraint_row.rs` | `solve_constraint_row()`, `warm_start_constraint_row()`, impulse application |
| `conditioning.rs` | `ManifoldConditioner` trait, `ManifoldConditions`, `IdentityConditioner` |
| `shock_propagation.rs` | `ShockPropagationConditioner` with BFS contact graph |
| `pgs_ngs.rs` | PGS+NGS solver: config, state, trait impl |
| `body_pair.rs` | `BodyPairState` — extracted kinematics for a contact pair (shock-scaled) |
| `normal.rs` | Normal impulse solve (restitution, accumulated clamping) |
| `friction.rs` | Per-contact friction + manifold-level friction projection |
| `impulse.rs` | `apply_impulse_pair()`, tangent basis construction |
| `warm_start.rs` | Warm-start application, adaptive iteration count |
| `position_correction.rs` | NGS direct correction + Baumgarte fallback |
| `diagnostics.rs` | Optional per-impulse diagnostic logging |
| `ccd.rs` | Transient contact solve for CCD (no warm-start, no shock propagation) |

## Adding a new solver

1. Create a new file (e.g., `tgs_soft.rs`).
2. Implement `ConstraintSolver` for your struct.
3. Re-export from `mod.rs`.
4. Pass to `PhysicsWorld::with_solver()` or `PhysicsWorld::with_components()`.

```rust
let solver = Box::new(TgsSoftSolver::new(config));
let world = PhysicsWorld::with_solver(physics_config, solver);
```

## Adding a new conditioner

1. Implement `ManifoldConditioner` for your struct.
2. Re-export from `mod.rs`.
3. Pass to `PhysicsWorld::with_components()`.

```rust
let conditioner = Box::new(MyConditioner::new(config));
let world = PhysicsWorld::with_components(
    physics_config,
    Box::new(PgsNgsSolver::default()),
    conditioner,
    Box::new(SweepClampCcd::default()),
);
```

Bench scenarios can use `build_world()` to return a world with a custom solver
or conditioner, allowing side-by-side comparison on the same scenario.
