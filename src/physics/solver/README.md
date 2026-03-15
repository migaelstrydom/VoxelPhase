# Contact Solver

Pluggable contact constraint solver for the physics engine.

## Architecture

```
ContactSolver (trait)
├── prepare()   — called once after contact generation, before substeps
└── solve()     — called each substep to solve velocity + position constraints

PgsNgsSolver (implementation)
├── PgsNgsConfig        — solver iterations, warm-start scale, position correction config
├── contact_generation_positions — body position snapshots for stale-depth correction
└── 4-phase solve pipeline:
    1. Capture pre-solve velocities + warm-start scales
    2. Warm-start from cached impulses
    3. Iterative sequential-impulse (normal + friction)
    4. Position correction (NGS or Baumgarte)
```

## Files

| File | Purpose |
|------|---------|
| `contact_solver.rs` | `ContactSolver` trait definition |
| `pgs_ngs.rs` | PGS+NGS solver: config, state, trait impl |
| `body_pair.rs` | `BodyPairState` — extracted kinematics for a contact pair |
| `normal.rs` | Normal impulse solve (restitution, accumulated clamping) |
| `friction.rs` | Per-contact friction + manifold-level friction projection |
| `impulse.rs` | `apply_impulse_pair()`, tangent basis construction |
| `warm_start.rs` | Warm-start application, adaptive iteration count |
| `position_correction.rs` | NGS direct correction + Baumgarte fallback |
| `diagnostics.rs` | Optional per-impulse diagnostic logging |
| `ccd.rs` | Transient contact solve for CCD (no warm-start) |

## Adding a new solver

1. Create a new file (e.g., `tgs_soft.rs`).
2. Implement `ContactSolver` for your struct.
3. Re-export from `mod.rs`.
4. Pass to `PhysicsWorld::with_solver()` or `PhysicsWorld::with_components()`.

```rust
let solver = Box::new(TgsSoftSolver::new(config));
let world = PhysicsWorld::with_solver(physics_config, solver);
```

Bench scenarios can use `build_world()` to return a world with a custom solver,
allowing side-by-side comparison of solver strategies on the same scenario.
