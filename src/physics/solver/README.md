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
├── islands             — SolverIslands, rows grouped by the movable bodies they share
├── island_solvers      — one IslandSolver per island: its own SolverBodies,
│                         ContactRows, joint slots and IterationBudget
└── 4-phase solve pipeline:
    1. Build islands from the rows' body handles (static geometry links nothing).
    2. In parallel (rayon), per island: gather its bodies into its own
       SolverBodies, prepare its contact rows, warm-start (joints, then
       contacts), and iterate for the island's own count (joints, then
       contacts, per iteration). Islands share no movable body, so the result
       does not depend on thread scheduling.
    3. Scatter every island's velocities back to the arena.
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
| `constraint_row.rs` | `solve_constraint_row()`, `warm_start_constraint_row()`, impulse application (on `SolverBodies`) |
| `conditioning.rs` | `ManifoldConditioner` trait, `ManifoldConditions`, `IdentityConditioner` |
| `shock_propagation.rs` | `ShockPropagationConditioner` with BFS contact graph |
| `pgs_ngs.rs` | PGS+NGS solver: config, state, trait impl |
| `body_pair.rs` | `BodyPairState` — pose and shock-scaled mass properties of a contact pair |
| `contact_row.rs` | `ContactRow` — per-contact lever arms, effective masses and impulse response, prepared once per substep |
| `normal.rs` | Normal impulse solve (restitution, accumulated clamping) |
| `normal_block.rs` | `NormalCoupling` — a manifold's normal rows solved together, exactly |
| `friction.rs` | Per-contact friction + manifold-level friction projection |
| `impulse.rs` | Tangent basis construction |
| `solver_bodies.rs` | `SolverBodies` — dense velocities of every body the rows touch, gathered before the velocity phase and scattered after it |
| `solver_islands.rs` | `SolverIslands` — union-find over solver slots; static geometry never joins two islands |
| `iteration_budget.rs` | `IterationBudget` — extra iterations for many contacts or disagreeing normals, per island |
| `island_solver.rs` | `IslandSolver` — the velocity phase of one island, self-contained so islands run on separate threads |
| `warm_start.rs` | Warm-start application |
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
