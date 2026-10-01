# Physics Engine Architecture

How the rigid-body engine in `src/physics/` (with collision primitives in `src/collision/`)
steps a frame, what each stage owns, and the invariants that are easy to break. This
describes the engine as built. The documents listed under "Related documents" record the
reasoning behind individual parts.

Each file's `//!` header is the authority on its own details. This doc is the map that
connects them.

## Boundaries

- **No ECS, no rendering, no terrain types.** The engine is handle-based
  (`RigidBodyHandle`, `ColliderHandle`, `ConstraintHandle` over generational arenas).
  `systems/physics_sync.rs` is the only ECS glue.
- **Static geometry comes through a trait.** `StaticGeometry::query_region(aabb)` returns
  a `MeshPatch` (triangles plus local adjacency), and `StaticGeometry::surface` names what
  each triangle is made of. Queries run from several threads at once, so implementors are
  `Sync`. Terrain implements it; the engine never sees chunks, segments or voxels.
- **Gameplay asks for motion through the drive seam** (`physics::drive`), never by
  writing velocities each frame. See "Drive" below.
- **Media see a body's bulk, not its colliders.** Water, buoyancy, drag and splash read
  `RigidBody::envelope()`, and mass properties read `mass_parts()`. Both fall back to the
  colliders unless the body declares a `BulkShape` (`bulk.rs`).

## Bodies, colliders, constraints

- `RigidBody` (`body.rs`): `Dynamic`, `Kinematic` or `Static`. **Its origin is its centre
  of mass.** Detaching colliders (fracture, cleaving) breaks that until
  `PhysicsWorld::recenter_on_colliders` is called. Free rotation integrates the gyroscopic
  term implicitly (a few Newton steps), because the explicit form gained energy without
  bound on asymmetric bodies.
- `Collider` (`collider.rs`): `Sphere`, `Box`, `Capsule`, `ConvexHull`, with a
  `ColliderMaterial` (friction, restitution, density). Colliders attach to and detach from
  bodies at runtime. `detach_collider` swap-removes, so collider order changes after a split.
- Constraints (`constraint/`): persistent definitions in an arena, `ConstraintKind` =
  `KeepUpright`, `KeepAttitude`, `Fixed`, `BallJoint`, `Hinge`, `FollowPoint`. Each is
  expanded every frame into solver `ConstraintRow`s (`expand.rs`, built from
  `primitives.rs`). The enforcement mode is `Iterative` (PGS, optionally NGS) or
  `HardProjection` (a post-solve velocity projection). Compliance rules out hard
  projection. A constraint whose bodies are all asleep is skipped, and
  `ConstraintKind::permits_sleep` decides whether a constrained body may sleep.

## A frame

`PhysicsSyncSystem` drives a `Stepper`. The game uses `SequentialStepper` with a fixed
1/240 s substep and at most 12 substeps per frame (`FixedTimestep` clamps the accumulator,
which prevents a spiral of death). A frame is **one contact pass, then N substeps**:

```text
PhysicsWorld::update_contacts(dt, substeps)         once per frame
  ├─ sleep bookkeeping, wake events, one-shot impulses (explosions)
  ├─ generate_contacts                               static, then dynamic; repeated for
  │     └─ narrowphase (+ speculative contacts)      bodies each pass wakes, until none do
  ├─ ManifoldCache::merge                            match by FeatureId → warm-start impulses
  ├─ filter manifolds of sleeping islands; record NarrowphaseOwnership per pair
  ├─ ManifoldConditioner::condition                  shock propagation: order + mass scales
  ├─ SupportResolver::resolve                        Support Sets (what holds each body up)
  ├─ stamp_non_support_grip, TractionPlanner::plan   drive targets written onto contacts
  ├─ ConstraintSolver::prepare                       expand constraint rows
  └─ CcdStrategy::begin_frame

PhysicsWorld::substep(dt)                            × N
  ├─ SubstepForceProvider forces (buoyancy), integrate_forces (gravity)
  ├─ apply_allowances                                the drive's non-conservative authority
  ├─ ConstraintSolver::solve                         velocity phase per island + NGS position pass
  ├─ ledgers: impacts, contact work, traction
  ├─ ManifoldCache::write_back / prune, constraint write-back
  ├─ ConstraintSolver::project_velocities            HardProjection constraints
  ├─ integrate_bodies                                positions and rotations
  ├─ CcdStrategy::run                                sweep fast colliders the narrowphase does not own
  └─ SleepManager::update_sleep_states
```

**The central fact: contacts are generated once per frame but integrated over up to 12
substeps.** Every safeguard against tunnelling and every contact reference has to cover the
whole frame, not one substep. Speculative contacts, closing allowances, NGS's reference
positions, CCD's frame gate and narrowphase ownership all exist because of this.

## Narrowphase

`narrowphase/` produces `PairManifold`s (at most 4 points each, with a `FeatureId` per
point) into a `NarrowphaseWorkBuffer` that is reset once per frame.

**Scope** (`scope.rs`): only awake bodies start pairs. When a pass wakes a sleeping body,
another pass generates contacts for exactly the bodies it woke, until a pass wakes nobody.
A stack touched at the top is therefore solved whole, down to the ground, on the frame it
wakes, rather than one layer per frame.

**Against static geometry** (`static_contacts.rs`, in parallel per collider): query the
patch around the collider, then run the mesh pipeline in `collision/mesh/`:

```text
MeshPatch ──seam_filter──▶ FilteredPatch ──shape routine──▶ manifold
            merge coplanar      (faces + boundary/       sphere_patch, capsule_patch,
            triangles into      crease edges)            obb_patch, gjk_patch (hulls)
            convex quads                                  + crease_contacts, solid_side
```

- Mesh contacts take the **face normal**, and depth comes from projecting the shape's
  deepest support point. GJK/EPA is not used against large flat polygons (the extreme
  aspect ratio makes it unstable).
- `solid_side`: a face whose plane the shape's centre has crossed still pushes while the
  centre is within reach behind it, unless a back-to-back face would push the shape out a
  shorter way. A shape always leaves solid on the side its centre is on.
- `crease_edges` / `crease_contacts`: convex ridges that no face contact reaches (a post's
  edge, a hilltop fold) push the shape out along whichever of its face normals clears the
  whole run of the crease.
- `static_surface.rs`: friction and restitution come from `StaticSurface::meet`. Ground
  that yields (grass, sand) imposes its own grip; two rigid surfaces combine as two
  colliders do.
- A body with `ignores_static` skips static contacts and CCD. This is used for bodies
  welded to the world whose colliders are bedded in terrain.

**Between bodies** (`dynamic_contacts.rs`): per-collider world bounds, widened over the
frame's travel for colliders in the speculative band, go through `SweepAndPrune`
(`broadphase/`). `collision/dispatch.rs` routes each pair to an analytic or SAT routine
(sphere/capsule/OBB combinations, `obb_obb`, `hull_obb`, `hull_hull` with per-pair SAT
caches) and falls back to GJK/EPA with a per-pair GJK cache. Contacts are reduced to 4 by
area (`contact_reducer.rs`).

**Speculative contacts** (`speculative.rs`): a pair that is not touching but will meet
within the frame gets a contact generated at its predicted meeting pose and carried back,
with the **gap** it still has to close. The solver lets the pair approach by that gap and
arrests it only beyond it (`solver/closing_allowance.rs`, recomputed every substep from how
far the bodies have moved), so the pair stops on arrival. A predicted contact without its
gap halts bodies in mid-air.

## Manifold cache

`pipeline/manifold.rs` matches new contacts to last frame's by `FeatureId`, using proximity
as the tiebreak among points that share one. Matched contacts inherit their accumulated
impulses (warm start). A contact point survives `manifold_max_age` frames without a
narrowphase refresh before it is pruned.
This persistence is the single largest contributor to stable resting contact.

## Conditioning: shock propagation

`ManifoldConditioner` (`solver/conditioning.rs`) runs once per frame on the active
manifolds. The default is `ShockPropagationConditioner`. It runs a BFS outward from static
geometry through the contact graph, orders manifolds top-down, and gives each manifold
within 45° of vertical a mass scale so that the lower body appears heavier
(`shock_alpha` 0.3).

- A single-body constraint (`KeepAttitude` on the player) does not make a body
  "ground". Only static geometry roots the BFS.
- The scaling creates energy when a structure collapses dynamically, because scaled
  impulses are not equal and opposite. It is kept because heavy-on-light stacks sag, creep
  or topple without it. Fixes tried and rejected are in `docs/BOX3D_SOLVER_COMPARISON.md`
  and in `physics_fuzz`'s history.

## Solver

`ConstraintSolver` is the pluggable trait (`prepare`, `solve`, `write_back`,
`project_velocities`). The only implementation is `PgsNgsSolver` (`solver/pgs_ngs.rs`).

**Velocity phase, per island, in parallel.** `SolverIslands` (union-find over arena slots)
groups rows that share no movable body; static bodies never join islands. Each
`IslandSolver` gathers its own copy of its bodies' velocities (`SolverBodies`), builds
`ContactRow`s, warm-starts, iterates, and scatters velocities back after every island is
done. The result is independent of thread scheduling. Within an iteration:

- **Normal impulses per manifold are solved together and exactly** (`normal_block.rs`). A
  manifold has at most 4 contacts, so the LCP is solved by trying sets of active contacts.
  Contacts are not solved one at a time, which converged slowly and left a phantom torque
  between close contacts.
- **Friction** (`friction.rs`): the tangential row is bounded by μ·N and driven toward
  `SolverContact::traction.target`. That target is zero for ordinary friction and nonzero
  for a drive.
- **Torsional** (`torsional.rs`): spin about the normal, bounded by μ·N·r. It is inert
  unless an actuator declares a patch radius.
- **Constraint rows** (`constraint_row.rs`), using Baumgarte bias for joints.
- Restitution applies only above `restitution_velocity_threshold`. Warm starting is
  whole (`warm_start_scale` 1.0), and fast contacts are not warm-started.
- Iterations: base 3, plus extra passes earned by an island's hardest body (many contacts,
  or disagreeing normals) (`iteration_budget.rs`).

**Position phase.** NGS (`position_correction.rs`) runs after the velocity phase with real
masses (no shock scaling), and corrects penetration beyond `slop` from the positions at
contact generation, with a per-contact correction-speed cap. It also corrects
`PositionAndVelocity` constraint rows and applies contact rolling resistance and damping.
It moves bodies without giving them velocity.

**Hard projection** then removes the velocity that `HardProjection` constraints forbid.

## Drive

How gameplay moves bodies honestly (`physics/drive/`, design in
`docs/TRACTION_DRIVE_DESIGN.md`). Gameplay sets a `DriveCommand` with
`PhysicsWorld::set_body_drive`. Its gameplay half (`DriveIntent`, `Actuator`) lives in
`src/drive/`.

```text
manifolds ──▶ SupportResolver ──▶ SupportSets ──┬──▶ Grounding (what characters read)
                                                ├──▶ stamp_non_support_grip
                                                └──▶ TractionPlanner ──▶ contact traction targets
DriveCommand ─┬─ support-anchored ──▶ traction targets (friction with a non-zero target)
              ├─ medium-anchored  ──▶ 6 world-anchored motor rows (platform motors)
              └─ allowance        ──▶ apply_allowances (jump, air steer: bounded, ledgered)
```

- **Support Set**: which contacts hold a body up, along the gravity axis.
  `grounding.rs` is its boolean projection, plus carry-over for sleeping bodies.
- **Support anchor**: a drive is friction with a target, so the reaction goes into whatever
  the body stands on (a platform is pushed back). Nothing holding the body up means no
  authority.
- **Medium anchor**: the reaction goes into the world, through ordinary constraint rows
  bounded by `max_accel·dt`, solved alongside gravity.
- **Allowance**: the one sanctioned non-conservative authority (jumping, air steering).
  It is opt-in, bounded per entity, and spent before the solve, so contacts still answer it.
  Edge-triggered verbs fire on the first substep only.
- Both cheats (`drive_gain` > 1 and allowances) are measured in `TractionLedger` /
  `AllowanceLedger` and printed under F3.

## CCD

A safety net for motion the frame's contacts cannot catch. `CcdStrategy` is pluggable; the
default is `SweepClampCcd` (`ccd/sweep_clamp.rs`):

```text
collect_candidates ──▶ static_sweep / dynamic_sweep ──▶ clamp to earliest impact ──▶ solve contact
```

- **Gate**: a collider is swept if it outruns contact generation across one substep
  (`ccd_threshold` × radius) **or** across the whole frame (`ccd_frame_coverage` × radius).
- **Ownership is per pair**: a pair with frame-start narrowphase contacts belongs to the
  solver until it drifts away from the separation its manifold was made at
  (`ownership.rs`). A grenade skimming the floor is still swept against the wall ahead.
- **Grazes at t≈0 are rejected** inside the per-triangle search (`is_tunnelling_hit`), so a
  body sliding along a surface is not frozen and a floor graze cannot mask a wall behind it.
- Static sweeps reuse a per-frame patch cache. Dynamic sweeps use the shared
  `SweepAndPrune` over swept bounds and `collision/continuous/` (analytic or GJK raycast).
- A freed piece exactly flush with a static neighbour is clamped at t=0. Spawn pieces with a
  hairline gap.

## Sleep

`sleep/`: `SleepTracker` counts substeps a body has been still, which means below both
velocity thresholds **and** not moved in pose. Pose matters because NGS moves bodies
without giving them velocity. When every body of a contact island is a candidate, the
island sleeps and its velocities are zeroed. Sleeping bodies start no contacts, are skipped
by integration and CCD, and are woken by contact from an awake body, by `apply_impulse` /
`apply_angular_impulse` / `wake_body`, or by a drive being set. **Every created body starts
asleep.** Body-level impulse methods bypass the wake, so use `PhysicsWorld`'s methods.

## Outputs and diagnostics

- `contact_events()`: contacts this frame (narrowphase and CCD), emitted before the solve.
- `impacts()`: `ImpactLedger`, the normal impulse each body (and collider) received this
  frame, used by fracture, damage and grenade fuses.
- `contact_work()`: `ContactWorkLedger`, the work each contact did and on whom. Off by
  default; `EnergyAudit` uses it to catch energy the solver made up.
- `frame_profile()`: `FrameProfile` per `PhysicsStage`. Printed under `Physics/Time/` on F3.
- `raycast_excluding`, `probe_bodies`: queries used by sensing and foot placement.
- `PhysicsDebugger` (`debug.rs`) and env-gated solver diagnostics (`solver/diagnostics.rs`).

## Tools and tests

- `src/physics/bench_harness/`: `PhysicsBenchScenario`s run headlessly by
  `run_scenario()` with assertions in `tests/` (`--features bench_harness`, release), and
  in a window by `bench_viewer`.
- `physics_fuzz`: seeded structures disturbed by the real player body, judged by
  `EnergyAudit`.
- `physics_perf`: stage timings on real terrain.
- `level_check`: wakes every level object and reports what doesn't stay at rest.
- When adding a regression test for tunnelling or contact behaviour, check that it **fails
  with the fix reverted**, with frame boundaries straddling the obstacle.

## Extension points

| Trait | Default | Swaps |
|---|---|---|
| `StaticGeometry` | terrain (`TerrainWorld`) | bench geometry in `bench_harness/geometry.rs` |
| `Stepper` | `SequentialStepper` | |
| `ConstraintSolver` | `PgsNgsSolver` | |
| `ManifoldConditioner` | `ShockPropagationConditioner` | `IdentityConditioner` |
| `CcdStrategy` | `SweepClampCcd` | |
| `SubstepForceProvider` | none | `BuoyancyForceProvider` (water) |

## Related documents

- `TRACTION_DRIVE_DESIGN.md`: the drive seam, Support Sets, allowances.
- `BOX3D_SOLVER_COMPARISON.md`: why soft-step substepping was shelved.
- `BLOCK_LCP_POSTMORTEM.md`: an earlier attempt at exact 3–4 contact manifold solves that
  regressed. `normal_block.rs` is the later approach that worked.
- `TERRAIN_BEDDING_DESIGN.md`: `ignores_static` and welded bodies.
- `GJK_EPA_DESIGN.md`, `HULL_SAT_MANIFOLD_PLAN.md`: convex collision.
- `SHOCK_PROPAGATION_PLAN.md`, `CONSTRAINT_SYSTEM_PLAN.md`,
  `SOLVER_CONVERGENCE_ANALYSIS.md`: historical design records. The code has moved on
  since they were written.
