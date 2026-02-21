# Feature-Aware Pipeline

Analysis and implementation plan for replacing `ContactConstraint` with manifold-based
contact persistence, using `FeatureId` for warm-start matching.

## Decision: Option B (manifold-first pipeline)

The engine is young enough that locking in a grab-bag intermediate type
(`ContactConstraint`) would be architectural debt. Option B eliminates the
redundant contact type and makes the manifold the natural unit of persistence,
matching how mature physics engines (Bullet, Box2D, Rapier) structure their
solver loops.

See [Analysis](#analysis) below for the full Option A vs B comparison.

---

## Current state

The collision library produces `ContactPoint` with a `FeatureId` on every contact.
The feature ID is discarded at the narrowphase-to-pipeline boundary, where
`ContactManifold` is converted into `Vec<ContactConstraint>`. From that point on,
the pipeline matches contacts across frames using local-space distance heuristics.

```
collision library           narrowphase              manifold cache
ContactManifold ──────► ContactConstraint ──────► ManifoldPoint
  .feature_id               (no feature ID)         .local_point_b
                             discarded here          matched by distance
```

### Target state

```
collision library           narrowphase              manifold cache
ContactManifold ──────► PairManifold ────────────► CachedManifold
  .feature_id               .manifold                .feature_id → impulses
                             .pair_header             matched by feature
                             (body/collider/material)
```

---

## New data model

### PairHeader

Metadata shared by all contacts in a manifold. Extracted from the current
`ContactConstraint` fields that are identical across all contacts in a pair.

```rust
pub struct PairHeader {
    pub body_a: Option<RigidBodyHandle>,
    pub body_b: RigidBodyHandle,
    pub collider_a: Option<ColliderHandle>,
    pub collider_b: Option<ColliderHandle>,
    pub restitution: f32,
    pub friction: f32,
}
```

### PairManifold

A manifold tagged with its pair metadata. This replaces `Vec<ContactConstraint>`
as the unit of data flowing through the pipeline.

```rust
pub struct PairManifold {
    pub header: PairHeader,
    pub manifold: ContactManifold,  // from collision library (has FeatureId)
}
```

### CachedManifold

The manifold cache stores per-point impulses keyed by `FeatureId`, replacing
the current `ManifoldPoint` with its local-space distance matching.

```rust
struct CachedManifold {
    points: SmallVec<[CachedContact; 4]>,
}

struct CachedContact {
    feature_id: FeatureId,
    normal_impulse: f32,
    tangent_impulse: [f32; 2],
    normal: Vector3<f32>,
    depth: f32,
    age: u8,
}
```

### SolverManifold

Working data for the solver. The manifold cache produces these by merging
`PairManifold` with cached impulses. The solver writes accumulated impulses
back after solving.

```rust
pub struct SolverManifold {
    pub header: PairHeader,
    pub contacts: SmallVec<[SolverContact; 4]>,
}

pub struct SolverContact {
    pub point: Point3<f32>,
    pub normal: Vector3<f32>,
    pub raw_normal: Vector3<f32>,
    pub depth: f32,
    pub raw_depth: f32,
    pub feature_id: FeatureId,
    pub warm_normal_impulse: f32,
    pub warm_tangent_impulse: [f32; 2],
    pub accumulated_normal_impulse: f32,
    pub accumulated_tangent_impulse: [f32; 2],
}
```

The solver iterates manifolds, then contacts within each manifold. Accumulated
impulses are written in-place during solving, then read back by the cache.

---

## Data flow

```
                  Narrowphase
                  ───────────
static_contacts ──► Vec<PairManifold>  ─┐
dynamic_contacts ─► Vec<PairManifold>  ─┤
                                        │
                  Manifold Cache        │
                  ──────────────        │
                  merge(raw) ◄──────────┘
                      │
                      ▼
                  Vec<SolverManifold>  (warm-start impulses populated)
                      │
                  Solver               │
                  ──────               │
                  solve() ◄────────────┘
                      │
                      ├──► write_back() → update CachedManifold impulses
                      │
                  Post-stabilizer      │
                  ────────────────     │
                  post_stabilize() ◄───┘
                      │
                  Sleep / Grounding / Debug
                  ─────────────────────────
                  Consume SolverManifold or flattened ContactEvent
```

---

## Component changes

### Deleted

| Component | File | Reason |
|---|---|---|
| `ContactConstraint` | `pipeline/solver.rs` | Replaced by `PairManifold` + `SolverManifold` |
| `ManifoldPoint` | `pipeline/manifold.rs` | Replaced by `CachedContact` |
| `ContactReducer` | `pipeline/contact_reducer.rs` | Redundant with collision library's `ContactReducer` |
| `reduce_contacts_with_manifold` | `pipeline/manifold.rs` | Replaced by feature-based cache logic |
| `find_closest_point_excluding` | `pipeline/manifold.rs` | Distance matching eliminated |

### Rewritten

| Component | File | Change |
|---|---|---|
| `ManifoldCache` | `pipeline/manifold.rs` | Match by `FeatureId` instead of local-space distance. Input: `Vec<PairManifold>`. Output: `Vec<SolverManifold>`. |
| `ManifoldCache::write_back` | `pipeline/manifold.rs` | Read accumulated impulses from `SolverManifold` contacts by `FeatureId`. |
| `solve` | `pipeline/solver.rs` | Iterate `Vec<SolverManifold>` (manifolds, then contacts within). Warm-start and accumulation operate on `SolverContact` fields directly. |
| `solve_contacts` | `pipeline/solver.rs` | Same restructuring for CCD contacts (no warm-start path). |
| `post_stabilize` | `pipeline/post_stabilizer.rs` | Iterate `SolverManifold` instead of `ContactConstraint`. Field access is the same (point, normal, depth, body handles). |

### Modified (mechanical)

| Component | File | Change |
|---|---|---|
| `generate_static_contacts` | `narrowphase/static_contacts.rs` | Return `Vec<PairManifold>` instead of `Vec<ContactConstraint>`. Remove `manifold_to_constraints` / `manifold_to_speculative_constraints` — wrap `ContactManifold` in `PairManifold` directly. |
| `generate_dynamic_contacts` | `narrowphase/dynamic_contacts.rs` | Same: output `Vec<PairManifold>`. Remove `push_manifold_constraints`. |
| `NarrowphaseWorkBuffer` | `narrowphase/dynamic_contacts.rs` | Store `Vec<PairManifold>` instead of `Vec<ContactConstraint>`. |
| `PhysicsWorld::update_contacts` | `world.rs` | Wire new types through. |
| `PhysicsWorld::substep` | `world.rs` | Wire new types through. |
| `PhysicsWorld::ccd_pass` | `world.rs` | Build `PairManifold` / `SolverManifold` for CCD contacts. |
| `ContactEvent` | `world.rs` | Constructed from `SolverManifold` instead of `ContactConstraint`. |
| `SleepManager` | `sleep/manager.rs` | Accept `&[SolverManifold]` instead of `&[ContactConstraint]`. Flatten to body-handle pairs for island building. |
| `IslandBuilder` | `sleep/islands.rs` | Accept `&[SolverManifold]` instead of `&[ContactConstraint]`. |
| `PhysicsDebugger` | `debug.rs` | Accept `&[SolverManifold]` instead of `&[ContactConstraint]`. |
| `NormalClusterer` | `narrowphase/normal_cluster.rs` | Operate on `ContactManifold` / `ContactPoint` instead of `ContactConstraint`. |

### Unchanged

| Component | File | Why |
|---|---|---|
| Collision library | `src/collision/` | Already produces `ContactManifold` with `FeatureId` |
| `NormalSmoother` | `pipeline/normal_smoothing.rs` | Pure function on normals, no type dependency |
| `GroundingDetector` | `grounding.rs` | Consumes `ContactEvent`, which still exists |
| Force integration | `pipeline/integration.rs` | No contact dependency |

---

## Solver iteration order

The current solver iterates a flat `Vec<ContactConstraint>`. The new solver
iterates `Vec<SolverManifold>`, solving all contacts within a manifold before
moving to the next.

This is **more deterministic** than the current ordering, which depends on
arena iteration order and the interleaving of static and dynamic contacts.
Manifold-then-contact iteration groups related contacts together, which
generally improves convergence for stacking (contacts between the same pair
are solved together rather than interleaved with unrelated pairs).

The `deterministic_contact_ordering` flag currently sorts contacts by body
handle, then collider handle, then depth. In the new model, manifolds are
naturally keyed by collider pair, so the same determinism is achieved by
sorting manifolds by pair key. The per-contact sort within a manifold is
unnecessary since the collision library already produces contacts in a
deterministic order (deepest first from `ContactReducer`).

---

## Acceptance criteria

All existing and passing bench harness tests must pass without regression. (Two are currently failing.)

| Test | Validates |
|---|---|
| `flat_sphere_rest_settles_on_ground` | Sphere-static settling, tail speed < 0.02 |
| `flat_sphere_rest_bouncy_sphere_reaches_higher_peak` | Restitution differentiation |
| `sphere_slide_no_jitter_over_seam` | No seam jitter, max 1 contact on flat terrain |
| `flat_box_rest_zero_restitution_settles_in_tail_window` | Box-static settling, tail speed < 0.02, depth < 0.02 |
| `flat_box_rest_restitution_sweep_exports_and_effective_restitution_order` | Restitution sweep |
| `sphere_on_ramp_rolls_downhill` | Ramp contact, no tunneling |
| `box_on_ramp_slides_downhill` | Box-ramp contact, no tunneling |
| `box_on_step_settles_on_lower_level` | Step terrain, settling |
| `sphere_in_bowl_settles_without_falling_through` | Multi-face concave, >= 2 contacts, tail speed < 0.02 |
| `sphere_sphere_collision_transfers_momentum` | (FAILING) Dynamic sphere-sphere momentum transfer |
| `sphere_obb_collision_transfers_momentum` | Dynamic sphere-OBB momentum transfer |
| `obb_obb_collision_transfers_momentum` | Dynamic OBB-OBB momentum transfer |
| `high_speed_sphere_does_not_tunnel` | CCD prevents tunneling |
| `box_grid_settles_without_explosions` | Many-body stability |
| `box_grid_narrowphase_throughput` | Narrowphase throughput (no perf regression) |
| `box_stack_settles_without_overlap` | (FAILING) 4-box stack: settling, no overlap, tail jitter < 0.03 |

All `cargo test` tests must pass (collision library unit tests, manifold cache
tests, etc.), except for the two called out as failing above.

---

## Implementation plan

### Step 1: New types

Create `PairHeader`, `PairManifold`, `SolverManifold`, `SolverContact` in a
new file `pipeline/pair.rs`. These are pure data types with no logic, so they
compile immediately alongside the existing code.

Also create `CachedContact` and `CachedManifold` in `pipeline/manifold.rs`
(alongside the existing types, which are not yet deleted).

**Files:** `pipeline/pair.rs` (new), `pipeline/manifold.rs` (add types)

### Step 2: Rewrite manifold cache

Rewrite `ManifoldCache` to accept `Vec<PairManifold>` and output
`Vec<SolverManifold>`. The matching logic switches from local-space distance
to `FeatureId` lookup. The `write_back` method reads accumulated impulses
from `SolverContact` fields.

During this step, the old `update` and `write_back` signatures are replaced.
This will break compilation of `world.rs` and everything downstream — that's
fine, we fix it in the next steps.

**Files:** `pipeline/manifold.rs` (rewrite)

### Step 3: Rewrite narrowphase output

Change `generate_static_contacts` and `generate_dynamic_contacts` to return
`Vec<PairManifold>`. Delete `manifold_to_constraints`,
`manifold_to_speculative_constraints`, `push_manifold_constraints`. The
`NarrowphaseWorkBuffer` stores `Vec<PairManifold>`.

**Files:** `narrowphase/static_contacts.rs`, `narrowphase/dynamic_contacts.rs`

### Step 4: Rewrite solver

Change `solve` to accept `&mut [SolverManifold]`. Iterate manifolds, then
contacts within each manifold. Warm-start reads from `SolverContact` fields.
Accumulated impulses are written into `SolverContact` fields in-place.

Change `solve_contacts` (CCD path) similarly.

The solver's physics (effective mass, impulse clamping, friction cone) is
unchanged — only the iteration structure and field access change.

**Files:** `pipeline/solver.rs`

### Step 5: Rewrite post-stabilizer

Change `post_stabilize` to accept `&[SolverManifold]`. The per-contact logic
(Baumgarte/split-impulse, rolling resistance, linear damping) is unchanged —
only the iteration and field access change.

**Files:** `pipeline/post_stabilizer.rs`

### Step 6: Wire through world.rs

Update `PhysicsWorld::update_contacts` and `substep` to use the new types.
Update CCD contact construction to build `PairManifold` / `SolverManifold`.
Update `ContactEvent` construction. Update cached contact fields.

**Files:** `world.rs`

### Step 7: Update downstream consumers

Update `SleepManager`, `IslandBuilder`, `PhysicsDebugger`, and
`NormalClusterer` to accept the new types. These are mechanical changes —
flatten manifolds to iterate contacts where needed, read the same fields
from different struct paths.

**Files:** `sleep/manager.rs`, `sleep/islands.rs`, `debug.rs`,
`narrowphase/normal_cluster.rs`

### Step 8: Cleanup

Delete `ContactConstraint`, old `ManifoldPoint`, `ContactReducer`
(pipeline version — the collision library's reducer remains).
Remove `reduce_contacts_with_manifold`, `find_closest_point_excluding`,
and other dead code. Clean up `mod.rs` re-exports.

Run `cargo test`. All bench harness tests must pass. Run `cargo build` clean
with no dead-code warnings from removed types.

**Files:** `pipeline/solver.rs`, `pipeline/manifold.rs`,
`pipeline/contact_reducer.rs` (delete), `pipeline/mod.rs`

---

## Analysis

### Option A: Add `FeatureId` to the existing pipeline

Add `feature_id` to `ContactConstraint` and `ManifoldPoint`, update the conversion
functions to carry it through, and modify the manifold cache matching logic to prefer
feature-based matching with a distance fallback.

**Pros:**
- Minimal diff. Every component except the manifold cache is a mechanical change.
- No risk to solver, post-stabilizer, sleep, or any other system.
- The distance fallback ensures the transition is seamless — if a collision routine
  produces an unexpected or transitional feature ID, the old matching behavior kicks in.
- Can be done in a single step; the change is small enough to verify in one pass.

**Cons:**
- `ContactConstraint` accumulates more fields. It's already carrying warm-start
  impulses, raw/solver normals, raw/solver depths, body handles, collider handles,
  material properties, and now feature IDs. It's a grab-bag of "everything the
  pipeline needs" rather than a clean abstraction.
- The distance fallback means we're maintaining two matching strategies. If feature-
  based matching works well, the distance code becomes dead weight that's still being
  tested and maintained.

### Option B: Replace `ContactConstraint` with `ContactManifold` (chosen)

Eliminate `ContactConstraint` entirely. The pipeline operates on `ContactManifold`
(from the collision library) directly, with body/collider handles and material
properties stored alongside in a wrapper struct. The manifold cache stores and
matches entries by `FeatureId`.

**Pros:**
- Eliminates the redundant contact type. The collision library's `ContactPoint`
  already has every geometric field that `ContactConstraint` carries.
- Makes the manifold the natural unit of persistence — the cache stores manifolds
  per pair, not individual points reconstructed from flat contact lists.
- Feature-based matching is the only strategy; no dual-path maintenance.
- Solver iterates manifold-then-contacts, matching how mature physics engines
  structure their constraint solver. Generally improves convergence for stacking.

**Cons:**
- Larger diff. Every consumer of `ContactConstraint` must be updated.
- The solver iteration order changes, which affects sequential impulse convergence.
  This needs validation via the bench harness tests.
- Higher risk in one shot. Mitigated by the step-by-step plan above, where each
  step produces a compilable (though temporarily broken at the `world.rs` boundary)
  intermediate state.
