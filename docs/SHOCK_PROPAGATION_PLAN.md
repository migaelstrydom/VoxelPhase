# Shock Propagation Plan

Design document for eliminating inter-manifold velocity oscillation in stacks, heaps, and eccentric load scenarios.

---

## Status
This plan was abandoned because it was not seemed necessary. It's checked in in case we want to get back to it in the future.

## Problem statement

The contact solver processes manifolds independently. When body B participates in two manifolds (e.g. a platform resting on terrain while supporting a heavy sphere), the solver's PGS iterations alternate between the two manifolds, each seeing stale velocities left by the other. With large mass ratios or eccentric loads, this produces persistent velocity oscillation that manifests as visible jitter.

### Why existing fixes don't reach this

The recent solver stability work (world-space friction warm-start, manifold friction budget, block normal micro-iterations) addresses **intra-manifold** coupling — contacts within a single collider pair fighting each other. The remaining jitter comes from **inter-manifold** coupling — multiple manifolds sharing a body and ping-ponging its velocity.

### Concrete failure case

```text
          o         (sphere, 43,000 kg, off-center)
  ─────────────────
 |    platform      |   (box, 2,700 kg)
  ─────────────────
 ═══════════════════   (static terrain)
```

Two manifolds: sphere↔platform and platform↔terrain. The solver processes them in arbitrary order. Whichever manifold solves first leaves the platform's velocity in a state that's wrong for the other manifold. With a 15:1 mass ratio, the corrections are large relative to the platform's momentum, and the system limit-cycles at ~0.026 rad/s angular speed instead of settling.

### Broader symptom

Jumbled heaps of boxes exhibit visible jitter because every body in the heap participates in multiple manifolds with no coordination between them.

---

## Solution: contact graph ordering + shock propagation

Shock propagation is the standard fix for this class of problem (Guendelman et al. 2003, Erleben 2007). Note: most open-source game physics engines (Bullet, Box2D, Rapier) do not actually implement graph-ordered shock propagation — they rely on high iteration counts and warm-starting instead. PhysX achieves similar results through its TGS solver. This implementation draws directly from the academic literature. It combines two ideas:

1. **Contact graph ordering:** Sort manifolds top-down so that constraints furthest from static geometry are solved first and ground-level constraints are solved last, seeing the most accurate velocities.

2. **Mass scaling by depth:** When solving a manifold, temporarily reduce the inverse mass (increase apparent mass) of the body that is closer to static geometry. This makes lower bodies appear heavier during the solve, so upper bodies absorb most of the velocity correction. The load "propagates" down the graph in a single pass instead of needing many PGS iterations.

---

## Contact graph

### Definition

The contact graph is a directed acyclic graph (DAG) where:

- **Nodes** are dynamic rigid body handles.
- **Edges** represent contact support: an edge from A to B means "A rests on B" (B supports A).

Static geometry is not a node — it's the implicit root. Bodies with direct static contacts are at depth 0.

### Edge direction heuristic

For a manifold with `body_a` and `body_b`:

- If `body_a` is `None` (static geometry), `body_b` is at depth 0.
- If both are dynamic, the body whose contact normal points more upward is the **supporter** (lower depth). Specifically: if the average contact normal points from A toward B (i.e. A supports B), then the edge is B→A. In practice, the normal convention in RustDude points from A to B, so `dot(normal, gravity_dir) < 0` means A supports B.
- Ties (near-horizontal contacts, e.g. wall contacts) get no depth edge — both bodies keep their depth from other contacts. A contact is considered horizontal when `|dot(normal, gravity_dir)| < 0.3` (roughly > 72° from gravity). This threshold is a simple geometric cutoff, deliberately independent of friction — tying graph topology to material properties would cause structural instability when materials change.

### Depth assignment

BFS from all depth-0 bodies outward through the contact graph. Each body's depth is the minimum distance from any static-contacting body. Bodies with no path to static geometry get depth = `max_depth` (a configurable cap, e.g. 8).

### Data structure

```rust
/// Per-body depth in the contact support graph.
///
/// Persisted on PhysicsWorld and rebuilt each frame via `rebuild()`. The
/// HashMap allocation is reused across frames (clear + reinsert) to avoid
/// per-frame allocation churn.
pub struct ContactGraph {
    /// Depth for each body handle. Bodies not in the map have no contacts.
    depth: HashMap<RigidBodyHandle, u32>,
    /// Maximum depth assigned this frame.
    max_depth: u32,
    /// BFS work queue, kept allocated between frames.
    bfs_queue: VecDeque<RigidBodyHandle>,
    /// Adjacency edges, kept allocated between frames.
    edges: Vec<(RigidBodyHandle, RigidBodyHandle)>,
}
```

The `ContactGraph` is owned by `PhysicsWorld` and persists across frames. Each frame calls `rebuild()` which clears and repopulates the internal collections without deallocating. This is O(manifolds + bodies) — negligible compared to the solve.

---

## Manifold ordering

After building the contact graph, sort manifolds so that **higher-depth** manifolds are solved **first**. This means:

1. Top-of-stack contacts (leaf nodes) are solved first.
2. Ground-level contacts (depth-0 bodies) are solved last.

Within a single PGS iteration, this ordering lets velocity corrections flow downward through the stack. By the time ground-level contacts are solved, they see velocities that already account for the full load above.

### Sort key

For a manifold between body A (depth `d_a`) and body B (depth `d_b`):

```
sort_key = max(d_a, d_b)    // descending (highest depth first)
```

Tie-breaking uses the existing deterministic collider-handle ordering.

---

## Shock propagation mass scaling

### Core idea

When solving a manifold between two dynamic bodies at different depths, temporarily scale the **lower body's** (smaller depth) inverse mass toward zero. This makes it appear heavier, so the upper body absorbs the correction.

### Scaling formula

For a manifold between bodies at depths `d_upper` and `d_lower` (where `d_upper > d_lower`):

```
shock_factor = shock_alpha ^ (d_upper - d_lower)
```

The lower body's inverse mass and inverse inertia are multiplied by `shock_factor` during the solve:

```
effective_inv_mass_lower   = real_inv_mass   * shock_factor
effective_inv_inertia_lower = real_inv_inertia * shock_factor
```

`shock_alpha` is a tuning parameter in `[0, 1]`:

- `shock_alpha = 0`: lower body is treated as fully immovable (strongest propagation, most stable stacks, but lateral contacts may feel stiff).
- `shock_alpha = 0.5`: moderate — lower body absorbs some correction but is partially stabilized.
- `shock_alpha = 1.0`: no shock propagation (current behavior).

A good starting value is `shock_alpha = 0.3`.

### Depth-0 bodies (resting on static geometry)

Bodies at depth 0 already have a "supporter" with infinite mass (static geometry). No special treatment needed — the existing static-geometry code path handles this.

### Equal-depth contacts

When both bodies have the same depth (lateral contacts, e.g. two boxes side-by-side), no mass scaling is applied. Both bodies use their real mass. This avoids artificially stiffening horizontal interactions.

### Interaction with kinematic-static override

The existing kinematic-static mass override (unit inverse mass for normal solve) takes precedence over shock propagation. Shock propagation only applies to dynamic-dynamic pairs.

---

## Implementation plan

### New file: `src/physics/pipeline/contact_graph.rs`

Owns the `ContactGraph` struct and the graph-building logic.

```rust
pub struct ContactGraph { ... }

impl ContactGraph {
    /// Create an empty contact graph.
    pub fn new() -> Self { ... }

    /// Rebuild the contact graph from the active manifold set.
    /// Clears and repopulates internal collections without deallocating.
    pub fn rebuild(
        &mut self,
        manifolds: &[SolverManifold],
        gravity_dir: Vector3<f32>,
    ) { ... }

    /// Depth of a body, or max_depth if not in the graph.
    pub fn depth(&self, handle: RigidBodyHandle) -> u32 { ... }

    /// Compute the shock mass-scaling factor for each body in a manifold.
    /// Returns (scale_for_a, scale_for_b). The lower-depth body gets the
    /// shock factor; the upper body gets 1.0. When body_a is None (static
    /// geometry), returns (1.0, 1.0) — no scaling needed since the static
    /// body already has infinite mass.
    pub fn shock_factor(
        &self,
        body_a: Option<RigidBodyHandle>,
        body_b: RigidBodyHandle,
        shock_alpha: f32,
    ) -> (f32, f32) { ... }

    /// Sort manifolds in top-down order for shock propagation.
    pub fn sort_manifolds(&self, manifolds: &mut [SolverManifold]) { ... }
}
```

### Modified file: `src/physics/pipeline/solver.rs`

The `solve` function gains a `ContactGraph` parameter:

```rust
pub fn solve(
    bodies: &mut Arena<RigidBody>,
    manifolds: &mut [SolverManifold],
    config: &PhysicsConfig,
    dt: f32,
    contact_graph: &ContactGraph,  // new
) { ... }
```

Inside the solve:

1. **Before warm-start:** call `contact_graph.sort_manifolds(manifolds)` to reorder.
2. **Per manifold:** look up shock factors for each manifold's body pair. Pre-scale the `BodyPairState` fields (`inv_mass_a/b`, `inv_inertia_a/b`) by the shock factors immediately after extracting them from the bodies. This way warm-start, normal solve, friction solve, and post-stabilization all consistently see the shock-scaled masses without threading overrides through every call site.
3. **Kinematic-static override** continues to take precedence — it is applied after shock scaling (and only affects the normal solve path, as before).

### Modified file: `src/physics/world.rs`

Add `contact_graph: ContactGraph` field to `PhysicsWorld`, initialized with `ContactGraph::new()`.

In `substep()`, rebuild and pass the contact graph to `solve()`:

```rust
self.contact_graph.rebuild(
    &self.cached_active_manifolds,
    self.config.gravity.normalize(),
);
solve(
    &mut self.bodies,
    &mut self.cached_active_manifolds,
    &self.config,
    dt,
    &self.contact_graph,
);
```

The gravity direction is derived from the configured gravity vector, so non-downward gravity (e.g. radial gravity for planetary bodies) works without any special-casing.

### Modified file: `src/physics/pipeline/pair.rs`

Add a `shock_scales` field to `PairHeader`:

```rust
/// Shock propagation mass scaling factors for this manifold's body pair.
/// Set by `ContactGraph::sort_manifolds()`. (scale_for_a, scale_for_b)
/// where the lower-depth body gets the shock factor and the upper gets 1.0.
/// Default: (1.0, 1.0) (no scaling).
pub shock_scales: (f32, f32),
```

This is populated during `sort_manifolds()` so that the solver, warm-start, and post-stabilization can all read the factors without re-querying the contact graph.

### Modified file: `src/physics/world.rs` (config)

Add to `PhysicsConfig`:

```rust
/// Shock propagation decay factor. Controls how much lower bodies are
/// stabilized when solving upper contacts. 0.0 = full propagation
/// (lower body immovable), 1.0 = disabled.
pub shock_alpha: f32,
```

Default: `0.3`.

### Modified file: `src/physics/pipeline/mod.rs`

Add `pub mod contact_graph;`.

---

## Interaction with existing solver features

### Block normal micro-iterations

Block normal micro-iterations operate **within** a manifold and are unaffected by shock propagation. The mass overrides from shock propagation are applied to `BodyPairState` before the block solve begins, so the micro-iterations naturally use the shock-scaled masses.

### Manifold friction budget

The friction budget (`mu * sum(lambda_n)`) uses the accumulated normal impulses, which are computed with shock-scaled masses. This means the budget naturally adapts — lower bodies with reduced inverse mass produce larger normal impulses, giving a larger friction budget, which is physically correct (heavier apparent mass → more friction capacity).

### Warm-start

Warm-start impulses from the previous frame were computed with the previous frame's shock factors. Since the contact graph can change between frames (bodies join/leave the stack), the warm impulses may be slightly inconsistent with the new shock factors. This is acceptable — the PGS iterations will correct any mismatch within a few iterations. The warm-start scale (`0.6`) already provides margin for this kind of frame-to-frame variation.

### Post-stabilization

Post-stabilization (split impulse) should also use shock-scaled masses. The `post_stabilize` function reads shock factors from `header.shock_scales` (populated during `sort_manifolds()`), so no additional parameters are needed.

### Deterministic ordering

The current `deterministic_contact_ordering` flag sorts manifolds by collider handle for reproducibility. Repurpose this flag as `enable_shock_propagation` (default `true`) which controls both the depth-based ordering and mass scaling. When disabled, the solver falls back to the current unordered behavior with real masses — useful for A/B testing and debugging regressions. The depth-based ordering is itself deterministic given deterministic contact generation.

### Sleep system

Sleeping bodies are excluded from the active manifold set before the solver runs, so they don't participate in the contact graph. When a sleeping body is woken (e.g. by a new contact), it enters the graph at whatever depth its contacts dictate. No special handling needed.

### CCD path

The CCD path uses `solve_contacts()` — a separate entry point with no warm-starting and a single pass. CCD contacts are typically single-manifold (one fast body vs one surface), so the contact graph provides no benefit. The CCD path uses real masses with no shock propagation.

---

## Validation plan

### Target test

`heavy_sphere_on_platform_near_edge_settles_without_rotational_jitter` should pass with `tail_max_angular_speed < 0.005`. Current value: 0.026.

### New tests

1. **Tall stack convergence:** 6-box stack on flat terrain. All boxes should settle with low tail speed. This tests that shock propagation allows the load to propagate from top to bottom in few iterations.

2. **Lateral contact stability:** Two boxes side-by-side on flat terrain, pushed together. Neither should jitter or gain energy. This tests that equal-depth contacts (no shock scaling) remain stable.

3. **Heavy-on-light pyramid:** A heavy box resting on two lighter boxes, each resting on terrain. The lighter boxes should not oscillate. This tests multi-path support in the contact graph.

### Regression tests

All existing bench harness tests must remain green, including:
- `box_stack_settles_without_overlap`
- `sliding_sphere_decelerates_without_angular_spikes`
- `low_friction_ramp_sphere_slides_down`
- `sphere_in_bowl_settles_without_falling_through`

### Metrics

- Tail max angular speed for eccentric-load scenarios.
- Solver iteration count vs. convergence quality (measure residual velocity at contacts).
- Contact graph build time (should be negligible).

---

## Tuning guidance

### `shock_alpha`

| Value | Behavior | Use case |
|-------|----------|----------|
| 0.0   | Lower body fully immovable during upper solve | Maximum stack stability, but lateral contacts between stacked bodies feel stiff |
| 0.2–0.4 | Strong propagation with some lower-body response | Good default range for games |
| 0.5–0.7 | Moderate propagation | Better for scenarios with significant lateral forces |
| 1.0   | Disabled (current behavior) | Debugging, A/B comparison |

### Interaction with solver iterations

Shock propagation reduces the number of PGS iterations needed for convergence. `IterationBudget` currently adds extra iterations to islands with multi-contact bodies. With shock propagation active, these extras may become unnecessary. Consider reducing the base iteration count after shock propagation is validated, to reclaim the performance spent on micro-iterations.

---

## Risks and mitigations

1. **Risk:** Lateral contacts feel stiff because one body has artificially high mass.
   **Mitigation:** Only apply shock scaling when `depth_upper != depth_lower`. Equal-depth contacts use real masses.

2. **Risk:** Contact graph has cycles (two bodies mutually supporting each other).
   **Mitigation:** Use BFS from depth-0 seeds. Bodies not reached by BFS get `max_depth`. Cycles are broken by the BFS traversal order — the first body reached gets the lower depth.

3. **Risk:** Graph changes rapidly between frames, causing warm-start inconsistency.
   **Mitigation:** The warm-start scale (0.6) and PGS iterations absorb small frame-to-frame variations. If needed, damp the depth assignment (use a running average) to smooth transitions.

4. **Risk:** Performance regression from graph building and manifold sorting.
   **Mitigation:** Graph building is O(manifolds) with a HashMap. Manifold sorting is O(n log n). Both are negligible compared to the solve. Benchmark to confirm.

---

## Appendix: why PGS needs this

In standard PGS, information propagates one constraint per iteration. For a stack of N bodies, it takes N iterations for a force applied at the top to reach the bottom. With 8 base iterations and a 10-body stack, the bottom contacts never see the full load — they under-correct, the top over-corrects, and the system oscillates.

Shock propagation short-circuits this by making lower bodies appear heavier. When the top body's contact is solved, the platform barely moves (it's artificially heavy). When the platform's ground contact is solved, it sees a velocity that already reflects the top load. The effect is that load propagates through the entire stack in a single iteration pass, regardless of stack height.

This is why contact graph ordering and mass scaling must work together: ordering ensures the solver processes top-down, and mass scaling ensures each level's correction doesn't over-disturb the level below.
