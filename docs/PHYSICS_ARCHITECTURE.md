# Physics Engine Architecture

## Design goals

1. **Correctness first.** No tunneling, no energy gain, no jitter at rest.
2. **Independent library.** The engine knows nothing about ECS, rendering, or terrain
   implementation. External geometry is accessed through a trait.
3. **Discrete-primary pipeline.** The narrowphase is the primary contact source; CCD is
   a safety net for fast bodies only. This replaces the existing TOI-first architecture
   where all contacts go through sweep tests and are resolved in time order.
4. **Incremental buildability.** Each section marks its dependencies. Features can be
   implemented in dependency order, with the engine remaining functional after each step.

---

## Pipeline overview

One call to `PhysicsWorld::step(dt)` executes these phases in order:

```
1. Apply gravity and external forces         (force accumulators → velocity)
2. Integrate velocities                       (forces → velocities, clear accumulators)
3. Broadphase                                 (produce candidate pairs)
4. Narrowphase                                (candidate pairs → contact manifolds)
5. Build constraint list                      (contacts + joints → constraints)
6. Warm-start solver                          (apply cached impulses from previous frame)
7. Solve velocity constraints                 (sequential impulses, N iterations)
8. Store solver impulses                      (cache impulses for next frame's warm-start)
9. Integrate positions                        (velocities → positions)
10. CCD pass                                  (fast bodies only: sweep, correct, re-solve)
11. Update sleeping                           (deactivate/wake islands based on energy)
```

Each phase is described in detail below.

---

## Phase 1–2: Force application and velocity integration

**What happens:**
- Gravity is applied to all dynamic bodies, scaled by per-body `gravity_scale`.
- Accumulated external forces and torques are integrated into linear and angular velocities.
- Damping is applied (linear and angular, per-body).
- Force and torque accumulators are cleared.

**Key detail:** Only velocities are updated here. Positions are not touched until phase 9.
This is the semi-implicit Euler scheme: forces update velocities, then the solver corrects
velocities, then velocities update positions. This ordering is what gives the solver authority
to prevent penetration before it happens.

**Dependencies:** None.

---

## Phase 3: Broadphase

**Purpose:** Reduce the O(n^2) pair count to a small set of candidate pairs that might
actually be colliding.

**Approach:** Axis-Aligned Bounding Box (AABB) overlap test. Each collider's world-space AABB
is computed from its shape and its parent body's transform. Pairs whose AABBs overlap are
forwarded to the narrowphase.

**AABB computation:**
- Sphere: center +/- radius on each axis.
- Future shapes (capsule, box, convex hull): compute from rotated vertices or support mapping.

**AABB margin:** Each AABB is expanded by `contact_margin` (typically 0.01–0.05) so that the
narrowphase can generate contacts slightly before geometric overlap. This is essential for
resting contact stability — the solver sees the contact and cancels approach velocity before
the body actually penetrates.

**Data structure options (in order of implementation simplicity):**
1. **Brute force O(n^2):** Sufficient for < 100 bodies. Good starting point.
2. **Sort-and-sweep on one axis:** Good for scenes with a dominant axis.
3. **AABB tree (BVH):** Best general-purpose choice for dynamic scenes.

**Static geometry:** Not part of the broadphase. Dynamic-vs-static contacts are generated
directly in the narrowphase by querying the `StaticGeometry` trait.

**Output:** A set of `(ColliderHandle, ColliderHandle)` candidate pairs for dynamic-vs-dynamic
contacts.

**Dependencies:** Colliders must be able to compute world-space AABBs.

---

## Phase 4: Narrowphase

**Purpose:** For each candidate pair from the broadphase (and for each dynamic collider vs
static geometry), compute the contact manifold: the set of contact points, normals, and
penetration depths.

### 4a: Dynamic-vs-static contacts

For each dynamic body's collider, query the `StaticGeometry` trait:
```
static_geometry.query_sphere(center, radius + contact_margin) → Vec<StaticContact>
```

Each `StaticContact` provides a point, normal, and depth. Because the query uses the expanded
radius, the returned depth includes the margin. The narrowphase must subtract the margin
before passing to the solver:
```
solver_depth = (terrain_depth - contact_margin).max(0.0)
```

This ensures:
- Contacts within the margin skin generate constraints with depth=0 (velocity correction
  only, no position correction).
- Contacts with actual penetration get depth > 0 (position correction applies).

### 4b: Dynamic-vs-dynamic contacts

For each candidate pair from the broadphase, perform shape-specific overlap tests:

**Sphere-vs-sphere:**
```
delta       = center_b - center_a
dist        = |delta|
normal      = delta / dist  (or fallback Y-up if dist < epsilon)
depth       = (radius_a + radius_b) - dist
contact_pt  = center_a + normal * (radius_a - depth/2)
```

A contact is generated if `dist < radius_a + radius_b + contact_margin`. The depth passed to
the solver uses the real radii (no margin inflation), so margin contacts naturally have
depth <= 0 and receive velocity-only correction.

### 4c: Contact manifold persistence

Contact manifolds are cached per collider pair across frames. Each contact point is identified
by a **feature ID** (or by nearest-point matching within a distance threshold). This allows:
- **Warm-starting:** Previous impulses are associated with specific contact points and
  reapplied at the start of the solver (phase 6).
- **Stable normals:** For resting contacts, reusing the previous frame's normal avoids
  jitter from floating-point noise in the narrowphase.

**Manifold data structure:**
```
ContactManifold {
    collider_a: ColliderHandle,
    collider_b: ColliderHandle,     // or None for static
    contacts: SmallVec<[ContactPoint; 4]>,
    last_seen_frame: u64,
}

ContactPoint {
    local_point_a: Point3,          // in body-A local space
    local_point_b: Point3,          // in body-B local space (or world for static)
    normal: Vector3,                // world-space, A-to-B
    depth: f32,
    normal_impulse: f32,            // cached from solver, used for warm-starting
    tangent_impulse: [f32; 2],      // cached friction impulses
}
```

Storing contact points in local space allows matching across frames even as bodies move.
Each frame, recompute world positions from the local points and update depth/normal.

**Contact point matching:** When the narrowphase produces new contacts, match each to the
closest existing contact in the manifold (by local-space distance). Matched contacts inherit
the cached impulses. Unmatched new contacts start with zero impulse. Old contacts not matched
to any new contact are removed.

**Contact reduction:** For 3D, keep at most 4 contacts per manifold (the 4 that maximize the
contact area). This keeps solver cost bounded without losing stability.

**Dependencies:** Manifold cache requires a persistent store indexed by collider pair,
surviving across frames.

---

## Phase 5: Build constraint list

**Purpose:** Flatten all active contact manifolds (and future joints) into a flat list of
constraints for the solver.

Each `ContactPoint` in an active manifold becomes a `ContactConstraint`:
```
ContactConstraint {
    body_a: Option<RigidBodyHandle>,
    body_b: RigidBodyHandle,
    point: Point3,                  // world-space contact point
    normal: Vector3,                // world-space normal, A-to-B
    depth: f32,
    restitution: f32,               // combined material property
    friction: f32,                  // combined material property
    normal_impulse_cache: f32,      // from manifold, for warm-starting
    tangent_impulse_cache: [f32; 2],
}
```

**Material combination:**
- Restitution: average `(e_a + e_b) / 2`.
- Friction: geometric mean `sqrt(mu_a * mu_b)`.

**Dependencies:** Phases 3–4 must have produced contact manifolds.

---

## Phase 6: Warm-start solver

**Purpose:** Apply cached impulses from the previous frame to give the solver a head start.
Without warm-starting, the solver must reconverge from scratch each frame, leading to visible
jitter on resting contacts (the solver doesn't reach equilibrium in a small number of
iterations).

**Procedure:** For each constraint with a cached `normal_impulse_cache > 0`:
1. Compute the impulse vector: `impulse = normal * normal_impulse_cache`.
2. Apply to both bodies: body_a gets `-impulse`, body_b gets `+impulse`, both at the
   contact point (affecting linear and angular velocity).
3. Similarly apply cached tangent impulses along the two friction directions.

**Scaling:** Optionally scale cached impulses by a factor (e.g., 0.8–1.0) for robustness.
A factor of 1.0 is correct when contacts are well-matched; a factor < 1.0 adds safety margin
for poorly-matched contacts. Start with 1.0 and reduce if instability is observed.

**Dependencies:** Manifold persistence (phase 4c) must provide cached impulses.

---

## Phase 7: Solve velocity constraints

**Purpose:** Iteratively adjust body velocities so that all contact constraints are satisfied
(no interpenetration, correct restitution, Coulomb friction).

**Algorithm:** Sequential impulses (Projected Gauss-Seidel). For each solver iteration, loop
over all constraints and solve each one independently, immediately updating body velocities.

### Normal impulse (non-penetration)

For each constraint:
1. Compute relative velocity at the contact point:
   ```
   v_rel = (vel_b + omega_b x r_b) - (vel_a + omega_a x r_a)
   v_n   = v_rel . normal
   ```
2. Compute the velocity bias for position correction (Baumgarte stabilization):
   ```
   bias = (baumgarte_factor / dt) * max(depth - slop, 0)
   ```
   Typical values: `baumgarte_factor = 0.1–0.2`, `slop = 0.001–0.01`.
   If split impulse is enabled, this bias is applied in a separate positional solve
   so it does not add energy to the velocity solution.
3. Compute the desired velocity change:
   ```
   restitution_velocity = restitution * v_n_initial
   ```
   Where `v_n_initial` is the pre-solver relative normal velocity (computed once before
   iterations begin, not updated during iteration). Apply restitution only when
   `|v_n_initial| > restitution_threshold` (typically 0.5–1.0 m/s) to prevent micro-bouncing
   at resting contacts.
4. Compute the effective mass:
   ```
   K = inv_mass_a + inv_mass_b
     + ((I_inv_a * (r_a x n)) x r_a) . n
     + ((I_inv_b * (r_b x n)) x r_b) . n
   m_eff = 1 / K
   ```
5. Compute the impulse magnitude:
   ```
   lambda = -m_eff * (v_n + restitution_velocity + bias)
   ```
6. **Accumulated impulse clamping:**
   ```
   old_accumulated = constraint.accumulated_normal_impulse
   constraint.accumulated_normal_impulse = max(old_accumulated + lambda, 0)
   lambda = constraint.accumulated_normal_impulse - old_accumulated
   ```
   This is critical. Clamping the accumulated impulse (not the per-iteration impulse) ensures
   the solver converges to the correct solution over multiple iterations. Per-iteration
   clamping can overshoot.
7. Apply the impulse: `normal * lambda` to both bodies at the contact point.

### Friction impulse

After solving the normal constraint for a contact:
1. Compute two tangent directions orthogonal to the normal.
2. For each tangent direction, compute relative tangent velocity and solve similarly.
3. Clamp accumulated friction impulse to the friction cone:
   ```
   |accumulated_tangent| <= friction * accumulated_normal_impulse
   ```

### Post-stabilization (split impulse)

To reduce energy injection from Baumgarte, optionally run a positional correction pass
after the main velocity solver. This applies the `bias` term as a position-level impulse
that does not affect linear or angular velocity. It improves tall stacks and resting
stability at the cost of an extra loop over constraints.

Implementation sketch:
- Keep the velocity solve exactly as above with `bias = 0`.
- Run N position iterations using the same constraints, but only solve for the
  penetration bias term.
- Apply the correction to positions/orientations directly (or via a separate
  "pseudo-velocity" accumulator that is not carried into the next frame).

Start with this disabled and enable once the base solver is stable. It can be added
after warm-starting and accumulated impulse clamping are in place.

### Iteration count

Start with 4–8 iterations. More iterations improve convergence for stacking and resting
stability. Fewer iterations are cheaper. This is a tunable parameter.

**Dependencies:** Phase 6 (warm-start) must have run first. Phases 3–5 must have produced
constraints.

---

## Phase 8: Store solver impulses

**Purpose:** Write the accumulated impulses from the solver back into the contact manifold's
cache for next frame's warm-starting.

For each constraint, copy `accumulated_normal_impulse` and `accumulated_tangent_impulse`
back into the corresponding `ContactPoint` in the manifold.

**Dependencies:** Phase 7 must have completed.

---

## Phase 9: Integrate positions

**Purpose:** Apply corrected velocities to update body positions and orientations.

```
position += linear_velocity * dt
orientation = integrate_orientation(orientation, angular_velocity, dt)
```

**Key detail:** By this point, the solver has already adjusted velocities to prevent
penetration. The position integration simply carries out the corrected motion. This is why
contacts must be generated at current positions (before integration) — the solver prevents
the bad motion rather than reacting to it after the fact.

**Dependencies:** Phase 7 must have completed.

---

## Phase 10: CCD pass

**Purpose:** Prevent tunneling for fast-moving bodies whose per-frame displacement exceeds
their collision geometry size.

### When CCD activates

CCD is **not** the primary collision detection mechanism. It is a safety net for bodies moving
too fast for the narrowphase margin to catch. A body requires CCD when:
```
|linear_velocity| * dt > radius * ccd_threshold
```
Where `ccd_threshold` is typically 0.5 (half the body radius). The narrowphase margin and
CCD threshold must be coordinated so there is no gap:
```
contact_margin >= ccd_threshold * min_radius * (1 / min_expected_fps)
```
In practice, using `contact_margin = 0.02` and `ccd_threshold = 0.5` with radius 0.15 at
60fps gives: narrowphase catches up to `0.02 / 0.0167 = 1.2 m/s` displacement; CCD activates
at `0.15 * 0.5 / 0.0167 = 4.5 m/s`. There is a gap between 1.2 and 4.5 m/s. The preferred
solution is to add **speculative contacts** for fast bodies below the CCD threshold:

- For bodies with `|v| * dt > contact_margin`, expand their AABB along the velocity vector
  and allow the narrowphase to emit contacts with `depth <= 0` (velocity-only correction).
- This keeps discrete contacts primary, closes the gap without forcing full CCD, and avoids
  excessive margins that can cause jitter.

If speculative contacts are not yet implemented, reduce `ccd_threshold` as a stopgap, but
plan to replace the stopgap with speculative contacts later.

### CCD procedure

1. Before phase 9, save all dynamic body positions.
2. After phase 9, for each body flagged for CCD:
   a. Sweep the body's collider from pre-integration position to post-integration position
      against static geometry and other bodies.
   b. If a sweep hit occurs at time `t in [0, 1]`:
      - Move the body to the hit position: `pos = pre_pos + (post_pos - pre_pos) * t`.
      - Generate one or more CCD contact constraints at the hit point(s).
      - **CCD mini-solve:** run a small sequential-impulse solve (2–4 iterations) over the
        CCD constraints for the impacted bodies (or their island). This handles multiple
        hits in a single frame and avoids single-contact overshoot.
      - The body's velocity is now corrected and it will not tunnel.

### CCD vs static geometry

Use `StaticGeometry::sweep_sphere(start, end, radius)` to find the first impact.

### CCD vs dynamic bodies

For pairs where at least one body is flagged for CCD, use `swept_sphere_sphere` to find
the first impact. Process hits in time order per CCD body, then run the CCD mini-solve
for the bodies involved in those hits.

**Dependencies:** Phase 9 must have run. Pre-integration positions must be saved before
phase 9. Speculative contacts (if used) are generated in phase 4 and only require the
expanded AABB.

---

## Phase 11: Sleeping and islands

**Purpose:** Bodies at rest should stop being simulated. This eliminates resting jitter
and improves performance.

### Energy-based sleep criterion

A body is a sleep candidate when its kinetic energy stays below a threshold for N consecutive
frames:
```
kinetic_energy = 0.5 * mass * |v|^2 + 0.5 * omega . (I * omega)
sleep_candidate = kinetic_energy < sleep_threshold for sleep_delay frames
```
Typical values: `sleep_threshold = 0.01`, `sleep_delay = 60` frames (1 second at 60fps).

### Island building

Bodies connected by active contacts or joints form an island. All bodies in an island must
sleep together (one fast body keeps the whole island awake). Implementation:
1. Build a graph where bodies are nodes and active contacts/joints are edges.
2. Find connected components (union-find or BFS).
3. An island sleeps only if all its bodies are sleep candidates.

### Wake-up

A sleeping body wakes (along with its island) when:
- An external force or impulse is applied to it.
- A non-sleeping body enters contact with it (detected via broadphase AABB overlap).
- Its velocity is set externally (e.g., by game code via `set_linear_velocity`).

### Sleeping bodies in the pipeline

Sleeping bodies skip phases 1–2 (integration), are excluded from broadphase pair generation,
and their contacts are not solved. They retain their cached contact manifolds so warm-starting
works immediately upon wake-up.

**Dependencies:** Requires broadphase and contact manifold persistence.

---

## Data structures summary

### PhysicsWorld
```
PhysicsWorld {
    config: PhysicsConfig,
    bodies: Arena<RigidBody>,
    colliders: Arena<Collider>,
    manifolds: ManifoldCache,               // persistent contact manifolds
    broadphase: BroadPhase,                 // AABB acceleration structure
    islands: IslandManager,                 // connected components for sleeping
}
```

### PhysicsConfig
```
PhysicsConfig {
    gravity: Vector3,
    solver_iterations: u32,                 // 4–8
    contact_margin: f32,                    // 0.01–0.05
    ccd_threshold: f32,                     // 0.5
    baumgarte_factor: f32,                  // 0.1–0.2
    baumgarte_slop: f32,                    // 0.001–0.01
    restitution_velocity_threshold: f32,    // 0.5–1.0
    sleep_threshold: f32,                   // 0.01
    sleep_delay_frames: u32,               // 30–60
}
```

### ManifoldCache
```
ManifoldCache {
    manifolds: HashMap<ColliderPairKey, ContactManifold>,
}
```
Indexed by ordered collider pair. Pruned each frame: remove manifolds not refreshed by the
narrowphase for N frames (e.g., 10), or whose colliders have been removed.

---

## Module structure

```
src/physics/
    mod.rs                  (public API re-exports only)
    body.rs                 (RigidBody, RigidBodyDesc, BodyType)
    collider.rs             (Collider, ColliderDesc, ColliderShape, ColliderMaterial)
    handle.rs               (RigidBodyHandle, ColliderHandle)
    math.rs                 (inertia tensors, orientation integration)
    static_geometry.rs      (StaticGeometry trait, StaticContact, SweptStaticContact)
    world.rs                (PhysicsWorld, PhysicsConfig — orchestrates the pipeline)
    broadphase/
        mod.rs
        aabb.rs             (AABB type, overlap test, from-shape computation)
        brute_force.rs      (O(n^2) broadphase — initial implementation)
        bvh.rs              (AABB tree — later implementation)
    narrowphase/
        mod.rs
        manifold.rs         (ContactManifold, ContactPoint, ManifoldCache)
        sphere_sphere.rs    (sphere-sphere overlap + contact generation)
        sphere_static.rs    (sphere-static geometry contact generation)
    collision/
        mod.rs
        swept.rs            (swept sphere-sphere, swept sphere-static)
    solver/
        mod.rs
        sequential_impulse.rs   (the main solver loop)
        constraints.rs          (ContactConstraint, JointConstraint types)
    island.rs               (island building, sleep management)
```

---

## Implementation order

Each step should be a standalone PR that leaves the engine functional and testable.

### Step 1: Immediate solver fixes ✅

Two fixes to the existing solver without changing pipeline structure:
1. **Restitution velocity threshold.** Add the threshold to prevent micro-bouncing at
   resting contacts. Fixes resting contact jitter.
2. **Friction angular impulses.** Friction impulses are currently applied directly to
   linear velocity, bypassing `apply_impulse_at_point()`. This means friction never
   generates torque — bodies slide instead of rolling. Fix by applying friction impulses
   at the contact point so they affect angular velocity. **This unblocks all other work
   by fixing the immediate bugs.**

**Implementation notes:**
- `restitution_velocity_threshold` added to `PhysicsConfig` (default 1.0 m/s). When
  approach speed is below the threshold, restitution is zeroed in `solve_single_contact`.
- `apply_friction()` in `solver.rs` now calls `apply_impulse_at_point()` for both bodies
  instead of modifying linear velocity directly. This generates torque via `r × impulse`.
- **Known limitation:** After a wall collision redirects a ball's velocity, the static
  contact cache can fail to re-acquire the terrain contact, causing balls to fall through.
  This is a fundamental gap in the CCD-primary architecture, fixed by the discrete
  narrowphase in Steps 2–3.

### Step 2: Narrowphase contact generation
Add `narrowphase/` module. Generate contacts at current positions for sphere-static and
sphere-sphere. Wire into `step()` alongside the existing CCD pipeline (both run; the
narrowphase handles resting contacts, CCD handles fast motion). Subtract contact margin
from terrain-reported depth.

### Step 3: Pipeline reorder
Rewrite `step()` to follow the new pipeline order: integrate velocities → narrowphase →
solve → integrate positions → CCD. This replaces the existing TOI-first architecture
(predict positions, sweep all pairs, resolve events in time order, re-sweep after each
collision). The predict/event-loop/re-CCD machinery, manifold cache, static contact cache,
and warm-start cache in their current form are all removed. The narrowphase is now the
primary contact source; CCD is the tunneling safety net.

**Implementation notes:**
- Current implementation uses narrowphase-first contact generation plus a CCD sweep after
  integration. CCD is skipped for bodies already handled by static narrowphase contacts.
- Static contacts currently keep a single strongest triangle contact per sphere to avoid
  conflicting normals (multiple triangle contacts at once cause impulse jitter near edges).
- Contact margin is applied in narrowphase queries, with solver depth clamped to `>= 0`.
- No manifold persistence, warm-starting, or accumulated impulse caching yet; those start
  in Step 4. Expect higher bounce energy and normal jitter until then.

### Step 4: Contact manifold persistence and warm-starting ✅
Add `ManifoldCache`. Store contact points in local space, match across frames, cache
impulses. Implement warm-starting in the solver. This is the single biggest stability
improvement for resting and stacking contacts.

**Implementation notes:**
- `ManifoldCache` lives in `pipeline/manifold.rs`, bridging narrowphase output and solver
  input. Keyed by `(Option<ColliderHandle>, ColliderHandle)` ordered pair (`None` for
  static geometry). Contact points stored in body-local space, matched across frames by
  local-space distance (`contact_match_threshold`, default 0.05). Stale points pruned
  after `manifold_max_age` frames (default 3) without a narrowphase refresh.
- Narrowphase returns raw contacts; the manifold cache merges them with persistent
  data and populates warm-start impulse fields on `ContactConstraint`. CCD contacts
  are transient and bypass the cache (`collider_b: None`).
- `solve()` in `solver.rs` is now the single entry point: warm-start, N iterations,
  position correction (once), return `SolvedImpulses` for writeback. The iteration
  loop moved out of `world.rs` into the solver. `solve_contacts()` remains as a
  low-level function for CCD's one-off transient contacts.
- Friction refactored to use a stable tangent basis (`compute_tangent_basis`) derived
  from the contact normal, replacing the previous velocity-derived tangent direction.
  This ensures tangent impulses from the manifold cache are applied in a consistent
  frame across warm-start and iterative solving.
- **Bug fix:** Position correction (Baumgarte) was previously inside `solve_single_contact`,
  causing it to run `solver_iterations` times per frame instead of once. With
  `correction_factor=0.2` and 4 iterations, bodies received 0.8 effective correction —
  over-correcting penetration, injecting energy via gravity on the next frame, and
  producing visible perpetual bouncing at low energy. Now runs once after iterations.
- **Bug fix:** Warm-start writeback was initialized to zero, only capturing iterative
  impulses. The total impulse (warm-start + iterative) must be written back so the
  cache converges to the correct steady-state value. Without this, cached impulses
  oscillate between correct and near-zero on alternating frames.

### Step 5: Accumulated impulse clamping
Change the solver from per-iteration impulse clamping to accumulated impulse clamping.
This improves convergence and prevents the solver from overshooting on contacts that
are solved multiple times per iteration loop.

### Step 6: Broadphase
Add AABB computation for colliders. Implement brute-force broadphase (loop over all
pairs, test AABB overlap). Replace the current all-pairs sphere check in body-body
narrowphase. This is a performance improvement, not a correctness change.

### Step 7: Speculative contacts (CCD gap closure)
Add speculative contacts for fast bodies below the CCD threshold. Expand AABBs along the
velocity vector, allow narrowphase to emit velocity-only contacts with `depth <= 0`, and
solve them normally. This closes the CCD activation gap without forcing full CCD on
moderate-speed bodies.

### Step 8: CCD mini-solve
When a CCD sweep hits, build CCD constraints and run a small solver pass (2–4 iterations)
for the impacted bodies or island. This reduces artifacts from multiple hits in a single
frame.

### Step 9: Split impulse (post-stabilization)
Add the optional positional correction pass to reduce energy injection from Baumgarte.
Keep it disabled by default; enable once base stability is proven.

### Step 10: Sleeping
Add energy tracking, island building, and sleep/wake logic. Sleeping bodies skip
integration and solving. This eliminates residual micro-jitter and improves performance.

### Step 11: BVH broadphase (optional)
Replace brute-force broadphase with an AABB tree for better scaling to large body counts.

---

## Stability techniques reference

These are the specific techniques that prevent common physics engine bugs. Each is annotated
with which phase implements it.

| Technique | Phase | What it prevents |
|---|---|---|
| Semi-implicit Euler (solve before integrate) | 1–2, 7, 9 | Solver fights forces instead of preventing penetration |
| Contact margin / AABB expansion | 3–4 | Contacts not detected until penetration already happened |
| Speculative contacts (AABB expansion along velocity) | 4 | CCD activation gap for moderate-speed bodies |
| Margin depth subtraction | 4 | Position correction pushing apart non-penetrating contacts |
| Restitution velocity threshold | 7 | Micro-bouncing at resting contacts from gravity |
| Accumulated impulse clamping | 7 | Solver overshoot on multi-iteration convergence |
| Warm-starting | 6, 8 | Solver reconverging from scratch each frame (jitter) |
| Baumgarte position correction | 7 | Accumulated penetration drift over time |
| Split impulse (post-stabilization) | 7, 9 | Energy injection from Baumgarte in tall stacks |
| Baumgarte slop | 7 | Position correction jitter on shallow contacts |
| CCD for fast bodies | 10 | Tunneling through thin geometry |
| CCD mini-solve | 10 | Multiple CCD hits in a single frame |
| Sleeping | 11 | Residual jitter from floating-point noise in solver |
