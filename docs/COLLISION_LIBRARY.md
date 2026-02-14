# Narrowphase Collision Detection Library

Design document for a general, performant narrowphase collision detection library.

## Design aims

1. **Manifold-first.** Narrowphase directly computes a deterministic, feature-aware
   support manifold per pair (after seam filtering), instead of emitting many per-triangle
   contacts and relying on clustering/reduction to approximate a manifold.

2. **General with fast-path specializations.** Every convex shape implements a single
   `ConvexSupport` trait, giving it immediate GJK/EPA support. Special-case routines for
   common pairs (sphere-sphere, sphere-OBB, OBB-OBB) are added as profiling-driven
   optimizations without changing the dispatch interface.

3. **Unified CCD.** Continuous collision detection uses GJK raycast (translation-only) or
   conservative advancement (translation + rotation), both driven by the same support
   function. Analytic fast-paths exist only for trivial cases (sphere-sphere quadratic).

4. **Mesh-aware manifold generation.** Triangle meshes (via `MeshPatch`) are not decomposed
   into per-triangle convex queries. Instead, seam filtering merges adjacent coplanar
   triangles into polygonal contact regions, and the manifold is clipped against those
   regions directly.

5. **Parallelizable.** All narrowphase inputs are read-only during contact generation.
   Manifolds are the only output, collected per-thread and merged. No architectural
   changes needed to distribute across cores.

6. **Cache-friendly.** Shape data is gathered into contiguous work buffers before dispatch.
   Temporary geometry (clipping polygons, merged faces) uses per-thread arena allocators.

7. **Margin-aware.** All collision queries accept an explicit `contact_margin` parameter.
   The collision library inflates shapes by the margin during queries and reports raw
   geometric depth alongside solver-ready depth. The physics pipeline decides what margin
   value to use; the collision library just applies it.

---

## Current state

Collision code is currently split across two locations:

| Location | Contents |
|---|---|
| `src/collision/` | Low-level geometry: `aabb.rs`, `contact.rs`, `shapes.rs`, `mesh_patch.rs`, `sphere_triangle.rs`, `swept.rs` |
| `src/physics/collision/` | Shape-pair contact generators used by the physics pipeline: `sphere_sphere.rs`, `obb.rs`, `obb_sphere.rs`, `obb_triangle.rs`, `obb_obb.rs` |

Additionally, the physics narrowphase layer (`src/physics/narrowphase/`) contains several
contact post-processing components that exist because the current architecture generates
per-triangle contacts and then fixes them up:

| Component | Location | Purpose | New architecture equivalent |
|---|---|---|---|
| `adjacency_filter.rs` | narrowphase | Fix normals on contacts that land on internal mesh edges/vertices of coplanar neighbors | Subsumed by seam filter — internal edges are merged before contact generation |
| `coplanar_stabilizer.rs` | narrowphase | Group coplanar contacts and replace with geometrically stable manifold (OBB face corners, sphere center-to-plane) | Subsumed by `*_patch` routines — manifold is generated directly from merged faces |
| `normal_cluster.rs` | narrowphase | Group contacts by similar normals and spatial proximity, reduce to max N points | Retained as utility for GJK/EPA fallback and edge cases |
| `contact_reducer.rs` | pipeline | Reduce contact count to max N using area-maximizing selection | Retained as utility in collision library |
| `normal_smoothing.rs` | pipeline | Blend cached (previous frame) normals with current normals for temporal stability | Stays in manifold cache layer (physics pipeline) |

The proposed library consolidates all collision geometry into `src/collision/`, with
`src/physics/` consuming it as a dependency. The physics pipeline owns dispatch and
manifold caching; the collision library owns geometry, algorithms, and contact generation.

---

## Core abstraction: ConvexSupport

Every convex shape implements a single trait:

```rust
/// The furthest point on a convex shape in a given direction.
pub trait ConvexSupport {
    /// Returns the point on the shape's surface that is furthest along `direction`
    /// (world space). Used by GJK, EPA, and conservative advancement.
    fn support(&self, direction: Vec3) -> Vec3;

    /// Upper bound on the shape's radius from its local origin.
    /// Used by conservative advancement to bound rotational surface speed.
    fn bounding_radius(&self) -> f32;
}
```

Implementations:

| Shape | `support(d)` | Notes |
|---|---|---|
| Sphere | `center + d.normalize() * radius` | Rotation-invariant: CCD never needs conservative advancement |
| OBB | `center + R * (half_extents * sign(R^T * d))` | Branch-free sign-flip, SIMD-friendly |
| Capsule | `segment_support(d) + d.normalize() * radius` | Segment support + inflate |
| ConvexHull | `vertices[argmax(v · d)]` | Hill-climbing with adjacency for large hulls |
| Triangle | `vertices[argmax(v · d)]` | Trivial 3-way max |

---

## Contact data model

### ContactPoint

Every contact point carries both raw geometric data and solver-ready values:

```rust
pub struct ContactPoint {
    /// World-space contact position.
    pub point: Point3,

    /// Geometric contact normal (directly from collision test, A-to-B).
    pub raw_normal: Vec3,

    /// Solver-facing normal. Initially equals `raw_normal`, but may be
    /// smoothed by the manifold cache for temporal stability.
    pub normal: Vec3,

    /// Geometric penetration depth (positive = overlapping, negative = margin-only contact).
    /// This is the unmodified value from the collision test with margin applied.
    pub raw_depth: f32,

    /// Solver-facing depth, clamped to >= 0. Margin-only contacts (raw_depth < 0)
    /// get depth = 0, meaning velocity-only correction with no position push.
    pub depth: f32,

    /// Identifies the geometric feature pair that produced this contact.
    pub feature_id: FeatureId,
}
```

The `raw_depth` / `depth` and `raw_normal` / `normal` split serves two purposes:

- **Margin contacts:** When `contact_margin` expands a query, contacts within the margin
  skin have `raw_depth < 0` and `depth = 0`. The solver applies velocity correction
  (preventing approach) without position correction (no pushing apart). `raw_depth` is
  preserved so the physics pipeline can gate warm-starting and restitution on the true
  geometric state.

- **Normal smoothing:** The manifold cache may blend `raw_normal` with the previous
  frame's normal for temporal stability (prevents jitter from floating-point noise on
  resting contacts). The collision library always outputs `normal == raw_normal`; the
  smoothing is applied by the physics pipeline's manifold cache layer.

### ContactManifold

```rust
pub struct ContactManifold {
    /// Up to 4 contact points (area-maximizing selection for > 4).
    pub points: SmallVec<[ContactPoint; 4]>,
}
```

### FeatureId

```rust
/// Identifies the geometric feature pair that generated a contact point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FeatureId(pub u64);
```

Feature IDs are constructed from the shape-pair's contact features:

| Pair type | Feature encoding |
|---|---|
| Sphere-sphere | Constant (only one possible contact) |
| Sphere-face | Face index |
| Sphere-edge | Edge index |
| OBB face-face | Face index on A, face index on B |
| OBB edge-edge | Edge index on A, edge index on B |
| Shape-mesh face | Mesh triangle indices composing the merged face |

Feature IDs enable:

- **Warm-starting:** Cached impulses are matched to the same feature next frame via the
  manifold cache, not by local-space distance heuristics.
- **Temporal coherence:** When the feature ID is unchanged, skip full re-clipping and
  update contact points incrementally.
- **Separating axis caching:** For SAT-based pairs, cache the last frame's separating
  axis (indexed by feature ID). Test it first — if it still separates, skip the remaining
  axes.

---

## Shape types and dispatch

### Shape enum

```rust
pub enum ColliderShape {
    Sphere { radius: f32 },
    Box { half_extents: Vec3 },
    Capsule { half_height: f32, radius: f32 },
    ConvexHull { vertices: Arc<Vec<Vec3>>, adjacency: Arc<HullAdjacency> },
}
```

### Dispatch table

A single entry point selects the algorithm for each shape pair. All dispatch functions
accept `contact_margin` so the collision library can inflate shapes consistently:

```rust
pub fn generate_manifold(
    a: &ShapeView,
    b: &ShapeView,
    contact_margin: f32,
) -> ContactManifold {
    match (a.shape(), b.shape()) {
        // --- Analytic fast-paths (tier 1) ---
        (Sphere, Sphere)      => sphere_sphere::manifold(a, b, contact_margin),
        (Sphere, Box)         => sphere_obb::manifold(a, b, contact_margin),
        (Sphere, Capsule)     => sphere_capsule::manifold(a, b, contact_margin),

        // --- SAT fast-paths (tier 2) ---
        (Box, Box)            => obb_obb::manifold(a, b, contact_margin),
        (Box, Capsule)        => obb_capsule::manifold(a, b, contact_margin),
        (Capsule, Capsule)    => capsule_capsule::manifold(a, b, contact_margin),

        // --- GJK/EPA fallback (tier 3) ---
        _                     => gjk_epa::manifold(a, b, contact_margin),
    }
}
```

Mesh contacts dispatch at the patch level (see Mesh-aware manifold generation below):

```rust
pub fn generate_mesh_manifold(
    shape: &ShapeView,
    patch: &FilteredPatch,
    contact_margin: f32,
) -> ContactManifold {
    match shape.shape() {
        Sphere  => sphere_patch::manifold(shape, patch, contact_margin),
        Box     => obb_patch::manifold(shape, patch, contact_margin),
        _       => gjk_patch::manifold(shape, patch, contact_margin),
    }
}
```

### Algorithm tiers

| Tier | Method | Used for |
|---|---|---|
| 1. Analytic | Closed-form distance/overlap | Sphere-sphere, sphere-OBB, sphere-capsule |
| 2. Algebraic | SAT with known axis sets | OBB-OBB (15 axes), OBB-capsule, capsule-capsule |
| 3. Iterative | GJK + EPA + Sutherland-Hodgman | Any convex pair without a specialization |

Each tier is strictly faster than the one below. New shapes get immediate tier-3 support
by implementing `ConvexSupport`, and can be promoted to a higher tier later.

---

## Continuous collision detection (CCD)

CCD has two independent dimensions: **shape pair** and **motion type**.

### Motion type dispatch

```rust
pub fn time_of_impact(a: &ShapeView, b: &ShapeView, motion: &MotionPair) -> Option<TOI> {
    let needs_rotational_ccd =
        motion.has_significant_angular_velocity()
        && has_non_spherical_rotating_shape(a, b);

    if needs_rotational_ccd {
        conservative::advancement(a, b, motion)
    } else {
        match (a.shape(), b.shape()) {
            (Sphere, Sphere) => analytic::sphere_sphere(a, b, motion),
            _                => gjk_raycast::cast(a, b, motion),
        }
    }
}
```

### CCD method summary

| Method | Motion type | Mechanism | Complexity |
|---|---|---|---|
| Analytic | Translation | Closed-form quadratic/linear | O(1) |
| GJK raycast | Translation | Ray cast against Minkowski difference | Single GJK pass |
| Conservative advancement | Translation + rotation | Iterative: `t += distance / closing_speed` | 3-6 GJK iterations typical |

Key insight: a sphere is rotation-invariant, so sphere-vs-anything never needs conservative
advancement even if the sphere is spinning. Conservative advancement is only required when
a non-spherical shape has significant angular velocity.

### GJK raycast

For translation-only CCD, the swept volume of each convex shape is convex — so the problem
reduces to a ray cast against the Minkowski difference in 3D:

```
M = A ⊖ B                           (Minkowski difference, convex)
d = (v_a - v_b) * dt                (relative displacement)
Find smallest t: origin + t*d ∈ M   (ray-vs-convex query via modified GJK)
```

This is a single GJK computation, not an iterative loop.

### Conservative advancement

For rotating bodies, the swept volume is not convex, so an iterative approach is required:

```
t = 0
loop:
    d = gjk_distance(A at t, B at t)
    if d < tolerance: return t
    v_close = relative_closing_speed + omega_bound * bounding_radius
    if v_close <= 0: return None
    t += d / v_close
    if t > 1: return None
```

The `omega_bound * bounding_radius` term upper-bounds the surface speed due to rotation,
making the advancement conservative (more iterations) but correct.

---

## Mesh-aware manifold generation

Triangle mesh contacts (static terrain, any mesh-based geometry) use `MeshPatch` — a
localized fragment of a triangle mesh with adjacency information, produced by broadphase
geometry queries.

### Pipeline

```
Broadphase query → MeshPatch (triangles + adjacency)
       ↓
Seam filter: merge coplanar/smooth adjacent triangles → FilteredPatch (polygonal faces + boundary edges)
       ↓
Per-shape manifold generation against the filtered patch
       ↓
ContactManifold with feature IDs
```

### Seam filtering

The seam filter uses `MeshPatch.neighbors` adjacency to:

1. Identify **internal edges** (shared between two triangles in the patch).
2. Classify internal edges as **smooth** (coplanar or near-coplanar within a tolerance)
   or **crease** (significant dihedral angle).
3. Merge triangles across smooth internal edges into **polygonal contact regions**.
4. Mark **boundary edges** (patch boundary or crease edges) as valid contact features.

This prevents phantom contacts on internal mesh edges while preserving real geometric
edges. It replaces the current `adjacency_filter.rs` approach (which fixes normals
after contact generation) by eliminating internal-edge contacts at their source.

### Data structures

```rust
/// A MeshPatch after seam filtering: merged faces and classified edges.
pub struct FilteredPatch {
    /// Polygonal contact regions formed by merging coplanar triangles.
    pub faces: SmallVec<[ContactFace; 4]>,

    /// Boundary and crease edges that are valid contact features.
    pub boundary_edges: SmallVec<[ContactEdge; 8]>,
}

/// A merged polygonal contact region (one or more coplanar triangles).
pub struct ContactFace {
    /// Vertices of the polygon (wound CCW when viewed from the normal side).
    pub vertices: SmallVec<[Vec3; 6]>,

    /// Outward face normal.
    pub normal: Vec3,

    /// Feature ID for manifold persistence (e.g. hash of constituent triangle indices).
    pub feature_id: FeatureId,
}

/// A boundary or crease edge, valid as a contact feature.
pub struct ContactEdge {
    pub a: Vec3,
    pub b: Vec3,
    pub feature_id: FeatureId,
}
```

### Per-shape manifold generation against filtered patches

Each `*_patch` routine operates on the whole `FilteredPatch`, not individual triangles:

- **Sphere vs patch:** Find the closest point on the merged surface (faces + boundary
  edges). Single contact point. Depth is computed geometrically as
  `radius - dot(center - plane_point, normal)` for stability, rather than from
  per-triangle closest-point distances which fluctuate as edge contacts enter and leave
  the query region. This carries forward the technique from the existing
  `coplanar_stabilizer.rs`.

- **OBB vs patch:** For each merged face, test SAT between the OBB and the face polygon.
  For the contact face with deepest penetration, clip the OBB's support face against the
  merged polygon via Sutherland-Hodgman. Contact points land at the OBB's face corners
  projected onto the contact plane — the same stable geometry the existing
  `coplanar_stabilizer.rs` computes as a post-processing step. Produces a multi-point
  manifold directly.

- **General convex vs patch:** Same structure as OBB, but use GJK for the initial
  separation test and EPA for penetration depth, then clip support faces. Contact
  reduction (`ContactReducer`) may be needed here if clipping produces > 4 points.

---

## Contact reduction and clustering

### ContactReducer

Reduces a set of contacts to at most N points using area-maximizing selection:

1. Pick the deepest contact.
2. Pick the contact farthest from the first.
3. Pick the contact that maximizes triangle area with the first two.
4. Pick the contact that maximizes minimum distance to the selected set.

This is used:
- Inside `obb_obb.rs` after Sutherland-Hodgman clipping (may produce > 4 points).
- Inside `gjk_patch.rs` as a fallback when GJK/EPA + clipping produces excess contacts.
- As a final safety net in any dispatch path that can exceed 4 contacts.

### NormalClusterer

Groups contacts by normal alignment and spatial proximity, then reduces each group
independently. Retained as a utility for cases where manifold-first generation isn't
applicable (e.g., the GJK/EPA fallback path generating contacts from multiple EPA
queries, or future compound collider support where multiple sub-shapes produce
independent contact sets).

```rust
pub struct NormalClusterer {
    max_points: usize,
    normal_cluster_dot: f32,      // default 0.98
    point_cluster_distance: f32,  // default 0.05
    contact_margin: f32,
    reducer: ContactReducer,
}
```

---

## Physics pipeline responsibilities

The following features live in the physics pipeline (`src/physics/`), not in the
collision library. They consume the collision library's output:

### Normal smoothing (manifold cache)

The `NormalSmoother` blends cached (previous frame) normals with current `raw_normal`
values for temporal stability on resting contacts. It operates on manifold cache entries
during the warm-start matching phase:

```rust
if smoother.aligned(cached_normal, contact.raw_normal) {
    contact.normal = smoother.smooth(cached_normal, contact.raw_normal);
} else {
    contact.normal = contact.raw_normal;  // feature changed, don't smooth
}
```

The collision library always outputs `normal == raw_normal`. Smoothing is a
physics-pipeline concern because it depends on frame-to-frame manifold persistence.

### Speculative contacts

For bodies moving faster than the contact margin can catch but below the CCD threshold,
the physics pipeline:

1. Expands the broadphase query region along the velocity vector.
2. Calls the collision library with the expanded region (margin-inflated shapes).
3. Accepts contacts with `raw_depth < 0` (not yet overlapping).
4. The solver applies velocity-only correction (`depth = 0`) to prevent future
   penetration without position push.

The collision library supports this by accepting `contact_margin` and faithfully
reporting negative `raw_depth` for margin-only contacts.

### Warm-start gating

The manifold cache uses `raw_depth` and `raw_normal` to gate warm-start impulse reuse:

- Only reuse cached impulses when normals are aligned (`raw_normal` dot above threshold).
- Suppress warm-start for contacts where the approach velocity exceeds the restitution
  threshold (avoid injecting stale impulses into impact events).
- Use `raw_depth` (not solver `depth`) to distinguish true penetrations from margin contacts
  when deciding whether to apply cached impulses.

---

## Performance optimizations

### 1. Separating axis caching (SAT pairs)

For OBB-OBB and other SAT-based pairs, cache the separating axis from the previous frame.
Test this axis first: if it still separates the shapes, the pair is non-colliding in a
single dot product instead of 15.

```rust
pub struct SATCache {
    /// Last frame's separating axis (world space). None if the pair was colliding.
    pub separating_axis: Option<Vec3>,
}
```

Stable non-colliding pairs (the majority in any scene) become nearly free.

### 2. Temporal manifold coherence

When the contact feature pair hasn't changed between frames (same `FeatureId`), update
contact points by re-projecting from local space rather than running full clipping.
Only re-run the full algorithm when the feature ID changes.

### 3. Data layout: narrowphase work buffer

Before dispatch, gather all shape pair data into a contiguous SoA work buffer:

```rust
pub struct NarrowphaseWorkBuffer {
    pub transforms_a: Vec<Isometry>,
    pub transforms_b: Vec<Isometry>,
    pub shapes_a: Vec<ShapeData>,
    pub shapes_b: Vec<ShapeData>,
    pub pair_indices: Vec<(u32, u32)>,
    pub sat_caches: Vec<Option<SATCache>>,
}
```

This avoids chasing entity/component pointers during the hot narrowphase loop and keeps
data cache-line friendly. The buffer is reused across frames (clear + refill, no
reallocation).

### 4. Per-thread arena allocation

Seam filtering and Sutherland-Hodgman clipping produce temporary polygon vertices. Use a
per-thread bump allocator that resets at frame end:

```rust
pub struct ScratchArena {
    buffer: Vec<u8>,
    offset: usize,
}

impl ScratchArena {
    pub fn alloc_slice<T>(&mut self, count: usize) -> &mut [T];
    pub fn reset(&mut self);
}
```

This eliminates per-pair heap allocation in the narrowphase hot path.

### 5. SIMD for support functions and clipping

The `ConvexSupport::support()` function is the innermost loop for GJK. Key SIMD targets:

- **OBB support:** Sign-flip of half-extents — maps to `_mm_sign_ps` or equivalent.
- **Sutherland-Hodgman clipping:** Each clip plane is a dot product + lerp, SIMD-friendly.
- **GJK simplex operations:** Barycentric coordinates, closest-point-on-simplex.

The architecture keeps these functions small and self-contained so SIMD versions can be
swapped in without touching dispatch logic.

### 6. Parallelization

The pair list is the parallelism boundary. Each pair is independent:

```rust
let manifolds: Vec<ContactManifold> = work_buffer
    .pairs()
    .par_iter()         // rayon or custom thread pool
    .filter_map(|pair| dispatch(pair))
    .collect();
```

All inputs (transforms, shapes, mesh patches) are immutable during narrowphase.
Manifolds are collected per-thread and merged. Per-thread scratch arenas avoid
allocator contention.

---

## Proposed module structure

```
src/collision/
    mod.rs                      // Re-exports only

    // ── Core types ──
    shapes.rs                   // ColliderShape enum, ShapeData, ShapeView
    support.rs                  // ConvexSupport trait + per-shape implementations
    contact.rs                  // ContactManifold, ContactPoint, FeatureId
    aabb.rs                     // AABB type, overlap, from-shape computation
    mesh_patch.rs               // MeshPatch, PatchTriangle (exists)

    // ── Discrete contact generation ──
    discrete/
        mod.rs
        sphere_sphere.rs        // Analytic (tier 1)
        sphere_obb.rs           // Analytic (tier 1)
        sphere_capsule.rs       // Analytic (tier 1)
        obb_obb.rs              // SAT (tier 2)
        obb_capsule.rs          // SAT (tier 2)
        capsule_capsule.rs      // Closest-point-between-segments (tier 2)
        gjk.rs                  // GJK closest-point / intersection (tier 3)
        epa.rs                  // EPA penetration depth + normal (tier 3)
        clipping.rs             // Sutherland-Hodgman face clipping (tier 3)

    // ── Mesh contact generation (manifold-first) ──
    mesh/
        mod.rs
        seam_filter.rs          // MeshPatch → FilteredPatch
        sphere_patch.rs         // Sphere vs filtered patch → manifold
        obb_patch.rs            // OBB vs filtered patch → manifold
        gjk_patch.rs            // General convex vs filtered patch → manifold

    // ── Continuous collision detection ──
    continuous/
        mod.rs
        analytic.rs             // Sphere-sphere (quadratic), sphere-plane (linear)
        gjk_raycast.rs          // Translation-only, any convex pair
        conservative.rs         // Translation + rotation, any convex pair

    // ── Shared utilities ──
    sat.rs                      // Separating axis test helpers, SATCache
    contact_reducer.rs          // Area-maximizing contact reduction to N points
    normal_cluster.rs           // Group by normal + reduce (fallback/utility)
    scratch_arena.rs            // Per-thread bump allocator
    work_buffer.rs              // NarrowphaseWorkBuffer (SoA pair data)
    segment.rs                  // Segment-segment closest points, point-segment distance
```

The physics pipeline (`src/physics/`) consumes this library:

```
src/physics/
    narrowphase/
        mod.rs                  // Dispatch loop: iterates pairs, calls src/collision/
        static_contacts.rs      // Dynamic-vs-static: queries StaticGeometry → MeshPatch → src/collision/mesh/
        dynamic_contacts.rs     // Dynamic-vs-dynamic: broadphase pairs → src/collision/discrete/
    pipeline/
        manifold.rs             // ManifoldCache: persistence, warm-start impulse storage, feature matching
        normal_smoothing.rs     // NormalSmoother: temporal normal blending (operates on manifold cache)
```

---

## Implementation plan

The new collision library is built alongside the existing code. Each step produces a
compiling, testable result. New code is wired into the live physics pipeline as soon
as each piece is ready, replacing old code one pair/path at a time rather than in a
single big-bang cutover. This means bugs surface immediately in the running engine.

### Step 1: Core types and utilities

**Goal:** Establish the type foundation that every subsequent step depends on.

**New files:**
- `src/collision/contact.rs` — rewrite with `ContactPoint` (dual depth/normal fields),
  `ContactManifold`, `FeatureId`
- `src/collision/shapes.rs` — rewrite with `ShapeView` struct (transform + shape ref)
- `src/collision/support.rs` — `ConvexSupport` trait + impls for Sphere, OBB, Triangle
- `src/collision/segment.rs` — `segment_segment_closest_points`,
  `point_segment_distance_sq` (extracted from existing `obb_triangle.rs`)
- `src/collision/contact_reducer.rs` — port from `pipeline/contact_reducer.rs`, operate
  on new `ContactPoint` type

**Modified files:**
- `src/collision/aabb.rs` — keep as-is (already clean)
- `src/collision/mesh_patch.rs` — keep as-is

**Tests:**
- `ConvexSupport::support` correctness for each shape (known directions → expected points)
- `FeatureId` equality/hashing
- `ContactReducer` reduces to N with area-maximizing selection
- Segment utilities match existing `obb_triangle.rs` test cases

**Dependencies:** None.

---

### Step 2: Discrete convex-convex pair tests

**Goal:** Rewrite all shape-pair tests from scratch using new types, with
`contact_margin` support and `FeatureId`s. The existing OBB-OBB and OBB-triangle
implementations have known edge-case bugs and will not be ported — they are
rewritten clean.

**New files:**
- `src/collision/discrete/mod.rs`
- `src/collision/discrete/sphere_sphere.rs` — port from `physics/collision/sphere_sphere.rs`,
  add margin, add `FeatureId`
- `src/collision/discrete/sphere_obb.rs` — port from `physics/collision/obb_sphere.rs`,
  add margin, add `FeatureId`, handle inside-box case (existing code already does this)
- `src/collision/discrete/obb_obb.rs` — **rewrite from scratch.** 15-axis SAT with
  proper edge-edge contact generation. Known bugs in existing implementation around
  edge contacts: contact point selection falls back to heuristics that miss valid
  edge-edge contacts or produce incorrect normals. New implementation must:
  - Correctly identify edge-edge minimum-penetration axes
  - Generate contact points at the closest-point between the responsible edge pair
  - Produce face-face vs edge-edge `FeatureId` encoding
  - Reduce to 4 contacts via `ContactReducer`
- `src/collision/discrete/clipping.rs` — Sutherland-Hodgman `clip_polygon` and
  `face_vertices` as reusable utilities (also needed by `obb_patch.rs` later)
- `src/collision/sat.rs` — SAT overlap test helpers, `SATCache` struct

**Tests:**
- All existing tests from `physics/collision/` adapted for new types (these must still
  pass — they cover the non-buggy cases)
- **New edge-case tests for OBB-OBB:**
  - Edge-edge contact: two OBBs rotated so only edges overlap (no face-face contact).
    Verify contact point lies on both edges, normal is perpendicular to both edge dirs
  - Grazing edge: OBB corner barely touching another OBB's edge. Verify contact is
    detected with correct depth
  - Axis-aligned stacking: two identical axis-aligned boxes stacked → 4 face-face
    contacts, not edge artifacts
- Margin tests: shapes separated by less than margin produce contacts with
  `raw_depth < 0`, `depth == 0`
- `FeatureId` stability: same configuration produces same ID across calls

**Dependencies:** Step 1.

---

### Step 2w: Wire dynamic-dynamic contacts

**Goal:** Replace the shape-pair dispatch in `dynamic_contacts.rs` with calls to the
new collision library. The broadphase (sort-and-sweep) stays untouched.

**Modified files:**
- `src/physics/narrowphase/dynamic_contacts.rs` — in the shape-pair match block,
  replace:
  - `(Sphere, Sphere)` → call `collision::discrete::sphere_sphere`
  - `(Sphere, Box)` → call `collision::discrete::sphere_obb`
  - `(Box, Box)` → call `collision::discrete::obb_obb`
  - Convert returned `ContactManifold` → `ContactConstraint` for the solver
- `src/physics/pipeline/manifold.rs` — accept `FeatureId` for contact matching
  (alongside existing local-space distance matching, so both old and new contacts
  work during the transition)

**Deleted files:**
- `src/physics/collision/sphere_sphere.rs`
- `src/physics/collision/obb_sphere.rs`
- `src/physics/collision/obb_obb.rs`
- `src/physics/collision/obb.rs` — the `Obb` struct moves to `src/collision/`
  (it's now part of the collision library's shape definitions)

**Verification:**
- Run the engine. Spawn spheres and boxes. Verify dynamic-dynamic collisions work:
  balls bouncing off each other, boxes colliding, sphere-box interactions
- The OBB edge bugs from the old implementation should be visibly fixed

**Dependencies:** Step 2.

---

### Step 3: Seam filter and mesh pipeline

**Goal:** Implement the manifold-first mesh contact pipeline. This is the biggest
behavioral change from the existing architecture.

**New files:**
- `src/collision/mesh/mod.rs`
- `src/collision/mesh/seam_filter.rs` — `MeshPatch` → `FilteredPatch`. Flood-fill
  through `PatchTriangle.neighbors`, classify edges as smooth/crease by dihedral angle,
  merge coplanar triangles into `ContactFace` polygons, collect boundary edges
- `src/collision/mesh/sphere_patch.rs` — sphere vs `FilteredPatch`. Find closest point
  on merged surface (faces + boundary edges). Compute depth geometrically as
  `radius - dot(center - face_point, face_normal)`. Single contact with `FeatureId`
  from the face's constituent triangle indices
- `src/collision/mesh/obb_patch.rs` — OBB vs `FilteredPatch`. **Rewrite from scratch**
  (not ported from `obb_triangle.rs` which has the same edge bugs as `obb_obb.rs`).
  For each merged face: SAT test, then clip OBB support face against the polygon.
  Contact points are OBB face corners projected onto the contact plane. Uses
  `clipping.rs` from Step 2. Reduce to 4 points via `ContactReducer`

**Tests:**
- Seam filter unit tests: flat quad (2 triangles) → 1 merged face, no boundary edges
  between them. L-shaped surface → 2 faces joined at crease edge
- Sphere-patch: sphere resting on flat quad produces single contact with face normal
  (not edge normal from the internal diagonal). Compare depth with direct geometric
  calculation
- OBB-patch: box resting on flat quad produces 4 corner contacts. Box on an L-shaped
  crease produces contacts on the appropriate face(s)
- OBB-patch edge cases: box balanced on a terrain ridge (crease edge), box straddling
  a step between two height levels
- Regression: replicate key scenarios from existing `adjacency_filter` and
  `coplanar_stabilizer` tests — verify same or better behavior

**Dependencies:** Steps 1, 2 (for clipping utilities and `Obb` type).

---

### Step 3w: Wire static contacts

**Goal:** Replace the static contact pipeline with the new mesh-aware manifold
generation.

**Modified files:**
- `src/physics/narrowphase/static_contacts.rs` — replace per-triangle contact
  generation + adjacency filter + coplanar stabilizer with:
  `query_region → seam_filter → generate_mesh_manifold`. Convert `ContactManifold`
  to `ContactConstraint` for the solver. Both sphere-static and box-static paths
  are replaced in this step

**Deleted files:**
- `src/physics/collision/obb_triangle.rs`
- `src/physics/collision/mod.rs` (entire `physics/collision/` module is now gone)
- `src/physics/narrowphase/adjacency_filter.rs`
- `src/physics/narrowphase/coplanar_stabilizer.rs`
- `src/physics/narrowphase/contact_source.rs` (replaced by `FeatureId`)
- `src/collision/sphere_triangle.rs` (subsumed by `mesh/sphere_patch.rs`)

**Verification:**
- Run the engine. Spheres and boxes on terrain must rest stably, roll correctly,
  and not exhibit internal-edge artifacts
- Walk the player (kinematic) on terrain — contacts still generated correctly
- Boxes on slopes and steps — the manifold-first pipeline should handle these
  better than the old per-triangle + post-process pipeline

**Dependencies:** Step 3.

---

### Step 4: Analytic CCD

**Goal:** Port the existing analytic swept tests into the new library structure
and wire them in.

**New files:**
- `src/collision/continuous/mod.rs`
- `src/collision/continuous/analytic.rs` — port `swept_sphere_sphere` (quadratic) from
  existing `physics/collision/sphere_sphere.rs`. Port `swept_sphere_triangle`
  (plane + edge + vertex tests) from existing `src/collision/swept.rs`. Add
  `sphere_plane` as a standalone fast-path

**Tests:**
- Port all existing `swept_sphere_sphere` and `swept_sphere_triangle` tests
- Edge cases: zero relative velocity → None. Already overlapping → Some(0.0)

**Dependencies:** Step 1.

---

### Step 4w: Wire CCD

**Goal:** Replace CCD calls in the physics pipeline with the new continuous module.

**Modified files:**
- `src/physics/` CCD code paths — replace calls to old `swept_sphere_sphere` and
  `swept_sphere_triangle` with `collision::continuous::analytic`

**Deleted files:**
- `src/collision/swept.rs` (subsumed by `continuous/analytic.rs`)

**Verification:**
- Fast-moving spheres don't tunnel through terrain or each other

**Dependencies:** Step 4.

---

### Step 5: Cleanup

**Goal:** Remove all remaining old collision code and verify a clean build.

**Deleted files (anything left over):**
- `src/collision/shapes.rs` (old `Sphere` struct, replaced by new `shapes.rs` in Step 1)
- `src/collision/contact.rs` (old `ContactPoint`, replaced in Step 1)
- Any remaining dead code in `src/physics/narrowphase/` that was only used by the
  old pipeline

**Modified files:**
- `src/collision/mod.rs` — clean up re-exports, remove references to deleted modules

**Verification:**
- `cargo build` and `cargo test` clean with no dead-code warnings from old collision
  modules
- Full regression pass: all physics scenarios working

**Dependencies:** Steps 2w, 3w, 4w all complete.

---

### Step 6: GJK, EPA, and general CCD

**Goal:** Add the general-purpose fallback algorithms. These are not needed for the
current shape set (sphere + box) but are required before adding capsules, convex hulls,
or any new shape.

**New files:**
- `src/collision/discrete/gjk.rs` — GJK algorithm. Operates on `ConvexSupport` trait.
  Returns closest points + distance (separated case) or simplex (intersecting case).
  Voronoi simplex solver with 1/2/3/4-simplex cases
- `src/collision/discrete/epa.rs` — EPA algorithm. Takes intersecting simplex from GJK,
  expands polytope to find penetration depth + normal. Returns contact normal, depth,
  witness points
- `src/collision/mesh/gjk_patch.rs` — general convex vs `FilteredPatch`. Uses GJK
  for separation test per merged face, EPA for penetration, then clip support faces.
  `NormalClusterer` as safety net if multiple faces produce contacts
- `src/collision/continuous/gjk_raycast.rs` — GJK raycast against Minkowski difference.
  Translation-only, any convex pair via `ConvexSupport`. Modified GJK that advances a
  ray origin rather than testing intersection
- `src/collision/continuous/conservative.rs` — conservative advancement for
  translation + rotation. Uses GJK distance query. Angular velocity bound
  via `ConvexSupport::bounding_radius()`
- `src/collision/dispatch.rs` — the three top-level dispatch functions
  (`generate_manifold`, `generate_mesh_manifold`, `time_of_impact`) with shape-pair
  matching, symmetric pair handling, and fallback-to-GJK for unspecialized pairs

**Modified files:**
- `src/collision/normal_cluster.rs` — port from `physics/narrowphase/normal_cluster.rs`,
  operate on new `ContactPoint` type
- `src/collision/mod.rs` — re-export dispatch functions

**Tests:**
- GJK: two separated spheres → correct distance. Two overlapping OBBs → reports
  intersection. Sphere vs OBB → matches analytic `sphere_obb` results
- EPA: overlapping OBBs → penetration depth and normal match SAT results from
  `obb_obb.rs`. Sphere inside OBB → correct depth
- GJK-patch: convex hull vs flat terrain patch → contact manifold. Verify against
  known OBB-patch results when using a box-shaped hull
- GJK raycast: two OBBs on collision course → correct TOI. Sphere vs OBB → matches
  analytic result. No-hit for diverging pairs
- Conservative advancement: rotating OBB vs static plane → correct TOI. Sphere
  (should skip conservative path) → same result as GJK raycast
- Dispatch routing: verify symmetric pairs produce consistent results

**Dependencies:** Steps 1-5 (the library is live and working; GJK/EPA slot in as the
fallback tier).

---

### Step 7: Performance optimizations

**Goal:** Add performance features that aren't required for correctness.

These can be done independently in any order:

**7a. SAT caching**
- Add `SATCache` storage per collider pair (in manifold cache or work buffer)
- In `obb_obb.rs` and `sat.rs`, test cached axis first, early-out if still separating
- Benchmark: measure pair-test cost for stable non-colliding OBB pairs

**7b. Work buffer (SoA layout)**
- `src/collision/work_buffer.rs` — gather shape pair data into contiguous arrays
  before dispatch. Physics pipeline fills the buffer; collision library iterates it
- Benchmark: measure cache-miss reduction on 100+ body scenes

**7c. Scratch arena**
- `src/collision/scratch_arena.rs` — per-thread bump allocator
- Wire into `clipping.rs` and `seam_filter.rs` for temporary polygon storage
- Benchmark: measure allocation overhead reduction

**7d. Parallelization**
- Add rayon `par_iter` over pair list in dispatch loop
- Per-thread scratch arenas, collect manifolds per-thread and merge
- Benchmark: measure scaling on 200+ body scenes

**Dependencies:** Steps 1-5 (working pipeline to benchmark against).

---

### Implementation order summary

```
Step 1: Core types ─────────────────────────────────────────┐
    │                                                        │
    ├── Step 2: Discrete pair tests (rewrite OBB from scratch)
    │       │                                                │
    │       └── Step 2w: Wire dynamic-dynamic ◄── LIVE ──── │
    │                                                        │
    ├── Step 3: Seam filter + mesh pipeline                  │
    │       │                                                │
    │       └── Step 3w: Wire static contacts ◄── LIVE ──── │
    │                                                        │
    ├── Step 4: Analytic CCD                                 │
    │       │                                                │
    │       └── Step 4w: Wire CCD ◄── LIVE ──────────────── │
    │                                                        │
    └── Step 5: Cleanup (delete old code) ───────────────────┘
                    │
        Step 6: GJK/EPA + general CCD (enables new shapes)
                    │
        Step 7a-d: Performance (independent, any order)
```

Each "w" (wire) step puts new code into the live engine immediately after its
corresponding library step. Bugs are caught in context, not months later during
a big-bang cutover. Steps 2/3/4 can be worked on in any order after Step 1; each
has its own wire step that can be done independently.

GJK/EPA (Step 6) is deferred to the end because the current shape set (sphere + box)
is fully covered by analytic and SAT specializations. GJK/EPA becomes necessary only
when adding capsules, convex hulls, or other new shapes.

### Final implementation notes
Step 1 has a naming collision. The new contact.rs and shapes.rs replace files that already exist with the same names. During the transition (Steps 1 through 5), both old and new versions need to coexist. Options:
- Put the new types in a submodule (e.g. src/collision/types/contact.rs) and rename at cleanup
- Or just rename the old files first (e.g. contact_legacy.rs) before creating the new ones

Pick an approach in Step 1 rather than discovering the conflict mid-implementation.

The Obb struct needs to move early. Step 2w deletes physics/collision/obb.rs but the Obb type is used by the solver, narrowphase, CCD, and spawners — not just collision. Move Obb into src/collision/ (or
src/collision/shapes.rs) in Step 1, re-export it, and update all use paths before anything else touches it. Otherwise Step 2w becomes a tangled diff.