# GJK/EPA and General Convex Support

> **Deprecated.** A historical design document, kept for the reasoning behind
> it; the code has moved on and parts of it no longer describe what exists —
> speculative contacts in particular, now in `physics/narrowphase/speculative.rs`
> and described in `COLLISION_LIBRARY.md`. Read the code, not this, for how
> things work today.

Design document for adding GJK, EPA, general-purpose CCD, centralized dispatch,
and the `ConvexHull` shape variant. Corresponds to Step 6 from `COLLISION_LIBRARY.md`.

---

## Goals

1. Any convex shape that implements `ConvexSupport` gets immediate collision support
   (discrete + CCD + mesh) without writing shape-pair-specific code.
2. Centralize shape-pair dispatch so adding a new shape is a single-point change.
3. Add the `ConvexHull` collider shape as the first consumer of the general path.
4. Maintain full compatibility with existing analytic/SAT fast-paths for
   Sphere, Box, and Capsule.

## Decisions

- **Centralized dispatch** over inline match blocks. The existing match-per-file
  approach (dynamic_contacts, static_contacts, sweep_clamp) duplicates routing
  logic across three locations. A `dispatch.rs` module will own the shape-pair →
  algorithm mapping. The narrowphase files will call through it. Performance is
  unaffected (the dispatch is a match statement the compiler can inline).

- **Face clipping for multi-point manifolds.** GJK/EPA's witness points identify the
  penetration axis and depth, but a single contact point is insufficient for stable
  resting contact (a box on a hull would wobble). After EPA identifies the contact
  normal, we extract support faces from both shapes, clip them via Sutherland-Hodgman
  (reusing the existing `clipping.rs`), and project clipped vertices onto the contact
  plane. This produces 1-4 contact points per manifold, matching the existing pipeline.

- **Convex hulls capped at 64 vertices.** Enforced at construction time. This bounds
  EPA polytope growth and keeps brute-force support function scans fast. Pre-computed
  simplified hulls are expected — runtime convex hull computation is not in scope.

- **No ultra-thin shapes.** A minimum thickness constraint is enforced: the thinnest
  dimension of any convex hull must be > 1% of the largest. This avoids degenerate
  Minkowski differences that cause GJK/EPA numerical issues.

- **EPA iteration cap with graceful degradation.** EPA is capped at 64 iterations. If
  it doesn't converge, the best result so far is returned. For near-touching shapes the
  contact will be slightly imprecise for one frame and refined via manifold persistence.

- **No compound colliders.** Each rigid body gets one convex collider. Compound
  (multi-shape) colliders are a separate feature.

- **Brute-force convex hull support.** For hulls under 64 vertices, scanning all
  vertices for the max dot product is fast enough. Hill-climbing with adjacency
  tables is deferred.

## Deferred work

These items are explicitly out of scope for this implementation. Each is a viable
follow-up once the general path is working and profiled.

### D1: Conservative advancement (rotational CCD)

Translation-only CCD (GJK raycast) handles the vast majority of CCD cases.
Conservative advancement adds iterative GJK distance queries bounded by
`omega * bounding_radius` to handle fast-spinning non-spherical shapes.
**When to revisit:** when gameplay introduces fast-rotating convex hulls that
tunnel through geometry.

### D2: ConvexHull shape variant — DONE (Step 6.7)

Adding `ConvexHull` to `ColliderShape` is the primary consumer of the
GJK/EPA path. Implemented as Step 6.7 including mass/inertia computation,
ECS spawnable wiring, and dispatch integration.

### D3: Hill-climbing support with adjacency

For hulls with many vertices, the brute-force `O(n)` support scan becomes a
bottleneck inside GJK's inner loop. Hill-climbing uses a precomputed adjacency
graph to find the support point in `O(sqrt(n))` amortized time by walking from
the previous frame's support vertex. **When to revisit:** when profiling shows
support function cost dominating GJK for hulls with > 32 vertices.

### D4: GJK warm-starting (cache seeding) — DONE (Step 6.4)

Implemented as **direction seeding** (not full simplex caching):
- Per-pair `GjkCache` stores last direction.
- `gjk_query_seeded()` seeds GJK's initial search direction with that cached vector.
- `gjk_epa_manifold_cached()` threads the cache through the full pipeline.
- Dynamic narrowphase owns and prunes `GjkCacheMap` (parallel to `SatCacheMap`).

Future extension: full simplex caching on top of direction seeding if profiling
shows GJK iteration count is still a bottleneck.

### D5: EPA vertex welding / merge threshold

When EPA expands the polytope, nearly-coincident vertices from floating-point
imprecision can create degenerate (zero-area) triangles. A merge threshold that
welds vertices closer than epsilon would improve robustness for shapes with
nearly-parallel faces. **When to revisit:** if EPA produces visibly wrong normals
or depths for specific hull geometries.

### D6: NormalClusterer port

The `NormalClusterer` groups contacts by normal direction and reduces each group
independently. Currently unused — `ContactReducer` (greedy spread maximization)
handles all reduction needs. **When to revisit:** if gjk_patch produces contacts
from multiple merged faces with divergent normals that confuse the reducer.

### D7: Parallelized narrowphase dispatch

Investigated in Step 7d of the collision library plan. Rayon `par_iter` over
narrowphase pairs regressed performance for current shape pairs (analytic/SAT are
too cheap at 100-500ns per pair). GJK/EPA pairs will be heavier (estimated 1-5 μs),
so parallelization may break even at 200+ GJK pairs. **When to revisit:** after
GJK/EPA is live, profile a scene with 200+ convex hull pairs.

### D8: Dispatch-level CCD centralization

The CCD path (`sweep_clamp.rs`) currently inlines its own shape dispatch for
contact generation at the hit point. A centralized `dispatch::time_of_impact`
function would unify this with the narrowphase dispatch. Deferred because CCD
dispatch is simpler than narrowphase dispatch (fewer shape combinations matter)
and the current approach works.

---

## Architecture

### Step 6.1 dispatch refactor: detailed design

The dispatch refactor is the most integration-heavy step. This section
specifies exactly how existing code restructures.

#### Current helper function pattern

Each shape pair in `dynamic_contacts.rs` has a dedicated helper function
(e.g. `sphere_sphere_pair`, `box_box_pair`) that bundles three concerns:

1. **Shape construction** — builds concrete collision shapes (`Obb::new()`,
   `Capsule::new()`) from `ColliderState` fields (center, rotation, half_extents).
2. **Collision call** — calls the collision library function
   (e.g. `sphere_obb_manifold(&obb, center, radius, margin)`).
3. **Result wrapping** — creates `PairHeader` with body handles and combined
   material, pushes `PairManifold` to output buffer if non-empty.

Concern (3) is already factored into shared utilities `make_pair_header()`
and `push_if_nonempty()`. Concerns (1) and (2) are interleaved — the helper
takes raw `ColliderState` fields, constructs shapes, and immediately calls the
collision function.

`static_contacts.rs` has the same pattern: `sphere_vs_static()`,
`box_vs_static()`, `capsule_vs_static()` each construct a shape, query
`StaticGeometry`, run `filter_patch()`, and call the shape-specific patch
manifold function.

#### Where each concern lives after refactoring

- **Shape construction** moves into `ShapeView::from_collider_state()` /
  `ShapeView::from_collider()`. This is a method on `ShapeView` that takes
  (center, rotation, shape) and stores them. The concrete `Obb`/`Capsule`
  objects are constructed lazily inside the dispatch functions when needed —
  but crucially, the *narrowphase code* no longer needs to know which
  concrete type to build.

- **Collision call** moves into `dispatch.rs`. The dispatch functions match
  on `ColliderShape` variants and construct the concrete types internally:

  ```rust
  pub fn generate_manifold(
      a: &ShapeView,
      b: &ShapeView,
      margin: f32,
      sat_cache: Option<&mut SatCache>,
  ) -> ContactManifold {
      match (a.shape, b.shape) {
          (Sphere { radius: ra }, Sphere { radius: rb }) => {
              sphere_sphere_manifold(a.center, *ra, b.center, *rb, margin)
          }
          (Sphere { radius }, Box { half_extents }) => {
              let obb = Obb::new(b.center, b.rotation, *half_extents);
              sphere_obb_manifold(&obb, a.center, *radius, margin)
          }
          (Box { half_extents }, Sphere { radius }) => {
              let obb = Obb::new(a.center, a.rotation, *half_extents);
              sphere_obb_manifold(&obb, b.center, *radius, margin)
          }
          (Box { half_extents: he_a }, Box { half_extents: he_b }) => {
              let obb_a = Obb::new(a.center, a.rotation, *he_a);
              let obb_b = Obb::new(b.center, b.rotation, *he_b);
              match sat_cache {
                  Some(cache) => obb_obb_manifold_cached(&obb_a, &obb_b, margin, cache),
                  None => obb_obb_manifold_cached(&obb_a, &obb_b, margin, &mut SatCache::default()),
              }
          }
          // ... sphere-capsule, box-capsule, capsule-capsule arms ...
          _ => gjk_epa_manifold(a, b, margin),  // added in Step 6.4
      }
  }
  ```

- **Result wrapping** stays in the narrowphase. After dispatch returns a
  `ContactManifold`, the narrowphase wraps it with `PairHeader` exactly
  as it does today (via `make_pair_header` / `push_if_nonempty`).

#### SAT cache threading

The SAT cache is OBB-OBB-specific. The dispatch function accepts it as
`Option<&mut SatCache>`:
- The narrowphase passes `Some(&mut cache)` when both shapes are boxes.
  The `SatCacheMap` lookup (by `SatPairKey`) stays in `dynamic_contacts.rs`,
  which owns the cache map.
- All other pairs pass `None`. The dispatch function ignores it for non-OBB
  arms.
- The `active_box_pairs` tracking and `sat_cache_map.prune()` call stay in
  `dynamic_contacts.rs`.

This keeps the cache ownership in the narrowphase (where it belongs —
it's per-pair state tied to collider handles) while letting dispatch use it
for the OBB-OBB fast-path.

#### Speculative contacts and CCD checks

Speculative contacts stay in the narrowphase. The current flow is:

```
1. Call collision function (dispatch)
2. If empty AND speed criteria met → run speculative prediction
3. Speculative: predict future position → call collision again → clamp depths
```

After the refactor:
```
1. Call dispatch::generate_manifold()
2. If empty AND speed criteria met → call dispatch::generate_manifold() again
   with predicted positions
3. Clamp depths via make_speculative()
```

The speculative logic (speed gates, predicted position computation, depth
clamping) is physics-pipeline policy, not collision-library concern. It stays
in `dynamic_contacts.rs` and `static_contacts.rs`.

The sphere-sphere speculative path in `dynamic_contacts.rs` is special: it
uses `swept_sphere_sphere` CCD rather than re-running the manifold at a
predicted position. This stays as a special case in the narrowphase. The
dispatch layer doesn't know about CCD or speculative contacts.

#### ShapeView

```rust
/// World-space snapshot of a convex shape for collision dispatch.
///
/// Constructed from ColliderState (dynamic pairs) or from Collider + body
/// transform (static pairs). The dispatch layer reads the shape enum to
/// select the algorithm; GJK/EPA reads center/rotation/shape to evaluate
/// the support function.
pub struct ShapeView<'a> {
    pub center: Point3<f32>,
    pub rotation: UnitQuaternion<f32>,
    pub shape: &'a ColliderShape,
}
```

`ShapeView` implements `ConvexSupport` by matching on the shape enum and
delegating to the per-shape implementations:

```rust
impl ConvexSupport for ShapeView<'_> {
    fn support(&self, direction: Vector3<f32>) -> Point3<f32> {
        match self.shape {
            ColliderShape::Sphere { radius } => {
                SupportSphere { center: self.center, radius: *radius }.support(direction)
            }
            ColliderShape::Box { half_extents } => {
                Obb::new(self.center, self.rotation, *half_extents).support(direction)
            }
            ColliderShape::Capsule { half_height, radius } => {
                Capsule::new(self.center, self.rotation, *half_height, *radius)
                    .support(direction)
            }
            ColliderShape::ConvexHull { .. } => {
                // added in Step 6.7
            }
        }
    }

    fn bounding_radius(&self) -> f32 {
        self.shape.bounding_radius()
    }
}
```

This is the bridge between the physics-layer shape enum and GJK/EPA's
trait-based algorithms. The fast-path dispatch arms never call `support()` —
they construct `Obb`/`Capsule` directly and call the specialized functions.
`ConvexSupport` is only used when falling through to the GJK/EPA wildcard.

#### Narrowphase after refactoring

The main pair loop in `dynamic_contacts.rs` becomes:

```rust
for pair_idx in 0..buf.pairs.len() {
    let (i, j) = buf.pairs[pair_idx];
    let si = &buf.states[i];
    let sj = &buf.states[j];

    if si.body_handle == sj.body_handle {
        continue;
    }

    let view_a = ShapeView { center: si.center, rotation: si.rotation, shape: &si.shape };
    let view_b = ShapeView { center: sj.center, rotation: sj.rotation, shape: &sj.shape };

    // SAT cache: only look up for Box-Box pairs.
    let mut sat_cache_opt = match (&si.shape, &sj.shape) {
        (ColliderShape::Box { .. }, ColliderShape::Box { .. }) => {
            buf.active_box_pairs.insert((si.collider_handle, sj.collider_handle));
            let key = SatPairKey::new(si.collider_handle, sj.collider_handle);
            Some(sat_cache_map.caches.entry(key).or_default())
        }
        _ => None,
    };

    let manifold = dispatch::generate_manifold(
        &view_a, &view_b, contact_margin,
        sat_cache_opt.as_deref_mut(),
    );
    push_if_nonempty(&mut buf.manifolds, make_pair_header(si, sj), manifold);

    // Speculative contacts for sphere-sphere pairs (existing special case).
    // ... stays here, unchanged ...
}
```

The ~140 lines of per-pair match arms and 8 helper functions collapse into
the above. `make_pair_header()` and `push_if_nonempty()` are unchanged.

`static_contacts.rs` similarly simplifies: the per-shape match in the main
loop becomes a `dispatch::generate_mesh_manifold()` call. The AABB query
region computation needs the shape's world-space AABB, which we add as a
method on `ShapeView`:

```rust
impl ShapeView<'_> {
    /// Compute the world-space AABB of this shape, expanded by margin.
    pub fn query_aabb(&self, margin: f32) -> AABB {
        match self.shape {
            ColliderShape::Sphere { radius } => { /* center ± (radius + margin) */ }
            ColliderShape::Box { half_extents } => {
                let obb = Obb::new(self.center, self.rotation, *half_extents);
                let (min, max) = obb.enclosing_aabb();
                /* expand by margin */
            }
            ColliderShape::Capsule { half_height, radius } => {
                let capsule = Capsule::new(self.center, self.rotation, *half_height, *radius);
                let (min, max) = capsule.enclosing_aabb();
                /* expand by margin */
            }
            _ => { /* fallback: center ± (bounding_radius + margin) */ }
        }
    }
}
```

#### Mesh dispatch

```rust
pub fn generate_mesh_manifold(
    shape: &ShapeView,
    patch: &FilteredPatch,
    margin: f32,
) -> ContactManifold {
    match shape.shape {
        ColliderShape::Sphere { radius } => {
            sphere_patch_manifold(shape.center, *radius, patch, margin)
        }
        ColliderShape::Box { half_extents } => {
            let obb = Obb::new(shape.center, shape.rotation, *half_extents);
            obb_patch_manifold(&obb, patch, margin)
        }
        ColliderShape::Capsule { half_height, radius } => {
            let capsule = Capsule::new(shape.center, shape.rotation, *half_height, *radius);
            let (seg_a, seg_b) = capsule.segment_endpoints();
            capsule_patch_manifold(seg_a, seg_b, *radius, patch, margin)
        }
        _ => gjk_patch_manifold(shape, patch, margin),  // added in Step 6.5
    }
}
```

The `static_contacts.rs` main loop becomes:

```rust
let view = ShapeView { center, rotation, shape: collider.shape() };
let query = view.query_aabb(contact_margin);
let patch = static_geometry.query_region(&query);
let filtered = filter_patch(&patch, SEAM_FILTER_COPLANAR_DOT);
let manifold = dispatch::generate_mesh_manifold(&view, &filtered, contact_margin);
```

The `speculative_static_manifold()` function uses the same pattern with a
predicted center.

### Margin handling in GJK/EPA

The existing collision functions handle margin by inflating shapes before
testing and then reporting raw geometric depth relative to the uninflated
shape:

```
expanded_radius = radius + contact_margin
test with expanded_radius
raw_depth = (radius_a + radius_b) - distance    // can be negative
```

For GJK/EPA, we use **Minkowski sum inflation** — the standard approach.
Instead of modifying each shape's geometry, we inflate the support function
by the margin. This works because the Minkowski sum of a convex shape with
a sphere of radius `m` is equivalent to expanding the support function by `m`:

```rust
fn inflated_support(shape: &dyn ConvexSupport, direction: Vector3<f32>, margin: f32) -> Point3<f32> {
    let p = shape.support(direction);
    let len = direction.magnitude();
    if len < 1e-10 { return p; }
    p + direction * (margin / len)
}
```

In practice, for GJK/EPA we inflate both shapes by `contact_margin` and
run the algorithms on the inflated Minkowski difference. The reported
penetration depth from EPA is then the inflated depth; raw geometric depth
is `epa_depth - 2 * contact_margin`. This matches the convention used by
the existing analytic/SAT functions.

For GJK distance queries (separated case), the distance between uninflated
shapes is `gjk_distance - 2 * contact_margin`. If this is negative but
`gjk_distance > 0`, the shapes are in the margin zone: `raw_depth < 0`,
`depth = 0`.

### FeatureId generation for GJK/EPA contacts

Existing fast-paths generate stable `FeatureId`s from geometric features:
- Sphere-sphere: `FeatureId::SINGLE` (only one possible contact)
- OBB face-face: `FeatureId::from_face_pair(ref_face_idx, inc_face_idx)` —
  face indices 0-5 encoding axis + sign
- OBB edge-edge: `FeatureId::from_edge_pair(edge_a, edge_b)` — 12 edges per OBB

The manifold cache matches contacts between frames by `FeatureId` to reuse
warm-start impulses. Unstable or random IDs cause warm-start failure, which
means jittery settling.

For GJK/EPA contacts, we construct `FeatureId`s from the **support face
indices** used during face clipping:

```rust
// Face-face contact (both shapes have support faces):
// Encode which face of A and which face of B are in contact.
FeatureId::from_face_pair(face_index_a, face_index_b)

// With per-contact-point uniqueness via vertex index:
base_feature_id.with_vertex(clip_vertex_index)
```

Where `face_index_a` / `face_index_b` come from the support face extraction
(see below). For OBBs, the face index is `axis * 2 + sign` (0-5), matching
the SAT fast-path's encoding exactly. For convex hulls, the face index is
the index into the hull's face array.

For the single-contact fallback (when one or both shapes have no face —
e.g. sphere), we use `FeatureId::SINGLE`, matching the sphere-sphere
convention.

This approach ensures:
- **Temporal stability:** Same resting configuration → same face indices →
  same `FeatureId` → warm-start reuse.
- **Compatibility:** When GJK/EPA handles an OBB-OBB pair (shouldn't happen
  in practice, but could in testing), the face indices match what the SAT
  path would produce.
- **Simplicity:** No need for EPA-specific feature encoding. The face clip
  stage produces the same kind of features as the SAT path.

### Support face extraction (SupportFace trait)

The face clipping step after EPA needs to extract the "support face" — the
polygonal face of a convex shape most aligned with a given direction. This
is a new trait method added to the collision library:

```rust
/// Polygonal face of a convex shape, used for contact manifold clipping.
pub struct SupportFace {
    /// Vertices of the face polygon (CCW winding from outside).
    pub vertices: SmallVec<[Point3<f32>; 8]>,
    /// Outward face normal.
    pub normal: Vector3<f32>,
    /// Face index for FeatureId construction.
    pub face_index: u32,
}

/// Trait for shapes that can extract a support face for manifold clipping.
///
/// Not all ConvexSupport shapes have meaningful faces (spheres don't).
/// Shapes without faces return None, and the manifold generator falls
/// back to a single-point contact from the EPA witness point.
pub trait SupportFaceExtractor {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace>;
}
```

This is a **separate trait**, not a method on `ConvexSupport`. Rationale:
`ConvexSupport` is a minimal trait (one function) that any convex shape can
implement. Support face extraction requires face topology knowledge that
not all shapes have (spheres, point clouds). Keeping them separate means
GJK/EPA work with any `ConvexSupport` impl, while face clipping is an
optional enhancement for shapes that have face data.

Per-shape implementations:

**Sphere:** Returns `None`. No face to clip — the manifold generator uses
the EPA witness point directly (single contact).

**OBB:** Delegates to existing `obb_face()` from `clipping.rs`. Finds the
face axis with the largest dot product with the direction (same logic as
`find_most_aligned_face` in `obb_obb.rs`). Returns 4 vertices + normal.
Face index: `axis * 2 + sign` (0-5).

```rust
impl SupportFaceExtractor for Obb {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        let axes = self.axes();
        let (axis_idx, sign) = find_most_aligned_face(&axes, direction);
        let face = obb_face(self, axis_idx, sign);
        let face_index = (axis_idx as u32) * 2 + if sign > 0.0 { 0 } else { 1 };
        Some(SupportFace {
            vertices: SmallVec::from_slice(&face.vertices),
            normal: face.normal,
            face_index,
        })
    }
}
```

**Capsule:** Returns `None`. Capsules are smooth — no flat face to clip.
The manifold uses 1-2 contacts from segment endpoint projection.
(Alternatively, could return a 2-vertex "face" from the segment endpoints,
but this adds complexity for minimal benefit. Capsule-vs-flat contacts are
already well-handled by the capsule fast-path.)

**ConvexHull (Step 6.7):** Iterates the hull's face array, finds the face
with the largest `normal · direction` dot product. Returns the face's
vertices. Face index: index into the hull's face array.

```rust
impl SupportFaceExtractor for ConvexHull {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        let (best_idx, face) = self.faces.iter().enumerate()
            .max_by(|(_, a), (_, b)|
                a.normal.dot(&direction).total_cmp(&b.normal.dot(&direction))
            )?;
        Some(SupportFace {
            vertices: face.vertices.clone(),
            normal: face.normal,
            face_index: best_idx as u32,
        })
    }
}
```

**ShapeView:** Delegates to the concrete shape:

```rust
impl SupportFaceExtractor for ShapeView<'_> {
    fn support_face(&self, direction: Vector3<f32>) -> Option<SupportFace> {
        match self.shape {
            ColliderShape::Sphere { .. } => None,
            ColliderShape::Box { half_extents } => {
                Obb::new(self.center, self.rotation, *half_extents)
                    .support_face(direction)
            }
            ColliderShape::Capsule { .. } => None,
            ColliderShape::ConvexHull { .. } => { /* Step 6.7 */ }
        }
    }
}
```

### GJK/EPA manifold generation: detailed flow

This is the function the dispatch wildcard arm calls. It orchestrates
GJK → EPA → face extraction → clipping → contact point construction.

```rust
pub fn gjk_epa_manifold(
    a: &ShapeView,
    b: &ShapeView,
    margin: f32,
) -> ContactManifold {
    // 1. Run GJK on margin-inflated shapes.
    let result = gjk_query(a, b, margin);

    match result {
        GjkResult::Separated { distance, closest_a, closest_b } => {
            // Shapes are separated even with margin — no contact.
            if distance > 2.0 * margin + GJK_TOLERANCE {
                return ContactManifold::empty();
            }
            // Within margin zone: speculative contact.
            let normal = (closest_b - closest_a).normalize();
            let raw_depth = -distance + 2.0 * margin;  // negative (margin-only)
            let point = Point3::from((closest_a.coords + closest_b.coords) * 0.5);
            return ContactManifold::single(
                ContactPoint::new(point, normal, raw_depth, FeatureId::SINGLE)
            );
        }
        GjkResult::Intersecting { simplex } => {
            // 2. Run EPA to find penetration depth + normal.
            let epa = epa_penetration(a, b, margin, simplex);

            // 3. Try face clipping for multi-point manifold.
            // Normal points A→B, so A's face faces toward B (+normal)
            // and B's face faces toward A (-normal).
            let face_a = a.support_face(epa.normal);
            let face_b = b.support_face(-epa.normal);

            match (face_a, face_b) {
                (Some(fa), Some(fb)) => {
                    // 4. Full face-face clipping (OBB-OBB, OBB-Hull, Hull-Hull).
                    clip_face_face_manifold(&fa, &fb, epa.normal, epa.depth, margin)
                }
                (Some(fa), None) | (None, Some(fa)) => {
                    // One shape has a face, other doesn't (e.g. sphere-OBB).
                    // Project the face-less shape's witness point onto the face.
                    single_face_manifold(&fa, epa.normal, epa.depth,
                                        epa.witness_a, epa.witness_b, margin)
                }
                (None, None) => {
                    // Both shapes are faceless (e.g. sphere-sphere, sphere-capsule).
                    // Single contact from EPA witness points.
                    let point = Point3::from(
                        (epa.witness_a.coords + epa.witness_b.coords) * 0.5
                    );
                    let raw_depth = epa.depth - 2.0 * margin;
                    ContactManifold::single(
                        ContactPoint::new(point, epa.normal, raw_depth, FeatureId::SINGLE)
                    )
                }
            }
        }
    }
}
```

The `clip_face_face_manifold` function follows the same pattern as the
existing `face_face_contacts` in `obb_obb.rs`:

1. Choose reference face (larger area or deeper penetration) and incident face.
2. Clip incident face vertices against reference face side planes via
   Sutherland-Hodgman (reuse `clip_polygon` from `clipping.rs`).
3. For each clipped vertex, compute signed distance to reference face plane.
4. Accept vertices behind the plane (penetrating) or within margin tolerance.
5. Construct `ContactPoint` with `raw_depth = -signed_dist`,
   `FeatureId::from_face_pair(ref_face_index, inc_face_index).with_vertex(i)`.
6. Reduce to 4 contacts via `ContactReducer` if needed.

### Dispatch module

`src/collision/dispatch.rs` owns two entry points:

```
generate_manifold(
    a: &ShapeView,
    b: &ShapeView,
    margin: f32,
    sat_cache: Option<&mut SatCache>,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold
generate_mesh_manifold(shape: &ShapeView, patch: &FilteredPatch, margin: f32) -> ContactManifold
```

Shape-pair matching uses analytic/SAT fast-paths for known pairs and falls
back to GJK/EPA for anything else. See detailed function bodies above.

The SAT cache for OBB-OBB pairs passes through dispatch as `Option<&mut SatCache>`.
The GJK warm-start cache passes as `Option<&mut GjkCache>` and is consumed only
by the wildcard GJK/EPA fallback arm.

### GJK algorithm

`src/collision/discrete/gjk.rs`

Gilbert-Johnson-Keerthi distance algorithm operating on `ConvexSupport`.

**Input:** Two convex shapes (via trait), initial search direction.
**Output:** `GjkResult` — either `Separated { closest_a, closest_b, distance }`
or `Intersecting { simplex }`.

**Algorithm:**
1. Pick initial support point on Minkowski difference `A ⊖ B`.
2. Build simplex iteratively (point → line → triangle → tetrahedron).
3. Each iteration: find the closest feature of the simplex to the origin
   (Voronoi region test). If the origin is inside the simplex, report
   intersection. Otherwise, compute new search direction toward origin
   and sample a new support point.
4. Terminate when the new support point doesn't make progress
   (dot product with search direction ≤ current best + epsilon).
5. Maximum 32 iterations with early-out.

**Voronoi simplex solver:** handles 1-simplex (point), 2-simplex (line segment),
3-simplex (triangle), and 4-simplex (tetrahedron) cases. Each case computes
barycentric coordinates and the closest point to the origin, then reduces the
simplex by discarding vertices not contributing to the closest feature.

### EPA algorithm

`src/collision/discrete/epa.rs`

Expanding Polytope Algorithm for penetration depth and normal.

**Input:** Intersecting simplex (tetrahedron) from GJK, two `ConvexSupport` shapes.
**Output:** `EpaResult { normal, depth, witness_a, witness_b }`.

**Algorithm:**
1. Initialize polytope from GJK simplex:
   - Fast path: if GJK already provides a valid enclosing tetrahedron, use it.
   - Fallback: construct/select a tetrahedron from robust support samples.
2. Find the face closest to the origin (minimum distance along its normal).
3. Sample a new support point in the direction of that face's normal.
4. If the new point doesn't extend beyond the face (within tolerance),
   the face normal and distance are the penetration normal and depth.
5. Otherwise, remove all faces visible from the new point, create new
   faces connecting the silhouette edges to the new point, repeat.
6. Cap at 64 iterations; return best result so far if not converged.

**Polytope representation:** Flat array of triangular faces, each storing:
- Three vertex indices into a vertex buffer
- Face normal (precomputed)
- Distance from origin to face plane

No half-edge structure — faces are stored as `(v0, v1, v2)` tuples with
adjacency reconstructed during silhouette computation. This is simpler and
sufficient for the bounded vertex counts we support.

**Witness points:** Tracked by maintaining barycentric coordinates of the
closest point on the Minkowski-difference polytope face, then mapping back
to points on shapes A and B using the per-vertex support point history.

### GJK/EPA manifold generation (face clipping)

After EPA returns a penetration normal and witness points, we need to produce
a multi-point contact manifold for solver stability.

**Algorithm:**
1. EPA gives penetration normal `n`, depth `d`, witness points.
2. Extract support face of shape A along `-n` (the face most opposed to the
   penetration direction).
3. Extract support face of shape B along `+n`.
4. Clip face A's polygon against face B's side planes using
   Sutherland-Hodgman (reuse `clipping.rs`).
5. Project clipped vertices onto the contact plane
   (midplane between the two support faces, offset by penetration depth).
6. Each clipped vertex becomes a `ContactPoint` with the EPA normal,
   projected depth, and a `FeatureId` derived from the vertex indices.
7. Reduce to 4 contacts via `ContactReducer` if clipping produced more.

**Support face extraction:** For each shape type:
- **Sphere:** No face — single contact point (the witness point).
- **OBB:** The face whose outward normal best aligns with the query direction.
  Returns 4 vertices. (Reuse existing `obb_face()` from `clipping.rs`.)
- **Capsule:** The two hemisphere endpoints define a "face" — but capsules
  are smooth, so the contact degenerates to 1-2 points (segment endpoints
  projected onto the contact plane).
- **ConvexHull:** Find the face whose normal has the largest dot product
  with the query direction. Return its vertices.

This means sphere-vs-anything through GJK/EPA always produces 1 contact,
OBB-vs-OBB through GJK/EPA produces up to 4 (matching the SAT fast-path),
and hull-vs-anything produces up to 4.

### GJK raycast (translation-only CCD)

`src/collision/continuous/gjk_raycast.rs`

**Input:** Two `ConvexSupport` shapes, relative displacement vector `d` over the timestep.
**Output:** `Option<GjkRaycastHit { t, normal, point }>` where `t ∈ [0, 1]`.

**Algorithm (Minkoski-space ray cast):**
1. The swept collision of A moving by `d` relative to B is equivalent to:
   does the ray `origin + t*d` intersect the Minkowski difference `A ⊖ B`?
2. Modified GJK: instead of testing the origin, test a point advancing along
   the ray. Maintain `t` (fraction along ray) and `normal` (last separating
   plane normal).
3. Each iteration: compute support point on `A ⊖ B` in the search direction.
   If the support point is behind the ray origin at current `t`, advance `t`
   to the plane defined by the support point.
4. Terminate when the simplex encloses the ray point (hit) or the ray exits
   (no hit).
5. Maximum 32 iterations.

### gjk_patch (general convex vs filtered mesh patch)

`src/collision/mesh/gjk_patch.rs`

**Input:** Any `S: ConvexSupport + SupportFaceExtractor`, `FilteredPatch` (merged mesh faces).
**Output:** `ContactManifold` with up to 4 contacts.

**Algorithm (direct face-normal projection):**
1. For each `ContactFace` in the filtered patch:
   a. Backface check: reject if the shape's highest support point along the
      face normal is behind the face plane.
   b. Depth check: find the shape's deepest support point along `-normal`.
      If the signed distance to the face plane exceeds the margin, skip.
   c. For shallow contacts (near-touching), emit a single projected point.
   d. For deeper contacts: extract the shape's support face along `-normal`,
      clip it against the mesh face edges via Sutherland-Hodgman, and project
      clipped vertices onto the face plane.
2. Collect all contacts from all faces, reduce to 4 via `ContactReducer`.

The original design planned GJK/EPA per mesh face. The implementation uses
direct face-normal projection instead: the contact normal for one-sided
terrain is always the face normal, and penetration depth is the projection
of the shape's deepest support point onto the face plane. This avoids GJK's
numerical instability when a small convex shape collides with a large mesh
polygon (the Minkowski difference has extreme aspect ratio, causing simplex
refinement failures).

---

## Implementation plan

### Step 6.1: Dispatch refactor

**Goal:** Centralize shape-pair routing. No new algorithms — just restructure
the existing code so that adding GJK/EPA later is a single fallback arm.
See "Step 6.1 dispatch refactor: detailed design" in the Architecture section
for the full specification.

**New files:**
- `src/collision/dispatch.rs` — `generate_manifold()` and `generate_mesh_manifold()`.
  Routes to existing specialized functions based on shape-pair matching.
  Constructs concrete types (`Obb`, `Capsule`) internally from `ShapeView`
  fields. Accepts `Option<&mut SatCache>` for OBB-OBB pairs.
- `src/collision/shape_view.rs` — `ShapeView` struct with `query_aabb()` method
  and `ConvexSupport` impl. `SupportFaceExtractor` trait and `SupportFace`
  struct (face extraction used later by GJK/EPA manifold clipping).

**Modified files:**
- `src/collision/mod.rs` — add `pub mod dispatch`, `pub mod shape_view`,
  re-export `ShapeView`.
- `src/collision/support.rs` — remove `#[allow(unused)]` from `ConvexSupport`
  trait (it's now used by `ShapeView`).
- `src/physics/narrowphase/dynamic_contacts.rs`:
  - Delete per-pair helper functions: `sphere_sphere_pair`, `sphere_box_pair`,
    `box_sphere_pair`, `box_box_pair`, `sphere_capsule_pair`,
    `box_capsule_pair`, `capsule_capsule_pair` (~160 lines).
  - Replace the shape-pair match block with: construct two `ShapeView`s,
    look up SAT cache for Box-Box pairs, call `dispatch::generate_manifold()`,
    wrap result with `push_if_nonempty(make_pair_header(...), manifold)`.
  - Keep `sphere_sphere_pair`'s speculative CCD path as a special case
    (it uses `swept_sphere_sphere` which isn't a dispatch concern).
  - Keep `make_pair_header()`, `push_if_nonempty()`, `should_add_speculative()`
    unchanged.
  - Keep `SatCacheMap` and its `prune()` logic unchanged.
- `src/physics/narrowphase/static_contacts.rs`:
  - Delete per-shape helpers: `sphere_vs_static`, `box_vs_static`,
    `capsule_vs_static` (~60 lines).
  - Replace the shape match with: construct `ShapeView`, call
    `view.query_aabb(margin)`, query static geometry, filter patch, call
    `dispatch::generate_mesh_manifold()`.
  - Rewrite `speculative_static_manifold()` to use `ShapeView` + dispatch
    instead of per-shape match.
  - Keep `make_speculative()`, `is_speculative_candidate()` unchanged.

**Not modified:**
- `src/physics/ccd/sweep_clamp.rs` — CCD dispatch stays inline for now (D8).
- All existing collision functions — they're called through dispatch, not changed.

**Testing plan:**
- `cargo test` — all existing unit tests must pass unchanged.
- `cargo test --features bench_harness` — all bench harness tests must pass.
  These are integration tests that exercise the full narrowphase pipeline
  (sphere/box/capsule vs static and dynamic). If dispatch routing is wrong,
  these will fail.
- **New unit tests in `dispatch.rs`:**
  - `dispatch_sphere_sphere`: verify `generate_manifold()` for two overlapping
    spheres produces the same `ContactManifold` as calling
    `sphere_sphere_manifold` directly (bitwise equal).
  - `dispatch_sphere_obb`: same approach with sphere + box.
  - `dispatch_obb_obb_with_sat_cache`: verify OBB-OBB dispatch with a
    `SatCache` produces the same result as `obb_obb_manifold_cached` directly,
    and that the cache is updated.
  - `dispatch_symmetric`: verify `generate_manifold(sphere, box)` and
    `generate_manifold(box, sphere)` produce consistent results (same normal
    direction convention, same depth).
  - `dispatch_mesh_sphere`: verify `generate_mesh_manifold()` for a sphere
    produces the same result as `sphere_patch_manifold` directly.
  - `dispatch_mesh_obb`: same for OBB.
- Manual: `cargo run` — spawn spheres and boxes in test_arena level. Verify
  collisions behave identically to before the refactor.

**Status: COMPLETE** (commit `1564efa`)

---

### Step 6.2: GJK

**Goal:** Implement the GJK distance/intersection algorithm.

**New files:**
- `src/collision/discrete/gjk.rs` — `gjk_distance()` function, `GjkResult` enum,
  Voronoi simplex solver. Operates purely on `ConvexSupport` trait objects.

**Modified files:**
- `src/collision/discrete/mod.rs` — add `pub mod gjk`

**Testing plan (unit tests in `gjk.rs`):**
- **Separated spheres:** Two `SupportSphere`s with known separation. Verify
  `GjkResult::Separated` with correct distance (±0.01) and closest points
  on each sphere surface.
- **Overlapping spheres:** Two overlapping `SupportSphere`s. Verify
  `GjkResult::Intersecting` with a valid tetrahedron simplex containing
  the origin.
- **Separated OBBs:** Two axis-aligned `Obb`s with a known gap. Verify
  distance matches `gap_size` (±0.01).
- **Overlapping OBBs:** Two overlapping `Obb`s. Verify intersection detected.
- **Sphere vs OBB:** Cross-validate against `sphere_obb_manifold` — for a
  separated sphere-OBB pair, GJK's distance should match the analytic result.
- **Capsule vs OBB:** Separated capsule near an OBB. Verify reasonable distance.
- **Degenerate: coincident shapes.** Two spheres at the same center. Must not
  loop forever — should report intersection.
- **Degenerate: zero-radius sphere.** Point vs OBB. Verify correct distance.
- **Iteration cap:** Shapes that cause slow convergence (long thin OBB vs
  distant point). Verify terminates within 32 iterations.

**Status: COMPLETE** (commit `f029d8c`)

**Implementation notes:**
- Function is `gjk_query()` (not `gjk_distance()` as originally planned) — returns
  `GjkResult::Separated` or `GjkResult::Intersecting`.
- Uses distance-aware GJK variant: tracks closest point `v` on the simplex,
  convergence test is `v·v - v·w ≤ ε * max(v·v, 1)`.
- Voronoi simplex solver uses Ericson's method (Real-Time Collision Detection §5.1.5).
- `MinkowskiVertex` tracks support points on both shapes for witness reconstruction.
- GJK may return `Intersecting` with fewer than 4 simplex vertices (early exit
  when `|v| ≈ 0`). EPA's `ensure_tetrahedron` handles expansion.
- 11 unit tests covering all planned cases plus rotated OBBs and barely-touching.

---

### Step 6.3: EPA

**Goal:** Implement the EPA penetration depth algorithm with face clipping
for multi-point manifold generation. See "GJK/EPA manifold generation:
detailed flow", "Margin handling in GJK/EPA", "FeatureId generation for
GJK/EPA contacts", and "Support face extraction" in the Architecture section.

**New files:**
- `src/collision/discrete/epa.rs` — `epa_penetration()` function, `EpaResult`
  struct, polytope expansion logic. Takes GJK's intersecting simplex. Inflates
  support function by margin (see "Margin handling in GJK/EPA").
- `src/collision/discrete/gjk_epa_manifold.rs` — `gjk_epa_manifold()` function
  that orchestrates GJK → EPA → face extraction → Sutherland-Hodgman clipping →
  contact point generation. See "GJK/EPA manifold generation: detailed flow"
  for the complete function structure. This is what the dispatch wildcard
  arm calls.

**Modified files:**
- `src/collision/discrete/mod.rs` — add `pub mod epa`, `pub mod gjk_epa_manifold`
- `src/collision/shape_view.rs` — add `SupportFaceExtractor` impls for `Obb`
  and `ShapeView`. See "Support face extraction" for per-shape behavior.
  `SupportFace` struct provides vertices, normal, and `face_index` for
  `FeatureId` construction (see "FeatureId generation for GJK/EPA contacts").

**Testing plan (unit tests):**

*EPA core (`epa.rs`):*
- **Overlapping spheres:** Known overlap. Verify penetration depth matches
  `r1 + r2 - distance` (±0.02) and normal points from A to B.
- **Overlapping OBBs (face-face):** Two axis-aligned OBBs overlapping on one axis.
  Verify depth and normal match the SAT result from `obb_obb_manifold`.
- **Overlapping OBBs (edge-edge):** Two OBBs rotated so edges cross. Verify
  normal is perpendicular to both edges.
- **Deep overlap:** Sphere centered inside an OBB. Verify reasonable depth and
  normal (smallest escape direction).
- **Barely overlapping:** Overlap of 0.001 units. Verify EPA converges (doesn't
  return the iteration-cap fallback).
- **Iteration cap:** Craft a case with many nearly-coplanar faces. Verify
  graceful termination and result within 2x of expected depth.

*GJK+EPA manifold (`gjk_epa_manifold.rs`):*
- **OBB-OBB cross-validation:** For configurations also testable with
  `obb_obb_manifold`, verify the GJK/EPA path produces:
  - Same normal direction (dot > 0.99)
  - Same depth (±0.05)
  - Same number of contact points (within ±1)
  - Contact points within 0.1 of the SAT result positions
- **Sphere-OBB cross-validation:** Same approach, compare against
  `sphere_obb_manifold`.
- **OBB face-face clipping:** Two OBBs in clear face-face contact (parallel
  faces). Verify 4 contact points at the intersection corners.
- **Capsule-OBB:** Capsule lying on an OBB face. Verify 2 contact points
  (hemisphere projections).

*Bench harness tests (`tests/dynamic_pairs.rs`):*
- **New test: gjk_epa_matches_analytic.** Run `SphereSphereCollisionScenario`
  but force the GJK/EPA path (by temporarily wrapping spheres as ConvexHull
  with sphere-approximating vertices, or by adding a test-only dispatch override).
  Verify the scenario still passes — momentum transfer, settling, etc.

**Status: COMPLETE** (commit `f029d8c`)

**Implementation notes:**
- EPA's `ensure_tetrahedron` first checks if the GJK simplex is already a valid
  origin-enclosing tetrahedron (fast path). If not, it gathers a diverse set of
  Minkowski support points (axis-aligned + diagonal directions + simplex vertices,
  ~14 candidates) and exhaustively searches all 4-tuples for the largest-volume
  tetrahedron enclosing the origin. The O(n⁴) search is negligible for n ≤ 14.
  Falls back to the largest-volume tetrahedron overall if none encloses the origin.
- EPA convergence on spheres is loose (~10% error) due to polytope approximation
  of curved surfaces. Acceptable since spheres use the analytic fast-path in
  practice.
- `gjk_epa_manifold` orients the EPA normal deterministically using witness point
  ordering (`witness_b - witness_a`), with `midpoint_of_supports` fallback for
  degenerate witness separation (e.g. coincident shapes).
- `SupportFaceExtractor` implemented for `Obb` (delegates to `obb_face()` from
  clipping.rs) and `ShapeView`. Sphere/Capsule return `None` → single-point
  contact fallback.
- Face clipping reuses `clip_polygon` from `clipping.rs` via Sutherland-Hodgman
  against reference face side planes, reduced to 4 contacts via `ContactReducer`.
- `FeatureId` uses `from_face_pair(face_a, face_b).with_vertex(i)` for clipped
  contacts, matching the SAT path's convention.
- Bench harness test (`gjk_epa_matches_analytic`) deferred — requires ConvexHull
  shape variant (Step 6.7) or test-only dispatch override.
- 6 EPA tests + 5 manifold tests (including OBB-OBB and sphere-OBB cross-validation,
  coincident sphere deterministic normal hemisphere).

---

### Step 6.4: Wire into narrowphase

**Goal:** Connect GJK/EPA as the fallback in the dispatch module, so that
any shape pair without a specialization gets automatic collision support.

**Modified files:**
- `src/collision/dispatch.rs` — add the wildcard `_ => gjk_epa_manifold(a, b, margin)`
  arm to `generate_manifold()`.

**Testing plan:**
- All existing `cargo test` and `cargo test --features bench_harness` pass
  (no behavioral change for existing shape pairs — they still use fast-paths).
- **New unit test in `dispatch.rs`:** Call `generate_manifold()` with two OBBs.
  Verify it routes to `obb_obb` (not GJK/EPA) by checking that the result
  matches the direct `obb_obb_manifold` call exactly (same `FeatureId`s).
- At this point, adding a new shape variant to `ColliderShape` would
  automatically get GJK/EPA collision support through the wildcard arm.

**Status: COMPLETE** (commit `f029d8c`)

**Implementation notes:**
- Added `#[allow(unreachable_patterns)]` wildcard `_ => gjk_epa_manifold(a, b, margin)`
  arm to `generate_manifold()`. Currently unreachable since all 3 shape variants
  have dedicated arms; activates when `ConvexHull` is added.
- `generate_mesh_manifold()` wildcard deferred to Step 6.5 (needs `gjk_patch`).
- New test `dispatch_obb_obb_uses_sat_not_gjk` verifies fast-path routing by
  checking FeatureId equality with direct `obb_obb_manifold_cached` call.

The following items are implemented and may differ slightly from the original
plan text below:

- **GJK warm-start cache is live (direction seeding).**
  - `gjk.rs` now has `GjkCache` + `gjk_query_seeded(...)`.
  - `gjk_epa_manifold.rs` has `gjk_epa_manifold_cached(...)`.
  - `dispatch::generate_manifold(...)` accepts optional `GjkCache`.
  - Dynamic narrowphase owns and prunes `GjkCacheMap` (parallel to `SatCacheMap`).
- **Normal orientation determinism was tightened in GJK/EPA manifolds.**
  - Intersecting manifold normals are oriented using EPA witness ordering,
    with deterministic fallback for degenerate witness separation.
- **Face clipping path was optimized.**
  - Stack-backed buffers + double-buffer clipping reduce transient allocations.
- **EPA tetrahedron init now has a fast path.**
  - If GJK already returns a valid enclosing tetrahedron, EPA uses it directly.
  - Otherwise, EPA falls back to robust support-based tetra selection.

---

### Step 6.5: gjk_patch (general convex vs mesh)

**Goal:** Any convex shape can collide with static terrain meshes.

**New files:**
- `src/collision/mesh/gjk_patch.rs` — `gjk_patch_manifold()` using the
  GJK + EPA + face clipping pipeline against `FilteredPatch` faces.

**Modified files:**
- `src/collision/mesh/mod.rs` — add `pub mod gjk_patch`
- `src/collision/dispatch.rs` — add the wildcard arm to
  `generate_mesh_manifold()`.
- `src/physics/narrowphase/static_contacts.rs` — the dispatch call already
  handles this (from Step 6.1), but verify the wildcard path works.

**Testing plan (unit tests in `gjk_patch.rs`):**
- **OBB cross-validation:** Create an `Obb` and a flat `FilteredPatch`.
  Compare `gjk_patch_manifold` results against `obb_patch_manifold`:
  - Normal direction (dot > 0.99)
  - Depth (±0.05)
  - Contact count (within ±1)
  - Contact positions (within 0.1)
- **Capsule cross-validation:** Same approach vs `capsule_patch_manifold`.
- **Sphere cross-validation:** Same approach vs `sphere_patch_manifold`.
- **Ramp geometry:** Convex shape on a sloped `FilteredPatch`. Verify normal
  aligns with slope and depth is physically reasonable.
- **Step geometry:** Shape straddling two height levels. Verify contacts on
  the correct face(s).

*Bench harness tests:*
- **New scenario: `ConvexOnFlatScenario`.** An OBB (used as a stand-in for a
  general convex shape) dropped onto flat terrain, forced through the gjk_patch
  path. Assert settling time and final rest height match `FlatBoxRestScenario`
  within tolerance.

**Status: COMPLETE**

**Implementation notes:**
- `gjk_patch_manifold()` is generic over `S: ConvexSupport + SupportFaceExtractor`,
  not `ShapeView`-specific, so future shapes automatically get mesh support.
- Uses direct face-normal projection instead of GJK/EPA per face (see gjk_patch
  architecture section above for rationale). Per-face pipeline: backface check →
  depth check via support function → support face clipping → plane projection →
  `ContactReducer` to 4 points. Each `ContactFace` is wrapped in a
  `SupportPolygon` struct implementing `ConvexSupport` and `SupportFaceExtractor`
  for the clipping step.
- Backface check uses exact support points (shape's highest support along face
  normal) rather than center estimation, avoiding center-estimation jitter.
- Shallow contacts (near-touching) emit a single projected point to avoid
  premature face-face manifolds during edge/vertex-to-face transitions.
- Clipping fallback: if Sutherland-Hodgman produces no points, projects the
  shape's deepest point onto the face plane if it lands inside the polygon.
- Wildcard arm in `generate_mesh_manifold()` uses `#[allow(unreachable_patterns)]`
  (same pattern as `generate_manifold()`). Currently unreachable; activates when
  `ConvexHull` is added.
- Bench harness scenario (`ConvexOnFlatScenario`) deferred — requires test-only
  dispatch override or `ConvexHull` shape variant (Step 6.7).
- 7 unit tests: OBB/sphere/capsule cross-validation, ramp, step, no-contact,
  and manifold reduction.

---

### Step 6.6: GJK raycast (translation-only CCD)

**Goal:** Non-sphere shapes get continuous collision detection for the first time.

**New files:**
- `src/collision/continuous/gjk_raycast.rs` — `gjk_raycast()` function.
  Translation-only CCD for any `ConvexSupport` pair.

**Modified files:**
- `src/collision/continuous/mod.rs` — add `pub mod gjk_raycast`, re-export.
- `src/physics/ccd/sweep_clamp.rs` — for non-sphere CCD candidates, use
  `gjk_raycast` for the time-of-impact query instead of the bounding-sphere
  sweep. The existing per-shape contact generation at the hit point remains
  unchanged.

**Testing plan (unit tests in `gjk_raycast.rs`):**
- **Sphere-sphere cross-validation:** Two spheres on collision course. Compare
  TOI with `swept_sphere_sphere` analytic result (±0.01).
- **OBB vs static plane:** Box moving toward a plane (represented as a large
  thin OBB or triangle). Verify TOI matches geometric prediction.
- **OBB vs OBB:** Two OBBs approaching each other. Verify TOI > 0 and
  the normal at impact is reasonable.
- **Miss case:** Two shapes moving apart. Verify `None` returned.
- **Already overlapping:** Shapes starting in overlap. Verify `Some(0.0)` or
  similar sentinel.
- **Grazing contact:** Shape barely clips the corner of another. Verify TOI
  detected (not missed).
- **Zero relative velocity:** Shapes not moving relative to each other. Verify
  `None` (no collision if not approaching).

*Bench harness tests:*
- **New test in `ccd.rs`: `fast_box_tunneling`.** A fast-moving box aimed at
  thin static geometry. Without CCD it would tunnel through. Verify the box
  is stopped. Compare against existing `fast_sphere_ccd` scenario.

**Status: COMPLETE**

**Implementation notes:**
- Algorithm is conservative advancement on the Minkowski difference: repeatedly
  queries `gjk_query()` for the minimum distance, then advances `t += distance /
  rel_speed`. Guaranteed to converge since each step covers at least `distance`
  units of relative motion. Capped at 64 iterations with conservative miss.
- `TranslatedShape` wrapper offsets a shape's support function by `displacement * t`
  without copying/mutating the original shape. This keeps the GJK queries
  allocation-free.
- Shape center estimated from 6 axis support queries for overlap normal fallback
  (same utility pattern as gjk_patch and gjk_epa_manifold).
- Already-overlapping shapes at t=0: detected by initial GJK query, returns t=0
  with estimated separation normal.
- Zero relative velocity: short-circuits to `None` before entering the march loop.
- Wired into `sweep_clamp.rs`: non-sphere CCD candidates use
  `sweep_shape_against_static()` which runs `gjk_raycast` between a `ShapeView`
  and each triangle in the query region. Falls back to bounding-sphere sweep if
  GJK raycast misses (for robustness during the transition period).
- Bench harness test (`fast_box_tunneling`) deferred — requires runtime testing
  with actual static geometry and physics pipeline.
- 9 unit tests: sphere cross-validation, OBB vs plane, OBB vs OBB, miss, overlap,
  grazing, zero velocity, both-moving, rotated OBB.

---

### Step 6.7: ConvexHull shape variant

**Goal:** Add `ConvexHull` as a first-class shape, proving out the full
GJK/EPA pipeline end-to-end.

**New files:**
- `src/collision/convex_hull.rs` — `ConvexHull` struct with:

  ```rust
  pub struct ConvexHull {
      /// Vertices of the hull in local space.
      pub vertices: Vec<Vector3<f32>>,
      /// Faces of the hull, each with vertex indices and precomputed normal.
      pub faces: Vec<HullFace>,
      /// Precomputed bounding radius (max vertex distance from origin).
      pub bounding_radius: f32,
  }

  pub struct HullFace {
      /// Indices into `vertices` (CCW winding from outside).
      pub vertex_indices: SmallVec<[u16; 6]>,
      /// Outward face normal (precomputed, normalized).
      pub normal: Vector3<f32>,
  }
  ```

  Construction validates: vertex count ≤ 64, minimum thickness (thinnest
  AABB dimension > 1% of largest), faces are provided (not computed —
  hull generation is out of scope).

  `ConvexSupport` impl: brute-force scan of all vertices, computing
  `dot(vertex, direction)` and returning the maximum. Vertices are in local
  space; the `ShapeView`'s rotation and center transform the result to world
  space (handled in `ShapeView`'s `ConvexSupport` impl).

  `SupportFaceExtractor` impl: scan faces for max `dot(face.normal, direction)`,
  transform vertex indices to world-space points, return `SupportFace`.
  Face index is the index into the `faces` array.

**Modified files:**
- `src/physics/collider.rs` — add variant to `ColliderShape`:
  ```rust
  ConvexHull { hull: Arc<ConvexHull> }
  ```
  `Arc` because the hull data (vertices + faces) is potentially large and
  shared across cloned colliders. Implement `compute_mass` (tetrahedron
  decomposition from origin: sum signed volumes of tetrahedra formed by
  each face and the origin), `compute_inertia` (same decomposition, sum
  per-tetrahedron inertia contributions), `bounding_radius` (delegate to
  `hull.bounding_radius`).
- `src/collision/support.rs` — add `ConvexSupport` impl for `ConvexHull`
  (brute-force vertex scan in local space).
- `src/collision/shape_view.rs` — add `ConvexHull` arms to `ShapeView`'s
  `ConvexSupport` impl (transform local support point to world space) and
  `SupportFaceExtractor` impl (transform face vertices to world space).
- `src/collision/mod.rs` — add `pub mod convex_hull`, re-export.
- `src/collision/dispatch.rs` — no changes needed; the `_ =>` wildcard
  arm already routes ConvexHull pairs through GJK/EPA.
- `src/physics/collider.rs` — add `ColliderDesc::convex_hull(hull: Arc<ConvexHull>)`
  builder method.

**Not in scope (but needed for full game integration):**
- Mesh loading pipeline (creating hulls from .obj or similar)
- ECS spawnable wiring
- Debug rendering of convex hull wireframe
- Convex hull generation from arbitrary point clouds

**Testing plan (unit tests):**
- **ConvexSupport correctness:** Create a cube as `ConvexHull` (8 vertices).
  Verify `support()` matches `Obb::support()` for the same box geometry in
  multiple directions.
- **Mass/inertia:** Cube hull vs `ColliderShape::Box` — mass and inertia
  tensor should match (±1%).
- **Bounding radius:** Verify matches expected value for known geometries.
- **Vertex cap:** Attempting to create a hull with > 64 vertices panics
  (or returns error).
- **Thickness check:** A hull with one degenerate dimension (all vertices
  coplanar) is rejected.

*Integration tests via dispatch:*
- **Hull-hull collision:** Two cube-hulls approaching each other. Verify
  `generate_manifold()` routes through GJK/EPA and produces contacts with
  correct normals and depths. Cross-validate against OBB-OBB result for
  the same geometry.
- **Hull vs sphere:** Cube-hull vs sphere. Cross-validate against
  `sphere_obb_manifold`.
- **Hull vs mesh:** Cube-hull dropped onto flat `FilteredPatch`. Cross-validate
  against `obb_patch_manifold` for same box geometry.

*Bench harness:*
- **New scenario: `ConvexHullRestScenario`.** A cube-shaped convex hull dropped
  onto flat terrain. Assert settling, rest height, and contact stability match
  `FlatBoxRestScenario` within tolerance.
- **New scenario: `HullHullCollisionScenario`.** Two convex hulls colliding
  mid-air. Assert momentum transfer.

*Manual testing:*
- Spawn convex hull objects in test_arena level (once spawnable wiring exists).
  Verify they collide with terrain, spheres, boxes, capsules, and each other.
  Verify they don't tunnel at moderate speeds (CCD via gjk_raycast).

**Status: COMPLETE**

**Implementation notes:**
- `ConvexHull` struct with `new()` validation (vertex cap, minimum thickness),
  `compute_volume()` and `compute_inertia()` via tetrahedron decomposition.
- `ConvexSupport` impl: brute-force vertex scan (local space). `TransformedHull`
  wrapper applies center/rotation for world-space queries.
- `SupportFaceExtractor` impl: linear scan of face normals. First face wins on
  ties for consistent axis ordering with OBB support-face path.
- `ColliderShape::ConvexHull { hull: Arc<ConvexHull> }` variant with mass, inertia,
  bounding radius delegation. `ColliderDesc::convex_hull()` builder.
- `ShapeView` delegates `ConvexSupport` and `SupportFaceExtractor` to
  `TransformedHull` for the `ConvexHull` variant.
- Dispatch routes all `ConvexHull` pairs through the GJK/EPA wildcard arm.
  Mesh dispatch routes through `gjk_patch_manifold`.
- `cube_hull()` utility for testing: constructs a cube hull matching OBB geometry.
- ECS spawnable wiring for tetrahedron, octahedron, icosahedron, dodecahedron.
- 10+ unit tests: support correctness vs OBB, mass/inertia cross-validation,
  vertex cap, thickness check, face extraction, bounding radius.
- Hull-hull, hull-OBB, hull-sphere, and hull-capsule cross-validation tests
  in `gjk_epa_manifold.rs` against analytic/SAT reference results.
- Bench harness scenarios deferred — requires runtime testing with physics pipeline.

---

## Performance notes

GJK/EPA is significantly more expensive than analytic/SAT fast-paths:

| Algorithm | Estimated cost per pair | Notes |
|-----------|----------------------|-------|
| Sphere-sphere (analytic) | ~20 ns | Closed-form |
| Sphere-OBB (analytic) | ~50 ns | Closest point on OBB |
| OBB-OBB (SAT) | ~300-400 ns | 15 axes + clipping |
| GJK (separated) | ~200-500 ns | 3-6 iterations typical |
| GJK + EPA (overlapping) | ~500-2000 ns | EPA adds 3-10 iterations |
| GJK + EPA + face clip | ~800-3000 ns | Clipping adds ~200-400 ns |

This means the existing fast-paths remain important. GJK/EPA is the fallback,
not the primary path, for Sphere/Box/Capsule pairs.

**Potential optimizations (beyond deferred items):**
- **EPA face cache:** Cache the closest face index between frames. If the
  penetration direction hasn't changed much, start expansion from the
  cached face.
- **GJK portal refinement:** For near-touching shapes, the Minkowski Portal
  Refinement (MPR) algorithm can be faster than GJK+EPA since it finds
  penetration in a single pass. Could be added as a tier-2.5 between SAT
  and GJK/EPA.
- **Support function batching:** For hull-vs-mesh with multiple faces,
  batch the support queries to improve branch prediction.
- **Constructive tetrahedron init for EPA:** The current `ensure_tetrahedron`
  exhaustively searches all 4-tuples of candidate support points for the
  best origin-enclosing tetrahedron (O(n⁴)). A constructive O(n) approach —
  pick a point, find the farthest from it, find the point farthest from that
  line, find the point farthest from that triangle plane on the origin's side
  — would be faster but greedy (no guarantee of finding the largest-volume
  enclosing tetrahedron). With n bounded at ~14, the exhaustive search
  evaluates ~1001 combinations and is negligible. **When to revisit:** only
  if the candidate set grows significantly (e.g. adding more probe directions
  for difficult geometries).

---

## Summary

```
Step 6.1: Dispatch refactor ◄── COMPLETE ──────────────────────┐
    │                                                           │
Step 6.2: GJK algorithm ◄── COMPLETE                           │
    │                                                           │
Step 6.3: EPA + face clipping + manifold generation ◄── COMPLETE│
    │                                                           │
Step 6.4: Wire GJK/EPA fallback into dispatch ◄── COMPLETE ── │
    │                                                           │
Step 6.5: gjk_patch (general convex vs mesh) ◄── COMPLETE ─── │
    │                                                           │
Step 6.6: GJK raycast CCD ◄── COMPLETE ───────────────────── │
    │                                                           │
Step 6.7: ConvexHull shape variant ◄── COMPLETE ──────────── │
```

Each step produces a compiling, testable result. Steps are wired live as
soon as they're ready. The existing fast-paths are never removed — GJK/EPA
slots in as a fallback alongside them.
