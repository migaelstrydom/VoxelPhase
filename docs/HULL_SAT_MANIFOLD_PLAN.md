# Hull-Hull SAT Manifold Plan

Replace the heuristic EPA→face-clipping manifold path for convex hull pairs with
a principled SAT-based minimum-penetration-axis search, followed by standard
reference/incident face clipping.

---

## Problem statement

The current hull-hull manifold pipeline is GJK → EPA → support-face lookup →
Sutherland-Hodgman clipping. EPA returns a single penetration direction from the
Minkowski polytope, which doesn't necessarily correspond to any face normal of
either hull. When the EPA normal falls between two face normals (common on
high-face-count hulls like the 20-gon tapered cylinders), the face selection can
pick the wrong pair, producing contact points that lie outside one or both hulls.

Multiple mitigation layers have been added (`refine_face_face_normal`,
`best_face_pair_for_normal`, jittered probe directions), each recovering some
failure cases heuristically. Despite this, phantom contacts persist — three
distinct regression tests reproduce the problem. The heuristic searches are also
expensive: `best_face_pair_for_normal` alone probes ~30 directions, collects
candidate face pairs, and trial-clips up to 36 combinations.

### Root cause

The face-selection problem is inherent to using EPA's normal for face lookup.
EPA solves for the closest point on the Minkowski difference boundary, not the
minimum-penetration face normal. These coincide for face-face contacts where the
EPA polytope converges cleanly, but diverge near face boundaries, edges, and
symmetric configurations. No amount of jittered probing can guarantee finding the
correct axis because the search is heuristic over a continuous space.

### Why SAT works

SAT tests a finite, exhaustive set of candidate axes. For two convex polyhedra,
the minimum penetration axis must be either:
- A face normal from hull A, or
- A face normal from hull B, or
- A cross product of an edge from A and an edge from B.

Face normals alone find the correct axis for all face-face and face-edge
contacts. Edge-edge cross products are only needed for pure edge-edge contacts
(two edges touching at a point), which are geometrically rare on high-poly hulls
and produce shallow penetrations that face normals handle acceptably. Professional
engines (Jolt, Bullet) typically omit edge-edge SAT for general convex hulls.

---

## Design

### Pipeline: GJK → SAT → clip

1. **GJK (cached)** — unchanged. Tests for overlap on the uninflated Minkowski
   difference. Separated pairs early-out. The GJK cache (`last_direction`) makes
   repeated queries for stable pairs nearly free.

2. **SAT over face normals** — runs only when GJK reports intersection or margin
   contact. Tests all face normals from both hulls as candidate separating axes.
   For each axis, computes overlap via support projections. Finds the
   minimum-penetration axis (shallowest overlap). This replaces EPA + all
   heuristic normal refinement.

3. **Reference/incident face clipping** — identical to the existing OBB-OBB
   clipping path. The SAT axis identifies which face is the reference face, the
   opposing shape's most-anti-aligned face is the incident face. Sutherland-
   Hodgman clips the incident face against the reference face's side planes.
   Clipped vertices below the reference plane become contact points.

### SAT cache

A `SatCache` (same type already used by OBB-OBB) stores a separating axis
direction (`Option<Vector3<f32>>`) from the previous frame. On the next frame:
- Test the cached axis first.
- If it still separates, return immediately — shapes aren't colliding.
- If it doesn't separate, run the full axis search. If the pair is now
  colliding, clear the cached axis. If separated on a different axis, store
  the new one.

Note: the SAT cache is a *separation* cache, not a penetration cache. It
stores the last separating axis for fast rejection of non-colliding pairs. For
colliding pairs, the full axis search always runs (but is bounded at
F_a + F_b axes — no iterative convergence needed).

The SAT cache serves a different role from the GJK cache:
- **GJK cache**: seeds the search direction for faster convergence (reduces
  iterations from ~10 to ~2-3 for stable pairs).
- **SAT cache**: stores a separating axis for fast rejection (reduces the
  separation test from F_a + F_b axis tests to 1 for stable separated pairs).

Both caches are stored per-collider-pair in the narrowphase, keyed the same way
as the existing `GjkCacheMap`.

### Margin handling

Contact margin is handled identically to OBB-OBB: the overlap computation
inflates each shape's half-extent projection by `margin`. Contacts with overlap
in `(-margin, 0]` are margin-only contacts (raw_depth < 0, solver depth clamped
to 0 for velocity-only correction).

### What gets removed

- `refine_face_face_normal` — replaced by SAT axis search.
- `best_face_pair_for_normal` — replaced by SAT axis directly identifying the
  reference face.
- `jittered_probe_directions` — no longer needed.
- `face_contact_candidate` — no longer needed.
- `normal_precedes` / `pick_best_candidate` — tie-breaking heuristics no longer
  needed; SAT has a deterministic minimum.
- EPA call in the hull-hull `(Some(fa), Some(fb))` arm — EPA is no longer used
  for polyhedra-vs-polyhedra pairs.
- `phantom_debug_enabled` / phantom debug logging — the problem it diagnosed
  goes away.

EPA remains for pairs involving at least one curved/faceless shape (hull-sphere,
hull-capsule, sphere-capsule, etc.) where the GJK/EPA single-point path is
correct and sufficient.

### What stays

- GJK (`gjk_query_seeded`) — still the overlap/separation test.
- EPA (`epa_penetration`) — still used for curved shape pairs.
- `support_overlap` — reused for SAT axis testing.
- `ContactReducer` — still reduces to 4 contacts.
- All analytic/SAT fast-paths (sphere-sphere, sphere-OBB, OBB-OBB, etc.) —
  unchanged.

### Relationship to OBB-OBB SAT code

The hull-hull SAT follows the same structure as `obb_obb.rs` — same phases
(cache check → axis enumeration → classification → face clip or edge-edge),
same flow — but uses different primitives because OBB-OBB is optimized for
rectangular geometry.

**Shared utility layer** (already in `clipping.rs`, `segment.rs`, `sat.rs`):
- `clip_polygon()` — core Sutherland-Hodgman half-plane clipper.
- `segment_segment_closest_points()` — for edge-edge contacts.
- `SatCache` — same struct, same semantics.
- `ContactReducer` — same reduce-to-4 logic.

**OBB-specific optimizations that don't generalize:**
- `Obb::project_half_extent()` — closed-form axis projection from 3 axes +
  half-extents. Hull-hull uses `ConvexSupport::support()` (vertex scan).
- `ObbFace.clip_against_sides()` — clips against 4 axis-aligned half-planes
  using tangent/bitangent half-extents. Hull-hull needs general N-gon clipping.
- Edge enumeration from 3 axes × 4 edges. Hull-hull iterates hull edge pairs.

**New shared code to extract:** move the general polygon-vs-polygon side
clipping from `gjk_epa_manifold.rs` (`clip_against_face_sides`) into
`clipping.rs`. This clips an incident polygon against each edge of a
reference polygon by computing inward-pointing edge normals, calling
`clip_polygon()` per edge. It sits alongside `ObbFace.clip_against_sides()`
as the general-purpose equivalent. Hull-hull calls the general version;
OBB-OBB keeps its rectangular fast path.

The two implementations are parallel — same phases, shared utilities,
different shape-specific primitives. Forcing them into a generic trait would
sacrifice OBB's optimizations without real benefit.

---

## Performance analysis

### Axis count

For two hulls with F_a and F_b faces:
- **Axes to test**: F_a + F_b face normals.
- **Per axis**: 2 support function evaluations (one per shape). Each support
  function scans V vertices (brute-force dot product).
- **Total**: (F_a + F_b) × 2 × max(V_a, V_b) dot products.

For two 22-face tapered cylinders (40 vertices each):
- 44 axes × 2 × 40 = **3,520 dot products**.

### Comparison with current approach

| Path | Dot products | Other overhead |
|------|-------------|----------------|
| Current (common case: EPA + clip works) | ~3,000 | EPA polytope alloc |
| Current (hard case: + refinement + face-pair search) | ~6,000+ | 36 clip attempts |
| SAT (uncached) | ~3,500 | None |
| SAT (cached, stable contact) | ~80 | None |

The cached steady-state cost (1 axis test = 2 × 40 dot products) is
significantly cheaper than even the current common case.

### Allocations and cache friendliness

The hull-hull SAT path should be **zero-allocation** in the hot loop. All
intermediate buffers fit in stack-allocated `SmallVec`s:

- Clipped polygon vertices: `SmallVec<[Point3<f32>; MAX_CLIP_VERTS]>` (same
  as existing `clipping.rs`). A 20-gon incident face clipped against a 20-gon
  reference face can produce up to 40 vertices, which exceeds `SmallVec`'s
  inline capacity and would spill to heap. However, in practice incident faces
  on these hulls are quads (4 vertices — the side faces), not the 20-gon end
  caps, so inline capacity of 8-12 should suffice for the common case. Use
  `SmallVec<[Point3<f32>; 12]>` for the clip buffer. If it spills on rare
  cap-vs-cap contacts, that's one allocation per frame — acceptable.

- Contact points: `SmallVec<[ContactPoint; 4]>` before reduction.

- World-space face vertices: computed on-the-fly during clipping (transform
  each vertex as needed). No need to pre-transform all hull vertices.

By contrast, the current EPA path allocates `Vec`s for the polytope vertices
and faces on every call (`epa.rs:63-64`), plus `Vec<SupportFace>` in
`best_face_pair_for_normal` (`gjk_epa_manifold.rs:797-798`). The SAT path
eliminates all of this.

**Memory access pattern:** the SAT loop iterates hull A's face normals, then
hull B's face normals. Each axis test calls `support()` on both shapes, which
scans all vertices. The `ConvexHull::vertices` vec is contiguous in memory, so
the support scan is cache-friendly. Face normals are accessed sequentially
from `ConvexHull::faces`. No special work buffer is needed — the data layout
is already favorable.

No changes to `NarrowphaseWorkBuffer` are needed beyond renaming
`active_box_pairs` → `active_sat_pairs`.

### Edge-edge axes

For high-face-count hulls (20-gon cylinders: 22 faces, 60 edges), edge-edge
contacts are geometrically rare and the face normals are closely spaced, so
the depth overestimate from skipping edge-edge axes is small. Full edge-edge
SAT would add E_a × E_b = 3,600 axes, which is prohibitive without Gauss map
culling.

For low-face-count hulls (tetrahedra: 4 faces, 6 edges), edge-edge contacts
are common (two tetrahedra resting edge-to-edge) and the face normals are
widely spaced (~109° apart). The depth overestimate can be ~1.7x the true
depth. In practice this means the solver pushes bodies apart slightly more
than necessary — probably invisible, but worth verifying with actual scenes.

**Approach:** implement face-normal-only SAT first and test. If edge-edge
causes visible artifacts on low-face hulls, add a fast path for small hulls:

```
if hull_a.faces.len() + hull_b.faces.len() <= EDGE_EDGE_FACE_THRESHOLD {
    // Include edge-edge cross product axes.
    // Cost for two tetrahedra: 6 × 6 = 36 extra axes — negligible.
}
```

A threshold of ~24 total faces would cover tetrahedra (4+4=8), octahedra
(8+8=16), and similar low-poly shapes while excluding the 20-gon cylinders
(22+22=44). The edge-edge cost at this threshold is at most ~144 cross
products (12 × 12 edges), which is cheap.

For large hulls, edge-edge SAT remains deferred pending Gauss map culling
(O(E²) → O(E) expected). See deferred work D1.

---

## Implementation plan

### Step 1: Hull-hull SAT function

New file: `src/collision/discrete/hull_hull.rs`

Register it in `src/collision/discrete/mod.rs`.

```rust
pub fn hull_hull_manifold(
    a: &ShapeView,    // must be ConvexHull
    b: &ShapeView,    // must be ConvexHull
    margin: f32,
    sat_cache: &mut SatCache,
    gjk_cache: Option<&mut GjkCache>,
) -> ContactManifold
```

Core loop:
1. Extract `&ConvexHull` refs from both `ShapeView`s (match on
   `ColliderShape::ConvexHull { hull }`).
2. **GJK pre-filter:** run `gjk_query_seeded(a, b, seed)` where seed comes
   from `gjk_cache`.
   - `Separated` with `distance > 2.0 * margin`: update GJK cache, return
     empty.
   - `Separated` with `distance <= 2.0 * margin`: margin-only contact.
     Construct a single-point contact from GJK's witness points directly
     (same as the current `gjk_epa_manifold.rs:148-171`). No SAT needed —
     margin contacts get `raw_depth < 0` and velocity-only correction, so
     a single point is sufficient.
   - `Intersecting`: proceed to SAT (step 3).
3. Test SAT cached axis first (if present). If it still separates, return
   empty.
4. Iterate all face normals from hull A. Each `HullFace` stores a local-space
   normal; transform to world space: `world_normal = a.rotation * face.normal`.
   Compute overlap via `support_overlap(a, b, world_normal)` (already exists in
   `gjk_epa_manifold.rs` — move it to a shared location or re-implement
   inline). Track minimum overlap and the winning face index.
5. Same for hull B face normals (testing `-(b.rotation * face.normal)` as the
   axis, or equivalently testing `b.rotation * face.normal` and negating).
6. If any overlap is negative, shapes are separated — store the most-negative
   axis in `sat_cache.separating_axis`, update GJK cache, return empty.
   (Note: GJK said "intersecting" but SAT found separation — this can happen
   because GJK tests the uninflated Minkowski difference while SAT includes
   margin. It also happens due to GJK's numerical tolerance.)
7. The winning axis identifies the reference face directly (it's the face whose
   normal we tested). Select the incident face from the opposing hull: iterate
   the opposing hull's faces, find the one whose world-space normal has the
   smallest (most negative) dot product with the reference normal.
8. Transform reference and incident face vertices to world space using the
   same pattern as `TransformedHull` in `convex_hull.rs`:
   `world_vertex = center + rotation * local_vertex`.
9. Clip incident face polygon against reference face side planes.
10. Project clipped vertices onto reference plane, filter by depth, reduce to 4.
11. Clear `sat_cache.separating_axis` (shapes are colliding).

#### SAT axis overlap computation

For each candidate axis (world-space unit vector `n`):
```
overlap = a.support(n).dot(n) - b.support(-n).dot(n) + 2 * margin
```
This is the same as `support_overlap` in `gjk_epa_manifold.rs` plus margin.
Positive overlap = penetrating on this axis. The minimum positive overlap
across all axes is the penetration depth, and its axis is the contact normal.

If any axis gives negative overlap, the shapes are separated (early-out).

Ensure the axis is oriented A→B: if `(center_b - center_a).dot(axis) < 0`,
negate the axis. This matches the OBB-OBB convention in `obb_obb.rs:305`.

#### Face clipping for general N-gon polygons

OBB-OBB uses `ObbFace.clip_against_sides()` which exploits the rectangular
structure (4 axis-aligned side planes from tangent/bitangent half-extents).
Hull faces are general convex N-gons, so we need general polygon-vs-polygon
Sutherland-Hodgman clipping.

This already exists in `gjk_epa_manifold.rs` as `clip_against_face_sides()`.
It clips an incident polygon against each edge of a reference polygon by
computing inward-pointing edge normals. **Move this function to
`clipping.rs`** alongside `ObbFace.clip_against_sides()` as the
general-purpose equivalent (see "Relationship to OBB-OBB SAT code" above).

The clipping pipeline is:
1. Compute the reference face center (centroid of its vertices).
2. For each edge of the reference face polygon:
   - Compute the edge vector: `edge = verts[(i+1) % n] - verts[i]`
   - Compute the inward normal: `inward = edge.cross(ref_normal)`, oriented
     so it points toward the face center.
   - Clip the incident polygon against this half-plane using `clip_polygon()`
     from `clipping.rs`.
3. After clipping against all edges, the remaining polygon vertices are the
   candidate contact points.

#### Depth computation and contact construction

After clipping, for each surviving vertex:
```
signed_dist = (vertex - ref_center).dot(ref_normal)
```
Keep vertices where `signed_dist < margin_tolerance` (penetrating or within
margin). The raw depth is `(-signed_dist).min(axis_overlap - 2 * margin)`.
Construct `ContactPoint` with the SAT axis as the normal.

This matches the pattern in `obb_obb.rs:348-361`.

#### Feature IDs

Use `FeatureId::from_face_pair(ref_face_index, inc_face_index)` as the base,
with `.with_vertex(i)` for each contact point — same pattern as existing
`gjk_epa_manifold.rs:680`. The face indices come directly from the hull's
face array index.

#### Normal direction convention

The contact normal must point from body A toward body B (matching the solver
convention). The dispatch layer in `dispatch.rs` handles argument ordering
via `shape_rank`, but hull-hull will now have its own match arm. Ensure:
- If the winning axis is a face normal from hull A: normal = world_normal_a
  (already points away from A toward B).
- If the winning axis is a face normal from hull B: normal = -world_normal_b
  (flip to point from A toward B).

#### Implementation notes (Step 1)

Completed. Deviations and observations from the plan:

- **Margin contact depth convention:** The plan stated margin contacts would
  have `raw_depth < 0`. In practice, GJK catches margin contacts before SAT
  runs, and its depth formula is `2 * margin - distance`, which is positive
  when the gap is smaller than `2 * margin`. This is consistent with the
  existing `gjk_epa_manifold.rs` margin path. The SAT path itself is never
  reached for margin-only contacts.

- **Shared clipping extraction:** Moved `clip_against_face_sides`,
  `clip_polygon_into`, and `compute_face_center` (renamed `face_centroid`)
  from `gjk_epa_manifold.rs` to `clipping.rs`. Added `HullClipPolygon`
  type alias (`SmallVec<[Point3<f32>; 12]>`) for the larger inline buffer.
  `gjk_epa_manifold.rs` now imports these from `clipping.rs`.

- **Normal orientation:** The plan suggested orienting each axis A→B using
  `(center_b - center_a).dot(axis) < 0` and negating. The implementation
  does this for all face normals from both hulls uniformly, rather than
  flipping B's normals separately. The reference/incident face selection
  then uses the `best_from_a` flag to determine which hull owns the
  reference face.

- **`support_overlap` not moved to shared location.** Re-implemented inline
  in `hull_hull.rs` as a two-liner (as the plan suggested as an alternative)
  with an additional `margin` parameter. The `gjk_epa_manifold.rs` version
  remains for its own callers.

- **Contact buffer:** Uses `SmallVec<[ContactPoint; 4]>` (matching
  `ContactManifold::from_vec`'s expected type) rather than the
  `SmallVec<[ContactPoint; 8]>` initially planned.

### Step 2: Wire into dispatch

In `dispatch.rs`, add a match arm for `(ConvexHull, ConvexHull)` **before** the
GJK/EPA wildcard `_` arm:

```rust
(ColliderShape::ConvexHull { .. }, ColliderShape::ConvexHull { .. }) => {
    hull_hull_manifold(
        a, b, margin,
        sat_cache.map_or(&mut SatCache::default(), |c| c),
        gjk_cache,
    )
}
```

The existing `generate_manifold` signature already accepts
`sat_cache: Option<&mut SatCache>`, so no signature change is needed.

**GJK pre-filter:** keep GJK as the overlap test before SAT. The broadphase
produces false positives (overlapping AABBs for non-colliding pairs). For
these separated pairs, GJK with a warm cache confirms separation in 1-3
iterations (~100-200 dot products). Full face-normal SAT would test all
F_a + F_b axes (~3,500 dot products) to reach the same conclusion. The SAT
cache helps for stable separated pairs, but not for transient broadphase
false positives where there's no useful cached axis.

Pipeline: GJK (cached) → if intersecting → SAT (cached) → clip.

Both caches are stored per-pair. This is different from OBB-OBB, which skips
GJK because OBBs have only 6 face normals (cheap full SAT). For hulls with
40+ face normals, the GJK pre-filter is worthwhile.

#### Implementation notes (Step 2)

Completed. The `(ConvexHull, ConvexHull)` match arm was added before the
wildcard `_` arm. Also removed the `validate_hull_hull_contacts` call from
the wildcard arm since hull-hull pairs no longer reach it. The debug
validation functions (`validate_hull_hull_contacts`, `max_face_plane_violation`,
`print_hull_geometry`) are now dead code — they'll be removed in Step 6.

All 3 phantom contact regression tests now pass:
- `phantom_contact_tapered_cylinders`
- `phantom_contact_cylinder_vs_tetrahedron`
- `phantom_contact_cylinders_large_violation`

### Step 3: SAT cache storage in narrowphase

In `src/physics/narrowphase/dynamic_contacts.rs`, the SAT cache is currently
gated to Box-Box pairs only:

```rust
let mut sat_cache_opt = match (&si.shape, &sj.shape) {
    (ColliderShape::Box { .. }, ColliderShape::Box { .. }) => { ... }
    _ => None,
};
```

Extend this match to also provide caches for ConvexHull-ConvexHull pairs:

```rust
(ColliderShape::Box { .. }, ColliderShape::Box { .. })
| (ColliderShape::ConvexHull { .. }, ColliderShape::ConvexHull { .. }) => {
    buf.active_box_pairs.insert(...);  // rename to active_sat_pairs
    let key = SatPairKey::new(si.collider_handle, sj.collider_handle);
    Some(sat_cache_map.caches.entry(key).or_default())
}
```

Rename `active_box_pairs` → `active_sat_pairs` to reflect its broader use.
Update the prune call at line 270 accordingly.

The `SatCache` struct (`src/collision/sat.rs`) stores
`separating_axis: Option<Vector3<f32>>` — this works as-is for hull-hull.
No changes to the struct needed.

The GJK cache is still needed for hull-hull pairs (GJK pre-filter). Hull-hull
pairs will populate both the SAT cache map and the GJK cache map.

#### Implementation notes (Step 3)

Completed. Straightforward — extended the SAT cache match arm with
`| (ColliderShape::ConvexHull { .. }, ColliderShape::ConvexHull { .. })`,
renamed `active_box_pairs` → `active_sat_pairs`. ConvexHull pairs already
got GJK caches via `requires_gjk_fallback` (they aren't in the exclusion
list), so hull-hull pairs now populate both cache maps as intended.

### Step 4: Clean up the heuristic code

Remove from `gjk_epa_manifold.rs`:
- `refine_face_face_normal`
- `best_face_pair_for_normal`
- `jittered_probe_directions`
- `face_contact_candidate`
- `phantom_debug_enabled` and associated logging

Check whether these are used by any remaining path (hull-sphere,
hull-capsule via the wildcard arm). The `face_vs_faceless_contact` function
uses `face_contact_candidate` and `jittered_probe_directions` — these serve
a different purpose (hull-vs-sphere normal selection) and may still be needed.
If so, keep them but remove only the hull-hull-specific heuristics
(`refine_face_face_normal`, `best_face_pair_for_normal`).

Check whether `normal_precedes` / `pick_best_candidate` are used outside the
removed functions. If only used by them, remove. If `face_vs_faceless_contact`
uses them, keep.

Move `support_overlap` to a shared location (e.g. `support.rs` or
`clipping.rs`) if `hull_hull.rs` needs it. Or re-implement inline — it's a
two-liner.

Move `clip_against_face_sides` from `gjk_epa_manifold.rs` to `clipping.rs`
(see "Relationship to OBB-OBB SAT code" above). Update any remaining callers
in `gjk_epa_manifold.rs` to import from the new location.

#### Implementation notes (Step 4)

Completed. Removed from `gjk_epa_manifold.rs`:
- `refine_face_face_normal` — hull-hull-specific heuristic, no longer needed.
- `best_face_pair_for_normal` — hull-hull-specific heuristic, no longer needed.
- `phantom_debug_enabled` and all `[phantom]` logging throughout the file.
- `ClippedFaceManifold` wrapper struct — `clip_face_face_manifold` now returns
  `ContactManifold` directly.
- `HashSet` import (only used by `best_face_pair_for_normal`).

Kept (still used by `face_vs_faceless_contact` for hull-sphere/hull-capsule):
- `jittered_probe_directions`
- `face_contact_candidate`
- `normal_precedes` / `pick_best_candidate`
- `DEPTH_TIE_EPS` / `NORMAL_TIE_EPS` / `NORMAL_PROBE_EPS`

Simplified the `(Some(fa), Some(fb))` arm to use the EPA normal directly
with `support_overlap` depth, falling back to EPA witness midpoint when
clipping produces no contacts. The heuristic refinement was specifically
needed for hull-hull; the remaining user (hull-OBB) works well with EPA's
normal because OBBs have well-separated face normals.

`clip_against_face_sides` and `face_centroid` were already moved to
`clipping.rs` in Step 1. `support_overlap` was already re-implemented
inline in `hull_hull.rs` in Step 1.

### Step 5: Update regression tests

The three phantom contact tests in `dispatch.rs` should now pass (SAT produces
correct contact points):
- `phantom_contact_tapered_cylinders`
- `phantom_contact_cylinder_vs_tetrahedron`
- `phantom_contact_cylinders_large_violation`

Verify they pass. Do not remove them — they serve as regression tests.

Add new targeted tests in `hull_hull.rs`:
- **Axis-aligned cube vs cube:** compare hull SAT result against OBB-OBB for
  the same geometry (same approach as existing sweep tests in `dispatch.rs`).
  Use `cube_hull()` from `convex_hull.rs` (cfg(test) utility).
- **SAT cache early-out:** call twice with the same separated pair, verify the
  cache stores a separating axis and the second call returns empty quickly.
- **SAT cache colliding:** call with overlapping pair, verify cache is cleared.
- **Symmetry:** `hull_hull_manifold(a, b)` and `hull_hull_manifold(b, a)`
  produce equivalent manifolds (same depth, negated normal).
- **Margin-only contact:** two hulls separated by less than `2 * margin`.
  Verify a contact is produced with `raw_depth < 0`.

#### Implementation notes (Step 5)

Completed. The 3 phantom contact regression tests in `dispatch.rs` already
pass (verified in Step 2). The 5 tests from the plan were already added in
Step 1. Added 5 additional regression tests in `hull_hull.rs`:

- **rotated_cubes_produce_contacts** — 45-degree Y rotation, verifies
  corner-to-face contact produces correct normal direction.
- **deeply_overlapping_cubes** — 50% X overlap, verifies ~1.0 depth and
  4-point face contact.
- **tetrahedron_vs_tetrahedron** — low-face-count hulls (4 faces each),
  verifies contacts are produced for the geometry class most affected by
  the lack of edge-edge SAT axes.
- **sat_cache_stores_axis_on_separation** — margin=0 separated pair,
  verifies empty manifold.
- **contacts_inside_both_hulls** — validates every contact point lies
  within both hulls' face planes (the core property phantom contacts
  violated).

Total: 10 hull_hull tests + 3 phantom regression tests = 13 regression tests.

#### Edge-edge SAT (added post-plan)

Implemented edge-edge cross-product SAT axes for small hulls (total faces
≤ 24), addressing depth overestimation on tetrahedra (up to 21x), wedges
(up to 38x), and hexagonal prisms (up to 1.6x). False collisions (face-only
says overlap, edge-edge finds separation) were also observed in testing:
5/300 for wedges, 3/300 for tetrahedra.

**Key implementation details:**

- **Threshold:** `EDGE_EDGE_FACE_THRESHOLD = 24`. Covers tetrahedra (8),
  wedges (10), hex prisms (16), cubes (12). Excludes 20-gon cylinders (44).
- **Classification:** Edge-edge only wins over face if overlap is shallower
  by `EDGE_WIN_MARGIN_FRACTION * margin` (same as OBB-OBB). Edge axes
  aligned with the best face axis (dot² > 0.99) are filtered out.
- **Endpoint clamping fallback:** When the segment-segment closest point
  clamps to a segment endpoint (t ≈ 0 or t ≈ 1), the true contact feature
  is vertex-face rather than edge-edge. In this case, the function returns
  `None` and the caller falls back to face clipping with the edge-edge
  depth. This prevents phantom contacts from midpoint placement on
  non-interior edge-edge contacts.
- **Edge extraction:** `extract_edges()` deduplicates edges from face
  windings using a linear scan with sorted vertex pairs.

**Root cause of hex prism phantom (from bug investigation):** The SAT loop
was flipping face normals to point A→B but storing the original face index.
When the normal was flipped, the stored face was on the wrong side of the
hull, producing contact points ~0.78 outside hull A. Fixed by using
`find_most_aligned_face()` to select the reference face after axis selection,
instead of tracking face indices during the SAT loop.

**Tests added (8 new, 19 total in hull_hull.rs):**
- `edge_edge_tetrahedra_separation` — verifies edge-edge prevents false contacts
- `edge_edge_tetrahedra_correct_depth` — verifies depth < 0.35 (face-only gives ~0.50)
- `edge_edge_wedge_contact` — wedge pair produces valid contacts
- `edge_edge_wedge_false_collision_prevented` — edge-edge separation check
- `extract_edges_tetrahedron` — 6 edges
- `extract_edges_cube` — 12 edges
- `extract_edges_hexagonal_prism` — 18 edges
- `edge_edge_not_used_for_large_hulls` — cubes with edge-edge still valid

### Step 6: Debug validation → compile-time constant

Rather than removing the phantom contact validation, it was placed behind a
`DEBUG_HULL_CONTACTS` constant at the top of `dispatch.rs`. When `false`
(default), dead-code elimination strips the validator entirely. Flip to `true`
to re-enable phantom diagnostics during development.

**Implementation notes:**

`validate_hull_hull_contacts`, `max_face_plane_violation`, and
`print_hull_geometry` remain in `dispatch.rs` but are only called when
`DEBUG_HULL_CONTACTS` is `true`. The `cube_hull()` helper in `convex_hull.rs`
was demoted to `#[cfg(test)]` since the runtime OBB-Hull dispatch no longer
needs it.

### Dedicated Hull-OBB path (was deferred D3)

Implemented in `hull_obb.rs`. Uses the OBB's native `project_half_extent()`
for SAT overlap and `obb_face()` for clipping, eliminating the per-frame
`Arc::new(cube_hull(...))` allocation from the earlier pragmatic approach.

**Key design decisions:**

- Hull is always `a`, OBB is always `b` — normal points hull→OBB, matching
  the solver's body_a→body_b convention (hull has higher shape rank).
- OBB face normals: 3 axes (each tested with A→B orientation), vs the hull's
  N face normals. Uses `project_half_extent` for OBB overlap instead of
  vertex-scanning support functions.
- OBB edge-edge: hull edges × 3 OBB axis directions. When edge-edge wins,
  searches all 4 OBB edge variants per axis to find the closest pair (matching
  `find_best_edge_pair` in obb_obb.rs).
- Face clipping: `obb_face()` provides native quad geometry for reference or
  incident face, `clip_against_face_sides()` for hull reference faces.
- SAT cache extended in `dynamic_contacts.rs` for Box-ConvexHull pairs.

### Edge-edge phantom contact fixes

Two classes of phantom contacts were identified and fixed in the edge-edge
path:

**1. Face-clipping fallback with wrong normal.** When `edge_edge_contact()`
returns `None` (closest point clamped to segment endpoint), the fallback was
passing the edge-edge cross-product axis to `face_contact()`. Since that axis
doesn't align with any face normal, `find_most_aligned_face()` picks a poorly
aligned reference face, producing clipped points far outside the opposing hull.
Fix: the fallback now passes `best_face_axis` instead of the edge-edge normal.

**2. Distant edge pairs winning SAT.** The SAT overlap for an edge-edge axis
is computed from hull support functions (which scan all vertices), not just the
specific edge pair. Two edges can define a valid SAT axis with positive overlap
even when they are spatially far apart in 3D — the support vertices that
produce the overlap may lie on different edges entirely. The edge-edge midpoint
then lands in empty space. Fix: `support_witness_edge()` validates that the
hull's support vertex for the edge-edge axis actually lies on the winning edge
pair. If not, the edge-edge classification is rejected and the face path is
used instead.

---

## Deferred work

### D1: Edge-edge SAT for large hulls

The small-hull fast path (see "Edge-edge axes" above) handles low-face-count
shapes. For large hulls (> threshold), edge-edge SAT is deferred. If needed,
Gauss map culling prunes the O(E²) search to O(E) expected by only testing
edge pairs whose Gauss map arcs overlap.

### D2: Hill-climbing support

For hulls with > 32 vertices, the brute-force support function becomes the
bottleneck in both GJK and SAT. Hill-climbing with a precomputed adjacency
graph reduces support cost to O(sqrt(V)) amortized. Already tracked in
`GJK_EPA_DESIGN.md` as D3.

### D3: Dedicated hull-sphere and hull-capsule paths

The GJK/EPA fallback in `dispatch.rs` is still used for Sphere-Hull and
Capsule-Hull pairs. Dedicated SAT or analytic paths would eliminate the EPA
face-selection risk for these pairs.
