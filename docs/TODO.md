# TODO

Tracked future work for Flipphase.

---

## Constraint system

Add a generalized constraint system (joints, orientation locks, distance constraints) to the physics solver. Immediate need: keep the player capsule upright.

Design doc: [CONSTRAINT_SYSTEM_PLAN.md](CONSTRAINT_SYSTEM_PLAN.md)

### Deferred work

Items explicitly deferred from the initial implementation. These should be addressed before the corresponding features ship.

- **Constraint islands for sleeping** — constraints must participate in island building so that a sleeping body constrained to an awake body gets woken. Safe to skip only while all constraints are world-anchored on always-awake bodies (keep-upright on player). **Must be wired in before any two-body constraint (distance, hinge) ships.**
- **Constraint-aware shock propagation** — the conditioner may need to see constraint edges to compute correct BFS depths when a constraint bridges two contact sub-graphs. Not needed for world-anchored constraints.
- **Per-substep row re-expansion** — initial implementation expands rows once per frame. If multi-substep drift causes visible artifacts with positional constraints (distance, hinge), re-expansion can be added to `substep()`.
- **Hinge, ball-joint, prismatic, motor variants** — add as needed, the row-based infrastructure supports them.
- **Constraint breaking** — max force thresholds for breakable joints.
- **Constraint debug visualization** — rendering constraint axes, limits, error vectors in the debug overlay.

---

## Mesh patch collision: mixed-normal manifolds on concave terrain

When a convex shape (hull or OBB) collides with a concave mesh patch (e.g. a terrain step), the manifold can contain contacts from multiple non-coplanar faces with conflicting normals. Deep contacts from a face the shape shouldn't physically reach (e.g. the floor behind a step) cause solver "pops."

Regression tests: `dispatch.rs::hull_pop_replay_minimal_manifold_should_not_mix_normals` and `obb_patch.rs::pop_replay_obb_manifold_should_not_mix_normals` (both currently `#[should_panic]`).

### Root cause

`hull_patch_manifold` (now removed, was in `src/collision/mesh/hull_patch.rs`) and `obb_patch_manifold` iterate over all faces in a `FilteredPatch`, generate contacts per face using that face's normal, and merge everything into one manifold. On concave terrain the shape can overlap a face it would never reach in practice — the boundary clipping against the face polygon doesn't always reject these because the shape is wide enough to extend past the face boundary into the polygon's area.

### Approach

Per-face full SAT: treat each mesh face (triangle or merged quad) as an independent convex shape and run proper SAT against it — testing hull/OBB face normals, the mesh face normal, and Gauss-map-filtered edge-edge axes. The minimum-penetration axis wins per face, producing contacts with the correct normal and depth. This is the same algorithm as `hull_hull_manifold` with the mesh face acting as "hull B."

Key considerations:

- **Mesh face as convex shape.** A face polygon has spatial extent — its vertices project to a range on any SAT axis. Hull face normals can find separation when the shape doesn't overlap the face's footprint. The face's edge normals (outward-pointing in the face plane, i.e. the side faces of a thin prism) are additional SAT axes that catch grazing boundary overlaps.
- **Reference/incident clipping direction.** When a hull face normal wins (not the mesh face normal), the mesh face becomes the incident face clipped against the hull face's side planes — the reverse of the current clipping direction. `hull_hull_manifold` already handles both directions via the `from_a` flag.
- **Edge-edge with seam filtering.** Internal edges between coplanar faces must be skipped in edge-edge testing. The seam filter's `ContactEdge` classification provides the Gauss map normals needed for filtering.
- **Coplanar merging still valuable.** The seam filter's triangle-pair merging reduces contact count and suppresses internal-edge artifacts. The per-face SAT operates on the merged `ContactFace` polygons (3–6 vertices), not raw triangles.
- **OBB needs the same fix.** `obb_patch_manifold` has the identical bug (proven by the OBB replay test).

---

## Speculative contacts: the band beneath CCD is not covered

A pair closing faster than the discrete contact margin but slower than CCD's
activation gates can interpenetrate by up to a full collider radius before
anything notices. NGS then pushes them apart over roughly 50 ms, so it reads as
a soft or spongy impact rather than a tunnel — nothing passes through — but the
overlap is visible and the impulse arrives late.

**Only one body needs to be moving**, and the speeds are ordinary. For a 0.2 m
sphere at 60 Hz the window is roughly 10–18 m/s. Below it the discrete margin
copes; above it CCD engages and works. Grenades at 20 m/s sit above the window.
The window's position scales with collider radius and with frame rate, so it
moves under you: the same throw can be fine at 30 Hz and wrong at 60.

Red regression tests, currently `#[ignore]`d:
`ccd.rs::spheres_closing_in_the_speculative_band_do_not_interpenetrate` and
`ccd.rs::boxes_closing_in_the_speculative_band_do_not_interpenetrate`. Bench
scenario: `speculative_band_approach`.

### Root cause

Three compounding faults, all in the discrete narrowphase — CCD itself is
correct here.

1. **The broadphase bounds colliders where they are, not where they are going.**
   `generate_dynamic_contacts` builds AABBs from the instantaneous pose, so a
   pair 0.2 m apart closing at 0.4 m per frame is never paired and the
   speculative branch is not reached at all. Speculative contacts can only fire
   for pairs already nearly touching, which is exactly when they are not needed.
2. **The prediction horizon is one substep, but generation is once per frame.**
   `sphere_sphere_speculative` sweeps over the substep `dt` and the result is
   then reused for every substep of the frame, so it looks an eighth of the way
   ahead that it must cover.
3. **The band's ceiling and CCD's floor are in different units.**
   `SpeculativeConfig::ccd_threshold` is substep travel; `ccd_frame_coverage` is
   frame travel. The claim that speculative contacts cover the band below CCD
   was never checkable, and does not hold.

Only sphere-sphere has a speculative path at all. Every other shape pair has
nothing, so a box in the band simply overlaps.

### Approach

Fix all three — bound over the frame's travel, predict over the frame, and
express both gates in frame travel with the speculative ceiling set to
`ccd_frame_coverage` so the two mechanisms meet. Replace the sphere-sphere
special case with `gjk_raycast` over the pair's relative motion, which is
shape-agnostic and is what `ccd/dynamic_sweep.rs` already uses.

That alone is **not sufficient**, and landing it alone is worse than the
current state. The pair then meets and stops 0.6 m apart instead of 0.4 m,
because a zero-depth contact tells the solver to arrest approach *now* rather
than on arrival. Bodies halting in mid-air is a worse failure than bodies
briefly overlapping.

The missing piece is in the solver: a speculative contact must carry its
separation so the normal constraint permits approach velocity up to
`separation / dt` and arrests only beyond it. That is a change to what the
solver reads out of a `SolverContact`, and it needs its own design pass —
`warm_start_depth_slop` and the `MARGIN_PULL_BIAS` handling in the normal
constraint are the neighbouring concerns.

Both tests assert two bounds — not interpenetrating *and* not stopping short.
The first alone is satisfied by the mid-air stall and passes the broken fix.

---

## Terraced slopes in generated terrain

Sloped ground carries faint terraces — bands running along the contours, spaced
like the voxel lattice. Flat ground is clean, which is the diagnostic detail: a
surface at constant height has nothing to step between, so this is the surface
stepping between lattice planes rather than the general mottling that a
quantised density field would produce everywhere.

Long-standing and previously invisible. It surfaced when terrain gained detail
normals (VISUAL_DIRECTION.md §4.2), which give the eye enough surface structure
to read the banding; it is *not* caused by them, and survives setting the detail
strength to zero.

### Suspected cause, unverified

`generate_terrain` writes an exact sub-voxel density for the topmost solid voxel
of each column and its air cap, so the *vertical* crossing lands on the true
surface height. Neighbouring columns of differing height still carry saturated
±1 densities, so marching cubes' *horizontal* edges interpolate between
saturated endpoints and land on the lattice rather than on the surface. On flat
ground neighbouring columns agree and nothing steps; on a slope every column
boundary is a potential step.

If that is right, the fix is to give the samples flanking a surface crossing a
signed distance to the *surface* rather than to the top of their own column —
the same principle as the carve path, which already computes a real distance
(`csg::carve_density`, and the note on `Voxel::air`).

### Why it matters beyond looks

§4.4 slope zoning thresholds on the surface normal, and this artifact lives in
the normal. A zoning band tracking terraces instead of geometry would be much
harder to diagnose once the two are layered.

Reproduce with `cargo run --bin visual_bench -- terrain_forms` and look at the
open ground in the `slope sweep` tile.
