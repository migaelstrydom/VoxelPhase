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
