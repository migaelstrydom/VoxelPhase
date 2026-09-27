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

## Speculative contacts: no prediction for a collider already touching the ground

Static-geometry prediction runs only for a collider with no static contacts this frame,
so a ball rolling along the floor toward a wall at band speeds (roughly 2–18 m/s for a
0.2 m sphere at 60 Hz) gets no speculative contact against the wall and overlaps it by up
to a frame's travel before the discrete manifold sees it. Body pairs are not affected —
each pair is predicted on its own.

The fix is to predict against the geometry the current manifold does not already cover
and merge the two, which needs a rule for a manifold that holds both real and predicted
points under the four-point limit.

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
