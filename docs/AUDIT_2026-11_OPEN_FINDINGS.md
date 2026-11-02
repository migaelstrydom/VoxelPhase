# Old-code audit, November 2026: what is still open

In November 2026 nine read-only reviewers audited the code older than
2026-05-02: about 70k of 237k lines by `git blame`, one reviewer per area. The
physics bench-harness tests were left out.

- **Correctness:** every finding was fixed, each in its own commit with a test
  or an in-game check (branch `audit-fixes`, on main at `50f0abb`).
- **Code smell:** the dead code, misleading names and smaller duplications were
  cleaned up on branch `smell-cleanup`.

This file holds what is left: the performance findings, and the smell
findings that were held back because each needs a decision first.

**How to read it:**

- **✔** means the finding was checked against the code by hand. Anything
  unmarked is a reviewer's reading, not yet checked or measured.
- `file:line` references are as of `50f0abb` and will drift.
- Measure performance in release with validation off (`render_perf`,
  `physics_perf`, `terrain_perf`). The validation layer alone costs several
  ms of render CPU a frame (see CLAUDE.md).

## Performance

Ranked by likely payoff.

1. ✔ **No frustum culling.** Nothing culls the main pass or the shadow casters.
   - All ~880 draws on Stack Yard are recorded twice. MoltenVK encodes them all
     inside `vkQueueSubmit`, about 1.2 ms of CPU, and the GPU transforms them
     all.
   - `ViewVolume` exists but only the reflection probes use it.
   - Where: `rendering/renderer.rs` `submit_draw`.
2. ✔ **A texture per block.** Each block of the old spawnables bakes its own
   fbm texture and uploads it at 128².
   - Stack Yard has about 653 of them: 54 MiB, 653 descriptor sets, and a
     set-1 bind per draw.
   - `TextureCache` exists, but these spawnables bypass it.
   - Each upload is three serialized submits with two fence waits. The
     sequence now lives in one place, `TransferService::upload_texture`, so
     batching it is a single change.
   - Where: `box_object.rs`, `jenga`, `pyramid`, `domino`, `hex_prism`,
     `beach_ball`.
3. **Sleep is a hash set consulted inside loops over all bodies**, not a list
   of the awake ones. A quiet level pays for every body it has.
   - ✔ Buoyancy runs for every dynamic body every substep, asleep or not.
     Three reviewers found this independently.
   - The sleeping set is cloned about 17 times a frame.
   - The broadphase sorts every collider every frame.
   - CCD snapshots every collider whenever any candidate exists.
   - `physics_sync` copies every body.
   - `sync_bodies` and `contact_generation_positions` rebuild maps over all
     bodies.
   - `is_kept_awake` scans every constraint for each body.
4. **Ray casts brute-force every collider**, while the bounds index
   (`colliders_meeting`) sits unused.
   - Aiming makes up to 36 casts a frame, plus the foot probes.
   - The terrain probe builds a full adjacency patch per ray.
   - Where: `physics/world.rs` `raycast_bodies`, `terrain/world.rs`.
5. **Terrain vertices are never shared.**
   - Each triangle carries 3 × 52 B. The largest level is about 95 MB, held in
     three CPU copies.
   - Any edit rebuilds and re-uploads the whole level buffer.
   - Where: `terrain/marching_cubes.rs`, `mesh_octree.rs`, `terrain/world.rs`.
6. **Debug output is built every frame.** The DebugLog and the physics contact
   snapshot are built (about 100 `format!` calls, plus O(contacts) work) and
   then thrown away unless F3 was pressed. Where: `debug.rs`,
   `physics/debug.rs`, `systems/physics_sync.rs`.
7. **Static terrain contacts:**
   - about 8 allocations per collider per frame;
   - triangle normals recomputed up to 8 times;
   - a second query on the speculative path;
   - no cache for an unchanged AABB.

   Where: `physics/narrowphase/static_contacts.rs`,
   `collision/mesh/seam_filter.rs`.
8. **Hull paths:**
   - The hull–hull edge loop rotates B's normals inside the O(Ea·Eb) loop:
     barrel against barrel is about 41k quaternion rotations.
   - EPA allocates on every expansion.
   - Each contact makes about 12 `support_face` probes.
   - GJK runs all 32 iterations when its first support is already exact,
     because of the `count >= 2` guard.

   Where: `hull_hull.rs`, `epa.rs`, `gjk.rs`.
9. **Buoyancy per part:**
   - It makes three full polyhedron clips, two of them only to estimate the
     waterplane, with about 90 allocations.
   - Sea swell wakes sunken sleepers every few frames.
   - Despawned bodies leak from the water sleep tracker.

   Where: `water/buoyancy.rs`, `water/sleep_tracker.rs`.
10. **Streamed buffers map and unmap per append**: per rig, per debug shape
    and per procedural mesh. Where: `rendering/frame.rs`,
    `in_flight/streamed_mesh.rs`.
11. **Rig meshes are rebuilt from scratch** per character per frame, with
    about 60 allocations (`animation/rig/mesh.rs`). Verlet clones its distance
    constraints on every solver iteration (`skeleton/verlet.rs`
    `solve_distance_constraints`).
12. **Particles:**
    - four full vertices per particle, not instanced;
    - a constant index buffer rebuilt every frame;
    - the partition allocates;
    - spawning into a full pool scans all 10k particles with
      `partial_cmp().unwrap()`.

    Where: `particles/renderer.rs`, `particles/particle.rs`.
13. **Smaller items:**
    - two allocations per model per frame (`to_vec`, an identity-transform vec);
    - the manifold cache is written back every substep;
    - position-correction scratch is allocated every substep;
    - the fire Jacobi solve wastes its last iteration and uses RGBA16F for a
      scalar;
    - the terrain sorts every chunk coordinate on each idle frame;
    - `build_edge_adjacency` is O(E²);
    - level generation writes voxels one at a time;
    - the fire sync is O(fires × burning entities).

Already gone on `smell-cleanup`:

- the discarded FABRIK solve per leg;
- the Verlet angle-constraint clone;
- the per-frame kinematic-sync join;
- the opaque debug-overlay draw.

## Code smell held back

Each item below is a redesign rather than a cleanup. It either changes
behaviour you have tuned, or touches a great deal of code. Decide before
starting.

### Changes contact behaviour

- **The SAT driver, written three times** (`obb_obb`, `hull_obb`,
  `hull_hull`). Each finds the axis of least overlap over faces and edge pairs,
  then builds the contacts; the copies already differ in their details. One
  generic driver over a shape trait would end the drift, but it changes which
  axis wins in near-ties, and so changes contacts. Do it only with the physics
  scenario suite and `physics_fuzz` as the judge, and expect to re-baseline.
- **`sphere_patch` is a copy of `capsule_patch`** with a zero-length segment.
  Folding spheres into the capsule path, as a degenerate capsule, deletes
  about 300 lines. But the capsule path's parallel-span logic would then run
  on spheres, so sphere contacts against terrain could change.

### Large, without changing behaviour

- **Graphics pipeline creation, written nine times.** Every renderer builds
  its own `GraphicsPipelineCreateInfo` from scratch, while `PipelineFactory` /
  `GraphicsPipelineConfig` in `rendering/pipeline.rs` covers only some of them. Moving
  every pipeline onto the config is mechanical but broad; check each with
  `render_perf` and `level_viewer` under validation.
- **Spawnable body setup, written nine or more times.** Most spawnables
  repeat the same create-body, attach-colliders, build-entity sequence with
  `Position`, `Velocity`, `Orientation`, `RigidBodyComponent`, `ModelInstance`
  and `Renderable`. One builder would hold the sequence.
- **Eight procedural wood generators** across the spawnables, each its own
  fbm grain. One parameterised generator, or a wood material in the texture
  cache (which perf item 2 wants anyway).
- **`LevelObject` restates every Def field by field.** Its `to_spawnable` is
  about 700 lines. Each spawnable's Def could be the level format's own type,
  with serde deriving the parse, so a new spawnable no longer edits two places.
  It touches the level loader and every Def.
- **Hull validity is held by asserts in the builder**, and callers copy the
  builder's thresholds to avoid tripping them. A fallible constructor, or a
  validated `ConvexHull` type returned by the builder, would let callers ask
  instead of copy.

### Documented, not changed

- **Box textures are mirrored on every face.** u follows the cube's clockwise
  corners, so each face shows its texture flipped left to right
  (`geometry/cube.rs`, where it is now documented). Fixing it is a two-line
  change that flips every box texture in the game, so it is a look decision.

## Correctness left open

- **Presentation support is not checked at device creation.** The device is
  created before the window's surface, so the chosen queue family is never
  checked against `vkGetPhysicalDeviceSurfaceSupportKHR`. That is harmless on
  every GPU we run on (the one family that draws also presents), but it should
  be checked if the device is ever created after the surface.

## Not covered by the audit

- the physics bench-harness tests;
- newer code inside old files: the foot placer, the locomotion state machine,
  crease contacts;
- `geometry_draw`, `resident` and `transparency` in the renderer;
- the water renderer's internals.
