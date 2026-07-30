# Stage 1 — Chunked terrain

**Read `docs/LEVEL_SEGMENTS_PLAN.md` first.** It holds the design rationale; this brief
holds only what stage 1 does.

Branch: `level-segments-stage-1` off `main`. Merge before stage 1.5 begins.

---

## Goal

Replace the single level-wide cubic SVO with a **sparse grid of fixed-size cubic voxel
chunks**, each owning its own SVO and mesh. Behaviour must be observably unchanged: the
same level file produces the same terrain, at the same resolution, with the same physics
and rendering.

This stage introduces the chunk machinery under a **single implicit segment** covering the
whole world. Stage 2 generalises the segment count from one to N. Do not implement
segments, anchors, or placement here.

## Why it is worth doing before anything else

Two defects fall out of the current model, and both are fixed by construction rather than
by patching:

1. `Level::octree_depth()` returns `log2(world_size / voxel_size)` while the SVO bounds
   span `2 * world_size`, so actual voxel size is **twice** the declared `voxel_size`.
   Every level declaring `voxel_size: 1.0` runs 2.0m voxels. Chunk depth is fixed by the
   chunk definition, so the faulty calculation disappears rather than being corrected.
2. `generate_terrain` walks every `(x, z)` column in the full footprint regardless of
   content. Chunk-scoped generation makes cost proportional to what features touch.

## Deliverables

### `ChunkCoord` and `Chunk`

A chunk is a cube of `CHUNK_VOXELS³` voxels at the grid's voxel size. Suggested
`CHUNK_VOXELS = 32`; justify in `PROGRESS.md` if you pick otherwise. A chunk owns:

- one `SparseVoxelOctree` covering exactly the chunk's local AABB, at a depth fixed by
  `log2(CHUNK_VOXELS)` — **not** derived from any level-wide extent;
- its meshed output (vertices, indices) and a dirty flag.

`ChunkCoord` is an integer lattice coordinate. Chunk world extent is
`CHUNK_VOXELS * voxel_size`.

### `ChunkGrid`

`FxHashMap<ChunkCoord, Chunk>` plus the voxel size and grid origin. Chunks are allocated
**on demand** — a chunk exists only if something wrote to it. Provide coordinate
conversion (world position ↔ `ChunkCoord` ↔ chunk-local position) and AABB→chunk-range
iteration, and unit-test those conversions directly, including negative coordinates,
which are the usual source of off-by-one errors in lattice indexing.

### Chunk-scoped generation

Rework `generate_terrain` (`src/terrain/generation.rs`) so each feature reports a local
AABB and only intersecting chunks are visited. The heightfield pass currently iterates the
whole footprint (`generation.rs:27`); it must become per-chunk. Feature evaluation maths —
`height_at`, `contribute`, `apply_island`, `apply_caves`, and the rest — is correct and
should be reused as-is, not rewritten.

Preserve the sub-voxel SDF density encoding on surface voxels and the matching partial-air
voxel above (`generation.rs:41-62`). It is what makes marching cubes land the surface at
the exact height, and losing it will visibly step the terrain.

**Noise must be sampled in coordinates local to the grid, not world position.** Stage 1 has
a single segment at the origin so the two coincide and nothing observable changes — which is
exactly why it is easy to bake in the wrong one here and not notice until stage 2, when
moving a segment starts silently changing its terrain. Route noise sampling through a
local-coordinate path now.

### `TerrainManager` reimplemented over `ChunkGrid`

**Keep the type name `TerrainManager` for this stage.** Seventeen files reference it; the
rename to `TerrainWorld` belongs in stage 2, when it genuinely becomes multi-segment.
Keeping it here holds the stage-1 diff to the terrain module.

Every existing public method must keep its current signature and semantics:
`damage_sphere`, `update`, `bounds`, `voxel_size`, `is_solid_at`,
`approx_surface_height_at`, `surface_heights_at`, `dirty_regions`, `is_mesh_solid_at`,
`mesh_surface_height_at`, `mesh_surface_heights_at`, `has_geometry`, `render_vertices`,
`render_indices`, `texture`, `get_render_data_culled`, `adjacency`, `triangle_count`,
`leaf_count`, plus the `StaticGeometry` and `ProbeTarget` impls.

`render_vertices()` / `render_indices()` return flat slices and have exactly one consumer
(`src/systems/render.rs:219`). **Keep returning concatenated buffers.** Per-chunk draw
calls are a rendering optimisation, not part of this stage.

The existing dirty-region machinery maps onto per-chunk dirty flags: `damage_sphere` marks
touched chunks, `update` remeshes only those, `dirty_regions`/`rebuilt_regions` report
their world AABBs. Adjacency (`src/terrain/adjacency.rs`) is rebuilt per dirty region
today and must keep working across chunk boundaries — triangles meeting at a chunk seam
still need to find each other, or the physics feature-aware pipeline will see false
boundary edges.

### Level format

`world_size` and `voxel_size` in the level file no longer mean what they meant. Give
`Terrain` an explicit voxel size, drop the derived-depth path, and update the validation
in `src/level/loader.rs` accordingly. Port the two surviving level files. Delete the other
eight:

```
levels/beach_ball_bowling.level.ron   levels/crate_canyon.level.ron
levels/demolition_derby.level.ron     levels/grenade_gauntlet.level.ron
levels/rube_goldberg.level.ron        levels/shattered_isles.level.ron
levels/sky_islands.level.ron          levels/the_great_escape.level.ron
```

Keep and port `levels/test_empty_terrain.level.ron` and `levels/test_arena.level.ron`.
`test_arena` holds one copy of every spawnable and is the primary fixture — it must load
and look right.

Note that porting at a *corrected* voxel size changes how these levels look, because they
were implicitly tuned against 2.0m voxels. That is intended. Re-tune `test_arena` so
objects still rest on terrain rather than floating or intersecting.

## Out of scope

Do not implement any of the following, even if the code seems to invite it:

- Segments, anchors, placement, or any multi-frame coordinate handling.
- Renaming `TerrainManager` to `TerrainWorld`.
- Per-chunk draw calls, LOD, or any rendering optimisation.
- Chunk streaming, background generation, or unloading.
- New terrain or traversal primitives (`Path`, `Platform`, `Staircase`).
- `level_check`, schematic export, or the viewer — that is stage 1.5.
- Water rework. `create_level_water` derives extents from `world_size`; do the minimum to
  keep it compiling and working for the two surviving levels, and record what you did in
  `PROGRESS.md`. The real answer is deferred to stage 2.

## Acceptance

All of these must pass:

```bash
cargo build
cargo test
cargo test --release --features bench_harness
```

The bench harness matters here: it exercises physics against static geometry, which is the
subsystem most likely to break silently if chunk seams produce bad triangle patches.

New tests required:

- `ChunkGrid` coordinate conversion round-trips, **including negative coordinates**.
- Chunk allocation is sparse: generating a level with one small feature allocates a number
  of chunks proportional to the feature, not to the world bounds.
- Voxel size is what the level file declares — a direct regression test for the
  factor-of-two defect.
- Terrain surface height is continuous across a chunk boundary (sample
  `mesh_surface_height_at` either side of a seam; no discontinuity beyond one voxel).

Manual check, since no automated visual check exists until stage 1.5: load
`levels/test_arena.level.ron` and confirm terrain renders without cracks at chunk seams
and that objects rest on the surface.

## On completion

Append to `docs/level_segments/PROGRESS.md`: what landed, where you deviated from this
brief and why, the `CHUNK_VOXELS` value you chose and the reasoning, what you did about
water, and anything the stage 2 agent will trip over. Note in particular any place where
you had to assume something about how segments will work — stage 2 needs to know where
those assumptions are buried.
