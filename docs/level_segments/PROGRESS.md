# Level Segments — Progress

Handoff channel between stage agents. Each agent appends its own section on completion.
Read the whole file before starting a stage: it records where earlier stages deviated from
their briefs, which the briefs themselves cannot.

Design: `docs/LEVEL_SEGMENTS_PLAN.md`. Stage briefs: `docs/level_segments/stage-*.md`.

---

## Stage 1 — Chunked terrain

**Landed** on branch `level-segments-stage-1`. `cargo build`, `cargo test` (554 pass) and
`cargo test --release --features bench_harness` (595 pass) are all green.

### What landed

New in `src/terrain/`:

- `chunk.rs` — `CHUNK_VOXELS`, `ChunkCoord`, `ChunkTriangleRef`, `Chunk`.
- `chunk_grid.rs` — `ChunkGrid`: sparse `FxHashMap<ChunkCoord, Chunk>`, one origin, one
  voxel size, coordinate conversion, AABB→chunk iteration.

Reworked:

- `manager.rs` — `TerrainManager` reimplemented over `ChunkGrid`. Every listed public
  method keeps its signature and semantics; `from_svo` is replaced by `from_grid`.
- `generation.rs` — writes into a `ChunkGrid`. The heightfield pass is per chunk column;
  the volumetric feature maths is untouched apart from the `&mut SparseVoxelOctree` →
  `&mut ChunkGrid` parameter swap (both expose the same `get`/`set`).
- `marching_cubes.rs` — `generate` replaced by `generate_range`, which takes a cell
  sub-range and a global sample-index base.
- `mesh_octree.rs` — `generate_from_voxels` replaced by `generate_block`.
- `adjacency.rs` — `AdjacencyMap` is now generic over the triangle identifier.
- Level format: `Level.world_size` / `Level.voxel_size` / `Level::octree_depth` are gone;
  `Terrain` gains `voxel_size` and `bounds: Extent`.

### `CHUNK_VOXELS = 32`

Kept the brief's suggestion. 32 voxels gives an octree depth of 5 and 32 768 voxels per
chunk. At 1 m voxels a chunk is a 32 m cube, which is roughly one authored "area" — small
enough that a whole-chunk remesh is cheap (**≈1.1 ms per chunk** in release, and a
3 m-radius grenade dirties 8), large enough that the seam-neighbour overhead described below
stays a minority of chunks. Halving it to 16 was tried: `test_arena` went from 114 chunks to
549 for the same geometry, with no benefit.

One cost note: `update()` also rebuilds the whole concatenated render buffer, so a single
grenade pays for every triangle in the level (573 k vertices on `test_arena`). That is not a
regression — the previous code rebuilt the whole buffer too — but per-chunk draw calls,
explicitly out of scope here, would fix it.

### Cell ownership and the seam-neighbour shell

This is the one structural decision worth reading before touching the chunk layer.

A marching-cubes cell spans two sample planes, so a cell on a chunk boundary could be
emitted by either side. Ownership is fixed: **chunk `c` emits exactly the cells inside its
own bounds** — grid cell indices `[cN, (c+1)N)`. Every triangle therefore lies within its
own chunk, and no cell is meshed twice. That last part matters for physics: duplicate seam
triangles would surface as duplicate contacts, which the physics notes already record as a
source of phantom forces. There is a test (`seam_triangles_are_not_duplicated`).

The consequence is that the cells closing a solid chunk's *minimum* faces belong to the
neighbour on that side. If that neighbour does not exist, those faces are simply missing —
most visibly the underside of the terrain. So `ChunkGrid::allocate_seam_neighbours()`
allocates the seven negative-octant neighbours of every chunk holding a solid voxel, and
`prune_vacant()` drops the ones that turned out to contribute nothing.

For `test_arena` this is not a rounding error: 114 chunks total, of which roughly half exist
only to own seam cells. It is not waste — those chunks hold real cap geometry — but it does
mean **chunk count is not a proxy for content volume**, which `level_check` should know.

Two supporting details:

- `MeshOctree::generate_block` samples one index beyond the block on every side, so
  marching-cubes gradients use central differences at every owned cell corner. Without the
  halo, normals would differ between the two sides of a seam and shade as a visible crease
  even though the geometry matched.
- `MarchingCubes::generate_range` computes vertex positions from the **global** sample index
  (`origin + index * cell_size`), not as an offset from a per-block origin. Two chunks
  sharing a sample plane therefore produce bit-identical vertices there.

### Seam quality — measured

- A solid slab spanning the `x = 32` chunk boundary meshes to a **watertight** surface:
  zero open edges (`mesh_has_no_open_edges_across_a_chunk_seam`).
- `test_arena` as a whole has **218 open edges out of 191 116 triangles** (0.04%);
  `test_empty_terrain` has 22 of 98 948. These are not seam-related: quartering the chunk
  volume (`CHUNK_VOXELS = 16`, 4.8× the chunks and so ~2× the seam area) moved the arena
  count only from 218 to 242. They come from ambiguous marching-cubes configurations in the
  cave noise, which is pre-existing behaviour. Recorded here as a **baseline for stage 1.5**:
  if `level_check` grows a manifold check, these are the numbers to compare against.

### Deviations from the brief

1. **A chunk owns a `MeshOctree`, not a bare vertex/index pair.** The brief says "its meshed
   output (vertices, indices) and a dirty flag". `TerrainManager` has to answer AABB and ray
   queries, and `MeshOctree` already does that; storing flat buffers would have meant
   reimplementing spatial indexing per chunk. `MeshOctree::get_render_data()` still yields
   the flat buffers, which is what `refresh_caches` concatenates.

2. **Remeshing is whole-chunk, not sub-region.** With chunk-sized meshes the region-scoped
   rebuild machinery had no remaining caller, so `generate_from_voxels`,
   `clear_region_recursive`, `rebuild_neighbor_refs_region` and their five helpers were
   deleted (~560 lines) along with their two tests, and `rebuild_neighbor_refs` was promoted
   out of `#[cfg(test)]`. Adjacency stays incremental — `update_region` is fed one chunk's
   triangles at a time — so its cost is still proportional to the change, just quantised to
   chunks. Leaving that code in place as unreachable would have been worse.

3. **`Terrain` gained `bounds`, which the brief did not ask for.** Dropping `world_size`
   removes the only thing that told the heightfield pass where to stop, and a heightfield is
   defined everywhere in xz. `bounds` is the generation extent, not a reservation: chunks are
   still allocated only where something is written, and the arena test asserts that raising
   the ceiling by 128 m allocates zero extra chunks.

4. **`bounds()` now returns the union of allocated chunk bounds**, not a declared extent.
   That is the tight, derived form Rule 4 of the plan calls for, and it was cheaper to do now
   than to retrofit. Note it can be *larger* than the authored `Terrain.bounds` by up to one
   chunk on the negative side, because of the seam shell.

5. **The voxel lattice is anchored to the grid origin, not to `bounds.min`.** It has to be,
   for chunks to align. Where the authored bounds are not a multiple of the voxel size, voxel
   positions shift by a sub-voxel amount relative to the old behaviour. Both shipped levels
   use chunk-aligned bounds, so nothing moved.

6. **Validation changed shape.** The power-of-two ratio and depth-range checks are gone —
   they described a derivation that no longer exists. Replaced with: positive `voxel_size`,
   positive extent on every axis, a 4096-voxels-per-axis ceiling (chunks are sparse, but the
   heightfield pass still walks every column, so an absurd extent is an authoring error), and
   `player_spawn` inside the terrain bounds.

### Water

Minimum change, as instructed. `create_level_water` took its extents from `world_size` and
assumed a symmetric grid centred on the origin; it now takes them from
`level.terrain.bounds` and allows different width and depth. Origin is the bounds' `(min.x,
0, min.z)` corner instead of `(-world_size, 0, -world_size)`. Still one world-space grid.
`test_arena`'s pool works unchanged, and the grid is far smaller than before (96×80 flow
cells instead of 256×256) because the extent is now authored to the content.

The real question — per-segment grids or one grid spanning the segment union — is untouched
and still belongs to stage 2.

### Level files

Deleted the eight listed. Ported `test_arena` and `test_empty_terrain`:

- `test_arena`: was `world_size: 256, voxel_size: 1.0`, which actually ran **2 m** voxels
  over a 512³ box. Now genuine 1 m voxels over `(-128, -32, -96) .. (64, 32, 64)`, sized to
  the content (objects, the cave region reaching x ≈ -95, the overhang along z = -60) and
  landing on the 32 m chunk lattice. Objects did **not** need re-tuning: the sub-voxel SDF
  encoding lands the surface at the exact authored height regardless of voxel size, so
  everything resting at y = 0 still rests at y = 0. The terrain is visibly more detailed and
  the caves are finer.
- `test_empty_terrain`: same treatment, `(-64, -32, -64) .. (64, 32, 64)`. Also renamed from
  "Test Arena" — it had been copy-pasted.

### Not done

**The manual visual check in the brief was not performed.** The game window cannot be
launched from the agent shell in this environment. The watertightness and seam-continuity
tests above are the substitute, and they are stronger than an eyeball for cracks — but they
say nothing about *shading* or about objects visually resting on the surface. Someone should
run `cargo run` against `test_arena` before this merges.

### For stage 2

Assumptions about segments that are baked in, and where:

- **`ChunkGrid` already has a local frame.** It stores an `origin` and offers
  `to_local`/`to_world`/`aabb_to_local`/`aabb_to_world`; `TerrainManager` converts at every
  query boundary and the generation path is entirely local. Stage 1 constructs the one grid
  at `Point3::origin()`, so those conversions are identity today — but they are real code on
  real paths, not stubs. **Translation is exercised only by unit tests**, so treat the first
  non-zero origin as the moment to re-run the seam tests.

- **Noise is sampled in grid-local coordinates**, as the plan requires. `generate_terrain`
  takes a grid-local `bounds` and every `height_at` / `fbm_*` call receives a local position.
  Nothing reads world position. Moving a grid will not change its terrain.

- **Rotation is not accommodated anywhere.** `to_local`/`to_world` are pure translation.
  Rule 2's 90° yaw needs an axis swap adding to those two functions plus `coord_at`,
  `chunk_bounds` and the AABB conversions. The places are localised but they are not written
  for it.

- **Render vertices are baked into world space.** `refresh_caches` adds the grid origin to
  every vertex as it concatenates. That is correct but it is the wrong long-term shape — a
  per-segment model transform in the renderer is. Cheap to change: it is three lines in
  `refresh_caches` and three in `get_render_data_culled`.

- **`TerrainManager` owns exactly one `ChunkGrid` field.** Becoming `TerrainWorld` means
  turning that field into a collection and adding the segment broadphase in front of the
  existing per-chunk dispatch in `query_region`, `ray_cast_all` and `refresh_caches` — those
  three already iterate chunks and union results, so the shape is right.

- **Adjacency is keyed by `ChunkTriangleRef { chunk, triangle }`** and matches triangles by
  quantised **world** position. Adding a segment field to that key is mechanical. Note that
  because matching is positional, adjacency will link triangles across a *segment* join too,
  wherever two segments' geometry happens to be coincident — probably desirable, but it is
  not a decision anyone made, so decide it deliberately.

- **Chunk contention (Rule 4) is unchecked.** Two grids overlapping would silently produce
  duplicate geometry in `query_region` — exactly the duplicate-contact failure the ownership
  rule avoids within a grid. `ChunkGrid::coords()` gives the allocated set for the check.
