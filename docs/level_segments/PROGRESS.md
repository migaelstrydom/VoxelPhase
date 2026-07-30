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

---

## Stage 1.5 — level_check, schematic export, timing

**Landed** on branch `level-segments-stage-1-5`. `cargo build`, `cargo test` (568 + 2 pass) and
`cargo test --release --features bench_harness` (609 + 2 pass) are all green. Both shipped
levels pass `level_check` with exit code 0 and no warnings.

### What landed

New library module `src/level_check/` — everything runs headlessly, so the checks are
testable and the binary is a thin CLI:

- `report.rs` — `Severity`, `Finding`, `Section`, `Report`. The checks never print; the
  binary formats and the tests assert against the same structure.
- `runner.rs` — `build_terrain` (generate + mesh with no graphics device) and `check_level`,
  which assembles the statistics, mesh-integrity, placement and reach output.
- `placement.rs` — player spawn and object placement checks.
- `reach.rs` — `JumpEnvelope::derive(&PlayerConfig, gravity)`.
- `baseline.rs` — committed open-edge baselines and the comparison policy.
- `svg.rs` — the two-panel schematic.

New binary `src/bin/level_check.rs`; new data file `levels/mesh_baselines.ron`.

Terrain changes:

- `TerrainManager::from_grid_headless` — meshes without a `TextureManager`, which is what
  lets any tool or test build real terrain.
- `TerrainManager::open_edge_count`, `solid_chunk_count`, `chunk_extent` — accessors the
  check needs; `boundary_edge_count` lost its `#[allow(dead_code)]`.
- `UpdateTimings` + `TerrainManager::last_update_timings()`, reported through `DebugLog` by
  `TerrainUpdateSystem`.
- `LevelObject::describe() -> ObjectInfo { kind, placement }` in `level/data.rs`, with
  `ObjectPlacement::{Free, TerrainAnchored}`.
- `PhysicsConfig` is now re-exported from `physics` (it was only reachable via the private
  `physics::world` module).

### The measured timing split — this is the deliverable

`cargo test --release --lib -- --ignored --nocapture grenade_update_cost` on `test_arena`
(114 chunks, 191 116 triangles, 573 348 vertices), explosion defaults (2.5 m crater):

| Grenade | Chunks dirtied | Remesh | Adjacency | Buffer concat | Total |
|---------|----------------|--------|-----------|---------------|-------|
| at a chunk corner (0, 0, 0) | 8 | **18.5 ms** | 6.2 ms | 6.3 ms | 31.0 ms |
| mid-chunk (16, 0, 16) | 2 | 4.5 ms | 1.4 ms | 3.7 ms | 9.6 ms |
| mid-chunk (−20, 0, 8) | 2 | 5.1 ms | 2.4 ms | 3.3 ms | 10.8 ms |

**Remesh dominates today, not buffer concatenation.** At the current level size the
O(chunks dirtied) terms (remesh + adjacency) are 60–80 % of the cost and the O(level size)
term is 20–35 %. Two consequences:

1. The first fix should be per-chunk cost — a smaller `CHUNK_VOXELS`, or sub-chunk dirty
   regions. Stage 1 rejected `CHUNK_VOXELS = 16` on storage grounds; on *this* metric it
   would cut per-chunk remesh work eightfold — a whole-chunk remesh meshes all
   `CHUNK_VOXELS³` cells regardless of how little changed. It would also raise the
   chunks-dirtied count, so the net win is well under 8×, but a fixed-radius blast touches a
   bounded number of chunks either way while the per-chunk cost falls with the cube.
2. **The tripwire has not tripped yet, but it is close.** Concatenation costs ~1.1 ms per
   100 k vertices. It overtakes an 8-chunk remesh at roughly 1.7 M vertices — about 3× the
   current arena. A level three times the size of `test_arena` is not ambitious, so per-chunk
   GPU buffers will be needed before levels grow much, exactly as the plan predicted.

Worth knowing: **a grenade dirties 2 chunks or 8, depending only on where it lands.**
`damage_sphere` marks every chunk in the (radius + voxel) AABB, so a blast near a chunk
corner touches all eight. That 4× swing in cost is a placement accident, and sub-chunk dirty
regions would remove it.

### Open-edge baselines — committed

`levels/mesh_baselines.ron`, keyed by level file name and read from the level's own
directory (so it is found by path, not by working directory):

| Level | Triangles | Open edges |
|-------|-----------|------------|
| `test_arena.level.ron` | 191 116 | 218 |
| `test_empty_terrain.level.ron` | 98 948 | 22 |

Both reproduce stage 1's measurements exactly. The policy: an error above
`baseline * 1.25` (with an absolute floor of +8 so small baselines are not hair-triggered),
a warning if the count falls more than 25 % below (the baseline has gone stale and stops
catching regressions), and a warning if a level has no committed figure at all.

### Chunk split — stage 1's estimate was optimistic

Stage 1 said "roughly half" of `test_arena`'s chunks are seam shell. Measured:
**32 of 114 hold solid voxels; 82 are shell** — 72 %, not 50 %. `test_empty_terrain` is
16 solid of 66. The shell is proportionally larger than expected because a chunk is 32 m and
both levels are broad and thin, so nearly every solid chunk contributes three new face
neighbours and their edge/corner partners. This matters for stage 2's Rule 4 check: an
allocated-extent test would be wrong by a *lot*, not by a little.

### Deviations from the brief

1. **The checks live in the library (`src/level_check/`), not in the binary.** The brief put
   `level_check` in `src/bin/level_check.rs`. Four of the required tests are tests *of the
   checks*, and a bin's test target cannot be reached from the library's. The binary is
   argument parsing and formatting only, ~150 lines.

2. **Not registered in `Cargo.toml`.** The brief asked for it "alongside `bench_viewer`" —
   but `bench_viewer` is not declared either; both are picked up by cargo's `src/bin`
   autodiscovery. Adding one explicit `[[bin]]` and leaving the other implicit would have
   been worse than consistency.

3. **"Object in rock" needs a burial margin.** A centre that merely reads as solid flags
   half a level, because objects are routinely authored resting exactly on the surface.
   The rule is: solid at the point *and* solid half a voxel above it. That distinguishes
   buried from resting, and — because the point itself must be solid — leaves an object
   sitting inside a cave alone. There is a test for the resting case specifically.

4. **The player-spawn fall limit is not derived.** The brief says "within a fall the player
   could survive". There is no fall-damage system, so nothing to derive from; the check uses
   a documented 20 m constant, and what it actually catches is a spawn over a void. Revisit
   if fall damage lands.

5. **Objects are numbered in the schematic, not labelled.** Named labels overlapped
   unreadably where objects cluster (which is everywhere in `test_arena`). Markers carry an
   index and a key below the panels resolves them, with coordinates — which also makes the
   picture answer "where is object 17" without opening the RON.

6. **`levels/*.ron` is no longer synonymous with "a level".** `mesh_baselines.ron` lives
   there too, so `shipped_levels_parse_and_validate` now matches `*.level.ron` rather than
   any `.ron`.

### The schematic

Committed at `docs/level_segments/arena.svg` (45 KB, renders in any browser). Two panels
over one shared horizontal scale:

- **Plan** — surface height as a shaded heightmap (quantised to 24 shades and run-length
  merged along each row, which is what keeps the file small), chunk lattice as faint white
  lines, numbered object markers coloured by whether their height is authored or resolved
  from terrain, a green diamond for the spawn, axis ticks on the chunk lattice, a scale bar
  and a ramp legend.
- **Elevation** — the terrain's vertical envelope along z, split at columns with no terrain
  so a void reads as a gap. Objects at their authored height, `y = 0` marked.

On `test_arena` it reads correctly: the overhang along z = −60 is the pale slab, the crater
is the dark disc, the three cave mouths are the notches in the elevation band, and the
object cluster sits around the origin.

Note the plan is drawn over the *derived* bounds, so `test_arena` shows an empty strip from
x = −160 to −128 — that is the seam shell, and it is honest rather than a bug.

Styling is inline on every shape rather than in the `<style>` block: viewers with partial
CSS support (PyMuPDF, some SVG-to-PNG converters) filled every shape black otherwise.
Only fonts are left to CSS.

### Not done

- No gap-vs-reach validation. There is nothing to measure gaps between until stage 2 has
  anchors; the reach envelope is reported but nothing is validated against it.
- No performance fix, as instructed. `CHUNK_VOXELS` is untouched.

### For stage 2

- **`check_level` takes one `Level` and one `TerrainManager`.** With segments it becomes
  per-segment stats plus whole-level checks. The sections are already a `Vec`, so N terrain
  sections is natural; `Report` needs no change. Placement checks take a point and ask the
  terrain about it, so they generalise as soon as `TerrainWorld` answers `is_mesh_solid_at`
  and `mesh_surface_heights_at` across segments.
- **Rule 4 (chunk contention) is the obvious next check**, and it belongs in `level_check`
  as an error. `ChunkGrid::coords()` plus `Chunk::has_solid()` — now surfaced as
  `TerrainManager::solid_chunk_count()` — give the data; the check must compare
  *solid-chunk* world extents, never allocated or derived bounds. The 72 % shell figure
  above is why.
- **Anchor gap validation should compare against `JumpEnvelope`,** which is already derived
  from live tuning and exposes `max_flat_range()` and `max_apex()`. Apply a margin: the
  figures are point-mass and optimistic.
- **The schematic is single-frame.** `HeightField::sample` walks one `TerrainManager`, and
  the plan/elevation projections assume one world-space lattice. With segments it wants a
  per-segment sample loop into a shared world-space field, plus segment outlines and names
  drawn over the plan — that outline is probably the single most useful thing the picture
  could gain in stage 2.
- **The open-edge baseline is per level file, not per segment.** If a segment is instanced
  twice the count doubles, which is correct but means baselines must be re-committed
  whenever placement changes. Consider moving to per-segment-definition counts if that
  becomes annoying.
