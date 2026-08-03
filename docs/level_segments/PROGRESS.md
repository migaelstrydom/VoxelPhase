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

---

## Stage 2 — segments, anchors, placement

**Landed** on branch `level-segments-stage-2`. `cargo build`, `cargo test` (611 + 2 pass) and
`cargo test --release --features bench_harness` (652 + 2 pass) are all green. All three
levels pass `level_check` with exit code 0 and no findings.

### What landed

New in `src/terrain/`:

- `frame.rs` — `SegmentFrame`: a translation plus a yaw in whole quarter turns, with
  `to_local`/`to_world`, exact AABB conversion, `compose` and `inverse`.
- `anchor.rs` — `Anchor` (a named local frame), `outward`, and `mate`, which solves for the
  child segment frame that puts two anchors `gap` apart facing each other.
- `segment.rs` — `Segment`: name + frame + `ChunkGrid` + adjacency + anchors + lifecycle
  state. Owns everything that used to be the per-grid half of `TerrainManager`.
- `world.rs` — `TerrainWorld`, replacing `TerrainManager`. Same public query surface,
  fanned out across segments.

New in `src/level/`:

- `placement.rs` — `resolve_placements`, `world_anchor`, and `PlacementError` with a
  message per failure mode.
- `data.rs` gains `SegmentDef`, `AnchorDef`, `Placement`, `Connection`, `Level::frames`,
  and `LevelObject::place_in`.
- `loader.rs` resolves placement at load and lifts objects and the spawn into world space.
- `spawner.rs` gains `build_segments`.

New in `src/level_check/`:

- `segments.rs` — per-segment statistics, the Rule 4 contention check, the connection and
  gap-vs-reach checks, and `Crossing` (which jump a gap demands).
- `svg.rs` reworked: segment outlines and names, anchor arrows, connection links carrying
  their gap, de-conflicted labels, and a content-fitted elevation range.

New level: `levels/test_segments.level.ron` — "Harbour Ascent", four segments.

### The anchor facing convention

**An anchor's local `+X` points outward, out of the segment.** Mating is therefore a 180°
relative yaw, with `gap` separating the two origins along the parent's outward direction.

```text
       segment "plaza"                    segment "tower"
  ┌───────────────────────┐   gap   ┌───────────────────────┐
  │              exit_east│         │entry                  │
  │                    ●──┼──▶ +X   │  +X ◀──●              │
  └───────────────────────┘         └───────────────────────┘
```

A quarter turn maps local `+X` onto world `−Z` (nalgebra's rotation sense about `+Y`), so
the face table is: `+x` → yaw 0, `−z` → yaw 90, `−x` → yaw 180, `+z` → yaw 270. If a
segment lands *inside* its neighbour, its anchor is facing inward. The diagram and the
table are in `src/terrain/anchor.rs` and `docs/LEVEL_FILE_FORMAT.md`.

### Non-identity frames — what actually broke

The brief said to expect the seam tests to find something at the first non-zero origin.
**They did not.** Every seam, watertightness and duplicate-triangle test passes unchanged
at a non-zero origin and at all four yaws, first time. That is a direct dividend of stage
1 keeping generation and meshing entirely grid-local: there was no world coordinate left
in the meshing path for a frame to disturb.

What did need care, and would all have been *silent* rather than loud:

1. **Ray direction.** `ray_cast_all` converted the origin into local space but the
   direction has to be rotated too, and the resulting hit **normal** rotated back. Under a
   pure translation this is invisible, which is exactly why it survived stage 1.
2. **Render normals.** Baking the frame into vertex positions is obvious; baking it into
   vertex normals is not. A rotated segment shaded with unrotated normals lights from the
   wrong direction and looks plausible. There is a test.
3. **AABB corner ordering.** `AABB::new` does not sort its corners, so transforming
   `min`/`max` under a quarter turn and handing the results straight to `AABB::new`
   produces an inside-out box that silently matches nothing. Both `SegmentFrame` AABB
   conversions re-span from the two transformed corners.

### Deviations from the brief

1. **`ChunkGrid` lost its origin; the frame moved up to `Segment`.** The brief left the
   split open; the plan's architecture diagram puts the frame on the segment, and that is
   the cleaner line — `ChunkGrid` is now purely "voxels in one frame at one resolution"
   and has no idea where it is. `chunk_world_bounds` and `allocated_world_bounds` became
   local (`chunk_bounds`, `allocated_bounds`) with `Segment` doing the conversion.

2. **Objects moved into segments and are authored segment-locally.** The brief did not ask
   for this, but leaving objects in one flat world-space list would have kept the original
   problem — "absolute coordinates do not compose" — alive in the half of the format an
   author touches most. `load_level` resolves placement and then rewrites object positions
   and `player_spawn` into world coordinates, so every downstream consumer
   (`level_check::placement`, the schematic, the spawner) is unchanged.

   **Limitation, deliberate:** only the *placement* is transformed. Half-extents, row
   directions and column layouts are authored in the spawnable's own axes, so an object in
   a yaw-90 segment keeps its world orientation. `PlankBridge` carries an explicit `yaw`
   and is the one exception. Teaching all 34 spawnables about orientation is real work that
   belongs with stage 3's gameplay entities, not here. Documented in `place_in` and in the
   format doc; `test_segments` puts only yaw-invariant objects in its rotated segments.

3. **`player_spawn` is authored in the root segment's local frame.** Everything else in a
   segment-based format is local, and the plan says the spawn eventually becomes "the
   checkpoint in the root segment". With the root at the origin it is identical to before.

4. **`placements` is a level-level list, not a field on each segment.** A `place:` field
   per segment would make over-determination *unrepresentable*, which sounds good until you
   notice the brief requires it to be a detectable error. A list of edges also reads better
   next to `connections`, since the two are the same kind of thing with different powers.

5. **Adjacency is per-segment, not level-wide.** Stage 1 flagged that positional matching
   would link triangles across a segment join wherever geometry coincides, and asked for a
   deliberate decision. The decision: **do not link across segments.** Segments are the unit
   of load and unload, so a level-wide map would have to be rebuilt whenever one came or
   went; and with welded joins out of scope there is no coincident geometry to link anyway.
   Matching is now done in *segment-local* positions, which is also more robust — two
   segments' local coordinates can collide numerically while their world positions are far
   apart.

6. **`TerrainWorld::voxel_size()` is the finest voxel size in the level**, and
   `chunk_extent()` is gone from the world API (it is per segment now, reported per segment
   by `level_check`). Callers use `voxel_size` as a step or a tolerance, so the finest is
   the conservative answer.

7. **The schematic dropped the chunk lattice.** With per-segment resolutions there is no
   single lattice to draw, and per-segment lattices would clutter the picture that segment
   outlines now carry. Axis ticks are computed from a round-number step fitted to the span
   instead of from the chunk extent, which also makes them read as round numbers.

8. **Welded joins are rejected at load**, as instructed — `Join(..., weld: true)` produces
   `PlacementError::WeldNotImplemented` naming the segment and explaining the cap-surface
   reason. No cap surface is ever emitted.

### `test_segments.level.ron` — "Harbour Ascent"

Four segments, chained, with a loop closed by an assertion:

```text
  start_beach ──5 m──▶ shoal_run ──6 m──▶ cliff_terrace ──5 m──▶ summit_arena
                           │                     ▲
                           └────── 6 m ──────────┘   (Connect, not a placement)
```

| Segment | Voxel | Derived frame | Solid chunks | Triangles |
|---------|-------|---------------|--------------|-----------|
| `start_beach` (root) | 1.0 m | (0, 0, 0) yaw 0 | 18 | 51 664 |
| `shoal_run` | 1.0 m | (101, 0, 16) yaw 0 | 8 | 29 084 |
| `cliff_terrace` | 0.5 m | (109, 4, 10) **yaw 90** | 18 | 73 288 |
| `summit_arena` | 0.5 m | (109, 8, −43) **yaw 90** | 18 | 59 768 |

Every gap is inside the 7.14 m standing-jump range. The terraces climb in 2 m steps,
inside the 2.50 m apex. `cliff_terrace` at 0.5 m sits next to `shoal_run` at 1.0 m, which
Rule 1 permits because the join is a gap. The second `shoal_run` ↔ `cliff_terrace` link is
the whole reason placement and connectivity are separate: it closes a loop, so it derives
nothing and is checked instead — `level_check` reports it as `measured 6.0 m`, exactly the
declared gap, because quarter-turn placement is exact rather than approximate.

Mesh baseline committed: 213 804 triangles, 310 open edges.

### The ported levels are byte-identical

`test_arena` and `test_empty_terrain` reproduce their stage 1.5 figures exactly:

| Level | Chunks | Solid | Triangles | Vertices | Open edges |
|-------|--------|-------|-----------|----------|------------|
| `test_arena` | 114 | 32 | 191 116 | 573 348 | 218 |
| `test_empty_terrain` | 66 | 16 | 98 948 | 296 844 | 22 |

That equivalence is the regression test that the frame maths is identity-correct before it
is trusted at non-zero origins, exactly as the brief asked.

### Performance — measured, not touched

`cargo test --release --lib -- --ignored --nocapture grenade_update_cost` on `test_arena`,
after the segment layer:

| Grenade | Chunks dirtied | Remesh | Adjacency | Concat | Total |
|---------|----------------|--------|-----------|--------|-------|
| corner (0, 0, 0) | 8 | 18.4 ms | 6.1 ms | 6.9 ms | 31.4 ms |
| mid-chunk (16, 0, 16) | 2 | 4.6 ms | 1.4 ms | 4.2 ms | 10.2 ms |
| mid-chunk (−20, 0, 8) | 2 | 5.1 ms | 2.3 ms | 3.8 ms | 11.2 ms |

Within noise of stage 1.5's figures, so the segment broadphase and the per-segment
transform cost nothing measurable at this level size. **No performance fix was made**, as
instructed. The tripwire from stage 1.5 still stands: concatenation overtakes an 8-chunk
remesh at roughly 1.7 M vertices, and `test_segments` is already at 641 412.

One structural note for whoever does fix it: `Segment::append_render_data` is now the only
place the frame is baked into vertices, and `TerrainWorld` owns the single concatenated
buffer. Per-segment GPU buffers are therefore a change to two functions plus the renderer,
not a redesign.

### The schematic

Committed at `docs/level_segments/segments.svg` (118 KB) and `docs/level_segments/arena.svg`
(44 KB, regenerated). Reading `segments.svg`:

- The four segments read as four outlined, named boxes: `start_beach` bottom-left with its
  tidal bowl as the dark disc, `shoal_run` to its right with two darker bites, then the
  three terraces of `cliff_terrace` stepping up, then `summit_arena` at the top with its
  crater and wall.
- The chain reads in order because the join links are drawn between the actual anchor
  positions and carry their gap in metres. The purple assertion link next to the orange
  join between `shoal_run` and `cliff_terrace` is visibly the second route.
- Anchor arrows point out of their segments, which makes a mis-facing anchor obvious.

Both stage 1.5 flaws are fixed:

1. **Label collisions.** All plan-panel text now shares one reservation grid, claimed in
   decreasing order of importance (gap figures, then anchor names, then object indices). A
   label that would overprint is dropped; the marker stays and the key below carries the
   exact coordinates. `test_arena`'s object cluster is now readable.
2. **Elevation squash.** The vertical range is fitted to the sampled terrain plus authored
   object heights, padded 12 %, with a 16 m floor so a flat level is not magnified into
   fake mountains. `test_arena` went from a −64..32 range to −14..10 and its cave mouths
   and overhang are now legible rather than a strip.

### What a human should check with `cargo run`

**The game window cannot be launched from the agent shell**, so this was not done. Please
run `cargo run` (which loads `test_arena`) and look at exactly these four things:

1. **Nothing moved.** The arena should look identical to before this branch — same terrain,
   same objects resting on the same surfaces. The port to a single segment at the origin is
   supposed to be a no-op.
2. **Shading.** Terrain normals now pass through the segment frame. At yaw 0 that is the
   identity, so any change in shading is a bug in the vertex path, not a subtlety.
3. **Grenade a hillside.** Destruction routes through the segment broadphase now. Craters
   should appear where thrown and the terrain should re-mesh without cracks.
4. **Then** change the level path in `src/main.rs` to `levels/test_segments.level.ron` and
   walk the route: beach → shoals → terraces → arena. Every gap is authored inside the
   standing-jump range, so if any of them cannot be crossed the reach model is optimistic
   in a way `level_check` cannot see.

### For stage 3

- **Traversal primitives belong inside `SegmentDef`, in segment-local coordinates**, as a
  list alongside `objects` and `anchors`. Follow `LevelObject::place_in`'s split: resolve
  the *placement* at load, keep generation local. Unlike the 34 legacy spawnables these are
  new types, so give them a real orientation from the start — `SegmentFrame::rotation()`
  returns a `UnitQuaternion` that agrees exactly with the integer rotation, so a primitive
  that stores a quaternion composes correctly under a rotated segment and does not inherit
  the object-orientation limitation recorded above.
- **Anchors are already the right primitive for a motion-path waypoint.** `Anchor` is a
  named `SegmentFrame`, not a point, so a `Path` can reference `segment.anchor` and get an
  orientation as well as a position. `world_anchor(level, &level.frames, "seg.anchor")`
  resolves one.
- **Gameplay entities should be per-segment too**, and `player_spawn` should become "the
  checkpoint in the root segment" — it is already authored in that frame, so the migration
  is a rename plus a lookup.
- **`Crossing::for_gap`** (`src/level_check/segments.rs`) already answers "which jump does
  this gap need". A traversal primitive that spans a gap should be checked the same way,
  and a gap bridged only by a primitive should stop being reported as a jump.
- **`SegmentState` exists and is always `Active`.** Nothing schedules on it yet. It is
  there so streaming and per-segment reset are a scheduling change rather than a redesign.
- **The open-edge baseline is still per level file.** `test_segments` at 310 is the sum of
  four segments; if a segment definition is ever instanced twice the number doubles, which
  is correct but means the baseline must be re-committed whenever placement changes.

---

## Stage 3 — traversal primitives

**Landed** on branch `level-segments-stage-3`. `cargo build`, `cargo test` (634 + 2 pass)
and `cargo test --release --features bench_harness` (677 pass) are green. All three levels
pass `level_check` with exit code 0 and no findings.

One pre-existing test fails under `bench_harness`:
`physics::bench_harness::tests::terrain_step::pop_replay_captured_pose_resolves_without_teleporting`.
It fails identically on `main` with this branch stashed — it is the open menhir/step
fall-through issue, untouched here.

### What landed

New `src/terrain/traversal/` — the primitives, one file each:

- `solid.rs` — the `TraversalSolid` trait (`sample` + `bounds`, and nothing else), the
  `rasterise`/`excavate` writers, and the half-space helpers the primitives are built
  from.
- `path.rs`, `platform.rs`, `staircase.rs`, `shaft.rs` — one signed-distance function
  each.
- `feature.rs` — `route_plan(&VolumeFeature) -> Option<RoutePlan>`, the single place that
  decides what a primitive *is*. Generation rasterises the plan and `level_check` reads
  the same plan, so the check cannot disagree with the geometry about where a deck is.

New `src/terrain/csg.rs` — `union_solid`, `carve_with_sdf` and `index_range` lifted out of
`generation.rs`, which is what lets a new primitive be a distance function rather than
another triple-nested loop.

New `src/level_check/routes.rs` — the four checks plus `RouteMap`.

New `src/app/spawnables/shared/orientation.rs` — `Yaw`.

Format: `VolumeFeature::{Path, Platform, Staircase, Shaft}`, `PathProfile`, `ShaftLedge`,
`VoxelMaterial(Id)::Sand`, `yaw` on eleven `LevelObject` variants, and
`LevelObject::orientability()`.

### The primitives read as authored — and here is what made that true

Two things, both of which were nearly-invisible bugs until measured:

1. **Sample on the lattice.** The existing volume features iterate `while x < max { x +=
   step }` from an arbitrary clipped minimum, so their samples sit at whatever sub-voxel
   offset the bounds happened to land on. A chunk stores one voxel per lattice cell, so
   the field is evaluated at one position and stored at another, and the sub-voxel offset
   the density encodes is wrong by exactly that difference. `rasterise` walks integer
   indices instead. This is the difference between a deck landing at 12.000 and at
   12.0 ± half a voxel.

2. **Never let a surface pass exactly through a sample.** This is the finding of the
   stage. Marching cubes degenerates when a lattice sample sits exactly on the
   iso-surface: two edges of the same cell interpolate to the same point, the triangle
   between them has zero area, and its edges match no neighbour. Noisy terrain almost
   never hits that case. **A route is made of round numbers and hits it constantly** — a
   deck at y = 12 with 0.5 m voxels hits it along its entire length, and a deck on a 1-in-4
   slope hits it every two metres. Before the fix, a single sloped `Path` produced *896
   open edges out of 3544 triangles*; the same path offset by 6 cm produced zero.
   `solid.rs` now pushes any sample within 1 % of a voxel of the surface onto the inside
   of it (`SURFACE_BAND`). Ties go the same way for solids and for voids, which is also
   what stops a shaft sunk to exactly the height of the slab it pierces keeping a
   one-plane lid over its mouth.

   **Worth knowing:** the heightfield pass has the same exposure. `test_arena`'s surfaces
   are noisy enough that it does not bite, but a perfectly flat authored plateau at an
   integer height is the same configuration.

Two smaller ones:

3. **A staircase is a union of *overlapping* boxes**, each running from its own tread's
   near edge to the head of the flight — not abutting boxes, and not a piecewise-constant
   height. Abutting boxes have a zero-distance plane at every shared face, which reads as
   neither solid nor air and cracks the mesh along every riser. A piecewise height puts
   each riser wherever the densities either side happen to interpolate to, which is
   exactly what would make the step rise uncheckable. The overlap only works one way
   round, so a flight authored downward is normalised to an ascending one.

4. **`Path`'s `Rounded` profile is an exact rounded-box distance**, not an ellipse
   intersected with a half-space. The ellipse version meets the top plane tangentially,
   and the knife edge that produces is not something to hand marching cubes.

### Deviations from the brief

1. **Traversal primitives are `VolumeFeature` variants, as instructed — but stage 2's
   handoff said to put them in `SegmentDef` alongside `objects`.** The brief overrides,
   and it is right: terrain features are already segment-local, so nothing was gained by
   a second list.

2. **`level_check`'s resolution threshold is 2 voxels, not 3.** Three was tried first and
   is unusable: it demands a 3 m deck thickness at 1 m voxels, and — once the staircase
   rise was folded in — a rise no player could climb. Two is the defensible number
   anyway: two samples is what it takes for a primitive to have an *interior*, one inside
   and one outside, which is the minimum the density encoding needs to place both faces.
   The rise and tread run are excluded from the resolution check entirely; they are the
   step *pattern*, and a one-metre riser at one-metre voxels is a perfectly good
   single-voxel step.

3. **The player's collider dimensions moved into `PlayerConfig`.** They were hardcoded at
   the spawn site (`ColliderDesc::capsule(0.5, 0.25)`). The brief asks for the width
   minimum to be derived "in the spirit of `reach.rs`", and there was nothing to derive it
   *from*. `Footprint::derive` now gives `fits` (one diameter — an error below it) and
   `walkable` (two — a warning), and retuning the player retunes what levels are validated
   against.

4. **`PlankBridge.yaw` changed from radians to degrees.** Every other yaw in the format —
   anchors, placements, and now eleven objects — is in degrees. No shipped level set it.

5. **Twelve spawnables turn with their segment; six do not.** See the audit below. The
   brief allows `level_check` to warn where a spawnable "genuinely cannot be oriented";
   the six left are ones that *can* be, and were not, for budget. That distinction is
   recorded in the code as `Orientable::Fixed` rather than hidden, and it warns.

### The spawnable orientation audit

`LevelObject::orientability()` is the authoritative list and is an exhaustive match, so a
new spawnable has to be classified rather than defaulting to "fine".

**Turns with its segment (12).** `Box`, `Plank`, `Stack`, `Tower`, `BoxWall`,
`HoneycombWall`, `Trampoline`, `Table`, `Dolos`, `Trilithon`, `PlankBridge`, and `Domino`
— which rotates its `direction` vector rather than carrying a yaw, since it already
carried its axis explicitly.

**Nothing to do (16).** `BeachBall`, `GlowingOrb`, `Crate`, `HeavyCrate`, `Capsule`,
`Menhir`, `FencePost`, `PlayWheel`, `Tetrahedron`, `Octahedron`, `Dodecahedron`,
`Icosahedron`, `HexPrism`, `Jack`, `Pyramid`, `Jenga`. Spheres, cubes, bodies of
revolution about the vertical, and regular solids whose resting pose is arbitrary anyway.
`Pyramid` and `Jenga` are the two judgement calls: a quarter turn maps each onto an
equally valid instance of itself.

**Directional and still fixed (6).** `Banana`, `House`, `Pendulum`, `Seesaw`,
`VoussoirArch`, `Temple`. Each builds internal structure that a body rotation does not
cover: `Pendulum` and `Seesaw` create **world-anchored** constraints
(`ConstraintKind::world_hinge` / `world_fixed` / `world_ball_joint`, where
`local_anchor_a` is a world position), and the rest lay out many bodies along an axis.

**A generic decorator was considered and rejected**, and the reason is worth recording
because it will look attractive again: a wrapper that rotated every entity a spawnable
returned, about a pivot, would have covered all 34 at once. It cannot, because of those
world-anchored constraints — rotating the bodies leaves the anchors behind, and a pendulum
would tear itself apart. Per-spawnable yaw is the only correct route without first making
joint anchors body-local.

### Cross-segment bridging does not work, and this is the important finding

**A gap between two segments cannot be bridged by a traversal primitive today.** The
check for it is built and tested; the *level* cannot be authored. Two rules collide:

- Generation clips every feature to its segment's `terrain.bounds`, so a deck cannot reach
  into the gap unless the bounds are extended to cover it.
- Extending the bounds puts solid voxels in a new chunk, and Rule 4 rejects two segments
  owning solid chunks whose world extents overlap.

A chunk is 32 m at 1 m voxels and 16 m at 0.5 m. Every join in `test_segments` is 5–6 m,
so a bridging deck and the far segment's first chunk *always* overlap, whatever the deck
does. Concretely: `shoal_run.north_low ↔ cliff_terrace.east_low` is 6 m and cannot be
bridged, and neither can any of the three placement joins.

Three ways out, for whoever picks this up:

1. **Sub-chunk contention granularity.** Compare solid-voxel extents rather than
   solid-chunk extents when two chunks overlap. Rule 4's own rationale — "checked against
   allocated or derived bounds it is a rejected level for no reason" — applies one level
   down.
2. **Cross-segment halo sampling** (the welded-join work), which makes the question moot.
3. **A bridge is its own segment.** Clean within today's rules, but it turns one join into
   two and the reach check then measures the two halves, not the crossing.

`RouteMap` clips to `terrain.bounds` exactly as generation does, so the check does **not**
claim a bridge that was never written — there is a test for that specifically
(`a_path_running_past_its_segments_bounds_does_not_bridge`). The bridged-gap behaviour is
tested against a synthetic level whose bounds do cover the deck.

### Open edges rose 3.8×, and it is not the primitives

`test_segments` went from 310 open edges to 1174, and the baseline is re-committed. This
was measured rather than assumed, by ablation:

| Level variant | Open edges | of triangles |
|---------------|------------|--------------|
| No traversal primitives at all | 310 | 213 804 |
| All primitives, as shipped | 1174 | 223 800 |
| `cliff_terrace`'s `Path` lifted into free air | 390 | 226 072 |
| …and made straight, with no turns | 390 | 226 024 |

**A primitive standing in free air contributes zero open edges.** Turns contribute
essentially nothing. Every one of the 864 comes from the cells where a deck *meets
terrain* — unioning two independently-encoded density fields produces ambiguous
marching-cubes configurations along the join. That is the same mechanism behind the cave
figures in the existing baselines and predates this stage; what is new is that a route is
*supposed* to meet the ground it crosses, so a level built from primitives meets it a
great deal more often. It wants a mesh-quality pass. It is not a defect in the primitives,
and the numbers above are the evidence.

### `test_segments.level.ron` — extended

Every primitive appears, at both of the level's voxel resolutions, including a `Path` in a
quarter-turned segment:

| Segment | Voxel | Yaw | Primitives |
|---------|-------|-----|-----------|
| `start_beach` | 1.0 m | 0 | two `Platform` stepping stones 6 m apart across the tidal bowl; a 3-step `Staircase` up to the launch ledge |
| `shoal_run` | 1.0 m | 0 | one `Path` — five waypoints, over both craters, turning and climbing to the north ledge |
| `cliff_terrace` | 0.5 m | **90** | one `Path`, `Rounded` profile, zigzagging up the three terraces |
| `summit_arena` | 0.5 m | **90** | a `Shaft` with a helical `ledge` (7 m bore, 6 m pitch — 1.5 m per quarter turn); a `Platform` perch |

Mesh baseline re-committed: 223 800 triangles, 1174 open edges.

### The schematic

Both flaws the brief named are fixed, and the routes are drawn.

- **`east_ledgelanding`.** `LabelSpace` reserved one 30 px cell per label while a name is
  ~60 px wide, so two anchors mated across a join landed in *different* cells and still
  overprinted. `claim_text` now reserves every cell a label covers. That alone was not
  enough: two mated anchors are ~15 px apart and their names are 60 px long, so putting
  them on opposite *sides* of the join still overlapped. The row is now chosen from the
  anchor's facing as well, which puts mated anchors on opposite sides of the link *line*,
  where they cannot collide. Seven of `test_segments`' eight anchor names are drawn, none
  overprinting (was: two drawn, overprinting). The eighth is `north_low`, 16 m from
  `north_ledge` on the same face — a fallback row is tried before a label is dropped.
- **"1 segments"** — pluralised.
- **Routes.** `route_overlay` draws each primitive in world space, in a colour nothing
  else in the picture uses: a `Path` stroked at its real deck width with a centreline over
  it, a `Platform` as a filled rectangle spanned from its *transformed* corners, a
  `Staircase` with one tick per riser so the steps can be counted, and a `Shaft` as its
  bore ring with the ledge annulus around it. The intended route through `segments.svg`
  can now be traced with a finger without opening the RON — and because it is drawn in
  world space, `cliff_terrace`'s zigzag visibly runs along world −Z, which makes the
  picture a check on the orientation work too.

### What a human should check with `cargo run`

**The game window cannot be launched from the agent shell.** Change the level path in
`src/main.rs` to `levels/test_segments.level.ron` and walk the route. The open question is
whether the primitives are *pleasant*, which no headless check answers. Specifically:

1. **The `shoal_run` catwalk.** 3 m wide, 1.5 m above the ground it crosses. Does a 3 m
   deck feel like a walkway or like a tightrope? The check's minimum is 1 m; if 3 m feels
   thin, `WALKABLE_WIDTH_IN_DIAMETERS` in `reach.rs` is the number to raise.
2. **Its climbing legs.** The deck rises 1 m over 8 m and then 1.5 m over 12 m. Does the
   player walk up a shallow slope smoothly, or catch on the voxel steps of it?
3. **The `start_beach` staircase**, 1 m per step over a 16 m run. Can it be walked up
   without jumping? There is no step-up assist in the controller, so this is the one that
   most likely needs a smaller rise — if it does, halve it and the check will still pass.
4. **The two `Platform` stepping stones** in the tidal bowl, 6 m apart. Inside the 7.14 m
   standing jump on paper; the reach model is point-mass, so this is the direct test of
   how much margin it really needs. Also: can you climb *back onto* one after missing?
5. **The `summit_arena` shaft.** 2.5 m ledge, 1.5 m gained per quarter turn. Descending a
   spiral is the case where a voxel ledge most easily feels like a series of trips. Does
   the outer wall keep you on it, or do you fall off the inner edge?
6. **`cliff_terrace`'s `Rounded` path**, in a quarter-turned segment. Two things: does the
   rounded edge read as a ledge rather than as a pipe, and does the whole thing sit where
   the schematic says it does.
7. **Grenade a deck.** Primitives are ordinary voxels, so they should crater and remesh
   like terrain. A `Path` blown in half mid-span is the interesting case.

### For stage 4 — what the viewer must show that the SVG cannot

- **Whether a deck reads as a deck.** The whole sub-voxel-SDF argument is about edges, and
  a plan view draws the authored polyline, not the mesh. The one picture that would settle
  the resolution threshold is a `Path` of the same width at 1.0 / 0.5 / 0.25 m voxels,
  side by side, from a low angle.
- **The seams where a route meets terrain.** That is where all 864 new open edges are, and
  they are invisible in plan. A shot along a deck's line of contact with the ground, plus
  an open-edge overlay, is the fastest way to know whether they are cosmetic or holes.
- **Undersides.** Everything the schematic shows is from above. A `Path`'s `Rounded`
  profile, a `Platform`'s thickness and a `Shaft`'s ledge are all things you only see from
  below or from inside.
- **A rotated segment from inside it.** The orientation tests compare sampled columns,
  which proves position and shape but says nothing about *shading* — stage 2 recorded that
  a rotated segment lit with unrotated normals looks plausible.
- **What `Orientable::Fixed` actually costs.** A `House` or a `Temple` in a quarter-turned
  segment, rendered, is the cheapest way to decide whether the remaining six are worth
  doing.

---

## Follow-up: the heightfield had the same degeneracy — and so did everything else

Stage 3 fixed sample-on-the-surface degeneracy for the traversal primitives only, and
noted that the heightfield pass had the same exposure. It did. So did every other
volumetric feature, and so did the cave carve. It was one bug wearing three different
explanations.

### Reproduction

New tests in `src/terrain/world.rs` generate a small terrain description and assert
`open_edge_count() == 0`. Measured before any fix:

| Configuration | Voxels | Open edges | Triangles |
| --- | --- | --- | --- |
| Flat `base_height: 8.0` | 1.0 m, 0.5 m, 2.0 m | **0** | — |
| `TerrainFeature::Plateau` at y = 8 | 1.0 m | **144** | 1820 |
| `TerrainFeature::Cliff`, shelves at y = 4 and y = 8 | 1.0 m | **172** | 1940 |
| `VolumeFeature::Overhang`, top at y = 10, 2 m thick | 0.5 m | **144** | 6832 |

The flat case passing is the informative one, and it is worth stating because it is why
this stayed hidden. A surface coincident with a lattice plane *everywhere* is not a crack:
every sample in the plane reads the same, the whole plane resolves to one side, and the
mesh closes. The degeneracy needs a **boundary** — a plateau edge, a cliff shelf running
out, an overhang face — where cells with a zero-density sample sit next to cells without
one. That is why authored features crack and a featureless test plane does not, and why
"flat terrain meshes fine" was never evidence of anything.

### The fix

`SURFACE_BAND` and its debias moved from `src/terrain/traversal/solid.rs` into
`src/terrain/csg.rs`, and are now applied **inside `union_solid` and `carve_with_sdf`
themselves** rather than at one call site. That is the whole change for volumetric
features: every feature that writes a density — island, pillar, arch, overhang, tunnel,
caves, and the traversal primitives — is debiased on identical terms, with one constant,
and a new one cannot be written that forgets to be.

The heightfield needed its own entry point because it does not go through those writes: it
derives a column's topmost solid index and its partial-air cap from the height, so
debiasing each distance separately would leave those indices describing a surface that is
no longer there. `debias_height` nudges the height once, before anything is derived from
it — the same constant, the same direction, applied to the surface rather than to the
sample.

### Cliff and Overhang: both affected

Both were tested rather than assumed, and both cracked. `Cliff` runs through `height_at`,
so it is fixed by `debias_height`; its two shelves are authored heights and land on the
lattice exactly as squarely as a plateau does. `Overhang` writes a slab through
`union_solid`, so it is fixed there; both its top face and its underside (top minus an
authored thickness) are round numbers.

### Baselines: all three fell to zero

| Level | Open edges before | After | Triangles before | After |
| --- | --- | --- | --- | --- |
| `test_arena` | 218 | **0** | 191116 | 193828 |
| `test_empty_terrain` | 22 | **0** | 98948 | 100276 |
| `test_segments` | 1174 | **0** | 223800 | 224756 |

Re-committed in `levels/mesh_baselines.ron`.

### What surprised me

**The two "known separate issues" were this issue.** Both were explicitly scoped out of
this work, and both closed anyway.

- The cave open edges (218 and 22) were attributed to cave noise finding ambiguous
  marching-cubes configurations. They were not. `carve_with_sdf` encodes
  `threshold - noise` as a density, and wherever the noise equals its threshold that is a
  density of exactly zero — the same degeneracy, arriving through a noise field instead of
  through a round number. Noise does not protect you when the surface is *defined* as a
  level set of that noise.
- `test_segments`' 1174 was attributed to unioning two independently-encoded density
  fields where a deck meets terrain. Also not it. A deck is authored at a round height, so
  its own encoded surface sits on the lattice; the join was where the boundary condition
  above was met, not where two encodings disagreed. Whether a genuine two-field ambiguity
  exists underneath is now unmeasurable at these levels, because there is nothing left to
  measure.

Triangle counts rose ~1 % everywhere. That is the degenerate cells now resolving to real
triangles rather than to zero-area ones, which is the fix working, not geometry moving.

### One side effect worth a decision

`level_check` on `test_arena` now emits **11 new warnings** of the form
`HoneycombWall #12 in 'main' at (6.0, 0.0, -6.0) has no terrain beneath it`. Nothing is
broken — all three levels still exit 0 — but the cause should be understood rather than
silenced.

`test_arena` has `base_height: 0.0` and authors objects at `y = 0.0`. The debias raises a
coincident surface by 1 % of a voxel, so the surface is now at y = 0.01 and
`surface_below` — which takes the first mesh height `<= p.y` — finds nothing under an
object sitting at exactly 0.0. The objects are 1 cm buried; physics resolves that on the
first step.

This is the documented cost of the band, and it now lands on authored content rather than
only on the mesh. The direction was kept consistent with `solid.rs` (a point exactly on a
surface belongs to the solid) rather than inverted to make the warning go away, because
inverting it for the heightfield alone would put two directions in one meshing path — the
thing the constant was consolidated to prevent. The alternatives, if the warnings are
judged not worth living with, are to give `surface_below` a tolerance of one band, or to
author `test_arena`'s objects a hair above zero. `level_check`'s checks were out of scope
here, so neither was done.

**Resolved:** `surface_below` was given a one-band tolerance reusing `terrain::SURFACE_BAND`
(commit `3319a4d`). The tolerance belongs in the check rather than in authored content —
authoring around an implementation epsilon would put the trap in every future level. All
three levels now report 0 errors and 0 warnings.

---

## Stage 3 played — and what it changed

Stage 3 merged (`45d012d`) and the route in `test_segments` was played by hand. Verbatim
findings, in the order of the check list it shipped with:

1. A 3 m deck width feels fine.
2. The player walks up a sloped deck fine, **but it looks lumpy.**
3. **The staircase cannot be walked up without jumping**, and the risers are not vertical —
   marching cubes tilts them.
4. The two stepping stones sit right at the limit of a standing jump. Reachable once
   ledge-grab exists. You can only climb back on from beside the bowl, not from below.
5. **The spiral ledge looks bumpy.** Walkable downward.
6. The quarter-turned segment looks jagged. It is exactly where the schematic says it is.
7. A deck survives a grenade that craters the ground beneath it, leaving it floating.

Findings 2, 3, 5 and 6 share one cause, and it is **not** resolution: marching cubes cannot
represent a sharp edge or corner at *any* voxel size, because each vertex is interpolated
along a cell edge. A riser falling between two sample planes comes out as a slope, and at
double resolution it comes out as a smaller slope. Stage 3's sub-voxel work got surfaces to
land at the right height; nothing in that approach can make them land at the right angle.

**The conclusion: voxel traversal primitives were built on the wrong side of a line.** They
keep their value as *landscape* shaping — a mesa, a natural ledge — but the walkable route
wants to be static compound rigid bodies instead. That analysis, the agreed role split
(landscape / structure / clutter), why "structure" is a spawnable rather than a new object
category, the `route_plan()` generalisation it needs, dual contouring as the medium-term voxel
option, and the spike to run first, are all written up in **`docs/TERRAIN_REPRESENTATION_PLAN.md`**.

Nothing from stage 3 is wasted: the marching-cubes fixes, `csg.rs`, `RoutePlan`/`RouteMap`,
the orientation work on twelve spawnables, and `Footprint` in `PlayerConfig` are all
independent of which side of that line a route ends up on.

### Outstanding at the point this was written down

- **`test_segments`' route wants re-authoring** once structure exists. Its staircase cannot
  be walked, so it currently documents a defect.
- **Cross-segment bridging is impossible** — generation clips features to segment bounds, and
  extending the bounds trips Rule 4's solid-chunk contention. Three candidate fixes are in the
  stage 3 notes above; sub-chunk contention granularity is the most promising.
- **Welded joins** remain unimplemented and rejected at load.
- **Terrain-destruction latency** — measured, unfixed, remesh-dominated.
- ~~Mesh-quality pass~~ — **closed.** It was the surface degeneracy, not a separate mechanism.
