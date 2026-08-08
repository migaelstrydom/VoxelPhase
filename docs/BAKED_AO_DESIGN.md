# Baked Ambient Occlusion — Design Doc

Stage 1 of the [Lighting Plan](LIGHTING_PLAN.md). A per-vertex occlusion factor
baked into terrain meshes at mesh-generation time, attenuating the *environment*
lighting terms so that creases, undersides and the ground beside a wall stop
receiving the full sky.

This doc replaces an earlier version that specified hemisphere ray marching
against the SVO. That version was never implemented; its own cost model arrived
at 20–100 ms per region rebuild, which is not affordable inside the destruction
budget. The technique here is different and the reason is entirely performance —
see [Why not ray marching](#why-not-ray-marching).

A second, more ambitious attempt also exists and was abandoned: branch `sdf-ao`
built a true Euclidean SDF (fast-sweeping) into a GPU atlas and cone-traced it
per fragment. Its lessons are recorded in [Traps](#traps-carried-over-from-the-sdf-attempt);
the branch itself no longer applies, since it integrates against a
`TerrainManager` and a `triangle.frag` that the chunked-terrain and lighting
reworks have both since replaced.

## What this has to achieve

The specific gap, stated precisely, because it determines what AO multiplies.

The sun is occluded — there is a shadow map. The **sky is not**, and since
`shader/environment.glsl` landed the sky is a real fill light arriving from the
whole hemisphere. `SKY_IRRADIANCE_FACTOR` (0.4) and `GROUND_ALBEDO` (0.25) are
flat global discounts standing in for that missing occlusion; both carry a
comment saying so. AO is what lets those become honest.

So: **AO attenuates the environment terms only.** It never touches the sun or
the point lights. Direct light already has an occluder that knows the actual
geometry, and stacking AO on top of it double-darkens exactly where the shadow
map is already correct.

## Non-goals

- **AO on props, the player, or any dynamic body.** Terrain is static and can be
  baked; a tumbling crate cannot. Occlusion between moving bodies and the ground
  they rest on is the screen-space item in `VISUAL_DIRECTION.md` §3, and it is a
  different feature with a different mechanism.
- **Long-range occlusion.** This darkens crevices at a scale of a few voxels. A
  valley floor does not get darker for being in a valley.
- **Bent normals**, directional occlusion, or anything that makes AO a function
  of the incoming direction. One scalar.
- **A reusable distance field.** That was the other branch. If soft shadows or
  GI ever want one, they can propose it on their own merits.

## Approach: a blurred occupancy field

Build a small scalar field over each chunk holding *how solid the neighbourhood
is*, sample it slightly off the surface along the normal, and calibrate so that
an open flat plane reads as unoccluded.

```text
  VoxelBlock (MC's padded sample grid, unchanged)
        │
        │  generate_block already fills this
        ▼
  MarchingCubes ──────────────────────────┐
                                          │  positions, normals, colours
  ChunkGrid ──fill_block──▶ OcclusionGrid │
        (separate, half-res, wider halo)  │
                │                         │
                │  binary occupancy       │
                ▼                         │
           separable blur                 │
                │                         │
                ▼                         ▼
        sample(pos + n·offset) ───────▶ Vertex.ao
```

Three steps.

**1. Occupancy.** Each sample of the occlusion grid is a ramp across the
surface:

```
   occupancy = saturate(0.5 + 0.5 * density)
```

**This replaces the binary test the first version of this doc specified.** That
version was wrong, and step 1 caught it — the record is kept here because the
reasoning behind it is still half right and would otherwise be re-derived.

The original argument was that density is not a distance, that its gradient
magnitude depends on how the generator, the CSG ops and the destruction path
each happened to author it, and that only its *sign* is guaranteed to mean
anything. The first half of that is a real hazard. The conclusion was not:
`csg::union_solid` writes `clamp(-sdf / voxel_size, -1, 1)`, so density is the
signed distance to the surface in voxels, saturating one voxel out. The scale is
neither arbitrary nor assumed; it is written down. And marching cubes already
trusts precisely the same linear model every time it places a vertex along an
edge with `lerp_t(d0, d1, iso)`.

What a binary test costs is severe, because occupancy is evaluated on a lattice
*coarser than the mesh*. Reading only the sign quantises the surface to that
coarse lattice, so a flat plane's brightness depends on where it happens to fall
between samples. Measured over one cell of phase, at `R = 2` and a half-cell
offset:

| occupancy | range of the blurred read across one cell of phase |
|---|---|
| binary | **0.320** |
| density ramp | **0.040** |

Against a usable range of `1 - B ≈ 0.68`, the binary figure is roughly half the
effect's entire dynamic range appearing as soft mottling on open ground, at the
grid's own spacing. `the_field_barely_ripples_as_a_plane_moves_between_samples`
bounds it at 0.06 in the final AO value.

The destruction path is the weak case and worth being clear about:
`Voxel::apply_damage` returns a whole `Voxel::air()` at `-1.0` rather than a
partial distance, so a fresh crater's surface carries no sub-voxel offset. The
mesh has that same limitation from that same cause, so AO and geometry still
agree about where the surface is — which is the property that actually matters.
If crater AO ever looks quantised, the fix is in `apply_damage`, and it fixes
the mesh at the same time.

**2. Blur.** Convolve the occupancy field with a small separable kernel of
radius `R` grid cells — three 1D passes, `x` then `y` then `z`. The result at a
point is the distance-weighted fraction of its neighbourhood that is solid.

**3. Calibrate and read.** For a mesh vertex at `p` with normal `n`:

```
    O = blurred_occupancy(p + n * NORMAL_OFFSET)      // trilinear
   ao = 1 - AO_STRENGTH * saturate((O - B) / (1 - B))
```

`B` is the value the same kernel and the same offset produce above an **infinite
flat half-space**. Subtracting it is what makes flat ground read as `ao = 1.0`
rather than as a uniform grey — a plane is half-solid, so an uncalibrated blur
reports ~50 % occlusion on the most open surface in the game.

`B` must be *derived from the kernel*, not hand-tuned, or changing `R` silently
re-tints the whole world. Compute it numerically at construction from the kernel
weights and the offset (a few dozen multiplies, once), and assert against it in
a test. `AO_STRENGTH` then means exactly one thing: how dark a fully enclosed
crease goes.

### Why not ray marching

Per-vertex hemisphere marching is `N` rays × `S` steps of *field queries per
vertex*, and a chunk emits thousands of vertices. The old doc's own numbers:
~100 queries per vertex, 2k–10k vertices, 20–100 ms per region.

A blur is `O(grid)` and independent of vertex count. Concretely, at the sizes
below: ~12k grid samples × 3 passes × 5 taps ≈ 180k multiply-adds, plus one
trilinear read per emitted vertex. That is two orders of magnitude less work,
and it is *flat* — a chunk full of intricate surface costs the same as an empty
one, which is the property that keeps a grenade's worst case bounded.

It is also smooth and deterministic by construction. Stochastic ray sets are the
usual source of AO that shimmers when a chunk is rebuilt; a fixed convolution
has nothing to shimmer.

What it gives up: a blur is isotropic, so it cannot tell a wall one voxel to the
side from a ceiling one voxel above. For crevice darkening at this scale that
distinction is not visible, and the normal offset recovers most of the
directionality that matters.

## Resolution, halo, and the cost

This is the section that answers "what does it do to remesh time", so the
numbers are spelled out.

### Do not widen the marching-cubes block

The tempting implementation is to widen `MeshOctree::generate_block`'s existing
padded grid — it already holds the voxels, so AO could read them for free. It is
the wrong move. That grid is at full voxel resolution with a 1-sample halo:
`cells + 3` = **35³ = 42,875 samples** for a 32-voxel chunk. Widening its halo
to cover an AO reach of ~5 voxels takes it to 43³ = 79,507, nearly doubling the
`sample` phase — which is the fill from the SVO, and one of the phases the
destruction-cost work already fought to bring down.

### Use a separate, coarser grid instead

AO is low-frequency. Build it on its own lattice at **half voxel resolution**
(`spacing = 2 * voxel_size`), which needs a far smaller grid even with a
generous halo:

| | full-res MC block | half-res AO grid |
|---|---|---|
| owned samples per axis | 33 | 17 |
| halo per side | 1 | 4 |
| total per axis | 35 | 25 |
| **samples** | **42,875** | **15,625** |

The AO grid is ~36 % of the existing block's sample count, and the MC block is
untouched.

The halo of 4 is derived rather than chosen (`OcclusionSettings::halo`): the
blur consumes `blur_radius = 2` samples at every boundary, a read may land
`ceil(offset_cells) = 1` outside the owned region, and trilinear interpolation
needs the sample beyond the one it lands in. An earlier draft of this table said
3, by forgetting the last of those. The added work per chunk is one extra `fill_block` over 12k samples,
three blur passes, and one trilinear read per vertex.

`SparseVoxelOctree::fill_block` is already resolution-agnostic — it works off
`SampleLattice` positions via `first_index_at_least` and has no assumption that
spacing equals `voxel_size`. Nothing in the SVO or `ChunkGrid` needs to change.

Halo of 3 half-cells = 6 voxels covers `NORMAL_OFFSET` (~1 voxel) plus the blur
radius (`R = 2` half-cells = 4 voxels) with a cell to spare.

### Measure it, don't trust this table

Add `ambient_occlusion: Duration` to `MeshBuildTimings` (`mesh_octree.rs:149`)
alongside `grid_alloc` / `sample` / `marching_cubes` / `insert` /
`neighbor_refs`, and include it in `total()` and `add()`. It then flows through
`ChunkRemeshTimings` and `SegmentTimings` to the existing debug output for free.

**Budget: ≤ 1 ms per chunk remesh.** For reference, a grenade cost ~14.9 ms
across all the chunks it touches before AO.

### What it actually cost

Measured on `test_arena` with `grenade_update_cost_split` (release, `--ignored`),
after step 2:

```
grenade 0: 8 chunks dirtied | remesh 13.3 ms | adjacency 6.3 | concat 0.8 | total 20.5 ms
  remesh split: grid alloc 0.04 | sample 2.84 | marching cubes 2.14
              | ambient occlusion 5.40 | octree insert 1.07 | neighbour refs 1.09
```

**0.67 ms per chunk — inside the budget, and now the largest single phase.** A
grenade goes from ~14.9 ms to ~20.5 ms, about +37 %.

Two things in the cost model above are wrong, and the split says so:

- **Half resolution does not make the fill cheaper.** `SparseVoxelOctree::fill_block`
  says in its own doc comment that it is proportional to the octree *nodes* a
  block touches rather than to its samples, and it means it. Measured per
  sample: 8.4 ns for the marching-cubes block, 44 ns for the occlusion grid —
  five times worse, which is what you get when the same tree walk is amortised
  over eight times fewer samples. Coarsening saves the blur, the memory and the
  per-vertex reads, but not the fill.
- **The occlusion grid's fill is therefore a bigger job than the MC block's**,
  despite having 36 % of the samples, because what matters is the *volume* it
  covers: 50 voxels per axis against the MC block's 34, which is 3.2× the space
  and so roughly 3.2× the nodes. **Two thirds of that volume is halo.**

The parts that are proportional to samples came in exactly as predicted and are
not where the money goes: the blur is ~0.20 ms and the per-vertex trilinear
reads are ~10 ns each, ~0.06 ms for a 6,000-vertex chunk. Rewriting the blur to
hoist its bounds checks out of the inner loop changed the total by nothing
measurable, which is the expected outcome once the split is read properly.

So the lever, if this needs to get cheaper, is **the halo's volume** — not the
resolution, and not the blur. Nothing else on the list is worth trying first.

If it comes in over budget, the levers in order:

1. **Drop the blur radius.** `R = 1` instead of 2 shrinks the halo to 3 and the
   grid to 23³, and cuts the tap count from 5 to 3 per axis. It costs more than
   crispness, though: the phase ripple rises from 0.040 to 0.170, because a
   narrower kernel does less to smooth the surface's quantisation. Treat `R = 1`
   as the floor and check the ripple test, not just the timing.
2. ~~**Quarter resolution.**~~ **Not available — measured, not guessed.**
   `resolution_divisor = 4` was the obvious next lever and it does not work.
   Density saturates one *voxel* out, so the sub-voxel information that keeps
   the field stable spans a quarter of a cell at that spacing instead of half of
   one, and the phase ripple goes to 0.162 at `R = 2` — four times worse than
   half resolution, and worse than dropping the blur radius. It is not that AO
   gets blobby; it is that flat ground starts to mottle. Half resolution is the
   floor.
3. **Skip chunks with no surface.** A chunk that emitted no triangles needs no
   AO grid at all. Cheap guard, should be in from the start.

Note lever 3 is free and should not wait for a measurement.

### Alignment is what keeps seams out

The half-res lattice must be anchored to **global even sample indices**, not to
the chunk. `ChunkGrid::first_sample(coord)` returns `coord * CHUNK_VOXELS`, and
`CHUNK_VOXELS = 32` is even, so `base = first_sample / 2` at `spacing =
2 * voxel_size` lands every chunk's grid on the same global lattice.

That matters because `SampleLattice` derives positions from the *global* index
(`voxel_block.rs:33`) — the same property that already makes marching cubes
seam-free between chunks. Two chunks evaluating AO for the same world position
therefore read the same lattice points from the same voxel data and get
bit-identical results. Seams do not need to be blended away; they need to not
exist, and this is how.

**A read must never be clamped to the grid.** Clamping at the boundary is
precisely what produced the visible seams on the `sdf-ao` branch, and it does so
silently. The halo is sized so that no legal read can fall outside; encode that
as a `debug_assert!` on the sample path, not a `clamp`. If it ever fires, the
halo is wrong and that is the bug to fix.

## From the lattice to the vertex

Marching cubes emits **unwelded** vertices — three per triangle, no sharing
(`marching_cubes.rs:200`). Computing AO per emitted vertex would be ~3× redundant
and, worse, would let two duplicates of the same position disagree and crack.

Neither happens here, because AO is a pure function of position: the trilinear
read at a given world point returns one value regardless of which vertex asked.

The natural insertion point mirrors the code that already exists for normals.
`process_cell` interpolates a normal along each crossed edge from the two
corners' gradients:

```rust
let grad = corner_gradients[v0].lerp(&corner_gradients[v1], t);
```

AO slots in beside it as `edge_ao[i]`, either as the same lerp between two
corner lookups or as a single trilinear read at `edge_vertices[i]`. Prefer the
single read at the final vertex position plus normal offset — it is one lookup
instead of two and it is the position the shading actually happens at.

`MarchingCubesMesh` gains a parallel `ao: Vec<f32>`, matching its existing
`positions` / `normals` / `colors` layout.

## Storage

Add `ao: f32` to `Vertex` (`src/rendering/vertex.rs`) at location 4,
`R32_SFLOAT` — **and narrow `pos` from `Vector4` to `Vector3` in the same
change.**

`pos.w` is `1.0` at all 84 construction sites, and both `triangle.vert:8` and
`shadow.vert:15` already declare the attribute as `in vec3 inPosition` and
rebuild `vec4(inPosition, 1.0)` themselves. The fourth component is uploaded
every frame and read by nothing. Reclaiming it pays for the AO float exactly:

```
   before:  Vector4 pos (16) + Vector4 colour (16) + Vector2 uv (8) + Vector3 normal (12)             = 52
   after:   Vector3 pos (12) + Vector4 colour (16) + Vector2 uv (8) + Vector3 normal (12) + f32 ao (4) = 52
```

So AO costs nothing per vertex, and nothing on the terrain buffer that is
currently re-uploaded every frame (`project_terrain_perframe_upload`). No shader
edit is needed either — they already read `vec3`. Only the attribute format
(`R32G32B32A32_SFLOAT` → `R32G32B32_SFLOAT`) and the offsets after it change in
`get_attribute_descriptions`.

The remaining cost is 84 struct literals across 29 files. Accept it: removing a
field makes every literal fail to compile, so the compiler enumerates the work
and no site can be silently missed — and every one of those literals currently
writes a trailing `1.0` that is being deleted anyway.

Non-terrain meshes carry `ao: 1.0` and are unaffected, which is also the correct
default: no occlusion.

Packing AO into `color.w` was considered and rejected. It saves no space now
that `pos.w` pays for the field, and `triangle.frag:78` feeds `inColor.a` into
the output alpha, so the overload is real and would have to be unpicked first.

## Shader integration

`triangle.vert` passes the attribute through. `triangle.frag` applies it to the
environment terms in `shadeEnvironment` — the diffuse irradiance at full
strength, the specular reflection at reduced strength.

Specular wants its own treatment because a mirror in a crevice still reflects
whatever is in front of it; fully occluding it reads as dirt rather than as
shade. Either a mild `mix(1.0, ao, 0.5)`, or Lagarde's horizon-based specular
occlusion if the simple version reads wrong on the metals in `material_grid`.
Start simple.

**Do not retune `SKY_IRRADIANCE_FACTOR` or `GROUND_ALBEDO` in the same change.**
Both exist to fake this occlusion and both should eventually rise toward 1.0 now
that it is computed. But raising them raises the absolute light level, and per
`VISUAL_HANDOFF.md` that level is pinned by content: the level albedos are
saturated primaries authored against the current lighting, and pushing them up
the tonemap's shoulder washes the game out. Land AO with the discounts
unchanged — the frame gets slightly darker, only in creases — then judge the
rebalance separately, on `palette`, as its own change.

## Rebuilds and destruction

A chunk's AO is baked from voxels within its halo, so editing a voxel changes
the AO of vertices up to the halo distance away — including in the *neighbouring*
chunk when the edit is near a boundary.

The existing dirty-chunk logic already handles this, and it is worth being clear
why rather than assuming it. `damage_sphere` marks every chunk the sphere
touches, and a chunk whose mesh is rebuilt rebuilds its AO grid from current
voxel data. The gap is an edit *just inside* chunk A near the boundary with B:
A is dirtied, B is not, and B holds vertices within the AO halo of the edit.

Confirm this against the adjacency/dirty-marking path before implementing, and
if the halo does reach further than the dirty set, expand the dirty region by
the AO halo (6 voxels) rather than trying to patch AO in place. This is the
"AO radius extension" concern the previous version of this doc raised, and it
remains correct even though everything around it changed.

Fresh craters getting correct AO is not a nice-to-have: it is most of the value.
A grenade that carves a hollow and leaves it flatly lit looks like a decal.

## Parameters

All on the AO baker, not in a global config struct.

| Name | Start | Meaning |
|---|---|---|
| `resolution_divisor` | 2 | Grid spacing as a multiple of `voxel_size`. |
| `blur_radius` | 2 cells | Kernel reach; drives the halo. |
| `normal_offset` | ~1 voxel | How far off the surface the field is read. |
| `strength` | 0.7 | Darkness of a fully enclosed crease. The only aesthetic dial. |

`blur_radius` and `normal_offset` determine the halo, and the halo determines
correctness — they are not free to tune at runtime. Deriving the halo from them
in one place, rather than writing both numbers down twice, is what stops the two
drifting apart.

## Tests

Pure-Rust, against analytic voxel fields, in the baker's module:

- **`flat_ground_is_unoccluded`** — the calibration test. A half-space reads
  `ao ≈ 1.0`. If `B` is wrong, this is what catches it, and its failure mode
  (everything uniformly grey) is otherwise easy to mistake for correct output.
- **`enclosed_point_reaches_full_strength`** — fully surrounded reads
  `ao ≈ 1 - strength`.
- **`inside_corner_darkens_toward_the_crease`** — two perpendicular planes;
  monotonic darkening approaching the line.
- **`convex_edge_is_not_darkened`** — the outside of a corner stays at 1.0.
  Guards against a sign error that would invert the whole effect.
- **`neighbouring_chunks_agree_on_a_shared_vertex`** — the seam invariant. Mesh
  two adjacent chunks; a vertex on the shared plane gets bit-identical AO from
  both. This is the test the `sdf-ao` branch did not have.
- **`no_read_falls_outside_the_grid`** — exercise a surface at the chunk
  boundary with the `debug_assert` armed.
- **`the_field_barely_ripples_as_a_plane_moves_between_samples`** — the bound on
  the artefact that sank the binary occupancy above. Ground at eight sub-cell
  heights must not vary by more than 0.06 in AO.
- **`the_baseline_tracks_the_blur_radius`** — the baseline is derived, not typed
  in; if it stops tracking the kernel, flat ground stops reading as open.
- **`rebaking_is_deterministic`** — same voxels twice, identical output.
- **`a_crater_darkens_after_damage`** — regression against AO going stale.

## Staging

Four steps, ordered so the performance question is answered before any visual
risk is taken.

1. ~~**`src/terrain/ao.rs` — `OcclusionGrid`, standalone.**~~ **Done.** Builds
   from a `VoxelSource`, separable blur, calibrated trilinear read, eight tests.
   It earned its place as a separate step immediately: the binary-occupancy
   decision above failed its own calibration test on the first run, and was
   replaced before anything depended on it.
2. ~~**Bake it, store it, measure it.**~~ **Done.** Wired into `generate_block`
   after marching cubes, skipping blocks that emitted no surface;
   `MeshBuildTimings::ambient_occlusion`; `ao` on `Vertex` with `pos` narrowed
   to `Vector3` to pay for it. The shader still ignores it, so nothing looks
   different and the whole cost is on the timing overlay. See the measurement
   above.

   `MarchingCubesMesh` did *not* gain a parallel `ao: Vec<f32>`. AO is a pure
   function of position, so reading it once per emitted vertex in
   `generate_block` gets the same answer and leaves marching cubes unaware that
   occlusion exists.
3. **Consume it in the shader.** `triangle.vert` / `triangle.frag`, recompile
   SPIR-V, plus a `terrain_ao` bench scene whose last tile is the same frame
   with AO off.
4. **Rebalance the environment discounts.** Separate, optional, judged on
   `palette`.

### A prerequisite for step 3

`visual_bench` has **no marching-cubes terrain scene**. Every existing scene is
props and primitives on a flat slab, which means terrain AO would be
unfalsifiable from an agent shell — and the bench has already misled twice by
being too flattering (see `VISUAL_HANDOFF.md`). A scene with real generated
terrain, a crevice, an overhang and a fresh crater is part of step 3, not a
follow-up to it.

## Traps carried over from the SDF attempt

Recorded because they cost a whole branch, and because most of them are
technique-independent.

**Seams were shipped as a known defect and never fixed.** Cone samples were
clamped to the current leaf and the bake had no halo, both documented as
"follow-up". A feature whose known defect is visible seams across the whole world
does not survive first contact. The halo is not an optimisation to defer; it is
the correctness condition. Here it is sized up front and asserted.

**AO silently vanished over large areas.** Chunks were mesh-octree leaves, which
subdivide by triangle count — a large flat low-triangle region became one leaf
too big for its atlas slot and was logged-and-skipped. Flat ground, where AO's
absence is most obvious. The fix, structurally, is to bind AO to the fixed
32³-voxel chunk lattice rather than to anything adaptive, which the chunked
terrain rework has since made the natural choice anyway.

**A single hand-tuned strength constant with no calibration.** `k = 1.6`,
"tuned by eye". With no baseline subtraction there is no value of `k` that makes
both flat ground and a crevice correct. Calibrate, then tune.

**The first real-level run was a crash, not a picture.** Nothing about the
approach was validated before a large amount of GPU residency machinery existed
to support it. Step 1 above is standalone and testable for exactly this reason.
