# Baked Ambient Occlusion — Design Doc

Stage 1 of the [Lighting Plan](LIGHTING_PLAN.md). Adds a per-vertex occlusion factor to terrain meshes, computed at mesh-generation time by sampling the voxel density field in a neighborhood around each vertex. The factor darkens the ambient lighting term in the fragment shader, making crevices, corners, and concavities read visually without any runtime cost.

## Goals

- Visible darkening in terrain recesses (under ledges, inside dips, between close surfaces).
- Smooth, flicker-free output — no banding, no chunk-boundary seams.
- Zero runtime cost after bake; results baked into the vertex buffer.
- Incremental: recomputable per affected region when terrain is edited.
- No impact on the physics engine or collision geometry.

## Non-goals

- Dynamic / moving-object AO (that's SSAO or similar runtime effects, out of scope for this stage).
- Global illumination or bounced light.
- AO on non-terrain meshes (player, props). Those can get runtime AO later if needed; for now terrain carries the bulk of the visual impact.

## Background: AO techniques considered

Three options were weighed:

**1. Minecraft-style per-corner AO.** For blocky voxels, each mesh vertex sits at a voxel corner and AO is computed from the occupancy of the 3 neighbor voxels diagonally adjacent. Cheap, iconic look — but our mesh comes from marching cubes with smooth surfaces, not axis-aligned quads. Vertices don't sit on voxel corners; they sit on interpolated edge crossings. This technique doesn't apply.

**2. SDF cone-trace AO (Iñigo Quilez).** For a true signed distance field, AO at point `p` with normal `n` can be approximated by sampling the SDF at a few points along the normal and comparing expected-vs-actual distance:

```
ao = 1 - k * Σᵢ (dᵢ - sdf(p + n·dᵢ)) * falloff(i)
```

Elegant and fast (5–8 samples). **The problem**: our underlying field is a voxel *density* field, not a true SDF. Density is signed and roughly linear near the surface, but far from the surface it does not equal Euclidean distance. Using it as an SDF produces noisy / wrong AO away from the surface.

**3. Hemisphere ray sampling against the voxel grid.** Cast N short rays from each vertex into the hemisphere oriented by its normal; each ray that finds occupied voxels within a radius contributes to occlusion. General-purpose, works correctly because it queries actual occupancy rather than trusting density as distance. More expensive than cone tracing, but parallelizable and well-bounded.

**Chosen: hemisphere ray sampling**, with short rays (local AO only). The extra cost is acceptable given that mesh generation is already the expensive step in terrain updates, and it produces correct results for our non-SDF field.

A future refinement could swap to cone-tracing if we ever store a real distance field alongside density.

## Approach

### Per-vertex algorithm

For each terrain vertex `v` with smoothed normal `n`:

1. Generate `N` sample directions `dᵢ` in a cosine-weighted hemisphere oriented by `n`. `N = 16` as a starting point.
2. For each `dᵢ`, march from `v + n·ε` along `dᵢ` up to a maximum distance `R` (the AO radius), stepping in increments no larger than one voxel.
3. At each step, query the SVO for the density at that point. If density > iso_level, mark this ray as occluded and record the hit distance `tᵢ`.
4. Each occluded ray contributes `smoothstep(R, 0, tᵢ)` to occlusion — closer hits darken more.
5. Sum contributions, divide by `N`, clamp to `[0, 1]`. Store `ao = 1 - occlusion` as the vertex attribute.

The `ε` bias along the normal avoids self-hits from the vertex's own surface. The cosine weighting matches the physical integral being approximated (ambient light arrives weighted by `N·L`).

### Sample directions

Precompute a fixed table of `N` hemisphere directions in tangent space (cosine-weighted, stratified to avoid clumping). At each vertex, transform the table into world space using `n` and an arbitrary tangent basis. A fixed table means all vertices sample identical *relative* directions, which keeps the result deterministic and avoids stochastic flicker between rebuilds of the same region.

### AO radius

A radius of ~2–3 voxel cell sizes captures local crevices without bleeding into unrelated geometry. This should be a tunable parameter on the AO baker, not hardcoded.

### Normals used

Marching cubes already produces per-vertex normals from the density gradient. Those are appropriate for AO — they define the hemisphere orientation. `NormalSmoother` / `NormalClusterer` are physics-only and not part of this path.

## Integration into the codebase

### New component: `AoBaker`

Lives in `src/terrain/ao_baker.rs`. Responsibilities:

- Owns the hemisphere sample-direction table.
- Owns AO config (radius, sample count).
- Exposes `bake(mesh: &mut MarchingCubesMesh, sdf: &impl DensitySource)` which iterates mesh vertices and writes occlusion values.

`DensitySource` is a trait abstracting "give me the density at this world-space point". The SVO (`src/terrain/svo.rs`) implements it. Keeping this behind a trait means the baker can be exercised in tests against analytic density fields (plane, sphere, corner) without bringing up a full SVO.

### Vertex format change

Add `ao: f32` to `Vertex` in `src/rendering/vertex.rs`. Update:

- `Vertex` struct definition.
- `get_attribute_descriptions()` to add a new `VkVertexInputAttributeDescription` at the next location.
- The conversion from `MarchingCubesMesh` to `Vertex` in `MeshOctree::generate_from_voxels()` (`src/terrain/mesh_octree.rs` around line 210).
- Add an `ao: Vec<f32>` field to `MarchingCubesMesh` so it can carry occlusion from the baker through to vertex construction.

### Pipeline order

Mesh generation becomes:

```
SVO sample → MarchingCubes::generate → AoBaker::bake → Vertex conversion → octree insert
```

The AO bake is a distinct step, not folded into marching cubes. This keeps MC focused on geometry and makes the AO stage independently testable, swappable, and skippable (for e.g. the bench viewer if we don't want the cost there).

### Shader changes

`shader/triangle.vert`: pass `inAo` through as a new vertex input and forward to the fragment shader.

`shader/triangle.frag`: the current shader computes `ambient + diffuse * NdotL`. Change to `ambient * ao + diffuse * NdotL`. Direct-light contribution is not modified — AO only attenuates the ambient term, matching its physical meaning.

A stronger, non-physical variant (also slightly attenuating direct light by AO) is a common stylization trick and can be exposed as a material knob later if we want deeper crevices.

### Incremental rebuild

`TerrainManager::update()` marks dirty AABBs and rebuilds affected regions. Two concerns:

1. **AO radius extension.** Editing a voxel at `p` affects AO on vertices within radius `R` of `p`. The regeneration region must be expanded by `R` before mesh generation, otherwise vertices just outside the edited region keep stale AO values.
2. **Chunk-boundary sampling.** When baking AO for a vertex near a chunk boundary, rays must sample the density field across that boundary. Since the `DensitySource` is the SVO (a single global structure), this is automatic — no chunk-local density copies required.

## Performance

### Cost model

Per vertex: `N` rays × `S` steps each = `N·S` density queries.

With `N = 16`, `R = 3·cell_size`, step = `0.5·cell_size` → `S ≈ 6`. That's ~100 SVO queries per vertex.

### Back-of-envelope for terrain chunks

Terrain vertex counts per region depend on surface complexity, but a ballpark for a typical updated region is 2k–10k vertices. At 100 queries per vertex:

- 2k vertices → 200k queries.
- 10k vertices → 1M queries.

SVO point queries are O(log depth) — a handful of pointer chases. Call it 100ns per query conservatively. Then:

- 2k vertices → ~20 ms.
- 10k vertices → ~100 ms.

### Where this lands

Mesh generation currently runs synchronously on the main thread in `TerrainUpdateSystem`. Adding 20–100 ms per region update on top of existing MC cost is likely to produce visible hitches during terrain edits.

Mitigations, in order of preference:

**1. Rayon over vertices.** Ray sampling is embarrassingly parallel — each vertex is independent. Parallelizing the bake across cores should give near-linear speedup and cut wall-clock time by ~4–8× on typical hardware. This is the cheapest fix and should be done from the start.

**2. Reduce sample count.** `N = 8` with a good stratified distribution is often indistinguishable from `N = 16` for short-radius AO. Halves the cost.

**3. Coarse sampling, trilinear interpolation.** Bake AO on a regular grid at lower resolution than vertex density (say, one AO sample per voxel cell), then look up per-vertex AO by trilinear interpolation into that grid. Amortizes cost across vertices that share neighborhoods. Adds complexity; defer until measured to be necessary.

**4. Background-thread mesh generation.** The bigger architectural win is moving all mesh generation (MC + AO bake) off the main thread. That's a broader refactor beyond this stage's scope, but AO's cost strengthens the case for it.

**5. Early-exit on ray marching.** As soon as a ray finds a hit, stop stepping. Already implied by step 3 of the algorithm; worth being explicit.

Starting point: rayon + `N = 16`, measure, and drop to `N = 8` or add interpolation only if the measurement says we need to.

### Memory

`f32` per vertex × typical terrain vertex count (low millions) → a few MB. Negligible.

### Runtime shader cost

One extra attribute read and one multiply in the fragment shader. Unmeasurable.

## Testing

- **Unit tests against analytic density fields** (`src/terrain/ao_baker.rs` test module):
  - Flat plane: AO should be near 1.0 everywhere except at edges.
  - Inside corner (two perpendicular planes meeting): AO should darken predictably toward the corner line.
  - Sphere: AO should be ~1.0 on the outside, ~0 on the inside.
  - Two close parallel planes: AO should darken on inner faces, scaling with gap distance.
- **Determinism test**: baking the same mesh twice produces bit-identical AO values.
- **Incremental test**: bake a region, edit voxels outside the AO radius, rebake only the edit region, check AO at distant-vertex is unchanged.
- **Visual smoke test**: load a saved level, eyeball the terrain for expected crevice darkening. Add a debug overlay that renders AO as grayscale (via the existing `DebugLines`/`DebugOverlays` plumbing or a shader toggle).

## Open questions / future work

- **AO for non-terrain meshes.** The player and other props use separate meshes. They can either bake AO at import time (if static) or rely on SSAO / cheap runtime AO later. Deferred.
- **Coupling AO to sky color.** Once we have a hemisphere sky term (sky color above, ground color below), the "ambient" being attenuated should come from the sky hemisphere in the direction the surface faces. AO extends naturally to this — the hemisphere rays that miss geometry sample the sky instead of a constant. Optional future upgrade.
- **Bent normals.** A richer variant stores the average unoccluded direction alongside AO; used to sample IBL or sky more accurately. Overkill for now.
- **Dynamic terrain during destruction.** The destructibles system performs frequent local edits; AO rebake cost there could matter more than during authoring-time edits. Worth profiling once destruction lands.

## Rollout

1. Implement `AoBaker` with `DensitySource` trait + unit tests against analytic fields.
2. Add `ao: f32` to `Vertex`; plumb through marching cubes output and mesh octree conversion.
3. Parallelize the bake with rayon.
4. Update `triangle.vert` / `triangle.frag`.
5. Wire AO radius into dirty-region expansion in `TerrainManager::update()`.
6. Recompile shaders, visual smoke test, measure per-region bake time, tune `N` / radius.
