# Water System Design

Design document for adding water simulation, rendering, and rigid body interaction to the engine.

---

## Design goals

1. **Heightfield-based simulation.** Water volume is tracked on a 2D grid. No full 3D fluid sim.
   The grid resolution matches or is coarser than the terrain voxel resolution.
2. **Destructible terrain interaction.** When terrain is destroyed, water naturally flows into
   newly opened space. When a floor is destroyed, water drains downward.
3. **Sky island support.** Each water column stores an explicit floor level, so water on a floating
   island does not affect the space below it.
4. **Rigid body buoyancy.** Bodies floating in water experience buoyancy forces, linear drag, and
   angular drag. Objects bob realistically on waves.
5. **Interactive surface waves.** A 2D wave equation simulation runs on a fine-resolution grid
   (~10cm cells) on top of the coarse flow grid (~1-2m cells), producing visible ripples that
   propagate from disturbances (body impacts, terrain destruction, player movement). Both grids
   are universal flat arrays — no per-body tracking, merge/split is automatic via wet/dry cell
   boundaries. An infinite ocean plane (future) surrounds the island.
6. **Particle effects for transients.** Splashes, waterfall streams, and leading-edge spray are
   handled by the existing particle system. The heightfield handles bulk flow.

---

## Level file format

Water is specified in `.level.ron` files alongside terrain and objects. Two constructs are
needed: global ocean configuration and individual water bodies.

### Data structs

```rust
/// Top-level water configuration for the level.
#[derive(Deserialize, Default)]
pub struct WaterConfig {
    /// Sea level for the infinite ocean plane. If None, no ocean is rendered
    /// and boundary cells do not act as sources/sinks.
    pub ocean_level: Option<f32>,

    /// Resolution of the flow grid relative to the voxel grid.
    /// 1 = same resolution as voxels, 2 = half resolution (one flow cell per
    /// 2x2 voxel columns). Default: 2.
    #[serde(default = "default_flow_grid_scale")]
    pub flow_grid_scale: u32,

    /// Wave grid cell size in meters. Controls ripple resolution.
    /// Smaller = finer ripples but more cells. Default: 0.1 (10cm).
    #[serde(default = "default_wave_cell_size")]
    pub wave_cell_size: f32,

    /// Fluid density in kg/m^3. Default: 1000.0 (water).
    #[serde(default = "default_fluid_density")]
    pub fluid_density: f32,

    /// Flow rate coefficient. Higher = faster spreading. Default: 2.0.
    #[serde(default = "default_flow_rate")]
    pub flow_rate: f32,

    /// Wave propagation speed in m/s. Controls how fast ripples travel across
    /// the surface. Default: 4.0 (ripples cross a 10m pond in ~2.5s).
    #[serde(default = "default_wave_speed")]
    pub wave_speed: f32,

    /// Wave damping coefficient in 1/s. Controls how quickly ripples die out.
    /// Default: 0.5 (ripples fade in ~4s, allowing 2-3 bank reflections).
    #[serde(default = "default_wave_damping")]
    pub wave_damping: f32,

    /// Individual water bodies placed in the level.
    #[serde(default)]
    pub bodies: Vec<WaterBody>,
}

/// A discrete body of water placed at level load time.
#[derive(Deserialize)]
pub enum WaterBody {
    /// Fill a rectangular region up to a given surface level.
    /// The floor_level is determined automatically from terrain height.
    /// Only cells where terrain height < surface_level receive water.
    Pool {
        /// XZ center of the pool region.
        center: (f32, f32),
        /// XZ half-extents of the fill region.
        half_extents: (f32, f32),
        /// Target water surface height (world Y).
        surface_level: f32,
    },

    /// Fill all connected terrain below a given height, flood-fill style.
    /// Starts from a seed point and fills outward until terrain rises above
    /// surface_level or the region boundary is reached.
    Lake {
        /// Seed point (x, z) — must be inside a depression.
        seed: (f32, f32),
        /// Target water surface height (world Y).
        surface_level: f32,
    },
}
```

### Addition to Level struct

```rust
#[derive(Deserialize)]
pub struct Level {
    pub name: String,
    pub world_size: f32,
    pub voxel_size: f32,
    pub terrain: Terrain,
    pub player_spawn: (f32, f32, f32),
    pub objects: Vec<LevelObject>,

    /// Water configuration. If omitted, no water system is created.
    #[serde(default)]
    pub water: Option<WaterConfig>,
}
```

### Example: island with ocean and a hilltop pond

```ron
Level(
    name: "Tropical Island",
    world_size: 64.0,
    voxel_size: 1.0,

    terrain: Terrain(
        base_height: -2.0,
        material_layers: [
            (depth: 1.0,  material: Sand),
            (depth: 3.0,  material: Dirt),
            (depth: 999.0, material: Rock),
        ],
        features: [
            Hill(center: (0.0, 0.0), radius: 25.0, height: 12.0),
            // Hilltop crater that holds the pond
            Crater(center: (5.0, 5.0), radius: 6.0, depth: 4.0),
            TerrainRoughness(frequency: 0.1, amplitude: 0.5, octaves: 3, seed: 42),
        ],
        volumes: [],
    ),

    water: Some(WaterConfig(
        ocean_level: Some(0.0),
        grid_scale: 2,
        bodies: [
            // Hilltop pond — fills the crater
            Pool(
                center: (5.0, 5.0),
                half_extents: (7.0, 7.0),
                surface_level: 9.0,
            ),
        ],
    )),

    player_spawn: (0.0, 14.0, 0.0),
    objects: [
        BeachBall(pos: (3.0, 10.0, 3.0)),
    ],
)
```

### Example: sky island with pond

```ron
Level(
    name: "Sky Pool",
    world_size: 64.0,
    voxel_size: 1.0,
    terrain: Terrain(
        base_height: -20.0,
        features: [],
        volumes: [
            Island(center: (0.0, 15.0, 0.0), half_extents: (10.0, 3.0, 10.0), edge_noise: 0.2),
        ],
    ),

    water: Some(WaterConfig(
        ocean_level: None,      // no ocean — it's a sky level
        bodies: [
            // Pond on the sky island surface
            Pool(
                center: (0.0, 0.0),
                half_extents: (5.0, 5.0),
                surface_level: 17.5,    // slightly below island rim
            ),
        ],
    )),

    player_spawn: (0.0, 20.0, 0.0),
    objects: [],
)
```

### Spawner integration

The level spawner creates the `WaterGrid` after terrain generation:

1. Build terrain from features (existing flow).
2. If `level.water` is `Some`:
   a. Create `WaterGrid` with dimensions derived from `world_size / voxel_size / grid_scale`.
   b. Set `ocean_level`.
   c. For each `WaterBody`:
      - `Pool`: iterate cells in the XZ region, query `approx_surface_height_at` for the floor,
        compute volume = `(surface_level - floor) * cell_area` for cells where floor < surface_level.
      - `Lake`: BFS flood-fill from the seed cell, expanding to neighbors where terrain height
        < surface_level. Set volume per cell as above.
   d. Insert `WaterGrid` as an ECS resource.

---

## Relationship to the voxel terrain system

### VoxelMaterial::Water — removed

The existing `VoxelMaterial::Water` variant in `src/terrain/voxel.rs` is unused (marked with
`#[allow(dead_code)]`). It was a placeholder for a hypothetical approach where water would be
a voxel type processed by marching cubes alongside rock/dirt/grass.

This design replaces that approach. Water is **not** a voxel material — it is a separate
simulation layer that sits on top of the terrain. `VoxelMaterial::Water` should be removed to
avoid confusion. The `is_solid()` method already excludes it from collision, confirming it was
never integrated.

### Separation from TerrainManager

The water system is **not** part of `TerrainManager`. Reasons:

- `TerrainManager` owns the SVO, mesh octree, adjacency map, and render data for solid
  terrain. Water has none of these — it has its own grid, its own mesh, its own shader.
- `TerrainManager` implements `StaticGeometry` for physics queries. Water does not provide
  static collision triangles — it applies forces (buoyancy) to bodies instead.
- Water updates every frame (flow sim). Terrain only rebuilds when damaged. Coupling them
  would force the terrain rebuild path to run every frame.

The water system lives in `src/water/` as its own module. It reads from `TerrainManager` but
does not write to it.

### Interface: what the water system needs from TerrainManager

The water system needs two queries from the terrain:

#### 1. Column height query (new method on TerrainManager)

```rust
impl TerrainManager {
    /// Get the highest solid terrain surface height at a given (x, z) position.
    /// Returns None if the column is entirely air (no solid voxels).
    pub fn approx_surface_height_at(&self, x: f32, z: f32) -> Option<f32>;

    /// Get all solid surface heights in a column (for sky islands with multiple
    /// layers). Returns heights sorted top-to-bottom.
    /// Used by the water system to find the floor that water rests on, and to
    /// detect when a floor has been destroyed.
    pub fn surface_heights_at(&self, x: f32, z: f32) -> Vec<f32>;
}
```

These are column queries against the SVO. Walk down the voxel column at `(x, z)` and find
sign transitions (positive density → negative density = surface). This is cheap — it is an
octree traversal of a single column, not a spatial query over triangles.

**Why not use `StaticGeometry::query_region`?** That returns triangles in an AABB, which is
overkill for "what is the terrain height at this XZ point." A column query is O(log N) in
SVO depth, while an AABB query returns potentially hundreds of triangles.

#### 2. Terrain damage notification

When `TerrainManager::damage_sphere()` destroys voxels, the water system needs to know so it
can recheck floor levels in the affected region. Two options:

- **Poll (simpler):** Each frame, the water system checks `TerrainManager::dirty_regions` (or
  a public accessor for recently-damaged regions) and rechecks floor levels in those regions.
- **Event (cleaner):** `damage_sphere` emits an event (e.g. pushes to a `Vec<AABB>` of
  "terrain modified" regions) that the water system consumes.

The poll approach fits naturally since `TerrainManager` already tracks `dirty_regions` — we
just need to expose them before they are cleared by `update()`.

```rust
impl TerrainManager {
    /// Regions damaged since the last call to `update()`.
    /// The water system reads these to invalidate floor levels.
    pub fn dirty_regions(&self) -> &[AABB];
}
```

---

## Data model

The water system uses **two separate grids** at different resolutions:

- **Flow grid** (coarse, ~1-2m cells): tracks water volume and bulk surface level. Handles
  pressure equalization, drainage, terrain interaction. Same resolution as in the current
  implementation.
- **Wave grid** (fine, ~10cm cells): tracks surface displacement and velocity for the 2D wave
  equation. Handles ripples, reflections, body interactions. Runs on top of the flow grid's
  bulk level.

Both grids are **universal flat arrays** covering the full terrain extent. No per-body
allocation, no body tracking, no merge/split logic. Wet/dry boundaries in the wave grid
naturally reflect waves. When terrain destruction connects two ponds, wave cells between them
become wet and ripples propagate through. When terrain splits a pond, those cells become dry
and waves reflect off the new boundary. This happens automatically — the wave equation just
sees wet and dry cells.

### FlowCell

```rust
struct FlowCell {
    /// Volume of water in this cell (cubic units). This is the primary state.
    /// A cell with volume == 0.0 is dry.
    volume: f32,

    /// Terrain surface this water rests on (world Y). Set when water is placed
    /// or flows into a cell via `TerrainManager::surface_heights_at()`.
    /// Used to derive bulk_level and to prevent underwater detection for
    /// entities below a sky island.
    floor_level: f32,
}
```

The **bulk water surface level** is derived:

```
bulk_level = floor_level + volume / cell_area
```

### FlowGrid

```rust
struct FlowGrid {
    cells: Vec<FlowCell>,         // row-major, dims.0 * dims.1
    dims: (usize, usize),        // grid dimensions (x, z)
    cell_size: f32,               // world-space width/depth of each cell (~1-2m)
    cell_area: f32,               // cached cell_size * cell_size
    origin: Vec3,                 // world-space position of grid corner (0,0)
    ocean_level: Option<f32>,     // sea level for the infinite ocean plane
    flow_rate: f32,               // equalization speed multiplier
    settle_epsilon: f32,          // equilibrium threshold
    settled: bool,                // skips flow work once the grid reaches equilibrium
    delta_volume: Vec<f32>,       // per-cell flow accumulator
    dirty_floors: Vec<(usize, usize)>,
}
```

### WaveCell

```rust
struct WaveCell {
    /// Surface displacement from the bulk water level.
    /// Positive = above bulk level, negative = below.
    displacement: f32,

    /// Time derivative of displacement (vertical velocity of the surface).
    velocity: f32,
}
```

### WaveGrid

```rust
struct WaveGrid {
    cells: Vec<WaveCell>,         // row-major, dims.0 * dims.1
    dims: (usize, usize),        // grid dimensions (x, z)
    cell_size: f32,               // world-space width/depth of each cell (~10cm)
    origin: Vec3,                 // same origin as flow grid
    wave_speed: f32,              // propagation speed c (m/s)
    wave_damping: f32,            // damping coefficient d (1/s)
}
```

### Grid relationship

The wave grid covers the same spatial extent as the flow grid but at higher resolution. Each
flow cell maps to a block of wave cells:

```
wave_cells_per_flow_cell = flow_cell_size / wave_cell_size
```

For `flow_cell_size = 2.0m` and `wave_cell_size = 0.1m`, each flow cell maps to a 20x20 block
of wave cells.

A wave cell is **wet** if the flow cell it maps to has `volume > 0`. The wave equation only
steps wet wave cells. When a flow cell transitions wet→dry, all wave cells in that block are
zeroed (displacement = 0, velocity = 0). When a flow cell transitions dry→wet, the wave cells
start at zero and respond to any incoming disturbances.

The **rendered surface level** at any wave cell is:

```
rendered_level = flow_cell.bulk_level() + wave_cell.displacement
```

Buoyancy sampling also uses `rendered_level` so bodies ride the waves.

### Memory budget

For a 128m terrain:
- Flow grid at 2m cells: 64x64 = 4,096 cells, ~32KB
- Wave grid at 10cm cells: 1,280x1,280 = 1,638,400 cells, ~12.5MB (2 floats per cell)

12.5MB is acceptable. Only wet cells are iterated, so the computational cost is proportional
to the water surface area, not the terrain area. A 10m pond at 10cm resolution is 100x100 =
10,000 wave cells per step — trivial.

**Future optimization (sparse tiles):** if memory becomes a concern, allocate the wave grid in
tiles (e.g. 32x32 wave cells per tile). Only allocate tiles that overlap wet flow cells. This
halves memory for a half-dry terrain without changing the algorithm — tile-internal iteration
is still flat-array, and tile boundary lookups check the neighbor tile. Not needed for v1.

Both grids are axis-aligned in the XZ plane. Cell `(i, j)` covers the world-space rectangle
`[origin.x + i*cell_size, origin.x + (i+1)*cell_size]` x
`[origin.z + j*cell_size, origin.z + (j+1)*cell_size]`.

---

## Flow simulation

### Continuous vs event-driven

The current implementation runs **continuously every frame** but iterates the full grid while
the simulation is active.

- **Quiescent state**: when surface level differences fall below `settle_epsilon`, the
  simulation sets `settled = true` and `step()` becomes an early return.
- **Terrain damage re-activation**: when `dirty_floors` is non-empty (terrain was destroyed),
  `settled` is cleared and the affected cells are reprocessed on the next step.
- **Future optimization**: the original `active_cells` plan is still reasonable, but it is not
  implemented yet because the current grid sizes are small enough that full-grid iteration is
  acceptable for bring-up.

This means a calm pond costs nearly zero per frame (settled = true, skip). A pond that just
had its bank blown up runs the full-grid flow sim until equilibrium is reached again.

### Optional optimization: active cells

The original design still includes an **`active_cells` optimization** for cases where the water
grid becomes large enough that full-grid iteration is wasteful.

Concept:
- Track all wet cells, plus their immediate neighbors, in `active_cells`.
- During flow, only iterate that working set instead of the entire grid.
- Rebuild or incrementally maintain the set as cells become wet, dry, or get marked dirty.

Expected behavior:
- A small pond in a large map only touches a few dozen to a few hundred cells.
- A calm pond converges to `settled = true`, so even the active set stops being processed.
- Terrain damage near a shoreline or pond only wakes the local region instead of the whole grid.

One reasonable implementation shape:

```
active_cells: Vec<(usize, usize)>
active_flags: Vec<bool>  // same dimensions as grid, avoids duplicates

fn activate(i, j):
    if !active_flags[index(i, j)]:
        active_flags[index(i, j)] = true
        active_cells.push((i, j))

fn rebuild_active_set():
    clear active_cells and active_flags
    for each wet cell:
        activate(i, j)
        activate(left/right/front/back neighbors if in bounds)
```

Incremental maintenance option:

```
for each cell with non-zero delta:
    if cell became wet:
        activate(cell)
        activate(neighbors)

    if cell became dry:
        leave it active for this step

after step:
    rebuild_active_set() if many cells changed
    // or lazily prune cells that are dry and have no wet neighbors
```

This is not required for correctness. It is a performance optimization that preserves the same
water model while making the cost proportional to the active water surface area rather than the
full grid area.

### Flow algorithm

Pressure equalization between neighbors, run once per physics step:

```
delta_volume: Vec<f32>  // per-cell accumulator, zeroed each step

for each cell (i, j):
    h_self = cell.surface_level()  // floor_level + volume / cell_area

    for each neighbor (ni, nj) in [left, right, front, back]:
        h_neighbor = neighbor.surface_level()
        delta_h = h_self - h_neighbor

        if delta_h > 0:
            flow = min(delta_h * flow_rate * dt, cell.volume * 0.25)
            delta_volume[(i,j)] -= flow
            delta_volume[(ni,nj)] += flow

// Apply accumulated deltas
for each cell with non-zero delta:
    cell.volume += delta_volume[cell]
    cell.volume = max(cell.volume, 0.0)

    // Set floor_level for newly-wet cells
    if cell was dry and is now wet:
        cell.floor_level = terrain.approx_surface_height_at(cell_x, cell_z)
```

The `0.25` cap ensures a cell never donates more than its total volume across 4 neighbors in
one step (stability constraint). The separate `delta_volume` accumulator avoids order-dependent
artifacts.

### Timestep stability

The flow rate `flow_rate * dt` must stay below a stability threshold, otherwise water
oscillates or explodes. The constraint is:

```
flow_rate * dt < 0.25
```

(A cell must not try to donate more than 25% of its height differential per step, which the
`min(... cell.volume * 0.25)` already enforces for volume, but the rate itself should be
capped to prevent oscillation.)

**Variable timestep handling**: clamp `dt` to a maximum of `max_water_dt` (e.g. 1/30s). If
the game's frame dt exceeds this, substep the water simulation:

```rust
fn step(&mut self, dt: f32, terrain: &TerrainManager) {
    let max_dt = 1.0 / 30.0;
    let mut remaining = dt;
    while remaining > 0.0 {
        let sub_dt = remaining.min(max_dt);
        self.step_internal(sub_dt, terrain);
        remaining -= sub_dt;
    }
}
```

This is still cheap at the current grid sizes, and the common case (60fps, dt=0.016s) does a
single substep. The substep guard only kicks in during lag spikes.

Starting value for `flow_rate`: **2.0**. This gives water that spreads visibly over ~1-2
seconds for small height differentials. Tune by watching a pond drain through a gap — it
should look natural, not instantaneous or glacially slow.

### Boundary cells

Cells at the grid edges are **ocean boundary cells** (only when `ocean_level` is `Some`).
They act as infinite sources/sinks at `ocean_level`:
- If `ocean_level > cell.surface_level()`, water flows in from the ocean.
- If `cell.surface_level() > ocean_level`, water flows out (drains to ocean).

Implementation notes:
- Ocean coupling is only active when the boundary cell's `floor_level <= ocean_level`.
  Coastlines above sea level do not auto-flood just because they touch the grid edge.
- Sea-connected boundary cells are treated as a reservoir and clamped back to their ocean target
  volume after each step, so they remain a stable source/sink instead of storing excess water.
- `surface_level_at()` should report the ocean surface for dry sea-connected boundary cells so
  gameplay queries and rendering queries see the same boundary state as the simulation.

### Floor collapse (drain events)

Triggered when `dirty_floors` is non-empty (terrain was damaged under water):

```
for each (i, j) in dirty_floors:
    if cell.volume == 0: continue

    old_floor = cell.floor_level
    new_floor = terrain.approx_surface_height_at(cell_x, cell_z)

    if new_floor is None:
        // Floor completely destroyed — drain all water, spawn falling particles
        emit_waterfall_particles(cell_x, cell_z, old_floor, volume)
        cell.volume = 0

    else if new_floor < old_floor - threshold:
        // Floor dropped — water falls to new level
        cell.floor_level = new_floor
        // Volume stays the same, but surface_level drops because floor dropped.
        // This creates a height differential with neighbors, so flow sim handles
        // the redistribution naturally.
        emit_waterfall_particles(cell_x, cell_z, old_floor, new_floor)
```

For sky islands with multiple terrain layers, `surface_heights_at` returns all surfaces.
When the top surface is destroyed, the water's floor snaps to the next surface below, or
drains entirely if none exists.

### Data structure efficiency

- **`delta_volume: Vec<f32>`** — flat array matching the grid dimensions. Reused every step and
  zeroed in place.
- **`settled: bool`** — set to true when neighboring surface differences fall below epsilon.
  Cleared by terrain damage or volume injection. When settled, the entire `step()` call is a
  single branch.
- **Future optimization: `active_cells`** — still a good next step if large water grids become
  hot, but not currently needed for correctness.

Grid sizing: for a 128x128 voxel terrain, a 64x64 water grid (2:1 ratio) is 4096 cells.
Even iterating the full grid is cheap enough for the current implementation, which is why the
active-cell optimization has been deferred.

---

## Wave equation surface

The flow simulation (on the coarse `FlowGrid`) produces a bulk water level per cell that
changes slowly (pond filling, draining). On top of this sits the **wave equation** simulation
(on the fine `WaveGrid`) that produces fast, reactive surface ripples — the "thin membrane."

### Why not Gerstner waves

Gerstner waves are a sum of sinusoids with fixed amplitude, frequency, and direction. They
produce convincing open-ocean swells but are fundamentally non-interactive — you cannot drop a
beach ball into a Gerstner wave field and see ripples radiate from the impact. They are a good
fit for the future infinite ocean plane (no simulation backing needed, just shader math), but
wrong for ponds and lakes where interactivity is the point.

Gerstner waves remain the plan for the **ocean plane** (Step 3) where there is no simulation
grid and the surface is purely cosmetic.

### The wave equation

The 2D wave equation governs how displacement propagates across the surface:

```
d²h/dt² = c² * ∇²h - d * dh/dt
```

Where:
- `h` = `displacement` at each wave cell (surface height offset from bulk level)
- `c` = wave propagation speed (m/s) — controls how fast ripples travel
- `∇²h` = discrete Laplacian of h over the wave grid
- `d` = damping coefficient — prevents perpetual oscillation

### Discrete integration

Each wave cell stores `displacement` (h) and `velocity` (dh/dt). Each step:

```
for each wet wave cell (i, j):
    // Discrete Laplacian: sum of neighbor displacements minus 4x center
    // Neighbors that are dry have displacement = 0 (reflective boundary)
    lap = (h[i-1,j] + h[i+1,j] + h[i,j-1] + h[i,j+1] - 4 * h[i,j])
          / (wave_cell_size * wave_cell_size)

    // Semi-implicit Euler: update velocity, then displacement
    velocity[i,j] += (c * c * lap - damping * velocity[i,j]) * dt
    displacement[i,j] += velocity[i,j] * dt
```

This is O(wet wave cells) per step and trivially parallelizable. A 10m pond at 10cm resolution
is 10,000 wave cells — each doing ~10 FLOPs per step.

**Wet/dry determination**: a wave cell is wet if the flow cell it maps to has `volume > 0`.
This is a lookup into the flow grid, not per-wave-cell state. When a flow cell transitions
dry→wet, the wave cells in that block start at zero and respond to incoming disturbances.
When a flow cell transitions wet→dry, the wave cells are zeroed.

**Boundary conditions**: at the edge of a water body (dry neighbor), the neighbor's
displacement is treated as 0. This reflects waves back from shores — physically correct for
pond banks. For open edges (ocean boundary cells, future), use absorbing boundary conditions
(set neighbor displacement = cell displacement) to let waves pass through without reflection.

**Topology changes from terrain destruction**: when an explosion connects two ponds, the flow
cells between them become wet, which makes the corresponding wave cells wet. Ripples from one
pond naturally propagate into the other via the wave equation — no merge logic needed. When
terrain splits a pond, the flow cells in the divide become dry, zeroing the wave cells. Waves
reflect off the new boundary. All of this is automatic from the wet/dry determination.

### Stability

The discrete wave equation has a CFL stability condition:

```
c * dt / cell_size < 1/√2  ≈ 0.707
```

If this is violated, the simulation explodes. For `wave_cell_size = 0.1` and `c = 4.0`:

```
dt_max = 0.707 * 0.1 / 4.0 = 0.0177s
```

At 60fps (dt = 0.016s) this is **just barely within limits**. A small safety margin is needed.
The substep guard handles this:

```rust
fn step_waves(&mut self, dt: f32) {
    let max_dt = 0.5 * self.cell_size / self.wave_speed;  // CFL with safety margin
    let mut remaining = dt;
    while remaining > 0.0 {
        let sub_dt = remaining.min(max_dt);
        self.step_waves_internal(sub_dt);
        remaining -= sub_dt;
    }
}
```

At 60fps with the above parameters, `max_dt = 0.5 * 0.1 / 4.0 = 0.0125s`, so each frame
does 2 substeps. This is still cheap (20,000 cell iterations for a 10m pond per frame).

If the substep count becomes a concern, the options are:
- Increase `wave_cell_size` (coarser ripples, fewer cells, more CFL headroom)
- Decrease `wave_speed` (slower ripples, more CFL headroom)
- Use a larger cell size like 15cm instead of 10cm (CFL max_dt increases to 0.027s, single
  substep at 60fps)

### Starting parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `wave_speed` (c) | 4.0 m/s | Ripples cross a 10m pond in ~2.5s — visible but not instant |
| `damping` (d) | 0.5 /s | Ripples die out in ~4s — enough time to see 2-3 reflections off the bank |

These are tuning values. `wave_speed` controls how "responsive" the water looks.
`damping` controls how long disturbances persist. Both should be configurable in `WaterConfig`.

### Injecting disturbances

Interactions with the wave surface are modeled as **displacement injections** — directly
modifying `displacement` and/or `velocity` on wave cells at specific world positions:

- **Body impact** (body enters water): set `velocity` negative (pushes surface down) on the
  wave cells under the body, proportional to the body's downward velocity. At 10cm wave
  resolution, a 1m-diameter beach ball touches ~80 wave cells, producing a detailed splash
  depression followed by a rebound ring.

- **Body bobbing** (body floating): each frame, the body's vertical oscillation continuously
  perturbs the surface at its footprint wave cells. This creates standing wave patterns around
  floating objects.

- **Terrain destruction** (bank blown up): when the flow sim starts moving water through a new
  gap, inject negative displacement at the leading edge wave cells. This creates a visible
  wave front ahead of the bulk flow.

- **Player movement** (walking/swimming through water): inject displacement at the wave cells
  around the player's position, proportional to movement speed. Creates a wake behind the
  player.

The wave equation naturally handles propagation, reflection, and interference — injecting at a
point produces circular ripple rings. Two objects bobbing near each other produce interference
patterns. Waves reflect off terrain banks and superimpose. All of this is emergent from the
equation, no special-casing needed.

### Interaction with flow simulation

The two grids are **decoupled** — they do not modify each other's state:

- The flow grid reads and writes `volume`. It computes `bulk_level` from `volume / cell_area`.
- The wave grid reads and writes `displacement` and `velocity`.
- The wave grid reads `bulk_level` from the flow grid to determine wet/dry status.
- Rendering uses `bulk_level + displacement`.
- Buoyancy uses `bulk_level + displacement`.

The wave displacement does not create or destroy water volume — it is a surface perturbation
only. This keeps both simulations simple and independently stable.

One coupling point: when the flow sim changes `bulk_level` significantly (e.g. terrain
destruction causes rapid drainage), this implicitly changes the surface that the waves sit on.
No special handling is needed — the wave displacement is relative to whatever the current
bulk level is.

---

## Rendering

Three rendering layers, from back to front:

### 1. Infinite ocean plane (future — Step 3)

A large quad rendered at `ocean_level` with a procedural water shader. Analogous to the existing
sky renderer — fullscreen coverage, no simulation backing.

- **Vertex shader**: Gerstner wave displacement for animated wave geometry (non-interactive,
  purely cosmetic — appropriate for the open ocean where there is no simulation grid).
- **Fragment shader**: Fresnel-based blend between reflection (sky color) and refraction (depth-
  based tint). Specular highlights from the sun direction.
- Rendered before terrain geometry, with depth test enabled so terrain and water mesh occlude it.

### 2. Simulated water mesh

A grid mesh generated from the `WaveGrid` each frame. The mesh is at **wave grid resolution**
(~10cm cells), giving smooth, detailed ripples. Each vertex position:

```
x = origin.x + i * wave_cell_size
y = flow_cell.bulk_level() + wave_cell.displacement
z = origin.z + j * wave_cell_size
```

The `bulk_level` comes from the flow grid cell that this wave cell maps to. The `displacement`
comes from the wave equation simulation. Together they produce a smooth surface with both the
macro water level and fine ripple detail.

- Only wet wave cells generate mesh quads. A wave cell is wet if its parent flow cell has
  `volume > 0`.
- The mesh uses the same water shader as the ocean plane (or a variant sharing the same fragment
  shader with a different vertex source).
- **Z-fighting at shoreline**: In the water fragment shader, sample the terrain depth buffer and
  discard water fragments that are behind terrain. This produces a clean waterline.

**Wet-dry boundary**: at the edge of a water body, one side of a quad has volume > 0 and the
other side is dry. Options:
- **Simplest (start here):** only emit quads where all 4 corner cells are wet. This leaves a
  staircase edge one cell_size inside the actual shoreline — acceptable at small cell sizes.
- **Better:** emit partial quads at the boundary, with the dry-side vertices pulled down to
  the terrain surface. This creates a tapered edge that meets the beach naturally.
- **Ocean seam (future):** where the simulated water mesh meets the ocean plane at boundary
  cells, the simulated mesh uses `ocean_level` as its bulk level and absorbing boundary
  conditions for waves, so the wave equation surface blends smoothly into the Gerstner ocean.
  The simulated mesh takes over from the ocean plane for cells that have active simulation.

### 3. Particle effects

Existing `ParticleRenderer` handles:
- Splash particles on rigid body impact with water surface.
- Waterfall particles for drain events.
- Foam/spray particles at the leading edge of fast-flowing water.

### Shader files

New shader pair in `shader/`:
- `water.vert` / `water.frag` — shared by ocean plane and water mesh (uniform flag to switch
  between infinite plane mode and mesh mode, or two separate pipelines if the vertex shaders
  diverge significantly).

Compile to SPIR-V like existing shaders:
```bash
glslc shader/water.vert -o shader/water.vert.spv
glslc shader/water.frag -o shader/water.frag.spv
```

### Descriptor set

The water shader needs:
- Set 0: Scene UBO (model/view/projection, camera position, sun direction, time) — same as
  existing pipelines.
- Set 1: Water-specific uniforms (ocean_level, wave parameters, water color/tint).
- Optionally: a depth texture from the terrain pass for the shoreline clip.

---

## Rigid body interaction

### Buoyancy

Applied during force accumulation (physics pipeline phase 1), before the solver runs.

For each dynamic body with a collider:

1. **Sample water height** at probe points on the body:
   - `Sphere`: single point at bottom of sphere, plus analytically compute spherical cap volume
     for the submerged portion.
   - `Box`: 4 bottom-face corners. Submerged fraction estimated from how many probes are below
     the surface and by how much.
   - `Capsule`: 2 hemisphere center points. Submerged fraction from hemisphere cap volumes.

2. **Query the water grid** at each probe's (x, z) position. The effective surface height is
   `bulk_level + wave_displacement`. A probe is underwater if:
   ```
   probe.y > cell.floor_level && probe.y < cell.bulk_level() + cell.wave_displacement
   ```
   The floor check prevents false positives under sky islands. Using the wave-displaced
   surface means bodies ride the waves — a passing ripple lifts and drops a floating object.

3. **Apply forces**:
   - **Buoyancy**: upward force = `fluid_density * gravity * submerged_volume`. Applied at the
     centroid of the submerged region (not the body center) to generate realistic torques.
   - **Linear drag**: force opposing velocity, proportional to submerged fraction and velocity
     magnitude. `F_drag = -drag_coeff * submerged_fraction * velocity`.
   - **Angular drag**: torque opposing angular velocity, proportional to submerged fraction.

4. **Wave coupling**: when a body enters or moves through water, inject a disturbance into the
   wave equation at the body's (x, z) cells. Specifically, set `wave_velocity` negative
   (proportional to impact speed) to push the surface down, producing a splash depression
   that radiates outward as circular ripples. Floating bodies continuously perturb the surface
   as they bob, creating standing wave patterns. See the "Injecting disturbances" section for
   details.

### Material properties

Add to `ColliderMaterial` or a new `BuoyancyProperties` component:

```rust
struct BuoyancyProperties {
    /// Density of the body (kg/m^3). Objects with density < fluid_density float.
    /// Wood ~500, steel ~7800, beach ball ~50.
    density: f32,

    /// Drag coefficient. Higher = more resistance in water.
    drag: f32,

    /// Angular drag coefficient.
    angular_drag: f32,
}
```

---

## Integration with existing systems

### ECS integration

New resources and systems:

| Component / Resource    | Purpose                                           |
|-------------------------|---------------------------------------------------|
| `FlowGrid` (Resource)  | Coarse water heightfield, stepped by `WaterSystem` |
| `WaveGrid` (Resource)  | Fine wave displacement grid, stepped by `WaterSystem` |
| `BuoyancyProperties`   | Per-entity buoyancy config                        |
| `WaterSystem`           | Steps both flow and wave simulations each frame   |
| `BuoyancySystem`        | Applies buoyancy/drag forces to rigid bodies      |
| `WaterRenderSystem`     | Generates the water mesh, submits draw calls      |

**System ordering:**
```
WaterSystem (step flow sim + wave equation)
  -> BuoyancySystem (sample water heights, apply forces, inject wave disturbances)
    -> PhysicsSystem (solver sees buoyancy forces)
      -> WaterRenderSystem (generate mesh from wave grid)
```

### Physics engine boundary

The physics engine (`src/physics/`) has no ECS dependency and does not know about water.

Buoyancy forces are applied by the ECS `BuoyancySystem` **before** `PhysicsWorld::step()` is
called. The system computes the buoyancy force vector and the submerged centroid for each body,
converts force to impulse (`force * dt`), and calls `RigidBody::apply_impulse_at_point(impulse,
buoyancy_centroid)`. This existing method (in `src/physics/body.rs`) applies both a linear
velocity change and an angular velocity change from the torque arm (`centroid - center_of_mass`).
This means a half-submerged box naturally tilts toward its heavier side — no physics engine
changes are needed.

Similarly, linear drag and angular drag are applied as impulses opposing the current velocity:
```rust
let drag_impulse = -drag_coeff * submerged_fraction * body.linear_velocity() * dt;
body.apply_impulse(drag_impulse);

let angular_drag_torque = -angular_drag * submerged_fraction * body.angular_velocity() * dt;
body.apply_angular_impulse(angular_drag_torque);  // may need to be added to RigidBody
```

**Why not use `ForceField`?** The existing `ForceField` enum applies forces at the body's
center of mass only (no off-center torques). It also lives inside the physics engine, and
adding a `Buoyancy` variant would require the physics engine to know about `WaterGrid`,
breaking the separation of concerns. Applying impulses from the ECS layer is the cleaner path.

The `WaterGrid` lives in `src/water/`, not `src/physics/`.

---

## Module structure

```
src/water/
    mod.rs              — re-exports
    flow_grid.rs        — FlowGrid, FlowCell, flow simulation
    wave_grid.rs        — WaveGrid, WaveCell, wave equation simulation
    buoyancy.rs         — buoyancy force calculation (no ECS dependency)

src/systems/
    water.rs            — WaterSystem (ECS: steps WaterGrid, reads TerrainManager dirty regions)
    buoyancy.rs         — BuoyancySystem (ECS: reads bodies + water, applies forces)

src/rendering/
    water_renderer.rs   — WaterRenderer (Vulkan pipeline, mesh generation, draw)

shader/
    water.vert          — vertex shader (reads per-vertex y from wave sim)
    water.frag          — fragment shader (Fresnel, depth-based tint, shoreline clip)
```

---

## Edge cases

### Water flowing off a sky island

When water flows to the edge of a sky island and the neighboring cell has no terrain at a
comparable height, the neighbor's `approx_surface_height_at` returns either the ground far below or
`None`. The flow sim sees a huge height differential and rapidly drains water from the edge
cell. This is correct behavior — water falls off the island. Spawn waterfall particles at the
island edge for the visual.

The drained water should appear at the ground level below if there is terrain there. The
simplest approach: treat it as lost volume (falls into the void or ocean). For a more complete
simulation, query `approx_surface_height_at` at the ground level and add the volume to that cell,
but this is a Step 6 refinement.

### Two water bodies at different heights in the same column

This can happen if a sky island has a pond and the ground below also has water. The 2D
heightfield cannot represent two water levels in the same (x, z) cell.

Resolution: water on a sky island and water on the ground are **separate `WaterGrid`
instances**. Each level can have multiple water grids, each associated with a different floor
layer. In practice, most levels need at most two (ground + one sky island). The level file's
`WaterBody` entries implicitly group by floor layer — the spawner checks the floor height of
each body and assigns it to the appropriate grid.

However, for the initial implementation, support **one `WaterGrid` per level** and document
that water on overlapping sky islands is a future extension. This keeps the implementation
simple and covers 90% of use cases (single-layer terrain with ocean, or a sky island with a
pond but no water below it).

### Explosion creates terrain above existing water

If an explosion somehow deposits terrain above water (e.g. debris fill), this is not currently
possible — `damage_sphere` only destroys voxels, it does not create them. If terrain creation
is added later (e.g. a "build" tool), the water system would need a "ceiling check" — detect
that a solid voxel now exists above a water cell's surface and reduce the water volume
accordingly (displace it to neighbors). For now, this case cannot occur and can be ignored.

### Water volume conservation under terrain destruction

When terrain is destroyed under a pond, the floor drops. The water's volume stays the same but
the surface level drops (because `surface_level = floor_level + volume / cell_area` and
`floor_level` decreased). This means the water column gets taller but its surface drops — which
is physically wrong. What should happen: the water should maintain its surface level and
volume should be redistributed.

The correct handling in floor collapse: if `new_floor < old_floor`, compute the volume that
was "above" the old floor: `volume_above = volume`. This same volume now sits on the new,
lower floor. The surface level will be `new_floor + volume / cell_area`, which is lower than
before if `new_floor < old_floor`. The flow sim then naturally pulls water from neighbors to
equalize — this is the correct behavior (water rushes in to fill the gap). Waterfall particles
handle the visual of water dropping to the new floor.

---

## Testing strategy

### Unit tests (src/water/grid.rs)

Core simulation correctness, no rendering or ECS:

- **Volume conservation**: create a grid, add volume to one cell, step N times. Assert total
  volume across all cells remains constant (within floating point tolerance).
- **Flow direction**: two adjacent cells with different surface levels. After one step, the
  higher cell has less volume and the lower cell has more.
- **Equilibrium**: flat terrain, uniform water. After enough steps, all cells have equal
  surface levels. `settled` flag is true.
- **Boundary source**: one boundary cell with `ocean_level` above terrain. After steps, water
  flows inward. Total volume increases (ocean is an infinite source).
- **Boundary sink**: inland water above `ocean_level` near the boundary. After steps, water
  drains to ocean level.
- **Floor collapse**: cell with water, drop `floor_level`. Verify surface_level changes and
  flow sim redistributes.
- **Settled detection**: create an equilibrium state, verify `settled` is true. Add volume to
  one cell, verify `settled` becomes false. Let it re-equalize, verify `settled` is true again.
- **Substep stability**: step with a very large dt (e.g. 1.0s). Verify no negative volumes,
  no NaN, no explosion.

### Wave equation tests (src/water/wave_grid.rs)

- **Ripple propagation**: create a wave grid backed by a wet flow grid. Inject displacement at
  center wave cell. After N steps, verify displacement has spread to neighboring cells and the
  center has rebounded.
- **Energy decay**: inject displacement, step many times. Verify total wave energy
  (`sum(velocity² + c² * displacement²)`) decreases monotonically (damping works).
- **Bank reflection**: inject displacement near the edge of a pond (wet/dry boundary). Verify
  displacement reflects back (non-zero displacement returns to the source region after the
  expected round-trip time `2 * distance / wave_speed`).
- **CFL stability**: set wave_speed high relative to wave_cell_size/dt. Verify the substep
  guard keeps the simulation stable (no explosion, no NaN).
- **Dry cell isolation**: inject displacement at a wet wave cell adjacent to a dry wave cell.
  Verify no displacement leaks into dry cells.
- **Pond merge**: two separate wet regions with a dry gap between them. Inject displacement in
  one region. Make the gap wet (simulating terrain destruction). Verify ripples propagate
  through the newly-wet cells into the second region.
- **Pond split**: one wet region, inject displacement. Make a strip of cells dry (simulating
  terrain creation). Verify waves reflect off the new dry boundary and do not leak through.

### Bench harness scenario (src/physics/bench_harness/)

A visual scenario for the bench viewer, following the existing `PhysicsBenchScenario` pattern:

- **"pond_drain" scenario**: flat terrain with a 10x10 pond. At step 100, destroy a 3-voxel
  sphere in the bank. Visual: water flows through the gap, drains, particles at the leading
  edge. Assertions: water volume decreases over time, no cells go negative.
- **"buoyancy_bob" scenario**: a sphere dropped into a deep pool. Visual: sphere sinks, bobs,
  settles at equilibrium depth. Assertions: sphere's final Y position matches the analytical
  equilibrium (where buoyancy = gravity), settling time is reasonable (< 5 seconds).
- **"ocean_flood" scenario**: island terrain above ocean level. Destroy the coastline bank.
  Visual: ocean floods inward. Assertions: water level inside the breach approaches ocean_level
  over time.

These scenarios serve as both regression tests and visual debugging tools. The bench viewer
lets the implementer see exactly what the water is doing frame by frame (pause, step, reset).

### Integration tests (manual, visual)

Not automated, but a checklist for the implementer to verify:

- [ ] Place a pond in a level, walk into it — player gets buoyancy forces, camera goes
      underwater (or at least the floor_level check works correctly).
- [ ] Drop a beach ball into the pond — it floats, bobs, settles.
- [ ] Drop a heavy crate into the pond — it sinks.
- [ ] Grenade the bank of the pond — water flows out through the gap.
- [ ] Stand under a sky island pond — no underwater effects.
- [ ] Ocean level set — water visible at world edges, terrain occludes it correctly.
- [ ] No water configured — game runs identically to before (no crashes, no phantom water).

---

## Implementation order

Each step leaves the engine functional. Later steps build on earlier ones.

### Step 1: Terrain interface + flow grid + level format
- Add `approx_surface_height_at()` and `surface_heights_at()` to `TerrainManager`.
- Expose `dirty_regions()` on `TerrainManager`.
- Remove `VoxelMaterial::Water`.
- Add `WaterConfig`, `WaterBody` to `src/level/data.rs`.
- Implement `FlowGrid`, `FlowCell`, and flow equalization.
- Unit tests: volume conservation, flow direction, boundary cells, settled detection.
- No rendering yet — debug via `DebugOverlays` (colored spheres at water surface levels).

Current implementation notes:
- `FlowGrid` (currently named `WaterGrid`) and unit tests are implemented in `src/water/grid.rs`.
- Ocean coupling is optional in code. Use `ocean_level: None` for inland or sky-island water,
  not a sentinel numeric sea level.
- Boundary ocean coupling currently requires `floor_level <= ocean_level`.
- The active-cell optimization from this plan has not been implemented yet; the current code
  iterates the full grid and relies on `settled` to skip steady-state work.

### Step 1.5: Active-cell optimization (optional)
- Add `active_cells` and a duplicate-suppression structure such as `active_flags`.
- Iterate only wet cells plus their neighbors during flow updates.
- Wake local cells when terrain damage or ocean coupling changes a region.
- Keep the same tests and behavior; this step is purely a performance optimization.

### Step 2: Water mesh rendering
- Implement `WaterRenderer` with a basic water shader (flat color, alpha blend).
- Generate grid mesh from `FlowGrid` each frame (all-wet-corners-only for simplicity).
- Integrate into the render pipeline (after terrain, before particles).
- Note: this initial mesh is at flow grid resolution (~1-2m cells). Step 3 upgrades it to
  wave grid resolution.

### Step 3: Wave equation surface
- Implement `WaveGrid` and `WaveCell` in `src/water/wave_grid.rs`.
- Wire wet/dry status from `FlowGrid` to `WaveGrid` (wave cell is wet if parent flow cell has volume > 0).
- Implement the discrete 2D wave equation step (Laplacian, damping, CFL substep guard).
- Upgrade the water mesh to render from the wave grid instead of the flow grid. The mesh is
  now at wave resolution (~10cm cells), with per-vertex y = `bulk_level + displacement`.
- Test: inject a displacement at one wave cell, verify circular ripples propagate and reflect
  off dry boundaries. Verify CFL stability under large dt.
- Add `wave_speed` and `wave_damping` to `WaterConfig`.

### Step 4: Buoyancy and rigid body interaction
- Implement buoyancy force calculation in `src/water/buoyancy.rs`.
- Buoyancy samples `bulk_level + wave_displacement` so bodies ride the waves.
- Wire into ECS via `BuoyancySystem`.
- Test with spheres and boxes in a pond.
- Add "buoyancy_bob" bench scenario.

### Step 5: Wave-body coupling
- Body impact: inject negative `wave_velocity` at body footprint cells on water entry.
- Body bobbing: continuously perturb surface at floating body cells.
- Player wake: inject displacement proportional to player movement speed.
- Test: drop a beach ball, verify ripple rings. Verify two floating objects create
  interference patterns.

### Step 6: Terrain destruction interaction
- Floor collapse detection via `dirty_regions()` and `surface_heights_at()`.
- Waterfall particles for vertical water flow.
- Ocean flooding through destroyed coastline.
- Inject wave disturbance at the leading edge of flowing water.
- Add "pond_drain" and "ocean_flood" bench scenarios.

### Step 7: Splash and spray particles
- Impact detection (body enters water surface).
- Radial splash particle emission.
- Leading-edge spray for fast-flowing cells.

### Step 8: Ocean plane (future)
- Add the infinite ocean quad with Gerstner wave shader (non-interactive, cosmetic).
- Absorbing boundary conditions at the ocean-simulation seam so wave equation ripples
  pass through into the ocean without reflecting.
- Upgrade fragment shader: Fresnel, specular, depth tint.

---

## Open questions for an experienced reviewer

These are design-level decisions that should be validated before significant implementation
work begins:

1. **Flow algorithm choice.** The pressure equalization approach is simple and stable but
   produces uniform spreading without momentum. The wave equation handles visible sloshing
   and ripples, but bulk flow (pond draining through a gap) still spreads uniformly. A
   pipe-based flow model (track velocity between cell pairs) would give more realistic bulk
   drainage with wave fronts and momentum. This is a potential upgrade if the pressure
   equalization looks too artificial for drainage scenarios.

2. **Water volume displacement by rigid bodies.** This design treats buoyancy as a force on the
   body but does not displace water volume when a body enters the water. A large box dropped
   into a small pond should raise the water level. Implementing this means subtracting the
   body's submerged volume from the water cell's volume each frame and redistributing it.
   Straightforward but creates a coupling between the physics system and the water grid that
   this design currently avoids.

3. **Player interaction.** The player is a dynamic velocity-driven body, so buoyancy and drag
   forces apply directly — no special-casing needed in the water system. The remaining
   questions are gameplay-layer: movement input when submerged (swim speed, reduced jump),
   detecting "player is underwater" for camera/audio effects, and whether the player can dive
   or just bobs at the surface. These need the water system to expose a query like
   `WaterGrid::sample_at(x, z) -> Option<(floor_level, surface_level)>` but are otherwise
   independent of the water simulation design.

4. **Rendering approach for alpha blending.** Transparent water rendered with standard alpha
   blending has order-dependent artifacts (water behind terrain rendered after terrain looks
   wrong). The depth-buffer discard approach handles water-behind-terrain but not
   water-behind-water (overlapping transparent layers). For a single water surface this is
   fine, but worth noting the limitation. OIT (order-independent transparency) is the full
   solution but is a significant rendering undertaking.

5. **Water grid resolution vs visual quality.** A 2:1 ratio (one water cell per 2x2 voxels)
   means the water surface has half the spatial resolution of the terrain. At shorelines, this
   produces visible staircase artifacts. A 1:1 ratio doubles the cell count and simulation
   cost. The right trade-off depends on the terrain scale and how close the camera gets to
   shorelines.
