# Level File Format Design

## Motivation

The engine currently defines levels procedurally in Rust code (`App::new()`, `TerrainGenerator::generate_simple_hills`). This makes iteration slow (recompile per change) and collaborative design impractical. A declarative level file format decouples level content from engine code, enabling:

- **Human-AI collaboration**: both parties edit the same text file.
- **Fast iteration**: hot-reload on file save (Tier 2, see Roadmap).
- **Version control**: levels are diffable, mergeable text.
- **Separation of concerns**: engine code handles *how* to render/simulate; level files describe *what* to render/simulate.

## Level Ideas

A catalogue of level concepts that play to the engine's strengths: destructible voxel terrain, physics boxes/planks, beach balls, and grenade throwing.

### Destruction-Based Puzzles

- **Blow Your Own Path**: Solid terrain walls blocking progress — figure out which spots to grenade to create footholds or tunnels. Limited grenade supply forces planning.
- **Structural Collapse**: A terrain bridge supports a platform you need to reach. Grenade the right support pillar so it tips toward you rather than falling into the void.
- **Buried Treasure**: Hidden passages behind destructible walls of varying depth. Soft voxels hint at the right path; wasting grenades on hard rock leaves you stuck.

### Physics Playground

- **Box Staircase**: Stack crates to reach high ledges. Different densities matter — heavy crates as a stable base, light ones on top.
- **Beach Ball Bowling**: Knock a beach ball into a tower of crates to clear a path. Or use grenades to launch beach balls at targets.
- **Teeter-Totter Launcher**: A plank balanced on a fulcrum. Stack heavy boxes on one end, then jump on it (or grenade it) to launch yourself or objects upward.

### Platforming Challenges

- **Crumbling Run**: Terrain that's already damaged — each footstep or grenade weakens it further. Sprint across before it collapses beneath you.
- **Crate Surfing**: Ride a floating crate across a gap by carefully balancing on it. Grenade the terrain dam holding back a "river" of beach balls to push your crate-raft forward.
- **The Gauntlet**: Narrow terrain bridges with swinging plank pendulums that knock you off. Time your crossings.

### Combined Mechanics

- **Rube Goldberg Gates**: Grenade a terrain support → crates fall onto a seesaw → beach ball launches onto a pressure plate → gate opens.
- **Demolition Golf**: Launch a beach ball across a course using grenades as "clubs." Fewest shots wins. Terrain hazards and crate obstacles in the way.
- **Tower Defense in Reverse**: A tall tower of crates guards the exit. Find the right angle/sequence of grenades to topple it without burying the door.

### Risk/Reward

- **Grenade Jumping**: Grenade the ground beneath you for a super-jump, but you destroy your landing zone. Commit or die.
- **Shortcut vs. Safety**: The safe path winds around. The fast path requires blowing through terrain — but over-destroy and you fall through.

## Format Choice: RON

[RON (Rusty Object Notation)](https://github.com/ron-rs/ron) is the natural fit:

- Direct `serde::Deserialize` into Rust structs — zero manual parsing.
- Supports enums, tuples, optional fields — maps cleanly to our type system.
- Comments for annotation (unlike JSON).
- Already idiomatic in the Rust gamedev ecosystem (Bevy, Amethyst).

**Dependency**: `ron = "0.8"` + `serde = { features = ["derive"] }`.

## Level File Structure

```ron
// levels/test_arena.level.ron
Level(
    name: "Test Arena",
    world_size: 64.0,        // SVO cube half-size
    voxel_size: 1.0,         // loader computes octree_depth = log2(world_size / voxel_size)

    terrain: Terrain(
        base_height: 0.0,
        material_layers: [
            (depth: 1.0,  material: Grass),
            (depth: 4.0,  material: Dirt),
            (depth: 999.0, material: Rock),
        ],

        // Heightfield features — modify surface height at each (x, z) column
        features: [
            Hill(center: (10.0, 15.0), radius: 8.0, height: 5.0),
            Hill(center: (-8.0, -5.0), radius: 12.0, height: 3.0),
            Crater(center: (0.0, 0.0), radius: 6.0, depth: 3.0),
            Plateau(min: (-20.0, -20.0), max: (-10.0, -10.0), height: 8.0),
            Wall(from: (5.0, -10.0), to: (5.0, 10.0), height: 6.0, thickness: 2.0),
            Ramp(from: (10.0, 5.0), to: (18.0, 5.0), start_height: 0.0, end_height: 8.0, width: 4.0),
            TerrainRoughness(frequency: 0.1, amplitude: 0.5, octaves: 3, seed: 42),
        ],

        // Volumetric features — place or carve voxels in 3D (evaluated after heightfield)
        volumes: [
            Island(center: (20.0, 15.0, 0.0), half_extents: (6.0, 2.0, 6.0), edge_noise: 0.3),
            Arch(from: (10.0, 8.0, 0.0), to: (20.0, 8.0, 0.0), radius: 3.0, thickness: 1.5),
            Pillar(center: (5.0, 0.0), height: 12.0, radius: 2.0),
            Tunnel(center: (0.0, -15.0), direction: (1.0, 0.0), length: 12.0, radius: 2.5, depth: 2.0),
        ],
    ),

    player_spawn: (0.0, 3.0, -15.0),

    objects: [
        // Beach balls
        BeachBall(pos: (3.0, 5.0, 2.0)),
        BeachBall(pos: (-2.0, 5.0, 4.0)),

        // Raw box with full control over appearance and physics
        Box(
            pos: (5.0, 1.0, 0.0),
            half_extents: (0.5, 0.5, 0.5),
            style: WoodenCrate,
            density: 50.0,
            restitution: 0.2,
            friction: 0.6,
        ),

        // Named types — bundled defaults for appearance + physics
        Plank(pos: (8.0, 4.0, 0.0), length: 4.0, width: 1.0),
        Crate(pos: (10.0, 1.0, 3.0), size: 1.0),
        HeavyCrate(pos: (12.0, 1.0, 3.0), size: 1.0),

        // Vertical stack — auto-computes Y positions bottom-up from base
        Stack(
            base: (12.0, 0.0, 5.0),
            items: [
                Crate(size: 2.0),
                Crate(size: 2.0),
                Crate(size: 1.0),
            ],
        ),

        // Tower — shorthand for N identical boxes stacked
        Tower(
            base: (-5.0, 0.0, 8.0),
            box_half_extents: (0.5, 0.5, 0.5),
            count: 6,
            density: 40.0,
        ),

        // Wall of boxes — grid arrangement
        BoxWall(
            base: (15.0, 0.0, 0.0),
            box_half_extents: (1.0, 0.5, 0.5),
            columns: 4,
            rows: 3,
            density: 50.0,
            // Alternating row offset for brick-like pattern
            stagger: true,
        ),

        // Prefab structures
        House(
            pos: (0.0, 10.0, 5.0),
            half_extents: (2.0, 1.5, 2.0),
        ),
    ],
)
```

## Data Model (Rust structs)

```
src/level/
├── mod.rs              // re-exports
├── data.rs             // serde structs (pure data, no engine deps)
├── loader.rs           // RON parsing, validation
└── spawner.rs          // data → ECS entities + terrain
```

### `data.rs` — Pure Data Structs

```rust
use serde::Deserialize;

#[derive(Deserialize)]
pub struct Level {
    pub name: String,
    pub world_size: f32,
    pub voxel_size: f32,         // octree depth computed: log2(world_size / voxel_size)
    pub terrain: Terrain,
    pub player_spawn: (f32, f32, f32),
    pub objects: Vec<LevelObject>,
}

#[derive(Deserialize)]
pub struct Terrain {
    pub base_height: f32,
    #[serde(default)]
    pub material_layers: Vec<MaterialLayer>,
    /// Heightfield features — modify surface height at each (x, z) column.
    pub features: Vec<TerrainFeature>,
    /// Volumetric features — place or carve voxels in 3D (evaluated after heightfield).
    #[serde(default)]
    pub volumes: Vec<VolumeFeature>,
}

#[derive(Deserialize)]
pub struct MaterialLayer {
    pub depth: f32,
    pub material: VoxelMaterialId,
}

#[derive(Deserialize)]
pub enum VoxelMaterialId { Grass, Dirt, Rock }

/// Heightfield features operate on 2D (xz) coordinates, modifying the terrain
/// surface height additively.
#[derive(Deserialize)]
pub enum TerrainFeature {
    Hill { center: (f32, f32), radius: f32, height: f32 },
    Crater { center: (f32, f32), radius: f32, depth: f32 },
    Plateau { min: (f32, f32), max: (f32, f32), height: f32 },
    Wall { from: (f32, f32), to: (f32, f32), height: f32, thickness: f32 },
    Ramp { from: (f32, f32), to: (f32, f32), start_height: f32, end_height: f32, width: f32 },
    TerrainRoughness { frequency: f32, amplitude: f32, octaves: u32, seed: u32 },
}

/// Volumetric features operate in 3D, directly placing or carving voxels.
/// Applied as a second pass after the heightfield, enabling floating islands,
/// arches, and tunnels that a pure heightfield cannot express.
#[derive(Deserialize)]
pub enum VolumeFeature {
    /// Floating solid mass with optional noisy edges for organic look.
    Island { center: (f32, f32, f32), half_extents: (f32, f32, f32), edge_noise: f32 },
    /// Curved bridge between two 3D points.
    Arch { from: (f32, f32, f32), to: (f32, f32, f32), radius: f32, thickness: f32 },
    /// Vertical column rising from the heightfield surface.
    Pillar { center: (f32, f32), height: f32, radius: f32 },
    /// Horizontal bore that carves through existing terrain.
    Tunnel { center: (f32, f32), direction: (f32, f32), length: f32, radius: f32, depth: f32 },
}

/// Visual style for box textures.
#[derive(Deserialize)]
pub enum BoxStyle {
    WoodenCrate, Cardboard, Metal, Gift, Stone, Brick, Warning, Random,
}

#[derive(Deserialize)]
pub enum LevelObject {
    BeachBall {
        pos: (f32, f32, f32),
    },
    /// Raw box with full control over dimensions, appearance, and physics.
    Box {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
        #[serde(default = "default_box_style")]
        style: BoxStyle,
        #[serde(default = "default_density")]
        density: f32,
        #[serde(default = "default_box_restitution")]
        restitution: f32,
        #[serde(default = "default_box_friction")]
        friction: f32,
    },
    /// Thin wooden board. Default: style=WoodenCrate, density=20, thickness=0.1.
    Plank {
        pos: (f32, f32, f32),
        length: f32,
        width: f32,
    },
    /// Standard crate. Default: style=WoodenCrate, density=50.
    Crate {
        pos: (f32, f32, f32),
        size: f32,             // cube half-extent
    },
    /// Dense crate. Default: style=Metal, density=150.
    HeavyCrate {
        pos: (f32, f32, f32),
        size: f32,
    },
    Stack {
        base: (f32, f32, f32),
        items: Vec<StackItem>,
    },
    Tower {
        base: (f32, f32, f32),
        box_half_extents: (f32, f32, f32),
        count: u32,
        #[serde(default = "default_density")]
        density: f32,
    },
    BoxWall {
        base: (f32, f32, f32),
        box_half_extents: (f32, f32, f32),
        columns: u32,
        rows: u32,
        #[serde(default = "default_density")]
        density: f32,
        #[serde(default)]
        stagger: bool,
    },
    House {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
    },
}

#[derive(Deserialize)]
pub enum StackItem {
    Crate { size: f32 },
    HeavyCrate { size: f32 },
    Plank { length: f32, width: f32 },
    BeachBall,
}

#[derive(Deserialize)]
pub enum Goal {
    ReachPoint { pos: (f32, f32, f32), radius: f32 },
}
```

### `loader.rs` — Parse + Validate

```rust
pub fn load_level(path: &Path) -> Result<Level, LevelError> {
    let contents = std::fs::read_to_string(path)?;
    let level: Level = ron::from_str(&contents)?;
    validate(&level)?;
    Ok(level)
}

fn validate(level: &Level) -> Result<(), LevelError> {
    // world_size > 0, voxel_size > 0, voxel_size is power-of-two divisor of world_size,
    // player_spawn inside bounds, etc.
}
```

### `spawner.rs` — Data → World

This is the bridge between level data and the existing spawner functions. It replaces the hardcoded setup in `App::new()`.

```rust
pub fn spawn_level(world: &mut World, level: &Level, materials: &LevelMaterials) {
    spawn_terrain(world, level);
    spawn_player(world, level.player_spawn.into());
    for obj in &level.objects {
        spawn_object(world, obj, materials);
    }
}
```

Terrain generation refactors `TerrainGenerator` to accept the `Terrain` struct instead of hardcoded sine waves. Generation runs in two passes:

**Pass 1 — Heightfield:** For each (x, z) column, compute the surface height by accumulating all `TerrainFeature` contributions:

```rust
fn height_at(x: f32, z: f32, terrain: &Terrain) -> f32 {
    let mut h = terrain.base_height;
    for feature in &terrain.features {
        h += feature.contribute(x, z);
    }
    h
}
```

Each feature's `contribute()` returns an additive height delta using smooth falloff functions (e.g. cosine blend for hills, linear interpolation for ramps). The column is then filled with solid voxels from the SVO floor up to the computed height.

**Pass 2 — Volumes:** After the heightfield is filled, iterate over each `VolumeFeature` and directly set or clear voxels in 3D. For example, `Island` iterates over all voxels within its bounding box, setting those inside its shape to solid. `Tunnel` does the inverse — clearing voxels within its cylindrical volume.

## Terrain Feature Semantics

Terrain generation runs in two passes:

### Pass 1: Heightfield Features

Each feature modifies the height at (x, z) additively. Features are evaluated in order.

| Feature | Effect | Parameters |
|---------|--------|------------|
| **Hill** | Smooth dome, cosine falloff from center | center (xz), radius, height |
| **Crater** | Inverted dome, subtracts height | center (xz), radius, depth |
| **Plateau** | Flat rectangular region raised to fixed height | min/max (xz), height |
| **Wall** | Thin raised strip along a line segment | from/to (xz), height, thickness |
| **Ramp** | Linear height gradient between two points | from/to (xz), start/end height, width |
| **TerrainRoughness** | FBM noise layer for natural surface variation | frequency, amplitude, octaves, seed |

### Pass 2: Volumetric Features

After the heightfield is filled, volumetric features directly set or clear voxels in 3D space. This enables geometry that a pure heightfield cannot express.

| Feature | Effect | Parameters |
|---------|--------|------------|
| **Island** | Floating solid mass (rounded box with optional noisy edges) | center (xyz), half_extents, edge_noise |
| **Arch** | Curved bridge between two 3D points | from/to (xyz), radius, thickness |
| **Pillar** | Vertical column rising from the heightfield surface | center (xz), height, radius |
| **Tunnel** | Horizontal bore that carves through existing terrain | center (xz), direction, length, radius, depth |

**Island** is the key primitive for platformer levels — it places a disconnected chunk of terrain at any position in space. The `edge_noise` parameter controls how rough the edges are (0.0 = smooth box, 1.0 = very craggy). Multiple islands at different heights create the classic floating-island platformer layout.

## Object Default Values

### Named Types

Named types bundle appearance and physics to encode gameplay intent:

| Type | Style | Density | Dimensions | Notes |
|------|-------|---------|------------|-------|
| **Crate** | WoodenCrate | 50.0 | `size` cube | General purpose |
| **HeavyCrate** | Metal | 150.0 | `size` cube | Hard to push, stable base |
| **Plank** | WoodenCrate | 20.0 | `length` x 0.1 x `width` | Bridges, seesaws, shelves |

### Raw Box Defaults

When using `Box` directly, these defaults apply for omitted fields:

| Property | Default | Notes |
|----------|---------|-------|
| `style` | Random | Randomly picked from 7 texture styles |
| `density` | 50.0 | Current `spawn_box` value |
| `restitution` | 0.2 | Current `spawn_box` value |
| `friction` | 0.6 | Current `spawn_box` value |
| `stagger` | false | BoxWall brick pattern |

BeachBall uses its own fixed physics params (density=1.0, restitution=0.6) since those are fundamental to being a beach ball.

## Integration with App::new()

The change to `App::new()` is minimal. Replace the hardcoded spawn block (lines 66–99) with:

```rust
let level = load_level(Path::new("levels/test_arena.level.ron"))?;
spawn_level(&mut world, &level, &materials);
```

Terrain creation moves into `spawn_level` since the level file now specifies world size and voxel size (octree depth is computed).

## Roadmap

### Tier 1: File-driven levels (this design)
- `src/level/` module with data, loader, spawner.
- Refactor `TerrainGenerator` to accept feature list.
- Move `App::new()` entity spawning to `spawn_level()`.
- Ship one or two example `.level.ron` files.

### Tier 2: In-game placement helper
- Fly-camera mode (F5 toggle).
- Place objects at crosshair with keyboard shortcuts (B=box, O=ball).
- Nudge selected object with arrow keys.
- Press F6 to dump current scene to `.level.ron` on stdout.
- This gives spatial intuition that text editing alone cannot.

### Tier 3: Hot reload
- `notify` crate watches the level file.
- On change: despawn all level entities, reload file, respawn.
- Terrain rebuild uses existing dirty-region system for efficiency.
- Edit-save-see loop in ~1 second.

## Implementation Plan

### Step 1: Add dependencies and create `src/level/data.rs`

Add `ron = "0.8"` and `serde = { version = "1", features = ["derive"] }` to `Cargo.toml`.

Create `src/level/` module. Start with `data.rs` containing all the pure serde structs: `Level`, `Terrain`, `TerrainFeature`, `VolumeFeature`, `LevelObject`, `StackItem`, `BoxStyle`, `Goal`, `MaterialLayer`, `VoxelMaterialId`. These have no engine dependencies — just `serde::Deserialize` derives and default value functions.

Create `src/level/mod.rs` with re-exports.

**Verify**: `cargo build` compiles (structs are unused but valid).

### Step 2: Create `src/level/loader.rs`

Implement `load_level(path: &Path) -> Result<Level, LevelError>` that reads a RON file and deserializes into `Level`. Add a `LevelError` enum wrapping `std::io::Error` and `ron::de::SpannedError`. Implement `validate()` checking:
- `world_size > 0`, `voxel_size > 0`
- `world_size / voxel_size` is a power of two (valid octree)
- `player_spawn` is within world bounds

Also add the helper `Level::octree_depth(&self) -> u32` that computes `(world_size / voxel_size).log2() as u32`.

**Verify**: Write a unit test that parses a hardcoded RON string into a `Level` struct.

### Step 3: Refactor `spawn_box` to accept physics parameters

Currently `spawn_box` hardcodes density=50.5, restitution=0.2, friction=0.6. Add a `BoxPhysics` struct (or extend the signature) so the caller can pass these values. The existing call sites in `App::new()` pass the current defaults so behaviour is unchanged.

This unblocks the level spawner from controlling per-box physics.

**Verify**: `cargo build`, `cargo run` — existing boxes behave identically.

### Step 4: Add `BoxStyle` to box material creation

Currently `create_box_materials` generates random styles via `generate_box_pixels()` which picks randomly from 7 styles. Refactor so each style generator (`generate_wooden_crate`, `generate_cardboard_box`, etc.) can be called by name via a `BoxStyle` enum. Add a public function:

```rust
pub fn create_box_material_for_style(
    style: BoxStyle,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<MaterialId>
```

`BoxStyle::Random` delegates to the existing random picker. Named styles call their specific generator. The existing `create_box_materials` can be reimplemented in terms of this.

**Verify**: `cargo build`, `cargo run` — existing boxes still work.

### Step 5: Refactor `TerrainGenerator` for feature-driven heightfield (Pass 1)

Replace the hardcoded sine/crater logic in `generate_simple_hills` with a new method:

```rust
pub fn generate_from_features(
    svo: &mut SparseVoxelOctree,
    terrain: &Terrain,
    durability: &DurabilityConfig,
)
```

The column-fill loop stays the same, but `height` is now computed by `height_at(x, z, terrain)` which iterates over `terrain.features`. Implement `contribute()` for each `TerrainFeature` variant:

- **Hill**: `height * (1 + cos(pi * dist / radius)) / 2` for smooth falloff, zero outside radius.
- **Crater**: same shape but negative.
- **Plateau**: if (x, z) inside rect, return `plateau_height - current_height` (sets absolute height).
- **Wall**: distance from point to line segment; if within `thickness/2`, return `height`.
- **Ramp**: project onto from→to segment, lerp between `start_height` and `end_height`, falloff outside `width/2`.
- **TerrainRoughness**: FBM noise using the existing `fbm_2d_periodic` utility.

Keep `generate_simple_hills` working by converting its current logic to an equivalent feature list internally, or replace `create_test_terrain` to build a `Terrain` struct. This ensures existing behaviour is preserved until level files are wired in.

**Verify**: `cargo run` — terrain looks the same as before (or intentionally improved with the same shapes expressed as features).

### Step 6: Add volumetric terrain features (Pass 2)

After the heightfield pass, iterate `terrain.volumes` and apply each:

- **Island**: for each voxel in the bounding box of `center ± half_extents`, compute signed distance to the rounded box. If inside (accounting for `edge_noise` via FBM displacement of the surface), set to solid with material based on depth from island surface.
- **Pillar**: for each voxel in a vertical cylinder from the heightfield surface up to `height`, set to solid. Use `radius` as the cylinder radius.
- **Tunnel**: for each voxel in the cylindrical volume, set to air. Direction + length define the cylinder axis; `depth` is the Y coordinate of the bore center.
- **Arch**: compute a curved path (circular arc or catenary) between `from` and `to`; for each voxel near the path within `thickness`, set to solid.

Island and Pillar are the priority — they cover most platformer layouts. Tunnel and Arch can be stubbed with `todo!()` and implemented later.

**Verify**: Add a test island to the hardcoded terrain, confirm it appears as a floating solid mass.

### Step 7: Create `src/level/spawner.rs`

Implement `spawn_level()` which:
1. Computes octree depth from `level.voxel_size` and `level.world_size`.
2. Creates the SVO and calls `generate_from_features()`.
3. Builds `TerrainManager` from the SVO.
4. Spawns the player at `level.player_spawn`.
5. Iterates `level.objects` and dispatches to the appropriate spawner:
   - `BeachBall` → `spawn_beach_ball()`
   - `Box` → `spawn_box()` with the specified style, half_extents, and physics params
   - `Crate` / `HeavyCrate` / `Plank` → `spawn_box()` with their bundled defaults
   - `Stack` → compute cumulative Y from base, spawn each item
   - `Tower` → spawn N boxes at stacked Y positions
   - `BoxWall` → nested row/column loop, with optional half-box stagger per row
   - `House` → `spawn_house()`

Material creation for boxes: for each object that needs a box material, call `create_box_material_for_style()` from Step 4. Materials can be created on-the-fly during spawning, or pre-scanned from the level file and batch-created.

**Verify**: `cargo build` — spawner compiles against existing spawner functions.

### Step 8: Wire into `App::new()` and create first level file

Create `levels/test_arena.level.ron` that reproduces the current hardcoded scene (5 beach balls, 5 stacked boxes, house, same terrain shape expressed as features).

Modify `App::new()`:
- Accept a level file path (or default to `levels/test_arena.level.ron`).
- Replace the hardcoded spawn block (lines 66–99) and `create_terrain()` call with `load_level()` + `spawn_level()`.
- Keep material setup, rendering context, and force field registration as-is.

**Verify**: `cargo run` loads the level file and produces the same scene as before. Delete `generate_simple_hills` and `create_test_terrain` since they are now unused.

### Step 9: Create the Grenade Gauntlet level

Write `levels/grenade_gauntlet.level.ron` using the example from this doc. Test it by changing the level path in `App::new()` (or adding a command-line argument). Iterate on the layout — this is the first real test of the format's expressiveness.

**Verify**: playable level with platforms, gaps, crate walls, floating island, and a goal point.

## Example: Obstacle Course Level

To illustrate how this format makes level design practical, here's a sketch of the "grenade jumping" concept from our brainstorm:

```ron
Level(
    name: "Grenade Gauntlet",
    world_size: 64.0,
    voxel_size: 1.0,
    terrain: Terrain(
        base_height: -5.0,
        features: [
            // Starting platform
            Plateau(min: (-8.0, -8.0), max: (8.0, 8.0), height: 0.0),

            // First gap — narrow bridge, destructible
            Wall(from: (8.0, 0.0), to: (18.0, 0.0), height: 0.0, thickness: 1.5),

            // Mid platform with crate puzzle
            Plateau(min: (18.0, -6.0), max: (28.0, 6.0), height: 0.0),

            // Ramp up to high ground
            Ramp(from: (28.0, 0.0), to: (36.0, 0.0), start_height: 0.0, end_height: 8.0, width: 4.0),

            // High ground with destructible floor
            Plateau(min: (36.0, -5.0), max: (50.0, 5.0), height: 8.0),

            // Gentle roughness for visual interest
            TerrainRoughness(frequency: 0.15, amplitude: 0.5, octaves: 2, seed: 7),
        ],

        // Floating island shortcut — risky but faster
        volumes: [
            Island(center: (30.0, 12.0, 8.0), half_extents: (3.0, 1.0, 3.0), edge_noise: 0.4),
        ],
    ),
    player_spawn: (0.0, 2.0, 0.0),
    objects: [
        // Crate wall blocking the ramp — knock it down
        BoxWall(
            base: (28.0, 0.0, 0.0),
            box_half_extents: (0.5, 0.5, 0.5),
            columns: 4,
            rows: 5,
            density: 30.0,
            stagger: true,
        ),

        // Beach balls on the narrow bridge — hazard
        BeachBall(pos: (11.0, 1.0, 0.0)),
        BeachBall(pos: (14.0, 1.0, 0.0)),

        // Heavy crate to push off the edge for a stepping stone
        HeavyCrate(pos: (22.0, 1.0, 0.0), size: 1.0),

        // Plank bridge to the floating island — risky shortcut
        Plank(pos: (26.0, 10.0, 4.0), length: 5.0, width: 1.0),

        // Tower at the end — topple it to reach the goal
        Tower(
            base: (45.0, 8.0, 0.0),
            box_half_extents: (0.6, 0.4, 0.6),
            count: 8,
            density: 25.0,
        ),
    ],
)
```
