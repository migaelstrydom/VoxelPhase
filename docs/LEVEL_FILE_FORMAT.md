# Level File Format Design

> **Partly superseded.** `docs/LEVEL_SEGMENTS_PLAN.md` replaces the single-cube world model.
> As of stage 2 a level is a list of **segments** joined by an **anchor** graph — see
> "Segments, anchors and placement" below, which is the authoritative description of the
> top level of the format. Everything about **object and spawnable syntax** still applies,
> now scoped inside a segment; the old world-sizing and octree-depth sections are gone.

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

## Segments, anchors and placement

A level is **N segments**. Each segment owns a coordinate frame, its own voxel resolution,
its terrain, its named anchors and its objects — and every coordinate inside it is
**segment-local**. Nothing but the placement root is authored in world coordinates.

### Placement is a tree; connectivity is a graph

- **`placements`** derive transforms. Exactly one `Root`, then one `Join` per remaining
  segment. A segment with two placements is over-determined, one with none is an orphan,
  and a loop is a cycle — all three are errors at load.
- **`connections`** derive nothing. They *assert* that two anchors meet, and `level_check`
  verifies it. Any relationship that would close a loop — a shortcut back to an earlier
  area, a second bridge across a chasm — belongs here, because two placement paths around
  a loop would disagree about where a segment is.

### The anchor facing convention

**An anchor's local `+X` points outward, out of the segment.**

```text
       segment "plaza"                    segment "tower"
  ┌───────────────────────┐   gap   ┌───────────────────────┐
  │                       │◀──────▶ │                       │
  │              exit_east│         │entry                  │
  │                    ●──┼──▶ +X   │  +X ◀──●              │
  │                       │         │                       │
  └───────────────────────┘         └───────────────────────┘
```

Mating two anchors is a **180° relative yaw**: the child's anchor is turned to face back
at the parent's, and the two origins are separated by `gap` metres along the parent's
outward direction. So:

| Face of the segment | Outward direction | Anchor `yaw` |
|---------------------|-------------------|--------------|
| `+x` (east)         | `+X`              | `0`          |
| `-z` (north)        | `−Z`              | `90`         |
| `-x` (west)         | `−X`              | `180`        |
| `+z` (south)        | `+Z`              | `270`        |

A quarter turn maps local `+X` onto world `−Z`, which is why the north face is 90 and not
270. **If a segment lands inside its neighbour instead of beside it, its anchor is facing
inward** — that is the mistake everyone makes first.

### Rotation is quarter turns only

Every `yaw` in the format — segment and anchor alike — must be a multiple of 90°. Anything
else is rejected at load with the offending segment or anchor named. Pitch and roll are not
supported; slanted geometry is expressed *within* a segment via terrain features.

### Welded joins are not implemented

`Join` accepts `weld: true`, and **rejects it at load**. Marching cubes samples a one-voxel
halo and an unallocated neighbour reads as air, so a welded boundary chunk emits a cap
surface sealing a join that should be open — real geometry that reaches physics. Use a gap,
or express the continuous structure as a spawnable. Cross-segment halo sampling is separate
future work.

### What is world-space and what is not

| Authored in | Field |
|-------------|-------|
| World       | `Root(origin:, yaw:)` — the one absolute transform in the file |
| Root-local  | `player_spawn` |
| Segment-local | `terrain.bounds`, all terrain features, `anchors[].pos`, all `objects` |

`load_level` resolves the tree and rewrites objects and the spawn into world coordinates.
Terrain extents stay local, because generation is local — that is what makes a segment
produce identical geometry wherever it is placed.

Placing a segment at a non-zero yaw rotates its terrain, moves its objects, **and** turns
the objects that carry a `yaw` of their own. An object's `yaw` is authored relative to its
segment, in degrees, and the segment's turn is added to it at load.

Three classes of object, from `LevelObject::orientability()`:

| Class | Objects | Behaviour under a rotated segment |
|-------|---------|-----------------------------------|
| **Turns** | `Box`, `Plank`, `Stack`, `Tower`, `BoxWall`, `HoneycombWall`, `Trampoline`, `Table`, `Dolos`, `Trilithon`, `PlankBridge`, `Domino` | Turns with the segment. `Domino` rotates its `direction` vector instead of taking a `yaw`. |
| **Symmetric** | `BeachBall`, `GlowingOrb`, `Crate`, `HeavyCrate`, `Capsule`, `Menhir`, `FencePost`, `PlayWheel`, the regular solids, `HexPrism`, `Jack`, `Pyramid`, `Jenga` | Nothing to do — a quarter turn changes nothing observable. |
| **Fixed** | `Banana`, `House`, `Pendulum`, `Seesaw`, `VoussoirArch`, `Temple` | **Keeps its world orientation while the segment turns around it.** `level_check` warns; author these in an unrotated segment. |

`PlankBridge.yaw` is in **degrees**, like every other yaw in the format. It was radians
before stage 3.

## Level File Structure

```ron
// levels/example.level.ron
Level(
    name: "Example",

    segments: [
        (
            name: "plaza",

            terrain: Terrain(
                voxel_size: 1.0,     // used directly; chunk depth is fixed by the chunk definition
                // Region terrain is generated within, in SEGMENT-LOCAL coordinates.
                // Chunks are allocated on demand, so an extent larger than the
                // content costs storage only where filled.
                bounds: (min: (0.0, -32.0, 0.0), max: (96.0, 32.0, 96.0)),
                base_height: 0.0,
                material_layers: [
                    (depth: 1.0,  material: Grass),
                    (depth: 4.0,  material: Dirt),
                    (depth: 999.0, material: Rock),
                ],

                // Heightfield features — modify surface height at each (x, z) column
                features: [
                    Hill(center: (10.0, 15.0), radius: 8.0, height: 5.0),
                    Crater(center: (34.0, 34.0), radius: 6.0, depth: 3.0),
                    Plateau(min: (86.0, 34.0), max: (96.0, 62.0), height: 4.0),
                    Wall(from: (5.0, 10.0), to: (5.0, 30.0), height: 6.0, thickness: 2.0),
                    Ramp(from: (58.0, 48.0), to: (88.0, 48.0), start_height: 0.0, end_height: 4.0, width: 26.0),
                    TerrainRoughness(frequency: 0.1, amplitude: 0.5, octaves: 3, seed: 42),
                ],

                // Volumetric features — place or carve voxels in 3D (evaluated after heightfield)
                volumes: [
                    Island(center: (20.0, 15.0, 40.0), half_extents: (6.0, 2.0, 6.0), edge_noise: 0.3),
                    Arch(from: (10.0, 8.0, 40.0), to: (20.0, 8.0, 40.0), radius: 3.0, thickness: 1.5),
                    Pillar(center: (5.0, 40.0), height: 12.0, radius: 2.0),
                    Tunnel(center: (0.0, 15.0), direction: (1.0, 0.0), length: 12.0, radius: 2.5, depth: 2.0),
                ],
            ),

            // Named local frames. `yaw` defaults to 0, which faces out along +X.
            anchors: [
                (name: "exit_east", pos: (96.0, 4.0, 48.0), yaw: 0.0),
            ],

            // Objects, in segment-local coordinates.
            objects: [
                BeachBall(pos: (3.0, 5.0, 2.0)),

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
                    items: [Crate(size: 2.0), Crate(size: 2.0), Crate(size: 1.0)],
                ),

                // Tower — shorthand for N identical boxes stacked
                Tower(base: (5.0, 0.0, 8.0), box_half_extents: (0.5, 0.5, 0.5), count: 6, density: 40.0),

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
                House(pos: (0.0, 10.0, 5.0), half_extents: (2.0, 1.5, 2.0)),
            ],
        ),

        (
            name: "tower",
            terrain: Terrain(
                // A different resolution from its neighbour is fine: the join is a
                // gap, so nothing has to mesh across it.
                voxel_size: 0.5,
                bounds: (min: (0.0, -16.0, 0.0), max: (48.0, 24.0, 48.0)),
                base_height: 2.0,
                features: [],
            ),
            anchors: [
                // On the -x face, facing back toward the plaza.
                (name: "entry", pos: (0.0, 2.0, 24.0), yaw: 180.0),
            ],
        ),
    ],

    placements: [
        Root(segment: "plaza", origin: (0.0, 0.0, 0.0), yaw: 0.0),
        Join(segment: "tower", anchor: "entry", to: "plaza.exit_east", gap: 6.0),
    ],

    // Assertions: two anchors meet, but nothing is derived from it.
    connections: [
        // (from: "tower.back_door", to: "plaza.side_gate", gap: 5.0),
    ],

    // In the ROOT segment's local frame.
    player_spawn: (30.0, 3.0, 62.0),
)
```

## Data Model (Rust structs)

```
src/level/
├── mod.rs              // re-exports
├── data.rs             // serde structs
├── loader.rs           // RON parsing, validation, placement resolution
├── placement.rs        // placement tree → one SegmentFrame per segment
└── spawner.rs          // data → segments, ECS entities, water
```

### `data.rs` — Pure Data Structs

```rust
use serde::Deserialize;

#[derive(Deserialize)]
pub struct Level {
    pub name: String,
    pub segments: Vec<SegmentDef>,
    pub placements: Vec<Placement>,
    #[serde(default)]
    pub connections: Vec<Connection>,
    /// In the root segment's local frame; rewritten to world by `load_level`.
    pub player_spawn: (f32, f32, f32),
    #[serde(default)]
    pub water: Option<WaterConfig>,
    /// Derived: one world frame per segment, filled in by `load_level`.
    #[serde(skip)]
    pub frames: Vec<SegmentFrame>,
}

#[derive(Deserialize)]
pub struct SegmentDef {
    pub name: String,
    pub terrain: Terrain,        // carries voxel_size and bounds, segment-local
    #[serde(default)]
    pub anchors: Vec<AnchorDef>,
    #[serde(default)]
    pub objects: Vec<LevelObject>,
}

#[derive(Deserialize)]
pub struct AnchorDef {
    pub name: String,
    pub pos: (f32, f32, f32),
    /// Multiple of 90°. 0 faces out along local +X.
    #[serde(default)]
    pub yaw: f32,
}

#[derive(Deserialize)]
pub enum Placement {
    Root { segment: String, origin: (f32, f32, f32), yaw: f32 },
    Join { segment: String, anchor: String, to: String, gap: f32, weld: bool },
}

#[derive(Deserialize)]
pub struct Connection {
    pub from: String,   // "segment.anchor"
    pub to: String,     // "segment.anchor"
    pub gap: f32,
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
    // voxel_size > 0, positive extent on every axis, a voxels-per-axis ceiling,
    // player_spawn inside terrain bounds, etc.
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

### Pass 2b: Traversal primitives

Volume features too, but a different kind of thing: everything above shapes *landscape*,
these shape a *route*. Each takes a `material` (default `Rock`) and a `thickness`
(default 1.0), and each puts its **walking surface at the authored `y`** rather than its
centre — so a deck at `y: 6.0` is a deck you stand on at 6 m.

| Feature | Effect | Parameters |
|---------|--------|------------|
| **Path** | A deck of constant width swept along a polyline: catwalk, bridge, cliff ledge, spiral ramp | points (xyz, ≥2), width, thickness, profile (`Flat`/`Rounded`), material |
| **Platform** | Free-standing slab — the atom of a jump sequence | center (xyz, y = surface), half_extents (x, z), thickness, material |
| **Staircase** | Discrete treads between two heights | from/to (xyz), width, steps, thickness, material |
| **Shaft** | Vertical bore, optionally with a helical ledge down its wall | center (xz), from_y, to_y, radius, ledge, material |

`Path` is the one to reach for first. Waypoints carry their own heights, so a route that
climbs while it turns is one `Path` rather than a construction of many, and each leg is
capped with a half-round of the deck's own half-width — which is what carries the outside
of a corner without the author placing anything there. The cost is that the two open ends
overhang their waypoints by a half-width, so author an endpoint where the deck should
*meet* what it joins, not half a deck short of it.

A `Shaft`'s `ledge` is `Some((width:, thickness:, pitch:, start_angle:))`. `width` is
measured inward from the bore wall; `pitch` is the height gained per full turn and sets
how steep the descent is.

Two things `level_check` will tell you off for, both worth knowing before you author:

- **A deck narrower than about 1 m** is under two player-collider diameters and has no
  margin either side; under 0.5 m the player does not fit at all.
- **Any dimension under two voxels** of its segment's resolution cannot be placed by the
  density encoding, and comes out at whatever the lattice decides — differently at every
  resolution. A 1.5 m deck needs 0.75 m voxels or finer.

A gap between two anchors that a `Path` or `Platform` spans is reported as a **walk**
rather than a jump. Note that generation clips every feature to its segment's
`terrain.bounds`, so a deck reaching past them does not exist, and the check knows that.

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

## Objectives

A level can ask something of the player. Two objects carry it; `src/objective/`
holds the state and the HUD.

```ron
// Hangs where authored, bobs and spins, no physics. Caught by touch.
Gem(pos: (4.0, 1.0, -3.0)),
Gem(pos: (6.0, 1.0, -4.5), colour: Some((0.4, 0.85, 1.0))),

// A plinth, a ring of posts and a beacon column. `pos` is the ground point
// at the foot of the beacon; `radius` (default 2.0) is how close the player
// has to come, and `required_gems` (default 0) how many gems must already be
// caught for arriving to count.
Goal(pos: (8.0, 0.0, -8.0), radius: 2.5, required_gems: 2),
```

The HUD shows `Gems  n / total` (hidden when the level has none), the elapsed
`Time`, `need n more gems` while standing in a goal that has not opened, and
`LEVEL COMPLETE` with the finish time. The count is not validated against the
level: a goal that wants more gems than the level places is unfinishable.

Demo levels built on this: `thin_ice`, `wrecking_yard`, `skyway`.

## Integration with App::new()

The change to `App::new()` is minimal. Replace the hardcoded spawn block (lines 66–99) with:

```rust
let level = load_level(Path::new("levels/test_arena.level.ron"))?;
spawn_level(&mut world, &level, &materials);
```

Terrain creation moves into the level module: `build_segments` generates each segment's grid at
its own resolution and places it at the frame the placement tree derived, and `TerrainWorld`
owns the result.

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

> **Historical — completed, and describing a superseded design.** These steps record how the
> level system was originally built. They still reference `world_size`, `voxel_size` on
> `Level`, and `Level::octree_depth()`, none of which exist any more. Read this section for
> the reasoning behind the *object and spawnable* design only; for anything about world
> sizing, octree depth or terrain storage, see `docs/LEVEL_SEGMENTS_PLAN.md`.

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

> **Historical — pre-segments syntax.** This sketch predates stage 2 and uses the old
> single-terrain top level (`world_size`, one `terrain`, one flat `objects` list). It is kept
> because the *level design* it illustrates is still the point. For a current, working
> multi-segment example see `levels/test_segments.level.ron`.

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
