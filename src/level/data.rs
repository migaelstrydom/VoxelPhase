//! Pure data types for level file deserialization.
//!
//! These structs map directly to the RON level file format. They carry no
//! engine dependencies — only `serde::Deserialize` — so the format can evolve
//! independently of the runtime.

use serde::Deserialize;

use crate::app::spawnables::{
    BeachBallDef, BoxDef, BoxWallDef, CapsuleDef, CrateDef, DodecahedronDef, DolosDef, DominoDef,
    FencePostDef, GlowingOrbDef, HeavyCrateDef, HexPrismDef, HoneycombWallDef, HouseDef,
    IcosahedronDef, JackDef, JengaDef, MenhirDef, OctahedronDef, PendulumDef, PlankBridgeDef,
    PlankDef, PlayWheelDef, PyramidDef, SeesawDef, Spawnable, StackDef, StackItemDef, TableDef,
    TempleDef, TetrahedronDef, TowerDef, TrampolineDef, TrilithonDef, VoussoirArchDef,
};

/// Top-level level description.
#[derive(Deserialize)]
pub struct Level {
    pub name: String,
    /// Half-size of the cubic SVO world bounds.
    pub world_size: f32,
    /// Size of the smallest voxel. Octree depth is computed as
    /// `log2(world_size / voxel_size)`.
    pub voxel_size: f32,
    pub terrain: Terrain,
    pub player_spawn: (f32, f32, f32),
    pub objects: Vec<LevelObject>,

    /// Water configuration. If omitted, no water system is created.
    #[serde(default)]
    pub water: Option<WaterConfig>,
}

impl Level {
    /// Compute the octree depth required for the configured world and voxel sizes.
    pub fn octree_depth(&self) -> u32 {
        (self.world_size / self.voxel_size).log2() as u32
    }
}

/// Terrain description: a heightfield plus optional volumetric features.
#[derive(Deserialize)]
pub struct Terrain {
    /// Base surface height before any features are applied.
    pub base_height: f32,
    /// Material layering by depth below the surface.
    #[serde(default)]
    pub material_layers: Vec<MaterialLayer>,
    /// Heightfield features — modify surface height at each (x, z) column.
    pub features: Vec<TerrainFeature>,
    /// Volumetric features — place or carve voxels in 3D (evaluated after heightfield).
    #[serde(default)]
    pub volumes: Vec<VolumeFeature>,
}

/// Maps a depth range below the terrain surface to a voxel material.
#[derive(Deserialize)]
pub struct MaterialLayer {
    /// Maximum depth (from surface) at which this material appears.
    pub depth: f32,
    pub material: VoxelMaterialId,
}

/// Voxel material identifiers matching the engine's `VoxelMaterial` enum.
#[derive(Deserialize, Clone, Copy)]
pub enum VoxelMaterialId {
    Grass,
    Dirt,
    Rock,
    Ite,
    Limestone,
    Slate,
}

/// Heightfield features operate on 2D (xz) coordinates, modifying the terrain
/// surface height additively.
#[derive(Deserialize)]
pub enum TerrainFeature {
    /// Smooth dome with cosine falloff.
    Hill {
        center: (f32, f32),
        radius: f32,
        height: f32,
    },
    /// Inverted dome — subtracts height.
    Crater {
        center: (f32, f32),
        radius: f32,
        depth: f32,
    },
    /// Flat rectangular region set to a fixed height.
    Plateau {
        min: (f32, f32),
        max: (f32, f32),
        height: f32,
    },
    /// Thin raised strip along a line segment.
    Wall {
        from: (f32, f32),
        to: (f32, f32),
        height: f32,
        thickness: f32,
    },
    /// Linear height gradient between two points.
    Ramp {
        from: (f32, f32),
        to: (f32, f32),
        start_height: f32,
        end_height: f32,
        width: f32,
    },
    /// FBM noise layer for natural surface variation.
    TerrainRoughness {
        frequency: f32,
        amplitude: f32,
        octaves: u32,
        seed: u32,
    },
    /// Sharp height transition along a line segment, creating a cliff face.
    ///
    /// Terrain on the `high_side` of the edge is at `high_height`; terrain on
    /// the opposite side is at `low_height`. The transition between them is
    /// controlled by `steepness` — higher values produce a narrower, more
    /// vertical face.
    Cliff {
        /// Start of the cliff edge line.
        from: (f32, f32),
        /// End of the cliff edge line.
        to: (f32, f32),
        /// Terrain height on the low side.
        low_height: f32,
        /// Terrain height on the high side.
        high_height: f32,
        /// Direction (xz) pointing toward the high side of the cliff.
        /// Does not need to be normalised.
        high_side: (f32, f32),
        /// Controls how narrow the height transition is. Higher values
        /// produce a steeper, more vertical cliff face.
        steepness: f32,
        /// Distance over which the cliff effect fades to nothing beyond each
        /// endpoint of the edge line. 0.0 = cliff extends infinitely.
        #[serde(default)]
        end_falloff: f32,
        /// Optional noise roughness applied to the cliff face. 0.0 = smooth.
        #[serde(default)]
        roughness: f32,
        /// Noise seed for face roughness.
        #[serde(default = "default_cliff_seed")]
        roughness_seed: u32,
    },
}

fn default_cliff_seed() -> u32 {
    42
}

/// A point on a depth-vs-threshold curve for cave generation.
///
/// The carve threshold is linearly interpolated between consecutive points.
/// At a given depth below the surface, if the 3D noise value exceeds the
/// interpolated threshold, the voxel is carved to air.
#[derive(Deserialize, Clone)]
pub struct CaveDepthPoint {
    /// Depth below the terrain surface.
    pub depth: f32,
    /// Noise threshold at this depth (0.0 = carve everything, 1.0 = carve nothing).
    pub threshold: f32,
}

/// Spatial region that controls where caves are generated.
///
/// Caves are strongest within `radius` of the center point, then fade out
/// over the `falloff` distance by blending the carve threshold toward 1.0
/// (no carving). Outside `radius + falloff`, no caves are generated.
#[derive(Deserialize, Clone)]
pub struct CaveRegion {
    /// Center of the cave region in world coordinates (x, y, z).
    pub center: (f32, f32, f32),
    /// Radius within which caves are at full strength.
    pub radius: f32,
    /// Distance beyond `radius` over which caves fade to nothing.
    pub falloff: f32,
}

/// Volumetric features operate in 3D, directly placing or carving voxels.
/// Applied as a second pass after the heightfield.
#[derive(Deserialize)]
pub enum VolumeFeature {
    /// Floating solid mass with optional noisy edges for an organic look.
    Island {
        center: (f32, f32, f32),
        half_extents: (f32, f32, f32),
        edge_noise: f32,
    },
    /// Curved bridge between two 3D points.
    Arch {
        from: (f32, f32, f32),
        to: (f32, f32, f32),
        radius: f32,
        thickness: f32,
    },
    /// Vertical column rising from the heightfield surface.
    Pillar {
        center: (f32, f32),
        height: f32,
        radius: f32,
    },
    /// Horizontal bore that carves through existing terrain.
    Tunnel {
        center: (f32, f32),
        direction: (f32, f32),
        length: f32,
        radius: f32,
        depth: f32,
    },
    /// 3D noise-driven cave carving with depth-dependent threshold.
    ///
    /// Carves overhangs near the surface, cave networks at medium depth,
    /// and leaves deep rock solid. The `depth_curve` controls the carve
    /// threshold at each depth below the heightfield surface.
    Caves {
        /// Noise sample frequency — lower values produce larger caverns.
        frequency: f32,
        /// Number of FBM octaves — more octaves add finer wall detail.
        octaves: u32,
        /// Noise seed for reproducibility.
        seed: u32,
        /// Piecewise-linear curve mapping depth below surface to carve threshold.
        /// Must be sorted by depth ascending. Noise values above the threshold
        /// at a given depth will be carved.
        depth_curve: Vec<CaveDepthPoint>,
        /// Optional spatial region with soft falloff. If omitted, caves span
        /// the entire terrain.
        #[serde(default)]
        region: Option<CaveRegion>,
        /// Material layering for cave surfaces, by depth from the cave wall.
        /// If empty, falls back to the terrain's material layers.
        #[serde(default)]
        material_layers: Vec<MaterialLayer>,
        /// Downward bias applied below the local cave midpoint to flatten
        /// cave floors, making them more walkable. 0.0 = no flattening.
        #[serde(default)]
        floor_bias: f32,
    },
    /// Solid rock lip extending horizontally from a cliff edge, creating
    /// a dramatic overhang. Pairs with `TerrainFeature::Cliff`.
    Overhang {
        /// Start of the cliff edge line (xz).
        from: (f32, f32),
        /// End of the cliff edge line (xz).
        to: (f32, f32),
        /// Y level of the overhang (typically the cliff's `high_height`).
        height: f32,
        /// How far the lip extends outward from the cliff edge.
        depth: f32,
        /// Vertical thickness of the lip at the cliff edge.
        thickness: f32,
        /// Direction (xz) the lip extends toward (away from the cliff top).
        /// Does not need to be normalised.
        direction: (f32, f32),
        /// Noise amplitude for organic underside roughness. 0.0 = smooth.
        #[serde(default)]
        noise: f32,
        /// Noise seed for underside roughness.
        #[serde(default = "default_overhang_seed")]
        noise_seed: u32,
    },
}

fn default_overhang_seed() -> u32 {
    99
}

/// Visual style for box textures.
#[derive(Deserialize, Clone, Copy, Default)]
pub enum BoxStyle {
    WoodenCrate,
    Cardboard,
    Metal,
    Gift,
    Stone,
    Brick,
    Warning,
    #[default]
    Random,
}

/// Objects that can be placed in a level.
///
/// Each variant maps to a RON-compatible format. The [`LevelObject::to_spawnable`]
/// method converts any variant into a boxed [`Spawnable`] implementation.
/// To add a new object type:
/// 1. Create its `*Def` struct with a `Spawnable` impl in `src/app/spawnables/`.
/// 2. Add a variant here with the same fields.
/// 3. Add a conversion arm in `to_spawnable()`.
#[derive(Deserialize)]
pub enum LevelObject {
    BeachBall {
        pos: (f32, f32, f32),
    },
    /// Self-illuminated sphere with beach-ball physics. Subject for lighting work.
    GlowingOrb {
        pos: (f32, f32, f32),
        /// Surface and glow tint. Defaults to cyan.
        #[serde(default)]
        colour: Option<(f32, f32, f32)>,
        /// Emissive strength multiplier.
        #[serde(default)]
        glow: Option<f32>,
    },
    /// Raw box with full control over dimensions, appearance, and physics.
    Box {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
        #[serde(default)]
        style: BoxStyle,
        #[serde(default = "BoxDef::default_density")]
        density: f32,
        #[serde(default = "BoxDef::default_restitution")]
        restitution: f32,
        #[serde(default = "BoxDef::default_friction")]
        friction: f32,
    },
    /// Thin wooden board. Defaults: style=WoodenCrate, density=20, thickness=0.1.
    Plank {
        pos: (f32, f32, f32),
        length: f32,
        width: f32,
    },
    /// Standard crate. Defaults: style=WoodenCrate, density=50.
    Crate {
        pos: (f32, f32, f32),
        /// Cube half-extent.
        size: f32,
    },
    /// Dense crate. Defaults: style=Metal, density=150.
    HeavyCrate {
        pos: (f32, f32, f32),
        size: f32,
    },
    /// Vertical stack — auto-computes Y positions bottom-up from base.
    Stack {
        base: (f32, f32, f32),
        items: Vec<StackItem>,
    },
    /// Shorthand for N identical boxes stacked vertically.
    Tower {
        base: (f32, f32, f32),
        box_half_extents: (f32, f32, f32),
        count: u32,
        #[serde(default = "TowerDef::default_density")]
        density: f32,
    },
    /// Grid of boxes, optionally staggered for a brick-like pattern.
    BoxWall {
        base: (f32, f32, f32),
        box_half_extents: (f32, f32, f32),
        columns: u32,
        rows: u32,
        #[serde(default = "BoxWallDef::default_density")]
        density: f32,
        #[serde(default)]
        stagger: bool,
    },
    /// Prefab house structure built from boxes.
    House {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
    },
    /// Capsule (cylinder + hemisphere caps).
    Capsule {
        pos: (f32, f32, f32),
        half_height: f32,
        radius: f32,
        #[serde(default = "CapsuleDef::default_density")]
        density: f32,
        #[serde(default = "CapsuleDef::default_restitution")]
        restitution: f32,
        #[serde(default = "CapsuleDef::default_friction")]
        friction: f32,
    },
    /// Standing stone anchored to terrain. Released when terrain is destroyed.
    Menhir {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "MenhirDef::default_half_height")]
        half_height: f32,
        #[serde(default = "MenhirDef::default_bottom_radius")]
        bottom_radius: f32,
        #[serde(default = "MenhirDef::default_top_radius")]
        top_radius: f32,
        #[serde(default = "MenhirDef::default_density")]
        density: f32,
    },
    /// Vertical post anchored to terrain. Released when terrain is destroyed.
    FencePost {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "FencePostDef::default_half_height")]
        half_height: f32,
        #[serde(default = "FencePostDef::default_radius")]
        radius: f32,
        #[serde(default = "FencePostDef::default_density")]
        density: f32,
    },
    /// Pendulum — terrain-anchored frame with a swinging ball.
    Pendulum {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "PendulumDef::default_frame_height")]
        frame_height: f32,
        #[serde(default = "PendulumDef::default_arm_length")]
        arm_length: f32,
        #[serde(default = "PendulumDef::default_rope_length")]
        rope_length: f32,
        #[serde(default = "PendulumDef::default_ball_radius")]
        ball_radius: f32,
        #[serde(default = "PendulumDef::default_ball_density")]
        ball_density: f32,
    },
    /// Spinning playground wheel anchored to terrain. Spins freely around Y.
    PlayWheel {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "PlayWheelDef::default_radius")]
        radius: f32,
        #[serde(default = "PlayWheelDef::default_thickness")]
        thickness: f32,
        #[serde(default = "PlayWheelDef::default_density")]
        density: f32,
        #[serde(default = "PlayWheelDef::default_hub_height")]
        hub_height: f32,
    },
    /// Seesaw — tilting beam with seats, anchored at fulcrum.
    Seesaw {
        /// Position (x, z). Y is determined by terrain surface height.
        pos: (f32, f32),
        #[serde(default = "SeesawDef::default_beam_half_length")]
        beam_half_length: f32,
        #[serde(default = "SeesawDef::default_beam_half_width")]
        beam_half_width: f32,
        #[serde(default = "SeesawDef::default_beam_half_thickness")]
        beam_half_thickness: f32,
        #[serde(default = "SeesawDef::default_fulcrum_height")]
        fulcrum_height: f32,
        #[serde(default = "SeesawDef::default_density")]
        density: f32,
    },
    /// Regular tetrahedron with ConvexHull collider.
    Tetrahedron {
        pos: (f32, f32, f32),
        #[serde(default = "TetrahedronDef::default_size")]
        size: f32,
        #[serde(default = "TetrahedronDef::default_density")]
        density: f32,
        #[serde(default = "TetrahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "TetrahedronDef::default_friction")]
        friction: f32,
    },
    /// Regular octahedron with ConvexHull collider.
    Octahedron {
        pos: (f32, f32, f32),
        #[serde(default = "OctahedronDef::default_size")]
        size: f32,
        #[serde(default = "OctahedronDef::default_density")]
        density: f32,
        #[serde(default = "OctahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "OctahedronDef::default_friction")]
        friction: f32,
    },
    /// Regular dodecahedron with ConvexHull collider.
    Dodecahedron {
        pos: (f32, f32, f32),
        #[serde(default = "DodecahedronDef::default_size")]
        size: f32,
        #[serde(default = "DodecahedronDef::default_density")]
        density: f32,
        #[serde(default = "DodecahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "DodecahedronDef::default_friction")]
        friction: f32,
    },
    /// Hexagonal prism with ConvexHull collider.
    HexPrism {
        pos: (f32, f32, f32),
        #[serde(default = "HexPrismDef::default_radius")]
        radius: f32,
        #[serde(default = "HexPrismDef::default_half_height")]
        half_height: f32,
        #[serde(default = "HexPrismDef::default_density")]
        density: f32,
        #[serde(default = "HexPrismDef::default_restitution")]
        restitution: f32,
        #[serde(default = "HexPrismDef::default_friction")]
        friction: f32,
    },
    /// Honeycomb wall — tiled hexagonal prisms in a honeycomb grid.
    HoneycombWall {
        base: (f32, f32, f32),
        #[serde(default = "HoneycombWallDef::default_columns")]
        columns: u32,
        #[serde(default = "HoneycombWallDef::default_rows")]
        rows: u32,
        #[serde(default = "HexPrismDef::default_radius")]
        radius: f32,
        #[serde(default = "HexPrismDef::default_half_height")]
        half_height: f32,
        #[serde(default = "HexPrismDef::default_density")]
        density: f32,
    },
    /// Regular icosahedron with ConvexHull collider.
    Icosahedron {
        pos: (f32, f32, f32),
        #[serde(default = "IcosahedronDef::default_size")]
        size: f32,
        #[serde(default = "IcosahedronDef::default_density")]
        density: f32,
        #[serde(default = "IcosahedronDef::default_restitution")]
        restitution: f32,
        #[serde(default = "IcosahedronDef::default_friction")]
        friction: f32,
    },
    /// Trampoline — bouncy pad on four short legs.
    Trampoline {
        pos: (f32, f32, f32),
        #[serde(default = "TrampolineDef::default_pad_half_extents")]
        pad_half_extents: (f32, f32, f32),
        #[serde(default = "TrampolineDef::default_leg_half_extents")]
        leg_half_extents: (f32, f32, f32),
        #[serde(default = "TrampolineDef::default_density")]
        density: f32,
        #[serde(default = "TrampolineDef::default_restitution")]
        restitution: f32,
    },
    /// Table — compound body (top slab + 4 legs).
    Table {
        pos: (f32, f32, f32),
        /// Half-extents of the table top (x, y_thickness, z).
        #[serde(default = "TableDef::default_top_half_extents")]
        top_half_extents: (f32, f32, f32),
        /// Half-extents of each leg.
        #[serde(default = "TableDef::default_leg_half_extents")]
        leg_half_extents: (f32, f32, f32),
        #[serde(default = "TableDef::default_density")]
        density: f32,
        #[serde(default = "TableDef::default_restitution")]
        restitution: f32,
        #[serde(default = "TableDef::default_friction")]
        friction: f32,
    },
    /// Pyramid — square-based pyramid of sandstone blocks.
    Pyramid {
        base: (f32, f32, f32),
        block_half_extents: (f32, f32, f32),
        /// Number of blocks along each side of the bottom layer.
        base_width: u32,
        #[serde(default = "PyramidDef::default_density")]
        density: f32,
    },
    /// Dolos — concrete breakwater armour unit. Central shank with two
    /// perpendicular flukes, one at each end.
    Dolos {
        pos: (f32, f32, f32),
        #[serde(default = "DolosDef::default_shank_length")]
        shank_length: f32,
        #[serde(default = "DolosDef::default_fluke_length")]
        fluke_length: f32,
        #[serde(default = "DolosDef::default_thickness")]
        thickness: f32,
        #[serde(default = "DolosDef::default_density")]
        density: f32,
        #[serde(default = "DolosDef::default_restitution")]
        restitution: f32,
        #[serde(default = "DolosDef::default_friction")]
        friction: f32,
    },
    /// Domino row — tall thin blocks spaced for chain toppling.
    Domino {
        base: (f32, f32, f32),
        /// Direction the row extends in (x, z).
        direction: (f32, f32),
        #[serde(default = "DominoDef::default_count")]
        count: u32,
        #[serde(default = "DominoDef::default_spacing")]
        spacing: f32,
        #[serde(default = "DominoDef::default_half_extents")]
        half_extents: (f32, f32, f32),
        #[serde(default = "DominoDef::default_density")]
        density: f32,
    },
    /// Voussoir arch — semicircular masonry arch with abutment pillars.
    VoussoirArch {
        base: (f32, f32, f32),
        #[serde(default = "VoussoirArchDef::default_inner_radius")]
        inner_radius: f32,
        #[serde(default = "VoussoirArchDef::default_thickness")]
        thickness: f32,
        #[serde(default = "VoussoirArchDef::default_depth")]
        depth: f32,
        #[serde(default = "VoussoirArchDef::default_num_voussoirs")]
        num_voussoirs: u32,
        #[serde(default = "VoussoirArchDef::default_abutment_height")]
        abutment_height: f32,
        #[serde(default = "VoussoirArchDef::default_density")]
        density: f32,
        #[serde(default = "VoussoirArchDef::default_friction")]
        friction: f32,
    },
    /// Jack — classic six-pointed metal toy. Three perpendicular bars.
    Jack {
        pos: (f32, f32, f32),
        #[serde(default = "JackDef::default_length")]
        length: f32,
        #[serde(default = "JackDef::default_thickness")]
        thickness: f32,
        #[serde(default = "JackDef::default_density")]
        density: f32,
        #[serde(default = "JackDef::default_restitution")]
        restitution: f32,
        #[serde(default = "JackDef::default_friction")]
        friction: f32,
    },
    /// Jenga tower — alternating layers of three planks rotated 90°.
    Jenga {
        base: (f32, f32, f32),
        #[serde(default = "JengaDef::default_layers")]
        layers: u32,
        #[serde(default = "JengaDef::default_block_half_length")]
        block_half_length: f32,
        #[serde(default = "JengaDef::default_density")]
        density: f32,
        #[serde(default = "JengaDef::default_friction")]
        friction: f32,
    },
    /// Plank bridge — rustic destructible bridge with two beams and cross-planks.
    PlankBridge {
        pos: (f32, f32, f32),
        #[serde(default = "PlankBridgeDef::default_length")]
        length: f32,
        #[serde(default = "PlankBridgeDef::default_beam_spacing")]
        beam_spacing: f32,
        #[serde(default = "PlankBridgeDef::default_plank_count")]
        plank_count: u32,
        #[serde(default = "PlankBridgeDef::default_beam_half_extents")]
        beam_half_extents: (f32, f32, f32),
        #[serde(default = "PlankBridgeDef::default_plank_half_extents")]
        plank_half_extents: (f32, f32, f32),
        #[serde(default = "PlankBridgeDef::default_density")]
        density: f32,
        #[serde(default)]
        yaw: f32,
        #[serde(default = "PlankBridgeDef::default_fracture_threshold")]
        fracture_threshold: f32,
    },
    /// Neolithic trilithon — two uprights with a lintel.
    Trilithon {
        pos: (f32, f32, f32),
        #[serde(default = "TrilithonDef::default_upright_half_height")]
        upright_half_height: f32,
        #[serde(default = "TrilithonDef::default_upright_half_width")]
        upright_half_width: f32,
        #[serde(default = "TrilithonDef::default_upright_half_depth")]
        upright_half_depth: f32,
        #[serde(default = "TrilithonDef::default_gap")]
        gap: f32,
        #[serde(default = "TrilithonDef::default_lintel_half_thickness")]
        lintel_half_thickness: f32,
        #[serde(default = "TrilithonDef::default_lintel_overhang")]
        lintel_overhang: f32,
        #[serde(default = "TrilithonDef::default_density")]
        density: f32,
    },
    /// Classical Greek Doric temple with peristyle colonnade.
    Temple {
        pos: (f32, f32, f32),
        #[serde(default = "TempleDef::default_column_height")]
        column_height: f32,
        #[serde(default = "TempleDef::default_front_columns")]
        front_columns: u32,
        #[serde(default = "TempleDef::default_side_columns")]
        side_columns: u32,
    },
}

impl LevelObject {
    /// Convert this level object into a boxed [`Spawnable`].
    ///
    /// This bridges the RON deserialization format (named-field enum variants)
    /// with the spawnable trait system. The allocation only happens at level
    /// load time, not per frame.
    pub fn to_spawnable(&self) -> Box<dyn Spawnable> {
        match self {
            LevelObject::BeachBall { pos } => Box::new(BeachBallDef { pos: *pos }),

            LevelObject::GlowingOrb { pos, colour, glow } => Box::new(GlowingOrbDef {
                pos: *pos,
                colour: *colour,
                glow: *glow,
            }),

            LevelObject::Box {
                pos,
                half_extents,
                style,
                density,
                restitution,
                friction,
            } => Box::new(BoxDef {
                pos: *pos,
                half_extents: *half_extents,
                style: *style,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Plank { pos, length, width } => Box::new(PlankDef {
                pos: *pos,
                length: *length,
                width: *width,
            }),

            LevelObject::Crate { pos, size } => Box::new(CrateDef {
                pos: *pos,
                size: *size,
            }),

            LevelObject::HeavyCrate { pos, size } => Box::new(HeavyCrateDef {
                pos: *pos,
                size: *size,
            }),

            LevelObject::Stack { base, items } => Box::new(StackDef {
                base: *base,
                items: items
                    .iter()
                    .map(|item| match item {
                        StackItem::Crate { size } => StackItemDef::Crate { size: *size },
                        StackItem::HeavyCrate { size } => StackItemDef::HeavyCrate { size: *size },
                        StackItem::Plank { length, width } => StackItemDef::Plank {
                            length: *length,
                            width: *width,
                        },
                        StackItem::BeachBall => StackItemDef::BeachBall,
                        StackItem::Capsule {
                            half_height,
                            radius,
                        } => StackItemDef::Capsule {
                            half_height: *half_height,
                            radius: *radius,
                        },
                    })
                    .collect(),
            }),

            LevelObject::Tower {
                base,
                box_half_extents,
                count,
                density,
            } => Box::new(TowerDef {
                base: *base,
                box_half_extents: *box_half_extents,
                count: *count,
                density: *density,
            }),

            LevelObject::BoxWall {
                base,
                box_half_extents,
                columns,
                rows,
                density,
                stagger,
            } => Box::new(BoxWallDef {
                base: *base,
                box_half_extents: *box_half_extents,
                columns: *columns,
                rows: *rows,
                density: *density,
                stagger: *stagger,
            }),

            LevelObject::House { pos, half_extents } => Box::new(HouseDef {
                pos: *pos,
                half_extents: *half_extents,
            }),

            LevelObject::Capsule {
                pos,
                half_height,
                radius,
                density,
                restitution,
                friction,
            } => Box::new(CapsuleDef {
                pos: *pos,
                half_height: *half_height,
                radius: *radius,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Menhir {
                pos,
                half_height,
                bottom_radius,
                top_radius,
                density,
            } => Box::new(MenhirDef {
                pos: *pos,
                half_height: *half_height,
                bottom_radius: *bottom_radius,
                top_radius: *top_radius,
                density: *density,
            }),

            LevelObject::FencePost {
                pos,
                half_height,
                radius,
                density,
            } => Box::new(FencePostDef {
                pos: *pos,
                half_height: *half_height,
                radius: *radius,
                density: *density,
            }),

            LevelObject::Pendulum {
                pos,
                frame_height,
                arm_length,
                rope_length,
                ball_radius,
                ball_density,
            } => Box::new(PendulumDef {
                pos: *pos,
                frame_height: *frame_height,
                arm_length: *arm_length,
                rope_length: *rope_length,
                ball_radius: *ball_radius,
                ball_density: *ball_density,
            }),

            LevelObject::PlayWheel {
                pos,
                radius,
                thickness,
                density,
                hub_height,
            } => Box::new(PlayWheelDef {
                pos: *pos,
                radius: *radius,
                thickness: *thickness,
                density: *density,
                hub_height: *hub_height,
            }),

            LevelObject::Seesaw {
                pos,
                beam_half_length,
                beam_half_width,
                beam_half_thickness,
                fulcrum_height,
                density,
            } => Box::new(SeesawDef {
                pos: *pos,
                beam_half_length: *beam_half_length,
                beam_half_width: *beam_half_width,
                beam_half_thickness: *beam_half_thickness,
                fulcrum_height: *fulcrum_height,
                density: *density,
            }),

            LevelObject::Tetrahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(TetrahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Octahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(OctahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Dodecahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(DodecahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::HexPrism {
                pos,
                radius,
                half_height,
                density,
                restitution,
                friction,
            } => Box::new(HexPrismDef {
                pos: *pos,
                radius: *radius,
                half_height: *half_height,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::HoneycombWall {
                base,
                columns,
                rows,
                radius,
                half_height,
                density,
            } => Box::new(HoneycombWallDef {
                base: *base,
                columns: *columns,
                rows: *rows,
                radius: *radius,
                half_height: *half_height,
                density: *density,
            }),

            LevelObject::Icosahedron {
                pos,
                size,
                density,
                restitution,
                friction,
            } => Box::new(IcosahedronDef {
                pos: *pos,
                size: *size,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Trampoline {
                pos,
                pad_half_extents,
                leg_half_extents,
                density,
                restitution,
            } => Box::new(TrampolineDef {
                pos: *pos,
                pad_half_extents: *pad_half_extents,
                leg_half_extents: *leg_half_extents,
                density: *density,
                restitution: *restitution,
            }),

            LevelObject::Table {
                pos,
                top_half_extents,
                leg_half_extents,
                density,
                restitution,
                friction,
            } => Box::new(TableDef {
                pos: *pos,
                top_half_extents: *top_half_extents,
                leg_half_extents: *leg_half_extents,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Pyramid {
                base,
                block_half_extents,
                base_width,
                density,
            } => Box::new(PyramidDef {
                base: *base,
                block_half_extents: *block_half_extents,
                base_width: *base_width,
                density: *density,
            }),

            LevelObject::Dolos {
                pos,
                shank_length,
                fluke_length,
                thickness,
                density,
                restitution,
                friction,
            } => Box::new(DolosDef {
                pos: *pos,
                shank_length: *shank_length,
                fluke_length: *fluke_length,
                thickness: *thickness,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Domino {
                base,
                direction,
                count,
                spacing,
                half_extents,
                density,
            } => Box::new(DominoDef {
                base: *base,
                direction: *direction,
                count: *count,
                spacing: *spacing,
                half_extents: *half_extents,
                density: *density,
            }),

            LevelObject::VoussoirArch {
                base,
                inner_radius,
                thickness,
                depth,
                num_voussoirs,
                abutment_height,
                density,
                friction,
            } => Box::new(VoussoirArchDef {
                base: *base,
                inner_radius: *inner_radius,
                thickness: *thickness,
                depth: *depth,
                num_voussoirs: *num_voussoirs,
                abutment_height: *abutment_height,
                density: *density,
                friction: *friction,
            }),

            LevelObject::Jack {
                pos,
                length,
                thickness,
                density,
                restitution,
                friction,
            } => Box::new(JackDef {
                pos: *pos,
                length: *length,
                thickness: *thickness,
                density: *density,
                restitution: *restitution,
                friction: *friction,
            }),

            LevelObject::Jenga {
                base,
                layers,
                block_half_length,
                density,
                friction,
            } => Box::new(JengaDef {
                base: *base,
                layers: *layers,
                block_half_length: *block_half_length,
                density: *density,
                friction: *friction,
            }),

            LevelObject::PlankBridge {
                pos,
                length,
                beam_spacing,
                plank_count,
                beam_half_extents,
                plank_half_extents,
                density,
                yaw,
                fracture_threshold,
            } => Box::new(PlankBridgeDef {
                pos: *pos,
                length: *length,
                beam_spacing: *beam_spacing,
                plank_count: *plank_count,
                beam_half_extents: *beam_half_extents,
                plank_half_extents: *plank_half_extents,
                density: *density,
                yaw: *yaw,
                fracture_threshold: *fracture_threshold,
            }),

            LevelObject::Trilithon {
                pos,
                upright_half_height,
                upright_half_width,
                upright_half_depth,
                gap,
                lintel_half_thickness,
                lintel_overhang,
                density,
            } => Box::new(TrilithonDef {
                pos: *pos,
                upright_half_height: *upright_half_height,
                upright_half_width: *upright_half_width,
                upright_half_depth: *upright_half_depth,
                gap: *gap,
                lintel_half_thickness: *lintel_half_thickness,
                lintel_overhang: *lintel_overhang,
                density: *density,
            }),

            LevelObject::Temple {
                pos,
                column_height,
                front_columns,
                side_columns,
            } => Box::new(TempleDef {
                pos: *pos,
                column_height: *column_height,
                front_columns: *front_columns,
                side_columns: *side_columns,
            }),
        }
    }
}

/// Items that can appear inside a `Stack`.
#[derive(Deserialize)]
pub enum StackItem {
    Crate { size: f32 },
    HeavyCrate { size: f32 },
    Plank { length: f32, width: f32 },
    BeachBall,
    Capsule { half_height: f32, radius: f32 },
}

/// Level-authored water placement data.
#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct WaterConfig {
    /// Sea level for the infinite ocean plane. If None, no ocean is rendered
    /// and boundary cells do not act as sources/sinks.
    pub ocean_level: Option<f32>,

    /// Individual water bodies placed in the level.
    #[serde(default)]
    pub bodies: Vec<WaterBody>,
}

/// A discrete body of water placed at level load time.
///
/// Flood-fills from a seed point outward through all connected terrain cells
/// whose floor is below `surface_level`. The extent is dilated by one cell
/// to cover marching-cubes shore smoothing.
#[derive(Deserialize)]
pub enum WaterBody {
    Pool {
        /// Seed point (x, z) — must be inside a terrain depression.
        seed: (f32, f32),
        /// Target water surface height (world Y).
        surface_level: f32,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn water_config_serde_defaults_apply_when_fields_are_omitted() {
        let config: WaterConfig = ron::from_str("(bodies: [])").expect("WaterConfig should parse");

        assert_eq!(config.ocean_level, None);
        assert!(config.bodies.is_empty());
    }

    #[test]
    fn water_config_default_matches_serde_defaults() {
        let config = WaterConfig::default();

        assert_eq!(config.ocean_level, None);
        assert!(config.bodies.is_empty());
    }

    #[test]
    fn water_config_rejects_runtime_tuning_fields() {
        let parse_result: Result<WaterConfig, _> = ron::from_str("(wave_damping: 2.0, bodies: [])");

        assert!(
            parse_result.is_err(),
            "WaterConfig should reject runtime tuning fields from level data"
        );
    }
}
