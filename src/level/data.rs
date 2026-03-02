//! Pure data types for level file deserialization.
//!
//! These structs map directly to the RON level file format. They carry no
//! engine dependencies — only `serde::Deserialize` — so the format can evolve
//! independently of the runtime.

use serde::Deserialize;

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
#[derive(Deserialize)]
pub enum LevelObject {
    BeachBall {
        pos: (f32, f32, f32),
    },
    /// Raw box with full control over dimensions, appearance, and physics.
    Box {
        pos: (f32, f32, f32),
        half_extents: (f32, f32, f32),
        #[serde(default)]
        style: BoxStyle,
        #[serde(default = "default_density")]
        density: f32,
        #[serde(default = "default_box_restitution")]
        restitution: f32,
        #[serde(default = "default_box_friction")]
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
        #[serde(default = "default_density")]
        density: f32,
    },
    /// Grid of boxes, optionally staggered for a brick-like pattern.
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
        #[serde(default = "default_density")]
        density: f32,
        #[serde(default = "default_capsule_restitution")]
        restitution: f32,
        #[serde(default = "default_capsule_friction")]
        friction: f32,
    },
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

// ---------------------------------------------------------------------------
// Default value functions for serde
// ---------------------------------------------------------------------------

fn default_density() -> f32 {
    50.0
}

fn default_box_restitution() -> f32 {
    0.2
}

fn default_box_friction() -> f32 {
    0.6
}

fn default_capsule_restitution() -> f32 {
    0.2
}

fn default_capsule_friction() -> f32 {
    0.6
}
