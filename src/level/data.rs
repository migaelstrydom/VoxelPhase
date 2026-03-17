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
        let parse_result: Result<WaterConfig, _> =
            ron::from_str("(wave_damping: 2.0, bodies: [])");

        assert!(
            parse_result.is_err(),
            "WaterConfig should reject runtime tuning fields from level data"
        );
    }
}
