//! Pure data types for level file deserialization.
//!
//! These structs map directly to the RON level file format. They carry no
//! engine dependencies — only `serde::Deserialize` — so the format can evolve
//! independently of the runtime.

use serde::Deserialize;

use crate::app::spawnables::{
    BarricadeDef, BeachBallDef, BoxDef, BoxWallDef, CapsuleDef, CrateDef, DominoDef,
    HeavyCrateDef, HouseDef, PlankDef, PyramidDef, StackDef, StackItemDef, TableDef,
    TrampolineDef, TowerDef, Spawnable,
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
    /// Barricade — two posts with planks welded across them.
    Barricade {
        pos: (f32, f32, f32),
        /// Number of horizontal planks.
        #[serde(default = "BarricadeDef::default_plank_count")]
        plank_count: u32,
        #[serde(default = "BarricadeDef::default_density")]
        density: f32,
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
                items: items.iter().map(|item| match item {
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
                }).collect(),
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

            LevelObject::Barricade {
                pos,
                plank_count,
                density,
            } => Box::new(BarricadeDef {
                pos: *pos,
                plank_count: *plank_count,
                density: *density,
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
        let parse_result: Result<WaterConfig, _> =
            ron::from_str("(wave_damping: 2.0, bodies: [])");

        assert!(
            parse_result.is_err(),
            "WaterConfig should reject runtime tuning fields from level data"
        );
    }
}
