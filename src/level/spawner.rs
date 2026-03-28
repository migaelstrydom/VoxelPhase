//! Spawns a level's terrain and objects into the ECS world.

use nalgebra::Point3;
use specs::World;

use crate::app::spawnables::MaterialCtx;
use crate::collision::AABB;
use crate::core::error::EngineResult;
use crate::level::data::{Level, WaterBody};
use crate::rendering::material::{MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::terrain::svo::SparseVoxelOctree;
use crate::terrain::{generate_terrain, DurabilityConfig, TerrainManager};
use crate::water::{WaterGrid, WaterGridConfig, WaterProperties, WaveGrid, WaveGridConfig};

/// Pre-created materials for all objects in a level.
///
/// Each inner `Vec` corresponds to one `LevelObject`, in order.
pub struct LevelMaterials {
    /// One material set per object, indexed by object position in the level.
    pub per_object: Vec<Vec<MaterialId>>,
}

/// Coarsening factor applied to the voxel size for the flow grid cell size.
const WATER_GRID_SCALE: u32 = 2;

/// Fine-grid cell size for visible ripples (meters).
const WAVE_CELL_SIZE: f32 = 0.5;

/// Pre-scan the level and create all needed materials during init.
///
/// Call this while `MaterialManagerBuilder` is still mutable, before
/// `build()` finalises the material manager.
pub fn create_level_materials(
    level: &Level,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<LevelMaterials> {
    let mut ctx = MaterialCtx {
        textures: texture_manager,
        materials: material_builder,
    };

    let mut per_object = Vec::with_capacity(level.objects.len());
    for obj in &level.objects {
        let spawnable = obj.to_spawnable();
        let mats = spawnable.create_materials(&mut ctx)?;
        per_object.push(mats);
    }

    Ok(LevelMaterials { per_object })
}

/// Create the terrain SVO and TerrainManager from the level description.
pub fn create_level_terrain(
    level: &Level,
    texture_manager: &TextureManager,
) -> EngineResult<TerrainManager> {
    log::info!("Generating terrain for '{}'...", level.name);

    let half_size = level.world_size;
    let depth = level.octree_depth();
    let bounds = AABB::new(
        Point3::new(-half_size, -half_size, -half_size),
        Point3::new(half_size, half_size, half_size),
    );

    let mut svo = SparseVoxelOctree::new(bounds, depth);
    let durability = DurabilityConfig::default();
    generate_terrain(&mut svo, &level.terrain, &durability);

    let terrain_manager = TerrainManager::from_svo(svo, texture_manager)?;
    log::info!(
        "Terrain generated: {} triangles in {} mesh leaves",
        terrain_manager.triangle_count(),
        terrain_manager.leaf_count()
    );

    Ok(terrain_manager)
}

/// Spawn the player and all objects described in the level file.
///
/// Returns the player entity (for camera follow). Terrain and camera are
/// handled separately — this only spawns entities.
pub fn spawn_level_objects(
    world: &mut World,
    level: &Level,
    materials: &LevelMaterials,
) -> specs::Entity {
    let (px, py, pz) = level.player_spawn;
    let player_entity = crate::app::spawners::spawn_player(world, Point3::new(px, py, pz));

    for (obj, mats) in level.objects.iter().zip(&materials.per_object) {
        obj.to_spawnable().spawn(world, mats);
    }

    player_entity
}

/// Create the water grids from the level's water configuration, if present.
///
/// Returns both the coarse flow grid and the fine wave grid. Pool extents are
/// determined by flood-filling from each body's seed point through terrain
/// that is air at the target surface level.
pub fn create_level_water(
    level: &Level,
    terrain: &TerrainManager,
) -> Option<(WaterGrid, WaveGrid)> {
    let water_config = level.water.as_ref()?;
    let properties = WaterProperties::default();

    let half_size = level.world_size;
    let cell_size = level.voxel_size * WATER_GRID_SCALE as f32;
    let origin = nalgebra::Vector3::new(-half_size, 0.0, -half_size);

    let grid_width = (2.0 * half_size / cell_size).ceil() as usize;
    let grid_depth = grid_width;

    let flow_config = WaterGridConfig {
        cell_size,
        dims: (grid_width, grid_depth),
        origin,
        ocean_level: water_config.ocean_level,
    };
    let mut flow_grid = WaterGrid::new(flow_config, &properties);

    for body in &water_config.bodies {
        match body {
            WaterBody::Pool {
                seed,
                surface_level,
            } => {
                crate::water::placer::fill_pool(&mut flow_grid, terrain, *seed, *surface_level);
            }
        }
    }

    let wave_cell_size = WAVE_CELL_SIZE;
    let cells_per_flow_cell = (cell_size / wave_cell_size).round() as usize;
    let wave_dims = (
        grid_width * cells_per_flow_cell,
        grid_depth * cells_per_flow_cell,
    );

    let wave_config = WaveGridConfig {
        cell_size: wave_cell_size,
        dims: wave_dims,
        origin,
        cells_per_flow_cell,
    };
    let wave_grid = WaveGrid::new(wave_config, &properties);

    log::info!(
        "Water grids created: flow={}x{} (cell_size={:.1}), wave={}x{} (cell_size={:.2}), {} bodies",
        grid_width,
        grid_depth,
        cell_size,
        wave_dims.0,
        wave_dims.1,
        wave_cell_size,
        water_config.bodies.len(),
    );

    Some((flow_grid, wave_grid))
}
