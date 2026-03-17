//! Spawns a level's terrain and objects into the ECS world.

use nalgebra::{Point3, Vector3};
use specs::World;

use crate::app::spawners::{
    create_box_material_for_style, create_capsule_material, spawn_beach_ball, spawn_box,
    spawn_capsule, spawn_house, spawn_player, BoxPhysics, CapsulePhysics,
};
use crate::collision::AABB;
use crate::core::error::EngineResult;
use crate::level::data::{BoxStyle, Level, LevelObject, StackItem, WaterBody};
use crate::rendering::material::{MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::terrain::svo::SparseVoxelOctree;
use crate::terrain::{generate_terrain, DurabilityConfig, TerrainManager};
use crate::water::{WaterGrid, WaterGridConfig, WaveGrid, WaveGridConfig};

/// Materials needed to spawn level objects.
pub struct LevelMaterials {
    /// Material used for beach balls (shared, coloured).
    pub beach_ball_material: MaterialId,
    /// Pre-created box materials, one per box object in the level.
    /// Indexed in the same order as box-producing objects appear in the level.
    pub box_materials: Vec<MaterialId>,
    /// House materials pool (passed directly to `spawn_house`).
    pub house_materials: Vec<MaterialId>,
    /// Pre-created capsule materials, one per capsule in the level.
    pub capsule_materials: Vec<MaterialId>,
}

struct WaterRuntimeConfig {
    /// Coarsening factor applied to the voxel size for the flow grid.
    grid_scale: u32,
    /// Fluid density reserved for water-object interactions.
    fluid_density: f32,
    /// Equalization rate for the coarse flow simulation.
    flow_rate: f32,
    /// Fine-grid cell size for visible ripples.
    wave_cell_size: f32,
    /// Ripple propagation speed in meters per second.
    wave_speed: f32,
    /// Ripple damping in inverse seconds.
    wave_damping: f32,
}

impl Default for WaterRuntimeConfig {
    fn default() -> Self {
        Self {
            grid_scale: 2,
            fluid_density: 1000.0,
            flow_rate: 30.0,
            wave_cell_size: 0.5,
            wave_speed: 4.0,
            wave_damping: 20.0,
        }
    }
}

/// Pre-scan the level and create all needed box materials during init.
///
/// Call this while `MaterialManagerBuilder` is still mutable, before
/// `build()` finalises the material manager.
pub fn create_level_materials(
    level: &Level,
    beach_ball_material: MaterialId,
    house_materials: Vec<MaterialId>,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<LevelMaterials> {
    let mut box_materials = Vec::new();
    let mut capsule_materials = Vec::new();

    for obj in &level.objects {
        collect_box_materials(obj, texture_manager, material_builder, &mut box_materials)?;
        collect_capsule_materials(
            obj,
            texture_manager,
            material_builder,
            &mut capsule_materials,
        )?;
    }

    Ok(LevelMaterials {
        beach_ball_material,
        box_materials,
        house_materials,
        capsule_materials,
    })
}

/// Recursively collect box materials for an object (handles Stack/Tower/BoxWall).
fn collect_box_materials(
    obj: &LevelObject,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
    out: &mut Vec<MaterialId>,
) -> EngineResult<()> {
    match obj {
        LevelObject::BeachBall { .. } => {}

        LevelObject::Box { style, .. } => {
            out.push(create_box_material_for_style(
                *style,
                texture_manager,
                material_builder,
            )?);
        }

        LevelObject::Crate { .. } => {
            out.push(create_box_material_for_style(
                BoxStyle::WoodenCrate,
                texture_manager,
                material_builder,
            )?);
        }

        LevelObject::HeavyCrate { .. } => {
            out.push(create_box_material_for_style(
                BoxStyle::Metal,
                texture_manager,
                material_builder,
            )?);
        }

        LevelObject::Plank { .. } => {
            out.push(create_box_material_for_style(
                BoxStyle::WoodenCrate,
                texture_manager,
                material_builder,
            )?);
        }

        LevelObject::Stack { items, .. } => {
            for item in items {
                match item {
                    StackItem::Crate { .. } => {
                        out.push(create_box_material_for_style(
                            BoxStyle::WoodenCrate,
                            texture_manager,
                            material_builder,
                        )?);
                    }
                    StackItem::HeavyCrate { .. } => {
                        out.push(create_box_material_for_style(
                            BoxStyle::Metal,
                            texture_manager,
                            material_builder,
                        )?);
                    }
                    StackItem::Plank { .. } => {
                        out.push(create_box_material_for_style(
                            BoxStyle::WoodenCrate,
                            texture_manager,
                            material_builder,
                        )?);
                    }
                    StackItem::BeachBall | StackItem::Capsule { .. } => {}
                }
            }
        }

        LevelObject::Tower { count, .. } => {
            for _ in 0..*count {
                out.push(create_box_material_for_style(
                    BoxStyle::Random,
                    texture_manager,
                    material_builder,
                )?);
            }
        }

        LevelObject::BoxWall { columns, rows, .. } => {
            for _ in 0..(*columns * *rows) {
                out.push(create_box_material_for_style(
                    BoxStyle::Random,
                    texture_manager,
                    material_builder,
                )?);
            }
        }

        LevelObject::House { .. } | LevelObject::Capsule { .. } => {}
    }
    Ok(())
}

/// Collect capsule materials for an object.
fn collect_capsule_materials(
    obj: &LevelObject,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
    out: &mut Vec<MaterialId>,
) -> EngineResult<()> {
    match obj {
        LevelObject::Capsule { .. } => {
            out.push(create_capsule_material(texture_manager, material_builder)?);
        }
        LevelObject::Stack { items, .. } => {
            for item in items {
                if let StackItem::Capsule { .. } = item {
                    out.push(create_capsule_material(texture_manager, material_builder)?);
                }
            }
        }
        _ => {}
    }
    Ok(())
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
    let player_entity = spawn_player(world, Point3::new(px, py, pz));

    let mut box_mat_idx = 0;
    let mut capsule_mat_idx = 0;

    for obj in &level.objects {
        spawn_object(
            world,
            obj,
            materials,
            &mut box_mat_idx,
            &mut capsule_mat_idx,
        );
    }

    player_entity
}

/// Spawn a single level object, advancing `box_mat_idx` for each box consumed.
fn spawn_object(
    world: &mut World,
    obj: &LevelObject,
    materials: &LevelMaterials,
    box_mat_idx: &mut usize,
    capsule_mat_idx: &mut usize,
) {
    match obj {
        LevelObject::BeachBall { pos } => {
            spawn_beach_ball(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                materials.beach_ball_material,
            );
        }

        LevelObject::Box {
            pos,
            half_extents,
            density,
            restitution,
            friction,
            ..
        } => {
            let mat = next_box_material(materials, box_mat_idx);
            let phys = BoxPhysics {
                density: *density,
                restitution: *restitution,
                friction: *friction,
            };
            spawn_box(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                Vector3::new(half_extents.0, half_extents.1, half_extents.2),
                mat,
                &phys,
            );
        }

        LevelObject::Crate { pos, size } => {
            let mat = next_box_material(materials, box_mat_idx);
            let phys = BoxPhysics {
                density: 50.0,
                ..Default::default()
            };
            spawn_box(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                Vector3::new(*size, *size, *size),
                mat,
                &phys,
            );
        }

        LevelObject::HeavyCrate { pos, size } => {
            let mat = next_box_material(materials, box_mat_idx);
            let phys = BoxPhysics {
                density: 150.0,
                ..Default::default()
            };
            spawn_box(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                Vector3::new(*size, *size, *size),
                mat,
                &phys,
            );
        }

        LevelObject::Plank { pos, length, width } => {
            let mat = next_box_material(materials, box_mat_idx);
            let phys = BoxPhysics {
                density: 500.0,
                ..Default::default()
            };
            spawn_box(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                Vector3::new(*length * 0.5, 0.3, *width * 0.5),
                mat,
                &phys,
            );
        }

        LevelObject::Stack { base, items } => {
            let mut y = base.1;
            for item in items {
                match item {
                    StackItem::Crate { size } => {
                        let half = *size;
                        y += half; // move up by half-extent before placing
                        let mat = next_box_material(materials, box_mat_idx);
                        let phys = BoxPhysics {
                            density: 50.0,
                            ..Default::default()
                        };
                        spawn_box(
                            world,
                            Point3::new(base.0, y, base.2),
                            Vector3::new(half, half, half),
                            mat,
                            &phys,
                        );
                        y += half; // move past top of this box
                    }
                    StackItem::HeavyCrate { size } => {
                        let half = *size;
                        y += half;
                        let mat = next_box_material(materials, box_mat_idx);
                        let phys = BoxPhysics {
                            density: 150.0,
                            ..Default::default()
                        };
                        spawn_box(
                            world,
                            Point3::new(base.0, y, base.2),
                            Vector3::new(half, half, half),
                            mat,
                            &phys,
                        );
                        y += half;
                    }
                    StackItem::Plank { length, width } => {
                        let hy = 0.1;
                        y += hy;
                        let mat = next_box_material(materials, box_mat_idx);
                        let phys = BoxPhysics {
                            density: 20.0,
                            ..Default::default()
                        };
                        spawn_box(
                            world,
                            Point3::new(base.0, y, base.2),
                            Vector3::new(*length * 0.5, hy, *width * 0.5),
                            mat,
                            &phys,
                        );
                        y += hy;
                    }
                    StackItem::BeachBall => {
                        let radius = 0.5;
                        y += radius;
                        spawn_beach_ball(
                            world,
                            Point3::new(base.0, y, base.2),
                            materials.beach_ball_material,
                        );
                        y += radius;
                    }
                    StackItem::Capsule {
                        half_height,
                        radius,
                    } => {
                        y += *half_height;
                        let mat = next_capsule_material(materials, capsule_mat_idx);
                        spawn_capsule(
                            world,
                            Point3::new(base.0, y, base.2),
                            *half_height,
                            *radius,
                            mat,
                            &CapsulePhysics::default(),
                        );
                        y += *half_height;
                    }
                }
            }
        }

        LevelObject::Tower {
            base,
            box_half_extents,
            count,
            density,
        } => {
            let he = Vector3::new(box_half_extents.0, box_half_extents.1, box_half_extents.2);
            let phys = BoxPhysics {
                density: *density,
                ..Default::default()
            };
            let mut y = base.1;
            for _ in 0..*count {
                y += he.y;
                let mat = next_box_material(materials, box_mat_idx);
                spawn_box(world, Point3::new(base.0, y, base.2), he, mat, &phys);
                y += he.y;
            }
        }

        LevelObject::BoxWall {
            base,
            box_half_extents,
            columns,
            rows,
            density,
            stagger,
        } => {
            let he = Vector3::new(box_half_extents.0, box_half_extents.1, box_half_extents.2);
            let phys = BoxPhysics {
                density: *density,
                ..Default::default()
            };
            let box_w = he.x * 2.0;
            let box_h = he.y * 2.0;

            for row in 0..*rows {
                let y = base.1 + he.y + row as f32 * box_h;
                let x_offset = if *stagger && row % 2 == 1 { he.x } else { 0.0 };
                let start_x = base.0 - (*columns as f32 - 1.0) * he.x + x_offset;

                for col in 0..*columns {
                    let x = start_x + col as f32 * box_w;
                    let mat = next_box_material(materials, box_mat_idx);
                    spawn_box(world, Point3::new(x, y, base.2), he, mat, &phys);
                }
            }
        }

        LevelObject::House { pos, half_extents } => {
            spawn_house(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                Vector3::new(half_extents.0, half_extents.1, half_extents.2),
                &materials.house_materials,
            );
        }

        LevelObject::Capsule {
            pos,
            half_height,
            radius,
            density,
            restitution,
            friction,
        } => {
            let mat = next_capsule_material(materials, capsule_mat_idx);
            let phys = CapsulePhysics {
                density: *density,
                restitution: *restitution,
                friction: *friction,
            };
            spawn_capsule(
                world,
                Point3::new(pos.0, pos.1, pos.2),
                *half_height,
                *radius,
                mat,
                &phys,
            );
        }
    }
}

/// Create the water grids from the level's water configuration, if present.
///
/// Returns both the coarse flow grid and the fine wave grid. Queries terrain
/// for floor heights at each cell center, then places water bodies (pools,
/// lakes) as described in the level file.
pub fn create_level_water(
    level: &Level,
    terrain: &TerrainManager,
) -> Option<(WaterGrid, WaveGrid)> {
    let water_config = level.water.as_ref()?;
    let runtime_config = WaterRuntimeConfig::default();

    let half_size = level.world_size;
    let cell_size = level.voxel_size * runtime_config.grid_scale as f32;

    // Grid covers the full world XZ extent.
    let grid_width = (2.0 * half_size / cell_size).ceil() as usize;
    let grid_depth = grid_width;

    let config = WaterGridConfig {
        cell_size,
        dims: (grid_width, grid_depth),
        origin: nalgebra::Vector3::new(-half_size, 0.0, -half_size),
        ocean_level: water_config.ocean_level,
        flow_rate: runtime_config.flow_rate,
        fluid_density: runtime_config.fluid_density,
        ..Default::default()
    };

    let mut flow_grid = WaterGrid::new(config);

    // Set floor levels for all cells from terrain.
    for j in 0..grid_depth {
        for i in 0..grid_width {
            let x = flow_grid.cell_center_x(i);
            let z = flow_grid.cell_center_z(j);
            if let Some(h) = terrain.surface_height_at(x, z) {
                flow_grid.cell_mut(i, j).floor_level = h;
            }
        }
    }

    // Place water bodies.
    for body in &water_config.bodies {
        match body {
            WaterBody::Pool {
                center,
                half_extents,
                surface_level,
            } => {
                place_pool(&mut flow_grid, *center, *half_extents, *surface_level);
            }
            WaterBody::Lake {
                seed,
                surface_level,
            } => {
                place_lake(&mut flow_grid, *seed, *surface_level);
            }
        }
    }

    // Create the wave grid at fine resolution.
    let wave_cell_size = runtime_config.wave_cell_size;
    let cells_per_flow_cell = (cell_size / wave_cell_size).round() as usize;
    let wave_dims = (
        grid_width * cells_per_flow_cell,
        grid_depth * cells_per_flow_cell,
    );

    let wave_grid = WaveGrid::new(WaveGridConfig {
        cell_size: wave_cell_size,
        dims: wave_dims,
        origin: nalgebra::Vector3::new(-half_size, 0.0, -half_size),
        wave_speed: runtime_config.wave_speed,
        wave_damping: runtime_config.wave_damping,
        cells_per_flow_cell,
    });

    log::info!(
        "Water grids created: flow={}x{} (cell_size={:.1}, rate={:.1}), wave={}x{} (cell_size={:.2}, speed={:.1}, damping={:.1}), fluid_density={:.1}, {} bodies",
        grid_width,
        grid_depth,
        cell_size,
        runtime_config.flow_rate,
        wave_dims.0,
        wave_dims.1,
        wave_cell_size,
        runtime_config.wave_speed,
        runtime_config.wave_damping,
        runtime_config.fluid_density,
        water_config.bodies.len(),
    );

    Some((flow_grid, wave_grid))
}

/// Fill a rectangular pool region with water up to `surface_level`.
fn place_pool(
    grid: &mut WaterGrid,
    center: (f32, f32),
    half_extents: (f32, f32),
    surface_level: f32,
) {
    let min_x = center.0 - half_extents.0;
    let max_x = center.0 + half_extents.0;
    let min_z = center.1 - half_extents.1;
    let max_z = center.1 + half_extents.1;

    let cell_area = grid.cell_area();
    let dims = grid.dims();

    for j in 0..dims.1 {
        for i in 0..dims.0 {
            let cx = grid.cell_center_x(i);
            let cz = grid.cell_center_z(j);

            if cx >= min_x && cx <= max_x && cz >= min_z && cz <= max_z {
                let floor = grid.cell(i, j).floor_level;
                if floor < surface_level {
                    let depth = surface_level - floor;
                    let volume = depth * cell_area;
                    grid.add_water(i, j, volume, floor);
                }
            }
        }
    }
}

/// Flood-fill from a seed point, filling all connected cells below `surface_level`.
fn place_lake(grid: &mut WaterGrid, seed: (f32, f32), surface_level: f32) {
    let Some((si, sj)) = grid.world_to_grid(seed.0, seed.1) else {
        log::warn!(
            "Lake seed ({:.1}, {:.1}) is outside the water grid",
            seed.0,
            seed.1
        );
        return;
    };

    let dims = grid.dims();
    let cell_area = grid.cell_area();
    let mut visited = vec![false; dims.0 * dims.1];
    let mut queue = std::collections::VecDeque::new();

    queue.push_back((si, sj));
    visited[sj * dims.0 + si] = true;

    while let Some((i, j)) = queue.pop_front() {
        let floor = grid.cell(i, j).floor_level;
        if floor >= surface_level {
            continue;
        }

        let depth = surface_level - floor;
        let volume = depth * cell_area;
        grid.add_water(i, j, volume, floor);

        // Expand to 4-connected neighbors.
        for (di, dj) in [(-1i32, 0), (1, 0), (0, -1i32), (0, 1)] {
            let ni = i as i32 + di;
            let nj = j as i32 + dj;
            if ni >= 0 && nj >= 0 && (ni as usize) < dims.0 && (nj as usize) < dims.1 {
                let ni = ni as usize;
                let nj = nj as usize;
                let idx = nj * dims.0 + ni;
                if !visited[idx] {
                    visited[idx] = true;
                    queue.push_back((ni, nj));
                }
            }
        }
    }
}

/// Consume the next pre-created box material.
fn next_box_material(materials: &LevelMaterials, idx: &mut usize) -> MaterialId {
    let mat = materials.box_materials[*idx];
    *idx += 1;
    mat
}

/// Consume the next pre-created capsule material.
fn next_capsule_material(materials: &LevelMaterials, idx: &mut usize) -> MaterialId {
    let mat = materials.capsule_materials[*idx];
    *idx += 1;
    mat
}
