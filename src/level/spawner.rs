//! Spawns a level's terrain and objects into the ECS world.

use nalgebra::{Point3, Vector3};
use specs::World;

use crate::app::spawners::{
    create_box_material_for_style, spawn_beach_ball, spawn_box, spawn_house, spawn_player,
    BoxPhysics,
};
use crate::collision::AABB;
use crate::core::error::EngineResult;
use crate::level::data::{BoxStyle, Level, LevelObject, StackItem};
use crate::rendering::material::{MaterialId, MaterialManagerBuilder};
use crate::resources::textures::TextureManager;
use crate::terrain::svo::SparseVoxelOctree;
use crate::terrain::{generate_terrain, DurabilityConfig, TerrainManager};

/// Materials needed to spawn level objects.
pub struct LevelMaterials {
    /// Material used for beach balls (shared, coloured).
    pub beach_ball_material: MaterialId,
    /// Pre-created box materials, one per box object in the level.
    /// Indexed in the same order as box-producing objects appear in the level.
    pub box_materials: Vec<MaterialId>,
    /// House materials pool (passed directly to `spawn_house`).
    pub house_materials: Vec<MaterialId>,
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

    for obj in &level.objects {
        collect_box_materials(obj, texture_manager, material_builder, &mut box_materials)?;
    }

    Ok(LevelMaterials {
        beach_ball_material,
        box_materials,
        house_materials,
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
                    StackItem::BeachBall => {}
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

        LevelObject::House { .. } => {
            // House uses house_materials pool, no extra materials needed.
        }
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

    for obj in &level.objects {
        spawn_object(world, obj, materials, &mut box_mat_idx);
    }

    player_entity
}

/// Spawn a single level object, advancing `box_mat_idx` for each box consumed.
fn spawn_object(
    world: &mut World,
    obj: &LevelObject,
    materials: &LevelMaterials,
    box_mat_idx: &mut usize,
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
                density: 20.0,
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
    }
}

/// Consume the next pre-created box material.
fn next_box_material(materials: &LevelMaterials, idx: &mut usize) -> MaterialId {
    let mat = materials.box_materials[*idx];
    *idx += 1;
    mat
}
