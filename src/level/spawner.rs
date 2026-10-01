//! Spawns a level's terrain and objects into the ECS world.

use nalgebra::Point3;
use specs::{World, WorldExt};

use crate::app::spawnables::MaterialCtx;
use crate::core::error::EngineResult;
use crate::level::data::{Level, LevelObject};
use crate::level::drop::{body_handles, drop_onto_below, DropOutcome};
use crate::level::placement::{local_frame, PlacementError};
use crate::physics::RigidBodyHandle;
use crate::rendering::material::{MaterialId, MaterialManagerBuilder};
use crate::rendering::pattern::TextureCache;
use crate::resources::textures::TextureManager;
use crate::systems::PhysicsResource;
use crate::terrain::{generate_terrain, Anchor, ChunkGrid, Segment, TerrainWorld};
use crate::water::WaterWorld;

/// Pre-created materials for all objects in a level.
///
/// Each inner `Vec` corresponds to one `LevelObject`, in order.
pub struct LevelMaterials {
    /// One material set per object, indexed by object position in the level.
    pub per_object: Vec<Vec<MaterialId>>,
}

/// Pre-scan the level and create all needed materials during init.
///
/// Call this while `MaterialManagerBuilder` is still mutable, before
/// `build()` finalises the material manager.
pub fn create_level_materials(
    level: &Level,
    texture_manager: &TextureManager,
    material_builder: &mut MaterialManagerBuilder,
) -> EngineResult<LevelMaterials> {
    // One cache for the whole level, so objects that ask for the same surface
    // get the same texture rather than each baking their own.
    let mut textures_cache = TextureCache::new();
    let mut ctx = MaterialCtx {
        textures: texture_manager,
        materials: material_builder,
        textures_cache: &mut textures_cache,
    };

    let mut per_object = Vec::with_capacity(level.object_count());
    for (_, obj) in level.objects() {
        let spawnable = obj.to_spawnable();
        let mats = spawnable.create_materials(&mut ctx)?;
        per_object.push(mats);
    }

    let (baked, shared) = textures_cache.stats();
    log::info!("level textures: {baked} baked, {shared} shared from the cache");

    Ok(LevelMaterials { per_object })
}

/// Generate every segment's terrain and place it at its resolved frame.
///
/// Generation is entirely segment-local — the frame is only ever applied to
/// query results — so the same segment definition produces identical geometry
/// wherever it is placed.
///
/// `level` must have been through [`load_level`], which fills in `frames`.
///
/// [`load_level`]: crate::level::load_level
pub fn build_segments(level: &Level) -> Result<Vec<Segment>, PlacementError> {
    let mut segments = Vec::with_capacity(level.segments.len());

    for (index, def) in level.segments.iter().enumerate() {
        let bounds = def.terrain.bounds.to_aabb();
        let mut grid = ChunkGrid::new(def.terrain.voxel_size);
        generate_terrain(&mut grid, &def.terrain, &bounds);

        let anchors = def
            .anchors
            .iter()
            .map(|a| Ok(Anchor::new(a.name.clone(), local_frame(&def.name, a)?)))
            .collect::<Result<Vec<_>, PlacementError>>()?;

        segments.push(Segment::new(
            def.name.clone(),
            level.frame(index),
            grid,
            anchors,
        ));
    }

    Ok(segments)
}

/// Build the level's `TerrainWorld`: every segment generated, placed and meshed.
pub fn create_level_terrain(
    level: &Level,
    texture_manager: &TextureManager,
) -> EngineResult<TerrainWorld> {
    log::info!(
        "Generating terrain for '{}' ({} segments)...",
        level.name,
        level.segments.len()
    );

    let segments = build_segments(level).map_err(|e| {
        crate::core::error::EngineError::InvalidState(format!("level placement failed: {e}"))
    })?;

    let terrain = TerrainWorld::from_segments(segments, texture_manager)?;
    log::info!(
        "Terrain generated: {} triangles in {} mesh leaves",
        terrain.triangle_count(),
        terrain.leaf_count()
    );

    Ok(terrain)
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
    world.insert(level.debris);
    spawn_objects(world, level, materials);

    // After the objects, so a dropped object cannot land on the player.
    let (px, py, pz) = level.player_spawn;
    crate::app::spawners::spawn_player(world, Point3::new(px, py, pz))
}

/// Spawn the level's objects and nothing else.
///
/// Split out for the tools that want to *look* at a level rather than play it:
/// a viewer has no use for a player, and spawning one would drag in the
/// animation and control chain for something that is never stepped.
///
/// Terrain-anchored objects read `TerrainWorld` out of the world as they spawn,
/// so it has to be in place before this is called.
pub fn spawn_objects(world: &mut World, level: &Level, materials: &LevelMaterials) {
    for entry in spawn_order(level) {
        let mats = &materials.per_object[entry.ordinal];
        let spawned = spawn_object(world, entry.object, mats);
        if let Some(problem) = spawned.drop.as_ref().and_then(DropOutcome::problem) {
            log::warn!(
                "{} #{} in '{}' {problem}",
                entry.object.describe().kind,
                entry.ordinal + 1,
                level.segments[entry.segment].name
            );
        }
    }
}

/// One level object, where it falls in the order objects are spawned.
pub struct SpawnEntry<'a> {
    /// Index in file order across the whole level, the index
    /// [`LevelMaterials::per_object`] and every report use.
    pub ordinal: usize,
    /// Index of the segment the object is authored in.
    pub segment: usize,
    pub object: &'a LevelObject,
}

/// The level's objects in the order they must be spawned: every placed object
/// first, then every dropped one, each group in file order.
///
/// A drop lands on what is already in the world, so whatever it is meant to
/// land on has to be there first. Putting placed objects first means an author
/// never has to order a temple before the dice dropped into it; drops onto
/// drops follow the file.
pub fn spawn_order(level: &Level) -> impl Iterator<Item = SpawnEntry<'_>> {
    let entries = || {
        level
            .objects()
            .enumerate()
            .map(|(ordinal, (segment, object))| SpawnEntry {
                ordinal,
                segment,
                object,
            })
    };
    entries()
        .filter(|e| !e.object.is_dropped())
        .chain(entries().filter(|e| e.object.is_dropped()))
}

/// What spawning one level object put into the world.
pub struct SpawnedObject {
    /// Every body the spawn created, of any kind.
    pub bodies: Vec<RigidBodyHandle>,
    /// How the drop went, for a dropped object.
    pub drop: Option<DropOutcome>,
}

/// Spawn one level object and, if it is dropped, drop it onto what is below.
///
/// The bodies are found by what the spawn added to the physics world, not by
/// the entities it returned, so every body counts whichever entity owns it.
pub fn spawn_object(
    world: &mut World,
    object: &LevelObject,
    materials: &[MaterialId],
) -> SpawnedObject {
    let before = body_handles(&world.read_resource::<PhysicsResource>().world);
    object.to_spawnable().spawn(world, materials);
    let bodies: Vec<_> = {
        let physics = &world.read_resource::<PhysicsResource>().world;
        let mut added: Vec<_> = body_handles(physics)
            .into_iter()
            .filter(|h| !before.contains(h))
            .collect();
        // Arena order: the order they were created in, since nothing is
        // removed while a level loads. The first is the object's own.
        added.sort_by_key(|h| h.0.into_raw_parts());
        added
    };

    let drop = object.is_dropped().then(|| drop_onto_below(world, &bodies));
    SpawnedObject { bodies, drop }
}

/// A level's water, placed over its terrain, if the level has any.
///
/// Pools that cannot be placed are logged and skipped; `level_check` reports
/// them, and any pool authored above where it spills.
pub fn create_level_water(level: &Level, terrain: &TerrainWorld) -> Option<WaterWorld> {
    let config = level.water.as_ref()?;
    let (water, errors) = WaterWorld::from_config(config, terrain);
    for error in errors {
        log::warn!("{error}");
    }
    log::info!(
        "Water placed: {} basins, {:.0} m³",
        water.basins().count(),
        water.volume()
    );
    Some(water)
}
