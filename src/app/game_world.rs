//! The game's world, assembled from a level: everything `App` runs, minus the
//! window it runs in.
//!
//! ```text
//!   Renderer (window or offscreen) ─┐
//!   ResourceManager, TextureManager ┼─▶ GameWorld::assemble ─▶ World + Dispatcher + player
//!   Level ──────────────────────────┘
//! ```
//!
//! Split from `App` so that an offline harness can run the real game — the
//! same systems, the same spawnables, the same frame — against an offscreen
//! renderer, and measure what the game does rather than a model of it.

use std::sync::Arc;

use specs::{Dispatcher, Entity, World};

use crate::core::error::{EngineError, EngineResult};
use crate::level::{
    create_level_materials, create_level_terrain, create_level_water, spawn_level_objects, Level,
};
use crate::projectile::{build_grenade_model, GrenadeMaterials, GrenadeModelResource};
use crate::rendering::material::{Emission, Material, MaterialManagerBuilder, SurfaceFinish};
use crate::rendering::renderer::Renderer;
use crate::rendering::Colour;
use crate::resources::manager::ResourceManager;
use crate::resources::textures::TextureManager;

use super::dispatcher_builder::build_dispatcher;
use super::world_builder::WorldBuilder;

/// A level, spawned and ready to run.
pub struct GameWorld<'a, 'b> {
    pub world: World,
    /// Every system, set up against `world`.
    pub dispatcher: Dispatcher<'a, 'b>,
    /// The player the level spawned, for a camera to follow.
    pub player: Entity,
}

impl<'a, 'b> GameWorld<'a, 'b> {
    /// Build the world a level describes, drawn by `renderer`.
    ///
    /// Spawns no camera: where the view comes from is the caller's business.
    pub fn assemble(
        renderer: Renderer,
        resource_manager: ResourceManager,
        texture_manager: TextureManager,
        level: &Level,
    ) -> EngineResult<Self> {
        // Create materials: grenade, house pool, and level-specific box materials
        let mut material_builder = MaterialManagerBuilder::new();

        let fallback_white = texture_manager
            .create_solid_colour(Colour::WHITE)
            .map_err(|e| EngineError::Mesh {
                path: None,
                reason: format!("Failed to create fallback texture: {}", e),
            })?;

        let grenade_materials = create_grenade_materials(&mut material_builder);

        let level_materials =
            create_level_materials(level, &texture_manager, &mut material_builder)?;

        let material_manager = material_builder.build(fallback_white);

        let grenade_model = create_grenade_model(&grenade_materials);

        // Generate terrain from level description
        let terrain_manager = create_level_terrain(level, &texture_manager)?;

        // Place the level's water over its terrain.
        let water = create_level_water(level, &terrain_manager);

        let mut world = WorldBuilder::new()
            .with_renderer(renderer)
            .with_resource_manager(resource_manager)
            .with_texture_manager(texture_manager)
            .with_material_manager(material_manager)
            .with_terrain(terrain_manager)
            .with_grenade_model(GrenadeModelResource {
                model: Some(grenade_model),
            })
            .with_default_resources()
            .build()?;

        // Insert the water and the wave-body coupler as optional resources
        // (WaterSystem handles the None case)
        if let Some(water) = water {
            world.insert(water);
            world.insert(crate::water::WaveBodyCoupler::new(
                crate::water::WaveCouplingConfig::default(),
            ));
        }

        // Build and set up the dispatcher *before* spawning anything.
        //
        // `setup` registers the storage for every component any system touches,
        // so a new component reaching the world only through a system needs no
        // entry in `WorldBuilder::register_components`. Without this, forgetting
        // that entry is a panic at spawn time that no test catches — it only
        // shows up when the game is launched with the right level.
        //
        // Components that no system reads (spawner-only marker data) still need
        // registering by hand, but that is a much smaller and more obvious set.
        let mut dispatcher = build_dispatcher();
        dispatcher.setup(&mut world);

        // Spawn level objects (player + all objects from the level file)
        let player = spawn_level_objects(&mut world, level, &level_materials);

        Ok(Self {
            world,
            dispatcher,
            player,
        })
    }
}

/// Register the three surfaces of a grenade.
///
/// Authored for a grenade *at rest*: `GrenadeVisualSystem` scales all three
/// emissions together as it heats up, so these are the coolest each surface
/// ever looks. Only the core starts above the bloom threshold
/// (`PostProcessConfig::bloom_threshold`), which is what makes the fissures
/// bleed light while the shell around them stays dark rock.
fn create_grenade_materials(builder: &mut MaterialManagerBuilder) -> GrenadeMaterials {
    GrenadeMaterials {
        crust: builder.register(
            Material::coloured(Colour::rgb(0.11, 0.10, 0.11))
                .with_finish(SurfaceFinish {
                    roughness: 0.30,
                    metallic: 0.85,
                })
                // Barely alight — enough that a resting grenade is not a
                // dead lump, and it has somewhere to go when heated.
                .with_emission(Emission {
                    colour: Colour::rgb(1.0, 0.25, 0.05),
                    strength: 0.06,
                    rim_strength: 0.4,
                    rim_power: 4.0,
                }),
        ),
        ember: builder.register(
            Material::coloured(Colour::rgb(0.55, 0.20, 0.06))
                .with_finish(SurfaceFinish {
                    roughness: 0.55,
                    metallic: 0.2,
                })
                .with_emission(Emission {
                    colour: Colour::rgb(1.0, 0.38, 0.08),
                    strength: 0.85,
                    rim_strength: 0.7,
                    rim_power: 3.0,
                }),
        ),
        core: builder.register(
            Material::coloured(Colour::rgb(1.0, 0.82, 0.45))
                .with_finish(SurfaceFinish::MATTE)
                .with_emission(Emission {
                    colour: Colour::rgb(1.0, 0.55, 0.15),
                    strength: 4.5,
                    rim_strength: 1.8,
                    rim_power: 2.0,
                }),
        ),
    }
}

fn create_grenade_model(grenade_materials: &GrenadeMaterials) -> Arc<crate::model::Model> {
    use crate::projectile::GrenadeConfig;

    let grenade_config = GrenadeConfig::default();
    let grenade_model = Arc::new(build_grenade_model(
        grenade_config.radius,
        Colour::rgb(1.0, 0.82, 0.45),
        grenade_materials,
    ));
    log::info!("Grenade model built");
    grenade_model
}
