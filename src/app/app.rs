use std::sync::Arc;

use winit::{
    dpi::LogicalSize,
    event::Event,
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use specs::{Dispatcher, World, WorldExt};

use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::projectile::{build_grenade_model, GrenadeMaterials, GrenadeModelResource};
use crate::rendering::material::{Material, MaterialManagerBuilder};
use crate::rendering::renderer::Renderer;
use crate::rendering::Colour;
use crate::resources::manager::ResourceManager;
use crate::terrain::{create_test_terrain, TerrainManager};
use crate::time::Time;

use super::dispatcher_builder::build_dispatcher;
use super::entity_spawner::{spawn_beach_ball, spawn_camera, spawn_player};
use super::event_handler::{clear_frame_state, set_mouse_captured, EventHandler, EventResult};
use super::world_builder::WorldBuilder;

pub struct App<'a, 'b> {
    event_loop: Option<EventLoop<()>>,
    window: Window,
    world: World,
    dispatcher: Dispatcher<'a, 'b>,
}

impl<'a, 'b> App<'a, 'b> {
    pub fn new(window_width: u32, window_height: u32, app_title: &str) -> EngineResult<Self> {
        let (event_loop, window) = Self::create_window(window_width, window_height, app_title)?;
        let (_vulkan_context, renderer, resource_manager, texture_manager) =
            Self::create_rendering_context(&window, window_width, window_height)?;
        let (material_manager, grenade_materials) = Self::create_materials(&texture_manager)?;
        let grenade_model = Self::create_grenade_model(&grenade_materials);
        let beach_ball_models = Self::create_beach_ball_models(&grenade_materials);
        let terrain_manager = Self::create_terrain(&texture_manager)?;

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

        let player_entity = spawn_player(&mut world, nalgebra::Point3::new(0.0, -10.0, 0.0));
        spawn_camera(&mut world, player_entity, window_width, window_height);

        // Spawn 3 beach balls at different positions
        let beach_ball_positions = vec![
            nalgebra::Point3::new(-3.0, 0.0, 2.0),
            // nalgebra::Point3::new(0.0, 5.0, -3.0),
            // nalgebra::Point3::new(4.0, 2.0, 1.0),
            // nalgebra::Point3::new(0.0, 10.0, 0.0),
            // nalgebra::Point3::new(-1.0, 10.0, 0.0),
        ];

        for (pos, model) in beach_ball_positions
            .into_iter()
            .zip(beach_ball_models.into_iter())
        {
            spawn_beach_ball(&mut world, pos, model);
        }

        let dispatcher = build_dispatcher();

        Ok(Self {
            event_loop: Some(event_loop),
            window,
            world,
            dispatcher,
        })
    }

    fn create_window(
        width: u32,
        height: u32,
        title: &str,
    ) -> EngineResult<(EventLoop<()>, Window)> {
        let event_loop = EventLoop::new()
            .map_err(|e| EngineError::Window(format!("Failed to create event loop: {}", e)))?;

        let window = WindowBuilder::new()
            .with_title(title)
            .with_inner_size(LogicalSize::new(f64::from(width), f64::from(height)))
            .build(&event_loop)
            .map_err(|e| EngineError::Window(format!("Failed to create window: {}", e)))?;

        Ok((event_loop, window))
    }

    fn create_rendering_context(
        window: &Window,
        width: u32,
        height: u32,
    ) -> EngineResult<(
        Arc<VulkanContext>,
        Renderer,
        ResourceManager,
        crate::resources::textures::TextureManager,
    )> {
        let vulkan_context = Arc::new(VulkanContext::new(window)?);

        let renderer = Renderer::new(Arc::clone(&vulkan_context), window, width, height)
            .map_err(|e| EngineError::InvalidState(format!("Failed to create renderer: {}", e)))?;

        let resource_manager = ResourceManager::new(Arc::clone(&vulkan_context))?;
        let descriptor_manager = renderer.descriptor_manager();
        let texture_manager = resource_manager.create_texture_manager(descriptor_manager)?;

        Ok((vulkan_context, renderer, resource_manager, texture_manager))
    }

    fn create_materials(
        texture_manager: &crate::resources::textures::TextureManager,
    ) -> EngineResult<(
        crate::rendering::material::MaterialManager,
        GrenadeMaterials,
    )> {
        let mut material_builder = MaterialManagerBuilder::new();

        let _eye_texture =
            texture_manager
                .load_texture("data/eye.bmp")
                .map_err(|e| EngineError::Mesh {
                    path: Some("eye.bmp".to_string()),
                    reason: format!("Failed to load eye texture: {}", e),
                })?;

        let fallback_white = texture_manager
            .create_solid_colour(Colour::WHITE)
            .map_err(|e| EngineError::Mesh {
                path: None,
                reason: format!("Failed to create fallback texture: {}", e),
            })?;

        let grenade_materials = GrenadeMaterials {
            body: material_builder.register(Material::coloured(Colour::new(0.2, 0.25, 0.2, 1.0))),
        };

        let material_manager = material_builder.build(fallback_white);

        Ok((material_manager, grenade_materials))
    }

    fn create_grenade_model(grenade_materials: &GrenadeMaterials) -> Arc<crate::model::Model> {
        use crate::projectile::GrenadeConfig;

        let grenade_config = GrenadeConfig::default();
        let grenade_model = Arc::new(build_grenade_model(
            grenade_config.radius,
            Colour::new(1.0, 0.7, 0.1, 1.0),
            grenade_materials,
        ));
        log::info!("Grenade model built");
        grenade_model
    }

    fn create_beach_ball_models(
        grenade_materials: &GrenadeMaterials,
    ) -> Vec<Arc<crate::model::Model>> {
        use crate::geometry::{
            generate_magic_sphere_vertices, generate_sphere_indices, MagicSphereConfig,
        };
        use crate::model::{MeshPrimitive, Model, ModelPart};

        let magic_configs = vec![
            // Fiery Phoenix - red/orange swirls with tight spirals
            MagicSphereConfig {
                base_hue: 0.02,
                spiral_frequency: 4.0,
                spiral_tightness: 2.5,
                accent_hue_offset: 0.08,
                color_variation: 0.12,
                glow_intensity: 1.1,
            },
            // Ocean Nebula - deep blue/cyan cosmic swirls
            MagicSphereConfig {
                base_hue: 0.55,
                spiral_frequency: 3.0,
                spiral_tightness: 1.8,
                accent_hue_offset: 0.15,
                color_variation: 0.18,
                glow_intensity: 1.0,
            },
            // Solar Plasma - yellow/gold with energetic patterns
            MagicSphereConfig {
                base_hue: 0.12,
                spiral_frequency: 5.0,
                spiral_tightness: 3.0,
                accent_hue_offset: 0.05,
                color_variation: 0.1,
                glow_intensity: 1.2,
            },
            // Emerald Vortex - green/teal magical swirls
            MagicSphereConfig {
                base_hue: 0.35,
                spiral_frequency: 3.5,
                spiral_tightness: 2.2,
                accent_hue_offset: 0.12,
                color_variation: 0.15,
                glow_intensity: 1.0,
            },
            // Void Crystal - purple/magenta cosmic energy
            MagicSphereConfig {
                base_hue: 0.75,
                spiral_frequency: 4.5,
                spiral_tightness: 2.8,
                accent_hue_offset: 0.2,
                color_variation: 0.2,
                glow_intensity: 1.15,
            },
        ];

        magic_configs
            .into_iter()
            .map(|config| {
                let radius = 0.5;
                let segments = 32;
                let rings = 24;

                let parts = vec![ModelPart::new(vec![MeshPrimitive {
                    vertices: generate_magic_sphere_vertices(radius, segments, rings, &config),
                    indices: generate_sphere_indices(segments, rings),
                    material: grenade_materials.body,
                }])];

                Arc::new(Model::flat(parts))
            })
            .collect()
    }

    fn create_terrain(
        texture_manager: &crate::resources::textures::TextureManager,
    ) -> EngineResult<TerrainManager> {
        log::info!("Generating procedural terrain...");
        let terrain_svo = create_test_terrain(64.0, 6);
        let terrain_manager = TerrainManager::from_svo(terrain_svo, texture_manager)?;
        log::info!(
            "Terrain generated: {} triangles in {} mesh leaves",
            terrain_manager.triangle_count(),
            terrain_manager.leaf_count()
        );
        Ok(terrain_manager)
    }

    pub fn run(&mut self) -> EngineResult<()> {
        let event_loop = self
            .event_loop
            .take()
            .ok_or_else(|| EngineError::InvalidState("Event loop already consumed".to_string()))?;

        set_mouse_captured(&self.window, &mut self.world, true);

        let window = &self.window;
        let world = &mut self.world;
        let dispatcher = &mut self.dispatcher;

        let run_result = event_loop.run(|event, elwt| {
            elwt.set_control_flow(ControlFlow::Poll);

            let is_about_to_wait = matches!(event, Event::AboutToWait);

            match EventHandler::handle(event, window, world, elwt) {
                EventResult::Exit => return,
                EventResult::Continue => {}
            }

            if is_about_to_wait {
                {
                    let mut time = world.write_resource::<Time>();
                    time.update();
                }

                dispatcher.dispatch(world);
                dispatcher.dispatch_thread_local(world);
                world.maintain();

                clear_frame_state(world);
            }
        });

        {
            let renderer = self.world.read_resource::<Renderer>();
            unsafe {
                renderer
                    .vulkan_context
                    .device()
                    .device_wait_idle()
                    .expect("Failed to wait for device idle");
            }
        }

        log::info!("App::run - Event loop finished. App will now drop.");

        run_result.map_err(|e| EngineError::Window(format!("Event loop error: {}", e)))?;
        Ok(())
    }
}
