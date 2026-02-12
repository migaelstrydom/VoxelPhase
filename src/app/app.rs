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
use crate::physics::ForceField;
use crate::projectile::{build_grenade_model, GrenadeMaterials, GrenadeModelResource};
use crate::rendering::material::{Material, MaterialId, MaterialManagerBuilder};
use crate::rendering::renderer::Renderer;
use crate::rendering::Colour;
use crate::resources::manager::ResourceManager;
use crate::systems::PhysicsResource;
use crate::terrain::{create_test_terrain, TerrainManager};
use crate::time::Time;

use super::dispatcher_builder::build_dispatcher;
use super::event_handler::{clear_frame_state, set_mouse_captured, EventHandler, EventResult};
use super::spawners::{
    create_box_materials, spawn_beach_ball, spawn_box, spawn_camera, spawn_player,
};
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
        let num_beach_balls = 5;
        let num_boxes = 5;
        let (material_manager, grenade_materials, box_materials) =
            Self::create_materials(&texture_manager, num_boxes)?;
        let grenade_model = Self::create_grenade_model(&grenade_materials);
        let body_material = grenade_materials.body;
        let box_half_extents = nalgebra::Vector3::new(0.5, 0.5, 0.5);
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

        for _ in 0..num_beach_balls {
            let pos = nalgebra::Point3::new(
                rand::random::<f32>() * 10.0 - 5.0,
                rand::random::<f32>() * 10.0 - 5.0,
                rand::random::<f32>() * 10.0 - 5.0,
            );
            spawn_beach_ball(&mut world, pos, body_material);
        }

        let box_spacing = box_half_extents.x * 3.0;
        let start_x = -box_spacing * (num_boxes as f32 - 1.0) * 0.5;
        for (i, &material) in box_materials.iter().enumerate() {
            let pos = nalgebra::Point3::new(start_x + i as f32 * box_spacing, 2.0, 3.0);
            spawn_box(&mut world, pos, box_half_extents, material);
        }

        // Register terrain boundary force field
        {
            let terrain_bounds = *world.read_resource::<TerrainManager>().bounds();
            let mut physics = world.write_resource::<PhysicsResource>();
            physics
                .0
                .add_force_field(ForceField::boundary(terrain_bounds, 500.0));
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
        num_boxes: usize,
    ) -> EngineResult<(
        crate::rendering::material::MaterialManager,
        GrenadeMaterials,
        Vec<MaterialId>,
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

        let box_materials =
            create_box_materials(num_boxes, texture_manager, &mut material_builder)?;

        let material_manager = material_builder.build(fallback_white);

        Ok((material_manager, grenade_materials, box_materials))
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
