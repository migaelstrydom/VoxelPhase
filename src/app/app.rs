use std::path::Path;
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
use crate::debug::DebugConfig;
use crate::level::{
    create_level_materials, create_level_terrain, create_level_water, load_level,
    spawn_level_objects,
};
use crate::projectile::{build_grenade_model, GrenadeMaterials, GrenadeModelResource};
use crate::rendering::material::{Emission, Material, MaterialManagerBuilder, SurfaceFinish};
use crate::rendering::renderer::Renderer;
use crate::rendering::Colour;
use crate::resources::manager::ResourceManager;
use crate::systems::FrameStart;
use crate::time::Time;

use super::dispatcher_builder::build_dispatcher;
use super::event_handler::{clear_frame_state, set_mouse_captured, EventHandler, EventResult};
use super::spawners::spawn_camera;
use super::world_builder::WorldBuilder;

pub struct App<'a, 'b> {
    event_loop: Option<EventLoop<()>>,
    window: Window,
    world: World,
    dispatcher: Dispatcher<'a, 'b>,
}

impl<'a, 'b> App<'a, 'b> {
    pub fn new(
        window_width: u32,
        window_height: u32,
        app_title: &str,
        level_path: &Path,
    ) -> EngineResult<Self> {
        let (event_loop, window) = Self::create_window(window_width, window_height, app_title)?;
        let (_vulkan_context, renderer, resource_manager, texture_manager) =
            Self::create_rendering_context(&window, window_width, window_height)?;

        // Load the level file
        let level = load_level(level_path)
            .map_err(|e| EngineError::InvalidState(format!("Failed to load level: {}", e)))?;

        // Create materials: grenade, house pool, and level-specific box materials
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

        let grenade_materials = Self::create_grenade_materials(&mut material_builder);

        let level_materials =
            create_level_materials(&level, &texture_manager, &mut material_builder)?;

        let material_manager = material_builder.build(fallback_white);

        let grenade_model = Self::create_grenade_model(&grenade_materials);

        // Generate terrain from level description
        let terrain_manager = create_level_terrain(&level, &texture_manager)?;

        // Create water grids (needs terrain for floor height queries)
        let water_grids = create_level_water(&level, &terrain_manager);

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

        // Insert water grids and wave-body coupler as optional resources
        // (WaterSystem handles the None case)
        if let Some((flow_grid, wave_grid)) = water_grids {
            world.insert(flow_grid);
            world.insert(wave_grid);
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
        let player_entity = spawn_level_objects(&mut world, &level, &level_materials);
        spawn_camera(&mut world, player_entity, window_width, window_height);

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
                {
                    let show_cpu_ms = world.read_resource::<DebugConfig>().show_cpu_ms;
                    let mut frame_start = world.write_resource::<FrameStart>();
                    frame_start.0 = show_cpu_ms.then(std::time::Instant::now);
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
