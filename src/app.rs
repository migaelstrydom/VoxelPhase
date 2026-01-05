use std::{cell::RefCell, sync::Arc};

use winit::{
    dpi::LogicalSize,
    event::{ElementState, Event, KeyEvent, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    keyboard::{Key, NamedKey},
    platform::run_on_demand::EventLoopExtRunOnDemand, // For run_on_demand
    window::{Window, WindowBuilder},
};

// ECS imports
use nalgebra::Vector3;
use specs::{Builder, Dispatcher, DispatcherBuilder, World, WorldExt};

// Project imports
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::camera::Camera; // For default camera creation
use crate::rendering::renderer::Renderer;
use crate::resources::textures::TextureManager;
use crate::systems::{RenderSystem, SpinningSystem};
use crate::world::geometry::LandscapeLoader;
use crate::{
    components::{CameraComponent, Mesh, Position, Renderable, Rotation, SpinSpeed},
    resources::manager::ResourceManager,
}; // Added Mesh, Vertex
pub struct App<'a, 'b> {
    // Add lifetimes for Dispatcher
    event_loop: RefCell<EventLoop<()>>,
    _window: Window,
    world: World,                   // Add ECS World
    dispatcher: Dispatcher<'a, 'b>, // Add ECS Dispatcher
}

impl<'a, 'b> App<'a, 'b> {
    pub fn new(window_width: u32, window_height: u32, app_title: &str) -> EngineResult<Self> {
        let event_loop = EventLoop::new()
            .map_err(|e| EngineError::Window(format!("Failed to create event loop: {}", e)))?;
        let window = WindowBuilder::new()
            .with_title(app_title)
            .with_inner_size(LogicalSize::new(
                f64::from(window_width),
                f64::from(window_height),
            ))
            .build(&event_loop)
            .map_err(|e| EngineError::Window(format!("Failed to create window: {}", e)))?;

        let vulkan_context = Arc::new(VulkanContext::new(&window)?);
        let renderer = Renderer::new(
            Arc::clone(&vulkan_context),
            &window,
            window_width,
            window_height,
        )
        .map_err(|e| EngineError::InvalidState(format!("Failed to create renderer: {}", e)))?;
        let resource_manager = ResourceManager::new(Arc::clone(&vulkan_context))?;

        // Create texture manager with descriptor manager from renderer
        let descriptor_manager = renderer.descriptor_manager();
        let texture_manager = resource_manager.create_texture_manager(descriptor_manager)?;

        let landscape_loader = LandscapeLoader::new(&texture_manager);

        let world_geometry = landscape_loader
            .load_ripple_obj()
            .map_err(|e| EngineError::Mesh {
                path: Some("ripple.obj".to_string()),
                reason: e.to_string(),
            })?;

        log::info!(
            "World geometry loaded. Texture loaded with id: {}",
            world_geometry.mesh.texture_handles[0].id()
        );

        let mut world = World::new();
        world.register::<Position>();
        world.register::<Rotation>();
        world.register::<SpinSpeed>();
        world.register::<Renderable>();
        world.register::<CameraComponent>();
        world.register::<Mesh>();

        world.insert(renderer);
        world.insert(resource_manager);
        world.insert(texture_manager);

        world
            .create_entity()
            .with(Position(Vector3::new(0.0, 0.0, 0.0)))
            .with(Rotation(0.0))
            .with(SpinSpeed(0.01))
            .with(world_geometry.mesh)
            .with(Renderable)
            .build();

        let initial_aspect_ratio = window_width as f32 / window_height as f32;
        let camera_entity = Camera::new(
            nalgebra::Point3::new(2.0, -90.0, 3.0),
            nalgebra::Point3::new(0.0, 1.0, 1.0),
            nalgebra::Vector3::y(),
            std::f32::consts::FRAC_PI_4,
            initial_aspect_ratio,
            0.1,
            100.0,
        );

        world
            .create_entity()
            .with(CameraComponent(camera_entity))
            .build();

        let mut dispatcher_builder =
            DispatcherBuilder::new().with(SpinningSystem, "spinning_system", &[]);

        dispatcher_builder.add_thread_local(RenderSystem);

        let dispatcher = dispatcher_builder.build();

        Ok(Self {
            event_loop: RefCell::new(event_loop),
            _window: window,
            world,
            dispatcher,
        })
    }

    pub fn run(&mut self) -> EngineResult<()> {
        let run_result = self.event_loop.borrow_mut().run_on_demand(|event, elwt| {
            elwt.set_control_flow(ControlFlow::Poll);

            match event {
                Event::WindowEvent {
                    event: WindowEvent::CloseRequested,
                    ..
                } => {
                    log::info!("Close requested");
                    elwt.exit();
                }
                Event::WindowEvent {
                    event:
                        WindowEvent::KeyboardInput {
                            event:
                                KeyEvent {
                                    logical_key: Key::Named(NamedKey::Escape),
                                    state: ElementState::Pressed,
                                    ..
                                },
                            ..
                        },
                    ..
                } => {
                    log::info!("Escape pressed");
                    elwt.exit();
                }
                Event::AboutToWait => {
                    self.dispatcher.dispatch(&self.world);
                    self.dispatcher.dispatch_thread_local(&self.world);
                    self.world.maintain();
                    // Textures are now automatically cleaned up when handles are dropped!
                }
                Event::WindowEvent {
                    event: WindowEvent::RedrawRequested,
                    ..
                } => {}
                _ => (),
            }
        });

        // Wait for GPU to finish before cleanup
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
