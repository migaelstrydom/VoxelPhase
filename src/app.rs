use std::error::Error;
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
use nalgebra::{Vector2, Vector3};
use specs::{Builder, Dispatcher, DispatcherBuilder, World, WorldExt};

// Project imports
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::camera::Camera; // For default camera creation
use crate::rendering::renderer::Renderer;
use crate::rendering::vertex::Vertex;
use crate::resources::textures::TextureComponent;
use crate::systems::{RenderSystem, SpinningSystem};
use crate::world::geometry::Landscape;
use crate::{
    components::{CameraComponent, Mesh, Position, Renderable, Rotation, SpinSpeed},
    resources::manager::ResourceManager,
}; // Added Mesh, Vertex
use nalgebra::Vector4;
pub struct App<'a, 'b> {
    // Add lifetimes for Dispatcher
    event_loop: RefCell<EventLoop<()>>,
    _window: Window,
    world: World,                   // Add ECS World
    dispatcher: Dispatcher<'a, 'b>, // Add ECS Dispatcher
}

impl<'a, 'b> App<'a, 'b> {
    pub fn new(
        window_width: u32,
        window_height: u32,
        app_title: &str,
    ) -> Result<Self, Box<dyn Error>> {
        let event_loop = EventLoop::new()?;
        let window = WindowBuilder::new()
            .with_title(app_title)
            .with_inner_size(LogicalSize::new(
                f64::from(window_width),
                f64::from(window_height),
            ))
            .build(&event_loop)?;

        let vulkan_context = Arc::new(VulkanContext::new(&window)?);
        let renderer = Renderer::new(
            Arc::clone(&vulkan_context),
            &window,
            window_width,
            window_height,
        )?;
        let resource_manager = ResourceManager::new(Arc::clone(&vulkan_context))?;

        // ECS Setup
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Rotation>();
        world.register::<SpinSpeed>();
        world.register::<Renderable>();
        world.register::<CameraComponent>();
        world.register::<Mesh>();
        world.register::<TextureComponent>();

        // Insert Renderer as a resource
        // This will make it available to RenderSystem
        // Note: Renderer does not impl Default, so insert is the correct way.
        world.insert(renderer);
        world.insert(resource_manager);

        // Define triangle vertices and indices for the Mesh component
        let triangle_vertices = vec![
            Vertex {
                pos: Vector4::new(-1.0, 1.0, 0.0, 1.0),
                color: Vector4::new(0.0, 1.0, 0.0, 1.0),
                tex_coords: Vector2::new(0.0, 0.0),
            },
            Vertex {
                pos: Vector4::new(1.0, 1.0, 0.0, 1.0),
                color: Vector4::new(0.0, 0.0, 1.0, 1.0),
                tex_coords: Vector2::new(1.0, 0.0),
            },
            Vertex {
                pos: Vector4::new(0.0, -1.0, 0.0, 1.0),
                color: Vector4::new(1.0, 0.0, 0.0, 1.0),
                tex_coords: Vector2::new(0.5, 1.0),
            },
        ];
        let triangle_indices = vec![0u32, 1, 2];

        let world_geometry = Landscape::load_ripple_obj()?;

        // Create the triangle entity with a Mesh component
        world
            .create_entity()
            .with(Position(Vector3::new(0.0, 0.0, 0.0))) // Initial position
            .with(Rotation(0.0)) // Initial rotation
            .with(SpinSpeed(0.01)) // Rotation speed
            .with(Mesh {
                // Add Mesh component
                vertices: world_geometry.mesh.vertices,
                indices: world_geometry.mesh.indices,
            })
            .with(Renderable) // Mark as renderable
            .build();

        // Create Camera Entity
        let initial_aspect_ratio = window_width as f32 / window_height as f32;
        let camera_entity = Camera::new(
            nalgebra::Point3::new(2.0, -90.0, 3.0), // Slightly adjusted default position for better view
            nalgebra::Point3::new(0.0, 1.0, 1.0),
            nalgebra::Vector3::y(),
            std::f32::consts::FRAC_PI_4, // Default FOV (45 degrees)
            initial_aspect_ratio,
            0.1,
            100.0,
        );

        world
            .create_entity()
            .with(CameraComponent(camera_entity))
            .build();

        // Setup dispatcher
        // SpinningSystem can run in parallel. RenderSystem likely needs to be thread-local.
        let mut dispatcher_builder =
            DispatcherBuilder::new().with(SpinningSystem, "spinning_system", &[]);

        // Register RenderSystem as thread-local. It will be dispatched separately.
        // Note: Systems added with `with_thread_local` are not given dependencies like normal systems.
        // They run sequentially on the thread that calls `dispatch_thread_local`.
        // If RenderSystem depends on SpinningSystem, that dependency is implicit by calling dispatch then dispatch_thread_local.
        dispatcher_builder.add_thread_local(RenderSystem);

        let dispatcher = dispatcher_builder.build();

        Ok(Self {
            event_loop: RefCell::new(event_loop),
            _window: window,
            world,
            dispatcher,
        })
    }

    pub fn run(&mut self) -> Result<(), Box<dyn Error>> {
        self.event_loop.borrow_mut().run_on_demand(|event, elwt| {
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
                    // Run ECS systems
                    // Dispatch parallel systems first
                    self.dispatcher.dispatch(&self.world);
                    // Then dispatch thread-local systems (like RenderSystem)
                    self.dispatcher.dispatch_thread_local(&self.world);

                    self.world.maintain();
                }
                Event::WindowEvent {
                    event: WindowEvent::RedrawRequested,
                    ..
                } => {
                    // Rendering is handled by the callback for now
                }
                _ => (),
            }
        })?;
        Ok(())
    }
}
