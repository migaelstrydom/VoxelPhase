use std::cell::RefCell;
use std::error::Error;

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
use crate::components::{Position, Renderable, Rotation, SpinSpeed}; // Import new components
use crate::core::vulkan_context::VulkanContext;
use crate::rendering::renderer::Renderer;
use crate::systems::SpinningSystem; // Import new system

pub struct App<'a, 'b> {
    // Add lifetimes for Dispatcher
    event_loop: RefCell<EventLoop<()>>,
    window: Window,
    renderer: Renderer,
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

        let vulkan_context = VulkanContext::new(&window)?;
        let renderer = Renderer::new(vulkan_context, &window, window_width, window_height)?;

        // ECS Setup
        let mut world = World::new();
        world.register::<Position>();
        world.register::<Rotation>();
        world.register::<SpinSpeed>();
        world.register::<Renderable>();

        // Create the triangle entity
        world
            .create_entity()
            .with(Position(Vector3::new(0.0, 0.0, 0.0))) // Initial position
            .with(Rotation(0.0)) // Initial rotation
            .with(SpinSpeed(0.01)) // Rotation speed (radians per frame/update)
            .with(Renderable) // Mark as renderable
            .build();

        // Setup dispatcher
        let dispatcher = DispatcherBuilder::new()
            .with(SpinningSystem, "spinning_system", &[]) // Add our spinning system
            .build();

        Ok(Self {
            event_loop: RefCell::new(event_loop),
            window,
            renderer,
            world,
            dispatcher,
        })
    }

    pub fn run<F: FnMut(&mut Renderer, &World)>(
        // Modified signature to pass World
        &mut self,
        mut game_logic_callback: F,
    ) -> Result<(), Box<dyn Error>> {
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
                    self.dispatcher.dispatch(&self.world);
                    self.world.maintain();

                    // Call the game logic/rendering update function
                    game_logic_callback(&mut self.renderer, &self.world); // Pass world
                    self.window.request_redraw();
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
