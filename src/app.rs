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

use crate::core::vulkan_context::VulkanContext;
use crate::rendering::renderer::Renderer;

pub struct App {
    event_loop: RefCell<EventLoop<()>>,
    window: Window,
    renderer: Renderer, // App owns the Renderer
}

impl App {
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

        // VulkanContext is created first, as Renderer needs it.
        // The original VulkanContext::new took a `&impl HasDisplayHandle` which the window provides.
        let vulkan_context = VulkanContext::new(&window)?;

        // Renderer is created using the VulkanContext and window details.
        let renderer = Renderer::new(vulkan_context, &window, window_width, window_height)?;

        Ok(Self {
            event_loop: RefCell::new(event_loop),
            window,
            renderer,
        })
    }

    // Placeholder for the main application loop
    pub fn run<F: FnMut(&mut Renderer)>(
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
                    // Call the game logic/rendering update function, passing the renderer
                    game_logic_callback(&mut self.renderer);
                    self.window.request_redraw(); // Important for winit's event loop model
                }
                Event::WindowEvent {
                    event: WindowEvent::RedrawRequested,
                    ..
                } => {
                    // This is where rendering would happen in a more typical winit setup.
                    // For now, our `game_logic_callback` handles it when called by `AboutToWait`.
                    // We might move drawing to be explicitly here later.
                }
                _ => (),
            }
        })?;
        Ok(())
    }
}
