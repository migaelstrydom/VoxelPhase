use std::sync::Arc;

use winit::{
    dpi::LogicalSize,
    event::{DeviceEvent, ElementState, Event, KeyEvent, MouseButton, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{CursorGrabMode, Window, WindowBuilder},
};

use nalgebra::Vector3;
use specs::{Builder, Dispatcher, DispatcherBuilder, World, WorldExt};

use crate::camera::{CameraConfig, FollowTarget};
use crate::components::{
    Acceleration, CameraComponent, Gravity, Mesh, Position, Renderable, Rotation, Velocity,
};
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::input::InputState;
use crate::player::{Player, PlayerConfig, PlayerState};
use crate::rendering::camera::Camera;
use crate::rendering::renderer::Renderer;
use crate::resources::manager::ResourceManager;
use crate::systems::{
    CameraControlSystem, GravitySystem, PhysicsSystem, PlayerInputSystem, RenderSystem,
};
use crate::time::Time;
use crate::world::geometry::LandscapeLoader;

pub struct App<'a, 'b> {
    event_loop: Option<EventLoop<()>>,
    window: Window,
    world: World,
    dispatcher: Dispatcher<'a, 'b>,
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

        // Set up Vulkan rendering
        let vulkan_context = Arc::new(VulkanContext::new(&window)?);
        let renderer = Renderer::new(
            Arc::clone(&vulkan_context),
            &window,
            window_width,
            window_height,
        )
        .map_err(|e| EngineError::InvalidState(format!("Failed to create renderer: {}", e)))?;

        let resource_manager = ResourceManager::new(Arc::clone(&vulkan_context))?;
        let descriptor_manager = renderer.descriptor_manager();
        let texture_manager = resource_manager.create_texture_manager(descriptor_manager)?;

        // Load world geometry
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

        // Initialize ECS world
        let mut world = World::new();

        // Register all components
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Acceleration>();
        world.register::<Gravity>();
        world.register::<Rotation>();
        world.register::<Mesh>();
        world.register::<Renderable>();
        world.register::<CameraComponent>();
        world.register::<Player>();
        world.register::<PlayerState>();
        world.register::<FollowTarget>();

        // Insert resources
        world.insert(renderer);
        world.insert(resource_manager);
        world.insert(texture_manager);
        world.insert(Time::new());
        world.insert(InputState::new());
        world.insert(PlayerConfig::default());
        world.insert(CameraConfig::default());

        // Create landscape entity (static, just for rendering)
        world
            .create_entity()
            .with(Position(Vector3::new(0.0, -5.0, 0.0)))
            .with(Rotation(0.0))
            .with(world_geometry.mesh)
            .with(Renderable)
            .build();

        // Create player entity
        let player_config = PlayerConfig::default();
        let player_entity = world
            .create_entity()
            .with(Player)
            .with(PlayerState::default())
            .with(Position(Vector3::new(0.0, 5.0, 0.0)))
            .with(Velocity(Vector3::zeros()))
            .with(Acceleration(Vector3::zeros()))
            .with(Gravity(player_config.gravity))
            .with(Rotation(0.0))
            .build();

        // Create camera entity that follows the player
        let initial_aspect_ratio = window_width as f32 / window_height as f32;
        let camera_config = CameraConfig::default();
        let camera = Camera::new(
            nalgebra::Point3::new(0.0, 10.0, camera_config.default_distance),
            nalgebra::Point3::new(0.0, 50.0, 0.0),
            nalgebra::Vector3::y(),
            std::f32::consts::FRAC_PI_4,
            initial_aspect_ratio,
            0.1,
            100.0,
        );

        world
            .create_entity()
            .with(CameraComponent(camera))
            .with(FollowTarget::new(
                player_entity,
                camera_config.default_distance,
            ))
            .build();

        // Build the dispatcher with systems in the correct order
        let dispatcher = DispatcherBuilder::new()
            // Input processing (reads InputState, writes Velocity)
            .with(PlayerInputSystem, "player_input", &[])
            // Physics simulation
            .with(GravitySystem, "gravity", &[])
            .with(PhysicsSystem, "physics", &["player_input", "gravity"])
            // Camera follows player (after physics updates position)
            .with(CameraControlSystem, "camera_control", &["physics"])
            // Rendering is thread-local (must be last)
            .with_thread_local(RenderSystem)
            .build();

        Ok(Self {
            event_loop: Some(event_loop),
            window,
            world,
            dispatcher,
        })
    }

    pub fn run(&mut self) -> EngineResult<()> {
        // Take ownership of the event loop
        let event_loop = self
            .event_loop
            .take()
            .ok_or_else(|| EngineError::InvalidState("Event loop already consumed".to_string()))?;

        // Capture the mouse for camera control
        set_mouse_captured(&self.window, &mut self.world, true);

        let window = &self.window;
        let world = &mut self.world;
        let dispatcher = &mut self.dispatcher;

        let run_result = event_loop.run(|event, elwt| {
            elwt.set_control_flow(ControlFlow::Poll);

            match event {
                // Window close
                Event::WindowEvent {
                    event: WindowEvent::CloseRequested,
                    ..
                } => {
                    log::info!("Close requested");
                    elwt.exit();
                }

                // Keyboard input (for game controls)
                Event::WindowEvent {
                    event:
                        WindowEvent::KeyboardInput {
                            event:
                                KeyEvent {
                                    physical_key: PhysicalKey::Code(key_code),
                                    state,
                                    ..
                                },
                            ..
                        },
                    ..
                } => {
                    // Handle escape to release mouse / exit
                    if key_code == KeyCode::Escape && state == ElementState::Pressed {
                        let input = world.read_resource::<InputState>();
                        let is_captured = input.is_mouse_captured();
                        drop(input);

                        if is_captured {
                            // First escape releases mouse
                            set_mouse_captured(window, world, false);
                        } else {
                            // Second escape exits
                            log::info!("Escape pressed - exiting");
                            elwt.exit();
                        }
                        return;
                    }

                    // Forward to input system
                    let mut input = world.write_resource::<InputState>();
                    input.handle_keyboard_input(key_code, state);
                }

                // Mouse button input
                Event::WindowEvent {
                    event: WindowEvent::MouseInput { state, button, .. },
                    ..
                } => {
                    // Click to recapture mouse if released
                    if button == MouseButton::Left && state == ElementState::Pressed {
                        let input = world.read_resource::<InputState>();
                        let is_captured = input.is_mouse_captured();
                        drop(input);

                        if !is_captured {
                            set_mouse_captured(window, world, true);
                            return;
                        }
                    }

                    let mut input = world.write_resource::<InputState>();
                    input.handle_mouse_button(button, state);
                }

                // Mouse motion (device event for raw/relative motion)
                Event::DeviceEvent {
                    event: DeviceEvent::MouseMotion { delta },
                    ..
                } => {
                    let mut input = world.write_resource::<InputState>();
                    if input.is_mouse_captured() {
                        input.handle_mouse_motion(delta.0, delta.1);
                    }
                }

                // Main game loop tick
                Event::AboutToWait => {
                    // Update timing
                    {
                        let mut time = world.write_resource::<Time>();
                        time.update();
                    }

                    // Prepare input for this frame
                    {
                        let mut input = world.write_resource::<InputState>();
                        input.begin_frame();
                    }

                    // Run all systems
                    dispatcher.dispatch(world);
                    dispatcher.dispatch_thread_local(world);
                    world.maintain();
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

/// Helper function to set mouse capture state
fn set_mouse_captured(window: &Window, world: &mut World, captured: bool) {
    let result = if captured {
        window
            .set_cursor_grab(CursorGrabMode::Confined)
            .or_else(|_| window.set_cursor_grab(CursorGrabMode::Locked))
    } else {
        window.set_cursor_grab(CursorGrabMode::None)
    };

    if let Err(e) = result {
        log::warn!("Failed to set cursor grab mode: {}", e);
    }

    window.set_cursor_visible(!captured);

    let mut input = world.write_resource::<InputState>();
    input.set_mouse_captured(captured);
}
