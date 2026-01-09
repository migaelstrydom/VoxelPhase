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
    Acceleration, CameraComponent, Gravity, ModelInstance, Position, Renderable, Rotation, Velocity,
};
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::input::InputState;
use crate::player::{
    build_player_model, Player, PlayerAnimationState, PlayerConfig, PlayerMaterials,
    PlayerModelConfig, PlayerState,
};
use crate::rendering::camera::Camera;
use crate::rendering::material::{Material, MaterialManagerBuilder};
use crate::rendering::renderer::Renderer;
use crate::rendering::Colour;
use crate::resources::manager::ResourceManager;
use crate::systems::{
    CameraControlSystem, GravitySystem, PhysicsSystem, PlayerAnimationSystem, PlayerInputSystem,
    PlayerStateSyncSystem, RenderSystem,
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

        // ==========================================
        // Phase 1: Register all materials
        // ==========================================
        let mut material_builder = MaterialManagerBuilder::new();

        // Load textures needed for materials
        let eye_texture =
            texture_manager
                .load_texture("data/eye.bmp")
                .map_err(|e| EngineError::Mesh {
                    path: Some("eye.bmp".to_string()),
                    reason: format!("Failed to load eye texture: {}", e),
                })?;

        let grass_texture = texture_manager
            .load_texture("data/grass.bmp")
            .map_err(|e| EngineError::Mesh {
                path: Some("grass.bmp".to_string()),
                reason: format!("Failed to load grass texture: {}", e),
            })?;

        // Create fallback white texture for untextured materials
        let fallback_white = texture_manager
            .create_solid_colour(Colour::WHITE)
            .map_err(|e| EngineError::Mesh {
                path: None,
                reason: format!("Failed to create fallback texture: {}", e),
            })?;

        // Register player materials
        let player_materials = PlayerMaterials {
            body: material_builder.register(Material::coloured(Colour::RED)),
            nose: material_builder.register(Material::coloured(Colour::GREEN)),
            eye: material_builder.register(Material::textured(eye_texture)),
        };

        // Register landscape material
        let landscape_material = material_builder.register(Material::textured(grass_texture));

        // ==========================================
        // Phase 2: Freeze materials - no more registration
        // ==========================================
        let material_manager = material_builder.build(fallback_white);

        // ==========================================
        // Phase 3: Build models using pre-registered material IDs
        // ==========================================
        let player_config = PlayerConfig::default();
        let player_model_config = PlayerModelConfig {
            body_radius: player_config.radius,
            ..Default::default() // Use default colours (RED body, GREEN nose, WHITE eyes)
        };
        let player_model = Arc::new(build_player_model(&player_model_config, &player_materials));
        log::info!("Player model built with {} parts", player_model.parts.len());

        // Load landscape model
        let landscape_loader = LandscapeLoader::textured(landscape_material);
        let landscape_model =
            Arc::new(
                landscape_loader
                    .load_ripple_obj()
                    .map_err(|e| EngineError::Mesh {
                        path: Some("ripple.obj".to_string()),
                        reason: e.to_string(),
                    })?,
            );
        log::info!(
            "Landscape model loaded with {} parts",
            landscape_model.parts.len()
        );

        // Initialize ECS world
        let mut world = World::new();

        // Register all components
        world.register::<Position>();
        world.register::<Velocity>();
        world.register::<Acceleration>();
        world.register::<Gravity>();
        world.register::<Rotation>();
        world.register::<ModelInstance>();
        world.register::<Renderable>();
        world.register::<CameraComponent>();
        world.register::<Player>();
        world.register::<PlayerState>();
        world.register::<PlayerAnimationState>();
        world.register::<FollowTarget>();

        // Insert resources
        world.insert(renderer);
        world.insert(resource_manager);
        world.insert(texture_manager);
        world.insert(material_manager);
        world.insert(Time::new());
        world.insert(InputState::new());
        world.insert(PlayerConfig::default());
        world.insert(CameraConfig::default());

        // Create landscape entity (static, just for rendering)
        world
            .create_entity()
            .with(Position(Vector3::new(0.0, -5.0, 0.0)))
            .with(Rotation(0.0))
            .with(ModelInstance::new(landscape_model))
            .with(Renderable)
            .build();

        // Create player entity with new model architecture
        // Single entity with body, nose, and eyes as model parts
        let player_entity = world
            .create_entity()
            .with(Player)
            .with(PlayerState::default())
            .with(PlayerAnimationState::default())
            .with(Position(Vector3::new(0.0, 5.0, 0.0)))
            .with(Velocity(Vector3::zeros()))
            .with(Acceleration(Vector3::zeros()))
            .with(Gravity(player_config.gravity))
            .with(Rotation(0.0))
            .with(ModelInstance::new(player_model))
            .with(Renderable)
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
            // Sync player state to rendering components
            .with(
                PlayerStateSyncSystem,
                "player_state_sync",
                &["player_input"],
            )
            // Physics simulation
            .with(GravitySystem, "gravity", &[])
            .with(PhysicsSystem, "physics", &["player_input", "gravity"])
            // Player animation (breathing, blinking, etc.)
            .with(PlayerAnimationSystem, "player_animation", &[])
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
