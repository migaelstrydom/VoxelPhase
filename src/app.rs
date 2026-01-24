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
    Acceleration, CameraComponent, Collider, ContactCandidates, Gravity, IKTargets, ModelInstance,
    MotionState, PelvisTarget, PhysicsBody, Position, Probe, ProbePurpose, Renderable, Rotation,
    SensorSet, Velocity,
};
use crate::core::error::{EngineError, EngineResult};
use crate::core::vulkan_context::VulkanContext;
use crate::debug::{DebugLines, DebugOverlays};
use crate::explosion::{Explosion, ExplosionSystem};
use crate::input::{GameplayActions, InputActionSystem, InputState};
use crate::particles::{
    ParticleConfig, ParticleEmitter, ParticlePool, ParticleSpawnSystem, ParticleUpdateSystem,
};
use crate::player::{Player, PlayerConfig, PlayerState};
use crate::projectile::{
    build_grenade_model, Grenade, GrenadeConfig, GrenadeCooldown, GrenadeMaterials,
    GrenadeModelResource, GrenadeSpawnSystem, Lifetime, LifetimeSystem, Projectile,
    ProjectileCollisionSystem,
};
use crate::rendering::camera::Camera;
use crate::rendering::material::{Material, MaterialManagerBuilder};
use crate::rendering::renderer::Renderer;
use crate::rendering::Colour;
use crate::resources::manager::ResourceManager;
use crate::skeleton::{SpringBipedCharacter, SpringBipedCharacterConfig};
use crate::systems::{
    CameraControlSystem, DynamicTerrainCollisionSystem, GravitySystem, IKTargetSystem,
    MotionPredictionSystem, PenetrationResolutionSystem, PlayerInputSystem, PlayerStateSyncSystem,
    ProceduralAnimationSystem, RenderSystem, SpringBipedCollisionSystem, TerrainCollisionSystem,
    TerrainQuerySystem, TerrainUpdateSystem, VelocityIntegrationSystem,
};
use crate::terrain::{create_test_terrain, TerrainManager};
use crate::time::Time;

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
        let _eye_texture =
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

        // Register grenade material
        let grenade_materials = GrenadeMaterials {
            body: material_builder.register(Material::coloured(Colour::new(0.2, 0.25, 0.2, 1.0))), // Dark green
        };

        // Register landscape material (reserved for future terrain texturing)
        let _landscape_material = material_builder.register(Material::textured(grass_texture));

        // ==========================================
        // Phase 2: Freeze materials - no more registration
        // ==========================================
        let material_manager = material_builder.build(fallback_white);

        // ==========================================
        // Phase 3: Build models using pre-registered material IDs
        // ==========================================
        let player_config = PlayerConfig::default();

        // Build grenade model
        let grenade_config = GrenadeConfig::default();
        let grenade_model = Arc::new(build_grenade_model(
            grenade_config.radius,
            Colour::new(1.0, 0.7, 0.1, 1.0), // Glowing fireball orange
            &grenade_materials,
        ));
        log::info!("Grenade model built");

        // Note: Old OBJ landscape removed - using procedural terrain instead

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
        world.register::<SpringBipedCharacter>();
        world.register::<FollowTarget>();
        // Collision components
        world.register::<Collider>();
        world.register::<MotionState>();
        world.register::<SensorSet>();
        world.register::<ContactCandidates>();
        world.register::<IKTargets>();
        world.register::<PelvisTarget>();
        world.register::<PhysicsBody>();
        world.register::<Grenade>();
        world.register::<Lifetime>();
        world.register::<Projectile>();
        world.register::<Explosion>();
        world.register::<ParticleEmitter>();

        // Insert resources
        world.insert(renderer);
        world.insert(resource_manager);
        world.insert(texture_manager);
        world.insert(material_manager);
        world.insert(Time::new());
        world.insert(InputState::new());
        world.insert(GameplayActions::default());
        world.insert(PlayerConfig::default());
        world.insert(CameraConfig::default());
        world.insert(DebugLines::default());
        world.insert(DebugOverlays::default());
        world.insert(GrenadeConfig::default());
        world.insert(GrenadeModelResource {
            model: Some(grenade_model),
        });
        world.insert(GrenadeCooldown::default());
        world.insert(ParticleConfig::new());
        world.insert(ParticlePool::default());
        // Create procedural terrain
        log::info!("Generating procedural terrain...");
        let terrain_svo = create_test_terrain(64.0, 6); // 64x64x64 world, depth 6
        let terrain_manager = TerrainManager::from_svo(terrain_svo);
        log::info!(
            "Terrain generated: {} triangles in {} mesh leaves",
            terrain_manager.triangle_count(),
            terrain_manager.leaf_count()
        );

        world.insert(terrain_manager);

        // Create player entity with spring biped skeleton (Stage 3 - physics-based)
        // Initial position: slightly above terrain so we can see the legs
        let initial_pos = nalgebra::Point3::new(0.0, -10.0, 0.0);
        let spring_config = SpringBipedCharacterConfig::default();
        let spring_biped = SpringBipedCharacter::new(spring_config.clone(), initial_pos);
        log::info!("Spring biped character created (Stage 3 - physics-based spring legs)");

        let hip_width = spring_config.skeleton.hip_width;
        let leg_length =
            spring_config.skeleton.upper_leg_length + spring_config.skeleton.lower_leg_length;
        let player_sensors = SensorSet::new(vec![
            Probe::ray(
                ProbePurpose::FootLeft,
                Vector3::new(hip_width, 0.0, 0.0),
                Vector3::new(0.0, -1.0, 0.0),
                leg_length * 1.5,
                0.05,
            ),
            Probe::ray(
                ProbePurpose::FootRight,
                Vector3::new(-hip_width, 0.0, 0.0),
                Vector3::new(0.0, -1.0, 0.0),
                leg_length * 1.5,
                0.05,
            ),
            Probe::sphere_sweep(
                ProbePurpose::Wall,
                Vector3::new(0.0, 0.5, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
                1.5,
                0.2,
            ),
        ]);

        let player_entity = world
            .create_entity()
            .with(Player)
            .with(PlayerState::default())
            .with(spring_biped)
            .with(Position(Vector3::new(0.0, -10.0, 0.0))) // Start above terrain
            .with(Velocity(Vector3::zeros()))
            .with(Acceleration(Vector3::zeros()))
            .with(Gravity(player_config.gravity))
            .with(Rotation(0.0))
            .with(Renderable)
            // Collision components
            .with(MotionState::new(initial_pos))
            .with(player_sensors)
            .with(ContactCandidates::default())
            .with(IKTargets::default())
            .with(PelvisTarget {
                target_y: initial_pos.y,
                has_contact: false,
            })
            .with(PhysicsBody {
                restitution: 0.1, // Slight bounce
                friction: 0.8,
            })
            .build();

        // Create camera entity that follows the player
        let initial_aspect_ratio = window_width as f32 / window_height as f32;
        let camera_config = CameraConfig::default();
        let camera = Camera::new(
            nalgebra::Point3::new(0.0, 10.0, camera_config.default_distance),
            nalgebra::Point3::new(0.0, 0.0, 0.0),
            nalgebra::Vector3::y(),
            std::f32::consts::FRAC_PI_4,
            initial_aspect_ratio,
            0.1,
            500.0, // Extended far plane to see terrain
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
        // Pipeline: Input -> TerrainQuery -> IK -> Animation -> PlayerInput -> Forces -> Velocity -> Prediction -> Collision -> Camera -> Render
        //
        // Key insight: Terrain probes and IK targets inform WHERE we want to go.
        // Animation computes target pelvis height from feet. Then collision prevents terrain penetration.
        let dispatcher = DispatcherBuilder::new()
            // === Phase 0: Input processing ===
            // Convert raw input to gameplay actions
            .with(InputActionSystem, "input_actions", &[])
            // === Phase 1: Sensing (from last frame's resolved position) ===
            // 1. Terrain probes query ground below entity
            .with(TerrainQuerySystem, "terrain_query", &["input_actions"])
            // 2. IK target selection from probe contacts
            .with(IKTargetSystem, "ik_targets", &["terrain_query"])
            // === Phase 2: Animation (compute desired position) ===
            // 3. Procedural animation: position feet from IK, compute pelvis target
            .with(
                ProceduralAnimationSystem,
                "procedural_animation",
                &["ik_targets"],
            )
            // === Phase 3: Intent (player input and forces) ===
            // 4. Player input (uses grounded state from last frame's collision)
            .with(PlayerInputSystem, "player_input", &["procedural_animation"])
            // 5. Apply gravity/forces
            .with(GravitySystem, "gravity", &["player_input"])
            // === Phase 4: Physics integration ===
            // 6. Velocity integration (velocity only, no position)
            .with(
                VelocityIntegrationSystem,
                "velocity_integration",
                &["gravity"],
            )
            // 7. Motion prediction (compute prev/predicted for CCD)
            .with(
                MotionPredictionSystem,
                "motion_prediction",
                &["velocity_integration"],
            )
            // === Phase 5: Collision resolution (final authority for position) ===
            // 8. SpringBiped CCD collision
            .with(
                SpringBipedCollisionSystem,
                "spring_biped_collision",
                &["motion_prediction"],
            )
            // 9. Dynamic terrain CCD collision (non-biped entities)
            .with(
                DynamicTerrainCollisionSystem,
                "dynamic_terrain_collision",
                &["motion_prediction"],
            )
            // 10. Legacy terrain collision for non-CCD entities
            .with(
                TerrainCollisionSystem,
                "terrain_collision",
                &["spring_biped_collision", "dynamic_terrain_collision"],
            )
            // 11. Penetration resolution (backup for deep overlaps)
            .with(
                PenetrationResolutionSystem,
                "penetration_resolution",
                &["terrain_collision"],
            )
            // === Phase 6: Post-collision state sync ===
            // 12. Sync player state to rendering components
            .with(
                PlayerStateSyncSystem,
                "player_state_sync",
                &["penetration_resolution"],
            )
            // 13. Camera follows player
            .with(
                CameraControlSystem,
                "camera_control",
                &["player_state_sync"],
            )
            // === Phase 7: Projectiles and effects ===
            .with(GrenadeSpawnSystem, "grenade_spawn", &["camera_control"])
            .with(LifetimeSystem, "lifetime", &["grenade_spawn"])
            .with(
                ProjectileCollisionSystem,
                "projectile_collision",
                &["lifetime"],
            )
            .with(ExplosionSystem, "explosion", &["projectile_collision"])
            .with(TerrainUpdateSystem, "terrain_update", &["explosion"])
            .with(ParticleSpawnSystem, "particle_spawn", &["explosion"])
            .with(ParticleUpdateSystem, "particle_update", &["particle_spawn"])
            // === Phase 8: Rendering ===
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

                    // Run all systems (input state from events is still valid)
                    dispatcher.dispatch(world);
                    dispatcher.dispatch_thread_local(world);
                    world.maintain();

                    // Clear per-frame state AFTER systems have processed it
                    {
                        let mut input = world.write_resource::<InputState>();
                        input.begin_frame();
                    }
                    {
                        let mut debug = world.write_resource::<DebugLines>();
                        debug.clear();
                    }
                    {
                        let mut overlays = world.write_resource::<DebugOverlays>();
                        overlays.clear();
                    }
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
