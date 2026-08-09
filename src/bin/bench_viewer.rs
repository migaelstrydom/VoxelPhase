//! Standalone physics bench scenario viewer.
//!
//! Runs a physics bench scenario with the full Vulkan renderer, allowing
//! visual debugging of physics behaviour. The scenario's static geometry
//! is drawn as wireframe, and rigid bodies are drawn as debug shapes.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin bench_viewer -- <scenario_name>
//! cargo run --bin bench_viewer -- --list
//! ```
//!
//! # Controls
//!
//! - **WASD / Arrow keys**: Orbit camera
//! - **Q / E**: Move camera up / down
//! - **+/-**: Zoom in / out
//! - **Space**: Pause / unpause simulation
//! - **N**: Single-step (while paused)
//! - **R**: Reset scenario
//! - **Escape**: Quit

use std::sync::Arc;

use nalgebra::{Point3, Vector3};
use winit::dpi::LogicalSize;
use winit::event::{ElementState, Event, KeyEvent, WindowEvent};
use winit::event_loop::{ControlFlow, EventLoop, EventLoopWindowTarget};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::WindowBuilder;

use voxel_phase::collision::AABB;
use voxel_phase::core::error::EngineResult;
use voxel_phase::core::vulkan_context::VulkanContext;
use voxel_phase::debug::{DebugLines, DebugOverlays};
use voxel_phase::physics::bench_harness::framework::{BenchRunConfig, PhysicsBenchScenario};
use voxel_phase::physics::bench_harness::scenarios::*;
use voxel_phase::physics::stepping::FixedTimestep;
use voxel_phase::physics::{ColliderShape, PhysicsWorld, StaticGeometry};
use voxel_phase::rendering::camera::Camera;
use voxel_phase::rendering::colour::Colour;
use voxel_phase::rendering::debug_render::{
    render_debug_overlays_opaque, render_debug_overlays_transparent,
};
use voxel_phase::rendering::material::MaterialManagerBuilder;
use voxel_phase::rendering::renderer::Renderer;
use voxel_phase::resources::manager::ResourceManager;

fn main() -> EngineResult<()> {
    env_logger::Builder::new()
        .filter_level(log::LevelFilter::Info)
        .filter_module("ash", log::LevelFilter::Info)
        .init();

    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 || args[1] == "--help" || args[1] == "-h" {
        print_usage();
        return Ok(());
    }

    if args[1] == "--list" {
        print_scenarios();
        return Ok(());
    }

    let scenario_name = &args[1];
    run_viewer(scenario_name)
}

fn print_usage() {
    println!("Physics bench scenario viewer");
    println!();
    println!("Usage: bench_viewer <scenario_name>");
    println!("       bench_viewer --list");
    println!();
    println!("Controls:");
    println!("  WASD / Arrows  Orbit camera");
    println!("  Q / E          Camera up / down");
    println!("  +/-            Zoom in / out");
    println!("  Space          Pause / unpause");
    println!("  N              Single-step (while paused)");
    println!("  R              Reset scenario");
    println!("  Escape         Quit");
}

fn print_scenarios() {
    println!("Available scenarios:");
    for name in SCENARIO_NAMES {
        println!("  {}", name);
    }
}

const SCENARIO_NAMES: &[&str] = &[
    "flat_sphere_rest",
    "sphere_slide",
    "flat_box_rest",
    "sphere_on_ramp",
    "box_on_ramp",
    "box_on_step",
    "heavy_sphere_on_platform",
    "sphere_in_bowl",
    "box_slides_down_wall",
    "sphere_sphere_collision",
    "sphere_obb_collision",
    "obb_obb_collision",
    "box_grid",
    "high_speed_sphere_ccd",
    "sphere_on_static_body",
    "sphere_through_dynamic_slab",
    "sphere_through_two_slabs",
    "sphere_into_dynamic_corner",
    "speculative_band_approach",
    "box_on_static_platform",
    "box_on_plank",
    "sliding_sphere",
    "low_friction_ramp",
    "keep_upright",
    "compound_table",
    "jenga_cross_wobble",
    "pendulum_ball_joint_settles",
    "hinge_settles_under_load",
    "hinge_holds_under_sustained_force",
    "hinge_axis_no_drift_zero_gravity",
];

/// Builds a scenario by name and runs the viewer with it.
fn run_viewer(name: &str) -> EngineResult<()> {
    match name {
        "flat_sphere_rest" => run_with_scenario(&FlatSphereRestScenario::new(0.3)),
        "sphere_slide" => run_with_scenario(&SphereSlideScenario::new()),
        "flat_box_rest" => run_with_scenario(&FlatBoxRestScenario::new(0.0)),
        "sphere_on_ramp" => run_with_scenario(&SphereOnRampScenario::new()),
        "box_on_ramp" => run_with_scenario(&BoxOnRampScenario::new()),
        "box_on_step" => run_with_scenario(&BoxOnStepScenario::new()),
        "heavy_sphere_on_platform" => run_with_scenario(&HeavySphereOnPlatformScenario::new()),
        "sphere_in_bowl" => run_with_scenario(&SphereInBowlScenario::new()),
        "box_slides_down_wall" => run_with_scenario(&BoxSlidesDownWallScenario::new(0.0)),
        "sphere_sphere_collision" => run_with_scenario(&SphereSphereCollisionScenario::new(0.8)),
        "sphere_obb_collision" => run_with_scenario(&SphereObbCollisionScenario::new(0.5)),
        "obb_obb_collision" => run_with_scenario(&ObbObbCollisionScenario::new(0.5)),
        "box_grid" => run_with_scenario(&BoxGridScenario::new(4)),
        "high_speed_sphere_ccd" => run_with_scenario(&HighSpeedSphereCcdScenario::new()),
        "sphere_on_static_body" => run_with_scenario(&SphereOnStaticBodyScenario::new(0.2)),
        "sphere_through_dynamic_slab" => {
            run_with_scenario(&SphereThroughDynamicSlabScenario::new())
        }
        "sphere_through_two_slabs" => run_with_scenario(&SphereThroughTwoSlabsScenario::new()),
        "sphere_into_dynamic_corner" => run_with_scenario(&SphereIntoDynamicCornerScenario::new()),
        "speculative_band_approach" => {
            run_with_scenario(&SpeculativeBandApproachScenario::spheres())
        }
        "box_on_static_platform" => run_with_scenario(&BoxOnStaticPlatformScenario::new(0.0)),
        "box_on_plank" => run_with_scenario(&BoxOnPlankScenario::new()),
        "sliding_sphere" => run_with_scenario(&SlidingSphereScenario::new()),
        "low_friction_ramp" => run_with_scenario(&LowFrictionRampScenario::new()),
        "keep_upright" => run_with_scenario(&KeepUprightScenario::new()),
        "compound_table" => run_with_scenario(&CompoundTableScenario::new()),
        "jenga_cross_wobble" => run_with_scenario(&JengaCrossWobbleScenario::new()),
        "pendulum_ball_joint_settles" => {
            run_with_scenario(&PendulumBallJointSettlesScenario::new())
        }
        "hinge_settles_under_load" => run_with_scenario(&HingeSettlesUnderLoadScenario::new()),
        "hinge_holds_under_sustained_force" => {
            run_with_scenario(&HingeHoldsUnderSustainedForceScenario::new())
        }
        "hinge_axis_no_drift_zero_gravity" => {
            run_with_scenario(&HingeAxisNoDriftZeroGravityScenario::new())
        }
        _ => {
            eprintln!("Unknown scenario: {}", name);
            eprintln!("Run with --list to see available scenarios.");
            std::process::exit(1);
        }
    }
}

/// Orbit camera state for the viewer.
struct OrbitCamera {
    target: Point3<f32>,
    distance: f32,
    azimuth: f32,
    elevation: f32,
}

impl OrbitCamera {
    fn new(target: Point3<f32>, distance: f32) -> Self {
        Self {
            target,
            distance,
            azimuth: std::f32::consts::FRAC_PI_4,
            elevation: 0.4,
        }
    }

    fn position(&self) -> Point3<f32> {
        let x = self.distance * self.elevation.cos() * self.azimuth.sin();
        let y = self.distance * self.elevation.sin();
        let z = self.distance * self.elevation.cos() * self.azimuth.cos();
        self.target + Vector3::new(x, y, z)
    }

    fn to_camera(&self, aspect: f32) -> Camera {
        Camera::new(
            self.position(),
            self.target,
            Vector3::y(),
            std::f32::consts::FRAC_PI_4,
            aspect,
            0.1,
            200.0,
        )
    }
}

/// Viewer application state.
struct ViewerState {
    world: PhysicsWorld,
    paused: bool,
    step_once: bool,
    orbit: OrbitCamera,
    sim_time: f32,
    physics_steps: u64,
    cfg: BenchRunConfig,
    timestep: FixedTimestep,
}

fn run_with_scenario<S: PhysicsBenchScenario>(scenario: &S) -> EngineResult<()> {
    let width = 1200u32;
    let height = 800u32;

    let event_loop = EventLoop::new()
        .map_err(|e| voxel_phase::core::error::EngineError::Window(format!("{}", e)))?;
    let window = WindowBuilder::new()
        .with_title(format!("Bench Viewer — {}", scenario.name()))
        .with_inner_size(LogicalSize::new(f64::from(width), f64::from(height)))
        .build(&event_loop)
        .map_err(|e| voxel_phase::core::error::EngineError::Window(format!("{}", e)))?;

    let vulkan_context = Arc::new(VulkanContext::new(&window)?);
    let mut renderer = Renderer::for_window(Arc::clone(&vulkan_context), &window, width, height)?;

    let resource_manager = ResourceManager::new(Arc::clone(&vulkan_context))?;
    let descriptor_manager = renderer.descriptor_manager();
    let texture_manager = resource_manager.create_texture_manager(descriptor_manager)?;

    let fallback_white = texture_manager
        .create_solid_colour(Colour::WHITE)
        .map_err(|e| {
            voxel_phase::core::error::EngineError::InvalidState(format!(
                "Failed to create fallback texture: {}",
                e
            ))
        })?;
    let material_manager = MaterialManagerBuilder::new().build(fallback_white);

    // Build the geometry mesh as wireframe lines for debug overlays
    let geometry = scenario.geometry();
    let geometry_lines = build_geometry_wireframe(geometry);

    // Build the physics world and set up the scenario
    let mut world = scenario.build_world();
    let _tracked = scenario.setup(&mut world);

    let cfg = BenchRunConfig::default();

    // Estimate a reasonable camera position from the geometry bounds
    let orbit = estimate_camera_orbit(geometry);

    let mut state = ViewerState {
        world,
        paused: false,
        step_once: false,
        orbit,
        sim_time: 0.0,
        physics_steps: 0,
        cfg,
        timestep: FixedTimestep::new(cfg.fixed_dt, cfg.max_substeps_per_frame as u32),
    };

    let mut last_instant = std::time::Instant::now();

    let scenario_name = scenario.name().to_string();

    let run_result = event_loop.run(move |event, elwt| {
        elwt.set_control_flow(ControlFlow::Poll);

        match &event {
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => {
                elwt.exit();
                return;
            }
            Event::WindowEvent {
                event:
                    WindowEvent::KeyboardInput {
                        event:
                            KeyEvent {
                                physical_key: PhysicalKey::Code(key),
                                state: ElementState::Pressed,
                                ..
                            },
                        ..
                    },
                ..
            } => {
                handle_key(&mut state, *key, scenario, elwt);
            }
            Event::AboutToWait => {
                let now = std::time::Instant::now();
                let real_dt = now.duration_since(last_instant).as_secs_f32();
                last_instant = now;

                // Step physics
                step_physics(&mut state, real_dt, scenario);

                // Build debug overlays for this frame
                let mut overlays = DebugOverlays::default();
                let mut debug_lines = DebugLines::default();

                // Draw static geometry as wireframe
                for (start, end) in &geometry_lines {
                    overlays.add_line(*start, *end, Colour::new(0.4, 0.4, 0.4, 1.0));
                }

                // Draw all rigid bodies
                draw_bodies(&state.world, &mut overlays);

                // Debug text
                debug_lines.add("Scenario", scenario_name.clone());
                debug_lines.add("Sim Time", format!("{:.2}s", state.sim_time));
                debug_lines.add("Steps", format!("{}", state.physics_steps));
                debug_lines.add(
                    "Paused",
                    if state.paused { "YES" } else { "no" }.to_string(),
                );
                debug_lines.add("Bodies", format!("{}", state.world.bodies().len()));

                // Render frame
                let aspect = width as f32 / height as f32;
                let camera = state.orbit.to_camera(aspect);
                let view = camera.get_view_matrix();
                let proj = camera.get_projection_matrix();
                let camera_pos =
                    Vector3::new(camera.position.x, camera.position.y, camera.position.z);

                match renderer.begin_frame() {
                    Ok((cb, present_index)) => {
                        if let Err(e) = renderer.update_scene(&view, &proj, &camera_pos) {
                            log::error!("Failed to update scene: {}", e);
                            return;
                        }

                        renderer.begin_opaque_pass(cb);

                        if let Err(e) = renderer.render_sky(cb, &view, &proj) {
                            log::error!("Failed to render sky: {}", e);
                        }

                        if let Err(e) = render_debug_overlays_opaque(
                            &mut renderer,
                            cb,
                            &overlays,
                            &material_manager,
                            &texture_manager,
                        ) {
                            log::error!("Failed to render debug overlays: {}", e);
                        }

                        renderer.begin_transparent_pass(cb, present_index);

                        if let Err(e) = render_debug_overlays_transparent(
                            &mut renderer,
                            cb,
                            &overlays,
                            &material_manager,
                            &texture_manager,
                        ) {
                            log::error!("Failed to render transparent debug overlays: {}", e);
                        }

                        if let Err(e) = renderer.render_overlay(cb, debug_lines.iter()) {
                            log::error!("Failed to render overlay: {}", e);
                        }

                        if let Err(e) = renderer.end_frame(cb, present_index) {
                            log::error!("Failed to end frame: {}", e);
                        }
                    }
                    Err(e) => {
                        log::error!("Failed to begin frame: {}", e);
                    }
                }
            }
            _ => {}
        }
    });

    unsafe {
        vulkan_context.device().device_wait_idle().ok();
    }

    run_result.map_err(|e| voxel_phase::core::error::EngineError::Window(format!("{}", e)))?;
    Ok(())
}

fn handle_key<S: PhysicsBenchScenario>(
    state: &mut ViewerState,
    key: KeyCode,
    scenario: &S,
    elwt: &EventLoopWindowTarget<()>,
) {
    const ORBIT_SPEED: f32 = 0.08;
    const ZOOM_SPEED: f32 = 0.9;
    const ELEV_SPEED: f32 = 0.08;

    match key {
        KeyCode::Escape => elwt.exit(),
        KeyCode::Space => state.paused = !state.paused,
        KeyCode::KeyN => state.step_once = true,

        // Camera orbit
        KeyCode::KeyA | KeyCode::ArrowLeft => state.orbit.azimuth -= ORBIT_SPEED,
        KeyCode::KeyD | KeyCode::ArrowRight => state.orbit.azimuth += ORBIT_SPEED,
        KeyCode::KeyW | KeyCode::ArrowUp => {
            state.orbit.elevation = (state.orbit.elevation + ELEV_SPEED).min(1.4);
        }
        KeyCode::KeyS | KeyCode::ArrowDown => {
            state.orbit.elevation = (state.orbit.elevation - ELEV_SPEED).max(-0.2);
        }
        KeyCode::KeyQ => state.orbit.target.y += 0.5,
        KeyCode::KeyE => state.orbit.target.y -= 0.5,
        KeyCode::Equal | KeyCode::NumpadAdd => {
            state.orbit.distance = (state.orbit.distance * ZOOM_SPEED).max(1.0);
        }
        KeyCode::Minus | KeyCode::NumpadSubtract => {
            state.orbit.distance = (state.orbit.distance / ZOOM_SPEED).min(200.0);
        }

        // Reset
        KeyCode::KeyR => {
            state.world = scenario.build_world();
            let _ = scenario.setup(&mut state.world);
            state.sim_time = 0.0;
            state.physics_steps = 0;
            state.timestep =
                FixedTimestep::new(state.cfg.fixed_dt, state.cfg.max_substeps_per_frame as u32);
            state.paused = false;
        }

        _ => {}
    }
}

fn step_physics<S: PhysicsBenchScenario>(state: &mut ViewerState, real_dt: f32, scenario: &S) {
    let should_step = !state.paused || state.step_once;
    state.step_once = false;

    if !should_step {
        return;
    }

    let dt = if state.paused {
        // Single-step: advance exactly one physics tick
        state.cfg.fixed_dt
    } else {
        real_dt.min(0.05) // Cap to avoid spiral of death
    };

    let substeps = state.timestep.accumulate(dt);
    if substeps == 0 {
        return;
    }

    let mut debug_lines = DebugLines::default();
    let fixed_dt = state.cfg.fixed_dt;

    // Contact generation once per frame
    let impulses = scenario.external_impulses(state.sim_time);
    state.world.update_contacts(
        fixed_dt,
        substeps,
        scenario.geometry(),
        &impulses,
        &mut debug_lines,
    );

    for _ in 0..substeps {
        state.world.substep(fixed_dt, scenario.geometry(), &[]);
        state.sim_time += fixed_dt;
        state.physics_steps += 1;
    }
}

/// Draw rigid bodies as debug shapes based on their collider types.
fn draw_bodies(world: &PhysicsWorld, overlays: &mut DebugOverlays) {
    for (_, body) in world.bodies().iter() {
        for &collider_handle in body.colliders() {
            let Some(collider) = world.collider(collider_handle) else {
                continue;
            };
            let world_tf = collider.world_transform(body.position(), body.rotation());
            let center = Point3::from(world_tf.translation.vector);
            let rot = world_tf.rotation;

            match collider.shape() {
                ColliderShape::Sphere { radius } => {
                    overlays.add_sphere(center, *radius, Colour::new(0.2, 0.7, 1.0, 1.0));
                }
                ColliderShape::Box { half_extents } => {
                    draw_box_wireframe(overlays, center, rot, *half_extents);
                }
                ColliderShape::Capsule {
                    half_height,
                    radius,
                } => {
                    overlays.add_sphere(center, *radius, Colour::new(0.2, 1.0, 0.5, 1.0));
                    let up = rot * Vector3::new(0.0, *half_height - *radius, 0.0);
                    overlays.add_line(
                        Point3::from(center.coords - up),
                        Point3::from(center.coords + up),
                        Colour::new(0.2, 1.0, 0.5, 1.0),
                    );
                }
                ColliderShape::ConvexHull { hull } => {
                    overlays.add_sphere(
                        center,
                        hull.bounding_radius,
                        Colour::new(0.8, 0.5, 1.0, 1.0),
                    );
                }
            }
        }
    }
}

/// Draw an OBB as 12 wireframe edges.
fn draw_box_wireframe(
    overlays: &mut DebugOverlays,
    center: Point3<f32>,
    rotation: nalgebra::UnitQuaternion<f32>,
    half_extents: Vector3<f32>,
) {
    let colour = Colour::new(1.0, 0.6, 0.2, 1.0);

    // Generate 8 corner points
    let signs: [(f32, f32, f32); 8] = [
        (-1.0, -1.0, -1.0),
        (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
    ];

    let corners: Vec<Point3<f32>> = signs
        .iter()
        .map(|(sx, sy, sz)| {
            let local = Vector3::new(
                sx * half_extents.x,
                sy * half_extents.y,
                sz * half_extents.z,
            );
            center + rotation * local
        })
        .collect();

    // 12 edges of a box
    let edges: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0), // bottom face
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4), // top face
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7), // verticals
    ];

    for (a, b) in &edges {
        overlays.add_line(corners[*a], corners[*b], colour);
    }
}

/// Build wireframe line segments from the scenario's static geometry.
fn build_geometry_wireframe(geometry: &dyn StaticGeometry) -> Vec<(Point3<f32>, Point3<f32>)> {
    // Query a huge AABB to get all triangles
    let big = 500.0;
    let aabb = AABB::new(Point3::new(-big, -big, -big), Point3::new(big, big, big));
    let patch = geometry.query_region(&aabb);
    let mut lines = Vec::new();

    for tri in &patch.triangles {
        let t = &tri.triangle;
        lines.push((t.v0, t.v1));
        lines.push((t.v1, t.v2));
        lines.push((t.v2, t.v0));
    }

    lines
}

/// Estimate a reasonable orbit camera position from the geometry.
fn estimate_camera_orbit(geometry: &dyn StaticGeometry) -> OrbitCamera {
    let big = 500.0;
    let aabb = AABB::new(Point3::new(-big, -big, -big), Point3::new(big, big, big));
    let patch = geometry.query_region(&aabb);

    if patch.triangles.is_empty() {
        return OrbitCamera::new(Point3::new(0.0, 1.0, 0.0), 15.0);
    }

    // Find bounding box center
    let mut min = Vector3::new(f32::MAX, f32::MAX, f32::MAX);
    let mut max = Vector3::new(f32::MIN, f32::MIN, f32::MIN);
    for tri in &patch.triangles {
        for v in [tri.triangle.v0, tri.triangle.v1, tri.triangle.v2] {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }
    }

    let center = Point3::from((min + max) * 0.5);
    let extent = (max - min).magnitude();
    let distance = (extent * 0.8).max(5.0).min(50.0);

    // Target slightly above center to see the ground plane
    let target = Point3::new(center.x, center.y.max(0.0) + 1.0, center.z);
    OrbitCamera::new(target, distance)
}
