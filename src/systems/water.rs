//! System that steps the water simulation and provides debug visualisation.

use nalgebra::{Point3, Vector3, Vector4};
use rand::Rng;
use specs::{Join, LendJoin, Read, ReadStorage, System, Write};

use crate::components::{Position, RigidBodyComponent, Velocity, VelocityDriven};
use crate::debug::{DebugLines, DebugOverlays};
use crate::particles::{ColourRamp, Particle, ParticleConfig, ParticlePool};
use crate::rendering::Colour;
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;
use crate::water::{BodySnapshot, SplashEvent, WakeEvent, WaterGrid, WaveBodyCoupler, WaveGrid};

/// Steps the water flow simulation and wave equation each frame.
///
/// Reads terrain dirty_regions to detect floor changes under water, then
/// advances the flow sim by the frame's delta time. Wave-body coupling
/// injects disturbances from rigid body interactions before the wave
/// equation step produces fine-resolution surface ripples.
pub struct WaterSystem;

const WATER_DEBUG_COLOUR: Colour = Colour {
    r: 0.2,
    g: 0.5,
    b: 1.0,
    a: 0.8,
};

struct WaterDebugConfig {
    draw_wet_cells: bool,
    debug_colour: Colour,
    sphere_radius: f32,
}

const WATER_DEBUG_CONFIG: WaterDebugConfig = WaterDebugConfig {
    draw_wet_cells: false,
    debug_colour: WATER_DEBUG_COLOUR,
    sphere_radius: 0.3,
};

fn overlapping_cell_range(
    min: f32,
    max: f32,
    grid_origin: f32,
    cell_size: f32,
    dim: usize,
) -> Option<(usize, usize)> {
    if dim == 0 {
        return None;
    }

    let grid_min = grid_origin;
    let grid_max = grid_origin + dim as f32 * cell_size;
    if max <= grid_min || min >= grid_max {
        return None;
    }

    let start = ((min - grid_origin) / cell_size).floor() as isize;
    let end = ((max - grid_origin) / cell_size).ceil() as isize - 1;

    let clamped_start = start.clamp(0, dim as isize - 1) as usize;
    let clamped_end = end.clamp(0, dim as isize - 1) as usize;
    if clamped_start > clamped_end {
        None
    } else {
        Some((clamped_start, clamped_end))
    }
}

impl<'a> System<'a> for WaterSystem {
    type SystemData = (
        Option<Write<'a, WaterGrid>>,
        Option<Write<'a, WaveGrid>>,
        Option<Write<'a, WaveBodyCoupler>>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, Time>,
        Read<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, VelocityDriven>,
        Write<'a, DebugOverlays>,
        Write<'a, DebugLines>,
        Write<'a, ParticlePool>,
        Read<'a, ParticleConfig>,
    );

    fn run(
        &mut self,
        (
            water_opt,
            wave_opt,
            coupler_opt,
            terrain_opt,
            time,
            physics,
            bodies,
            positions,
            velocities,
            velocity_driven,
            mut debug_overlays,
            mut _debug_lines,
            mut particle_pool,
            particle_config,
        ): Self::SystemData,
    ) {
        let Some(mut grid) = water_opt else {
            return;
        };

        let dt = time.delta_seconds();

        // Propagate terrain damage to water floor levels.
        if let Some(ref terrain) = terrain_opt {
            let dirty = terrain.dirty_regions();
            if !dirty.is_empty() {
                let mut dirty_cells = Vec::new();
                let dims = grid.dims();
                let origin = grid.origin();
                let cell_size = grid.cell_size();
                for region in dirty {
                    // Convert AABB to grid cells that overlap, including edge regions.
                    let x_range = overlapping_cell_range(
                        region.min.x,
                        region.max.x,
                        origin.x,
                        cell_size,
                        dims.0,
                    );
                    let z_range = overlapping_cell_range(
                        region.min.z,
                        region.max.z,
                        origin.z,
                        cell_size,
                        dims.1,
                    );
                    if let (Some((i0, i1)), Some((j0, j1))) = (x_range, z_range) {
                        for j in j0..=j1 {
                            for i in i0..=i1 {
                                dirty_cells.push((i, j));
                            }
                        }
                    }
                }
                if !dirty_cells.is_empty() {
                    grid.mark_dirty_floors(&dirty_cells);
                }
            }
        }

        // Step the flow simulation.
        let terrain_ref = terrain_opt.as_deref();
        grid.step(dt, |x, z| {
            terrain_ref.and_then(|t| t.mesh_surface_height_at(x, z))
        });

        // Wave-body coupling + wave equation step.
        if let Some(mut wave_grid) = wave_opt {
            // Inject wave disturbances from rigid body interactions.
            if let Some(mut coupler) = coupler_opt {
                let snapshots = build_body_snapshots(
                    &physics,
                    &bodies,
                    &positions,
                    &velocities,
                    &velocity_driven,
                );
                coupler.update(&snapshots, &mut wave_grid, &grid);

                for splash in coupler.drain_splash_events() {
                    spawn_splash_particles(&splash, &particle_config, &mut particle_pool);
                }

                for wake in coupler.drain_wake_events() {
                    spawn_wake_particles(&wake, &mut particle_pool);
                }
            }

            wave_grid.step(dt, &grid);
        }

        // Debug visualisation: spheres at each wet cell's surface.
        if WATER_DEBUG_CONFIG.draw_wet_cells {
            let dims = grid.dims();
            let cell_area = grid.cell_area();

            for j in 0..dims.1 {
                for i in 0..dims.0 {
                    let cell = grid.cell(i, j);
                    if cell.volume <= 0.0 {
                        continue;
                    }
                    let x = grid.cell_center_x(i);
                    let z = grid.cell_center_z(j);
                    let y = cell.surface_level(cell_area);
                    debug_overlays.add_sphere(
                        Point3::new(x, y, z),
                        WATER_DEBUG_CONFIG.sphere_radius,
                        WATER_DEBUG_CONFIG.debug_colour,
                    );
                }
            }
        }
    }
}

/// Minimum impact speed for splash scaling (m/s).
const SPLASH_MIN_SPEED: f32 = 1.0;
/// Maximum impact speed for splash scaling — impacts above this cap out.
const SPLASH_MAX_SPEED: f32 = 15.0;
/// Droplet count range [min, max], linearly interpolated by impact intensity.
const SPLASH_DROPLET_COUNT: (u32, u32) = (8, 40);
/// Mist count range [min, max].
const SPLASH_MIST_COUNT: (u32, u32) = (6, 20);

/// Spawn a two-tier splash: fast droplets in a conical ring, plus slow mist.
///
/// Particle count and speed scale with impact velocity, so a gentle entry
/// produces a small puff while a fast slam creates a dramatic crown.
fn spawn_splash_particles(splash: &SplashEvent, config: &ParticleConfig, pool: &mut ParticlePool) {
    let mut rng = rand::thread_rng();
    let cfg = &config.splash;

    // Normalized intensity: 0 at min speed, 1 at max speed.
    let t =
        ((splash.speed - SPLASH_MIN_SPEED) / (SPLASH_MAX_SPEED - SPLASH_MIN_SPEED)).clamp(0.0, 1.0);

    let droplet_count = lerp_u32(SPLASH_DROPLET_COUNT.0, SPLASH_DROPLET_COUNT.1, t);
    let mist_count = lerp_u32(SPLASH_MIST_COUNT.0, SPLASH_MIST_COUNT.1, t);

    // Speed scales with impact intensity.
    let speed_scale = 0.5 + t * 0.5;

    let pos = splash.position.coords;

    // --- Tier 1: Droplets (ring / crown shape) ---
    for _ in 0..droplet_count {
        let theta = rng.gen_range(0.0..std::f32::consts::TAU);
        // Elevation between 20° and 70° from horizontal for a crown shape.
        let elevation = rng.gen_range(20.0f32..70.0).to_radians();
        let cos_e = elevation.cos();
        let sin_e = elevation.sin();
        let dir = Vector3::new(theta.cos() * cos_e, sin_e, theta.sin() * cos_e);

        let speed = cfg.speed.sample(&mut rng) * speed_scale;
        let lifetime = cfg.lifetime.sample(&mut rng);
        let size = cfg.size.sample(&mut rng);

        let ramp = ColourRamp::fade(
            Vector4::new(0.85, 0.92, 1.0, 0.95),
            Vector4::new(0.7, 0.85, 1.0, 0.0),
        );

        pool.spawn(
            Particle::new(pos, dir * speed, size, lifetime, ramp)
                .with_dynamics(cfg.gravity_scale, cfg.drag),
        );
    }

    // --- Tier 2: Mist (slow, lingering, near surface) ---
    for _ in 0..mist_count {
        let theta = rng.gen_range(0.0..std::f32::consts::TAU);
        // Mist rises gently: mostly upward, slight outward spread.
        let elevation = rng.gen_range(50.0f32..85.0).to_radians();
        let cos_e = elevation.cos();
        let sin_e = elevation.sin();
        let dir = Vector3::new(theta.cos() * cos_e, sin_e, theta.sin() * cos_e);

        let speed = rng.gen_range(0.5..2.0) * speed_scale;
        let lifetime = rng.gen_range(0.6..1.2);
        let size = rng.gen_range(0.15..0.35);

        let ramp = ColourRamp::fade(
            Vector4::new(0.8, 0.9, 1.0, 0.5),
            Vector4::new(0.85, 0.93, 1.0, 0.0),
        );

        pool.spawn(Particle::new(pos, dir * speed, size, lifetime, ramp).with_dynamics(0.1, 1.5));
    }
}

/// Minimum horizontal speed for wake spray scaling (m/s).
const WAKE_MIN_SPEED: f32 = 0.5;
/// Maximum horizontal speed for wake spray scaling.
const WAKE_MAX_SPEED: f32 = 8.0;
/// Particles per frame at minimum / maximum wake intensity.
const WAKE_PARTICLES_PER_FRAME: (u32, u32) = (1, 4);

/// Spawn a few spray particles behind a body moving through water.
///
/// Called every frame while the body is generating a wake. Particles spray
/// outward in a V shape behind the body, angled away from the movement
/// direction, with an upward kick and gravity pulling them back down.
fn spawn_wake_particles(wake: &WakeEvent, pool: &mut ParticlePool) {
    let mut rng = rand::thread_rng();

    let t = ((wake.speed - WAKE_MIN_SPEED) / (WAKE_MAX_SPEED - WAKE_MIN_SPEED)).clamp(0.0, 1.0);
    let count = lerp_u32(WAKE_PARTICLES_PER_FRAME.0, WAKE_PARTICLES_PER_FRAME.1, t);
    let speed_scale = 0.4 + t * 0.6;

    // Build a local frame: forward = movement direction, right = perpendicular in XZ.
    let fwd = wake.direction;
    let right = Vector3::new(-fwd.z, 0.0, fwd.x);

    // Start particles slightly below the water surface so they emerge from it.
    let pos = wake.position.coords - Vector3::new(0.0, 0.15, 0.0);

    for _ in 0..count {
        // Spray in a wide, randomized cone rather than tight directional jets.
        // Full 360° azimuth with a backward bias from the movement direction.
        let base_angle = rng.gen_range(-std::f32::consts::PI..std::f32::consts::PI);
        let backward_bias = 0.3;
        let horizontal =
            -fwd * backward_bias + right * base_angle.sin() + fwd * base_angle.cos() * 0.3;
        let elevation = rng.gen_range(10.0f32..50.0).to_radians();

        let dir =
            (horizontal.normalize() * elevation.cos() + Vector3::y() * elevation.sin()).normalize();

        let speed = rng.gen_range(1.5..4.0) * speed_scale;
        let lifetime = rng.gen_range(0.3..0.6);
        let size = rng.gen_range(0.04..0.12);

        let ramp = ColourRamp::fade(
            Vector4::new(0.85, 0.92, 1.0, 0.7),
            Vector4::new(0.8, 0.9, 1.0, 0.0),
        );

        // Offset spawn position randomly around the wake point.
        let lateral_offset = right * rng.gen_range(-1.0..1.0) * wake.radius * 0.5
            + fwd * rng.gen_range(-0.3..0.3) * wake.radius;
        let spawn_pos = pos + lateral_offset;

        pool.spawn(
            Particle::new(spawn_pos, dir * speed, size, lifetime, ramp).with_dynamics(0.8, 0.5),
        );
    }
}

fn lerp_u32(a: u32, b: u32, t: f32) -> u32 {
    (a as f32 + (b as f32 - a as f32) * t).round() as u32
}

/// Collect rigid body snapshots for wave-body coupling.
fn build_body_snapshots(
    physics: &PhysicsResource,
    bodies: &ReadStorage<RigidBodyComponent>,
    positions: &ReadStorage<Position>,
    velocities: &ReadStorage<Velocity>,
    velocity_driven: &ReadStorage<VelocityDriven>,
) -> Vec<BodySnapshot> {
    let mut snapshots = Vec::new();

    for (body_comp, pos, vel, vd) in (bodies, positions, velocities, velocity_driven.maybe()).join()
    {
        let is_velocity_driven: bool = vd.is_some();
        let Some(rb) = physics.world.body(body_comp.0) else {
            continue;
        };
        if !rb.is_dynamic() {
            continue;
        }

        let collider_handle = match rb.colliders().first() {
            Some(h) => *h,
            None => continue,
        };
        let Some(collider) = physics.world.collider(collider_handle) else {
            continue;
        };

        // Use the arena index raw parts to create a stable u64 ID.
        let (slot, gen) = body_comp.0.raw_parts();
        let id = (gen << 32) | (slot as u64);

        let footprint_radius = collider.shape().footprint_radius();

        snapshots.push(BodySnapshot {
            id,
            position: Point3::new(pos.0.x, pos.0.y, pos.0.z),
            velocity: vel.0,
            footprint_radius,
            mass: rb.mass(),
            is_velocity_driven,
        });
    }

    snapshots
}
