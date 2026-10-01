//! System that steps the water and turns what bodies do in it into spray.

use nalgebra::{Point3, Vector3, Vector4};
use rand::Rng;
use specs::{Join, LendJoin, Read, ReadStorage, System, Write};

use crate::components::{CameraComponent, Position, RigidBodyComponent, Velocity};
use crate::debug::DebugLog;
use crate::drive::Actuator;
use crate::particles::{ColourRamp, Particle, ParticleConfig, ParticlePool};
use crate::systems::PhysicsResource;
use crate::terrain::TerrainWorld;
use crate::time::Time;
use crate::water::{
    BodySnapshot, Disturbances, SplashEvent, WakeEvent, WaterWorld, WaveBodyCoupler,
};

/// Steps the water each frame.
///
/// Catches the water up with any terrain edit the frame made, ranks its
/// ripple tiles by distance from the camera, advances the hydrology by the
/// frame's delta time, then lets the coupler find bodies entering or moving
/// through water, for splash and wake spray.
pub struct WaterSystem;

impl<'a> System<'a> for WaterSystem {
    type SystemData = (
        Option<Write<'a, WaterWorld>>,
        Option<Write<'a, WaveBodyCoupler>>,
        Option<Read<'a, TerrainWorld>>,
        Read<'a, Time>,
        Read<'a, PhysicsResource>,
        ReadStorage<'a, RigidBodyComponent>,
        ReadStorage<'a, Position>,
        ReadStorage<'a, Velocity>,
        ReadStorage<'a, Actuator>,
        Write<'a, DebugLog>,
        Write<'a, ParticlePool>,
        Read<'a, ParticleConfig>,
        ReadStorage<'a, CameraComponent>,
    );

    fn run(
        &mut self,
        (
            water_opt,
            coupler_opt,
            terrain_opt,
            time,
            physics,
            bodies,
            positions,
            velocities,
            actuators,
            mut debug_log,
            mut particle_pool,
            particle_config,
            cameras,
        ): Self::SystemData,
    ) {
        let Some(mut water) = water_opt else {
            return;
        };

        if let Some(ref terrain) = terrain_opt {
            water.on_terrain_update(terrain);
        }
        water.set_focus(cameras.join().next().map(|camera| camera.0.position));
        water.step(time.delta_seconds());

        if let Some(mut coupler) = coupler_opt {
            let snapshots =
                build_body_snapshots(&physics, &bodies, &positions, &velocities, &actuators);
            let mut disturbances = Disturbances::default();
            coupler.update(&snapshots, &mut disturbances, &water.query());
            water.apply(&disturbances);
            for splash in coupler.drain_splash_events() {
                spawn_splash_particles(&splash, &particle_config, &mut particle_pool);
            }
            for wake in coupler.drain_wake_events() {
                spawn_wake_particles(&wake, &mut particle_pool);
            }
        }

        water.debug_log(&mut debug_log);
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
    actuators: &ReadStorage<Actuator>,
) -> Vec<BodySnapshot> {
    let mut snapshots = Vec::new();

    for (body_comp, pos, vel, actuator) in (bodies, positions, velocities, actuators.maybe()).join()
    {
        // An `Actuator` is what declares that a body moves under its own
        // power, so it is also what answers the wake question. Nothing here
        // asks the engine whether a body is driven.
        let is_self_propelled = actuator.is_some();
        let Some(rb) = physics.world.body(body_comp.0) else {
            continue;
        };
        if !rb.is_dynamic() {
            continue;
        }

        // What the water sees of the body, which for a character is the
        // figure and not the capsule round it.
        let Some(part) = physics.world.envelope(body_comp.0).next() else {
            continue;
        };
        let footprint_radius = part.shape.footprint_radius();

        // Use the arena index raw parts to create a stable u64 ID.
        let (slot, gen) = body_comp.0.raw_parts();
        let id = (gen << 32) | (slot as u64);

        snapshots.push(BodySnapshot {
            id,
            position: Point3::new(pos.0.x, pos.0.y, pos.0.z),
            velocity: vel.0,
            footprint_radius,
            mass: rb.mass(),
            is_self_propelled,
        });
    }

    snapshots
}
