//! Particle-related ECS systems.

use nalgebra::Vector3;
use rand::Rng;
use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use super::config::{ParticleConfig, ParticleEffectConfig};
use super::emitter::{ParticleEffectType, ParticleEmitter};
use super::particle::{Particle, ParticlePool};
use crate::components::Position;
use crate::time::Time;

/// System that spawns particles from emitters.
pub struct ParticleSpawnSystem;

impl<'a> System<'a> for ParticleSpawnSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, Time>,
        Read<'a, ParticleConfig>,
        Write<'a, ParticlePool>,
        ReadStorage<'a, Position>,
        WriteStorage<'a, ParticleEmitter>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, config, mut pool, positions, mut emitters) = data;

        let dt = time.delta_seconds();
        let mut rng = rand::thread_rng();

        // Track emitters to delete
        let mut to_delete = Vec::new();

        for (entity, pos, emitter) in (&entities, &positions, &mut emitters).join() {
            if !emitter.active {
                continue;
            }

            // Update emitter lifetime
            if let Some(ref mut lifetime) = emitter.lifetime {
                *lifetime -= dt;
                if *lifetime <= 0.0 {
                    to_delete.push(entity);
                    continue;
                }
            }

            let spawn_pos = pos.0;

            // Handle initial burst
            if !emitter.burst_spawned && emitter.initial_burst > 0 {
                for _ in 0..emitter.initial_burst {
                    spawn_particle(&mut pool, &config, emitter.effect_type, spawn_pos, &mut rng);
                }
                emitter.burst_spawned = true;
            }

            // Handle continuous spawn
            if emitter.spawn_rate > 0.0 {
                emitter.spawn_accumulator += emitter.spawn_rate * dt;

                while emitter.spawn_accumulator >= 1.0 {
                    emitter.spawn_accumulator -= 1.0;
                    spawn_particle(&mut pool, &config, emitter.effect_type, spawn_pos, &mut rng);
                }
            }
        }

        // Delete expired emitters
        for entity in to_delete {
            let _ = entities.delete(entity);
        }
    }
}

/// Spawn a single particle of the given effect type.
fn spawn_particle(
    pool: &mut ParticlePool,
    config: &ParticleConfig,
    effect_type: ParticleEffectType,
    position: Vector3<f32>,
    rng: &mut impl Rng,
) {
    let particle = match effect_type {
        ParticleEffectType::ExplosionFlash => {
            let cfg = &config.flash;
            let lifetime = rng.gen_range(cfg.min_lifetime()..cfg.max_lifetime());
            let size = rng.gen_range(cfg.min_size()..cfg.max_size());
            let speed = rng.gen_range(cfg.min_speed()..cfg.max_speed());

            // Random direction (sphere)
            let dir = random_direction(rng);

            let start_color = cfg.start_color();
            let end_color = cfg.end_color();
            Particle {
                position,
                velocity: dir * speed,
                color: start_color, // Will be interpolated in update()
                start_color,
                end_color,
                size,
                life: lifetime,
                max_life: lifetime,
                gravity_scale: cfg.gravity_scale(),
                drag: cfg.drag(),
            }
        }

        ParticleEffectType::Smoke => {
            let cfg = &config.smoke;
            let lifetime = rng.gen_range(cfg.min_lifetime()..cfg.max_lifetime());
            let size = rng.gen_range(cfg.min_size()..cfg.max_size());
            let speed = rng.gen_range(cfg.min_speed()..cfg.max_speed());

            // Random horizontal direction with upward bias
            let mut dir = random_direction(rng);
            dir.y = dir.y.abs() + 0.5; // Bias upward
            dir = dir.normalize();

            let start_color = cfg.start_color();
            let end_color = cfg.end_color();
            Particle {
                position,
                velocity: dir * speed + Vector3::new(0.0, cfg.rise_speed, 0.0),
                color: start_color, // Will be interpolated in update()
                start_color,
                end_color,
                size,
                life: lifetime,
                max_life: lifetime,
                gravity_scale: cfg.gravity_scale(),
                drag: cfg.drag(),
            }
        }

        ParticleEffectType::Debris => {
            let cfg = &config.debris;
            let lifetime = rng.gen_range(cfg.min_lifetime()..cfg.max_lifetime());
            let size = rng.gen_range(cfg.min_size()..cfg.max_size());
            let speed = rng.gen_range(cfg.min_speed()..cfg.max_speed());

            // Random direction with upward bias (explosion throws debris up)
            let mut dir = random_direction(rng);
            dir.y = dir.y.abs() + 0.3;
            dir = dir.normalize();

            let start_color = cfg.start_color();
            let end_color = cfg.end_color();
            Particle {
                position,
                velocity: dir * speed,
                color: start_color, // Will be interpolated in update()
                start_color,
                end_color,
                size,
                life: lifetime,
                max_life: lifetime,
                gravity_scale: cfg.gravity_scale(),
                drag: cfg.drag(),
            }
        }

        ParticleEffectType::Sparks => {
            let cfg = &config.sparks;
            let lifetime = rng.gen_range(cfg.min_lifetime()..cfg.max_lifetime());
            let size = rng.gen_range(cfg.min_size()..cfg.max_size());
            let speed = rng.gen_range(cfg.min_speed()..cfg.max_speed());

            // Random direction
            let dir = random_direction(rng);

            let start_color = cfg.start_color();
            let end_color = cfg.end_color();
            Particle {
                position,
                velocity: dir * speed,
                color: start_color, // Will be interpolated in update()
                start_color,
                end_color,
                size,
                life: lifetime,
                max_life: lifetime,
                gravity_scale: cfg.gravity_scale(),
                drag: cfg.drag(),
            }
        }
    };

    pool.spawn(particle);
}

/// Generate a random direction on the unit sphere.
fn random_direction(rng: &mut impl Rng) -> Vector3<f32> {
    let theta = rng.gen_range(0.0..std::f32::consts::TAU);
    let phi = rng.gen_range(-1.0f32..1.0).acos();

    Vector3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos())
}

/// System that updates particle physics and removes dead particles.
pub struct ParticleUpdateSystem;

impl<'a> System<'a> for ParticleUpdateSystem {
    type SystemData = (
        ReadExpect<'a, Time>,
        Read<'a, ParticleConfig>,
        Write<'a, ParticlePool>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (time, config, mut pool) = data;
        let dt = time.delta_seconds();

        pool.update(dt, config.gravity);
    }
}
