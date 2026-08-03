//! Particle-related ECS systems.

use nalgebra::Vector3;
use rand::Rng;
use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use super::config::{ParticleConfig, ParticleEffectConfig};
use super::emitter::{ParticleEffectType, ParticleEmitter};
use super::particle::{Particle, ParticlePool};
use crate::components::{Position, Velocity};
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
        ReadStorage<'a, Velocity>,
        WriteStorage<'a, ParticleEmitter>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time, config, mut pool, positions, velocities, mut emitters) = data;

        let dt = time.delta_seconds();
        let mut rng = rand::thread_rng();

        // Track emitters to delete
        let mut to_delete = Vec::new();

        for (entity, pos, emitter) in (&entities, &positions, &mut emitters).join() {
            let spawn_pos = pos.0;

            // Where the emitter was when it last spawned. A moving emitter can
            // cross metres between frames, so continuous spawns are laid along
            // this segment rather than stacked at the frame position — the
            // difference between a trail and a row of puffs.
            //
            // Tracked even for emitters that spawn nothing this frame, so one
            // switching back on lays its next particles along the travel since
            // then rather than across everywhere it has been.
            let previous_pos = emitter.last_position.unwrap_or(spawn_pos);
            emitter.last_position = Some(spawn_pos);

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

            let inherited_velocity = velocities
                .get(entity)
                .map(|velocity| velocity.0 * emitter.velocity_inheritance)
                .unwrap_or_else(Vector3::zeros);

            // Handle initial burst
            if !emitter.burst_spawned && emitter.initial_burst > 0 {
                for _ in 0..emitter.initial_burst {
                    spawn_particle(
                        &mut pool,
                        &config,
                        emitter.effect_type,
                        spawn_pos,
                        inherited_velocity,
                        &mut rng,
                    );
                }
                emitter.burst_spawned = true;
            }

            // Handle continuous spawn
            if emitter.spawn_rate > 0.0 {
                emitter.spawn_accumulator += emitter.spawn_rate * dt;

                let count = emitter.spawn_accumulator.floor();
                emitter.spawn_accumulator -= count;

                let count = count as u32;
                for index in 0..count {
                    let position = trail_position(previous_pos, spawn_pos, index, count);

                    spawn_particle(
                        &mut pool,
                        &config,
                        emitter.effect_type,
                        position,
                        inherited_velocity,
                        &mut rng,
                    );
                }
            }
        }

        // Delete expired emitters
        for entity in to_delete {
            let _ = entities.delete(entity);
        }
    }
}

/// Where the `index`-th of `count` continuous spawns goes, spread along the
/// segment the emitter covered this frame.
///
/// Each particle sits at the centre of the slice of travel it stands for, so
/// the spacing between them is even and no particle lands exactly on either
/// endpoint — a particle at the current position would be spawned twice over
/// two frames at the same spot.
fn trail_position(
    previous: Vector3<f32>,
    current: Vector3<f32>,
    index: u32,
    count: u32,
) -> Vector3<f32> {
    debug_assert!(count > 0, "no particles to place");
    let along = (index as f32 + 0.5) / count as f32;
    previous.lerp(&current, along)
}

/// Spawn a single particle of the given effect type.
///
/// `inherited_velocity` is added to whatever launch velocity the effect gives
/// the particle, carrying across the motion of the emitting entity.
fn spawn_particle(
    pool: &mut ParticlePool,
    config: &ParticleConfig,
    effect_type: ParticleEffectType,
    position: Vector3<f32>,
    inherited_velocity: Vector3<f32>,
    rng: &mut impl Rng,
) {
    let mut particle = match effect_type {
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
                stretch: cfg.stretch(),
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
                stretch: cfg.stretch(),
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
                stretch: cfg.stretch(),
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
                stretch: cfg.stretch(),
            }
        }

        ParticleEffectType::EmberTrail => {
            let cfg = &config.ember_trail;
            let lifetime = rng.gen_range(cfg.min_lifetime()..cfg.max_lifetime());
            let size = rng.gen_range(cfg.min_size()..cfg.max_size());
            let speed = rng.gen_range(cfg.min_speed()..cfg.max_speed());

            // Scattered off the emitter's path in every direction; the tail
            // shape comes from the inherited velocity, not from this.
            let dir = random_direction(rng);

            let start_color = cfg.start_color();
            let end_color = cfg.end_color();
            Particle {
                position,
                velocity: dir * speed,
                color: start_color,
                start_color,
                end_color,
                size,
                life: lifetime,
                max_life: lifetime,
                gravity_scale: cfg.gravity_scale(),
                drag: cfg.drag(),
                stretch: cfg.stretch(),
            }
        }

        ParticleEffectType::WaterSplash => {
            let cfg = &config.splash;
            let lifetime = rng.gen_range(cfg.min_lifetime()..cfg.max_lifetime());
            let size = rng.gen_range(cfg.min_size()..cfg.max_size());
            let speed = rng.gen_range(cfg.min_speed()..cfg.max_speed());

            // Radial outward with upward bias (upper hemisphere).
            let mut dir = random_direction(rng);
            dir.y = dir.y.abs() + 0.6;
            dir = dir.normalize();

            let start_color = cfg.start_color();
            let end_color = cfg.end_color();
            Particle {
                position,
                velocity: dir * speed,
                color: start_color,
                start_color,
                end_color,
                size,
                life: lifetime,
                max_life: lifetime,
                gravity_scale: cfg.gravity_scale(),
                drag: cfg.drag(),
                stretch: cfg.stretch(),
            }
        }
    };

    particle.velocity += inherited_velocity;
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

#[cfg(test)]
mod tests {
    use super::*;

    fn distance(a: Vector3<f32>, b: Vector3<f32>) -> f32 {
        (b - a).magnitude()
    }

    #[test]
    fn a_single_spawn_lands_midway_along_the_travel() {
        let previous = Vector3::new(0.0, 0.0, 0.0);
        let current = Vector3::new(4.0, 0.0, 0.0);

        assert_eq!(
            trail_position(previous, current, 0, 1),
            Vector3::new(2.0, 0.0, 0.0)
        );
    }

    #[test]
    fn spawns_are_evenly_spread_between_the_two_positions() {
        let previous = Vector3::new(0.0, 0.0, 0.0);
        let current = Vector3::new(0.0, 0.0, 10.0);
        let count = 5;

        let placed: Vec<_> = (0..count)
            .map(|index| trail_position(previous, current, index, count))
            .collect();

        // Ordered from where the emitter was towards where it is now.
        for pair in placed.windows(2) {
            assert!(pair[0].z < pair[1].z);
        }

        // Evenly spaced, and inset from both ends by half a step.
        let step = distance(placed[0], placed[1]);
        for pair in placed.windows(2) {
            assert!((distance(pair[0], pair[1]) - step).abs() < 1e-5);
        }
        assert!((distance(previous, placed[0]) - step / 2.0).abs() < 1e-5);
        assert!((distance(placed[count as usize - 1], current) - step / 2.0).abs() < 1e-5);
    }

    #[test]
    fn a_stationary_emitter_spawns_everything_where_it_stands() {
        let here = Vector3::new(1.0, 2.0, 3.0);
        for index in 0..4 {
            assert_eq!(trail_position(here, here, index, 4), here);
        }
    }
}
