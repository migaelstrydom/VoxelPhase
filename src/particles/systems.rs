//! Particle-related ECS systems.

use nalgebra::Vector3;
use rand::Rng;
use specs::{Entities, Join, Read, ReadExpect, ReadStorage, System, Write, WriteStorage};

use super::config::ParticleConfig;
use super::emitter::ParticleEmitter;
use super::particle::{Budget, ParticlePool};
use super::spec::ParticleSpec;
use crate::components::{Position, Velocity};
use crate::time::Time;

/// Where an emitter is this frame, and where it was last frame.
///
/// Bundled because they are only meaningful together: continuous spawns are
/// laid along the segment between them.
#[derive(Debug, Clone, Copy)]
pub struct EmitterMotion {
    /// Where the emitter was when it last ran.
    pub previous: Vector3<f32>,
    /// Where it is now.
    pub current: Vector3<f32>,
    /// Velocity each particle inherits, already scaled by the emitter's
    /// `velocity_inheritance`.
    pub inherited_velocity: Vector3<f32>,
}

/// Whether an emitter is still running after being stepped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmitterState {
    /// Still alive; leave it be.
    Running,
    /// Its lifetime has run out and it should be removed.
    Finished,
}

/// Advance one emitter by `dt`, spawning whatever it owes into `pool`.
///
/// Free-standing rather than a method on the system so the same stepping drives
/// both the ECS system and the offline explosion preview — a tuning tool that
/// disagreed with the game about when particles appear would be worse than no
/// tool at all.
pub fn step_emitter(
    emitter: &mut ParticleEmitter,
    spec: &ParticleSpec,
    dt: f32,
    motion: EmitterMotion,
    pool: &mut ParticlePool,
    rng: &mut impl Rng,
) -> EmitterState {
    if !emitter.active || !emitter.tick_delay(dt) {
        return EmitterState::Running;
    }

    // The lifetime bounds the continuous emission only. The initial burst
    // fires on the emitter's first frame however long that frame was: a
    // burst-only emitter has a span shorter than a slow frame, and checking
    // the span first lost the burst outright on exactly the hitch a blast
    // causes.
    let expired = emitter.lifetime.as_mut().is_some_and(|lifetime| {
        *lifetime -= dt;
        *lifetime <= 0.0
    });

    let scale = emitter.scale;
    let palette = emitter.palette;
    let budget = spec.most_live.map(|most_live| Budget {
        effect: emitter.effect_type,
        most_live,
    });
    let mut spawn = |position: Vector3<f32>, rng: &mut _| {
        let mut particle = spec.sample(position, scale, rng);
        particle.budget = budget;
        if let Some(palette) = palette {
            particle.ramp = particle.ramp.recoloured(palette.pick(rng));
            particle.colour = particle.ramp.sample(0.0);
        }
        particle.velocity += motion.inherited_velocity;
        pool.spawn(particle);
    };

    if !emitter.burst_spawned && emitter.initial_burst > 0 {
        for _ in 0..emitter.initial_burst {
            spawn(motion.current, rng);
        }
        emitter.burst_spawned = true;
    }

    if expired {
        return EmitterState::Finished;
    }

    if emitter.spawn_rate > 0.0 {
        emitter.spawn_accumulator += emitter.spawn_rate * dt;

        let count = emitter.spawn_accumulator.floor();
        emitter.spawn_accumulator -= count;

        let count = count as u32;
        for index in 0..count {
            spawn(
                trail_position(motion.previous, motion.current, index, count),
                rng,
            );
        }
    }

    EmitterState::Running
}

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

            let motion = EmitterMotion {
                previous: previous_pos,
                current: spawn_pos,
                inherited_velocity: velocities
                    .get(entity)
                    .map(|velocity| velocity.0 * emitter.velocity_inheritance)
                    .unwrap_or_else(Vector3::zeros),
            };
            let spec = config.spec(emitter.effect_type);

            if step_emitter(emitter, spec, dt, motion, &mut pool, &mut rng)
                == EmitterState::Finished
            {
                to_delete.push(entity);
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
    use crate::particles::ParticleEffectType;

    fn distance(a: Vector3<f32>, b: Vector3<f32>) -> f32 {
        (b - a).magnitude()
    }

    /// A burst-only emitter's span is shorter than a slow frame; the burst
    /// still fires, and the emitter is then done.
    #[test]
    fn a_burst_fires_on_a_frame_longer_than_its_emitters_span() {
        let config = ParticleConfig::default();
        let effect = ParticleEffectType::Debris;
        let mut emitter = ParticleEmitter::new(effect)
            .with_burst(12)
            .with_lifetime(0.05);
        let mut pool = ParticlePool::new(100);
        let motion = EmitterMotion {
            previous: Vector3::zeros(),
            current: Vector3::zeros(),
            inherited_velocity: Vector3::zeros(),
        };

        let state = step_emitter(
            &mut emitter,
            config.spec(effect),
            0.1,
            motion,
            &mut pool,
            &mut rand::thread_rng(),
        );

        assert_eq!(pool.count(), 12);
        assert_eq!(state, EmitterState::Finished);
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
