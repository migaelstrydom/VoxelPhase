//! Particle emitter components for spawning effects.

use nalgebra::Vector3;
use specs::{Component, VecStorage};

/// Types of particle effects available.
///
/// Each names one [`ParticleSpec`](super::spec::ParticleSpec) in
/// [`ParticleConfig`](super::config::ParticleConfig); the tuning lives there,
/// not here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParticleEffectType {
    /// The blinding first instant of a detonation.
    BlastCore,
    /// The rolling ball of fire that cools into smoke.
    Fireball,
    /// The low ring of dust thrown outward along the ground.
    BlastDust,
    /// Burning fragments thrown clear of a blast.
    Embers,
    /// Physics-affected debris chunks.
    Debris,
    /// Slow-rising, fading smoke clouds.
    Smoke,
    /// Radial splash from a body impacting water.
    WaterSplash,
    /// Cooling embers shed by a hot object, for comet-tail trails.
    EmberTrail,
    /// The glint of a small shard vanishing: what the eye is given in place of
    /// a piece of debris the budget has taken away.
    ShardGlitter,
}

/// Component for entities that emit particles.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct ParticleEmitter {
    /// Type of effect to emit.
    pub effect_type: ParticleEffectType,
    /// Particles spawned per second.
    pub spawn_rate: f32,
    /// Accumulated fractional particles (for sub-frame spawning).
    pub spawn_accumulator: f32,
    /// Remaining lifetime of this emitter (None = infinite).
    pub lifetime: Option<f32>,
    /// Whether the emitter is currently active.
    pub active: bool,
    /// Initial burst count (spawned immediately on first update).
    pub initial_burst: u32,
    /// Whether the initial burst has been spawned.
    pub burst_spawned: bool,
    /// Seconds still to wait before this emitter does anything at all.
    ///
    /// What lets one event be choreographed out of several emitters: the fire
    /// of an explosion goes up immediately, the smoke column that replaces it
    /// starts a quarter of a second later. Its own `lifetime` does not begin
    /// running until the delay has elapsed, so a delayed emitter still gets the
    /// full run it was authored with.
    pub delay: f32,
    /// Spatial scale applied to every particle this emitter spawns — sizes,
    /// speeds and spawn spread. See [`ParticleSpec::sample`](super::spec::ParticleSpec::sample).
    pub scale: f32,
    /// Fraction of the emitting entity's velocity each particle is born with.
    ///
    /// A trail shed by a fast body wants some of that motion, or the particles
    /// appear to be flung backwards out of it. 0 leaves them at rest in world
    /// space, 1 has them keep pace with the emitter.
    pub velocity_inheritance: f32,
    /// Where this emitter spawned from last frame, used to spread continuous
    /// spawns along the path travelled since. `None` until the first update.
    pub last_position: Option<Vector3<f32>>,
}

impl ParticleEmitter {
    /// Create a new emitter with the given effect type.
    pub fn new(effect_type: ParticleEffectType) -> Self {
        Self {
            effect_type,
            spawn_rate: 0.0,
            spawn_accumulator: 0.0,
            lifetime: None,
            active: true,
            initial_burst: 0,
            burst_spawned: false,
            delay: 0.0,
            scale: 1.0,
            velocity_inheritance: 0.0,
            last_position: None,
        }
    }

    /// Set the fraction of the emitter's own velocity that particles inherit.
    pub fn with_velocity_inheritance(mut self, fraction: f32) -> Self {
        self.velocity_inheritance = fraction;
        self
    }

    /// Set the spawn rate (particles per second).
    pub fn with_spawn_rate(mut self, rate: f32) -> Self {
        self.spawn_rate = rate;
        self
    }

    /// Set the emitter lifetime in seconds.
    pub fn with_lifetime(mut self, seconds: f32) -> Self {
        self.lifetime = Some(seconds);
        self
    }

    /// Set the initial burst count.
    pub fn with_burst(mut self, count: u32) -> Self {
        self.initial_burst = count;
        self
    }

    /// Hold this emitter back for `seconds` before it starts.
    pub fn with_delay(mut self, seconds: f32) -> Self {
        self.delay = seconds;
        self
    }

    /// Scale every particle this emitter spawns.
    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }

    /// Tick the emitter's delay down by `dt`, reporting whether it may spawn
    /// this frame.
    ///
    /// A frame longer than the remaining delay still only starts the emitter;
    /// the leftover is discarded rather than credited to the effect, which
    /// keeps a hitching frame from firing a burst early.
    pub fn tick_delay(&mut self, dt: f32) -> bool {
        if self.delay <= 0.0 {
            return true;
        }
        self.delay -= dt;
        false
    }

    /// Create a debris emitter.
    pub fn debris() -> Self {
        Self::new(ParticleEffectType::Debris)
            .with_burst(15)
            .with_lifetime(0.1) // Short - just spawns the burst
    }

    /// Create a smoke emitter.
    pub fn smoke() -> Self {
        Self::new(ParticleEffectType::Smoke)
            .with_spawn_rate(20.0)
            .with_burst(8)
            .with_lifetime(1.0)
    }

    /// Create an ember trail emitter.
    ///
    /// Runs indefinitely at a rate its owner sets each frame — a trail should
    /// thin out when the object it trails from slows down — so no rate or
    /// lifetime is fixed here.
    pub fn ember_trail() -> Self {
        Self::new(ParticleEffectType::EmberTrail).with_velocity_inheritance(0.25)
    }

    /// Create a water splash emitter.
    pub fn water_splash() -> Self {
        Self::new(ParticleEffectType::WaterSplash)
            .with_burst(25)
            .with_lifetime(0.2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_undelayed_emitter_may_spawn_from_its_first_frame() {
        let mut emitter = ParticleEmitter::new(ParticleEffectType::Fireball);
        assert!(emitter.tick_delay(1.0 / 60.0));
    }

    #[test]
    fn a_delayed_emitter_waits_then_runs() {
        let delay = 0.1;
        let mut emitter = ParticleEmitter::new(ParticleEffectType::Smoke).with_delay(delay);

        let dt = 1.0 / 60.0;
        let mut frames_waited = 0;
        while !emitter.tick_delay(dt) {
            frames_waited += 1;
            assert!(frames_waited < 100, "emitter never started");
        }

        // It waits the delay out, to within the frame it cannot subdivide.
        let expected = (delay / dt).ceil() as i32;
        assert!(
            (frames_waited - expected).abs() <= 1,
            "waited {frames_waited} frames, expected about {expected}"
        );
        assert!(
            emitter.tick_delay(dt),
            "emitter stopped again after starting"
        );
    }
}
