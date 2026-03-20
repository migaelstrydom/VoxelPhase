//! Particle emitter components for spawning effects.

use specs::{Component, VecStorage};

/// Types of particle effects available.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParticleEffectType {
    /// Bright, short-lived explosion flash.
    ExplosionFlash,
    /// Slow-rising, fading smoke clouds.
    Smoke,
    /// Physics-affected debris chunks.
    Debris,
    /// Fast, small, bright trailing sparks.
    Sparks,
    /// Radial splash from a body impacting water.
    WaterSplash,
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
        }
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

    /// Create an explosion flash emitter.
    pub fn explosion_flash() -> Self {
        Self::new(ParticleEffectType::ExplosionFlash)
            .with_burst(20)
            .with_lifetime(0.3)
    }

    /// Create a smoke emitter.
    pub fn smoke() -> Self {
        Self::new(ParticleEffectType::Smoke)
            .with_spawn_rate(30.0)
            .with_burst(10)
            .with_lifetime(2.0)
    }

    /// Create a debris emitter.
    pub fn debris() -> Self {
        Self::new(ParticleEffectType::Debris)
            .with_burst(15)
            .with_lifetime(0.1) // Short - just spawns the burst
    }

    /// Create a sparks emitter.
    pub fn sparks() -> Self {
        Self::new(ParticleEffectType::Sparks)
            .with_burst(30)
            .with_spawn_rate(50.0)
            .with_lifetime(0.5)
    }

    /// Create a water splash emitter.
    pub fn water_splash() -> Self {
        Self::new(ParticleEffectType::WaterSplash)
            .with_burst(25)
            .with_lifetime(0.2)
    }
}
