//! CPU particle system for visual effects.
//!
//! ```text
//!   ParticleSpec (config) ──▶ ParticleEmitter ──▶ ParticleSpawnSystem
//!                                                        │
//!            ParticleRenderer ◀── ParticlePool ◀── ParticleUpdateSystem
//! ```
//!
//! Effects are authored as data (`spec`, `config`) rather than as code: the
//! spawner has one path, and every particle carries everything the renderer
//! needs to draw it — colour ramp, growth, spin, and whether it glows or
//! occludes. Adding an effect never means touching a system or a shader.

pub mod config;
mod emitter;
mod particle;
pub mod pipeline;
pub mod ramp;
pub mod renderer;
pub mod spec;
pub mod systems;
pub mod vertex;

pub use config::ParticleConfig;
pub use emitter::{ParticleEffectType, ParticleEmitter};
pub use particle::{Particle, ParticlePool};
pub use ramp::{ColourRamp, ColourStop};
pub use renderer::ParticleRenderer;
pub use spec::{LaunchPattern, ParticleSpec, Spread};
pub use systems::{
    step_emitter, EmitterMotion, EmitterState, ParticleSpawnSystem, ParticleUpdateSystem,
};
