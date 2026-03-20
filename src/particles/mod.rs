//! GPU-accelerated particle system for visual effects.
//!
//! This module provides a complete particle system with:
//! - Particle pool for efficient memory management
//! - Particle emitters for spawning effects
//! - Billboard rendering for camera-facing particles
//! - Multiple effect types (fire, smoke, debris, sparks)

pub mod config;
mod emitter;
mod particle;
pub mod pipeline;
pub mod renderer;
pub mod systems;
pub mod vertex;

pub use config::ParticleConfig;
pub use emitter::ParticleEmitter;
pub use particle::{Particle, ParticlePool};
pub use renderer::ParticleRenderer;
pub use systems::{ParticleSpawnSystem, ParticleUpdateSystem};
