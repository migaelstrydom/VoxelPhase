//! Explosion system for handling detonations and their effects.
//!
//! This module provides components and systems for explosions that
//! destroy terrain and apply knockback to entities.

mod components;
pub mod systems;

pub use components::Explosion;
pub use systems::ExplosionSystem;
