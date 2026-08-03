//! Projectile system for throwable objects like grenades.
//!
//! This module provides components and systems for spawning, tracking,
//! and handling projectiles in the game world.

mod components;
mod config;
pub mod model;
pub mod systems;
mod visuals;

pub use components::{Grenade, Lifetime, Projectile};
pub use config::GrenadeConfig;
pub use model::{build_grenade_model, GrenadeMaterials};
pub use systems::{
    GrenadeCooldown, GrenadeModelResource, GrenadeSpawnSystem, LifetimeSystem,
    ProjectileImpactDetectionSystem,
};
pub use visuals::{GrenadeVisualSystem, GrenadeVisuals};
