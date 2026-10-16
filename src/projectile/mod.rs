//! Projectile system for throwable objects like grenades.
//!
//! This module provides components and systems for spawning, tracking,
//! and handling projectiles in the game world.

mod components;
mod config;
mod held;
pub mod model;
mod spawn;
pub mod systems;
mod throw;
mod visuals;

pub use components::{Grenade, Lifetime, Projectile};
pub use config::GrenadeConfig;
pub use held::{HeldGrenade, HeldGrenadeSystem};
pub use model::{build_grenade_model, GrenadeMaterials};
pub use spawn::{create_grenade_body, spawn_grenade};
pub use systems::{
    GrenadeCooldown, GrenadeModelResource, GrenadeSpawnSystem, LifetimeSystem,
    ProjectileDetonationSystem,
};
pub use throw::grenade_launch;
pub use visuals::{GrenadeVisualSystem, GrenadeVisuals};
