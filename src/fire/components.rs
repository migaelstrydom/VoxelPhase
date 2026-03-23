//! Fire-related ECS components.

use specs::{Component, VecStorage};

/// Marks an entity as flammable. Required for fire ignition.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct Flammable {
    /// Total fuel available. Divided by FUEL_BURN_RATE to get burn duration.
    /// e.g. fuel=20 at rate=0.2 → ~100s burn. Also scaled by explosion
    /// distance falloff at ignition, so distant objects burn shorter.
    pub fuel: f32,
    /// Temperature threshold to catch fire (0.0–1.0).
    pub ignition_threshold: f32,
}

impl Flammable {
    pub fn wood() -> Self {
        Self {
            fuel: 20.0,
            ignition_threshold: 0.2,
        }
    }
}

/// Active fire on an entity. Created by FireIgnitionSystem.
///
/// The GPU resources (volume textures, descriptor sets) are managed by
/// `FireRenderer` and indexed by the entity. This component stores only
/// the lightweight simulation metadata.
#[derive(Component, Debug)]
#[storage(VecStorage)]
pub struct OnFire {
    /// Remaining fuel (decremented as fire burns).
    pub fuel_remaining: f32,
    /// Time the fire has been burning.
    pub burn_time: f32,
}
