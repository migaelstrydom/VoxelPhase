//! Fire-related ECS components.

use nalgebra::{UnitQuaternion, Vector3};
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
    /// Seconds since the fuel ran out, while the last of the fire burns off
    /// and its smoke clears.
    pub burnt_out_for: f32,
}

/// Fuel that outlasts any level: a fire tended rather than one burning down.
const EVERLASTING_FUEL: f32 = 1.0e9;

impl OnFire {
    /// A fire that is kept fed and never burns out: a brazier's.
    pub fn everlasting() -> Self {
        Self {
            fuel_remaining: EVERLASTING_FUEL,
            burn_time: 0.0,
            burnt_out_for: 0.0,
        }
    }
}

/// Where on its entity a fire burns, when that is not the whole body: the
/// bowl of a brazier, not the brazier.
///
/// Without one, a burning body's fire fills its bounds from its origin up.
#[derive(Component, Clone, Copy, Debug)]
#[storage(VecStorage)]
pub struct FireSeat {
    /// The middle of the fire's base, in the entity's frame, m.
    pub base: Vector3<f32>,
    /// The fire's volume: width, height and depth, m.
    pub size: Vector3<f32>,
}

impl FireSeat {
    /// Where its base is in the world, for an entity at `position` turned by
    /// `orientation`.
    pub fn world_base(
        &self,
        position: Vector3<f32>,
        orientation: UnitQuaternion<f32>,
    ) -> Vector3<f32> {
        position + orientation * self.base
    }

    /// The point its light seems to come from, in the entity's frame: low
    /// in the flame, where it is brightest.
    pub fn light_offset(&self) -> Vector3<f32> {
        self.base + Vector3::new(0.0, self.size.y * 0.3, 0.0)
    }

    /// How far its light reaches.
    pub fn light_range(&self) -> f32 {
        self.size.x.max(self.size.z) * LIGHT_RANGE_PER_WIDTH
    }
}

/// Reach of a seated fire's light for each metre of the fire's width.
const LIGHT_RANGE_PER_WIDTH: f32 = 14.0;
