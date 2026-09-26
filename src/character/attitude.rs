use specs::{Component, DenseVecStorage};

use crate::physics::{ConstraintHandle, ConstraintKind, PhysicsWorld};

/// The constraint holding a character's body at its attitude, for a character
/// whose body can lie down — to swim.
///
/// A character without one is held upright by whatever its spawner gave it,
/// and swims standing up. `CharacterState::body_pitch` is the pitch asked
/// for; this is where it is handed to the engine.
#[derive(Component, Debug, Clone, Copy)]
#[storage(DenseVecStorage)]
pub struct AttitudeControl {
    /// A `ConstraintKind::KeepAttitude` on the character's body.
    pub constraint: ConstraintHandle,
}

impl AttitudeControl {
    /// Hold the body at `pitch` from now on. A constraint that has been
    /// released — a corpse's — is left alone.
    pub fn hold(&self, physics: &mut PhysicsWorld, pitch: f32) {
        let Some(constraint) = physics.constraint_mut(self.constraint) else {
            return;
        };
        if let ConstraintKind::KeepAttitude { pitch: held, .. } = &mut constraint.kind {
            *held = pitch;
        }
    }
}

/// Move `current` toward `target` at no more than `rate` per second.
pub fn approach(current: f32, target: f32, rate: f32, dt: f32) -> f32 {
    let step = rate * dt;
    current + (target - current).clamp(-step, step)
}
