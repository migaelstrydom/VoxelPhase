use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// Whether a character is standing on something, and on what.
///
/// This exists to break `CharacterControlSystem`'s dependency on
/// `CharacterAnimator`. Grounding is a locomotion fact, not an animation fact —
/// a creature with no skeleton still needs to know it can jump.
///
/// One writer, for every kind of character: `ContactGroundingSystem`, which
/// projects the Support Set the physics engine already resolved. A rigged
/// humanoid and a roller get the same answer from the same contacts — the foot
/// probes measure where the ground is, never whether the character is on it.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct Grounding {
    /// True when the character has support beneath it this frame.
    pub is_grounded: bool,
    /// Surface normal of the supporting contact, if there is one. Points away
    /// from the ground, toward the character.
    pub normal: Option<Vector3<f32>>,
}

impl Grounding {
    pub fn airborne() -> Self {
        Self {
            is_grounded: false,
            normal: None,
        }
    }

    pub fn on(normal: Vector3<f32>) -> Self {
        Self {
            is_grounded: true,
            normal: Some(normal),
        }
    }

    /// Ground normal, falling back to world up when unsupported or unknown.
    pub fn normal_or_up(&self) -> Vector3<f32> {
        self.normal.unwrap_or_else(Vector3::y)
    }
}
