use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

/// Whether a character is standing on something, and on what.
///
/// This exists to break `CharacterControlSystem`'s dependency on
/// `CharacterAnimator`. Grounding is a locomotion fact, not an animation fact —
/// a creature with no skeleton still needs to know it can jump.
///
/// Two writers, one per kind of character:
///
/// - `CharacterAnimationSystem` for rigged humanoids. It already derives this
///   from the foot probes, which is the more accurate source: probes see the
///   ledge the sole is over, not just where the capsule happens to touch.
/// - `ContactGroundingSystem` for everything else, from physics contact
///   normals. Skipped for entities that have an animator, so the probe-derived
///   value is never overwritten by the coarser one.
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
