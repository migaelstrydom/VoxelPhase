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
/// The one thing contacts cannot say is whether the support is under the
/// body at all (a brink rounded into a bevel holds a body like any slope);
/// for a character that carries a `DropProbe`, the same system looks under
/// its centre and says so in `over_drop`.
#[derive(Component, Debug, Clone, Copy, Default, PartialEq)]
#[storage(DenseVecStorage)]
pub struct Grounding {
    /// True when the character has support beneath it this frame.
    pub is_grounded: bool,
    /// Surface normal of the supporting contact, if there is one. Points away
    /// from the ground, toward the character.
    pub normal: Option<Vector3<f32>>,
    /// Velocity of the surface holding the character up. Zero on static
    /// terrain, the platform's own motion while riding one.
    ///
    /// This is the frame locomotion is expressed in: a gait's speed, and the
    /// world position a planted foot holds, are both measured against the
    /// thing being stood on rather than against the world. Without it a
    /// character on a moving platform walks on the spot while the floor slides
    /// out from under its feet.
    pub surface_velocity: Vector3<f32>,
    /// The way off a drop the character stands over, its centre past the
    /// brink holding it up: a unit vector across the up axis. `None` when the
    /// ground goes on under its centre, when it is not grounded, and for a
    /// character that does not probe for drops.
    pub over_drop: Option<Vector3<f32>>,
}

impl Grounding {
    pub fn airborne() -> Self {
        Self {
            is_grounded: false,
            normal: None,
            surface_velocity: Vector3::zeros(),
            over_drop: None,
        }
    }

    pub fn on(normal: Vector3<f32>) -> Self {
        Self {
            is_grounded: true,
            normal: Some(normal),
            surface_velocity: Vector3::zeros(),
            over_drop: None,
        }
    }

    /// The same support, carried by a surface that is itself moving.
    pub fn carried_by(mut self, surface_velocity: Vector3<f32>) -> Self {
        self.surface_velocity = surface_velocity;
        self
    }

    /// The same support, standing over a drop whose way off is `way_off`.
    pub fn over_drop(mut self, way_off: Option<Vector3<f32>>) -> Self {
        self.over_drop = way_off;
        self
    }

    /// Ground normal, falling back to world up when unsupported or unknown.
    pub fn normal_or_up(&self) -> Vector3<f32> {
        self.normal.unwrap_or_else(Vector3::y)
    }
}
