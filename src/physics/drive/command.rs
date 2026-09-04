//! The command that crosses the seam: what an entity wants, and what it
//! declared about where the reaction goes.
//!
//! One type carries both halves because they are inseparable at the point of
//! delivery. The engine has one entry point for a drive
//! (`PhysicsWorld::set_body_drive`), and the anchor decides which of the two
//! delivery paths that call establishes — never both, see
//! [`crate::physics::body::BodyDrive`].

use nalgebra::Vector3;

/// Where the equal-and-opposite half of a drive impulse lands.
///
/// A declaration about the entity, not a branch in the engine: whoever spawns
/// the body states what its motor pushes against, and the drive is honest
/// about it either way.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReactionAnchor {
    /// Reaction goes into whatever the body is standing on, at the contact
    /// points. A character.
    #[default]
    Support,
    /// Reaction goes into the world — an inexhaustible reservoir. A thruster,
    /// a rotor, a magnetically levitated lift.
    Medium,
}

/// One frame's drive command, as the engine receives it.
///
/// Produced from the gameplay channels (`DriveIntent` + `Actuator`) by
/// `crate::drive::resolve_drive`, and consumed by `PhysicsWorld::set_body_drive`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveCommand {
    /// The declared recipient of every reaction impulse this drive applies.
    pub anchor: ReactionAnchor,
    /// Target linear velocity, in world space.
    pub linear_target: Vector3<f32>,
    /// Target angular velocity, in world space.
    pub angular_target: Vector3<f32>,
    /// Linear acceleration budget, in m/s².
    pub max_accel: f32,
    /// Angular acceleration budget, in rad/s².
    pub angular_max_accel: f32,
}

impl DriveCommand {
    /// A command whose reaction goes into the bodies holding this one up.
    pub fn support(
        linear_target: Vector3<f32>,
        angular_target: Vector3<f32>,
        max_accel: f32,
        angular_max_accel: f32,
    ) -> Self {
        Self {
            anchor: ReactionAnchor::Support,
            linear_target,
            angular_target,
            max_accel,
            angular_max_accel,
        }
    }

    /// A command whose reaction goes into the world.
    pub fn medium(
        linear_target: Vector3<f32>,
        angular_target: Vector3<f32>,
        max_accel: f32,
        angular_max_accel: f32,
    ) -> Self {
        Self {
            anchor: ReactionAnchor::Medium,
            linear_target,
            angular_target,
            max_accel,
            angular_max_accel,
        }
    }
}
