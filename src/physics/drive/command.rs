//! The command that crosses the seam: what an entity wants, and what it
//! declared about where the reaction goes.
//!
//! One type carries both halves because they are inseparable at the point of
//! delivery. The engine has one entry point for a drive
//! (`PhysicsWorld::set_body_drive`), and the anchor decides which of the two
//! delivery paths that call establishes — never both, see
//! [`crate::physics::body::BodyDrive`].

use nalgebra::Vector3;

use super::allowance::AllowanceCommand;

/// A projection allowance: what a jump verb does to the velocity component
/// along the jump axis — the world's up, or the support normal in a world with
/// no gravity.
///
/// Jump shaping cannot be expressed as an impulse. "Cut the jump in half" is
/// proportional to the velocity it acts on, so the same verb is a different
/// impulse at every point on the arc — a fixed impulse would under-cut a fast
/// jump and reverse a slow one. Hence a projection: a scale, then an optional
/// clamp, applied in that order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VerticalProjection {
    /// Multiplier on a *rising* vertical component. `1.0` is the identity.
    ///
    /// Only a component that points up the jump axis is scaled. Cutting
    /// a fall short is not a verb anyone has: the same multiplier applied to
    /// downward motion would read as a parachute.
    pub scale: f32,
    /// When true, a rising vertical component is zeroed after scaling — a
    /// walk-off starts falling immediately rather than lifting off the ramp.
    pub clamp_positive: bool,
}

impl Default for VerticalProjection {
    fn default() -> Self {
        Self {
            scale: 1.0,
            clamp_positive: false,
        }
    }
}

impl VerticalProjection {
    /// True when this projection would leave the vertical component untouched.
    pub fn is_identity(&self) -> bool {
        self.scale == 1.0 && !self.clamp_positive
    }

    /// Compose another scaling into this projection.
    ///
    /// Two verbs can fire on one frame — a buffered tap-jump applies the
    /// cutoff up front and the release edge applies it again — and the
    /// composition of two scalings is their product.
    pub fn scale_by(&mut self, factor: f32) {
        self.scale *= factor;
    }

    /// Apply this projection to one speed along the jump axis.
    pub fn applied_to(&self, along: f32) -> f32 {
        if along <= 0.0 {
            return along;
        }
        if self.clamp_positive {
            return 0.0;
        }
        along * self.scale
    }
}

/// The discrete half of one frame's command: the edge-triggered verbs that act
/// along the jump axis.
///
/// Consumed once, on the frame they are set. `DriveIntent::take_vertical_verbs`
/// takes them on gameplay's side of the seam, so the physics world never
/// writes back into an ECS component.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct VerticalVerbs {
    /// A jump: the speed to establish along the jump axis.
    pub jump_speed: Option<f32>,
    /// Jump shaping, applied after the jump.
    pub projection: VerticalProjection,
}

impl VerticalVerbs {
    /// True when these verbs would leave the body untouched — the ordinary
    /// case, every frame nobody presses jump.
    pub fn is_inert(&self) -> bool {
        self.jump_speed.is_none() && self.projection.is_identity()
    }
}

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
///
/// The targets are stated **relative to the anchor**: a medium anchor's world
/// is at rest, so its target is a world velocity, while a support anchor's is
/// a velocity across whatever holds the body up.
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
    /// Factor separating the drive's tangential budget from the contact's own
    /// grip, for a support anchor. `1.0` drives exactly as hard as it grips.
    ///
    /// A medium anchor ignores it: its rows push against the world and are
    /// bounded by `max_accel` instead, with no contact to be honest or
    /// dishonest about.
    pub drive_gain: f32,
    /// Radius of the contact patch this body's supports stand for.
    ///
    /// The engine cannot derive it: a `SolverContact` is a point, and a
    /// manifold's several points already resist spin through their own
    /// tangential rows. So the entity declares it, and the torsional row is
    /// inert — `μ·N·0` — for everything that does not. See
    /// `docs/TRACTION_DRIVE_DESIGN.md` §6.2.
    pub patch_radius: f32,
    /// The non-conservative half of the command: what this body may conjure
    /// where no contact can deliver it, and what it is asking to spend that on
    /// this frame.
    pub allowance: AllowanceCommand,
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
            drive_gain: 1.0,
            patch_radius: 0.0,
            allowance: AllowanceCommand::default(),
        }
    }

    /// Declare how much harder than the surface permits this drive may push.
    pub fn with_drive_gain(mut self, gain: f32) -> Self {
        self.drive_gain = gain;
        self
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
            drive_gain: 1.0,
            patch_radius: 0.0,
            allowance: AllowanceCommand::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A projection acts on a rise only. Cutting a fall short is not a verb
    /// anyone has, and the same multiplier applied downward reads as a
    /// parachute.
    #[test]
    fn a_projection_shapes_a_rise_and_leaves_a_fall_alone() {
        let cut = VerticalProjection {
            scale: 0.45,
            clamp_positive: false,
        };
        assert_eq!(cut.applied_to(10.0), 4.5);
        assert_eq!(cut.applied_to(-10.0), -10.0);
    }

    /// The walk-off verb: cancel the rise outright, whatever it was scaled by.
    #[test]
    fn a_clamp_cancels_a_rise_entirely() {
        let walk_off = VerticalProjection {
            scale: 0.45,
            clamp_positive: true,
        };
        assert_eq!(walk_off.applied_to(10.0), 0.0);
        assert_eq!(walk_off.applied_to(-10.0), -10.0);
        assert!(!walk_off.is_identity());
    }

    /// Two verbs on one frame compose as a product, which is what two
    /// unguarded `v *= factor` call sites did.
    #[test]
    fn two_scalings_compose_as_a_product() {
        let mut projection = VerticalProjection::default();
        assert!(projection.is_identity());
        projection.scale_by(0.5);
        projection.scale_by(0.5);
        assert_eq!(projection.applied_to(8.0), 2.0);
    }
}
