//! The three components that replace the single `Velocity`/`VelocityDriven`
//! channel: a command, a declaration, and a measurement.
//!
//! ```text
//!   gameplay ──DriveIntent──►┐
//!                            ├──► PhysicsSyncSystem ──► physics
//!   entity   ──Actuator────►─┘
//!   gameplay ◄──BodyMotion───────── PhysicsSyncSystem ◄── physics
//! ```
//!
//! Nothing travels up the command channel and nothing travels down the
//! measurement one. Gameplay may *read* a measurement to decide what to
//! command — that is a decision, not an edit — but it never writes a value
//! back into the channel it read it from.

use nalgebra::Vector3;
use specs::{Component, DenseVecStorage};

use crate::physics::ReactionAnchor;

/// A projection allowance: what a jump verb does to the velocity component
/// along the support normal.
///
/// Jump shaping cannot be expressed as an impulse. "Cut the jump in half" is
/// proportional to the velocity it acts on, so the same verb is a different
/// impulse at every point on the arc — a fixed impulse would under-cut a fast
/// jump and reverse a slow one. Hence a projection: a scale, then an optional
/// clamp, applied in that order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NormalProjection {
    /// Multiplier on a *rising* normal component. `1.0` is the identity.
    ///
    /// Only a component that points up the support normal is scaled. Cutting
    /// a fall short is not a verb anyone has: the same multiplier applied to
    /// downward motion would read as a parachute.
    pub scale: f32,
    /// When true, a rising normal component is zeroed after scaling — a
    /// walk-off starts falling immediately rather than lifting off the ramp.
    pub clamp_positive: bool,
}

impl Default for NormalProjection {
    fn default() -> Self {
        Self {
            scale: 1.0,
            clamp_positive: false,
        }
    }
}

impl NormalProjection {
    /// True when this projection would leave the normal component untouched.
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
}

/// The command channel: what gameplay wants this body to do.
///
/// Write-only from gameplay's side. Nothing reads a field back to modify it,
/// which is what keeps a command from being confused with a measurement.
#[derive(Component, Debug, Clone, Default)]
#[storage(DenseVecStorage)]
pub struct DriveIntent {
    /// Continuous target velocity, in world space.
    pub linear_target: Vector3<f32>,
    /// Continuous target angular velocity, in world space.
    pub angular_target: Vector3<f32>,
    /// Discrete: a jump, as a speed along the support normal. Consumed once,
    /// on the frame it is set.
    pub normal_impulse: Option<f32>,
    /// Discrete: jump shaping along the support normal. Consumed once,
    /// alongside `normal_impulse`.
    pub normal_projection: NormalProjection,
}

impl DriveIntent {
    /// Command a jump at `speed` along the support normal.
    pub fn jump(&mut self, speed: f32) {
        self.normal_impulse = Some(speed);
    }

    /// Cut the velocity along the support normal by `factor` — the
    /// variable-height jump verb.
    pub fn cut_normal(&mut self, factor: f32) {
        self.normal_projection.scale_by(factor);
    }

    /// Cancel any velocity *up* the support normal, leaving downward motion
    /// alone — the walk-off verb.
    pub fn clamp_normal_rise(&mut self) {
        self.normal_projection.clamp_positive = true;
    }

    /// Take the discrete half of the command, leaving the continuous half.
    ///
    /// The sync takes this at the frame boundary as it pushes, so consumption
    /// happens on gameplay's side of the seam and the physics world never
    /// writes to an ECS component.
    pub fn take_normal_verbs(&mut self) -> NormalVerbs {
        NormalVerbs {
            impulse: self.normal_impulse.take(),
            projection: std::mem::take(&mut self.normal_projection),
        }
    }
}

/// The discrete half of one frame's command, once taken.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct NormalVerbs {
    /// A jump: the speed to establish along the support normal.
    pub impulse: Option<f32>,
    /// Jump shaping, applied after the jump.
    pub projection: NormalProjection,
}

impl NormalVerbs {
    /// True when these verbs would leave the drive target untouched — the
    /// ordinary case, every frame nobody presses jump.
    pub fn is_inert(&self) -> bool {
        self.impulse.is_none() && self.projection.is_identity()
    }
}

/// The declaration channel: how this body converts intent into momentum.
///
/// A property of the entity. Removing it takes the body out of service — it
/// keeps its mass and its colliders and simply stops being driven, which is
/// how a corpse stops walking.
#[derive(Component, Debug, Clone)]
#[storage(DenseVecStorage)]
pub struct Actuator {
    /// The declared recipient of every reaction impulse this drive applies.
    ///
    /// Recorded from the start so an entity is never ambiguous about what it
    /// pushes against, though today's engine has only one delivery path and
    /// so cannot yet act on the distinction.
    pub anchor: ReactionAnchor,
    /// Maximum linear acceleration toward the target velocity, in m/s².
    pub max_accel: f32,
    /// Maximum angular acceleration toward the target angular velocity,
    /// in rad/s².
    pub angular_max_accel: f32,
    /// Fraction of a contact's tangential budget this body may draw where that
    /// contact is not in its Support Set. `1.0` grips everything it touches
    /// equally; near zero is a body that only holds onto what holds it up.
    ///
    /// A property of the actuator rather than of the collider material, so it
    /// scales what this body draws and leaves the surface's own friction
    /// alone: whatever the body leans on keeps its grip against everything
    /// else. A character sets it near zero so jumps along vertical surfaces
    /// are not grabbed; magnetic boots would set it to 1.0.
    pub non_support_grip: f32,
}

impl Default for Actuator {
    fn default() -> Self {
        Self {
            anchor: ReactionAnchor::Support,
            max_accel: 500.0,
            angular_max_accel: 500.0,
            non_support_grip: 1.0,
        }
    }
}

impl Actuator {
    /// A body that pushes against the ground it stands on: a character.
    pub fn character() -> Self {
        Self::default()
    }

    /// A body whose motor pushes against the world: a platform, a lift.
    pub fn medium(max_accel: f32, angular_max_accel: f32) -> Self {
        Self {
            anchor: ReactionAnchor::Medium,
            max_accel,
            angular_max_accel,
            ..Self::default()
        }
    }

    /// Declare how much of a contact's tangential budget this body draws where
    /// the contact does not hold it up.
    pub fn with_non_support_grip(mut self, grip: f32) -> Self {
        self.non_support_grip = grip;
        self
    }
}

/// The measurement channel: what the body actually did.
///
/// Read-only from gameplay's side, refreshed from the solver every frame.
#[derive(Component, Debug, Clone, Copy, Default)]
#[storage(DenseVecStorage)]
pub struct BodyMotion {
    /// Measured linear velocity, in world space.
    pub linear: Vector3<f32>,
    /// Measured angular velocity, in world space.
    pub angular: Vector3<f32>,
}

impl BodyMotion {
    /// Speed across the plane perpendicular to `up`.
    pub fn speed_across(&self, up: &Vector3<f32>) -> f32 {
        (self.linear - up * self.linear.dot(up)).magnitude()
    }
}
