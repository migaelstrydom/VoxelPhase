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

use crate::physics::{Allowance, AllowanceCommand, NormalProjection, NormalVerbs, ReactionAnchor};

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
    /// Rate at which this frame's `linear_target` may be steered toward while
    /// nothing holds the body up, in m/s².
    ///
    /// Only ever spent against the actuator's allowance, and only while the
    /// body is unsupported: with a contact underneath, the ramp toward the
    /// target is the traction budget and this says nothing. `None` asks for no
    /// steering at all — a committed long jump, whose arc is ballistic and
    /// wants no correcting.
    pub steer_accel: Option<f32>,
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
    ///
    /// A **medium** anchor's authority: it is the bound on the motor rows that
    /// push against the world. A support anchor is bounded by its contacts
    /// instead — `μ · drive_gain · N` at each of them — and reads neither this
    /// nor the angular one.
    pub max_accel: f32,
    /// Maximum angular acceleration toward the target angular velocity,
    /// in rad/s². A medium anchor's authority; see [`Actuator::max_accel`].
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
    /// Factor separating this body's drive budget from the grip of the
    /// contacts it drives through. Default `1.0`.
    ///
    /// **This is not a coefficient of friction and does not model one.** No
    /// friction formulation supplies it: a real surface bounds acceleration at
    /// `μ·g`, which for this project's terrain is about 7.85 m/s² — close to
    /// the honest number for a person, and far too slow to play. Above `1.0`
    /// the body pushes harder through a contact than that contact permits,
    /// which buys cartoon responsiveness at the cost of honesty about what `μ`
    /// means. The player is `5.0`; an NPC that should feel heavy leaves it
    /// alone.
    ///
    /// It is one of the design's two sanctioned cheats and it is confined to
    /// this scalar: the drive remains a real impulse exchange at a real
    /// contact, with the correct torque arm, distributed across supports and
    /// silently absorbed by infinite-mass partners. Because it multiplies, one
    /// surface's response relative to another survives exactly — ice still
    /// reads as ice. What it spends is measured per body by
    /// `PhysicsWorld::traction_usage`. See `docs/TRACTION_DRIVE_DESIGN.md`
    /// §11, decision D1.
    pub drive_gain: f32,
    /// Radius of the contact patch this body's supports stand for, in metres.
    ///
    /// The third term of a torsional row's `μ·N·r` bound, and the one the
    /// engine cannot derive: a contact is a point, and the several contacts of
    /// a wide manifold already resist spin through their own tangential rows.
    /// So it is declared, by the only thing that knows — a turntable or a
    /// tracked vehicle sets it, a capsule leaves it at zero and turns with its
    /// allowance instead (§6.2).
    pub support_patch_radius: f32,
    /// What this body may conjure where no contact can deliver it: air
    /// steering, turning on the spot, and a jump with nothing underneath.
    ///
    /// `None` — the default — is a body with no non-conservative authority at
    /// all, which is every crate and prop in the game and every character
    /// whose actuator has been taken away. The design's second sanctioned
    /// cheat, opt-in and bounded per entity by construction; what it spends is
    /// measured by `PhysicsWorld::allowance_usage`. See
    /// `docs/TRACTION_DRIVE_DESIGN.md` §6.3 and R8.
    pub allowance: Option<Allowance>,
}

impl Default for Actuator {
    fn default() -> Self {
        Self {
            anchor: ReactionAnchor::Support,
            max_accel: 500.0,
            angular_max_accel: 500.0,
            non_support_grip: 1.0,
            drive_gain: 1.0,
            support_patch_radius: 0.0,
            allowance: None,
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

    /// Declare how much harder than its surfaces permit this body may push.
    ///
    /// Read [`Actuator::drive_gain`] before raising it: anything above `1.0`
    /// is a deliberate departure from what the contact could deliver.
    pub fn with_drive_gain(mut self, gain: f32) -> Self {
        self.drive_gain = gain;
        self
    }

    /// Declare how wide a patch this body's supports stand for, so its
    /// torsional row has something to bear on.
    pub fn with_patch_radius(mut self, radius: f32) -> Self {
        self.support_patch_radius = radius;
        self
    }

    /// Grant this body a budget of non-conservative authority.
    ///
    /// Read [`Actuator::allowance`] first: everything spent through it is
    /// momentum the world did not have.
    pub fn with_allowance(mut self, allowance: Allowance) -> Self {
        self.allowance = Some(allowance);
        self
    }

    /// The allowance half of one frame's command: this body's budget, the
    /// verbs it is spending this frame, and the rate it asked to be steered
    /// at.
    pub fn allowance_command(
        &self,
        verbs: NormalVerbs,
        steer_accel: Option<f32>,
    ) -> AllowanceCommand {
        AllowanceCommand {
            budget: self.allowance,
            verbs,
            steer_accel,
        }
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
