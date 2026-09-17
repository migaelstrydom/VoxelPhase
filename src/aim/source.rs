//! What the player is currently aiming.

use nalgebra::{Point3, Vector3};

use crate::character::grab::{self, GrabConfig};
use crate::character::ArmState;
use crate::physics::{PhysicsWorld, RigidBodyHandle};
use crate::projectile::{grenade_launch, GrenadeConfig};

use super::launch::Launch;

/// Which throw a prediction belongs to.
///
/// The HUD reads this rather than inferring from the arm: what is being aimed
/// is the aim module's decision, and a second reader of the arm state would be
/// a second place to keep the rule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AimKind {
    /// A grenade, thrown along the camera's look direction.
    Grenade,
    /// Whatever the character is holding, thrown along its facing.
    HeldObject,
}

/// Everything a source needs to state its launch.
pub struct AimContext<'a> {
    /// The physics world, for the bodies a source may need to weigh.
    pub physics: &'a PhysicsWorld,
    /// The character doing the throwing.
    pub thrower: RigidBodyHandle,
    /// The thrower's position (pelvis).
    pub position: Point3<f32>,
    /// The thrower's horizontal facing, unit length.
    pub facing: Vector3<f32>,
    /// The direction the camera looks, as a Y rotation.
    pub look_angle: f32,
    /// Camera pitch, positive looking down.
    pub camera_pitch: f32,
    /// What the arm is doing, which is what decides whether there is anything
    /// in hand to throw.
    pub arm: &'a ArmState,
}

/// Something the player can throw.
///
/// Each source owns two things and nothing else: the arc its throw produces,
/// and the bodies that arc must not be allowed to hit. Adding a weapon means
/// adding an impl here — the prediction system, the cursor and the flight
/// integrator stay untouched.
pub trait AimSource {
    /// Which throw this is, for whoever draws the result.
    fn kind(&self) -> AimKind;

    /// The arc a throw right now would fly, or `None` when this source has
    /// nothing to throw.
    fn launch(&self, ctx: &AimContext) -> Option<Launch>;

    /// Bodies the predicted flight must ignore: the thrower it starts inside,
    /// and anything that is about to become the projectile.
    fn ignored_bodies(&self, ctx: &AimContext) -> Vec<RigidBodyHandle> {
        vec![ctx.thrower]
    }
}

/// Aiming a grenade. Always has something to throw.
pub struct GrenadeAim<'a> {
    config: &'a GrenadeConfig,
}

impl<'a> GrenadeAim<'a> {
    pub fn new(config: &'a GrenadeConfig) -> Self {
        Self { config }
    }
}

impl AimSource for GrenadeAim<'_> {
    fn kind(&self) -> AimKind {
        AimKind::Grenade
    }

    fn launch(&self, ctx: &AimContext) -> Option<Launch> {
        Some(grenade_launch(
            self.config,
            ctx.position,
            ctx.look_angle,
            ctx.camera_pitch,
            ctx.physics.config().gravity,
        ))
    }
}

/// Aiming whatever is in hand. Has nothing to throw unless the arm is holding
/// something.
pub struct HeldObjectAim<'a> {
    config: &'a GrabConfig,
}

impl<'a> HeldObjectAim<'a> {
    pub fn new(config: &'a GrabConfig) -> Self {
        Self { config }
    }

    /// The body in hand, if there is one.
    fn held_body(ctx: &AimContext) -> Option<RigidBodyHandle> {
        match ctx.arm {
            ArmState::Holding { target_body, .. } => Some(*target_body),
            _ => None,
        }
    }
}

impl AimSource for HeldObjectAim<'_> {
    fn kind(&self) -> AimKind {
        AimKind::HeldObject
    }

    fn launch(&self, ctx: &AimContext) -> Option<Launch> {
        let body = Self::held_body(ctx)?;
        grab::throw_launch(ctx.physics, body, ctx.facing, self.config.throw_impulse)
    }

    fn ignored_bodies(&self, ctx: &AimContext) -> Vec<RigidBodyHandle> {
        match Self::held_body(ctx) {
            Some(body) => vec![ctx.thrower, body],
            None => vec![ctx.thrower],
        }
    }
}
