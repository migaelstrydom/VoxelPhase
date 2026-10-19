//! What a suspension's owner declares.

use nalgebra::{UnitQuaternion, UnitVector3, Vector3};
use smallvec::SmallVec;

use crate::physics::collider::ColliderMaterial;
use crate::physics::constraint::spring::Spring;
use crate::physics::handle::RigidBodyHandle;

/// Which way a suspension's spring pushes. The cast runs against it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SuspensionAxis {
    /// A fixed world direction: a character's legs push against gravity.
    World(UnitVector3<f32>),
    /// A direction in the body's own frame: a vehicle's pods push along its
    /// up axis, and tip with it.
    Body(UnitVector3<f32>),
}

impl SuspensionAxis {
    /// The axis in world space for a body turned to `rotation`.
    pub fn in_world(&self, rotation: &UnitQuaternion<f32>) -> Vector3<f32> {
        match self {
            SuspensionAxis::World(axis) => axis.into_inner(),
            SuspensionAxis::Body(axis) => rotation * axis.into_inner(),
        }
    }
}

/// What a suspension is: everything its owner declares.
#[derive(Debug, Clone)]
pub struct SuspensionDesc {
    /// The body held up.
    pub body: RigidBodyHandle,
    /// The point the cast starts from, in the body's frame.
    pub anchor: Vector3<f32>,
    /// The direction the spring pushes.
    pub axis: SuspensionAxis,
    /// Radius of the cast sphere, in metres.
    pub cast_radius: f32,
    /// Distance from the anchor to the cast sphere's centre at which the
    /// spring is at rest, in metres.
    pub rest_length: f32,
    /// How much further than the rest length the cast reaches, in metres.
    /// Ground in this band holds the body as a support but pushes nothing.
    pub slack: f32,
    /// The spring, against the body's own mass: `k = m·ω²`.
    pub spring: Spring,
    /// Most force the spring may push with, in newtons. Its impulse per
    /// substep is bounded at `max_force·dt`, so nothing launches a body.
    pub max_force: f32,
    /// Bodies the cast passes through: what the body holds, is jointed to or
    /// carries. The body itself is never hit.
    pub exclude: SmallVec<[RigidBodyHandle; 4]>,
    /// The foot's material, which meets the surface at the hit as a
    /// collider's does to give the friction coefficient. `None`: the
    /// suspension grips nothing.
    pub friction: Option<ColliderMaterial>,
    /// Radius of the patch the torsion row bears on, in metres. Zero: no
    /// torsional grip.
    pub torsion_radius: f32,
}
