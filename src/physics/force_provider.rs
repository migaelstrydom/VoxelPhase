//! Per-substep external force callback.
//!
//! External systems (buoyancy, wind, etc.) implement `SubstepForceProvider`
//! so the physics engine can recompute position-dependent forces each substep
//! instead of using stale frame-start values.

use generational_arena::Arena;
use nalgebra::Vector3;

use super::body::RigidBody;
use super::collider::Collider;
use super::handle::RigidBodyHandle;

/// Context passed to force providers each substep, giving read access to
/// body and collider state needed to compute forces.
pub struct ForceContext<'a> {
    pub bodies: &'a Arena<RigidBody>,
    pub colliders: &'a Arena<Collider>,
    pub gravity: Vector3<f32>,
    pub gravity_magnitude: f32,
}

/// Mutable target for a single body's force output.
pub struct ForceOutput {
    pub force: nalgebra::Vector3<f32>,
    pub torque: nalgebra::Vector3<f32>,
    pub linear_drag_coeff: f32,
    pub angular_drag_coeff: f32,
}

impl ForceOutput {
    pub fn zero() -> Self {
        Self {
            force: nalgebra::Vector3::zeros(),
            torque: nalgebra::Vector3::zeros(),
            linear_drag_coeff: 0.0,
            angular_drag_coeff: 0.0,
        }
    }
}

/// External force provider called each substep before force integration.
///
/// Implementations recompute position-dependent forces (buoyancy, wind fields,
/// etc.) from the body's current position, ensuring forces stay accurate as
/// bodies move within a frame.
pub trait SubstepForceProvider {
    /// Return the set of body handles that this provider affects.
    /// Called once per substep; only these bodies will have `compute_force` called.
    fn affected_bodies(&self) -> &[RigidBodyHandle];

    /// Compute force, torque, and drag for a single body.
    /// Called for each handle returned by `affected_bodies()`.
    fn compute_force(&self, handle: RigidBodyHandle, ctx: &ForceContext) -> ForceOutput;
}
