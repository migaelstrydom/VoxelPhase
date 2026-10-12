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

/// Drag from the medium a body moves through: it relaxes the body's velocity
/// and spin towards the medium's own, exactly however stiff the drag is
/// against the body's mass and inertia, so water carries a light body at its
/// own speed and spins it no faster than it moves past it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MediumDrag {
    /// Linear drag per unit velocity relative to `velocity`, N·s/m.
    pub linear: f32,
    /// Angular drag per unit angular velocity relative to `spin`,
    /// N·m·s/rad.
    pub angular: f32,
    /// Velocity of the medium, m/s: a current or a falling stream.
    pub velocity: nalgebra::Vector3<f32>,
    /// Spin the medium drives the body towards, rad/s: zero in still
    /// water; where a stream strikes a body off its centre, the spin at
    /// which the struck side moves with the water.
    pub spin: nalgebra::Vector3<f32>,
}

impl MediumDrag {
    /// No medium.
    pub const NONE: Self = Self {
        linear: 0.0,
        angular: 0.0,
        velocity: nalgebra::Vector3::new(0.0, 0.0, 0.0),
        spin: nalgebra::Vector3::new(0.0, 0.0, 0.0),
    };
}

/// Mutable target for a single body's force output.
pub struct ForceOutput {
    pub force: nalgebra::Vector3<f32>,
    pub torque: nalgebra::Vector3<f32>,
    pub drag: MediumDrag,
}

impl ForceOutput {
    pub fn zero() -> Self {
        Self {
            force: nalgebra::Vector3::zeros(),
            torque: nalgebra::Vector3::zeros(),
            drag: MediumDrag::NONE,
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
