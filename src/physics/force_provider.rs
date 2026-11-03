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

    /// Whether this medium drags nothing.
    pub fn is_none(&self) -> bool {
        self.linear == 0.0 && self.angular == 0.0
    }

    /// The one medium that drags a body as `self` and `other` do together.
    ///
    /// Two linear drags pulling towards different velocities sum to one drag
    /// as stiff as both, pulling towards their stiffness-weighted mean
    /// velocity: `k1(v1 - v) + k2(v2 - v) = (k1 + k2)(v̄ - v)`. Exact, so a
    /// body wading out of water into a bush is dragged by both at once.
    pub fn combined(self, other: Self) -> Self {
        // Exactly the other medium when one is none, so a body only one
        // provider drags is dragged bit for bit as that provider says.
        if self.is_none() {
            return other;
        }
        if other.is_none() {
            return self;
        }
        let linear = self.linear + other.linear;
        let angular = self.angular + other.angular;
        let velocity = if linear > 0.0 {
            (self.velocity * self.linear + other.velocity * other.linear) / linear
        } else {
            nalgebra::Vector3::zeros()
        };
        let spin = if angular > 0.0 {
            (self.spin * self.angular + other.spin * other.angular) / angular
        } else {
            nalgebra::Vector3::zeros()
        };
        Self {
            linear,
            angular,
            velocity,
            spin,
        }
    }
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

    /// Add what another provider asks of the same body: forces and torques
    /// sum, and the two media combine into one ([`MediumDrag::combined`]).
    pub fn accumulate(&mut self, other: &ForceOutput) {
        self.force += other.force;
        self.torque += other.torque;
        self.drag = self.drag.combined(other.drag);
    }
}

/// External force provider called each substep before force integration.
///
/// Several providers may affect one body (water and foliage both drag a body
/// wading out of one into the other); their outputs accumulate.
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

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::*;

    fn drag(linear: f32, velocity: Vector3<f32>) -> MediumDrag {
        MediumDrag {
            linear,
            angular: 0.0,
            velocity,
            spin: Vector3::zeros(),
        }
    }

    /// Two media pulling a body pull it as one: as stiff as both, towards
    /// their stiffness-weighted velocity, so the force each would exert on a
    /// body at any velocity sums to what the combined medium exerts.
    #[test]
    fn two_media_drag_as_their_sum() {
        let a = drag(3.0, Vector3::new(2.0, 0.0, 0.0));
        let b = drag(1.0, Vector3::new(0.0, 0.0, -4.0));
        let both = a.combined(b);
        let v = Vector3::new(0.5, 1.0, 0.25);
        let separate = (a.velocity - v) * a.linear + (b.velocity - v) * b.linear;
        let combined = (both.velocity - v) * both.linear;
        assert!((separate - combined).magnitude() < 1e-5);
    }

    /// No medium combined with a medium is that medium.
    #[test]
    fn no_medium_changes_nothing() {
        let a = drag(2.0, Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(MediumDrag::NONE.combined(a), a);
        assert_eq!(a.combined(MediumDrag::NONE), a);
    }
}
