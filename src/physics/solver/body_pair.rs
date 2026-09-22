//! Body pair mass properties for preparing contact rows.

use generational_arena::Arena;
use nalgebra::{Matrix3, Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::PairHeader;

/// Snapshot of both bodies' pose and mass properties for a contact pair.
///
/// Velocities are deliberately absent: they change on every row the solver
/// runs, so rows read them live (see `contact_row`), while everything here is
/// fixed for the substep.
///
/// Stores real physics values — no kinematic overrides. Callers that need the
/// kinematic-static treatment apply the override via `effective_mass_with_overrides`.
///
/// When shock propagation is active, `inv_mass` and `inv_inertia` fields are
/// pre-scaled by the per-body shock factor during extraction.
pub(crate) struct BodyPairState {
    pub pos_a: Point3<f32>,
    pub inv_mass_a: f32,
    pub inv_inertia_a: Matrix3<f32>,

    pub pos_b: Point3<f32>,
    pub inv_mass_b: f32,
    pub inv_inertia_b: Matrix3<f32>,
}

impl BodyPairState {
    /// Extract pose and mass properties for both sides of a contact pair.
    ///
    /// `contact_point` is used as the fallback position for static body_a (where
    /// body_a is None). The actual value doesn't affect physics since static bodies
    /// have zero mass and inertia.
    ///
    /// `shock_scales` applies shock propagation mass scaling: `(scale_a, scale_b)`.
    /// Inverse mass and inertia are pre-multiplied by the corresponding scale so
    /// that all downstream effective-mass computations see shock-adjusted values.
    pub fn extract(
        bodies: &Arena<RigidBody>,
        header: &PairHeader,
        contact_point: Point3<f32>,
        shock_scales: (f32, f32),
    ) -> Option<Self> {
        let body_b = bodies.get(header.body_b.0)?;

        let (pos_a, inv_mass_a, inv_inertia_a) = match header.body_a {
            Some(handle) => {
                let body_a = bodies.get(handle.0)?;
                (
                    body_a.position(),
                    body_a.inv_mass() * shock_scales.0,
                    body_a.world_inv_inertia() * shock_scales.0,
                )
            }
            None => (contact_point, 0.0, Matrix3::zeros()),
        };

        Some(Self {
            pos_a,
            inv_mass_a,
            inv_inertia_a,
            pos_b: body_b.position(),
            inv_mass_b: body_b.inv_mass() * shock_scales.1,
            inv_inertia_b: body_b.world_inv_inertia() * shock_scales.1,
        })
    }

    /// Compute the effective mass for an impulse along the given direction.
    pub fn effective_inv_mass(&self, point: Point3<f32>, direction: &Vector3<f32>) -> f32 {
        self.effective_inv_mass_with_overrides(
            point,
            direction,
            self.inv_mass_b,
            self.inv_inertia_b,
        )
    }

    /// Compute effective (inverse) mass with explicit body_b overrides.
    ///
    /// Used by the normal solver to treat kinematic-vs-static contacts
    /// as having unit (inverse) mass, while friction uses the real (zero) mass so
    /// kinematic bodies aren't slowed by friction against static geometry.
    pub fn effective_inv_mass_with_overrides(
        &self,
        point: Point3<f32>,
        direction: &Vector3<f32>,
        inv_mass_b: f32,
        inv_inertia_b: Matrix3<f32>,
    ) -> f32 {
        let r_a = point - self.pos_a;
        let r_b = point - self.pos_b;

        let r_a_cross = r_a.cross(direction);
        let r_b_cross = r_b.cross(direction);

        let angular_a = (self.inv_inertia_a * r_a_cross).cross(&r_a);
        let angular_b = (inv_inertia_b * r_b_cross).cross(&r_b);

        self.inv_mass_a + inv_mass_b + (angular_a + angular_b).dot(direction)
    }
}

/// Returns true if body_b is kinematic and body_a is static geometry.
#[inline]
pub(crate) fn is_kinematic_static(body: &RigidBody, header: &PairHeader) -> bool {
    body.is_kinematic() && header.body_a.is_none()
}
