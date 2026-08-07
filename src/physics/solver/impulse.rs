//! Impulse application utilities shared across solver implementations.

use generational_arena::Arena;
use nalgebra::{Point3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::pipeline::pair::PairHeader;

use super::body_pair::is_kinematic_static;

/// Apply equal-and-opposite impulses to both bodies in a contact pair.
///
/// When shock propagation is active, each body's impulse is scaled by its
/// shock factor so the lower body absorbs less velocity change. Pass
/// `(1.0, 1.0)` for standard unscaled behaviour.
pub(crate) fn apply_impulse_pair(
    bodies: &mut Arena<RigidBody>,
    header: &PairHeader,
    point: Point3<f32>,
    impulse: Vector3<f32>,
    shock_scales: (f32, f32),
) {
    if let Some(handle_a) = header.body_a {
        if let Some(body_a) = bodies.get_mut(handle_a.0) {
            if body_a.is_dynamic() {
                body_a.apply_impulse_at_point(-impulse * shock_scales.0, point);
            }
        }
    }

    if let Some(body_b) = bodies.get_mut(header.body_b.0) {
        if body_b.is_dynamic() {
            body_b.apply_impulse_at_point(impulse * shock_scales.1, point);
        } else if is_kinematic_static(body_b, header) {
            body_b.set_linear_velocity(body_b.linear_velocity() + impulse * shock_scales.1);
        }
    }
}

/// Compute a stable orthonormal tangent basis from a normal vector.
pub(crate) fn compute_tangent_basis(normal: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    let reference = if normal.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let t1 = normal.cross(&reference).normalize();
    let t2 = normal.cross(&t1);
    (t1, t2)
}
