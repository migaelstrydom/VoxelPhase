//! Row refresh: a spring row's bias, recomputed before each substep.
//!
//! Constraint rows are expanded once per frame and reused by every substep.
//! A rigid joint gets away with a bias that is a frame old, because NGS
//! repairs whatever position error it leaves. A spring cannot: its force must
//! follow its compression substep by substep, or a frame of twelve substeps
//! pushes twelve times with the compression it had at the top of the frame.
//!
//! A row that opts in carries a [`RowRefresh`]: its error at expansion, its
//! bias rate and its bodies' poses then. Before each velocity phase the error
//! is carried forward by the Jacobian times the bodies' displacement since,
//! the same reference the contacts' closing allowances measure from
//! (`closing_allowance.rs`), and the bias is that error times the rate.

use generational_arena::Arena;
use nalgebra::{Isometry3, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::constraint::types::{ConstraintRow, RowRefresh};
use crate::physics::handle::RigidBodyHandle;

/// Recompute the bias of every row that carries a [`RowRefresh`]. Rows
/// without one are left exactly as they are.
pub(crate) fn refresh_rows(bodies: &Arena<RigidBody>, rows: &mut [ConstraintRow]) {
    for row in rows.iter_mut() {
        let Some(refresh) = row.refresh else {
            continue;
        };
        row.bias = refreshed_bias(bodies, row, &refresh);
    }
}

/// `bias_rate · (error + J · Δq)` for one row.
fn refreshed_bias(bodies: &Arena<RigidBody>, row: &ConstraintRow, refresh: &RowRefresh) -> f32 {
    let moved_a = displacement(bodies, row.body_a, refresh.pose_a.as_ref())
        .map_or(0.0, |(linear, angular)| {
            row.lin_jac_a.dot(&linear) + row.ang_jac_a.dot(&angular)
        });
    let moved_b = displacement(bodies, row.body_b, refresh.pose_b.as_ref())
        .map_or(0.0, |(linear, angular)| {
            row.lin_jac_b.dot(&linear) + row.ang_jac_b.dot(&angular)
        });
    refresh.bias_rate * (refresh.error + moved_a + moved_b)
}

/// How far a body has moved and turned since `reference`, as a translation
/// and a rotation vector. `None` for a world-anchored side or a body that is
/// gone, which has not moved.
fn displacement(
    bodies: &Arena<RigidBody>,
    handle: Option<RigidBodyHandle>,
    reference: Option<&Isometry3<f32>>,
) -> Option<(Vector3<f32>, Vector3<f32>)> {
    let body = bodies.get(handle?.0)?;
    let reference = reference?;
    let linear = body.position().coords - reference.translation.vector;
    let angular = (body.rotation() * reference.rotation.inverse()).scaled_axis();
    Some((linear, angular))
}

#[cfg(test)]
mod tests {
    use generational_arena::Arena;
    use nalgebra::{Isometry3, Point3, Vector3};

    use super::*;
    use crate::physics::body::RigidBodyDesc;
    use crate::physics::constraint::types::{CorrectionMode, Enforcement, RowKind};
    use crate::physics::handle::RigidBodyHandle;

    fn row(body: RigidBodyHandle, refresh: Option<RowRefresh>) -> ConstraintRow {
        ConstraintRow {
            body_a: None,
            body_b: Some(body),
            lin_jac_a: Vector3::zeros(),
            ang_jac_a: Vector3::zeros(),
            lin_jac_b: Vector3::y(),
            ang_jac_b: Vector3::new(0.0, 0.0, 0.5),
            effective_mass_inv: 1.0,
            bias: 7.0,
            softness: 0.0,
            accumulated_impulse: 0.0,
            bounds: (0.0, f32::INFINITY),
            constraint_index: generational_arena::Index::from_raw_parts(0, 0),
            row_index: 0,
            correction_mode: CorrectionMode::VelocityOnly,
            row_kind: RowKind::Linear,
            enforcement: Enforcement::Iterative,
            refresh,
            coupling: None,
        }
    }

    /// The bias follows the body: up by `dy` along the row's linear
    /// Jacobian, turned by `θ` about its angular one.
    #[test]
    fn a_refreshed_bias_follows_the_bodys_motion() {
        let mut bodies = Arena::new();
        let start = Point3::new(0.0, 1.0, 0.0);
        let handle = RigidBodyHandle(bodies.insert(crate::physics::body::RigidBody::new(
            RigidBodyDesc::dynamic().position(start),
        )));
        let mut rows = vec![row(
            handle,
            Some(RowRefresh {
                error: -0.25,
                bias_rate: 10.0,
                pose_a: None,
                pose_b: Some(Isometry3::translation(start.x, start.y, start.z)),
            }),
        )];

        refresh_rows(&bodies, &mut rows);
        assert!((rows[0].bias - -2.5).abs() < 1e-6, "{}", rows[0].bias);

        let body = bodies.get_mut(handle.0).unwrap();
        body.set_position(Point3::new(0.0, 1.1, 0.0));
        body.set_rotation(nalgebra::UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            0.2,
        ));
        refresh_rows(&bodies, &mut rows);
        let expected = 10.0 * (-0.25 + 0.1 + 0.5 * 0.2);
        assert!((rows[0].bias - expected).abs() < 1e-5, "{}", rows[0].bias);
    }

    #[test]
    fn a_row_without_a_refresh_keeps_its_bias() {
        let mut bodies = Arena::new();
        let handle = RigidBodyHandle(bodies.insert(crate::physics::body::RigidBody::new(
            RigidBodyDesc::dynamic(),
        )));
        let mut rows = vec![row(handle, None)];
        refresh_rows(&bodies, &mut rows);
        assert_eq!(rows[0].bias.to_bits(), 7.0f32.to_bits());
    }
}
