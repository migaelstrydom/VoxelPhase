//! A suspension's rows: spring, friction pair, torsion, in that order.

use nalgebra::{Isometry3, Point3};
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::constraint::primitives::{self, BodySide, RowParams};
use crate::physics::constraint::types::{
    ConstraintRow, CorrectionMode, CoupledBound, Enforcement, RowRefresh,
};
use crate::physics::handle::RigidBodyHandle;

use super::state::Suspension;

/// Rows a suspension expands to: spring, two friction rows, torsion.
pub const ROW_COUNT: usize = 4;

/// Row index of the spring, which the other three are coupled to.
pub(super) const SPRING_ROW: usize = 0;
/// Row indices of the friction pair.
const FRICTION_ROWS: [usize; 2] = [1, 2];
/// Row index of the torsion row.
const TORSION_ROW: usize = 3;

/// One side of a suspension's rows, with its lever arm to `point`.
fn side(handle: RigidBodyHandle, body: &RigidBody, point: Point3<f32>) -> BodySide {
    BodySide {
        handle: Some(handle),
        inv_mass: body.inv_mass(),
        inv_inertia: body.world_inv_inertia(),
        lever_arm: point - body.position(),
    }
}

/// The partner's side of a suspension's rows: the world when it is static.
fn partner_side(partner: Option<(&RigidBody, RigidBodyHandle)>, point: Point3<f32>) -> BodySide {
    match partner {
        Some((body, handle)) => side(handle, body, point),
        None => BodySide::world(),
    }
}

/// Where a body is, for a refreshed row to measure its motion from.
fn pose(body: &RigidBody) -> Isometry3<f32> {
    Isometry3::from_parts(body.position().coords.into(), body.rotation())
}

/// Expand a suspension into its rows: spring, friction pair, torsion.
///
/// Nothing when the cast hit nothing. `partner` is the body hit, `None` for
/// static geometry (or when it is gone, which reads the same).
pub fn expand(
    suspension: &Suspension,
    body: &RigidBody,
    partner: Option<(&RigidBody, RigidBodyHandle)>,
    dt: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> SmallVec<[ConstraintRow; ROW_COUNT]> {
    let mut rows = SmallVec::new();
    let Some(&hit) = suspension.hit() else {
        return rows;
    };
    let desc = suspension.desc();
    let handle = desc.body;
    let params = |row_index: usize| RowParams {
        constraint_index,
        row_index,
        warm_impulse: warm_impulses.get(row_index).copied().unwrap_or(0.0),
    };
    let coupled = |cone_with: Option<usize>, coefficient: f32, load_share: f32| CoupledBound {
        coupled_to: SPRING_ROW as u32,
        cone_with: cone_with.map(|row| row as u32),
        coefficient,
        load_share,
    };

    // The spring: along the axis through the anchor, and through the cast
    // sphere's centre on the partner.
    let anchor = suspension.anchor_in_world(body);
    let soft = desc.spring.soft_step(body.mass(), dt);
    let error = hit.length - desc.rest_length;
    let mut spring = primitives::lock_linear_axis(
        &partner_side(partner, hit.sphere_centre()),
        &side(handle, body, anchor),
        hit.axis,
        soft.bias_rate * error,
        soft.softness,
        desc.max_force * dt,
        &params(SPRING_ROW),
    );
    spring.bounds = (0.0, desc.max_force * dt);
    spring.correction_mode = CorrectionMode::VelocityOnly;
    spring.refresh = Some(RowRefresh {
        error,
        bias_rate: soft.bias_rate,
        pose_a: partner.map(|(partner, _)| pose(partner)),
        pose_b: Some(pose(body)),
    });
    rows.push(spring);

    // Friction and torsion: at the hit point, bounded by the spring's load.
    let partner_at_hit = partner_side(partner, hit.point);
    let body_at_hit = side(handle, body, hit.point);
    let (t1, t2) = hit.tangents();
    let mu = hit.friction * hit.load_factor();
    for (row_index, tangent, other) in [
        (FRICTION_ROWS[0], t1, FRICTION_ROWS[1]),
        (FRICTION_ROWS[1], t2, FRICTION_ROWS[0]),
    ] {
        let mut row = primitives::lock_linear_axis(
            &partner_at_hit,
            &body_at_hit,
            tangent,
            0.0,
            0.0,
            0.0,
            &params(row_index),
        );
        row.correction_mode = CorrectionMode::VelocityOnly;
        row.coupling = Some(coupled(Some(other), mu, hit.axis.dot(&tangent)));
        rows.push(row);
    }

    let mut torsion = primitives::lock_angular_axis(
        &partner_at_hit,
        &body_at_hit,
        hit.normal,
        0.0,
        0.0,
        0.0,
        CorrectionMode::VelocityOnly,
        Enforcement::Iterative,
        &params(TORSION_ROW),
    );
    torsion.coupling = Some(coupled(None, mu * desc.torsion_radius, 0.0));
    rows.push(torsion);

    rows
}
