//! Primitive row builders: reusable functions that emit a single `ConstraintRow`
//! with correct Jacobians, effective mass, and metadata. Joint types compose
//! these to build their full row sets.

use nalgebra::{Matrix3, Vector3};

use crate::physics::handle::RigidBodyHandle;

use super::types::{ConstraintRow, CorrectionMode, Enforcement, RowKind};

/// One side of a constraint (body A or body B). `None` means world-anchored.
pub struct BodySide {
    pub handle: Option<RigidBodyHandle>,
    pub inv_mass: f32,
    pub inv_inertia: Matrix3<f32>,
    /// Lever arm from body center to constraint anchor, in world space.
    /// Zero for pure angular rows or world-anchored sides.
    pub lever_arm: Vector3<f32>,
}

impl BodySide {
    /// World-anchored side (no body). Used when one end of the constraint is
    /// fixed in world space.
    pub fn world() -> Self {
        Self {
            handle: None,
            inv_mass: 0.0,
            inv_inertia: Matrix3::zeros(),
            lever_arm: Vector3::zeros(),
        }
    }
}

/// Common parameters shared by all primitive row builders.
pub struct RowParams {
    pub constraint_index: generational_arena::Index,
    pub row_index: usize,
    pub warm_impulse: f32,
}

/// Pin two anchor points together along one world-space axis.
///
/// Produces a single linear constraint row that drives the separation between
/// two anchor points to zero along `axis`. When `side_a` is world-anchored,
/// the row pins `side_b`'s anchor to a fixed world position.
///
/// Jacobians: `lin_jac = ±axis`, `ang_jac = lever_arm × axis`.
/// Metadata: `PositionAndVelocity`, `Linear`, `Iterative`.
pub fn lock_linear_axis(
    side_a: &BodySide,
    side_b: &BodySide,
    axis: Vector3<f32>,
    bias: f32,
    compliance_term: f32,
    max_impulse: f32,
    params: &RowParams,
) -> ConstraintRow {
    let zeros = Vector3::zeros();

    let ang_jac_a = if side_a.handle.is_some() {
        -(side_a.lever_arm.cross(&axis))
    } else {
        zeros
    };
    let ang_jac_b = if side_b.handle.is_some() {
        side_b.lever_arm.cross(&axis)
    } else {
        zeros
    };

    let ang_term_a = (side_a.inv_inertia * ang_jac_a).dot(&ang_jac_a);
    let ang_term_b = (side_b.inv_inertia * ang_jac_b).dot(&ang_jac_b);
    let eff_mass =
        1.0 / (side_a.inv_mass + ang_term_a + side_b.inv_mass + ang_term_b + compliance_term);

    let lin_jac_a = if side_a.handle.is_some() {
        -axis
    } else {
        zeros
    };
    let lin_jac_b = if side_b.handle.is_some() { axis } else { zeros };

    ConstraintRow {
        body_a: side_a.handle,
        body_b: side_b.handle,
        lin_jac_a,
        ang_jac_a,
        lin_jac_b,
        ang_jac_b,
        effective_mass_inv: eff_mass,
        bias,
        softness: compliance_term,
        accumulated_impulse: params.warm_impulse,
        bounds: (-max_impulse, max_impulse),
        constraint_index: params.constraint_index,
        row_index: params.row_index,
        correction_mode: CorrectionMode::PositionAndVelocity,
        row_kind: RowKind::Linear,
        enforcement: Enforcement::Iterative,
    }
}

/// Drive one world-space axis of a body's linear velocity toward a target.
///
/// The world-anchored motor row: the reaction lands on nothing, which is what
/// a medium anchor declares. `bias` carries the negated target, so solving
/// `J·v + bias = 0` drives `v·axis` to the target, and `max_impulse` is the
/// authority the actuator declared.
///
/// Jacobians: `lin_jac_b = axis`, no angular term — the row acts through the
/// centre of mass and so induces no torque.
/// Metadata: `VelocityOnly` (a velocity target has no position error to
/// correct), `Linear`, `Iterative`.
pub fn drive_linear_axis(
    side: &BodySide,
    axis: Vector3<f32>,
    bias: f32,
    max_impulse: f32,
    params: &RowParams,
) -> ConstraintRow {
    let zeros = Vector3::zeros();

    ConstraintRow {
        body_a: None,
        body_b: side.handle,
        lin_jac_a: zeros,
        ang_jac_a: zeros,
        lin_jac_b: axis,
        ang_jac_b: zeros,
        effective_mass_inv: 1.0 / side.inv_mass,
        bias,
        softness: 0.0,
        accumulated_impulse: params.warm_impulse,
        bounds: (-max_impulse, max_impulse),
        constraint_index: params.constraint_index,
        row_index: params.row_index,
        correction_mode: CorrectionMode::VelocityOnly,
        row_kind: RowKind::Linear,
        enforcement: Enforcement::Iterative,
    }
}

/// Drive one world-space axis of a body's angular velocity toward a target.
///
/// The angular half of [`drive_linear_axis`], with the same conventions: the
/// world is the other side of the row, `bias` is the negated target spin, and
/// `max_impulse` is the declared authority.
pub fn drive_angular_axis(
    side: &BodySide,
    axis: Vector3<f32>,
    bias: f32,
    max_impulse: f32,
    params: &RowParams,
) -> ConstraintRow {
    lock_angular_axis(
        &BodySide::world(),
        side,
        axis,
        bias,
        0.0,
        max_impulse,
        CorrectionMode::VelocityOnly,
        Enforcement::Iterative,
        params,
    )
}

/// Lock relative orientation around one world-space axis.
///
/// Produces a single angular constraint row. For one-body constraints the
/// axis appears on whichever side has a body. For two-body constraints the
/// Jacobians are `ang_jac_a = -axis`, `ang_jac_b = +axis`.
///
/// Jacobians: pure angular (no linear component).
pub fn lock_angular_axis(
    side_a: &BodySide,
    side_b: &BodySide,
    axis: Vector3<f32>,
    bias: f32,
    compliance_term: f32,
    max_impulse: f32,
    correction_mode: CorrectionMode,
    enforcement: Enforcement,
    params: &RowParams,
) -> ConstraintRow {
    let zeros = Vector3::zeros();

    // Jacobian sign convention: body_a gets -axis, body_b gets +axis.
    // For one-body constraints, the absent side gets zero.
    let ang_jac_a = if side_a.handle.is_some() {
        if side_b.handle.is_some() {
            -axis
        } else {
            axis
        }
    } else {
        zeros
    };
    let ang_jac_b = if side_b.handle.is_some() {
        if side_a.handle.is_some() {
            axis
        } else {
            axis
        }
    } else {
        zeros
    };

    let ang_term_a = (side_a.inv_inertia * ang_jac_a).dot(&ang_jac_a);
    let ang_term_b = (side_b.inv_inertia * ang_jac_b).dot(&ang_jac_b);
    let eff_mass = 1.0 / (ang_term_a + ang_term_b + compliance_term);

    ConstraintRow {
        body_a: side_a.handle,
        body_b: side_b.handle,
        lin_jac_a: zeros,
        ang_jac_a,
        lin_jac_b: zeros,
        ang_jac_b,
        effective_mass_inv: eff_mass,
        bias,
        softness: compliance_term,
        accumulated_impulse: params.warm_impulse,
        bounds: (-max_impulse, max_impulse),
        constraint_index: params.constraint_index,
        row_index: params.row_index,
        correction_mode,
        row_kind: RowKind::Angular,
        enforcement,
    }
}
