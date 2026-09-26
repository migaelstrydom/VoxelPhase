//! KeepAttitude constraint: hold a body's pitch and roll about its own
//! heading, and leave the heading free.
//!
//! The body's local +Y is its long axis and local +Z its front. The attitude
//! is a pitch that tips the long axis forward, toward the heading:
//!
//! ```text
//!    pitch 0            pitch 90°
//!      +Y                  front
//!      │  front            ───────▶ +Y (long axis, along the heading)
//!      │ ─▶                   │
//!                             ▼ +Z (belly, down)
//! ```
//!
//! Two angular rows lock rotation about the two *horizontal* axes of the
//! heading frame — the lateral axis (pitch) and the heading itself (roll) — so
//! rotation about world up is left to whatever turns the body. At pitch 0 this
//! is `KeepUpright` with +Y as its target. It is a separate constraint because
//! `KeepUpright` frees rotation about its *target*, and once that target leaves
//! the vertical, yaw is one of the two axes it locks: a swimmer held by it
//! could not turn.

use nalgebra::{UnitQuaternion, Vector3};

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;

use super::primitives::{self, BodySide, RowParams};
use super::types::{ConstraintRow, CorrectionMode, Enforcement};

/// Horizontal direction a body at `rotation` is heading, for a body meant to
/// be pitched by `pitch`.
///
/// For a body exactly at its attitude, `sin(pitch)·up + cos(pitch)·front` in
/// its local frame lies along the heading with no vertical part, whatever the
/// pitch. For one near it, its horizontal part is the natural reading of where
/// the body points: its front while upright, its long axis while flat.
pub fn heading(rotation: &UnitQuaternion<f32>, pitch: f32) -> Vector3<f32> {
    let (sin, cos) = pitch.sin_cos();
    let pointing = rotation * (Vector3::y() * sin + Vector3::z() * cos);
    Vector3::new(pointing.x, 0.0, pointing.z)
        .try_normalize(1e-4)
        .or_else(|| {
            let front = rotation * Vector3::z();
            Vector3::new(front.x, 0.0, front.z).try_normalize(1e-4)
        })
        .unwrap_or_else(Vector3::z)
}

/// Yaw of a heading, about +y, with 0 facing +z.
pub fn yaw_of(heading: &Vector3<f32>) -> f32 {
    heading.x.atan2(heading.z)
}

/// The rotation a body heading along `heading` holds when pitched by `pitch`.
pub fn attitude(heading: &Vector3<f32>, pitch: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw_of(heading))
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), pitch)
}

/// How far a body at `rotation` is from its attitude, as a world-space
/// rotation vector restricted to the two locked axes, and those axes:
/// `(error, lateral, heading)`.
///
/// The rotation that would carry the body to its attitude, less whatever part
/// of it is a turn about world up — that part is not the constraint's to
/// correct.
pub fn attitude_error(
    rotation: &UnitQuaternion<f32>,
    pitch: f32,
) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    let heading = heading(rotation, pitch);
    let lateral = Vector3::y().cross(&heading);
    let to_target = attitude(&heading, pitch) * rotation.inverse();
    let error = to_target.scaled_axis();
    let locked = lateral * error.dot(&lateral) + heading * error.dot(&heading);
    (locked, lateral, heading)
}

/// Expand a KeepAttitude constraint into two solver rows.
///
/// Always iterative. The solver's hard projection recovers the held axis from
/// the two rows' cross product and stands the body's +Y along it, which is
/// KeepUpright's semantics; here that cross product is world up whatever the
/// pitch, so projecting would stand a swimmer on end. Drift is corrected in the
/// NGS pass instead (`correct_attitude_drift`).
#[allow(clippy::too_many_arguments)]
pub fn expand(
    body: &RigidBody,
    body_handle: RigidBodyHandle,
    pitch: f32,
    compliance: f32,
    max_impulse: f32,
    dt: f32,
    beta: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> [ConstraintRow; 2] {
    let (error, lateral, heading) = attitude_error(&body.rotation(), pitch);

    let compliance_term = if compliance > 0.0 {
        compliance / (dt * dt)
    } else {
        0.0
    };

    let side_a = BodySide {
        handle: Some(body_handle),
        inv_mass: 0.0,
        inv_inertia: body.world_inv_inertia(),
        lever_arm: Vector3::zeros(),
    };
    let side_b = BodySide::world();

    let row = |axis: Vector3<f32>, index: usize| {
        primitives::lock_angular_axis(
            &side_a,
            &side_b,
            axis,
            -(beta / dt) * error.dot(&axis),
            compliance_term,
            max_impulse,
            CorrectionMode::PositionAndVelocity,
            Enforcement::Iterative,
            &RowParams {
                constraint_index,
                row_index: index,
                warm_impulse: warm_impulses[index],
            },
        )
    };
    [row(lateral, 0), row(heading, 1)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn close(a: Vector3<f32>, b: Vector3<f32>) -> bool {
        (a - b).magnitude() < 1e-4
    }

    /// Upright, the heading is the body's front, as it has always been read.
    #[test]
    fn an_upright_body_heads_where_it_faces() {
        let yaw = 0.7;
        let rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw);
        let h = heading(&rotation, 0.0);
        assert!((yaw_of(&h) - yaw).abs() < 1e-5);
    }

    /// Flat, the front points at the floor and the heading is the long axis.
    #[test]
    fn a_flat_body_heads_along_its_length() {
        let yaw: f32 = -1.2;
        let flat = attitude(&Vector3::new(yaw.sin(), 0.0, yaw.cos()), FRAC_PI_2);
        assert!(close(flat * Vector3::z(), -Vector3::y()), "belly down");
        let h = heading(&flat, FRAC_PI_2);
        assert!((yaw_of(&h) - yaw).abs() < 1e-5);
    }

    /// Every pitch between reads the same heading from its own attitude.
    #[test]
    fn heading_survives_every_pitch() {
        let h = Vector3::new(0.6, 0.0, -0.8);
        for step in 0..=10 {
            let pitch = FRAC_PI_2 * step as f32 / 10.0;
            let r = attitude(&h, pitch);
            assert!(close(heading(&r, pitch), h), "pitch {pitch}");
            assert!(attitude_error(&r, pitch).0.magnitude() < 1e-5);
        }
    }

    /// A turn about world up is not an error at any pitch. That is the whole
    /// difference from KeepUpright.
    #[test]
    fn a_turn_is_never_an_error() {
        for pitch in [0.0, 0.6, FRAC_PI_2] {
            let r = attitude(&Vector3::z(), pitch);
            let turned = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.9) * r;
            assert!(attitude_error(&turned, pitch).0.magnitude() < 1e-5);
        }
    }

    /// Off in pitch, the error is about the lateral axis and carries the body
    /// back to its attitude.
    #[test]
    fn a_pitch_error_turns_the_body_back() {
        let target = attitude(&Vector3::x(), 1.0);
        let off = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.2) * target;
        let (error, lateral, _) = attitude_error(&off, 1.0);
        assert!(
            error.cross(&lateral).magnitude() < 1e-4,
            "about the lateral axis"
        );
        let corrected = UnitQuaternion::from_scaled_axis(error) * off;
        assert!(attitude_error(&corrected, 1.0).0.magnitude() < 1e-4);
    }
}
