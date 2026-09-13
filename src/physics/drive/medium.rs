//! The medium anchor: a drive whose reaction goes into the world.
//!
//! A platform's motor is not a foot. It pushes against nothing and is entitled
//! to, provided it says so — `ReactionAnchor::Medium` is that statement. What
//! it expands to is ordinary constraint rows with `body_a: None`, the
//! world-anchored form the constraint system already uses for `world_fixed`
//! and `world_hinge`: six motor rows, three linear and three angular, each
//! driving one world axis of the body's velocity toward the commanded target.
//!
//! ```text
//!   DriveCommand ──► expand() ──► 6 × ConstraintRow ──► PgsNgsSolver
//!        (target, authority)          (bias, bounds)
//! ```
//!
//! Two properties are the whole point of the shape:
//!
//! - **The target is a bias, so the row is solved alongside gravity** rather
//!   than before it. The lift does not sag by a frame of gravity and then
//!   recover; it simply holds its speed, because the motor and the weight are
//!   two rows in the same solve.
//! - **The authority is a bound, so the actuator's `max_accel` means
//!   something.** Each row may accumulate at most the impulse that changes the
//!   body's velocity along its axis by `max_accel · dt` — no more, whether the
//!   load is a passenger, a crate, or nothing at all.
//!
//! The bound is per substep: rows are expanded once per frame, but the
//! accumulated impulse is rescaled by the warm start at the top of every
//! substep, so a row pinned at its bound delivers exactly `max_accel · dt`
//! per substep and the declared acceleration over the frame. A platform with
//! acceleration to spare never reaches its bound outside the first frames of
//! spin-up and so never sees it.

use nalgebra::Vector3;
use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::constraint::primitives::{self, BodySide, RowParams};
use crate::physics::constraint::types::ConstraintRow;
use crate::physics::handle::RigidBodyHandle;

/// Rows a medium anchor expands to: three linear, three angular.
///
/// The angular rows are emitted even when the commanded spin is zero, because
/// a zero angular target is a command to *hold still*, not the absence of a
/// command — a platform told not to yaw is being told something. An actuator
/// that declares no angular authority gets rows bounded at zero, which cost
/// the solve nothing and say the same thing.
pub const ROW_COUNT: usize = 6;

/// Expand a medium-anchored drive into its solver rows.
///
/// Returns nothing for a body with no mass to accelerate — a static or
/// kinematic body has no velocity the solver may change, so a motor aimed at
/// it would be a row with zero effective mass rather than a weak one.
#[allow(clippy::too_many_arguments)]
pub fn expand(
    body: &RigidBody,
    handle: RigidBodyHandle,
    linear_target: &Vector3<f32>,
    angular_target: &Vector3<f32>,
    max_accel: f32,
    angular_max_accel: f32,
    dt: f32,
    constraint_index: generational_arena::Index,
    warm_impulses: &[f32],
) -> SmallVec<[ConstraintRow; ROW_COUNT]> {
    let mut rows = SmallVec::new();

    let inv_mass = body.inv_mass();
    if inv_mass <= 0.0 {
        return rows;
    }

    let side = BodySide {
        handle: Some(handle),
        inv_mass,
        inv_inertia: body.world_inv_inertia(),
        lever_arm: Vector3::zeros(),
    };
    let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

    // A motor row drives `J·v + bias` to zero, so the bias is the negated
    // target. The bound is the impulse that moves this body's velocity along
    // the axis by exactly the actuator's declared acceleration times `dt`,
    // which for a row through the centre of mass is `accel · dt · m_eff`.
    let linear_authority = (max_accel * dt / inv_mass).max(0.0);
    for (i, axis) in axes.iter().enumerate() {
        rows.push(primitives::drive_linear_axis(
            &side,
            *axis,
            -linear_target.dot(axis),
            linear_authority,
            &RowParams {
                constraint_index,
                row_index: i,
                warm_impulse: warm_impulses[i],
            },
        ));
    }

    for (i, axis) in axes.iter().enumerate() {
        let inv_inertia_axis = (side.inv_inertia * axis).dot(axis);
        let angular_authority = if inv_inertia_axis > 0.0 {
            (angular_max_accel * dt / inv_inertia_axis).max(0.0)
        } else {
            0.0
        };
        rows.push(primitives::drive_angular_axis(
            &side,
            *axis,
            -angular_target.dot(axis),
            angular_authority,
            &RowParams {
                constraint_index,
                row_index: 3 + i,
                warm_impulse: warm_impulses[3 + i],
            },
        ));
    }

    rows
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;
    use crate::physics::body::RigidBodyDesc;
    use crate::physics::collider::ColliderDesc;
    use crate::physics::constraint::types::{ConstraintKind, CorrectionMode, RowKind};
    use crate::physics::world::PhysicsWorld;

    const DT: f32 = 1.0 / 240.0;

    /// A 2 m cube of unit density: mass 8 kg, inertia 2/3 · m · half² per axis.
    fn boxed_body(world: &mut PhysicsWorld) -> RigidBodyHandle {
        let handle = world.create_body(RigidBodyDesc::dynamic().position(Point3::origin()));
        let _ = world.attach_collider(
            handle,
            ColliderDesc::box_shape(Vector3::new(1.0, 1.0, 1.0)).density(1.0),
        );
        handle
    }

    fn expand_for(
        world: &PhysicsWorld,
        handle: RigidBodyHandle,
        linear: Vector3<f32>,
        angular: Vector3<f32>,
        max_accel: f32,
        angular_max_accel: f32,
    ) -> SmallVec<[ConstraintRow; ROW_COUNT]> {
        let index = generational_arena::Arena::<()>::new().insert(());
        expand(
            world.body(handle).unwrap(),
            handle,
            &linear,
            &angular,
            max_accel,
            angular_max_accel,
            DT,
            index,
            &[0.0; ROW_COUNT],
        )
    }

    #[test]
    fn the_bias_is_the_negated_target_on_every_axis() {
        let mut world = PhysicsWorld::default();
        let handle = boxed_body(&mut world);
        let rows = expand_for(
            &world,
            handle,
            Vector3::new(1.0, -2.0, 3.0),
            Vector3::new(0.0, 0.5, 0.0),
            10.0,
            10.0,
        );

        assert_eq!(rows.len(), ROW_COUNT);
        assert_eq!(
            [rows[0].bias, rows[1].bias, rows[2].bias],
            [-1.0, 2.0, -3.0]
        );
        assert_eq!([rows[3].bias, rows[4].bias, rows[5].bias], [0.0, -0.5, 0.0]);
    }

    #[test]
    fn the_authority_is_exactly_what_the_actuator_declares() {
        let mut world = PhysicsWorld::default();
        let handle = boxed_body(&mut world);
        let body = world.body(handle).unwrap();
        let mass = body.mass();
        let inertia = 1.0 / (body.world_inv_inertia() * Vector3::y()).dot(&Vector3::y());

        let rows = expand_for(
            &world,
            handle,
            Vector3::new(50.0, 50.0, 50.0),
            Vector3::new(50.0, 50.0, 50.0),
            4.0,
            7.0,
        );

        for row in rows.iter().take(3) {
            assert!((row.bounds.1 - 4.0 * mass * DT).abs() < 1e-6);
            assert_eq!(row.bounds.0, -row.bounds.1);
        }
        for row in rows.iter().skip(3) {
            assert!((row.bounds.1 - 7.0 * inertia * DT).abs() < 1e-4);
            assert_eq!(row.bounds.0, -row.bounds.1);
        }
    }

    #[test]
    fn no_declared_angular_authority_means_rows_that_cannot_act() {
        let mut world = PhysicsWorld::default();
        let handle = boxed_body(&mut world);
        let rows = expand_for(&world, handle, Vector3::zeros(), Vector3::zeros(), 4.0, 0.0);

        assert_eq!(rows.len(), ROW_COUNT);
        for row in rows.iter().skip(3) {
            assert_eq!(row.bounds, (0.0, 0.0));
        }
    }

    #[test]
    fn a_motor_row_never_asks_for_a_position_correction() {
        let mut world = PhysicsWorld::default();
        let handle = boxed_body(&mut world);
        let rows = expand_for(
            &world,
            handle,
            Vector3::new(0.0, 2.0, 0.0),
            Vector3::zeros(),
            10.0,
            10.0,
        );

        for row in rows.iter() {
            assert_eq!(row.correction_mode, CorrectionMode::VelocityOnly);
            assert!(row.body_a.is_none(), "the world is the other side");
        }
        assert_eq!(rows[0].row_kind, RowKind::Linear);
        assert_eq!(rows[3].row_kind, RowKind::Angular);
    }

    #[test]
    fn a_body_with_no_mass_to_move_gets_no_rows() {
        let mut world = PhysicsWorld::default();
        let handle = world.create_body(RigidBodyDesc::static_body().position(Point3::origin()));
        let rows = expand_for(&world, handle, Vector3::y(), Vector3::zeros(), 10.0, 10.0);
        assert!(rows.is_empty());
    }

    #[test]
    fn the_kind_declares_the_row_count_it_expands_to() {
        assert_eq!(
            ConstraintKind::MediumDrive {
                body: RigidBodyHandle(generational_arena::Arena::<()>::new().insert(())),
                linear_target: Vector3::zeros(),
                angular_target: Vector3::zeros(),
                max_accel: 1.0,
                angular_max_accel: 1.0,
            }
            .row_count(),
            ROW_COUNT
        );
    }
}
