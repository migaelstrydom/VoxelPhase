//! Post-solve angular velocity projection for hard constraint enforcement.
//!
//! PGS constraint rows use linearized Jacobians that lose effectiveness at
//! large tilt angles and can't fully override friction torques with limited
//! iterations. This module provides a hard post-solve projection that removes
//! angular velocity components forbidden by the constraint, guaranteeing the
//! constraint is satisfied regardless of solver iteration count.

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;

use super::types::{Constraint, ConstraintKind};

/// Project angular velocities so that hard constraints are exactly satisfied.
///
/// For KeepUpright: removes all tilt angular velocity (keeps only spin around
/// the target up axis) and injects corrective angular velocity to bring the
/// body back upright when tilted.
///
/// Called once per substep, after the PGS solver and before position
/// integration.
pub fn project_angular_velocities(
    constraints: &Arena<Constraint>,
    bodies: &mut Arena<RigidBody>,
    dt: f32,
    beta: f32,
) {
    for (_index, constraint) in constraints.iter() {
        if !constraint.active {
            continue;
        }

        match &constraint.kind {
            ConstraintKind::KeepUpright {
                body,
                target_up,
                compliance,
            } => {
                if *compliance > 0.0 {
                    continue;
                }

                let Some(body) = bodies.get_mut(body.0) else {
                    continue;
                };

                let up = target_up.into_inner();
                let local_up = body.rotation() * Vector3::y();

                // Preserve only the spin component (rotation around target up)
                let omega = body.angular_velocity();
                let spin = omega.dot(&up) * up;

                // Corrective angular velocity to reduce tilt error.
                // The cross product local_up × target_up gives the rotation
                // axis and its magnitude equals sin(θ), which works correctly
                // at all angles (unlike the linearized dot-product Jacobian).
                let correction_axis = local_up.cross(&up);
                let correction = correction_axis * (beta / dt);

                body.set_angular_velocity(spin + correction);
            }

            ConstraintKind::FollowPoint { .. } => {
                // FollowPoint is a soft positional constraint — no post-solve
                // projection needed. PGS rows alone suffice.
            }
        }
    }
}
