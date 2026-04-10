//! Constraint expansion: converts persistent constraint definitions into solver-ready rows.

use generational_arena::Arena;

use super::anchor_point;
use super::fixed;
use super::follow_point;
use super::hinge;
use super::keep_upright;
use super::types::{Constraint, ConstraintKind, ConstraintRow};
use crate::physics::body::RigidBody;

/// Expand all active constraints into solver-ready rows.
///
/// Clears `rows` and refills it from the constraint arena. Warm-start
/// impulses are copied from each constraint's cached values.
///
/// `beta` is the position correction factor (from solver config).
pub fn expand_constraints(
    constraints: &Arena<Constraint>,
    bodies: &Arena<RigidBody>,
    dt: f32,
    beta: f32,
    rows: &mut Vec<ConstraintRow>,
) {
    rows.clear();

    for (index, constraint) in constraints.iter() {
        if !constraint.active {
            continue;
        }

        match &constraint.kind {
            ConstraintKind::KeepUpright {
                body,
                target_up,
                compliance,
            } => {
                let Some(rigid_body) = bodies.get(body.0) else {
                    continue;
                };
                let expanded = keep_upright::expand(
                    rigid_body,
                    *body,
                    target_up,
                    *compliance,
                    dt,
                    beta,
                    index,
                    &constraint.warm_impulses,
                );
                rows.extend(expanded);
            }

            ConstraintKind::AnchorPoint {
                body,
                local_anchor,
                world_anchor,
                compliance,
                max_impulse,
                lock_yaw,
                lock_roll,
            } => {
                let Some(rigid_body) = bodies.get(body.0) else {
                    continue;
                };
                let expanded = anchor_point::expand(
                    rigid_body,
                    *body,
                    local_anchor,
                    world_anchor,
                    *compliance,
                    *max_impulse,
                    *lock_yaw,
                    *lock_roll,
                    dt,
                    beta,
                    index,
                    &constraint.warm_impulses,
                );
                rows.extend(expanded);
            }

            ConstraintKind::Fixed {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                compliance,
                max_impulse,
            } => {
                let Some(rigid_body_b) = bodies.get(body_b.0) else {
                    continue;
                };
                let body_a_pair = body_a.and_then(|ha| {
                    bodies.get(ha.0).map(|ba| (ba, ha))
                });
                if body_a.is_some() && body_a_pair.is_none() {
                    continue;
                }
                let expanded = fixed::expand(
                    body_a_pair,
                    rigid_body_b,
                    *body_b,
                    local_anchor_a,
                    local_anchor_b,
                    *compliance,
                    *max_impulse,
                    dt,
                    beta,
                    index,
                    &constraint.warm_impulses,
                );
                rows.extend(expanded);
            }

            ConstraintKind::Hinge {
                body_a,
                body_b,
                local_anchor_a,
                local_anchor_b,
                local_axis_b,
                local_axis_a,
                local_ref_b,
                compliance,
                max_impulse,
            } => {
                let Some(rigid_body_b) = bodies.get(body_b.0) else {
                    continue;
                };
                let body_a_pair = body_a.and_then(|ha| {
                    bodies.get(ha.0).map(|ba| (ba, ha))
                });
                if body_a.is_some() && body_a_pair.is_none() {
                    continue;
                }
                let expanded = hinge::expand(
                    body_a_pair,
                    rigid_body_b,
                    *body_b,
                    local_anchor_a,
                    local_anchor_b,
                    local_axis_a,
                    local_axis_b,
                    local_ref_b,
                    *compliance,
                    *max_impulse,
                    dt,
                    beta,
                    index,
                    &constraint.warm_impulses,
                );
                rows.extend(expanded);
            }

            ConstraintKind::FollowPoint {
                body_a,
                local_anchor_a,
                body_b,
                local_anchor_b,
                compliance,
                max_impulse,
                relative_orientation,
                angular_compliance,
                angular_max_impulse,
            } => {
                let Some(rigid_body_a) = bodies.get(body_a.0) else {
                    continue;
                };
                let Some(rigid_body_b) = bodies.get(body_b.0) else {
                    continue;
                };
                let expanded = follow_point::expand(
                    rigid_body_a,
                    *body_a,
                    rigid_body_b,
                    *body_b,
                    local_anchor_a,
                    local_anchor_b,
                    *compliance,
                    *max_impulse,
                    relative_orientation,
                    *angular_compliance,
                    *angular_max_impulse,
                    dt,
                    index,
                    &constraint.warm_impulses,
                );
                rows.extend(expanded);
            }
        }
    }
}

/// Write solved impulses back from constraint rows to persistent constraints.
///
/// Called after each substep so warm-start values are available for the next
/// substep (and the next frame).
pub fn write_back_constraints(constraints: &mut Arena<Constraint>, rows: &[ConstraintRow]) {
    for row in rows {
        if let Some(constraint) = constraints.get_mut(row.constraint_index) {
            if row.row_index < constraint.warm_impulses.len() {
                constraint.warm_impulses[row.row_index] = row.accumulated_impulse;
            }
        }
    }
}

/// Deactivate breakable constraints whose rows have saturated.
///
/// A constraint is breakable when its rows have finite impulse bounds
/// (max_impulse < f32::MAX). When any row's accumulated impulse reaches the
/// bound, the joint cannot provide enough force and is permanently deactivated.
pub fn check_constraint_breakage(constraints: &mut Arena<Constraint>, rows: &[ConstraintRow]) {
    const SATURATION_THRESHOLD: f32 = 0.999;

    for row in rows {
        let max_bound = row.bounds.1;
        if max_bound >= f32::MAX * 0.5 {
            continue;
        }
        if row.accumulated_impulse.abs() >= max_bound * SATURATION_THRESHOLD {
            if let Some(constraint) = constraints.get_mut(row.constraint_index) {
                constraint.active = false;
            }
        }
    }
}
