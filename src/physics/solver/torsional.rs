//! Torsional impulse resolution for contact constraints.
//!
//! The same idea as the tangential row, about the contact normal instead of
//! within the tangent plane: drive the relative spin about the normal toward a
//! target under a torsional budget. Torsional friction — spin resistance — is
//! the case where that target is zero, and a character turning on the spot
//! would be the case where it is not.
//!
//! **The bound has a term the engine cannot supply.** Torsional friction is
//! `μ·N·r`, where `r` is the radius of the contact patch. A `SolverContact` is
//! a point, and the honest patch radius of a capsule on terrain is very nearly
//! zero — so this row is inert for everything in the game unless an entity
//! declares otherwise through `Actuator::support_patch_radius`. That is not a
//! gap to be filled by inventing a radius: where a manifold really does have
//! several contacts, they already resist spin through their own tangential
//! rows at their own lever arms, and a torsional row over the same patch would
//! count the same friction twice. The radius belongs to bodies whose supports
//! are wider than the contacts standing for them — a turntable, a tracked
//! vehicle, a body lying prone — and only its owner knows that.
//!
//! See `docs/TRACTION_DRIVE_DESIGN.md` §6.2, which concludes from the same
//! three facts that the player's ground yaw is an Allowance and not this row.
//!
//! **Nothing in the game declares a patch radius, and the row is kept anyway.**
//! Stage 7's audit weighed deleting it as surface with no consumer and did not:
//! it is R10's only literal delivery — the requirement that linear and angular
//! authority share one formulation — it is covered end to end by
//! `a_declared_contact_patch_is_what_a_torsional_row_turns_on`, and it is a
//! parameter rather than a drive-aware branch. Its cost where nobody wants it
//! is one comparison against a zero radius per contact per iteration. Deleting
//! it would leave the requirement satisfied by argument alone, and re-deriving
//! it for the first turntable would be strictly more work than keeping it.

use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::contact_row::ContactRow;
use super::solver_bodies::SolverBodies;

/// Smallest effective inverse inertia a torsional row will solve against.
///
/// Two bodies that both resist spin about this normal absolutely — a pair of
/// static or infinitely yaw-inert bodies — divide by nothing.
const MIN_EFFECTIVE_INV_INERTIA: f32 = 1e-9;

/// Solve one contact's torsional row toward the relative spin the drive asks
/// for about the contact normal.
///
/// Bounded by `μ · N · r`, so an unloaded contact and a contact standing for no
/// patch both spend nothing. The early return on `patch_radius` is what keeps
/// this free for the whole game: no declaration, no row, no cost.
///
/// `row` is `None` when a body of the pair is gone.
pub(crate) fn solve_torsional_impulse(
    bodies: &mut SolverBodies,
    row: Option<&ContactRow>,
    header: &PairHeader,
    contact: &mut SolverContact,
) {
    let radius = contact.traction.patch_radius;
    if radius <= 0.0 || contact.accumulated_normal_impulse <= 0.0 {
        contact.accumulated_torsional_impulse = 0.0;
        return;
    }
    let mu = header.friction * contact.tangential_scale * contact.traction.gain;
    if mu <= 0.0 {
        contact.accumulated_torsional_impulse = 0.0;
        return;
    }

    let Some(row) = row else {
        return;
    };

    let normal = contact.normal;
    let effective_inv_inertia = row.torsional_inv_inertia;
    if effective_inv_inertia <= MIN_EFFECTIVE_INV_INERTIA || !effective_inv_inertia.is_finite() {
        return;
    }

    let relative_spin = row.relative_angular_velocity(bodies).dot(&normal);
    let delta = (contact.traction.target_spin - relative_spin) / effective_inv_inertia;

    let max_torsional = mu * contact.accumulated_normal_impulse * radius;
    let previous = contact.accumulated_torsional_impulse;
    let accumulated = (previous + delta).clamp(-max_torsional, max_torsional);
    contact.accumulated_torsional_impulse = accumulated;

    let applied = accumulated - previous;
    if applied.abs() <= 1e-10 {
        return;
    }

    row.apply_angular_impulse(bodies, normal * applied);
}

#[cfg(test)]
mod tests {
    use generational_arena::Arena;
    use nalgebra::{Matrix3, Point3, Vector3};

    use super::*;
    use crate::collision::contact::FeatureId;
    use crate::physics::body::{RigidBody, RigidBodyDesc};
    use crate::physics::drive::TractionRow;
    use crate::physics::handle::RigidBodyHandle;

    /// A contact on level ground carrying a full normal impulse, so the bound
    /// is decided by `μ` and `r` alone.
    fn contact(row: TractionRow) -> SolverContact {
        SolverContact {
            point: Point3::origin(),
            normal: Vector3::y(),
            raw_normal: Vector3::y(),
            depth: 0.0,
            raw_depth: 0.0,
            gap: 0.0,
            closing_allowance: 0.0,
            feature_id: FeatureId(0),
            warm_normal_impulse: 0.0,
            warm_friction_impulse_ws: Vector3::zeros(),
            accumulated_normal_impulse: 10.0,
            accumulated_friction_impulse_ws: Vector3::zeros(),
            accumulated_torsional_impulse: 0.0,
            tangential_scale: 1.0,
            traction: row,
        }
    }

    /// One unit-inertia body on static ground.
    fn ground_pair() -> (Arena<RigidBody>, PairHeader) {
        let mut bodies = Arena::new();
        let mut body = RigidBody::new(RigidBodyDesc::dynamic());
        body.set_mass_properties(1.0, Matrix3::identity());
        let index = bodies.insert(body);
        let header = PairHeader {
            body_a: None,
            body_b: RigidBodyHandle(index),
            collider_a: None,
            collider_b: None,
            restitution: 0.0,
            friction: 0.5,
        };
        (bodies, header)
    }

    /// One solve of the row, prepared from the bodies as they are now and
    /// written back to them.
    fn solve(bodies: &mut Arena<RigidBody>, header: &PairHeader, contact: &mut SolverContact) {
        let mut solver_bodies = SolverBodies::default();
        let row = ContactRow::prepare(bodies, &mut solver_bodies, header, contact, (1.0, 1.0));
        solve_torsional_impulse(&mut solver_bodies, row.as_ref(), header, contact);
        solver_bodies.scatter(bodies);
    }

    fn spin(bodies: &Arena<RigidBody>, header: &PairHeader) -> f32 {
        bodies.get(header.body_b.0).unwrap().angular_velocity().y
    }

    /// A contact standing for no patch has no torsional authority, which is
    /// every contact in the game.
    #[test]
    fn a_point_contact_has_no_torsional_row() {
        let (mut bodies, header) = ground_pair();
        let mut contact = contact(TractionRow {
            target_spin: 4.0,
            ..Default::default()
        });
        solve(&mut bodies, &header, &mut contact);
        assert_eq!(spin(&bodies, &header), 0.0);
    }

    /// Given a patch to push against, the row drives the commanded spin.
    #[test]
    fn a_declared_patch_turns_the_body_toward_its_target() {
        let (mut bodies, header) = ground_pair();
        let mut contact = contact(TractionRow {
            target_spin: 1.0,
            patch_radius: 0.5,
            ..Default::default()
        });
        for _ in 0..8 {
            solve(&mut bodies, &header, &mut contact);
        }
        assert!((spin(&bodies, &header) - 1.0).abs() < 1e-4);
    }

    /// And no further than `μ·N·r` will carry it.
    #[test]
    fn the_row_is_bounded_by_the_torsional_budget() {
        let (mut bodies, header) = ground_pair();
        let mut contact = contact(TractionRow {
            target_spin: 100.0,
            patch_radius: 0.5,
            ..Default::default()
        });
        for _ in 0..8 {
            solve(&mut bodies, &header, &mut contact);
        }
        // μ·N·r = 0.5 · 10 · 0.5 = 2.5 N·m·s, into a unit inertia.
        assert!((spin(&bodies, &header) - 2.5).abs() < 1e-4);
        assert!((contact.accumulated_torsional_impulse - 2.5).abs() < 1e-4);
    }

    /// With no target it is torsional friction: a spinning body is slowed by
    /// the patch it is spinning on, by no more than the same budget.
    #[test]
    fn a_zero_target_is_torsional_friction() {
        let (mut bodies, header) = ground_pair();
        bodies
            .iter_mut()
            .next()
            .unwrap()
            .1
            .set_angular_velocity(Vector3::new(0.0, 5.0, 0.0));
        let mut contact = contact(TractionRow {
            patch_radius: 0.5,
            ..Default::default()
        });
        solve(&mut bodies, &header, &mut contact);
        assert!((spin(&bodies, &header) - 2.5).abs() < 1e-4);
    }
}
