//! Tangential impulse resolution for contact constraints.
//!
//! The row drives the relative tangential velocity at a contact toward a target
//! under the `mu * N` traction budget. Friction is the case where that target is
//! zero.

use nalgebra::Vector3;
use smallvec::SmallVec;

use crate::physics::pipeline::pair::{PairHeader, SolverContact};

use super::contact_row::ContactRow;
use super::diagnostics::log_impulse_torque_diag;
use super::normal::MIN_EFFECTIVE_INV_MASS;
use super::solver_bodies::SolverBodies;

/// What one contact's tangential row may spend, before the normal impulse it
/// is multiplied by: the pair's combined friction, scaled by what its
/// participants are allowed to draw there, then by the gain a body driving
/// through it declared.
///
/// The gain is the design's one sanctioned scalar cheat (§11): above `1.0` a
/// body pushes harder than the surface honestly permits. It reaches only the
/// contacts holding that body up, because that is where the planner put a
/// target — a driven body's other contacts keep the honest bound.
fn tangential_coefficient(header: &PairHeader, contact: &SolverContact) -> f32 {
    honest_tangential_coefficient(header, contact) * contact.traction.gain
}

/// The same coefficient as the surface would give with no drive gain at all —
/// the bound a body with `drive_gain: 1.0` would have had.
///
/// Kept as its own function because the difference between the two is what the
/// gain borrowed, which §11 requires be measurable rather than asserted.
fn honest_tangential_coefficient(header: &PairHeader, contact: &SolverContact) -> f32 {
    header.friction * contact.tangential_scale
}

/// The velocity error one tangential row corrects: how far the relative motion
/// at the contact is from what the drive asks for, measured along one tangent.
///
/// The subtraction happens in world space, before the projection onto the
/// tangent. `target.dot(t) - relative.dot(t)` is the same expression in algebra
/// and a different one in floating point — with a zero target only this form
/// reproduces the plain `-(v_rel . t)` friction row bit for bit. Pinned by
/// `tangential_error_is_not_reassociated`.
fn tangential_error(target: &Vector3<f32>, relative: &Vector3<f32>, tangent: &Vector3<f32>) -> f32 {
    (target - relative).dot(tangent)
}

/// A tangential impulse, as two components in a tangent basis, after being
/// held inside a Coulomb cone.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ConeProjection {
    /// The component along the first tangent.
    pub t1: f32,
    /// The component along the second tangent.
    pub t2: f32,
    /// The impulse's magnitude before it was projected.
    pub magnitude: f32,
    /// Whether it lay outside the cone and was scaled back to its edge.
    pub saturated: bool,
}

/// Hold a tangential impulse `(t1, t2)` inside the cone of radius `radius`:
/// scaled back along itself to the edge when it lies outside, untouched
/// otherwise.
///
/// Bounded as a disc rather than per component, so friction grips as hard on
/// a diagonal as along either tangent. Shared by contact friction and by
/// coupled constraint rows (`coupled_row.rs`).
pub(crate) fn project_onto_cone(t1: f32, t2: f32, radius: f32) -> ConeProjection {
    let magnitude = (t1 * t1 + t2 * t2).sqrt();
    let saturated = magnitude > radius;
    let (t1, t2) = if saturated {
        let scale = radius / magnitude;
        (t1 * scale, t2 * scale)
    } else {
        (t1, t2)
    };
    ConeProjection {
        t1,
        t2,
        magnitude,
        saturated,
    }
}

/// Solve one contact's tangential row toward the Target Relative Velocity the
/// drive asks for at this point. Zero — the default the planner leaves on
/// every contact nothing is driving through — is ordinary friction.
///
/// The early return on an unloaded contact is what makes "no load, no drive"
/// true: an airborne body has no normal impulse anywhere, so it has no drive
/// authority of any kind.
///
/// `row` is `None` when a body of the pair is gone; the bookkeeping for an
/// unloaded contact still happens, and nothing else does.
pub(crate) fn solve_friction_impulse(
    bodies: &mut SolverBodies,
    row: Option<&ContactRow>,
    header: &PairHeader,
    contact: &mut SolverContact,
) {
    let target_relative_velocity = &contact.traction.target;
    let mu = tangential_coefficient(header, contact);
    if contact.accumulated_normal_impulse <= 0.0 || mu <= 0.0 {
        // The budget is gone, so whatever the row had applied is taken back:
        // the accumulator says what this contact has done to the bodies.
        let applied = contact.accumulated_friction_impulse_ws;
        contact.accumulated_friction_impulse_ws = Vector3::zeros();
        contact.traction.borrowed_impulse = 0.0;
        contact.traction.saturated = false;
        if let Some(row) = row.filter(|_| applied.magnitude_squared() > 1e-20) {
            row.apply_impulse(bodies, -applied);
        }
        return;
    }

    let Some(row) = row else {
        return;
    };

    let rel_vel = row.relative_velocity(bodies);
    let (t1, t2) = row.tangents;
    let (effective_mass_t1, effective_mass_t2) = row.tangent_inv_mass;
    if effective_mass_t1 <= MIN_EFFECTIVE_INV_MASS
        || !effective_mass_t1.is_finite()
        || effective_mass_t2 <= MIN_EFFECTIVE_INV_MASS
        || !effective_mass_t2.is_finite()
    {
        return;
    }

    // Decompose world-space accumulator into current tangent basis
    let curr_t1 = contact.accumulated_friction_impulse_ws.dot(&t1);
    let curr_t2 = contact.accumulated_friction_impulse_ws.dot(&t2);

    let delta_t1 = tangential_error(target_relative_velocity, &rel_vel, &t1) / effective_mass_t1;
    let delta_t2 = tangential_error(target_relative_velocity, &rel_vel, &t2) / effective_mass_t2;

    let max_friction = mu * contact.accumulated_normal_impulse;
    let ConeProjection {
        t1: new_t1,
        t2: new_t2,
        magnitude: mag,
        saturated,
    } = project_onto_cone(curr_t1 + delta_t1, curr_t2 + delta_t2, max_friction);

    // What the gain bought, measured rather than assumed (§11). The row's
    // honest ceiling is the same bound without the gain; anything the
    // accumulator holds above it came from somewhere the surface did not.
    let honest_max =
        honest_tangential_coefficient(header, contact) * contact.accumulated_normal_impulse;
    contact.traction.borrowed_impulse = (mag.min(max_friction) - honest_max).max(0.0);
    contact.traction.saturated = saturated;

    let applied_t1 = new_t1 - curr_t1;
    let applied_t2 = new_t2 - curr_t2;

    // Reconstruct world-space accumulator from current basis
    contact.accumulated_friction_impulse_ws = t1 * new_t1 + t2 * new_t2;

    if applied_t1.abs() > 1e-10 || applied_t2.abs() > 1e-10 {
        let impulse = t1 * applied_t1 + t2 * applied_t2;
        log_impulse_torque_diag("friction", header, contact, row, &impulse);
        row.apply_impulse(bodies, impulse);
    }
}

/// Enforce a shared friction budget across all contacts in a manifold.
///
/// After per-contact friction solve, the total friction effort (sum of individual
/// friction impulse magnitudes) must not exceed the sum of the per-contact
/// budgets `mu_i * lambda_n_i`. If it does, all contacts' friction impulses are
/// scaled down proportionally. This prevents individual contacts from each
/// maxing out their friction cones and producing oscillating net torque.
///
/// `rows` are the manifold's prepared rows, in contact order.
pub(crate) fn manifold_friction_projection(
    bodies: &mut SolverBodies,
    rows: &[Option<ContactRow>],
    header: &PairHeader,
    contacts: &mut SmallVec<[SolverContact; 4]>,
) {
    let budget: f32 = contacts
        .iter()
        .map(|c| tangential_coefficient(header, c) * c.accumulated_normal_impulse.max(0.0))
        .sum();
    if budget <= 0.0 {
        return;
    }

    let total_friction_mag: f32 = contacts
        .iter()
        .map(|c| c.accumulated_friction_impulse_ws.magnitude())
        .sum();

    if total_friction_mag <= budget || total_friction_mag < 1e-8 {
        return;
    }

    let scale = budget / total_friction_mag;
    for (contact, row) in contacts.iter_mut().zip(rows) {
        let old = contact.accumulated_friction_impulse_ws;
        contact.accumulated_friction_impulse_ws = old * scale;
        let delta = contact.accumulated_friction_impulse_ws - old;
        if delta.magnitude_squared() > 1e-20 {
            if let Some(row) = row {
                row.apply_impulse(bodies, delta);
            }
        }
    }
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

    /// One unit-mass body sliding along +x on static ground, touching it at
    /// its centre of mass so friction only slows it.
    fn sliding_on_ground() -> (Arena<RigidBody>, PairHeader, SolverContact) {
        let mut bodies = Arena::new();
        let mut body = RigidBody::new(RigidBodyDesc::dynamic());
        body.set_mass_properties(1.0, Matrix3::identity());
        body.set_linear_velocity(Vector3::new(1.0, 0.0, 0.0));
        let index = bodies.insert(body);
        let header = PairHeader {
            body_a: None,
            body_b: RigidBodyHandle(index),
            collider_a: None,
            collider_b: None,
            restitution: 0.0,
            friction: 0.5,
        };
        let contact = SolverContact {
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
            traction: TractionRow::default(),
        };
        (bodies, header, contact)
    }

    /// One solve of the row, prepared from the bodies as they are now and
    /// written back to them.
    fn solve(bodies: &mut Arena<RigidBody>, header: &PairHeader, contact: &mut SolverContact) {
        let mut solver_bodies = SolverBodies::default();
        let row = ContactRow::prepare(bodies, &mut solver_bodies, header, contact, (1.0, 1.0));
        solve_friction_impulse(&mut solver_bodies, row.as_ref(), header, contact);
        solver_bodies.scatter(bodies);
    }

    fn velocity(bodies: &Arena<RigidBody>, header: &PairHeader) -> Vector3<f32> {
        bodies.get(header.body_b.0).unwrap().linear_velocity()
    }

    /// A contact that stops carrying gives back the friction it applied: a
    /// friction impulse at a contact with no normal impulse is energy from
    /// nowhere. A ball warm-started with the friction of a hard landing,
    /// bouncing clear in the same substep, spun up by tens of rad/s on it.
    #[test]
    fn a_contact_that_stops_carrying_takes_its_friction_back() {
        let (mut bodies, header, mut contact) = sliding_on_ground();
        solve(&mut bodies, &header, &mut contact);
        assert!(velocity(&bodies, &header).norm() < 1e-5);

        contact.accumulated_normal_impulse = 0.0;
        solve(&mut bodies, &header, &mut contact);
        assert!((velocity(&bodies, &header) - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-5);
        assert_eq!(contact.accumulated_friction_impulse_ws, Vector3::zeros());
    }

    /// With a zero target the row must be the plain friction row it replaced,
    /// bit for bit — `to_bits` rather than `==`, so a result that drifts by one
    /// ulp is caught where a tolerance would hide it.
    #[test]
    fn a_zero_target_is_the_plain_friction_row() {
        let tangent = Vector3::new(-0.749082, 0.48637468, 0.44979537);
        let cases = [
            Vector3::new(3.25, -1.5, 0.75),
            Vector3::new(-7.125, 0.03125, 2.5),
            Vector3::new(1.0e-7, 4.6692016, -9.0e6),
        ];

        for relative in cases {
            let generalised = tangential_error(&Vector3::zeros(), &relative, &tangent);
            assert_eq!(generalised.to_bits(), (-relative.dot(&tangent)).to_bits());
        }
    }

    /// The one place the two forms part company: a tangential error of exactly
    /// zero comes back positive here and negative from `-(v_rel . t)`, because
    /// `0.0 - 0.0` is `+0.0`. The solver adds the result to an accumulator and
    /// bounds it, so the sign alone changes nothing — recorded because it is the
    /// single exception to the test above.
    #[test]
    fn only_the_sign_of_zero_differs() {
        let tangent = Vector3::new(1.0, 0.0, 0.0);
        let relative = Vector3::new(0.0, 1.0, 0.0);

        let generalised = tangential_error(&Vector3::zeros(), &relative, &tangent);
        assert_eq!(generalised, 0.0);
        assert!(!generalised.is_sign_negative());
        assert!((-relative.dot(&tangent)).is_sign_negative());
    }

    /// The one trap in the generalisation: `(target - relative) . t` and
    /// `target . t - relative . t` are the same in algebra and not in floating
    /// point. These operands separate them, so reassociating the row fails here.
    #[test]
    fn tangential_error_is_not_reassociated() {
        let target = Vector3::new(2.5019093, 7.944276, 5.513714);
        let relative = Vector3::new(2.5013597, 7.9438763, 5.514461);
        let tangent = Vector3::new(-0.749082, 0.48637468, 0.44979537);

        let reassociated = target.dot(&tangent) - relative.dot(&tangent);
        assert_ne!(tangential_error(&target, &relative, &tangent), reassociated);
    }
}
