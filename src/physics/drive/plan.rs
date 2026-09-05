//! The traction planner: what each supporting contact is asked to drive
//! toward, and what the driving body may spend getting there.
//!
//! ```text
//!   SupportSets ──┐
//!                 ├──► TractionPlanner ──► SolverContact::traction
//!   bodies ───────┘                          ├── target  (relative velocity, B − A)
//!                                            └── gain    (multiplier on μ·N)
//! ```
//!
//! A drive is friction with a non-zero target. The planner is the whole of the
//! difference: it reads a body's support-anchored command and writes, at every
//! contact holding that body up, the *relative* velocity the tangential row
//! should aim for. Zero — every contact of every undriven body — is ordinary
//! friction, unchanged in form and in cost.
//!
//! Two things are deliberate and neither is an optimisation.
//!
//! **The target carries the commanded spin.** Contact-point velocity is
//! `v + ω × r`, so two contacts handed the same vector do not merely constrain
//! `v`: together they pin `ω` about every axis perpendicular to their
//! separation. The per-contact target is therefore
//! `linear_target + angular_target × r_contact`, which is the well-posedness
//! condition for a drive distributed over several supports rather than a
//! refinement of one.
//!
//! **Nothing is weighted.** A contact carrying little normal impulse saturates
//! at its own `μ·N` and stops contributing while a loaded one keeps pushing,
//! which is load distribution the solver already performs. A second weighting
//! term would either halve the commanded speed (if it scaled the target) or
//! double-count the load (if it scaled the bound).

use generational_arena::Arena;
use nalgebra::Vector3;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::support::{ContactSite, SupportSets};

/// What the drive asks of one contact's tangential row.
///
/// Default is ordinary friction: hold still relative to whatever you touch,
/// under the contact's honest budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TractionRow {
    /// Relative velocity the row drives toward, in the same sense the solver
    /// measures it: the velocity of body B at the contact point minus that of
    /// body A.
    pub target: Vector3<f32>,
    /// Multiplier on this contact's tangential budget for the bodies driving
    /// through it. `1.0` is the honest `μ·N`.
    pub gain: f32,
    /// Impulse this row carried beyond what a gain of `1.0` would have
    /// permitted, as of the last solver iteration. Instrumentation only —
    /// nothing reads it back into the solve.
    pub borrowed_impulse: f32,
    /// Whether the row finished the last iteration pinned at its bound.
    pub saturated: bool,
}

impl Default for TractionRow {
    fn default() -> Self {
        Self {
            target: Vector3::zeros(),
            gain: 1.0,
            borrowed_impulse: 0.0,
            saturated: false,
        }
    }
}

impl TractionRow {
    /// True when this row is plain friction — no drive is aiming it and no
    /// body is spending more than the surface honestly permits.
    pub fn is_passive(&self) -> bool {
        self.target == Vector3::zeros() && self.gain == 1.0
    }
}

/// Turns support-anchored drive commands into per-contact tangential targets.
#[derive(Clone, Copy, Debug, Default)]
pub struct TractionPlanner;

impl TractionPlanner {
    /// Stamp every contact in `manifolds` with the row the drive wants there.
    ///
    /// `supports` must have been resolved from this same slice: a
    /// `ContactSite` names a position in it, and a drive reaches only the
    /// contacts the resolver decided are holding its body up. A driven body's
    /// other contacts — a shoulder against a wall — carry no target, which is
    /// how "no load, no drive" stays a property of where the rows are rather
    /// than a test inside the solve.
    pub fn plan(
        &self,
        bodies: &Arena<RigidBody>,
        supports: &SupportSets,
        manifolds: &mut [SolverManifold],
    ) {
        for (manifold_index, manifold) in manifolds.iter_mut().enumerate() {
            let (body_a, body_b) = (manifold.header.body_a, manifold.header.body_b);
            for (contact_index, contact) in manifold.contacts.iter_mut().enumerate() {
                let site = ContactSite::new(manifold_index, contact_index);
                let mut row = TractionRow::default();

                // Relative velocity is measured B minus A, so B's command
                // enters with its own sign and A's with the opposite one.
                if let Some(drive) = driving_at(bodies, supports, body_b, site, contact.point) {
                    row.target += drive.0;
                    row.gain *= drive.1;
                }
                if let Some(body_a) = body_a {
                    if let Some(drive) = driving_at(bodies, supports, body_a, site, contact.point) {
                        row.target -= drive.0;
                        row.gain *= drive.1;
                    }
                }

                contact.traction = row;
            }
        }
    }
}

/// What one body asks of one contact: the velocity it wants at that point
/// relative to its support, and the gain it draws the budget at.
///
/// `None` unless the body carries a support-anchored command *and* this
/// contact is one of the contacts holding it up.
fn driving_at(
    bodies: &Arena<RigidBody>,
    supports: &SupportSets,
    handle: RigidBodyHandle,
    site: ContactSite,
    point: nalgebra::Point3<f32>,
) -> Option<(Vector3<f32>, f32)> {
    if !supports.get(handle).is_some_and(|set| set.holds(site)) {
        return None;
    }
    let body = bodies.get(handle.0)?;
    let drive = body.support_drive()?;
    let arm = point - body.position();
    Some((
        drive.linear_target + drive.angular_target.cross(&arm),
        drive.gain,
    ))
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, UnitVector3};

    use super::*;
    use crate::physics::body::{RigidBodyDesc, SupportDrive};
    use crate::physics::drive::support::{tests::manifold, SupportResolver};

    fn down() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(-Vector3::y()))
    }

    fn bodies(count: usize) -> Arena<RigidBody> {
        let mut arena = Arena::new();
        for _ in 0..count {
            arena.insert(RigidBody::new(RigidBodyDesc::dynamic()));
        }
        arena
    }

    fn handle(arena: &Arena<RigidBody>, index: usize) -> RigidBodyHandle {
        RigidBodyHandle(arena.iter().nth(index).unwrap().0)
    }

    fn walking(linear: Vector3<f32>) -> SupportDrive {
        SupportDrive {
            linear_target: linear,
            angular_target: Vector3::zeros(),
            gain: 1.0,
        }
    }

    /// Resolve the supports and plan the rows, the way one frame does.
    fn plan(arena: &Arena<RigidBody>, manifolds: &mut [SolverManifold]) {
        let supports = SupportResolver::default().resolve(manifolds, down());
        TractionPlanner.plan(arena, &supports, manifolds);
    }

    fn rows(manifolds: &[SolverManifold]) -> Vec<TractionRow> {
        manifolds
            .iter()
            .flat_map(|m| m.contacts.iter().map(|c| c.traction))
            .collect()
    }

    #[test]
    fn an_undriven_contact_is_plain_friction() {
        let arena = bodies(1);
        let mut manifolds = vec![manifold(None, handle(&arena, 0), &[Vector3::y()])];
        plan(&arena, &mut manifolds);
        assert!(rows(&manifolds)[0].is_passive());
    }

    #[test]
    fn a_walkers_command_becomes_the_target_at_the_contact_holding_it_up() {
        let mut arena = bodies(1);
        let walker = handle(&arena, 0);
        arena
            .get_mut(walker.0)
            .unwrap()
            .set_support_drive(walking(Vector3::new(5.0, 0.0, 0.0)));

        let mut manifolds = vec![manifold(None, walker, &[Vector3::y()])];
        plan(&arena, &mut manifolds);
        assert_eq!(rows(&manifolds)[0].target, Vector3::new(5.0, 0.0, 0.0));
    }

    /// A drive reaches only what holds it up. A shoulder against a wall is not
    /// something to push off, whatever the body is asking for.
    #[test]
    fn a_contact_that_does_not_support_the_body_carries_no_target() {
        let mut arena = bodies(1);
        let walker = handle(&arena, 0);
        arena
            .get_mut(walker.0)
            .unwrap()
            .set_support_drive(walking(Vector3::new(5.0, 0.0, 0.0)));

        let mut manifolds = vec![manifold(None, walker, &[Vector3::y(), Vector3::x()])];
        plan(&arena, &mut manifolds);
        let planned = rows(&manifolds);
        assert_eq!(planned[0].target, Vector3::new(5.0, 0.0, 0.0));
        assert!(planned[1].is_passive());
    }

    /// The solver measures relative velocity as B minus A, so which slot the
    /// driven body occupies decides the sign of the target it asks for.
    #[test]
    fn the_target_takes_the_sign_of_the_driven_bodys_slot() {
        let mut arena = bodies(2);
        let (deck, walker) = (handle(&arena, 0), handle(&arena, 1));
        arena
            .get_mut(walker.0)
            .unwrap()
            .set_support_drive(walking(Vector3::new(5.0, 0.0, 0.0)));

        let mut as_b = vec![manifold(Some(deck), walker, &[Vector3::y()])];
        plan(&arena, &mut as_b);
        assert_eq!(rows(&as_b)[0].target, Vector3::new(5.0, 0.0, 0.0));

        let mut as_a = vec![manifold(Some(walker), deck, &[-Vector3::y()])];
        plan(&arena, &mut as_a);
        assert_eq!(rows(&as_a)[0].target, Vector3::new(-5.0, 0.0, 0.0));
    }

    /// Two contacts handed the same vector would pin the body's spin between
    /// them. The commanded spin is what makes the pair consistent.
    #[test]
    fn the_target_carries_the_commanded_spin_per_contact() {
        let mut arena = bodies(1);
        let walker = handle(&arena, 0);
        arena
            .get_mut(walker.0)
            .unwrap()
            .set_support_drive(SupportDrive {
                linear_target: Vector3::zeros(),
                angular_target: Vector3::new(0.0, 2.0, 0.0),
                gain: 1.0,
            });

        let mut manifolds = vec![manifold(None, walker, &[Vector3::y(), Vector3::y()])];
        manifolds[0].contacts[0].point = Point3::new(0.0, 0.0, 1.0);
        manifolds[0].contacts[1].point = Point3::new(0.0, 0.0, -1.0);
        plan(&arena, &mut manifolds);

        // ω × r for ω = +2ŷ: +2x̂ at z = +1, −2x̂ at z = −1.
        let planned = rows(&manifolds);
        assert_eq!(planned[0].target, Vector3::new(2.0, 0.0, 0.0));
        assert_eq!(planned[1].target, Vector3::new(-2.0, 0.0, 0.0));
    }

    #[test]
    fn the_gain_reaches_the_rows_the_body_drives_through() {
        let mut arena = bodies(1);
        let walker = handle(&arena, 0);
        arena
            .get_mut(walker.0)
            .unwrap()
            .set_support_drive(SupportDrive {
                linear_target: Vector3::new(5.0, 0.0, 0.0),
                angular_target: Vector3::zeros(),
                gain: 5.0,
            });

        let mut manifolds = vec![manifold(None, walker, &[Vector3::y(), Vector3::x()])];
        plan(&arena, &mut manifolds);
        let planned = rows(&manifolds);
        assert_eq!(planned[0].gain, 5.0);
        assert_eq!(planned[1].gain, 1.0, "a wall is not something to push off");
    }

    /// A medium anchor's command lives in the constraint arena. Nothing of it
    /// may reach a contact, or the body would get both deliveries.
    #[test]
    fn a_medium_anchored_body_drives_no_contact() {
        let arena = bodies(1);
        let lift = handle(&arena, 0);
        // No support drive set: a medium anchor holds a constraint handle
        // instead, and `support_drive` answers `None` for it.
        let mut manifolds = vec![manifold(None, lift, &[Vector3::y()])];
        plan(&arena, &mut manifolds);
        assert!(rows(&manifolds)[0].is_passive());
    }
}
