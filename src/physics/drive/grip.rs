//! Non-support grip: what a body may draw at a contact that is not holding it
//! up.
//!
//! ```text
//!   SupportSets ──┐
//!                 ├──► stamp_non_support_grip ──► SolverContact::tangential_scale
//!   bodies ───────┘                                (read by the tangential bound)
//! ```
//!
//! One rule, applied once per frame between the Support Set and the solve: a
//! body scales its own share of a contact's tangential budget when that
//! contact is outside its Support Set. The contact's coefficient of friction
//! is untouched, so the rule is one-sided by construction — a crate a
//! low-grip body leans on keeps its own grip against everything else.
//!
//! Two bodies each scaling the same row compose as a product, which is the
//! only composition that leaves the ordinary case — every body gripping at
//! `1.0` — an identity.

use generational_arena::Arena;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

use super::support::{ContactSite, SupportSets};

/// Stamp each contact's tangential budget scale from the non-support grip of
/// the bodies it touches.
///
/// `supports` must have been resolved from this same `manifolds` slice: a
/// `ContactSite` names a position in it, and membership is what the resolver
/// decided rather than a second reading of the rule it decided by.
pub fn stamp_non_support_grip(
    bodies: &Arena<RigidBody>,
    supports: &SupportSets,
    manifolds: &mut [SolverManifold],
) {
    for (manifold_index, manifold) in manifolds.iter_mut().enumerate() {
        let (body_a, body_b) = (manifold.header.body_a, manifold.header.body_b);
        for (contact_index, contact) in manifold.contacts.iter_mut().enumerate() {
            let site = ContactSite::new(manifold_index, contact_index);
            let mut scale = grip_at(bodies, supports, body_b, site);
            if let Some(body_a) = body_a {
                scale *= grip_at(bodies, supports, body_a, site);
            }
            contact.tangential_scale = scale;
        }
    }
}

/// What one body may draw at one contact: its full share where the contact
/// holds it up, its non-support grip everywhere else.
fn grip_at(
    bodies: &Arena<RigidBody>,
    supports: &SupportSets,
    body: RigidBodyHandle,
    site: ContactSite,
) -> f32 {
    if supports.get(body).is_some_and(|set| set.holds(site)) {
        return 1.0;
    }
    bodies
        .get(body.0)
        .map_or(1.0, |body| body.non_support_grip())
}

#[cfg(test)]
mod tests {
    use nalgebra::{UnitVector3, Vector3};

    use super::*;
    use crate::physics::body::RigidBodyDesc;
    use crate::physics::drive::support::{tests::manifold, SupportResolver};

    fn down() -> Option<UnitVector3<f32>> {
        Some(UnitVector3::new_normalize(-Vector3::y()))
    }

    /// A world of dynamic bodies, indexed by the order they were added.
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

    /// Resolve the supports and stamp the grips, the way one frame does.
    fn stamp(arena: &Arena<RigidBody>, manifolds: &mut [SolverManifold]) {
        let supports = SupportResolver::default().resolve(manifolds, down());
        stamp_non_support_grip(arena, &supports, manifolds);
    }

    fn scales(manifolds: &[SolverManifold]) -> Vec<f32> {
        manifolds
            .iter()
            .flat_map(|m| m.contacts.iter().map(|c| c.tangential_scale))
            .collect()
    }

    #[test]
    fn a_body_that_grips_everything_scales_nothing() {
        let arena = bodies(1);
        let body = handle(&arena, 0);
        let mut manifolds = vec![manifold(None, body, &[Vector3::y(), Vector3::x()])];
        stamp(&arena, &mut manifolds);
        assert_eq!(scales(&manifolds), vec![1.0, 1.0]);
    }

    #[test]
    fn a_low_grip_body_keeps_its_budget_where_it_is_supported() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        arena.get_mut(body.0).unwrap().set_non_support_grip(0.0);

        // A floor contact and a wall contact on the same body: the floor holds
        // it up and the wall does not.
        let mut manifolds = vec![manifold(None, body, &[Vector3::y(), Vector3::x()])];
        stamp(&arena, &mut manifolds);
        assert_eq!(scales(&manifolds), vec![1.0, 0.0]);
    }

    #[test]
    fn an_airborne_body_grips_nothing_at_its_declared_fraction() {
        let mut arena = bodies(1);
        let body = handle(&arena, 0);
        arena.get_mut(body.0).unwrap().set_non_support_grip(0.25);

        let mut manifolds = vec![manifold(None, body, &[Vector3::x()])];
        stamp(&arena, &mut manifolds);
        assert_eq!(scales(&manifolds), vec![0.25]);
    }

    #[test]
    fn the_scale_is_one_sided_between_the_two_bodies_of_a_contact() {
        let mut arena = bodies(2);
        let (lower, upper) = (handle(&arena, 0), handle(&arena, 1));
        arena.get_mut(upper.0).unwrap().set_non_support_grip(0.0);

        // The normal points lower → upper, so the contact holds the upper body
        // up and pushes the lower one down. The upper body's low grip is not
        // spent here; the lower body's full grip is what the row keeps.
        let mut supported = vec![manifold(Some(lower), upper, &[Vector3::y()])];
        stamp(&arena, &mut supported);
        assert_eq!(scales(&supported), vec![1.0]);

        // Turn the same pair on its side and neither body is held up by it, so
        // the upper body's declaration closes the row for both.
        let mut sideways = vec![manifold(Some(lower), upper, &[Vector3::x()])];
        stamp(&arena, &mut sideways);
        assert_eq!(scales(&sideways), vec![0.0]);
    }

    #[test]
    fn two_low_grip_bodies_compose_as_a_product() {
        let mut arena = bodies(2);
        let (a, b) = (handle(&arena, 0), handle(&arena, 1));
        arena.get_mut(a.0).unwrap().set_non_support_grip(0.5);
        arena.get_mut(b.0).unwrap().set_non_support_grip(0.5);

        let mut manifolds = vec![manifold(Some(a), b, &[Vector3::x()])];
        stamp(&arena, &mut manifolds);
        assert_eq!(scales(&manifolds), vec![0.25]);
    }
}
