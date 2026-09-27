//! Closing allowances: how fast a speculative contact's pair may still approach.
//!
//! A speculative contact is generated once per frame with the gap its pair had
//! then. By any later substep the pair has closed some of it, so the allowance
//! is recomputed before each velocity phase from how far the bodies have moved
//! since generation — the same reference position correction measures
//! penetration from. What remains of the gap, over the substep, is the approach
//! speed the normal row permits; the pair is arrested only beyond it, and so
//! stops on arrival instead of wherever it was when the contact was generated.

use generational_arena::{Arena, Index};
use nalgebra::{Point3, Vector3};
use rustc_hash::FxHashMap;

use crate::physics::body::RigidBody;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::pipeline::pair::SolverManifold;

/// Set every contact's `closing_allowance` for a substep of `dt`.
///
/// `generation_positions` holds each movable body's position when contacts were
/// generated; a body missing from it is taken not to have moved. Contacts with
/// no gap keep the zero allowance they were built with. Rotation since
/// generation is not accounted for, as it is not in position correction either.
pub(crate) fn set_closing_allowances(
    bodies: &Arena<RigidBody>,
    manifolds: &mut [SolverManifold],
    generation_positions: &FxHashMap<Index, Point3<f32>>,
    dt: f32,
) {
    if dt <= 0.0 {
        return;
    }
    let moved = |handle: RigidBodyHandle| -> Vector3<f32> {
        match (bodies.get(handle.0), generation_positions.get(&handle.0)) {
            (Some(body), Some(base)) => body.position() - base,
            _ => Vector3::zeros(),
        }
    };
    for manifold in manifolds {
        if manifold.contacts.iter().all(|c| c.gap <= 0.0) {
            continue;
        }
        let moved_a = manifold.header.body_a.map_or(Vector3::zeros(), moved);
        let relative = moved(manifold.header.body_b) - moved_a;
        for contact in manifold.contacts.iter_mut().filter(|c| c.gap > 0.0) {
            let remaining = contact.gap + relative.dot(&contact.normal);
            contact.closing_allowance = remaining.max(0.0) / dt;
        }
    }
}
