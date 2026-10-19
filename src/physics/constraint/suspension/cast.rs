//! The world's per-frame cast for each suspension: what is under it, and what
//! that is made of.
//!
//! ```text
//!   Suspension (anchor, axis, cast radius, reach) ──┐
//!   body's pose and velocity ──────────────────────┼──► PhysicsWorld::cast_sphere ──► SuspensionHit
//!   bodies, static geometry ───────────────────────┘     (sphere along −axis)          point, normal,
//!                                                                                      partner, μ, length
//! ```
//!
//! Once per frame, as the narrowphase runs once per frame. The cast reaches
//! the suspension's reach plus however far the anchor travels along the axis
//! over the frame, so ground the body falls onto mid-frame is already there
//! for the spring to catch; ground beyond the reach is not a support and the
//! push-only spring gives nothing until it is pressed in.
//!
//! The hit's normal is the face the foot arrived on, not the rounded normal
//! of an edge or a corner. A foot caught on a plank's far edge across a gap
//! stands on the plank's top: the edge's normal leans along the gap, and the
//! spring's push and the friction plane built on it drove the body on.

use smallvec::SmallVec;

use crate::physics::body::RigidBody;
use crate::physics::collider::ColliderMaterial;
use crate::physics::handle::RigidBodyHandle;
use crate::physics::sphere_cast::{CastObstacles, SphereCast};
use crate::physics::static_geometry::StaticGeometry;
use crate::physics::sweep::SweptSurface;
use crate::physics::world::PhysicsWorld;

use super::hit::SuspensionHit;
use super::state::Suspension;

/// Cast `suspension` from its body's current pose. `frame_dt` is the whole
/// frame the hit must serve.
pub(crate) fn cast_suspension(
    world: &PhysicsWorld,
    static_geometry: &dyn StaticGeometry,
    suspension: &Suspension,
    body: &RigidBody,
    frame_dt: f32,
) -> Option<SuspensionHit> {
    let desc = suspension.desc();
    let axis = desc.axis.in_world(&body.rotation());
    let anchor = suspension.anchor_in_world(body);
    let approach = (-body.velocity_at(anchor).dot(&axis)).max(0.0);
    let reach = suspension.reach() + approach * frame_dt;
    if reach <= 0.0 {
        return None;
    }

    let mut exclude: SmallVec<[RigidBodyHandle; 5]> = desc.exclude.iter().copied().collect();
    exclude.push(desc.body);
    let hit = world.cast_sphere(
        static_geometry,
        &SphereCast {
            origin: anchor,
            direction: -axis,
            length: reach,
            radius: desc.cast_radius,
            exclude: &exclude,
            solid_bodies: false,
            obstacles: CastObstacles::Everything,
        },
    )?;

    let partner = hit.body.filter(|&handle| {
        world
            .body(handle)
            .is_some_and(|partner| !partner.is_static())
    });
    let friction = desc.friction.map_or(0.0, |foot| {
        surface_friction(world, static_geometry, hit.surface, &foot)
    });
    Some(SuspensionHit {
        point: hit.point,
        normal: hit.face_normal,
        partner,
        friction,
        axis,
        anchor,
        length: hit.distance,
    })
}

/// The friction coefficient where `foot` meets `surface`, as a collider's
/// contact would have it.
fn surface_friction(
    world: &PhysicsWorld,
    static_geometry: &dyn StaticGeometry,
    surface: SweptSurface,
    foot: &ColliderMaterial,
) -> f32 {
    match surface {
        SweptSurface::Static(id) => static_geometry
            .surface(id)
            .map_or(foot.friction(), |surface| surface.meet(foot).1),
        SweptSurface::Collider(handle) => world.collider(handle).map_or(foot.friction(), |c| {
            ColliderMaterial::combine(foot, c.material()).1
        }),
    }
}
