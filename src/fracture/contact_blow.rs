//! The blows a compound's colliders took, read from the world's hits.
//!
//! ```text
//!   PhysicsWorld::hits_on(body)  ──►  contact_blows  ──►  one ContactBlow
//!     (contacts that closed fast)      (hardest each)       per collider
//! ```
//!
//! A hit is a contact that closed fast, so what a body only carries (its own
//! weight, a structure settling, an arch's thrust swinging between blocks)
//! is no blow at all, and needs no deadband to keep it out. A blow is the
//! impulse the hit drove through the collider's side, which for a body
//! lying on something is the whole stop of what struck it.
//!
//! Each frame's blows are judged on their own. A stop the solver split
//! across two frames reads at its full approach in the first and at what
//! is left in the second, so it never reads harder than it was; a second
//! contact the next frame (a column landing on one end and slapping down
//! on the other) is a blow of its own.

use nalgebra::{Point3, Vector3};

use crate::physics::{ColliderHandle, Hit, PhysicsWorld, RigidBodyHandle};

/// The hardest blow one collider took in the last step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactBlow {
    /// The impulse the hit drove through its side, N·s
    /// ([`Hit::impulse_through`]). Zero for no blow.
    pub magnitude: f32,
    /// Where it was struck, world space.
    pub point: Point3<f32>,
    /// The way the blow pushed it, unit length; zero for no blow.
    pub push: Vector3<f32>,
    /// What struck it: the other side's collider, if it has one.
    pub by: Option<ColliderHandle>,
}

impl Default for ContactBlow {
    fn default() -> Self {
        Self {
            magnitude: 0.0,
            point: Point3::origin(),
            push: Vector3::zeros(),
            by: None,
        }
    }
}

impl ContactBlow {
    /// The blow `hit` dealt `body`, `None` if it was not on `body`.
    fn of(hit: &Hit, body: RigidBodyHandle) -> Option<Self> {
        let push = hit.push_on(body)?;
        let magnitude = hit.impulse_through(body)?;
        let by = if hit.body_b == body {
            hit.collider_a
        } else {
            hit.collider_b
        };
        Some(Self {
            magnitude,
            point: hit.point,
            push,
            by,
        })
    }
}

/// The hardest blow each of `colliders`, on `body`, took in the last step,
/// in their order. A hit that names no collider on the body's side (a
/// sweep's) lands on the nearest of them.
pub fn contact_blows(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    colliders: &[ColliderHandle],
) -> Vec<ContactBlow> {
    let mut blows = vec![ContactBlow::default(); colliders.len()];
    for hit in world.hits_on(body) {
        let Some(blow) = ContactBlow::of(hit, body).filter(|b| b.magnitude.is_finite()) else {
            continue;
        };
        let own = if hit.body_b == body {
            hit.collider_b
        } else {
            hit.collider_a
        };
        let index = own
            .and_then(|own| colliders.iter().position(|&c| c == own))
            .or_else(|| nearest(world, body, colliders, hit.point));
        if let Some(index) = index {
            if blow.magnitude > blows[index].magnitude {
                blows[index] = blow;
            }
        }
    }
    blows
}

/// Which of `colliders` on `body` is centred nearest `point`.
fn nearest(
    world: &PhysicsWorld,
    body: RigidBodyHandle,
    colliders: &[ColliderHandle],
    point: Point3<f32>,
) -> Option<usize> {
    let body = world.body(body)?;
    let (position, rotation) = (body.position(), body.rotation());
    colliders
        .iter()
        .enumerate()
        .filter_map(|(index, handle)| {
            let centre = world
                .collider(*handle)?
                .world_transform(position, rotation)
                .translation
                .vector;
            Some((index, (centre - point.coords).norm_squared()))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(index, _)| index)
}
