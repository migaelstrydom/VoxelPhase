//! Replacing one child of a compound body with the pieces it broke into.
//!
//! ```text
//!   body: [ a ][ b ][ c ]        child b breaks into two
//!             │
//!             ▼
//!   detach b, attach b1 b2  ──▶  body: [ a ][ c ][ b1 ][ b2 ]
//! ```
//!
//! Everything the object records about a child is keyed by its collider
//! handle, which detaching and attaching leave alone, so the split only has
//! to forget the child that went and report the handles of the pieces.
//!
//! What a piece *is* stays with the caller: a sheet of glass cuts a polygon
//! into shards and a block of ice cleaves along a plane, and neither is this
//! module's business. It is handed finished colliders.

use crate::physics::{ColliderDesc, ColliderHandle, FrictionModel, RigidBodyHandle};
use crate::systems::PhysicsResource;

use super::components::CompoundFracture;

/// Swap `child` for `pieces` on `body_handle`, and forget the child's joints
/// and records in the fracture.
///
/// The body's mass properties come out unchanged if the pieces fill the same
/// volume the child did, which is what a fracture of a solid means; the body
/// is not recentred, because its centre of mass has not moved. That happens
/// later, when pieces actually leave.
///
/// Returns the new pieces' handles, in the order their colliders were given;
/// shorter than that list if the engine refused one. A new piece arrives
/// with no joints, no load history, no kick and the compound's material:
/// whoever added it says what it is joined to and what it wears.
///
/// `None` if the child is not on this body. An empty `pieces` list is a way
/// of removing a child outright, and is allowed.
pub fn split_child(
    physics: &mut PhysicsResource,
    fracture: &mut CompoundFracture,
    body_handle: RigidBodyHandle,
    child: ColliderHandle,
    pieces: impl IntoIterator<Item = ColliderDesc>,
) -> Option<Vec<ColliderHandle>> {
    if !physics
        .world
        .body(body_handle)?
        .colliders()
        .contains(&child)
    {
        return None;
    }

    let world = &mut physics.world;
    world.detach_collider(body_handle, child);
    let fresh: Vec<ColliderHandle> = pieces
        .into_iter()
        .filter_map(|desc| world.attach_collider(body_handle, desc))
        .collect();
    world.wake_body(body_handle);

    fracture.forget(child);
    Some(fresh)
}

/// The density and surface properties of a collider, so pieces cut from it
/// can be given the same ones.
///
/// Density rather than mass: the pieces are smaller than what they came from,
/// and each should weigh what its own volume says it weighs.
pub struct ChildSubstance {
    pub density: f32,
    pub restitution: f32,
    pub friction: FrictionModel,
}

impl ChildSubstance {
    /// Read off the collider `handle`, or `None` if it is not there.
    pub fn of(physics: &PhysicsResource, handle: ColliderHandle) -> Option<Self> {
        let collider = physics.world.collider(handle)?;
        let material = collider.material();
        Some(Self {
            density: collider.mass() / collider.shape().compute_mass(1.0).max(1e-9),
            restitution: material.restitution,
            friction: material.friction,
        })
    }

    /// Dress `desc` in this substance.
    pub fn clothe(&self, desc: ColliderDesc) -> ColliderDesc {
        desc.density(self.density)
            .restitution(self.restitution)
            .friction_model(self.friction)
    }
}
