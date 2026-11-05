//! Replacing one child of a compound body with the pieces it broke into.
//!
//! ```text
//!   body: [ a ][ b ][ c ]        child b breaks into two
//!             │
//!             ▼
//!   detach b, attach b1 b2  ──▶  body: [ a ][ c ][ b1 ][ b2 ]
//!             │
//!             ▼
//!   order = [Some(0), Some(2), None, None]
//! ```
//!
//! The awkward part is not the swap but everything indexed by child. A body
//! keeps its colliders in a list, detaching one moves the last into its
//! place, and every joint, material, kick, spike baseline and damage figure
//! the object holds is indexed by position in that list. So a split reports
//! the permutation it caused — `order[new]` is the old index now sitting at
//! `new` — and each thing keyed by child follows it across.
//!
//! What a piece *is* stays with the caller: a sheet of glass cuts a polygon
//! into shards and a block of ice cleaves along a plane, and neither is this
//! module's business. It is handed finished colliders.

use crate::physics::{ColliderDesc, ColliderHandle, FrictionModel, RigidBodyHandle};
use crate::systems::PhysicsResource;

use super::components::CompoundFracture;

/// What a split did to a body's child list.
pub struct ChildSplit {
    /// `order[new]` is the index this child had before the split, or `None`
    /// for one of the new pieces. Everything the caller keys by child is
    /// reindexed with it.
    pub order: Vec<Option<usize>>,
    /// Where the new pieces ended up, in the order their colliders were
    /// given. Shorter than that list if the engine refused one.
    pub pieces: Vec<usize>,
}

/// Swap `child` for `pieces` on `body_handle`, and reindex the fracture with
/// the permutation that caused.
///
/// The body's mass properties come out unchanged if the pieces fill the same
/// volume the child did, which is what a fracture of a solid means; the body
/// is not recentred, because its centre of mass has not moved. That happens
/// later, when pieces actually leave.
///
/// `None` if the child is not on this body. An empty `pieces` list is a way
/// of removing a child outright, and is allowed.
pub fn split_child(
    physics: &mut PhysicsResource,
    fracture: &mut CompoundFracture,
    body_handle: RigidBodyHandle,
    child: ColliderHandle,
    pieces: impl IntoIterator<Item = ColliderDesc>,
) -> Option<ChildSplit> {
    let before: Vec<ColliderHandle> = physics.world.body(body_handle)?.colliders().to_vec();
    if !before.contains(&child) {
        return None;
    }

    let world = &mut physics.world;
    world.detach_collider(body_handle, child);
    let fresh: Vec<ColliderHandle> = pieces
        .into_iter()
        .filter_map(|desc| world.attach_collider(body_handle, desc))
        .collect();
    world.wake_body(body_handle);

    let now: Vec<ColliderHandle> = world.body(body_handle)?.colliders().to_vec();
    let order: Vec<Option<usize>> = now
        .iter()
        .map(|handle| before.iter().position(|old| old == handle))
        .collect();
    fracture.reindex(&order);

    let pieces = fresh
        .iter()
        .filter_map(|handle| now.iter().position(|h| h == handle))
        .collect();
    Some(ChildSplit { order, pieces })
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
