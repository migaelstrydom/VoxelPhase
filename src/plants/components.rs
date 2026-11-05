//! A plant's ECS components.

use std::sync::Arc;

use specs::{Component, VecStorage};

use crate::model::{MeshPrimitive, Model};
use crate::physics::ColliderHandle;

/// A crown of leaves held on a limb, which a blast close enough breaks off.
///
/// On the limb's entity, beside the crown's `Foliage`: the crown's core is
/// part of the limb's body and its skin part of the limb's model, so
/// breaking it off takes both apart. What it needs to do that is here.
#[derive(Component)]
#[storage(VecStorage)]
pub struct HeldCrown {
    /// The crown's core colliders on the limb's body.
    pub colliders: Vec<ColliderHandle>,
    /// The crown's skin, in the crown's own frame: its model once loose.
    pub skin: Arc<Model>,
    /// The limb drawn without its crown, in the body's frame as it is now.
    pub wood: MeshPrimitive,
}
