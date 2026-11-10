//! What a stilt walker wears in place of the peeper's eye.
//!
//! ```text
//!   head   in the head's frame: its middle at the origin, +Z the way it
//!          looks, +Y its crown. Rears and spears with the neck.
//!   body   in the frame standing upright at the pelvis, +Z the way the
//!          walker faces. Rides level over the legs.
//! ```
//!
//! A costume is made once, by whoever defines the creature, and drawn every
//! frame where the skeleton puts it. Its colours ride on its vertices.

use crate::rendering::vertex::Vertex;

/// A mesh worn on one of the rig's frames.
#[derive(Clone, Default)]
pub struct Garment {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

/// Everything a walker wears above its legs.
#[derive(Clone, Default)]
pub struct Costume {
    /// Worn on the head.
    pub head: Garment,
    /// Worn on the body.
    pub body: Garment,
    /// Whether the rig's own haunch and neck are drawn under it: off for a
    /// costume that covers them or stands in for them.
    pub shows_trunk: bool,
}
