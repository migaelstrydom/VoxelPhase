//! The game's collision layers: what each of the engine's `Layers` bits
//! means here, named in one place so no two systems claim the same bit.
//!
//! A collider is in [`Layers::DEFAULT`] and meets everything unless one of
//! the filters below says otherwise.

use crate::physics::{CollisionFilter, Layers};

/// The firm core of a loose clump of leaves.
pub const FOLIAGE: Layers = Layers::layer(1);
/// A plant's limbs: trunks and branches.
pub const WOOD: Layers = Layers::layer(3);

/// A loose clump's core: it meets whatever is willing to meet it.
pub const FOLIAGE_CORE: CollisionFilter = CollisionFilter {
    membership: FOLIAGE,
    meets: Layers::ALL,
};

/// A plant's limb and the crown it holds up: solid to everything but other
/// limbs, since a branch overlaps the limb it forks from at the fork.
pub const PLANT: CollisionFilter = CollisionFilter {
    membership: WOOD,
    meets: Layers::ALL.without(WOOD),
};
