//! The game's collision layers: what each of the engine's `Layers` bits
//! means here, named in one place so no two systems claim the same bit.
//!
//! A collider is in [`Layers::DEFAULT`] and meets everything unless one of
//! the filters below says otherwise.

use crate::physics::{CollisionFilter, Layers};

/// The firm core of a loose clump of leaves.
pub const FOLIAGE: Layers = Layers::layer(1);

/// A loose clump's core: it meets whatever is willing to meet it.
pub const FOLIAGE_CORE: CollisionFilter = CollisionFilter {
    membership: FOLIAGE,
    meets: Layers::ALL,
};

/// A creature's body.
pub const CREATURE: Layers = Layers::layer(2);

/// A creature's body: in the default layer as well, so it meets and is met
/// as anything else is; its own layer only says it is a creature (harm
/// spares what one strikes it with from death).
pub const CREATURE_BODY: CollisionFilter = CollisionFilter {
    membership: Layers::DEFAULT.with(CREATURE),
    meets: Layers::ALL,
};
