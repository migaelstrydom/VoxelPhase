//! Identifiers shared across the water layers.

/// A store in the hydrology network: a basin, a reach, the ocean, a sink or a
/// reservoir. Indexes the network's store table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct StoreId(pub u32);

/// A link in the hydrology network. Indexes the network's link table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct LinkId(pub u32);

/// A body of still water: the store id of a basin or the ocean.
///
/// A merge or split makes a new store, so a body's identity ends with it. That
/// is deliberate: ripple tiles are keyed by body, and restart calm across one.
pub type WaterBodyId = StoreId;
