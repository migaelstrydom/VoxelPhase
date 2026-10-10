//! Identifiers shared across the water layers and the water renderer.

/// A drawn water surface or body: a chunk of the field's surface by slot,
/// the sea (`FIELD_BODY`), a bench's pool.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SurfaceId(pub u32);

/// A body of still water, as ripple tiles are keyed by it.
pub type WaterBodyId = SurfaceId;
