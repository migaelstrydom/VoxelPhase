//! Geometry kept on the GPU between frames: models and terrain, uploaded when
//! they change rather than every frame they are drawn.

mod arena;
mod geometry;
mod range_allocator;

pub use arena::{MeshArena, ResidentMesh, UploadTally};
pub use geometry::{ResidentGeometry, ResidentPrimitive, VersionedMeshId};
pub use range_allocator::RangeAllocator;
