//! Traversal primitives — the geometry a platformer route is made of.
//!
//! Heightfield features and blobby volumes describe *landscape*. These describe
//! a *route*: a deck to walk along, a slab to jump to, a flight of steps to
//! climb, a bore to descend.
//!
//! ```text
//!   VolumeFeature::Path      ─▶ PathSolid      ─┐
//!   VolumeFeature::Platform  ─▶ PlatformSolid  ─┤
//!   VolumeFeature::Staircase ─▶ StaircaseSolid ─┼─▶ TraversalSolid ─┬─ rasterise ─▶ ChunkGrid
//!   VolumeFeature::Shaft     ─▶ ShaftBore      ─┤                   └─ excavate  ─▶ ChunkGrid
//!                             ╰ ShaftLedge     ─┘
//! ```
//!
//! Every primitive is a signed-distance function in **segment-local**
//! coordinates and nothing else; the rasteriser owns the voxel lattice, the
//! sub-voxel density encoding and the CSG write. Adding a primitive is
//! therefore one distance function, not another triple-nested loop — and it
//! rotates with its segment for free, because nothing here has ever seen a
//! world coordinate.

mod feature;
mod path;
mod platform;
mod shaft;
mod solid;
mod staircase;
#[cfg(test)]
mod tests;

pub use feature::{route_plan, RoutePart, RoutePlan};
pub use path::PathSolid;
pub use platform::PlatformSolid;
pub use shaft::{ShaftBore, ShaftLedgeSolid};
pub use solid::{excavate, rasterise, Sample, TraversalSolid};
pub use staircase::StaircaseSolid;
