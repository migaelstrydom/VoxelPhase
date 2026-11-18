//! Falling water: the streams in the air between a lip or a spring and the
//! water below, as physics and rendering both see them.
//!
//! ```text
//!   lip       where water goes over an edge, and how it leaves it
//!   path      its ballistic path and the body of water along it
//!   streams   this frame's streams in the field, and what they strike
//! ```

mod lip;
mod path;
mod streams;

pub use lip::{along_edge, edge_ends, fall_time, falls, launch, ledge_reach, off_map, GRAVITY};
pub use path::{FallPath, Slab};
pub use streams::{FallingWater, Landing, Strike};
