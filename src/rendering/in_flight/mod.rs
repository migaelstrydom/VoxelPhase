//! Letting the CPU record one frame while the GPU draws the previous one.
//!
//! ```text
//!   CPU  │ record 1 │ record 2 │ record 3 │ ...
//!   GPU             │  draw 1  │  draw 2  │ ...
//!          slot 0     slot 1     slot 0
//! ```
//!
//! Whatever the CPU writes during recording has one copy per frame in flight
//! ([`PerFrame`]), and a slot's copy is rewritten only once that slot's fence
//! has signalled.

mod in_flight_frame;
mod per_frame;
mod streamed_mesh;

pub use in_flight_frame::InFlightFrame;
pub use per_frame::{FrameSlot, PerFrame, FRAMES_IN_FLIGHT};
pub use streamed_mesh::StreamedMesh;
