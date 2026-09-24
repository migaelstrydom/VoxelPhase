//! Recording geometry draws with as few state changes as the draws allow.

mod draw;
mod recorder;

pub use draw::{GeometryDraw, GeometryPush};
pub use recorder::{GeometryRecorder, SharedBindings};
