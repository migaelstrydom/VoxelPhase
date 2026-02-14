//! Re-export Obb from the collision library.
//!
//! The canonical definition now lives in `src/collision/obb.rs`.
//! This re-export preserves existing import paths during the transition.

pub use crate::collision::obb::Obb;
