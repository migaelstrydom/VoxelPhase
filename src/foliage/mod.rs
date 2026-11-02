//! Foliage: soft greenery, a firm core physics sees inside a skin of leaves that only drag and drawing see.

mod leaf_core;
mod shape;

#[cfg(all(test, feature = "foliage_scenarios"))]
mod scenarios;

pub use leaf_core::LeafCore;
pub use shape::{FoliageShape, Lobe, ShrubSpec};
