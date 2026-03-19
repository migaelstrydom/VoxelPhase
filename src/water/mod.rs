//! Water simulation system.
//!
//! Heightfield-based water simulation that sits on top of the voxel terrain.
//! Water is tracked on a 2D grid in the XZ plane. Each cell stores a volume
//! and a floor level (the terrain surface it rests on). Flow equalization
//! between neighbors runs each physics step, but only on active cells.
//!
//! A fine-resolution wave grid sits on top of the flow grid, running the 2D
//! wave equation to produce visible surface ripples.

pub mod buoyancy;
pub mod coupling;
mod grid;
pub mod placer;
mod properties;
mod wave;

pub use coupling::{BodySnapshot, WaveCouplingConfig, WaveBodyCoupler};
pub use grid::{WaterCell, WaterGrid, WaterGridConfig};
pub use properties::WaterProperties;
pub use wave::{WaveCell, WaveGrid, WaveGridConfig};
