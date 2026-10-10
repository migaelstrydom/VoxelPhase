//! Water as a shallow-water field: one cell per span, depths in cells and
//! discharges on the faces between them.

mod activity;
mod cells;
mod currents;
mod faces;
mod field;
mod fill;
mod remap;
mod solver;
mod sources;
mod versions;

pub use cells::{FieldCells, FieldChunk, CELL_AREA};
pub use faces::{Axis, BoundaryFace, Face, SeaBoundary};
pub use field::{FieldFrame, FieldLedger, WaterField, WET_DEPTH};
pub use fill::{pool_seed, SpillLevels};
pub use solver::{FlowParams, StepStats};
pub use sources::{Sink, Source};
pub use versions::{DrawnDepths, SurfaceVersions};
