//! Water as a shallow-water field: one cell per span, depths in cells and
//! discharges on the faces between them.

mod activity;
mod cells;
mod conduits;
mod currents;
mod faces;
mod falls;
mod field;
mod fill;
mod foam;
mod fronts;
mod pumps;
mod remap;
mod solver;
mod sources;
#[cfg(test)]
mod testing;
mod versions;

pub use cells::{FieldCells, FieldChunk, CELL_AREA};
pub use conduits::Conduit;
pub use faces::{past_edge, Axis, BoundaryFace, Face, Flight, Push, SeaBoundary};
pub use falls::{Fall, Falls};
pub use field::{FieldFrame, FieldLedger, WaterField, WET_DEPTH};
pub use fill::{pool_seed, SpillLevels};
pub use foam::MAX_FOAM;
pub use fronts::Front;
pub use pumps::{Jet, PumpId, PumpPlacement};
pub use solver::{FlowParams, StepStats};
pub use sources::{Sink, Source};
pub use versions::{DrawnDepths, SurfaceVersions};
