//! Generic sensing system for terrain probes.
//!
//! This module provides terrain sensing without any knowledge of what
//! the probes are used for. The probe system executes probes and returns
//! results - interpretation is left to the caller.

mod probe;
mod system;

pub use probe::{ContactCandidate, ContactCandidates, Probe, SensorSet};
pub use system::SensorProbeSystem;
