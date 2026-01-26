//! Probe definitions for terrain sensing.
//!
//! Probes are defined in world coordinates and have opaque tags
//! that the sensing system passes through without interpretation.

use nalgebra::{Point3, Vector3};
use specs::{Component, VecStorage};

/// A terrain probe in world coordinates.
///
/// The sensing system executes probes without knowing their purpose.
/// The `tag` field is passed through to results for the caller to interpret.
#[derive(Debug, Clone)]
pub struct Probe {
    /// Opaque identifier - the sensing system doesn't interpret this.
    /// Used by the caller to match results back to their meaning.
    pub tag: u32,
    /// Probe origin in world space.
    pub origin: Point3<f32>,
    /// Probe direction in world space (should be normalized).
    pub direction: Vector3<f32>,
    /// Maximum probe distance.
    pub length: f32,
    /// Radius for swept sphere queries.
    pub radius: f32,
}

/// A contact candidate from a terrain probe.
///
/// The `tag` matches the probe that generated this result.
#[derive(Debug, Clone)]
pub struct ContactCandidate {
    /// Tag from the probe that generated this contact.
    pub tag: u32,
    /// Contact point in world space.
    pub point: Point3<f32>,
    /// Surface normal at contact point.
    pub normal: Vector3<f32>,
    /// Distance from probe origin to contact.
    pub distance: f32,
}

/// Collection of contact candidates from probes.
#[derive(Component, Debug, Default, Clone)]
#[storage(VecStorage)]
pub struct ContactCandidates {
    pub candidates: Vec<ContactCandidate>,
}

/// Set of probes to execute each frame.
///
/// This component is written by animation controllers and read by
/// the sensing system. Probes are in world coordinates.
#[derive(Component, Debug, Default, Clone)]
#[storage(VecStorage)]
pub struct SensorSet {
    pub probes: Vec<Probe>,
}

impl SensorSet {
    /// Create a new sensor set with the given probes.
    pub fn new(probes: Vec<Probe>) -> Self {
        Self { probes }
    }
}
