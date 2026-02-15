//! Separating Axis Test helpers.
//!
//! Shared SAT projection and overlap utilities used by OBB-OBB and other
//! algebraic (tier 2) pair tests.

use nalgebra::Vector3;

/// Result of testing a single SAT axis.
#[derive(Debug, Clone, Copy)]
pub struct SatAxisResult {
    /// The normalized separating/penetration axis (A→B direction).
    pub axis: Vector3<f32>,
    /// Overlap on this axis. Positive = overlapping, negative = separated.
    pub overlap: f32,
}

/// Minimum axis normalization threshold.
pub const AXIS_EPS: f32 = 1e-6;

/// Tolerance for near-zero overlaps (avoid false contacts from float noise).
pub const OVERLAP_EPS: f32 = 1e-5;

/// Cached separating axis from a previous frame for early-out.
#[derive(Debug, Clone, Copy)]
pub struct SatCache {
    /// Last frame's separating axis (world space). None if the pair was colliding.
    pub separating_axis: Option<Vector3<f32>>,
}

impl SatCache {
    pub fn new() -> Self {
        Self {
            separating_axis: None,
        }
    }
}

impl Default for SatCache {
    fn default() -> Self {
        Self::new()
    }
}
